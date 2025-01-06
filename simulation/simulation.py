# Code based on Algorithms for Autonomous Driving.
# Also based on Carla examples, which are authored by
# Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).

import sys
import os
import glob

try:
    sys.path.append(
        glob.glob(
            f"{os.environ['CARLA_HOME']}/PythonAPI/carla/dist/carla-*%d.%d-%s.egg"
            % (
                sys.version_info.major,
                sys.version_info.minor,
                "win-amd64" if os.name == "nt" else "linux-x86_64",
            )
        )[0]
    )
except IndexError:
    pass


import carla
import random
from pathlib import Path
import numpy as np
import argparse
from carla_util import (
    carla_vec_to_np_array,
    carla_img_to_array,
    CarlaSyncMode,
    find_weather_presets,
    draw_image_np,
    should_quit,
)
import cv2
import copy
import pygame
from gtts import gTTS
from keyboard_control import KeyboardControl
from models.controller_factory import ControllerModelFactory
from models.controller_model import ControllerModel
from models.lane_detection.camera_geometry import CameraGeometry
from helpers import (
    draw_route,
    parse_spawn_point,
    draw_trajectory,
)
from configurator import ConflictConfigurator
from agents.navigation.basic_agent import BasicAgent

main_image_shape = (800, 600)
CAMERA_LOCATION_INSIDE_VEHICLE = carla.Location(x=0.2, y=-0.2, z=1.3)
CAMERA_LOCATION_BEHIND_VEHICLE = carla.Location(x=-5.5, z=2.8)
CAMERA_ROTATION = carla.Rotation(pitch=-10)

configuration = dict()


def create_wp(wp_raw):
    x = float(wp_raw.attrib["x"])
    y = float(wp_raw.attrib["y"])
    z = float(wp_raw.attrib["z"])
    pitch = float(wp_raw.attrib["pitch"])
    yaw = float(wp_raw.attrib["yaw"])
    roll = float(wp_raw.attrib["roll"])
    return carla.Transform(
        carla.Location(x, y, z), carla.Rotation(pitch, yaw, roll)
    )


def inject_noise(sensor_reading: np.array, sigma: float) -> np.array:
    """
    Adds noise to the given array.
    """
    noise = np.random.normal(0, sigma, sensor_reading.shape)
    noisy_reading = sensor_reading + noise
    return noisy_reading


def play_sound_file(sound_file: str):
    pygame.mixer.init()
    # Load the audio file into Pygame
    sound = pygame.mixer.Sound(sound_file)
    # Play the audio
    sound.play()
    # Wait for the sound to finish playing
    while pygame.mixer.get_busy():
        pygame.time.delay(100)


def audio_switch_to_manual_control():
    play_sound_file("assets/sounds/switching_to_manual_control.mp3")


def audio_switch_to_automatic_control():
    play_sound_file("assets/sounds/switching_to_automatic_control.mp3")


def send_control(
    vehicle, throttle, steer, brake, hand_brake=False, reverse=False
):
    throttle = np.clip(throttle, 0.0, 1.0)
    steer = np.clip(steer, -1.0, 1.0)
    brake = np.clip(brake, 0.0, 1.0)
    control = carla.VehicleControl(throttle, steer, brake, hand_brake, reverse)
    vehicle.apply_control(control)


def create_controller_model(model_name: str) -> ControllerModel:
    factory = ControllerModelFactory()
    return factory.create_model(model_name)


def main(args: dict):
    manual_control = args.model == "manual"

    actor_list = []
    pygame.init()

    display = pygame.display.set_mode(
        main_image_shape, pygame.HWSURFACE | pygame.DOUBLEBUF
    )
    font = pygame.font.SysFont("monospace", 15)
    clock = pygame.time.Clock()

    client = carla.Client("localhost", 2000)
    client.set_timeout(80.0)

    client.load_world(configuration["town"])
    world = client.get_world()

    # set weather conditions
    weather_preset, _ = find_weather_presets()[configuration["weather"]]
    world.set_weather(weather_preset)

    if not manual_control:
        controller = create_controller_model(args.model)

    manual_controller = KeyboardControl()

    try:
        sensor_data = {}
        m = world.get_map()
        sensor_data["carla_map"] = m

        # Starting spawn point for the ego vehicle
        spawn_point = create_wp(configuration["wp_start"])

        blueprint_library = world.get_blueprint_library()

        veh_bp = random.choice(blueprint_library.filter("vehicle.audi.tt"))
        veh_bp.set_attribute("color", "64,81,181")

        vehicle = world.spawn_actor(veh_bp, spawn_point)
        actor_list.append(vehicle)

        # Set a destination and draw the route to it
        destination = create_wp(configuration["wp_dest"])

        # Spawning obstacles and actors for certain scenarios
        if(configuration["scenario"]) == "obstaclestatic":
            spawn_point_obstacle = carla.Transform(carla.Location(151.40000000, -38.40, 0.600000),carla.Rotation(0,0,0))
            world.try_spawn_actor(veh_bp, spawn_point_obstacle)
        elif(configuration["scenario"]) == "narrowingroad":
            spawn_point_obstacle = carla.Transform(carla.Location(149.40000000, -38.40, 0.600000),carla.Rotation(0,90,0))
            world.try_spawn_actor(veh_bp, spawn_point_obstacle)
        elif(configuration["scenario"]) == "customramp":
            for i in range(8):
                spawn_point_obstacle = carla.Transform(carla.Location(2370.0 + i * 1000, -256.0, 0.600000),carla.Rotation(0,90,0))
                world.try_spawn_actor(veh_bp, spawn_point_obstacle) 
        elif(configuration["scenario"]) == "obstacledynamic":
            spawn_point_obstacle = carla.Transform(carla.Location(155.40000000, -38.40, 0.600000),carla.Rotation(0,0,0))
            obs_actor = world.spawn_actor(veh_bp, spawn_point_obstacle)
            actor_list.append(obs_actor)    

        agent = BasicAgent(vehicle, 30)
        agent.follow_speed_limits(True)
        agent.set_destination(destination.location)
        if args.show_route:
            draw_route(agent, world)

        # visualization cam (no functionality)
        camera_rgb = world.spawn_actor(
            blueprint_library.find("sensor.camera.rgb"),
            carla.Transform(CAMERA_LOCATION_INSIDE_VEHICLE, CAMERA_ROTATION),
            attach_to=vehicle,
        )
        actor_list.append(camera_rgb)

        sensors = [camera_rgb]

        if not manual_control:
            cg = CameraGeometry()

            # windshield cam
            cam_windshield_transform = carla.Transform(
                carla.Location(x=0.5, z=cg.height),
                carla.Rotation(pitch=cg.pitch_deg),
            )
            bp = blueprint_library.find("sensor.camera.rgb")
            fov = cg.field_of_view_deg
            bp.set_attribute("image_size_x", str(cg.image_width))
            bp.set_attribute("image_size_y", str(cg.image_height))
            bp.set_attribute("fov", str(fov))
            camera_windshield = world.spawn_actor(
                bp, cam_windshield_transform, attach_to=vehicle
            )
            actor_list.append(camera_windshield)
            sensors.append(camera_windshield)

        frame = 0
        max_error = 0
        FPS = 30
        # Create a synchronous mode context.
        with CarlaSyncMode(world, *sensors, fps=FPS) as sync_mode:
            while True:
                if should_quit():
                    return
                clock.tick()

                # Advance the simulation and wait for the data.
                tick_response = sync_mode.tick(timeout=2.0)

                if manual_controller.switch_to_auto():
                    # controller = create_controller_model(model)
                    if args.audio and manual_control:
                        audio_switch_to_automatic_control()
                    manual_control = False

                elif(configuration["scenario"]) == "obstacledynamic":
                    location = obs_actor.get_location()
                    location.x -= .01
                    obs_actor.set_location(location)    

                # get velocity and angular velocity
                vel = carla_vec_to_np_array(vehicle.get_velocity())
                forward = carla_vec_to_np_array(
                    vehicle.get_transform().get_forward_vector()
                )
                right = carla_vec_to_np_array(
                    vehicle.get_transform().get_right_vector()
                )
                up = carla_vec_to_np_array(
                    vehicle.get_transform().get_up_vector()
                )
                # vx = vel.dot(forward)
                # vy = vel.dot(right)
                # vz = vel.dot(up)
                # ang_vel = carla_vec_to_np_array(vehicle.get_angular_velocity())
                # w = ang_vel.dot(up)
                # print(
                #     "vx vy vz w {:.2f} {:.2f} {:.2f} {:.5f}".format(
                #         vx, vy, vz, w
                #     )
                # )

                speed = np.linalg.norm(
                    carla_vec_to_np_array(vehicle.get_velocity())
                )

                if not manual_control:
                    snapshot, image_rgb, image_windshield = tick_response
                    if frame % 2 == 0:
                        img = carla_img_to_array(image_windshield)
                        sensor_data["camera_image"] = inject_noise(
                            img, configuration["sensor_noise"]
                        )
                        throttle, steer, brake, traj = controller.control(
                            sensor_data, speed, vehicle
                        )
                        # print("traj:", traj[0], vehicle.get_transform())
                        draw_trajectory(traj, world, vehicle)

                        send_control(vehicle, throttle, steer, brake)
                        viz = controller.overlay_image()
                else:
                    snapshot, image_rgb, _ = tick_response
                    # print('tick_response:', snapshot, image_rgb)

                    manual_controller.manual_control(
                        vehicle, pygame.key.get_pressed(), clock.get_time()
                    )
                if viz is None:
                    viz = carla_img_to_array(image_rgb)

                # Draw the display.
                image_rgb = copy.copy(carla_img_to_array(image_rgb))
                viz = cv2.resize(viz, (400, 200), interpolation=cv2.INTER_AREA)
                image_rgb[0 : viz.shape[0], 0 : viz.shape[1], :] = viz

                # white background for text
                image_rgb[10:200, -280:-10, :] = [255, 255, 255]

                # draw txt
                dy = 20
                texts = [f"Speed: {speed:.2f} (m/s)", "-------------------"]
                texts.extend(controller.display_info())
                texts.append("-------------------")

                # Fill messages from the controller model
                warnings = controller.warning_messages()
                critical_messages = controller.critical_messages()
                takeover_messages = []
                # Check if takeover request suggested by the controller model.
                if controller.initiate_tor():
                    takeover_messages.append("Switching to manual control.")
                    if args.audio and not manual_control:
                        audio_switch_to_manual_control()
                    manual_control = True

                # Draw the text background
                draw_image_np(display, image_rgb)

                # print(f"actors: {world.get_actors()}")
                # print(f"blueprint_library: {blueprint_library}")

                for it, t in enumerate(texts):
                    display.blit(
                        font.render(t, True, (0, 0, 0)),
                        (image_rgb.shape[1] - 270, 20 + dy * it),
                    )

                if critical_messages:
                    for it, t in enumerate(critical_messages):
                        font.set_bold(True)
                        display.blit(
                            font.render(t, True, (255, 0, 0)),
                            (image_rgb.shape[1] - 270, 140 + dy * it),
                        )
                elif warnings:
                    for it, t in enumerate(warnings):
                        font.set_bold(True)
                        display.blit(
                            font.render(t, True, (255, 172, 28)),
                            (image_rgb.shape[1] - 270, 140 + dy * it),
                        )
                if takeover_messages:
                    for it, t in enumerate(takeover_messages):
                        font.set_bold(True)
                        display.blit(
                            font.render(t, True, (255, 0, 255)),
                            (image_rgb.shape[1] - 270, 180 + dy * it),
                        )

                # Stop the car if no lanes detected.
                if not manual_control:
                    if controller.initiate_tor():
                        send_control(vehicle, 0, 0, 0)

                pygame.display.flip()
                frame += 1
    finally:
        print("destroying actors.")
        for actor in actor_list:
            actor.destroy()
        pygame.quit()
        print("done.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Runs Carla simulation.")

    parser.add_argument(
        "--conflict",
        # choices=["vanishinglanes", "narrowingroad"],
        required=True,
        help="Model for controlling the vehicle.",
    )

    parser.add_argument(
        "--model",
        choices=["lane_detection"],
        required=True,
        help="Model for controlling the vehicle.",
    )

    parser.add_argument(
        "-a",
        "--audio",
        help="Whether to use audio notifications.",
        action="store_true",
    )

    parser.add_argument(
        "--show_route",
        help="Whether to display the route from the start to the destination.",
        action="store_true",
    )

    args = parser.parse_args()

    configuration = ConflictConfigurator.parse_xml(args.conflict)

    try:
        main(args)
    except KeyboardInterrupt:
        print("\nCancelled by user. Bye!")
