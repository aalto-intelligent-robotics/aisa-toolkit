import copy
from pathlib import Path
from typing import Tuple
import numpy as np

import carla
from carla_util import carla_img_to_array
from models.controller_model import ControllerModel
from models.lane_detection.camera_geometry import CameraGeometry
from models.lane_detection.lane_detector import LaneDetector
from models.lane_detection.pure_pursuit import PurePursuitPlusPID


def get_trajectory_from_lane_detector(ld, img):
    # get lane boundaries using the lane detector
    # img = carla_img_to_array(image)
    poly_left, poly_right, left_mask, right_mask = ld.get_fit_and_probs(img)
    # trajectory to follow is the mean of left and right lane boundary
    # note that we multiply with -0.5 instead of 0.5 in the formula for y below
    # according to our lane detector x is forward and y is left, but
    # according to Carla x is forward and y is right.
    x = np.arange(-2, 10, 0.1)
    y = -0.5 * (poly_left(x) + poly_right(x))
    # x,y is now in coordinates centered at camera, but camera is 0.5 in front of vehicle center
    # hence correct x coordinates
    x += 0.5
    traj = np.stack((x, y)).T
    return (
        traj,
        ld_detection_overlay(img, left_mask, right_mask),
        (left_mask, right_mask),
    )


def ld_detection_overlay(image, left_mask, right_mask):
    res = copy.copy(image)
    res[left_mask > 0.4, :] = [255, 0, 0]
    res[right_mask > 0.4, :] = [255, 0, 0]

    res[left_mask > 0.5, :] = [255, 160, 0]
    res[right_mask > 0.5, :] = [255, 160, 0]

    res[left_mask > 0.8, :] = [255, 255, 0]
    res[right_mask > 0.8, :] = [255, 255, 0]

    res[left_mask > 0.9, :] = [0, 255, 0]
    res[right_mask > 0.9, :] = [0, 255, 0]

    return res


class LaneControllerModel(ControllerModel):
    """
    Implements vehicle control from lane detection.
    """

    def __init__(self) -> None:
        super().__init__()
        # Initiating lane detection confidence with 1.0
        # in order to not require ToR in the beginning
        self._left_lane_confidence = 1.0
        self._right_lane_confidence = 1.0

        # Threshold for raising a warning for
        # low lane markings detection
        self._warning_threshold = 0.6
        # Threshold for raising a takeover request
        self._tor_threshold = 0.3

        self._camera_geometry = CameraGeometry()
        self._lane_detector = LaneDetector(
            model_path=Path(
                "models/lane_detection/saved_model/fastai_model.pth"
            ).absolute(),
            cam_geom=self._camera_geometry,
        )

        self._pid_controller = PurePursuitPlusPID()

        # TODO: Get below values from the simulation
        # Frames per second.
        self._fps = 30
        # Desired vehicle speed
        self._desired_speed = 5

    def control(
        self, sensor_data: dict, speed: float, vehicle: carla.Vehicle
    ) -> Tuple[float, float, int]:
        """
        Reads the camera image, predicts left and right lane.
        Trajectory follows the lanes.
        PID controller predicts the control from the trajectory.

        Expects the first sensor data to be the camera image.
        """
        # We exect the first sensor data to be the camera image.
        image_windshield = sensor_data["camera_image"]

        traj, viz, masks = get_trajectory_from_lane_detector(
            self._lane_detector, image_windshield
        )
        self._predicted_trajectory = traj
        self._overlay_image = viz
        self._left_lane_confidence = masks[0].max()
        self._right_lane_confidence = masks[1].max()

        throttle, steer = self._pid_controller.get_control(
            traj, speed, desired_speed=self._desired_speed, dt=1.0 / self._fps
        )

        self._fill_messages()

        return throttle, steer, 0, traj

    def _fill_messages(self):
        """
        Warning and critical messages after lane detection.
        """
        self._critical_messages = []
        self._warning_messages = []

        if self._left_lane_confidence < self._tor_threshold:
            self._critical_messages.append("LOST left lane.")
        elif self._left_lane_confidence < self._warning_threshold:
            self._warning_messages.append("WEAK left lane detection.")

        if self._right_lane_confidence < self._tor_threshold:
            self._critical_messages.append("LOST right lane.")
        elif self._right_lane_confidence < self._tor_threshold:
            self._warning_messages.append("WEAK right lane detection.")

        self._display_info = [
            "Lane detection confidence:",
            f" - Left lane: {self._left_lane_confidence:.2f}",
            f" - Right lane: {self._right_lane_confidence:.2f}",
        ]

    def initiate_tor(self) -> bool:
        return (
            self._right_lane_confidence < self._tor_threshold
            and self._left_lane_confidence < self._tor_threshold
        )
