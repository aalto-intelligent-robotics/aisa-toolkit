import carla

try:
    import pygame
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_TAB
    from pygame.locals import K_SPACE
    from pygame.locals import K_UP
    from pygame.locals import K_DOWN
    from pygame.locals import K_LEFT
    from pygame.locals import K_RIGHT
    from pygame.locals import K_w
    from pygame.locals import K_a
    from pygame.locals import K_s
    from pygame.locals import K_d
    from pygame.locals import K_q
    from pygame.locals import K_m
    from pygame.locals import K_COMMA
    from pygame.locals import K_PERIOD
    from pygame.locals import K_p
    from pygame.locals import K_i
    from pygame.locals import K_l
    from pygame.locals import K_z
    from pygame.locals import K_x
    from pygame.locals import K_r
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
except ImportError:
    raise RuntimeError(
        "cannot import pygame, make sure pygame package is installed"
    )


class KeyboardControl(object):
    """Class that handles keyboard input."""

    def __init__(self):
        print("KeyboardControl created.")
        self._control = carla.VehicleControl()
        self._steer_cache = 0.0
        self._switch_to_auto = False
        self._switch_to_manual = False

    def manual_control(self, vehicle, keys, milliseconds):
        # print(f"Manual control: {keys}")
        self._parse_vehicle_keys(keys, milliseconds)
        # print(f"Applying control: {self._control}")
        vehicle.apply_control(self._control)
        # print("Control applied.")

    def _parse_vehicle_keys(self, keys, milliseconds):
        if keys[K_m]:
            self._switch_to_auto = True

        if keys[K_UP] or keys[K_w]:
            self._control.throttle = min(self._control.throttle + 0.1, 1.00)
        else:
            self._control.throttle = 0.0

        if keys[K_DOWN] or keys[K_s]:
            self._control.brake = min(self._control.brake + 0.2, 1)
        else:
            self._control.brake = 0

        if keys[K_q]:
            self._control.gear = 1 if self._control.reverse else -1

        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            if self._steer_cache > 0:
                self._steer_cache = 0
            else:
                self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            if self._steer_cache < 0:
                self._steer_cache = 0
            else:
                self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.hand_brake = keys[K_SPACE]

    def switch_to_auto(self):
        return self._switch_to_auto
