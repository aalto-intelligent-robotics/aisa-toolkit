"""
Base class ControllerModel.
"""

from abc import ABC, abstractmethod
from typing import Tuple, List
import numpy as np
import carla


class ControllerModel(ABC):
    """
    Abstract class for a controller model.
    """

    def __init__(self) -> None:
        super().__init__()
        self._critical_messages = []
        self._initiate_tor = False
        self._warning_messages = []
        self._display_info = []
        self._predicted_trajectory = []
        self._overlay_image = None

    @abstractmethod
    def control(
        self, sensor_data: dict, speed: float, vehicle: carla.Vehicle
    ) -> Tuple[float, float, int]:
        """
        By given sensor data, return (throttle, steer, brake).
        """

    # @abstractmethod
    def overlay_image(self):
        """
        Returns overlay image from the model.
        """
        return self._overlay_image

    # @abstractmethod
    def text_description(self) -> str:
        """
        Returns text explanation from the model.
        """

    # @abstractmethod
    def display_info(self) -> List[str]:
        """
        Text informationto be displayed about the lane detection
        """
        return self._display_info

    # @abstractmethod
    def warning_messages(self) -> List[str]:
        """
        Return warning messages resulting from the model execution.
        """
        return self._warning_messages

    # @abstractmethod
    def critical_messages(self) -> List[str]:
        """
        Return critical messages resulting from the model execution
        """
        return self._critical_messages

    # @abstractmethod
    def predicted_trajectory(self) -> List[Tuple[float]]:
        """
        Return the redicted trajectory form the model
        """
        return self._predicted_trajectory

    # @abstractmethod
    def initiate_tor(self) -> bool:
        """
        Indicate whether takeover request needs to be initiated
        according to the result form the model.
        """
        return self._initiate_tor
