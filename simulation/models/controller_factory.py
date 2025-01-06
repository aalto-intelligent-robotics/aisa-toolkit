"""
Factory for classes inheriting the base class ControllerModel.
"""

from models.controller_model import ControllerModel
from models.lane_controller_model import LaneControllerModel

class ControllerModelFactory:
    """
    Creates model for vehicle control.
    """

    @staticmethod
    def create_model(model_type) -> ControllerModel:
        """
        Creates model for vehicle control.
        """
        if model_type == "lane_detection":
            return LaneControllerModel()
        else:
            raise ValueError(f"Unknown model type: {model_type}")
