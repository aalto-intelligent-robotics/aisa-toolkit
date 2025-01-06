"""This module provides the key configuration parameters for a XML-based scenario."""
import warnings
import xml.etree.ElementTree as ET
from collections.abc import Iterable

import carla

class ConflictConfigurator():

    @staticmethod
    def parse_xml(conflictname):

        config = {}
        # path to scenarios.xml
        config_file_name = "./scenarios/aisa_conflicts.xml"
        tree = ET.parse(config_file_name)
        root = tree.getroot()

        # get ego vehicle actor
        ego_vehicle = root.find('.//actors/actor[@name="{}"]'.format("ego"))
            
        # find scenario by name
        scenario = root.find('.//scenarios/scenario[@name="{}"]'.format(conflictname))

        config["scenario"] = scenario.attrib["name"]
        config["town"] = scenario.attrib["town"]
        config["weather"] = int(scenario.attrib["WeatherId"])
        npc_pedestrians = scenario.attrib["npc_pedestrians"]
        npc_vehicles = scenario.attrib["npc_vehicles"]
        config["sensor_noise"] = float(scenario.attrib["sensor_noise"])

        # get waypoints
        config["wp_start"] = scenario.find('.//waypoints/waypoint[@name="{}"]'.format("start"))
        config["wp_dest"] = scenario.find('.//waypoints/waypoint[@name="{}"]'.format("dest"))

        return config

#config = ConflictConfigurator.parse_xml()







