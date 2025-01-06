# ==============================================================================
# -- Custom Functions --------------------------------------------------------
# ==============================================================================

import carla
from agents.navigation.basic_agent import (
    BasicAgent,
)

def parse_spawn_point(point_string: str) -> carla.Transform:
    """
    Expected format of point_string: (xx.xx,yy.yy,zz.zz)
    """
    x, y, z = point_string.replace("(", "").replace(")", "").split(",")
    loc = carla.Location(float(x), float(y), float(z))
    return carla.Transform(loc, carla.Rotation())

def draw_route(agent: BasicAgent, world: carla.World):
    """Draws waypoints of the predicted route by the global planner."""
    waypoint_queue = agent.get_local_planner()._waypoints_queue
    for wq in waypoint_queue:
        loc = wq[0].transform.location
        world.debug.draw_point(
            loc, size=0.1, color=carla.Color(255, 255, 0), life_time=120
        )

def draw_trajectory(trajectory: list, world: carla.World, actor: carla.Actor):
    actor_loc = actor.get_transform().location
    for wp in trajectory:
        loc = carla.Location(x=wp[1]+actor_loc.x, y=wp[0]+actor_loc.y, z = actor_loc.z)
        world.debug.draw_point(
            loc, size=0.1, color=carla.Color(255, 0, 255), life_time=1
        )
