from agents.agent_NHDD import NHDDAgent
import pygame as pg
from pygame.math import Vector2
from utils import collective_motion


class TestNHDDAgent(NHDDAgent):

    """
    This agent requires a position and can be used to confirm calculations like relative angles.

    For it to work you'll need to modify the spawn_agent function in the original simulation file to also take the position,
    and remember to change it back after testing!
    """
    def __init__(self, simulation, images, position, linear_velocity=Vector2(0, 0), angular_velocity=0.0, heading=0.0):
        super().__init__(simulation, images, linear_velocity, angular_velocity, heading)

        # Add the extra parameter for position
        self.pos = Vector2(position)

    def update(self):
        neighbor = self.in_proximity_accuracy().without_distance().first()
        print(collective_motion.relative_angle(self, neighbor))