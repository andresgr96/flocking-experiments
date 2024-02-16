from agents.agent_NHDD import NHDDAgent
import pygame as pg
from pygame.math import Vector2
import math
from utils import collective_motion


class TestStatAgent(NHDDAgent):

    """
    This agent requires a position and can be used to confirm calculations like relative angles.

    For it to work you'll need to modify the spawn_agent function in the original simulation file to also take the position,
    and remember to change it back after testing!
    """
    def __init__(self, simulation, images, initial_position, linear_velocity=Vector2(0, 0), angular_velocity=0.0, heading=0.0):
        super().__init__(simulation, images, linear_velocity, angular_velocity, heading)

        # Add the extra parameter for position
        self.pos = Vector2(initial_position)
        self.linear_velocity = Vector2(0, 0)
        self.heading = 0.0

    def update(self):
        neighbor = self.in_proximity_accuracy().without_distance().first()
        # print(f"Current Angle: {self.relative_angle(neighbor)}")
        # print(f"Atahn Angle: {math.degrees(math.atan2(neighbor.pos.y, neighbor.pos.x))}")

    def relative_angle(self, target_agent) -> float:
        """
        Calculate the angle between two agents relative to the focal agent's local frame of reference

        """
        relative_vector = target_agent.pos - self.pos
        print(self.heading)
        rotated_vector = relative_vector.rotate(-self.heading)
        angle = rotated_vector.angle_to(Vector2(1, 0))
        # angle = relative_vector.angle_to(Vector2(1, 0))

        return angle