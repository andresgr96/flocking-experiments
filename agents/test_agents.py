from agents.agent_NHDD import NHDDAgent
import pygame as pg
from pygame.math import Vector2
from vi import Agent
from utils import collective_motion

rotate = True

class TestNHDDAgent(NHDDAgent):
    """
    This agent requires a position and can be used to confirm calculations like relative angles.

    For it to work you'll need to modify the spawn_agent function in the original simulation file to also take the position,
    and remember to change it back after testing!
    """

    def __init__(
        self,
        simulation,
        images,
        initial_position,  # Add the extra parameter for position
        heading,
        linear_velocity=Vector2(0, 0),
        angular_velocity=0.0,

    ):
        super().__init__(simulation, images, linear_velocity, angular_velocity, heading)

        # Use the initial_position parameter to set the position
        self.pos = Vector2(initial_position)
        self.rotate = True

    def update(self):

        neighbors = self.in_proximity_accuracy()

        for neighbor, dist, calculated_angle in neighbors:
            print("------------------------------------------------")
            print(f"Distance: {dist}")
            print(f"Sensor Angle: {calculated_angle}")
            print(f"Test: Angle: {self.relative_angle(neighbor)}")
            print(self.heading)

    def relative_angle(self, neighbor_agent: Agent) -> float:
        """
        Calculate the angle between two agents relative to the focal agents local frame of reference

        """
        diff_vector = neighbor_agent.pos - self.pos

        # Rotate the vector to align with the x-axis of the focal agents local frame
        rotated_vector = diff_vector.rotate(-self.heading)

        return rotated_vector.angle_to(Vector2(1, 0))


class StationaryAgent(NHDDAgent):
    """
    A stationary agent that inherits from TestNHDDAgent.
    """

    def __init__(
            self,
            simulation,
            images,
            initial_position,  # Add the extra parameter for position
            linear_velocity=Vector2(0, 0),
            angular_velocity=0.0,
            heading=0.0,
    ):
        # Call the constructor of the parent class (TestNHDDAgent)
        super().__init__(simulation, images, linear_velocity, angular_velocity, heading)
        self.pos = Vector2(initial_position)

    def update(self):
        pass

    def change_position(self):
        pass


