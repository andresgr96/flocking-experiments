from agents.agent_NHDD import NHDDAgent
import pygame as pg
from pygame.math import Vector2
import math
from utils import collective_motion
import matplotlib.pyplot as plt
import numpy as np

DES_DIST: float = 9
DES_DIST_COEFF: float = DES_DIST / 2**(1/2)   # Sigma
PROX_STR_GAIN: float = 12.0     # e
DT: float = 0.05        # dt
SCALE_FACTOR = 100000000000  # You can adjust this factor based on the scale of your visualization


def wrap_to_pi(angle: float) -> float:
    angle_rad = np.deg2rad(angle)
    angle_rad_wrapped = np.arctan2(np.sin(angle_rad), np.cos(angle_rad))
    angle_deg_wrapped = np.rad2deg(angle_rad_wrapped)

    return angle_deg_wrapped
class TestNHDDAgent(NHDDAgent):

    """
    This agent requires a position and can be used to confirm calculations like relative angles.

    """
    def __init__(self, simulation, images, initial_position, linear_velocity=Vector2(0, 0), angular_velocity=0.0, heading=-180):
        super().__init__(simulation, images, linear_velocity, angular_velocity, heading)

        # Add the extra parameter for position
        self.pos = Vector2(initial_position)
        self.linear_velocity = Vector2(0, 0)
        self.heading = -180

    # def update(self):
    #     print(wrap_to_pi(770))
    #     neighbors = self.in_proximity_accuracy()
    #
    #     for neighbor, dist, angle in neighbors:
    #         # angle = -angle
    #         force = -PROX_STR_GAIN * (
    #                 (2 * (DES_DIST_COEFF ** 4 / dist ** 5)) - (DES_DIST_COEFF ** 2 / dist ** 3))
    #
    #         # Calculate the vector to be added using the current approach
    #         vec = Vector2(force, 0)
    #         curr_approach_vec = vec.rotate(-self.heading) * SCALE_FACTOR
    #         curr_approach_vec= -curr_approach_vec
    #
    #         # Calculate the vector using cosine and sine of the angle and heading
    #         cosine_component = force * math.cos(angle)
    #         sine_component = force * math.sin(angle)
    #         cosine_sine_vec = Vector2(cosine_component, -sine_component) * SCALE_FACTOR
    #
    #         # Test prints
    #         print(f"Desired Distance: {DES_DIST}")
    #         print(f"Relative Distance: {dist}")
    #         print(f"Focal Heading: {self.heading}")
    #         print(f"Calculated Angle: {angle}")
    #         # print(f"Pygame Angle: {self.relative_angle(neighbor)}")
    #         print(f"Atahn Angle: {math.degrees(math.atan2(-neighbor.pos.y, neighbor.pos.x))}")
    #         print(f"Curr Approach Vec: {curr_approach_vec}")  # Vector that currently would be added to self.pos
    #         print(f"Cosine Approach Vec: {cosine_sine_vec}")  # Vector that currently would be added to self.pos
    #
    #         # For visualization
    #         scaled_curr_approach_vec = curr_approach_vec * SCALE_FACTOR
    #         scaled_cosine_sine_vec = cosine_sine_vec * SCALE_FACTOR
    #
    #         # Plotting
    #         plt.figure()
    #         plt.quiver(0, 0, scaled_curr_approach_vec.x, scaled_curr_approach_vec.y, angles='xy', scale_units='xy',
    #                    scale=1, color='r', label='Curr Approach Vec')
    #         plt.quiver(0, 0, scaled_cosine_sine_vec.x, scaled_cosine_sine_vec.y, angles='xy', scale_units='xy', scale=1,
    #                    color='b', label='Cosine-Sine Vec')
    #         plt.legend()
    #         plt.xlim(-10, 10)  # Adjust the limits based on your data
    #         plt.ylim(-10, 10)  # Adjust the limits based on your data
    #         plt.gca().invert_yaxis()  # Invert the y-axis
    #         plt.show()


    def relative_angle(self, target_agent) -> float:
        """
        Calculate the angle between two agents relative to the focal agent's local frame of reference

        """
        relative_vector = target_agent.pos - self.pos
        rotated_vector = relative_vector.rotate(-self.heading)
        angle = rotated_vector.angle_to(Vector2(1, 0))
        # angle = relative_vector.angle_to(Vector2(1, 0))

        return angle
