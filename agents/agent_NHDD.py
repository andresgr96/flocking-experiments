import math

from vi import Agent
import random
from pygame.math import Vector2

# Final Virtual force coefficients
PROX_WEIGHT: float = 1.0        # Alpha
ALIGN_WEIGHT: float = 2.0       # Beta
AVOID_WEIGHT: float = 1.0       # Gamma

# Strength coefficients
PROX_STR_GAIN: float = 12.0     # e

# Perception Ranges
SENSE_RANGE: float = 20.0        # Dp
B_SENSE_RANGE: float = 0.5      # Dr

# Desired distance variables
DES_DIST_COEFF: float = 30.0      # Sigma

# Boundary avoidance
AVOID_GAIN: float = 20.0        # Krep
L_THRESH: float = 1.0           # L0

# Linear velocity
LIN_GAIN: float = 0.06          # K1
MAX_SPEED: float = 0.2          # Umax
LINEAR_BIAS: float = 0.1        # Uc

# Angular velocity
ANG_GAIN: float = 0.8           # K2
MAX_ANG_SPEED: float = 180/3    # Wmax

# Env control speed
CTRL_SPEED: float = 0.01        # dt

# To include alignment control or not
align: bool = False


def single_proximal_vector_magnitude(dist: float) -> float:
    """
    Calculates the magnitude of the vector pointing to the neighbor (Eliseo's 2012 paper formula)

    """

    return -(4*PROX_STR_GAIN*2/dist) * ((2 * (DES_DIST_COEFF **(2*2) / dist ** 1)) - (DES_DIST_COEFF ** 2 / dist ** 2))


class NHDDAgent(Agent):
    """
    Non-holonomic differential drive agent, only moves at linear_velocity in the direction of its heading.

    The heading is only changed by applying angular_velocity to the agent, rotating its forward movement vector.

    """
    def __init__(self, simulation, images, linear_velocity=Vector2(0.1, 0), angular_velocity=0.0, heading=None):
        super().__init__(images, simulation)

        self.simulation = simulation
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.heading = heading if heading is not None else random.uniform(-180, 180)

    def update(self):
        """
        Updates the drone movement.
        """
        # Compute target velocities
        target_linear_vel, target_angular_vel = self.compute_target_velocities()

        # Update angular velocity based on the target angular velocity
        self.angular_velocity = target_angular_vel

        # Update heading based on the new angular velocity
        self.heading += self.angular_velocity * CTRL_SPEED

        # Update linear velocity based on the target linear velocity
        self.linear_velocity = target_linear_vel

        # Rotate the linear velocity vector according to the heading
        self.linear_velocity.rotate_ip(self.heading)
        self.linear_velocity.y = -self.linear_velocity.y

        # Update position for simulator to redraw
        self.pos += self.linear_velocity * CTRL_SPEED

    def compute_target_velocities(self) -> (Vector2, float):
        """
        Computes the target linear and angular of the drone

        """
        # Calculate the proximal control vector and scale it by the given weight
        prox_control_vec = self.proximal_control_force()
        align_control_vec = self.alignment_control_force()

        virtual_force_vec = ((PROX_WEIGHT * prox_control_vec) + (ALIGN_WEIGHT * align_control_vec)) if align\
            else (PROX_WEIGHT * prox_control_vec)

        # Project force onto the agents local frame of reference
        rotated_force_vec = virtual_force_vec.rotate(-self.heading)

        # Adjust for gain and add bias
        target_linear_vel = LIN_GAIN * rotated_force_vec[0] + LINEAR_BIAS
        target_angular_vel = ANG_GAIN * rotated_force_vec[1]

        # Clamp illegal linear and angular velocity values to avoid snappy movements
        target_linear_vel = 0 if target_linear_vel <= 0 else MAX_SPEED if target_linear_vel >= MAX_SPEED\
            else target_linear_vel
        target_angular_vel = -MAX_ANG_SPEED if target_angular_vel <= -MAX_ANG_SPEED else MAX_ANG_SPEED if\
            target_angular_vel >= MAX_ANG_SPEED else target_angular_vel

        # Build linear velocity vector
        target_vel = Vector2(target_linear_vel, 0)

        return target_vel, target_angular_vel

    def proximal_control_force(self) -> Vector2:
        """
        Calculates the final proximal control vector weighted by a set coefficient

        """
        force_vector = Vector2(0, 0)
        #                                                     Fake sensor
        for neighbor, distance, rel_angle, diff_vec in self.in_proximity_accuracy():
            # Get the vector pointing from focal to neighbor, its magnitude and angle
            magnitude = single_proximal_vector_magnitude(distance)
            polar_vector = Vector2(magnitude, rel_angle)
            force_vector += polar_vector

        return force_vector

    # ------------------------------------- Alignment Control Section -------------------------------------

    def alignment_control_force(self) -> Vector2:
        """
        Calculates the final alignment control force vector

        """
        # Get neighbor headings and express focal agents heading as vector
        focal_heading = Vector2(1, 0).rotate(self.heading)
        neighbors_heading = self.sum_neighbor_headings()

        # Calculate total heading and divide by its magnitude to get the final alignment control vector
        total_heading = focal_heading + neighbors_heading
        alignment_vector = total_heading / (total_heading.length())

        return alignment_vector

    def sum_neighbor_headings(self) -> Vector2:
        """
        Calculates the sum of the focal agents neighbor headings.

        We assume that Pygame's world reference works as global reference, checking needed!!

        """
        total_heading = Vector2(0, 0)

        for neighbor, _, _ in self.in_proximity_accuracy():
            neighbor_vector_heading = Vector2(1, 0).rotate(neighbor.heading)
            total_heading += neighbor_vector_heading

        return total_heading








