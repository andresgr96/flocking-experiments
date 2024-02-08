import math

from vi import Agent
import random
from pygame.math import Vector2

# Final Virtual force coefficients
PROX_WEIGHT: float = 1.0     # Alpha
ALIGN_WEIGHT: float = 2.0    # Beta
AVOID_WEIGHT: float = 1.0    # Gamma

# Strength coefficients
PROX_STR_GAIN: float = 5.0  # e

# Perception Ranges
SENSE_RANGE: float = 2.0     # Dp
B_SENSE_RANGE: float = 20.0  # Dr

# Desired distance variables
DES_DIST_COEFF: float = 1.0  # Sigma

# Boundary avoidance
AVOID_GAIN: float = 20.0    # Krep
L_THRESH: float = 1.0       # L0

# Biases
LINEAR_BIAS: float = 0.05   # Uc

# Speed
LIN_GAIN: float = 0.1     # K1
ANG_GAIN: float = 0.5      # K2
MAX_SPEED: float = 1.0              # Umax
MAX_ANG_SPEED: float = 90/3    # Wmax
CTRL_SPEED: float = 0.5             #dt


def single_proximal_vector_magnitude(dist: float) -> float:
    """
    Calculates the magnitude of the vector pointing to the neighbor

    """

    return -PROX_STR_GAIN * ((2 * (DES_DIST_COEFF ** 4 / dist ** 5)) - (DES_DIST_COEFF ** 2 / dist ** 3))


class NHDDAgent(Agent):
    def __init__(self, simulation, images, linear_velocity=None, angular_velocity=None, heading=None):
        super().__init__(images, simulation)

        self.simulation = simulation
        self.linear_velocity = linear_velocity if linear_velocity is not None else Vector2(random.uniform(0.5, MAX_SPEED))
        self.angular_velocity = angular_velocity if angular_velocity is not None else random.uniform(20, 360)
        self.heading = heading if heading is not None else random.uniform(20, 360)

    def update(self):
        """
        Updates the drone movement.
        """

        neighbors = self.in_proximity_accuracy().count() > 0

        # Compute target velocities
        target_linear_vel, target_angular_vel = self.compute_target_velocities()
        print(target_angular_vel)

        if neighbors:
            # Update angular velocity based on the target angular velocity
            self.angular_velocity = target_angular_vel

            # Update heading based on the new angular velocity
            self.heading += self.angular_velocity * self.simulation._clock.get_time() * CTRL_SPEED

            # Update linear velocity based on the target linear velocity
            self.linear_velocity = Vector2(target_linear_vel, 0)

            # Rotate the linear velocity according to the heading
            self.linear_velocity.rotate_ip(self.heading)
            self.linear_velocity.y = -self.linear_velocity.y

        # if neighbors:
        #     self.angular_velocity = target_angular_vel
        #     self.heading = self.angular_velocity
        #     self.linear_velocity = Vector2(target_linear_vel[0], target_angular_vel)

    def compute_target_velocities(self) -> (Vector2, float):
        """
        Computes the target linear and angular of the drone

        """

        # Calculate the proximal control vector and scale it by the given weight
        prox_control_vec = self.proximal_control_force()
        virtual_force_vec = (PROX_WEIGHT * prox_control_vec)

        # Rotate the force vector to align with the agent's heading
        rotated_force_vec = virtual_force_vec.rotate(-self.heading)

        # Adjust for gain and add bias
        target_linear_vel = LIN_GAIN * rotated_force_vec[0] + LINEAR_BIAS
        target_angular_vel = ANG_GAIN * rotated_force_vec[1]

        # Clamp illegal values
        target_linear_vel = 0 if target_linear_vel <= 0 else MAX_SPEED if target_linear_vel >= MAX_SPEED\
            else target_linear_vel

        target_angular_vel = 0 if target_angular_vel <= 0 else MAX_ANG_SPEED if target_angular_vel >= MAX_ANG_SPEED\
            else target_angular_vel

        # Build linear velocity vector
        target_vel = Vector2(target_linear_vel, 0)

        return target_vel, target_angular_vel

    # ------------------------------------- Proximal Control Section -------------------------------------
    def proximal_control_force(self) -> Vector2:
        """
        Calculates the final proximal control vector weighted by a set coefficient

        """
        force_vector = Vector2(0, 0)
        #                                                     Fake sensor
        for neighbor, distance, rel_angle, diff_vec in self.in_proximity_accuracy():

            # Get the vector pointing from focal to neighbor, its magnitude and angle
            magnitude = single_proximal_vector_magnitude(distance)

            # Scale to magnitude and rotate
            scaled_vec = diff_vec * magnitude
            rotated_vec = scaled_vec.rotate(rel_angle)

            # Add the result to the total force vector
            force_vector += rotated_vec

        return force_vector








