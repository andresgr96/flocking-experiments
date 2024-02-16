import math
from utils.collective_motion import random_swarm_position
from vi import Agent
import random
from pygame.math import Vector2
import numpy as np

# Final Virtual force coefficients
PROX_WEIGHT: float = 2.0        # Alpha
ALIGN_WEIGHT: float = 4.0       # Beta
AVOID_WEIGHT: float = 1.0       # Gamma

# Strength coefficients
PROX_STR_GAIN: float = 12.0     # e

# Perception Ranges
SENSE_RANGE: float = 20.0       # Dp
B_SENSE_RANGE: float = 0.5      # Dr

# Desired distance variables
DES_DIST: float = 1.11
DES_DIST_COEFF: float = 0.7 # DES_DIST / 2**(1/2)   # Sigma
DIST_COEFF_MAX: float = DES_DIST_COEFF + (DES_DIST_COEFF/2)    # Sigma max
DIST_COEFF_MIN: float = DES_DIST_COEFF - (DES_DIST_COEFF/4)    # Sigma min

# Boundary avoidance
AVOID_GAIN: float = 20.0        # Krep
L_THRESH: float = 1.0           # L0

# Linear velocity
LIN_GAIN: float = 0.06          # K1
MAX_SPEED: float = 0.1       # Umax
MIN_SPEED: float = 0.1          # Umin
LINEAR_BIAS: float = 0.05        # Uc

# Angular velocity
ANG_GAIN: float = 0.5           # K2
MAX_ANG_SPEED: float = 180/2    # Wmax

# Gradient
MAX_SCALAR_VAL: float = 255.0   # Gmax
CORR_EXP: float = 12.0           # Eu
PORTION: float = 0.5            # Pu

# Repulsion
REP_WEIGHT_COEFF: float = 1.0         # Delta

# Env control speed
DT: float = 0.05        # dt

# Others
eliseo: bool = False         # Whether to use eliseo's or tugay's PC formula.
align: bool = False            # Whether to use alignment control or not
pc_only: bool = True          # Whether to do proximal control alone (without source loc)


def single_proximal_vector_magnitude(dist: float) -> float:
    """
    Calculates the magnitude of the vector pointing to the neighbor

    """
    if eliseo:
        return -(4 * PROX_STR_GAIN * 2 / dist) * (
                    (2 * (DES_DIST_COEFF ** 4 / dist)) - ((DES_DIST_COEFF / dist) ** 2))
    else:
        return -PROX_STR_GAIN * (
                    (2 * (DES_DIST_COEFF ** 4 / dist ** 5)) - (DES_DIST_COEFF ** 2 / dist ** 3))


def wrap_to_pi(angle: float) -> float:
    """
    Wraps the given angle to [-360, 360]

    """
    angle_rad = np.deg2rad(angle)
    angle_rad_wrapped = np.arctan2(np.sin(angle_rad), np.cos(angle_rad))
    angle_deg_wrapped = np.rad2deg(angle_rad_wrapped)

    return angle_deg_wrapped


class NHDDAgent(Agent):
    """
    Non-holonomic differential drive agent, only moves at linear_velocity in the direction of its heading.

    The heading is only changed by applying angular_velocity to the agent, rotating its forward movement vector.

    """
    def __init__(self, simulation, images, position, linear_velocity=Vector2(0.15, 0), angular_velocity=0.0, heading=-180):
        super().__init__(images, simulation, position)

        self.simulation = simulation
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.heading = heading if heading is not None else random.uniform(-60, 60)

    def update(self):
        """
        Updates the drone movement.
        """
        print(f"Neighbors: {self.in_proximity_accuracy().count()}")

        # Compute target velocities
        target_linear_vel, target_angular_vel = self.compute_target_velocities()

        # Adjust translation to match the direction of the agents heading
        final_x = target_linear_vel * math.cos(self.heading)
        final_y = target_linear_vel * math.sin(self.heading)

        # Build and update variables
        self.linear_velocity = Vector2(final_x, final_y)
        self.angular_velocity = target_angular_vel

        # Update position for simulator to redraw
        self.pos += self.linear_velocity * DT

        # Update heading based on the new angular velocity
        self.heading += self.angular_velocity * DT

    def compute_target_velocities(self) -> (float, float):
        """
        Computes the target linear and angular of the drone

        """
        # neighbors = self.in_proximity_accuracy().count()
        # align_control_vec = self.alignment_control_force()

        # Calculate control vectors and scale it by the given weights
        forces, ij_angles = self.proximal_control_force()

        # Calculate components of Fi in the local reference frame
        fi_x = PROX_WEIGHT * (forces * math.cos(ij_angles))
        fi_y = PROX_WEIGHT * (forces * math.sin(ij_angles))

        # Adjust for gain and add bias
        target_linear_vel = LIN_GAIN * fi_x + LINEAR_BIAS
        target_angular_vel = ANG_GAIN * fi_y

        print(target_linear_vel, target_angular_vel)

        # Clamp illegal linear and angular velocity values
        target_linear_vel = 0 if target_linear_vel <= 0 else MAX_SPEED if target_linear_vel >= MAX_SPEED\
            else target_linear_vel
        target_angular_vel = -MAX_ANG_SPEED if target_angular_vel <= -MAX_ANG_SPEED else MAX_ANG_SPEED if\
            target_angular_vel >= MAX_ANG_SPEED else target_angular_vel

        return target_linear_vel, target_angular_vel

    # ------------------------------------- Proximal Control Section -------------------------------------

    def proximal_control_force(self) -> (float, float):
        """
        Calculates the final proximal control magnitude and angle values

        """
        final_magnitude: float = 0
        final_angle: float = 0
        for neighbor, distance, angle in self.in_proximity_accuracy():
            final_angle += angle

            # Get the vector pointing from focal to neighbor, its magnitude and angle
            magnitude = single_proximal_vector_magnitude(distance) if pc_only\
                else self.single_proximal_vector_magnitude_mod(distance)

            final_magnitude += magnitude

        final_angle = wrap_to_pi(final_angle)
        return final_magnitude, final_angle


    def single_proximal_vector_magnitude_mod(self, dist: float) -> float:
        """
        Calculates the magnitude of the vector pointing to the neighbor for desired distance modulation

        """
        scalar_val = self.get_scalar()

        dist_coeff = DIST_COEFF_MIN + ((scalar_val / MAX_SCALAR_VAL) * (DIST_COEFF_MAX - DIST_COEFF_MIN))

        if eliseo:
            return -(4 * PROX_STR_GAIN * 2 / dist) * (
                    (2 * (dist_coeff ** 4 / dist ** 1)) - ((dist_coeff / dist) ** 2))
        else:
            return -PROX_STR_GAIN * (
                    (2 * (dist_coeff ** 4 / dist ** 5)) - (dist_coeff ** 2 / dist ** 3))

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

    # ------------------------------------- Speed Modulation Section -------------------------------------

    def repulsion_effect_force(self, pi: Vector2) -> Vector2:
        """
        Calculates the final repulsion effect according to the sensed scalar value vector for speed modulation

        Needs updating for the actual value of pr instead of directly using pi

        """
        scalar_val = self.get_scalar()

        scaled_vec = pi * REP_WEIGHT_COEFF
        rep_weight = scalar_val / MAX_SCALAR_VAL

        return scaled_vec * rep_weight

    def modulate_linear_vel(self, lin_vel: float) -> float:
        """
        Modulates the linear velocity of the agent according to the sensed scalar value for speed modulation

        """
        scalar_val = self.get_scalar()
        u_portion = lin_vel * (1 - (PORTION * (scalar_val / MAX_SCALAR_VAL) ** CORR_EXP))
        u_sign = math.copysign(1, lin_vel)

        return max(u_portion, (lin_vel * u_sign))

    # ------------------------------------- Utilities Section -------------------------------------

    def get_scalar(self):
        """
        Returns the scalar value of the gradient at the current position of the drone
        """

        return self.simulation._proximity.sense_scalar(self)








