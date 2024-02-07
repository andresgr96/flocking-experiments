import pygame as pg
import random
from pygame.math import Vector2
from vi import ProximityIter, Agent
# from agents.agent_drone import DroneAgent

# Final Virtual force coefficients
PROX_WEIGHT: float = 1.0
ALIGN_WEIGHT: float = 2.0
AVOID_WEIGHT: float = 1.0

# Strength coefficients
E: float = 12.0

# Perception Ranges
DP: float = 2.0
DR: float = 0.5

# Desired distance variables
DESIRED_DIST_COEFF: float = 2.0
DESIRED_DIST: float = (2**(1/2)) * DESIRED_DIST_COEFF

# Boundary avoidance
AVOID_GAIN: float = 2.0
L_THRESH: float = 0.5


def compute_target_velocities(focal_drone: Agent, linear_velocity: Vector2, angular_velocity: float)\
        -> (Vector2, float):
    """
    Computes the target linear and angular of the drone given its current values.

    Placeholder!

    """
    focal_pos = focal_drone.pos

    target_vel = Vector2(1, 0)
    target_ang_vel = -10.0

    return target_vel, target_ang_vel


# -------------------------------- Proximal Control Section --------------------------------
def proximal_control_force(focal_agent: Agent) -> Vector2:
    """
    Calculates the final proximal control vector weighted by a set coefficient

    """
    force_vector = Vector2(0, 0)

    for neighbor, distance in focal_agent.in_proximity_accuracy():

        # Get the vector pointing from focal to neighbor, its magnitude and angle
        diff_vec = neighbor.pos - focal_agent.pos
        magnitude = single_proximal_vector_magnitude(distance)
        angle = single_proximal_vector_angle(focal_agent, neighbor)

        # Scale to magnitude and rotate
        scaled_vec = diff_vec * magnitude
        rotated_vec = scaled_vec.rotate(angle)

        # Add the result to the total force vector
        force_vector += rotated_vec

    return force_vector


def single_proximal_vector_magnitude(dist: float) -> float:
    """
    Calculates the magnitude of the vector pointing from the focal agent to one neighbor

    """

    return -E * ((2 * (DESIRED_DIST_COEFF**4/dist**5)) - (DESIRED_DIST_COEFF**2/dist**3))


def single_proximal_vector_angle(focal_agent: Agent, neighbor_agent: Agent) -> float:
    """
    Calculates the angle from one agent position to its neighbor.

    Might be that pygame uses the global reference frame, checking needed!!

    """

    return focal_agent.pos.angle_to(neighbor_agent.pos)

# -------------------------------- Alignment Control Section --------------------------------


def alignment_control_force(focal_agent: Agent) -> Vector2:

    # Get headings
    focal_heading = Vector2(1, 0).rotate(focal_agent.heading)
    neighbors_heading = sum_neighbor_headings(focal_agent)

    # Calculate total heading and divide by its length to get the final alignment control vector
    total_heading = focal_heading + neighbors_heading
    alignment_vector = total_heading / (total_heading.length())

    return alignment_vector


def sum_neighbor_headings(focal_agent: Agent) -> Vector2:
    """
    Calculates the sum of the focal agents neighbor headings.

    We assume that Pygame's world reference works as global reference, checking needed!!

    """
    total_heading = Vector2(0, 0)

    for neighbor, _ in focal_agent.in_proximity_accuracy():

        neighbor_vector_heading = Vector2(1, 0).rotate(neighbor.heading)
        total_heading += neighbor_vector_heading

    return total_heading


# -------------------------------- Boundary Avoidance Section --------------------------------
def boundary_avoidance_force() -> Vector2:
    pass

