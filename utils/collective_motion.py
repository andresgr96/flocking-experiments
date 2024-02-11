import pygame as pg
import random
from pygame.math import Vector2
from vi import ProximityIter, Agent
from shapely.geometry import Polygon, Point

# Final Virtual force coefficients
PROX_WEIGHT: float = 1.0     # Alpha
ALIGN_WEIGHT: float = 2.0    # Beta
AVOID_WEIGHT: float = 1.0    # Gamma

# Strength coefficients
PROX_STR_GAIN: float = 12.0  # e

# Perception Ranges
SENSE_RANGE: float = 2.0     # Dp
B_SENSE_RANGE: float = 20.0  # Dr

# Desired distance variables
DES_DIST_COEFF: float = 2.0  # Sigma

# Boundary avoidance
AVOID_GAIN: float = 20.0    # Krep
L_THRESH: float = 1.0       # L0

# Biases
LINEAR_BIAS: float = 0.05   # Uc

# Speed Gains
LIN_GAIN: float = 0.5      # K1
ANG_GAIN: float = 0.4       # K2


def compute_target_velocities(focal_agent: Agent) -> (Vector2, float):
    """
    Computes the target linear and angular of the drone given its current values.

    """

    prox_control_vec = proximal_control_force(focal_agent)
    align_control_vec = alignment_control_force(focal_agent)
    bound_avoidance_vec = boundary_avoidance_force(focal_agent)

    virtual_force_vec = (PROX_WEIGHT * prox_control_vec) + (ALIGN_WEIGHT * align_control_vec)\
                        + (AVOID_WEIGHT * bound_avoidance_vec)

    target_linear_vel = LIN_GAIN * virtual_force_vec[0] + LINEAR_BIAS
    target_angular_vel = ANG_GAIN * virtual_force_vec[1]

    # print(target_linear_vel, target_angular_vel)
    target_vel = Vector2(target_linear_vel, 0)

    return target_vel, target_angular_vel


# ------------------------------------- Proximal Control Section -------------------------------------
def proximal_control_force(focal_agent: Agent) -> Vector2:
    """
    Calculates the final proximal control vector weighted by a set coefficient

    """
    force_vector = Vector2(0, 0)

    for neighbor, distance, rel_angle in focal_agent.in_proximity_accuracy():
        print(rel_angle)

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

    return -PROX_STR_GAIN * ((2 * (DES_DIST_COEFF**4/dist**5)) - (DES_DIST_COEFF**2/dist**3))


def single_proximal_vector_angle(focal_agent: Agent, neighbor_agent: Agent) -> float:
    """
    Calculate the angle between two agents relative to the focal agents local frame of reference

    """
    diff_vector = neighbor_agent.pos - focal_agent.pos

    # Rotate the vector to align with the x-axis of the focal agents local frame
    rotated_vector = diff_vector.rotate(-focal_agent.heading)

    return rotated_vector.angle_to(Vector2(1, 0))

# ------------------------------------- Alignment Control Section -------------------------------------


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

    for neighbor, _ , _ in focal_agent.in_proximity_accuracy():

        neighbor_vector_heading = Vector2(1, 0).rotate(neighbor.heading)
        total_heading += neighbor_vector_heading

    return total_heading


# ------------------------------------- Boundary Avoidance Section -------------------------------------
def boundary_avoidance_force(focal_agent: Agent) -> Vector2:
    """
    Calculates the total boundary avoidance force for the focal agent

    Pygame's inverted y-coordinate might be poisoning the results, checking needed!

    """
    total_avoidance_vector = Vector2(0, 0)
    detected = detect_boundaries(focal_agent)

    # If no edges are detected, no force is applied
    if len(detected) == 0:
        return total_avoidance_vector
    else:
        for boundary in detected:
            avoidance_vector = single_boundary_magnitude(focal_agent, boundary)
            total_avoidance_vector += avoidance_vector

    return total_avoidance_vector


def single_boundary_magnitude(focal_agent: Agent, boundary: Polygon) -> Vector2:
    """
    Returns the magnitude of the avoidance vector from focal agent to the given edge

    """
    shortest_dist = distance_to_boundary(focal_agent, boundary)
    dir_to_edge = unit_vector_to_closest_point(focal_agent, boundary)

    return AVOID_GAIN * ((1/shortest_dist) - (1/L_THRESH)) * (dir_to_edge/shortest_dist**3)


def unit_vector_to_closest_point(focal_agent: Agent, boundary: Polygon) -> Vector2:
    """
    Calculates the unit vector pointing from the focal agent to the closest point on the given boundary

    Pygame's inverted y-axis might affect the calculations, checking needed!!

    """
    # Transform drone coordinates into a shapely point
    x, y = focal_agent.pos
    drone_pos = Point(x, y)

    # Find the closest point on the boundary to the agent and get the difference vector
    closest_point = boundary.exterior.interpolate(boundary.exterior.project(drone_pos))
    vec_to_closest_point = Vector2(closest_point.x - x, closest_point.y - y)

    # Normalize the vector to obtain the unit vector
    unit_vector = vec_to_closest_point.normalize()

    return unit_vector


def detect_boundaries(focal_agent: Agent) -> list[Polygon]:
    """
    Returns the boundaries within the boundary sensing range of the drone

    """
    detected = []

    for boundary in focal_agent.simulation.boundaries:
        dist = distance_to_boundary(focal_agent, boundary)

        if dist <= B_SENSE_RANGE:
            detected.append(boundary)

    return detected


def distance_to_boundary(focal_agent: Agent, boundary: Polygon) -> float:
    """
    Calculates the distance between the focal agent and the given boundary

    Pygame's inverted y-axis might affect the calculations, checking needed!!

    """

    # Transform drone coordinates into a shapely point
    x, y = focal_agent.pos
    drone_pos = Point(x, y)

    return drone_pos.distance(boundary)


def random_swarm_position(radious) -> Vector2:

    env_center = 250
    rand_x = random.uniform(env_center - radious, env_center + radious)
    rand_y = random.uniform(env_center - radious, env_center + radious)

    return Vector2(rand_x, rand_y)



