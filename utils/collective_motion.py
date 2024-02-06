import pygame as pg
import random
from pygame.math import Vector2


def compute_target_velocities(linear_velocity: Vector2, angular_velocity: float) -> (Vector2, float):
    """
    Computes the target linear and angular of the drone given its current values.

    """
    target_vel = Vector2(1, 1)
    target_ang_vel = 5.0

    return target_vel, target_ang_vel


# -------------------------------- Proximal Control Section --------------------------------
def proximal_control_force() -> Vector2:
    pass


def proximal_vector_magnitude() -> float:
    pass


def proximal_vector_angle() -> float:
    pass

# -------------------------------- Alignment Control Section --------------------------------


def alignment_control_force() -> Vector2:
    pass


# -------------------------------- Boundary Avoidance Section --------------------------------
def boundary_avoidance_force() -> Vector2:
    pass

