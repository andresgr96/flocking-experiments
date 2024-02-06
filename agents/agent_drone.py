from vi import Agent
import pygame as pg
import random
from pygame.math import Vector2
from utils import collective_motion
from utils.collective_motion import compute_target_velocities


class DroneAgent(Agent):
    def __init__(self, simulation, images, linear_velocity=Vector2(1, 0), angular_velocity=0.0,
                 target_linear_velocity=Vector2(0, 0), target_angular_velocity=0.0, heading=0.0):
        super().__init__(images, simulation)

        # Add parameters specific to drone movement
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.target_linear_velocity = linear_velocity
        self.target_angular_velocity = angular_velocity
        self.heading = heading if heading is not None else random.uniform(0, 360)

    def update(self):

        # Example of using self.linear_velocity, self.angular_velocity, and self.heading for moving
        # print(collective_motion.proximal_vector_magnitude(self))

        # Compute target velocities
        target_vel, self.angular_velocity = \
            compute_target_velocities(self, self.linear_velocity, self.angular_velocity)

        self.linear_velocity = target_vel
        # Rotate heading based on angular velocity
        self.heading += self.angular_velocity

        # Move in the direction of heading with at linear velocity
        self.linear_velocity = self.linear_velocity.rotate(self.heading)
        print(self.linear_velocity)

