from vi import Agent
import pygame as pg
import random
from pygame.math import Vector2


class DroneAgent(Agent):
    def __init__(self, simulation, images, linear_velocity=Vector2(0, 0), angular_velocity=0.0,
                 target_linear_velocity=Vector2(0, 0), target_angular_velocity=0.0, heading=None):
        super().__init__(images, simulation)

        # Add parameters specific to drone movement
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.target_linear_velocity = linear_velocity
        self.target_angular_velocity = angular_velocity
        self.heading = heading if heading is not None else random.uniform(0, 360)

    def update(self):

        # Example of using self.linear_velocity, self.angular_velocity, and self.heading for moving
        self.linear_velocity = Vector2(1, 1)
        self.angular_velocity = 0.0

        # Rotate heading based on angular velocity
        self.heading += self.angular_velocity

        # Move in the direction of heading with linear velocity
        self.move = self.linear_velocity.rotate(self.heading)

