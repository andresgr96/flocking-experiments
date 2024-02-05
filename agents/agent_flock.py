import random

from vi import Agent
import pygame as pg
import numpy as np
import cv2
from pygame.math import Vector2


# Inherit from Storms agent class
class FlockingAgent(Agent):
    def __init__(self, simulation, images, radius=5, speed=1, alignment_factor=0,  # Might try the matrix config trick
                 cohesion_factor=1.2, separation_factor=0.8, separation_radius=20):
        super().__init__(images, simulation)
        self.radius = radius
        self.speed = speed
        self.alignment_factor = alignment_factor
        self.cohesion_factor = cohesion_factor
        self.separation_factor = separation_factor
        self.separation_radius = separation_radius

    def update(self):
        nearby_agents = list(self.in_proximity_accuracy())

        # If there are neighbors, flock!
        if nearby_agents:
            average_velocity = pg.math.Vector2()
            center_of_mass = pg.math.Vector2()
            separation_vector = pg.math.Vector2()

            # Build nearby agents vectors
            for agent, distance in nearby_agents:
                average_velocity += agent.move
                center_of_mass += agent.pos
                if distance < self.separation_radius:
                    vector_to_agent = agent.pos - self.pos
                    separation_vector += vector_to_agent.normalize()

            # Calculate average vectors
            average_velocity /= len(nearby_agents)
            center_of_mass /= len(nearby_agents)

            # Alignment
            self.move += average_velocity * self.alignment_factor

            # Cohesion
            direction_to_center = center_of_mass - self.pos
            scaled_direction = direction_to_center.normalize() * self.cohesion_factor
            self.move += scaled_direction

            # Separation
            self.move -= separation_vector * self.separation_factor

            # Clamp speed and update position
            self.move.scale_to_length(self.speed)
            self.pos += self.move


class SourceAgent(FlockingAgent):
    def __init__(self, simulation, images, radius=5, speed=2, alignment_factor=1.0,
                 cohesion_factor=1.2, separation_factor=0.8, separation_radius=20,
                 image_path="images/light_source.jpg", epsilon=0.1):
        super().__init__(simulation, images, radius, speed, alignment_factor,
                         cohesion_factor, separation_factor, separation_radius)

        self.gradient_img = cv2.cvtColor(cv2.imread(image_path), cv2.COLOR_BGR2RGB)
        self.img_width, self.img_height, _ = self.gradient_img.shape
        self.gradient = np.sum(self.gradient_img, axis=2, keepdims=True)
        self.epsilon = epsilon

    def update(self):
        # Reset move to not accumulate velocity
        self.move = Vector2(0, 0)
        at_source = False

        # First version will just round the position
        pos_x, pos_y = self.pos
        pos_x, pos_y = round(pos_x), round(pos_y)
        source_direction = self.sense_highest(pos_x, pos_y, 20)

        # Exploitation vs exploration moment
        prob = random.uniform(0, 1)
        explore = prob > self.epsilon

        # Move towards light and handle tradeoff
        if source_direction.length() > 0:
            if explore:
                self.move += Vector2(random.randint(-1, 1), random.randint(-1, 1))
            else:
                x_dir = self.speed if source_direction[0] > pos_x else -self.speed if source_direction[0] < pos_x else 0
                y_dir = self.speed if source_direction[1] > pos_y else -self.speed if source_direction[1] < pos_y else 0
                final_dir = Vector2(x_dir, y_dir)
                self.move += final_dir
        else:
            self.move += Vector2(random.randint(-1, 1), random.randint(-1, 1))

        nearby_agents = list(self.in_proximity_accuracy())

        # If there are neighbors, flock!
        if nearby_agents:
            average_velocity = pg.math.Vector2()
            center_of_mass = pg.math.Vector2()
            separation_vector = pg.math.Vector2()

            # Build nearby agents vectors
            for agent, distance in nearby_agents:
                average_velocity += agent.move
                center_of_mass += agent.pos
                if distance < self.separation_radius:
                    vector_to_agent = agent.pos - self.pos
                    separation_vector += vector_to_agent.normalize()

            # Calculate average vectors
            average_velocity /= len(nearby_agents)
            center_of_mass /= len(nearby_agents)

            # Alignment
            self.move += average_velocity * self.alignment_factor

            # Cohesion
            direction_to_center = center_of_mass - self.pos
            scaled_direction = direction_to_center.normalize() * self.cohesion_factor
            self.move += scaled_direction

            # Separation
            self.move -= separation_vector * self.separation_factor

            # Clamp speed and update position
            self.move.scale_to_length(self.speed)
            self.pos += self.move

    def sense_highest(self, drone_x, drone_y, sensor_range) -> Vector2:

        # Don't move if nothing is better
        coord_vector = Vector2(0, 0)
        max_value = -0.1

        # Search around the current pos for the highest value
        for i in range(max(0, drone_x - sensor_range), min(self.img_width, drone_x + sensor_range + 1)):
            for j in range(max(0, drone_y - sensor_range), min(self.img_height, drone_x + sensor_range + 1)):
                if self.gradient[i][j] > max_value:
                    max_value = self.gradient[i][j]
                    coord_vector = Vector2(i, j)

        return coord_vector




