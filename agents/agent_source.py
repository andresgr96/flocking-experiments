from agent_flock import FlockingAgent
import pygame as pg
import cv2
import numpy as np
from pygame.math import Vector2
import random


# Extend the flocking functionality with hill climbing
class SourceAgent(FlockingAgent):
    def __init__(self, simulation, images, radius=5, speed=2, alignment_factor=1.0,
                 cohesion_factor=1.2, separation_factor=0.8, separation_radius=20,
                 image_path="images/light_source.jpg", epsilon = 0.5):
        # Call the constructor of the parent class (FlockingAgent)
        super().__init__(simulation, images, radius, speed, alignment_factor,
                         cohesion_factor, separation_factor, separation_radius)

        self.gradient_img = cv2.cvtColor(cv2.imread(image_path), cv2.COLOR_BGR2RGB)
        self.img_width, self.img_height, _ = self.gradient_img.shape
        self.gradient = np.sum(self.gradient_img, axis=2, keepdims=True)
        self.epsilon = epsilon

    def update(self):
        # Perform flocking calculations
        # super().update()

        # Then move towards light
        self.move = Vector2(0, 0)

        # First version will just round the pos and wont probably be able to move diagonally properly
        pos_x, pos_y = self.pos
        pos_x, pos_y = round(pos_x), round(pos_y)
        source_direction = self.sense_highest(pos_x, pos_y, 20)

        # Exploitation vs exploration moment
        prob = random.uniform(0, 1)
        explore = self.epsilon > prob

        # Exploit
        if source_direction.length() > 0:
            if explore:
                self.move += Vector2(random.randint(-1, 1), random.randint(-1, 1))
            else:
                x_dir = self.speed if source_direction[0] > pos_x else -self.speed if source_direction[0] < pos_x else 0
                y_dir = self.speed if source_direction[1] > pos_y else -self.speed if source_direction[1] < pos_y else 0
                final_dir = Vector2(x_dir, y_dir)
                self.move += final_dir
        else:
            self.move = Vector2(0, 0)

    def sense_highest(self, drone_x, drone_y, sensor_range) -> Vector2:

        # Return your own position if nothing is better
        coord_vector = Vector2(0, 0)
        max_value = -0.1

        # Search around the current pos for the highest value
        for i in range(max(0, drone_x - sensor_range), min(self.img_width, drone_x + sensor_range + 1)):
            for j in range(max(0, drone_y - sensor_range), min(self.img_height, drone_x + sensor_range + 1)):
                if self.gradient[i][j] > max_value:
                    max_value = self.gradient[i][j]
                    coord_vector = Vector2(i, j)

        return coord_vector





