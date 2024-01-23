from agent_flock import FlockingAgent
import pygame as pg
import cv2
import numpy as np


# Extend the flocking functionality with hill climbing
class SourceAgent(FlockingAgent):
    def __init__(self, simulation, images, radius=5, speed=2, alignment_factor=1.0,
                 cohesion_factor=1.2, separation_factor=0.8, separation_radius=20,
                 image_path="../images/light_source.jpg", ):
        super().__init__(simulation, images, radius, speed, alignment_factor,
                         cohesion_factor, separation_factor, separation_radius)
        # Set the gradient
        self.gradient_img = cv2.cvtColor(cv2.imread(image_path), cv2.COLOR_BGR2RGB)
        self.gradient = np.sum(self.gradient_img, axis=2, keepdims=True)
        print(f"Gradient Shape: {self.gradient.shape}")

    def update(self):
        super().update()
        # print(self.gradient[0][0])





