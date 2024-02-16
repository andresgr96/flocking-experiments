from __future__ import annotations
from vi import Agent, Config, Simulation
from typing import TYPE_CHECKING, Optional, Type, TypeVar
import cv2
from shapely.geometry import Polygon
import pygame as pg
from utils.sensor import Sensor
import numpy as np
import math
from pygame.math import Vector2
from agents.agent_NHDD import NHDDAgent

class SourceSim(Simulation):
    """
        This class inherits from Simulation, but has the extra parameter to have an image as custom background.
    """
    _background: pg.surface.Surface
    _clock: pg.time.Clock
    _screen: pg.surface.Surface

    def __init__(self, image, config: Optional[Config] = None):
        super().__init__(config)

        pg.display.init()
        pg.display.set_caption("Violet")

        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        img_width, img_height, _ = image_rgb.shape
        self._gradient = np.sum(image_rgb, axis=2, keepdims=True)

        size = (img_width, img_height)
        # size = (100, 100)
        self._screen = pg.display.set_mode(size, pg.RESIZABLE)  # Make the window resizable

        # Initialise background
        self._background = pg.surfarray.make_surface(image)

        # Show background immediately (before spawning agents)
        self._screen.blit(self._background, (0, 0))
        pg.display.flip()

        # Add the boundaries to the environment
        # self.boundaries = self.add_boundaries()

        # Initialise the clock. Used to cap FPS.
        self._clock = pg.time.Clock()

        self._proximity = Sensor(self._agents, self.config.radius, self._gradient)

    def batch_spawn_agents(
        self,
        count: int,
        agent_class: Type[Agent],
        images: list[str],
        sep_distance: float,
    ) -> "SourceSim":
        # Load images once so the files don't have to be read multiple times.
        loaded_images = self._load_images(images)

        # Calculate spacing between agents based on the desired separation distance
        spacing = sep_distance

        # Calculate grid size based on the desired separation distance
        grid_size = int(math.sqrt(count))  # assuming a square grid

        # Center of the environment
        center_x, center_y = 250, 250

        for i in range(grid_size):
            for j in range(grid_size):
                x = center_x + (i * spacing)
                y = center_y + (j * spacing)
                agent_class(images=loaded_images, simulation=self, position=Vector2(x, y))

        return self

    def add_boundaries(self) -> list[Polygon]:
        """
        Add boundaries to the simulation environment.
        """
        # Left, Right, Top, Bottom boundaries
        boundaries = [(0, 0, 0.5, 500), (500, 0, 0.5, 500), (0, 0, 500, 0.5), (0, 500, 500, 0.5)]
        boundary_polygons = []

        for rect_params in boundaries:
            # Draw Rect on the background
            rect = pg.Rect(rect_params)
            pg.draw.rect(self._background, (255, 0, 0), rect)

            # Create Polygon object from Rect coordinates, should facilitate distance calculation
            boundary_polygon = Polygon([
                (rect.left, rect.top),
                (rect.right, rect.top),
                (rect.right, rect.bottom),
                (rect.left, rect.bottom)
            ])
            boundary_polygons.append(boundary_polygon)

        # Update the screen to display the boundaries
        pg.display.flip()

        return boundary_polygons
