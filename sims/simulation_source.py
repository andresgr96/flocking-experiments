from __future__ import annotations
from vi import Agent, Config, Simulation
from typing import TYPE_CHECKING, Optional, Type, TypeVar
import cv2
from shapely.geometry import Polygon
import pygame as pg
from pygame.gfxdraw import hline, vline
from pygame.math import Vector2


class SourceSim(Simulation):

    """
        This class inherits from Simulation, but has the extra parameter to have an image as custom background.

        Eventually it'll build a coordinate system from the image array values to use as environment.
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

        size = (img_width, img_height)
        self._screen = pg.display.set_mode(size)

        # Initialise background
        # self._background = pg.surface.Surface(size).convert()
        self._background = pg.surfarray.make_surface(image)
        # self._background.fill((255, 255, 255))

        # Show background immediately (before spawning agents)
        self._screen.blit(self._background, (0, 0))
        pg.display.flip()

        # Add the boundaries to the environment
        self._boundaries = self.add_boundaries()

        # Initialise the clock. Used to cap FPS.
        self._clock = pg.time.Clock()

    def add_boundaries(self) -> list[Polygon]:
        """
        Add boundaries to the simulation environment.

        """
        #                  Left                Right              Top               Bottom
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


