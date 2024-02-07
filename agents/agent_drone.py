from vi import Agent
import pygame as pg
import random
from pygame.math import Vector2
from utils import collective_motion
from utils.collective_motion import compute_target_velocities


class DroneAgent(Agent):
    def __init__(self, simulation, images, linear_velocity=Vector2(1, 0), angular_velocity=0.0, heading=0.0,
                 b_sense_range=20.0):
        super().__init__(images, simulation)

        self.simulation = simulation
        self.b_sense_range = b_sense_range
        # Add parameters specific to drone movement
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        self.target_linear_velocity = linear_velocity
        self.target_angular_velocity = angular_velocity
        self.heading = heading if heading is not None else random.uniform(0, 360)

    def update(self):

        # Example of using self.linear_velocity, self.angular_velocity, and self.heading for moving
        # print(collective_motion.proximal_control_force(self))

        # Compute target velocities
        target_vel, self.angular_velocity = \
            compute_target_velocities(self, self.linear_velocity, self.angular_velocity)

        self.linear_velocity = target_vel
        # Rotate heading based on angular velocity
        self.heading = self.angular_velocity
        # print(collective_motion.alignment_control_force(self))

        # Move in the direction of heading with at linear velocity
        self.linear_velocity = self.linear_velocity.rotate(self.heading)
        # print(self.linear_velocity)

        # print(self.__simulation._boundaries)
        print(collective_motion.detect_boundaries(self))
        # print(collective_motion.distance_to_boundary(self, self.__simulation._boundaries[1]))

    def there_is_no_escape(self) -> bool:
        """
        Override original function to avoid warping. Now it only checks if the agent has collided with a boundary.

        """
        changed = False

        if self.pos.x < self._area.left:
            changed = True

        if self.pos.x > self._area.right:
            changed = True

        if self.pos.y < self._area.top:
            changed = True

        if self.pos.y > self._area.bottom:
            changed = True

        return changed

    def change_position(self):
        """
            Override original function, now it stops the agents movement if drone reached a boundary
        """
        if not self._moving:
            return

        # If the agent is at one of the boundaries, stop moving to emulate a crash.
        changed = self.there_is_no_escape()
        if changed:
            self.linear_velocity = Vector2(0, 0)

        # Actually update the position at last.
        self.pos += self.linear_velocity

