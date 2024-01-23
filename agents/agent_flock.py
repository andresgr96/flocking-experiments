from vi import Agent
import pygame as pg


# Inherit from Storms agent class
class FlockingAgent(Agent):
    def __init__(self, simulation, images, radius=5, speed=2, alignment_factor=1.0,  # Might try the matrix config trick
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



