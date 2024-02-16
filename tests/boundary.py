import pygame as pg
import math
from pygame.math import Vector2


class Boundary:
    def __init__(self, rect):
        self.rect = pg.Rect(rect)

    def check_collision(self, pos, radius):
        return self.rect.colliderect(pg.Rect(pos.x - radius, pos.y - radius, 2 * radius, 2 * radius))

# Initialize Pygame
pg.init()

# Set up the display
width, height = 200, 200
screen = pg.display.set_mode((width, height))

# Create a boundary
boundary_rect = (0, 0, 1, 200)
boundary = Boundary(boundary_rect)

agent_pos = Vector2(100, 100)
# agent_pos.rotate_ip(180)

agent2_pos = Vector2(150, 100)
agent_radius = 5  # Replace with your agent's radius


# Game loop
clock = pg.time.Clock()
running = True

while running:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            running = False


    if boundary.check_collision(agent_pos, agent_radius):
        # Handle collision behavior (e.g., stop the agent or change its direction)
        print("Collision with boundary!")

    # Draw the boundary
    pg.draw.rect(screen, (255, 255, 255), boundary.rect)

    # Draw the agent (replace with your agent drawing code)
    pg.draw.circle(screen, (255, 0, 0), (int(agent_pos.x), int(agent_pos.y)), agent_radius)

    pg.draw.circle(screen, (255, 0, 0), (int(agent2_pos.x), int(agent2_pos.y)), agent_radius)

    print(f"Pygame Angle: {agent_pos.angle_to(agent2_pos)}")
    print(f"Atahn Angle: {math.degrees(math.atan2(-(agent2_pos.y - agent_pos.y), (agent2_pos.x - agent_pos.x)))}")

    linear_velocity = Vector2(1, -1)
    print(f"Original Velocity: {linear_velocity}")
    linear_velocity.rotate_ip(180)
    print(f"Rotated Velocity: {linear_velocity}")



    pg.display.flip()
    clock.tick(60)

pg.quit()
