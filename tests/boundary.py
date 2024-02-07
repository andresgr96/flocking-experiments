import pygame as pg
from pygame.math import Vector2


class Boundary:
    def __init__(self, rect):
        self.rect = pg.Rect(rect)

    def check_collision(self, pos, radius):
        return self.rect.colliderect(pg.Rect(pos.x - radius, pos.y - radius, 2 * radius, 2 * radius))

# Initialize Pygame
pg.init()

# Set up the display
width, height = 500, 500
screen = pg.display.set_mode((width, height))

# Create a boundary
boundary_rect = (0, 0, 1, 500)
boundary = Boundary(boundary_rect)

agent_pos = Vector2(50, 300)  # Replace with your agent's position
agent_radius = 10  # Replace with your agent's radius


# Game loop
clock = pg.time.Clock()
running = True

while running:
    for event in pg.event.get():
        if event.type == pg.QUIT:
            running = False

    # Update logic here
    agent_pos -= Vector2(1, 0)
    # Check collisions with the boundary

    if boundary.check_collision(agent_pos, agent_radius):
        # Handle collision behavior (e.g., stop the agent or change its direction)
        print("Collision with boundary!")

    # Draw the boundary
    pg.draw.rect(screen, (255, 255, 255), boundary.rect)

    # Draw the agent (replace with your agent drawing code)
    pg.draw.circle(screen, (255, 0, 0), (int(agent_pos.x), int(agent_pos.y)), agent_radius)

    pg.display.flip()
    clock.tick(60)

pg.quit()
