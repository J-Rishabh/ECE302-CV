import pygame
from pygame.locals import *

# Initialize Pygame
pygame.init()

# Set up the display
WIDTH, HEIGHT = 640, 480
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Controller Test Game")

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)

# Game variables
x, y = WIDTH // 2, HEIGHT // 2
speed = 5
radius = 10

# Initialize the joystick
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    print("No joystick detected!")
    pygame.quit()
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Joystick Name: {joystick.get_name()}")

# Main game loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == QUIT:
            running = False
        elif event.type == JOYBUTTONDOWN:
            print(f"Button {event.button} pressed!")
        elif event.type == JOYBUTTONUP:
            print(f"Button {event.button} released!")

    # Get joystick input
    axis_x = joystick.get_axis(0)  # Left stick horizontal
    axis_y = joystick.get_axis(1)  # Left stick vertical

    # Update position
    x += int(axis_x * speed)
    y += int(axis_y * speed)

    # Boundaries check
    x = max(radius, min(WIDTH - radius, x))
    y = max(radius, min(HEIGHT - radius, y))

    # Clear the screen
    screen.fill(BLACK)

    # Draw the "character"
    pygame.draw.circle(screen, RED, (x, y), radius)

    # Update the display
    pygame.display.flip()

# Clean up
pygame.quit()
