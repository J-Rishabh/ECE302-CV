import pygame

# Initialize Pygame
pygame.init()

# Initialize the joystick
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    print("No joystick detected!")
    pygame.quit()
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Joystick Name: {joystick.get_name()}")
print(f"Number of Buttons: {joystick.get_numbuttons()}")

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.JOYBUTTONDOWN:
            print(f"Button {event.button} pressed!")
        elif event.type == pygame.JOYBUTTONUP:
            print(f"Button {event.button} released!")

pygame.quit()
