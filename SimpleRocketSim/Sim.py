import pygame
import numpy as np
import time
from Rocket import Rocket

DEBUG = True

    ### init ###

pygame.init()

screen_width = 1080
screen_height = 1840
screen = pygame.display.set_mode((screen_width, screen_height)) # makes the window. window dimentions = (width, height)
clock = pygame.time.Clock()
font = pygame.font.Font(None, 36)
font_small = pygame.font.Font(None, 24)
imported_rocket_image = pygame.image.load('rocket_img.png')
rocket_image = pygame.transform.scale(imported_rocket_image, (10, 50)) # Scales the picture of the rocket in the simulator. units = cm

rocket = Rocket()


    ### LOOP ###

running = True
last_time = time.time()

while running:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_RIGHT: # -1 degree when pressing the right arrowkey
                rocket.theta -= 1
            elif event.key == pygame.K_LEFT: # +1 degree when pressing the left arrowkey
                rocket.theta += 1
            elif event.key == pygame.K_SPACE: # toggles the launched state on and off when pressing the spacebar
                rocket.launched = not rocket.launched
            elif event.key == pygame.K_x: # shuts down the program when pressing x
                running = False


    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time

    rocket.dynamics_step(dt)

    ### Aestetics ###
    screen.fill((28, 37, 60)) # makes the background
    pygame.draw.rect(screen, (17, 124, 19), [0, (screen_height - 40) , screen_width, 250]) # makes the grass at the bottom
    

    # The coordinate system has origo in the top left corner and the y-axis increases downwards. units = cm
    rocket_alt = (screen_height - 65) - rocket.positiony * 10
    rocket_pos = (screen_width/2) - rocket.positionx * 10

    rocket_center = (rocket_pos, rocket_alt)
    rotated_image = pygame.transform.rotate(rocket_image, rocket.theta)
    rotated_rect = rotated_image.get_rect(center=rocket_center)
    screen.blit(rotated_image, rotated_rect)

    ### info text in the simulator ###

    alt_line100 = (screen_height - 65) - 1000
    pygame.draw.line(screen, (255, 255, 1), (0, alt_line100), (screen_width, alt_line100), 1)

    alt_line10 = (screen_height - 65) - 100
    pygame.draw.line(screen, (255, 255, 1), (0, alt_line10), (screen_width, alt_line10), 1)

    text_theta = font.render(f'theta: {round(rocket.theta, 2)} deg', True, (10, 10, 10))
    text_launched = font.render(f'launched: {rocket.launched}', True, (10, 10, 10))
    text_altitude = font.render(f'altitude: {round(rocket.positiony)}m', True, (10, 10, 10))
    screen.blit(text_theta, (30, (screen_height - 30)))
    screen.blit(text_altitude, (200, (screen_height - 30)))
    screen.blit(text_launched, (400, (screen_height - 30)))

    text_info_1 = font_small.render(f'Use RIGHT / LEFT arrows to increase / decrease theta', True, (255, 255, 1))
    text_info_2 = font_small.render(f'Press SPACE to launch rocket', True, (255, 255, 1))
    text_info_3 = font_small.render(f'Press X to quit', True, (255, 255, 1))
    text_info_4 = font_small.render(f'100m', True, (255, 255, 1))
    text_info_5 = font_small.render(f'10m', True, (255, 255, 1))
    screen.blit(text_info_1, (10, 10))
    screen.blit(text_info_2, (10, 30))
    screen.blit(text_info_3, (10, 50))
    screen.blit(text_info_4, (10, alt_line100 + 10))
    screen.blit(text_info_5, (10, alt_line10 + 10))

    pygame.display.flip()
    clock.tick(60)

pygame.quit()