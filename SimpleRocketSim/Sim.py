import pygame
import numpy as np
import time
from Rocket import Rocket
import matplotlib.pyplot as plt

DEBUG = True

    ### init ###

pygame.init()

screen_height = 990
screen_width = screen_height * 1080/1840

screen = pygame.display.set_mode((screen_width, screen_height)) # makes the window. window dimentions = (width, height)
clock = pygame.time.Clock()
font = pygame.font.Font(None, 36)
font_small = pygame.font.Font(None, 24)
imported_rocket_image = pygame.image.load('rocket_img.png')
imported_moon_image = pygame.image.load('moon_img.png')
imported_hills_image = pygame.image.load('hills_img.png')
rocket_image = pygame.transform.scale(imported_rocket_image, (15, 40)) # Scales the picture of the rocket in the simulator. 
moon_image = pygame.transform.scale(imported_moon_image, (150, 150))
hills_image = pygame.transform.scale(imported_hills_image, (screen_width, (135/256 * screen_width)))
rocket = Rocket()

timer = 0
running = True
last_time = time.time()


    ############
    ### LOOP ###
    ############


while running:

    for event in pygame.event.get(): # configures the input keys for interacting with the sim
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_RIGHT: # -1 degree when pressing the right arrowkey
                rocket.theta -= 1
            elif event.key == pygame.K_LEFT: # +1 degree when pressing the left arrowkey
                rocket.theta += 1
            elif event.key == pygame.K_SPACE: # toggles the launched state on and off when pressing the spacebar
                rocket.launched = not rocket.launched
                if rocket.launched:
                    timer = 0
            elif event.key == pygame.K_x: # shuts down the program when pressing x
                running = False

    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time
    if rocket.launched:
        timer += dt

    rocket.dynamics_step(dt) # runs the step function in Rocket.py
    rocket.timer(dt)

    ### Aestetics ###
    screen.fill((31, 62, 90)) # makes the background
    hills_height = screen_height - 10
    hills_center = (screen_width/2, hills_height)
    hills_rect = hills_image.get_rect(center=hills_center)
    screen.blit(hills_image, hills_rect)
    moon_height = 50
    moon_pos = screen_width - 20
    moon_center = (moon_pos, moon_height)
    moon_rect = moon_image.get_rect(center=moon_center)
    screen.blit(moon_image, moon_rect)
    pygame.draw.rect(screen, (200, 200, 100), [0, (screen_height - 40) , screen_width, 250])

    # The coordinate system has origo in the top left corner and the z-axis increases downwards. units = cm
    rocket_alt = (screen_height - 65) - rocket.positionZ * 10 * 3/4
    rocket_pos = (screen_width/2) - rocket.positionX * 10 * 3/4

    ### configures the rocket in the sim ###
    rocket_center = (rocket_pos, rocket_alt)
    rotated_image = pygame.transform.rotate(rocket_image, rocket.theta)
    rotated_rect = rotated_image.get_rect(center=rocket_center)
    screen.blit(rotated_image, rotated_rect)

    ### info/text in the sim ###
    alt_line100 = (screen_height - 65) - 1000 * 3/4
    pygame.draw.line(screen, (255, 255, 1), (0, alt_line100), (screen_width, alt_line100), 1)
    alt_line10 = (screen_height - 65) - 100 * 3/4
    pygame.draw.line(screen, (255, 255, 1), (0, alt_line10), (screen_width, alt_line10), 1)

    text_theta      = font.render(f'Theta: {round(rocket.theta, 2)} deg', True, (10, 10, 10))
    text_launched   = font.render(f'Thrust: {rocket.launched}', True, (10, 10, 10))
    text_altitude   = font.render(f'Altitude: {round(rocket.positionZ)}m', True, (10, 10, 10))
    screen.blit(text_theta, (30, (screen_height - 30)))
    screen.blit(text_altitude, (200, (screen_height - 30)))
    screen.blit(text_launched, (380, (screen_height - 30)))

    text_info_1 = font_small.render(f'Use RIGHT / LEFT arrows to increase / decrease theta', True, (255, 255, 1))
    text_info_2 = font_small.render(f'Press SPACE to launch rocket', True, (255, 255, 1))
    text_info_3 = font_small.render(f'Press X to quit', True, (255, 255, 1))
    text_info_4 = font_small.render(f'Time: {round(timer, 2)}s', True, (255, 255, 1))
    text_info_5 = font_small.render(f'100m', True, (255, 255, 1))
    text_info_6 = font_small.render(f'10m', True, (255, 255, 1))
    screen.blit(text_info_1, (10, 10))
    screen.blit(text_info_2, (10, 30))
    screen.blit(text_info_3, (10, 50))
    screen.blit(text_info_4, (10, 80))
    screen.blit(text_info_5, (10, (alt_line100 + 10)))
    screen.blit(text_info_6, (10, (alt_line10 + 10)))

    pygame.display.flip()
    clock.tick(60)

pygame.quit()