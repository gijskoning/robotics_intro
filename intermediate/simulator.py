import math

import numpy as np
from math import cos, sin

import pygame

# Simple way to create a pygame simulator.
# Bit ugly to not use a class however works easy
pygame.init()  # start pygame
DISPLAY = pygame.display.set_mode((800, 600))  # create a window (size in pixels)
DISPLAY.fill((255, 255, 255))  # white background
WINDOW_SCALE = 400


def draw_rectangle_from_config(configs, line_width=2, color=(255, 0, 0)):
    """
    
    :param configs: 
    :param line_width: 
    :param color: 
    :return: 
    """
    config_scaled = []
    for c in configs:
        c = c * WINDOW_SCALE
        c[1] *= -1
        c += DISPLAY.get_rect().center
        config_scaled.append(c)
    pygame.draw.polygon(DISPLAY, color, config_scaled, width=line_width)


def length(p1, p2=None):
    if p2 is None:
        p2 = [0, 0]
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def state_rotate_pygame(x, y, yaw):
    # Pygame coordinate frame is different
    # Yaw in radians required
    return np.array([x * cos(yaw) + y * sin(yaw), -x * sin(yaw) + y * cos(yaw)])


def state_rotate(x, y, yaw):
    # Yaw in radians required
    return np.array([x * cos(yaw) + y * sin(yaw), x * sin(yaw) + y * cos(yaw)])


def config_to_polygon(x, y, yaw, length, width):
    start = np.array([x, y])
    polygon = [
        start + state_rotate(x=0.0, y=width / 2, yaw=yaw),
        start + state_rotate(x=0.0, y=-width / 2, yaw=yaw),
        start + state_rotate(x=length, y=-width / 2, yaw=yaw),
        start + state_rotate(x=length, y=width / 2, yaw=yaw)]
    return polygon


def arm_to_polygon(x, y, yaw, length, width):
    additional_length_arm = 0.05  # In meters

    x, y = np.array([x, y]) + state_rotate_pygame(x=-additional_length_arm / 2, y=0, yaw=-yaw)
    return config_to_polygon_pygame(x, y, yaw, length + additional_length_arm, width)


def config_to_polygon_pygame(x, y, yaw, length, width):
    start = np.array([x, y])
    yaw *= -1
    polygon = [
        start + state_rotate_pygame(x=0.0, y=width / 2, yaw=yaw),
        start + state_rotate_pygame(x=0.0, y=-width / 2, yaw=yaw),
        start + state_rotate_pygame(x=length, y=-width / 2, yaw=yaw),
        start + state_rotate_pygame(x=length, y=width / 2, yaw=yaw)]
    return polygon
