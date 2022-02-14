import numpy as np
import math
import matplotlib.pyplot as plt
import pygame
from numpy import sin, cos

from intermediate.simulator import DISPLAY, WINDOW_SCALE


class Display:

    def __init__(self, dt, arm_lengths, start_pos):
        # initialise real-time plot with pygame
        self.arm_lengths = arm_lengths

        self.xc, self.yc = DISPLAY.get_rect().center  # window center
        pygame.display.set_caption('robot arm')

        self.font = pygame.font.Font('freesansbold.ttf', 12)  # printing text font and font size
        self.text = self.font.render('robot arm', True, (0, 0, 0), (255, 255, 255))  # printing text object
        self.textRect = self.text.get_rect()
        self.textRect.topleft = (10, 10)  # printing text position with respect to the top-left corner of the window

        self.clock = pygame.time.Clock()  # initialise clock

        # SIMULATION PARAMETERS
        dts = dt * 1  # desired simulation step time (NOTE: it may not be achieved)
        self.T = 3  # total simulation time

        self.FPS = int(1 / dts)  # refresh rate

        # scaling
        self.window_scale = WINDOW_SCALE  # conversion from meters to pixels

        self.start = start_pos

    def render(self, q, goal):
        # real-time plotting
        DISPLAY.fill((255, 255, 255))  # clear window

        # l1, l2, l3 = self.arm_lengths
        l1, = self.arm_lengths
        # update individual link position
        x0, y0 = self.start
        xbase, ybase = [0, 0]
        x1 = x0 + l1 * np.cos(q[0])
        y1 = y0 + l1 * np.sin(q[0])
        # x2 = x1 + l2 * np.cos(q[0] + q[1])
        # y2 = y1 + l2 * np.sin(q[0] + q[1])

        # x3 = x2 + l3 * np.cos(q[0] + q[1] + q[2])
        # y3 = y2 + l3 * np.sin(q[0] + q[1] + q[2])
        # x4 = x3 + l4 * np.cos(q[0] + q[1] + q[2] + q[3])
        # y4 = y3 + l4 * np.sin(q[0] + q[1] + q[2] + q[3])
        window_scale = self.window_scale
        xc, yc = self.xc, self.yc

        xy_list = list(zip([xbase, x0, x1], [ybase, y0, y1]))
        # xy_list = list(zip([xbase, x0, x1, x2], [ybase, y0, y1, y2]))

        xy_list_lines = xy_list.copy()
        points = np.array(xy_list_lines)

        points *= window_scale
        points[:, 1] *= -1
        points[:] += np.array([xc, yc])
        pygame.draw.lines(DISPLAY, (0, 0, 255), False, points, 3)
        xc, yc = DISPLAY.get_rect().center

        def draw_points(xy, color=(0, 0, 0)):
            for x, y in xy:
                pygame.draw.circle(DISPLAY, color,
                                   (int(window_scale * x) + xc, int(-window_scale * y) + yc),
                                   5)

        draw_points(xy_list[0:-1])
        draw_points(xy_list[-1:], color=(255, 0, 0))

        pygame.draw.circle(DISPLAY, (0, 255, 0),
                           (int(window_scale * goal[0]) + xc, int(-window_scale * goal[1]) + yc),
                           3)  # draw reference position

        text = self.font.render("FPS = " + str(round(self.clock.get_fps())), True, (0, 0, 0), (255, 255, 255))
        DISPLAY.blit(text, self.textRect)

    def tick(self):
        self.clock.tick(self.FPS)
