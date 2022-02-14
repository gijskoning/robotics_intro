import time

import numpy as np
import pygame
from pygame import K_LEFT, K_UP, K_RIGHT, K_DOWN

step_size = 0.1


def keyboard_arrows():
    # Using the arrow keys to create a vector
    q = np.array([0., 0.])

    pygame.event.get()  # refresh keys
    keys = pygame.key.get_pressed()
    if keys[K_LEFT]:
        q[0] -= 1
    if keys[K_RIGHT]:
        q[0] += 1

    if keys[K_UP]:
        q[1] += 1
    if keys[K_DOWN]:
        q[1] -= 1
    q *= step_size
    return q


if __name__ == '__main__':
    q = keyboard_arrows()
    print(f"vector:", q)
    time.sleep(0.01)
