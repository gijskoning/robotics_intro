import numpy as np
import pygame
from gym_robotic_arm.arduino_communication import ArduinoControl

from advanced.kinematics import RobotArm3dof
from advanced.pid import PIDController
from intermediate import simulator
from intermediate.constants import ARMS_LENGTHS
from intermediate.utils import keyboard_arrows
from intermediate.visualize_robot_arm import Display

if __name__ == '__main__':
    arduino = False
    arduino_control = None
    arduino_port = 'COM4'  # Ubuntu desktop bottom right
    if arduino:
        try:
            arduino_control = ArduinoControl(port=arduino_port)
        except IOError as e:
            print(e)
    # Add your python code here
    q = np.array([0.])  # q: angles of each joint. Could be just one joint
    dt = 0.01

    initial_q_values = np.array([0.])
    display = Display(dt, ARMS_LENGTHS, start_pos=[0, 0])
    robot_arm = RobotArm3dof(l=ARMS_LENGTHS, reset_q=initial_q_values, arduino_control=arduino_control)
    local_endp_start = robot_arm.end_p  # Get endposition of robot arm
    controller = PIDController(kp=15, ki=0.0, kd=0.0)

    goal = local_endp_start
    t = 0  # Current time
    while True:
        display.render(q, goal)
        keyboard_vector = keyboard_arrows() * dt
        goal += keyboard_vector

        F_end = controller.control_step(robot_arm.FK_end_p(), goal, dt)
        p, q, dq = robot_arm.move_endpoint(F_end)
        t += dt

        # Render in simulator
        for pol in robot_arm.arm_regions:
            pol = [xy for xy in pol]
            simulator.draw_rectangle_from_config(pol)

        display.tick()
        pygame.display.flip()  # update display