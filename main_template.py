import numpy as np
import pygame

from advanced.kinematics import RobotArm3dof
from advanced.pid import PIDController
from intermediate import simulator
from intermediate.communication import ArduinoCommunication
from intermediate.constants import ARMS_LENGTHS
from intermediate.utils import keyboard_arrows
from intermediate.visualize_robot_arm import Display

if __name__ == '__main__':
    arduino = False
    arduino_control = None
    arduino_port = 'COM4'  # Ubuntu desktop bottom right
    # You need to find out which port is used. This is not always COM4.
    # In powershell for windows use this command to find it 'Get-PnpDevice -PresentOnly | Where-Object { $_.InstanceId -match '^USB' }'
    # In ubuntu this is in the format: '/dev/ttyUSB0'
    if arduino:
        try:
            arduino_control = ArduinoCommunication(port=arduino_port)
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
        # use keyboard_arrows()
        # might change the goal
        # Do some control steps

        # Move the robot arm and sent it to the arduino
        t += dt

        # Render in simulator
        for pol in robot_arm.arm_regions:
            pol = [xy for xy in pol]
            simulator.draw_rectangle_from_config(pol)

        display.tick()
        pygame.display.flip()  # update display