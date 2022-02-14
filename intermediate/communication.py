import math
import time

import numpy as np
from serial import Serial


class ArduinoCommunication:

    def __init__(self, port='COM4'):
        print(f"trying port: {port}")
        # Baudrate should be the same as the rate on the arduino. Faster rate is higher data bandthwidth.
        #  However is less robust
        self.arduino = Serial(port=port, baudrate=115200, timeout=.1)

    def write_message(self, x, debug=False):
        if debug:
            print("Sending: ", x)
        _bytes = bytes(str(x), 'utf-8')
        self.arduino.write(_bytes)
        # Wait a bit to allow for messages from arduino to return
        time.sleep(0.05)
        data = self.arduino.readlines()
        if debug and data is not None:
            for line in data:
                print("Arduino msg:", line)
        return data

    def set_servo(self, q, debug=True):
        """
        :param q: numpy array for each joint angle
        :return:
        """
        resolution = 200  # Set some integer resolution to be used when sending
        print("q_global", q)
        q = q / math.pi * resolution
        q = np.clip(q, 0, resolution)

        q = q.astype(int)
        s1, = q

        self.write_message(f"0:{s1}", debug)
        # self.write_message(f"0:{s1},1:{s2},", debug)
