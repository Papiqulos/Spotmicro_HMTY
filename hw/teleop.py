from pydualsense import *
import numpy as np
import time
import os





class DualSenseController:

    def __init__(self):
        self.dualsense = pydualsense()
        self.dualsense.init()
    

    def _get_joystick_angle(self):
        return np.arctan2(self.dualsense.state.LY, self.dualsense.state.LX) + np.pi/2
    
        # Test method
    def move(self, callback):
        start_time = time.time()
        prev_state = self.dualsense.state

        print()
        print("\033[A", end="", flush=True)

        while not self.dualsense.state.R1:
            dpad_up = self.dualsense.state.DpadUp
            dpad_down = self.dualsense.state.DpadDown
            dpad_left = self.dualsense.state.DpadLeft
            dpad_right = self.dualsense.state.DpadRight

            self.ljX = self.dualsense.state.LX
            self.ljY = self.dualsense.state.LY
            self.ljAngle = np.arctan2(self.ljY, self.ljX) + np.pi/2
            self.ljAngle_deg = np.degrees(self.ljAngle)
            

            print(f"\033[2KLeft Joystick Angle: {self.ljAngle_deg:.2f}°", flush=True)

            if dpad_up:
                callback(0)
                prev_state = "up"
            elif dpad_down:
                callback(1)
                prev_state = "down"
            elif dpad_right:
                callback(2)
                prev_state = "right"
            elif dpad_left:
                callback(3)
                prev_state = "left"
            else:
                if prev_state in ("up", "down", "right", "left"):
                    callback(4)
                    prev_state = None
                print("\033[2KWaiting for command...", end="\r", flush=True)
                start_time = time.time()

            print("\033[A", end="", flush=True)


if __name__ == "__main__":
    controller = DualSenseController()

    def execute(i):
        if i == 0:
            print("\033[2Kforward", end="\r", flush=True)
        elif i == 1:
            print("\033[2Kbackward", end="\r", flush=True)
        elif i == 2:
            print("\033[2Kright", end="\r", flush=True)
        elif i == 3:
            print("\033[2Kleft", end="\r", flush=True)
        elif i == 4:
            for t in range(20, 0, -1):
                print(f"\033[2KDecelerating... {t}", end="\r", flush=True)
                time.sleep(0.05)

    controller.move(execute)
    # close device
    controller.dualsense.close()

