from pydualsense import *
import numpy as np
import time
import os

DEADZONE = 15


class DualSenseController:

    def __init__(self):
        self.dualsense = pydualsense()
        self.dualsense.init()
        print(f"Dualsense Battery: {self.dualsense.battery.State}%")

    def config_angle(self, angle, in_deg=False):
        if angle < 0 and angle > -np.pi:
            angle = angle + np.pi
        elif angle > 0 and angle < np.pi:
            angle = angle - np.pi
        return angle if not in_deg else np.degrees(angle)
    

    def _get_joystick_angle(self, in_deg=False):
        l_angle = np.arctan2(self.dualsense.state.LX, self.dualsense.state.LY)
        r_angle = np.arctan2(self.dualsense.state.RX, self.dualsense.state.RY)
        l_angle = self.config_angle(l_angle, in_deg)
        r_angle = self.config_angle(r_angle, in_deg)

        return  np.array([l_angle, r_angle])



    def _joystick_in_motion(self, joystick="l"):
        if joystick == "r":
            return self.dualsense.state.RX * self.dualsense.state.RX + self.dualsense.state.RY * self.dualsense.state.RY > DEADZONE * DEADZONE
        elif joystick == "l":
            return self.dualsense.state.LX * self.dualsense.state.LX + self.dualsense.state.LY * self.dualsense.state.LY > DEADZONE * DEADZONE
    
    def _get_rpy(self):
        return np.array([self.dualsense.state.gyro.Roll, self.dualsense.state.gyro.Pitch, self.dualsense.state.gyro.Yaw])


        # Test method
    def move(self, callback):

        prev_state = self.dualsense.state

        # print()
        # print("\033[A", end="", flush=True)

        while not self.dualsense.state.R1:
            dpad_up = self.dualsense.state.DpadUp
            dpad_down = self.dualsense.state.DpadDown
            dpad_left = self.dualsense.state.DpadLeft
            dpad_right = self.dualsense.state.DpadRight
            
            self.ljX = self.dualsense.state.LX
            self.ljY = self.dualsense.state.LY
            print(f"\033[2KMotion={self._joystick_in_motion()} angle={self._get_joystick_angle(in_deg=True)}", end="\r", flush=True)

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
                # print("\033[2KWaiting for command...", end="\r", flush=True)
              

            # print("\033[A", end="", flush=True)


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

