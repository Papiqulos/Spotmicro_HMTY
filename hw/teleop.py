from pydualsense import *
import time
import os





class DualSenseController:

    def __init__(self):
        self.dualsense = pydualsense()
        self.dualsense.init()

        self.cross_pressed = False
        self.circle_pressed = False
        self.dpad_down_pressed = False
        self.left_joystick_changed = False
        self.gyro_changed = False
        self.triangle_pressed = False
        
        # self.dualsense.cross_pressed += self.cross_down
        # self.dualsense.circle_pressed += self.circle_down
        # self.dualsense.dpad_down += self.dpad_down
        # self.dualsense.left_joystick_changed += self.joystick
        # self.dualsense.gyro_changed += self.gyro_changed
        
    def cross_down(self, state):
        self.cross_pressed = state

    def circle_down(self, state):
        self.circle_pressed = state

    def dpad_down(self, state):
        self.dpad_down_pressed = state

    def joystick(self, stateX, stateY):
        self.left_joystick_changed = True

    def gyro_changed(self, pitch, yaw, roll):
        self.gyro_changed = True

    def triangle_pressed(self):
        self.triangle_pressed = True
    
    def move(self, callback):
        i = 0
        time_step = 1.0 / 100
        start_time = time.time()
        prev_state = self.dualsense.state
        while True:
            dpad_up = self.dualsense.state.DpadUp
            dpad_down = self.dualsense.state.DpadDown
            dpad_left = self.dualsense.state.DpadLeft
            dpad_right = self.dualsense.state.DpadRight

            square_pressed = self.dualsense.state.square
            triangle_pressed = self.dualsense.state.triangle
            circle_pressed = self.dualsense.state.circle
            cross_pressed = self.dualsense.state.cross
            
            triangle_pressed = self.dualsense.state.triangle
            if dpad_up:
                t0 = time.time()
                current_time = t0 - start_time
                elapsed = time.time() - start_time
                callback(0)
                # time.sleep(time_step)
                prev_state = "up"
            elif dpad_down:
                t0 = time.time()
                current_time = t0 - start_time
                elapsed = time.time() - start_time
                callback(1)
                # time.sleep(time_step)
                prev_state = "down"
            elif dpad_right:
                t0 = time.time()
                current_time = t0 - start_time
                elapsed = time.time() - start_time
                callback(2)
                # time.sleep(time_step)
                prev_state = "right"
            elif dpad_left:
                t0 = time.time()
                current_time = t0 - start_time
                elapsed = time.time() - start_time
                callback(3)
                # time.sleep(time_step)
                prev_state = "left"
            else:
                if prev_state in ("up", "down", "right", "left"):
                    callback(4)
                    prev_state = None
                print("\033[2KWaiting for command...", end="\r", flush=True)
                start_time = time.time()
                # time.sleep(time_step)


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
    while not controller.dualsense.state.R1:
        ...

    # close device
    controller.dualsense.close()

