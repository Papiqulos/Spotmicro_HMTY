import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from adafruit_servokit import ServoKit
from tools.utils import rescale_number


kit = ServoKit(channels=16)

kit.servo[5].angle = 63 # THIS NEEDS DEGREES

# print("hello")
