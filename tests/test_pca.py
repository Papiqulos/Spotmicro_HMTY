import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from adafruit_servokit import ServoKit
from tools.utils import rescale_number
import yaml   


kit1 = ServoKit(channels=16)
kit2 = ServoKit(channels=16, address=0x41)

with open("config/servo_calib.yaml") as f:
    calib = yaml.safe_load(f)

LEGS   = ["FL", "FR", "RL", "RR"]
JOINTS = ["shoulder", "leg", "foot"]
zeros = [calib[leg][joint]["zero_deg"]  for leg in LEGS for joint in JOINTS]

# Left legs
# fl
kit1.servo[0].angle = zeros[0] # THIS NEEDS DEGREES
kit1.servo[1].angle = zeros[1] # THIS NEEDS DEGREES
kit1.servo[2].angle = zeros[2] # THIS NEEDS DEGREES


# rl
kit2.servo[15].angle = zeros[6] # THIS NEEDS DEGREES
kit2.servo[13].angle = zeros[7] # THIS NEEDS DEGREES
kit2.servo[12].angle = zeros[8] # THIS NEEDS DEGREES


# Right legs
# fr
kit1.servo[4].angle = zeros[3] # THIS NEEDS DEGREES
kit1.servo[5].angle = zeros[4] # THIS NEEDS DEGREES
kit1.servo[6].angle = zeros[5] # THIS NEEDS DEGREES

# rr
kit2.servo[0].angle = zeros[9] # THIS NEEDS DEGREES
kit2.servo[1].angle = zeros[10] # THIS NEEDS DEGREES
kit2.servo[2].angle = zeros[11] # THIS NEEDS DEGREES

print("hello")
