import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from adafruit_servokit import ServoKit
from tools.utils import rescale_number


kit1 = ServoKit(channels=16)
kit2 = ServoKit(channels=16, address=0x41)

# Left legs
# fl
kit1.servo[0].angle = 95 # THIS NEEDS DEGREES
kit1.servo[1].angle = 89 # THIS NEEDS DEGREES
kit1.servo[2].angle = 143 # THIS NEEDS DEGREES


# rl
kit2.servo[15].angle = 75 # THIS NEEDS DEGREES
kit2.servo[13].angle = 72 # THIS NEEDS DEGREES
kit2.servo[12].angle = 120 # THIS NEEDS DEGREES


# Right legs
# fr
kit1.servo[4].angle =  81 # THIS NEEDS DEGREES
kit1.servo[5].angle = 132 # THIS NEEDS DEGREES
kit1.servo[6].angle = 62 # THIS NEEDS DEGREES

# rr
kit2.servo[0].angle = 90 # THIS NEEDS DEGREES
kit2.servo[1].angle = 94 # THIS NEEDS DEGREES
kit2.servo[2].angle = 50 # THIS NEEDS DEGREES

# print("hello")
