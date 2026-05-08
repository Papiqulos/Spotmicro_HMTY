import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from adafruit_servokit import ServoKit
from tools.utils import rescale_number


kit1 = ServoKit(channels=16)
kit2 = ServoKit(channels=16, address=0x41)

# Left legs
# fl
kit1.servo[12].angle = 76 # THIS NEEDS DEGREES
kit1.servo[13].angle = 83 # THIS NEEDS DEGREES
kit1.servo[14].angle = 137 # THIS NEEDS DEGREES

# rl
kit2.servo[14].angle = 120 # THIS NEEDS DEGREES
kit2.servo[15].angle = 76 # THIS NEEDS DEGREES
kit2.servo[13].angle = 126 # THIS NEEDS DEGREES


# Right legs
# fr
kit1.servo[9].angle =  70 # THIS NEEDS DEGREES
kit1.servo[10].angle = 126 # THIS NEEDS DEGREES
kit1.servo[11].angle = 64 # THIS NEEDS DEGREES

# rr
kit2.servo[6].angle =  50 # THIS NEEDS DEGREES
kit2.servo[8].angle =  46 # THIS NEEDS DEGREES
kit2.servo[12].angle = 50 # THIS NEEDS DEGREES

# print("hello")
