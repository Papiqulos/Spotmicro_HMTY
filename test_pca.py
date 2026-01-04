from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)




# 1 - Left front foot 125 -1
# 2 - Left front leg 95 -1
# 3 - Left front shoulder 100 -1

# 13 - Left rear foot 130 -1
# 14 - Left rear leg 95 -1
# 15 - Left rear shoulder 95 -1

# 4 - Right front foot 70 1
# 5 - Right front leg 92 1
# 6 - Right front shoulder 85 -1

#  8 - Right rear foot 70 1
#  9 - Right rear leg 92 1
# 10 - Right rear shoulder 70 -1

kit.servo[8].angle = 75
kit.servo[9].angle = 85
kit.servo[10].angle = 75

kit.servo[4].angle = 75
kit.servo[5].angle = 95
kit.servo[6].angle = 105
