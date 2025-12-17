from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)




# 1 - Left front ankle 125 -1
# 2 - Left front knee 95 -1
# 3 - Left front shoulder 100 -1

# 13 - Left back ankle 130 -1
# 14 - Left back knee 95 -1
# 15 - Left back shoulder 95 -1

# 4 - Right front ankle 70 1
# 5 - Right front knee 92 1
# 6 - Right front shoulder 85 -1

#  8 - Right back ankle 70 1
#  9 - Right back knee 92 1
# 10 - Right back shoulder 70 -1

kit.servo[8].angle = 75
kit.servo[9].angle = 85
kit.servo[10].angle = 75

kit.servo[4].angle = 75
kit.servo[5].angle = 95
kit.servo[6].angle = 105

