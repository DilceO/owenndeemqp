# Owen Rouse - 11/18/2023
# Robo_Run

import numpy as np
from Robo_Control import *

# Here are the variables needed to set up the arm:
# Servo 1 is the base, servo 2 is the next one, servo 3 ...
# [Servo Num, Min Degrees, Max Degrees]
servoList = [[0, 0, 180], # Servo 0 (Base)
           [1, 0, 180], # Servo 1 
           [2, 0, 180], # ...
           [3, 0, 180],
           [4, 0, 180]]

# Automatic Setup of Robot
# After these steps, joints should no longer be able to move freely

start()
servo = []
for i in range(servoList[-1][0]+1):
    print("index: " + str(i))
    servo.append(Servo(servoList[i][0],servoList[i][1],servoList[i][2],))
    print("Servo " + str(i) + " Success")


##### Write Code to Control Arm Below #####
while(1):
    print(servo[0].readAngle())
    print(servo[1].readAngle())
    print(servo[2].readAngle())
    print(servo[3].readAngle())
    print(servo[4].readAngle())
# servo[1].writeAngle(90)
# print(servo[1].readAngle())

##### Write Code to Control Arm Above #####
# DO NOT TOUCH
end(servoList[-1][0])