# Owen Rouse - 11/18/2023
# Robo_Run

import numpy as np
from Robo_Control import *

# Here are the variables needed to set up the arm:
# Servo 1 is the base, servo 2 is the next one, servo 3 ...
# [Servo Num, Min Degrees, Max Degrees]
servos = [[1, 0, 180],
           [2, 0, 180],
           [3, 0, 180],
           [4, 0, 180],
           [5, 0, 180]]

# Automatic Setup of Robot
# After these steps, joints should no longer be able to move freely

start()
servo = []
for i in range(1,servos[-1][0]):
    servo[i] = Servo()

##### Write Code to Control Arm Below #####

print(servo[1].readAngle())
servo[1].writeAngle(90)
print(servo[1].readAngle())

##### Write Code to Control Arm Above #####
# DO NOT TOUCH
end(servos[-1][0])