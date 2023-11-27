# Owen Rouse - 11/18/2023
# Robo_Run

import numpy as np
from Robo_Control import *
from Robo_Functions import *
from Robo_Gamepad import *

# Here are the variables needed to set up the arm:
# Servo 1 is the base, servo 2 is the next one, servo 3 ...
# [Servo Num, Min Degrees, Max Degrees]
servoList = [[0, 90, 270], # Servo 0 (Base)
           [1, 90, 270], # Servo 1 
           [2, 90, 260], # ...
           [3, 90, 270],
           [4, 90, 230]] # CHECK THIS

# Home coordinates
r = 100     # mm
phi = 135   # Degrees
theta = 90  # Degrees
grip = 0    # 0 is open,140 is closed
### These will all be normailzed to 0. This means That MOST will be from 0 - 180 ###

# Sensetivity Values for contoller. 1 is natural. 2 is higher sensitivity

thetaSense = 1
phiSense = 1
rSense = 1
gripSense = 1

# Automatic Setup of Robot. After these steps, joints should no longer be able to move freely
# print("Press any key to Init")
# getch()

start()
servo = []
for i in range(servoList[-1][0]+1):
    print("index: " + str(i))
    servo.append(Servo((servoList[i][0],servoList[i][1],servoList[i][2]),EnableTorque = True))
    print("Servo " + str(i) + " Success")

#home(r, phi, theta, (servo[0], servo[1],servo[2], servo[3]))
servo[4].writeAngle(grip) # Opens gripper

##########################################
#### Write Code to Control Arm Below #####
##########################################

trajSteps = 50

print("Press any key to Start Controller")
getch()

# traj = interpolateJoint(servo[1],180, trajSteps)
# for i in traj:
#     servo[1].writeAngle(i)
servo[0].writeAngle(90)
print("press key for next servo")
getch()
servo[1].writeAngle(90)
print("press key for next servo")
getch()
servo[2].writeAngle(90)
print("press key for next servo")
getch()
servo[3].writeAngle(90)

# while(1):
#     time.sleep(0.2)
#     print("\n\n")
#     print("Servo 0 angle: " + str(servo[0].readAngle()))
#     print("Servo 1 angle: " + str(servo[1].readAngle()))
#     print("Servo 2 angle: " + str(servo[2].readAngle()))
#     print("Servo 3 angle: " + str(servo[3].readAngle()))
    
    # theta, phi, rOut, rIn, gripperClose, gripperOpen = joy.read()
    # if theta or phi or rOut or rIn or gripperClose or gripperOpen:  # don't run following lines if there isn't a need

    #     # adjust params based on controller input, then multiply by sensitivity think
    #     theta += input[0] * thetaSense
    #     phi += input[1] * phiSense
    #     r += (input[2] - input[3]) * rSense
    #     grip += (gripperClose - gripperOpen) * gripSense

    #     theta, phi, r, grip = protection(theta, phi, r, grip) # Basic input protection to prevent hyper-extension

    #     ikAndWrite(theta, phi, r, grip) # No interpolation. Designed to be used ONLY in conjunction with controller
        
        
###########################################
##### Write Code to Control Arm Above #####
###########################################

# DO NOT TOUCH
end(servoList[-1][0])
