# Owen Rouse - 11/18/2023
# Robo_Run

import numpy as np
from Robo_Control import *
from Robo_Functions import *
import Robo_Gamepad as gp

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

thetaSense = 5
phiSense = 5
rSense = 5
gripSense = 5

# Automatic Setup of Robot. After these steps, joints should no longer be able to move freely
print("Press any key to Init")
getch()

start()
servo = []
for i in range(servoList[-1][0]+1):
    print("index: " + str(i))
    servo.append(Servo((servoList[i][0],servoList[i][1],servoList[i][2]),EnableTorque = True))
    print("Servo " + str(i) + " Success")

home(r, phi, theta, (servo[0], servo[1],servo[2], servo[3]))
servo[4].writeAngle(grip) # Opens gripper

joy = gp.XboxController()

########################################## 
#### Write Code to Control Arm Below #####
##########################################

trajSteps = 50

print("Press any key to Start Controller")
getch()
print("Running! Press A on controller to quit")
# try:
while(1):
    
    theta_delta, phi_delta, rOut_delta, rIn_delta, gripperClose_delta, gripperOpen_delta, A = joy.read()
    if A: break
    if theta_delta or phi_delta or rOut_delta or rIn_delta or gripperClose_delta or gripperOpen_delta:  # don't run following lines if there isn't a need

        # adjust params based on controller input, then multiply by sensitivity think
        theta += theta_delta * thetaSense
        phi += phi_delta * phiSense
        r += (rOut_delta - rIn_delta) * rSense
        grip += (gripperOpen_delta - gripperClose_delta) * gripSense

        theta, phi, r, grip = protectionJoints(theta, phi, r, grip)
        ikAndWrite(r, phi, theta, grip, servo) # No interpolation. Designed to be used ONLY in conjunction with controller

# except:
#     print("Error has Occured. Terminating Robo Arm...")
#     end(servoList[-1][0])
        
###########################################
##### Write Code to Control Arm Above #####
###########################################

# DO NOT TOUCH
end(servoList[-1][0])
