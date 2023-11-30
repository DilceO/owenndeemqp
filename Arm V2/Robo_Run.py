# Owen Rouse - 11/18/2023
# Robo_Run

from Robo_Control import *
from Robo_Functions import *
import Robo_Gamepad as gp

# Here are the variables needed to set up the arm:
# Servo 0 is the base, servo 1 is the next one...
# [Servo Num, Min Degrees, Max Degrees]
servoList = [[0, 90, 270], # Servo 0 (Base)
           [1, 90, 270], # Servo 1 
           [2, 90, 260], # ...
           [3, 90, 270],
           [4, 90, 230]]
### These will all be normailzed to 0. This means That MOST will be from 0 - 180 ###

# Home coordinates
r = 100     # mm
phi = 135   # Degrees
theta = 90  # Degrees
grip = 20    # 0 is open,140 is closed

# Sensetivity Values for contoller. 1 is natural. 2 is higher sensitivity

thetaSense = 3
phiSense = 3
rSense = 3
gripSense = 5

# Automatic Setup of Robot. After these steps, joints should no longer be able to move freely
print("Press any key to Init")
getch()

start()
servo = []
for i in range(servoList[-1][0]+1):
    servo.append(Servo((servoList[i][0],servoList[i][1],servoList[i][2]),EnableTorque = True))

home(r, phi, theta, (servo[0], servo[1],servo[2], servo[3]))
servo[4].writeAngle(grip) # Opens gripper

joy = gp.XboxController()

########################################## 
#### Write Code to Control Arm Below #####
##########################################
print("Press any key to Start Controller")
getch()
print("Running! Press A on controller to quit")
ind = 0
try:
    while(1):
        theta_delta, phi_delta, rOut_delta, rIn_delta, gripperClose_delta, gripperOpen_delta, A_Break = joy.read()
        if A_Break: break
        #elif theta_delta or phi_delta or rOut_delta or rIn_delta or gripperClose_delta or gripperOpen_delta:  # don't run following lines if there isn't a need
        # adjust params based on controller input, then multiply by sensitivity
        if theta_delta: 
            theta += theta_delta * thetaSense
            theta = protectTheta(theta)
            ind = 1
        if phi_delta: 
            phi += phi_delta * phiSense
            phi = protectPhi(phi)
            ind = 1
        if rIn_delta or rOut_delta: 
            r += (rOut_delta - rIn_delta) * rSense
            r = protectR(r)
            ind = 1
        if gripperOpen_delta or gripperClose_delta: 
            grip += (gripperOpen_delta - gripperClose_delta) * gripSense
            grip = protectGrip(grip)
            ind = 1
        if ind:
            ind = 0
            ikAndWrite(r, phi, theta, grip, servo) # No interpolation. Designed to be used ONLY in conjunction with controller

except:
    print("Error has Occured. Terminating Robo Arm...")
    end(servoList[-1][0])
        
###########################################
##### Write Code to Control Arm Above #####
###########################################

# DO NOT TOUCH
end(servoList[-1][0])
