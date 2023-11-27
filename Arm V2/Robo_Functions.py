import numpy as np
from Robo_Control import *
import sympy as sym

def rad2deg(val):
    return val*180/np.pi

def deg2rad(val):
    return val/180*np.pi

def interpolateJoint(servo, end, steps = 50): # Needs servo[x], final degree, and "time"
    # Need to reformat this in the future. This is a little janky
    start = servo.readAngle()
    vector = np.round(np.linspace(start,end,steps))
    print(vector)
    return vector

def DHParams(a1, a2):   # Probably wont be used
    l1 = 95    # Given link lengths
    l2 = 100
    # a1,a2 = symbols('a1, a2')

    # DH table values which are wrong
    dhTable = np.array([[180 - a1,          0,     l1,      0],
                    [sym.pi/2 + (180-a2),    0,      l2,     0]])
    
    rows = len(dhTable)                          #    Finds number of rows
    retMat = np.array([ [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])     # Creates identity matrix as base
    for i in range(rows):                                      # loops for number of rows
        retMat = np.matmul(retMat, dh2mat(dhTable[i]))    # multiplies each transformation matrix

    return retMat

def dh2mat(dhparams):   # Probably wont be used
    theta = dhparams[0] # Extracts components
    d = dhparams[1]
    a = dhparams[2]
    alpha = dhparams[3]
    # DH transformation
    transform = np.array([[sym.cos(theta),  -sym.sin(theta)*sym.cos(alpha), sym.sin(theta)*sym.sin(alpha),  a*sym.cos(theta)],
                        [sym.sin(theta),    sym.cos(theta)*sym.cos(alpha),  -sym.cos(theta)*sym.sin(alpha), a*sym.sin(theta)],
                        [0,             sym.sin(alpha),             sym.cos(alpha),             d],
                        [0,             0,                      0,                      1]])
    return transform
            #print(retMat)

def ik(r,phi,theta):    # Give in degrees from 0 to 180(ish)
    ## R Calculations
    theta0 = deg2rad(theta)

    la = 130 # 
    lb = 25  # 
    l1 = np.sqrt(la**2 + lb**2)
    l2 = 125 # mm
    theta2P = np.arccos((r**2 - l1**2 - l2**2)/(-2*l1*l2))
    thetaA = np.arctan(lb/la)
    theta2 = np.pi-theta2P + thetaA

    # Phi Calculations
    #theta1, x, y = sym.symbols('theta1, x, y')
    # eq1 = sym.Eq(l2 * sym.cos(theta1 + theta2) + l1 * sym.cos(theta1 + thetaA), x)
    # eq2 = sym.Eq(l1 * sym.sin(theta1 + thetaA) + l2 * sym.sin(theta1 + theta2), y)
    # eq3 = sym.Eq(sym.tan(y/x), phi)
    # eq = sym.Eq(sym.atan2(l1 * sym.sin(theta1 + thetaA) + l2 * sym.sin(theta1 + theta2), l2 * sym.cos(theta1 + theta2) + l1 * sym.cos(theta1 + thetaA)), phi)
    # solution = sym.solve(eq, theta1)
    # print(solution)

    # This was found using MATLAB from equations above
    #theta1 = -np.log((la*((la**2 + lb**2)/la**2)**(1/2)*1j)/(la*(la**2 + lb**2)**(1/2)*(1 - 1j) + lb*(la**2 + lb**2)**(1/2)*(1 + 1j) + l2*la*np.exp(theta1*1j)*((la**2 + lb**2)/la**2)**(1/2)*(1 - 1j)))*1j
    theta1 = np.real(-np.log((la*np.exp(phi*1j)*((la**2 + lb**2)/la**2)**(1/2))/(4*(la*(la**2 + lb**2)**(1/2) + lb*(la**2 + lb**2)**(1/2)*1j + l2*la*np.exp(theta2*1j)*((la**2 + lb**2)/la**2)**(1/2))))*1j)
    theta3 = (np.pi - theta2P - (deg2rad(phi)-theta1 + thetaA))

    theta0 = abs(theta0)
    theta1 = abs(theta1)
    theta2 = abs(theta2)
    theta3 = abs(theta3)

    print("theta0: " + str(theta0))
    print("theta1: " + str(rad2deg(theta1)))
    print("theta2: " + str(rad2deg(theta2)))
    print("theta3: " + str(rad2deg(theta3)))
    return theta0, theta1, theta2, theta3

print(ik(100, 25, 45))

# Functions
def home(r,phi,theta,servos):
    j0, j1, j2, j3 = ik(r,phi,theta)
    j0Vec = interpolateJoint(servos[0],j0)
    j1Vec = interpolateJoint(servos[1],j1)
    j2Vec = interpolateJoint(servos[2],j2)
    j3Vec = interpolateJoint(servos[3],j3)
    for i in range(len(j0Vec)):
        servos[0].writeAngle(j0Vec[i])
        servos[1].writeAngle(j1Vec[i])
        servos[2].writeAngle(j2Vec[i])
        servos[3].writeAngle(j3Vec[i])

def ikAndWrite(r, phi, theta, grip, servos):
    theta0, theta1, theta2, theta3 = ik(r,phi,theta)
    servos[0].writeAngle(theta0)
    servos[1].writeAngle(theta1)
    servos[2].writeAngle(theta2)
    servos[3].writeAngle(theta3)
    servos[4].writeAngle(grip)

def openGripper(self): # Opens the gripper
    self.writeAngle(0)
    print("Closing Gripper")

def closeGripper(self): # Closes the gripper
    self.writeAngle(140)
    print("Openning Gripper")

def protection(theta, phi, r, grip): # Basic input protections to hopefully avoid breaking something
    if theta > 180: theta = 180
    if theta < 0: theta = 0
    if phi > 180: phi = 180
    if phi < 0: phi = 0
    if r > 180: r = 180
    if r < 20: r = 20
    if grip > 140: grip = 140
    if grip < 0: grip = 0

    return theta, phi, r, grip