import numpy as np
from Robo_Control import *
import sympy as sym

def rad2deg(val):
    return val*180/np.pi

def deg2rad(val):
    return val/180*np.pi


def protectionJoints(theta, phi, r, grip): # Basic input protections to hopefully avoid breaking something
    if theta > 180: 
        theta = 180
        print("Protection used at theta = " + str(theta))
    if theta < 0: 
        theta = 0
        print("Protection used at theta = " + str(theta))
    if phi > 180: 
        phi = 180
        print("Protection used at phi = " + str(phi))
    if phi < 0: 
        phi = 0
        print("Protection used at phi = " + str(phi))
    if r > 250: 
        r = 250
        print("Protection used at r = " + str(r))
    if r < 20: 
        r = 20
        print("Protection used at r = " + str(r))
    if grip > 140:
        grip = 140
        print("Protection used at grip = " + str(grip))
    if grip < 0:
        grip = 0
        print("Protection used at grip = " + str(grip))
    
    return theta, phi, r, grip

def interpolateJoint(servo, end, steps = 30): # Needs servo[x], final degree, and "time"
    # Need to reformat this in the future. This is a little janky
    start = servo.readAngle()
    vector = np.round(np.linspace(start,end,steps))
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

def ik(r,phi,theta):    # Give in degrees from 0 to 180(ish)
    ## r Calculations
    theta0 = deg2rad(theta)
    phi = deg2rad(phi)

    la = 130 # mm
    lb = 25  # mm
    l1 = np.sqrt(la**2 + lb**2)
    l2 = 125 # mm
    theta2P = np.arccos((r**2 - l1**2 - l2**2)/(-2*l1*l2))
    thetaA = np.arctan(lb/la)
    theta2 = np.pi-theta2P + thetaA
    theta2 = np.real(theta2)
    
    # Elbow Down(?)
    # theta1_0 = -np.log(-(2*r*np.exp(phi*1j)*(la**2 + lb**2)**(1/2))/(r**2 - l2**2 + la**2 + lb**2 + l2*(la**2 + lb**2)**(1/2)*(-(r**4 - 2*r**2*l2**2 - 2*r**2*la**2 - 2*r**2*lb**2 + l2**4 - 2*l2**2*la**2 - 2*l2**2*lb**2 + la**4 + 2*la**2*lb**2 + lb**4)/(4*l2**2*(la**2 + lb**2)))**(1/2)*2*1j))*1j - np.arctan(lb/la)
    # print(np.real(theta1_0))

    # Elbow Up(?)
    theta1_1 = -np.log((2*r*np.exp(phi*1j)*(la**2 + lb**2)**(1/2))/(r**2 - l2**2 + la**2 + lb**2 + l2*(la**2 + lb**2)**(1/2)*(-(r**4 - 2*r**2*l2**2 - 2*r**2*la**2 - 2*r**2*lb**2 + l2**4 - 2*l2**2*la**2 - 2*l2**2*lb**2 + la**4 + 2*la**2*lb**2 + lb**4)/(4*l2**2*(la**2 + lb**2)))**(1/2)*2*1j))*1j - np.arctan(lb/la)
    # print(np.real(theta1_1))
    theta1 = np.real(theta1_1)

    theta3 = np.pi/2 - (np.pi - (phi - theta1) - ((np.pi - theta2) + thetaA))
    theta3 = np.real(theta3)

    return rad2deg(theta0), rad2deg(theta1), rad2deg(theta2), rad2deg(theta3)

print(ik(20, 135, 90))

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
    self.writeAngle(20)

def closeGripper(self): # Closes the gripper
    self.writeAngle(130)