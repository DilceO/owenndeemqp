import numpy as np
from Robo_Control import *

# Linkages
la = 128 # mm
lb = 24  # mm
l1 = np.sqrt(la**2 + lb**2)
l2 = 124 # mm
thetaA = np.arctan(lb/la)
l2_sq = l2**2
la_sq = la**2
lb_sq = lb**2
elbowUp_denominator = 4 * l2_sq * (la_sq + lb_sq)

def rad2deg(val):
    return val*180/np.pi

def deg2rad(val):
    return val/180*np.pi

def protectTheta(theta):
    if theta > 180: 
        theta = 180
        print("Protection used at theta = " + str(theta))
    elif theta < 0: 
        theta = 0
        print("Protection used at theta = " + str(theta))
    return theta
    
def protectPhi(phi):
    if phi > 180: 
        phi = 180
        print("Protection used at phi = " + str(phi))
    elif phi < 0: 
        phi = 0
        print("Protection used at phi = " + str(phi))
    return phi

def protectR(r):
    if r > 250: 
        r = 250
        print("Protection used at r = " + str(r))
    elif r < 50: 
        r = 50
        print("Protection used at r = " + str(r))
    return r
    

def protectGrip(grip):
    if grip > 140:
        grip = 140
        print("Protection used at grip = " + str(grip))
    elif grip < 0:
        grip = 0
        print("Protection used at grip = " + str(grip))
    return grip
        

def interpolateJoint(servo, end, steps = 30): # Needs servo[x], final degree, and "time"
    # Need to reformat this in the future. This is a little janky, but also wont be used much
    start = servo.readAngle()
    vector = np.round(np.linspace(start,end,steps))
    return vector

def ik(r,phi,theta0):    # Give in degrees from 0 to 180(ish)
    ## Calculations. Everything below MUST be run every time. If it doesn't need to be run every time, it better not be below
    phi = deg2rad(phi)
    r_sq = r**2
    theta2P = np.arccos((r_sq - l1**2 - l2_sq)/(-2*l1*l2))
    theta2 = np.pi-theta2P + thetaA
    
    # Elbow Down - Don't need for calculations for this specific arm
    # theta1_0 = -np.log(-(2*r*np.exp(phi*1j)*l1)/(r**2 - l2**2 + la**2 + lb**2 + l2*l1*(-(r**4 - 2*r**2*l2**2 - 2*r**2*la**2 - 2*r**2*lb**2 + l2**4 - 2*l2**2*la**2 - 2*l2**2*lb**2 + la**4 + 2*la**2*lb**2 + lb**4)/(4*l2**2*(la**2 + lb**2)))**(1/2)*2*1j))*1j - thetaA

    # Elbow Up
    #theta1 = np.real(-np.log((2*r*np.exp(phi*1j)*l1)/(r**2 - l2**2 + la**2 + lb**2 + l2*l1*(-(r**4 - 2*r**2*l2**2 - 2*r**2*la**2 - 2*r**2*lb**2 + l2**4 - 2*l2**2*la**2 - 2*l2**2*lb**2 + la**4 + 2*la**2*lb**2 + lb**4)/(4*l2**2*(la**2 + lb**2)))**(1/2)*2*1j))*1j - thetaA)

    ### Hopefully More efficient Elbow up below
    numerator = r_sq - l2_sq + la_sq + lb_sq
    complex_expr = (r_sq**2 - 2*r_sq*l2_sq - 2*r_sq*la_sq - 2*r_sq*lb_sq + l2_sq**2 - 2*l2_sq*la_sq - 2*l2_sq*lb_sq + la_sq**2 + 2*la_sq*lb_sq + lb_sq**2)
    theta1 = np.real(-np.log((2*r*np.exp(phi*1j)*l1)/(numerator + l2*l1*(-complex_expr/elbowUp_denominator)**(1/2)*2*1j))*1j - thetaA)

    theta3 = np.pi/2 + phi - theta1 - theta2 - thetaA

    return theta0, rad2deg(theta1), rad2deg(theta2), rad2deg(theta3)

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

def closeGripper(self): # Closes the gripper
    self.writeAngle(140)

### Unnecessary Code Below - Probably will delete later on ###

# def protectionJoints(theta, phi, r, grip): # Basic input protections to hopefully avoid breaking something... again
#     if theta > 180: 
#         theta = 180
#         print("Protection used at theta = " + str(theta))
#     if theta < 0: 
#         theta = 0
#         print("Protection used at theta = " + str(theta))
#     if phi > 180: 
#         phi = 180
#         print("Protection used at phi = " + str(phi))
#     if phi < 0: 
#         phi = 0
#         print("Protection used at phi = " + str(phi))
#     if r > 250: 
#         r = 250
#         print("Protection used at r = " + str(r))
#     if r < 50: 
#         r = 50
#         print("Protection used at r = " + str(r))
#     if grip > 140:
#         grip = 140
#         print("Protection used at grip = " + str(grip))
#     if grip < 0:
#         grip = 0
#         print("Protection used at grip = " + str(grip))
    
#     return theta, phi, r, grip

# def DHParams(a1, a2):   # Probably wont be used
#     l1 = 95    # Given link lengths
#     l2 = 100
#     # a1,a2 = symbols('a1, a2')

#     # DH table values which are wrong
#     dhTable = np.array([[180 - a1,          0,     l1,      0],
#                     [sym.pi/2 + (180-a2),    0,      l2,     0]])
    
#     rows = len(dhTable)                          #    Finds number of rows
#     retMat = np.array([ [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])     # Creates identity matrix as base
#     for i in range(rows):                                      # loops for number of rows
#         retMat = np.matmul(retMat, dh2mat(dhTable[i]))    # multiplies each transformation matrix

#     return retMat

# def dh2mat(dhparams):   # Probably wont be used
#     theta = dhparams[0] # Extracts components
#     d = dhparams[1]
#     a = dhparams[2]
#     alpha = dhparams[3]
#     # DH transformation
#     transform = np.array([[sym.cos(theta),  -sym.sin(theta)*sym.cos(alpha), sym.sin(theta)*sym.sin(alpha),  a*sym.cos(theta)],
#                         [sym.sin(theta),    sym.cos(theta)*sym.cos(alpha),  -sym.cos(theta)*sym.sin(alpha), a*sym.sin(theta)],
#                         [0,             sym.sin(alpha),             sym.cos(alpha),             d],
#                         [0,             0,                      0,                      1]])
#     return transform