import numpy as np
from Robo_Control import *
from math import *

def interpolateJoint(servo, end, steps): # Needs servo[x], final degree, and time
    # Need to reformat this in the future. This is a little janky
    start = servo.readAngle()
    vector = np.round(np.linspace(start,end,steps))
    print(vector)
    return vector



def lab2_5(a1, a2, a3):
    l1 = 95    # Given link lengths
    l2 = 100
    l3 = 100
  
    # DH table values
    dh_table = [a1,         l1, 0,      -pi/2,
               -pi/2 + a2,   0,  l2,     0,
                pi/2 + a3,   0,  l3,     0]
    
    
    dh2fk(dh_table)


def dh2mat(dhparams):
    theta = dhparams(1) # Extracts components
    d = dhparams(2)
    a = dhparams(3)
    alpha = dhparams(4)
    # DH transformation
    transform = np.array([[cos(theta),  -sin(theta)*cos(alpha), sin(theta)*sin(alpha),  a*cos(theta)],
                        [sin(theta),    cos(theta)*cos(alpha),  -cos(theta)*sin(alpha), a*sin(theta)],
                        [0,             sin(alpha),             cos(alpha),             d],
                        [0,             0,                      0,                      1]])
    return transform


def dh2fk(dhTable):    
    rows = len(dhTable);                          #    Finds number of rows
    retMat = np.array([ [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]);     # Creates identity matrix as base
    for i in range(1,rows):                                      # loops for number of rows
        retMat = np.multiply(retMat, dh2mat(dhTable[i,:]));    # multiplies each transformation matrix
    return retMat
    
