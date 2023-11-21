from numpy import *
from Robo_Control import *
from sympy import *

def interpolateJoint(servo, end, steps): # Needs servo[x], final degree, and time
    # Need to reformat this in the future. This is a little janky
    start = servo.readAngle()
    vector = round(linspace(start,end,steps))
    print(vector)
    return vector



def DHParams(a1, a2):
    l1 = 95    # Given link lengths
    l2 = 100
    # a1,a2 = symbols('a1, a2')

    # DH table values which are wrong
    dhTable = array([[180 - a1,          0,     l1,      0],
                    [pi/2 + (180-a2),    0,      l2,     0]])
    
    rows = len(dhTable)                          #    Finds number of rows
    retMat = array([ [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])     # Creates identity matrix as base
    for i in range(rows):                                      # loops for number of rows
        retMat = matmul(retMat, dh2mat(dhTable[i]))    # multiplies each transformation matrix

    return retMat


def dh2mat(dhparams):
    theta = dhparams[0] # Extracts components
    d = dhparams[1]
    a = dhparams[2]
    alpha = dhparams[3]
    # DH transformation
    transform = array([[cos(theta),  -sin(theta)*cos(alpha), sin(theta)*sin(alpha),  a*cos(theta)],
                        [sin(theta),    cos(theta)*cos(alpha),  -cos(theta)*sin(alpha), a*sin(theta)],
                        [0,             sin(alpha),             cos(alpha),             d],
                        [0,             0,                      0,                      1]])
    return transform
            #print(retMat)
### Testing ###
print(DHParams(45., 45.))
