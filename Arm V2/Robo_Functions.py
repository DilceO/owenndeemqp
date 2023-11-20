import numpy as np

def lab2_5(a1, a2, a3):
    l1 = 95    # Given link lengths
    l2 = 100
    l3 = 100
  
    # DH table values
    dh_table = [a1,             l1, 0,      -np.pi/2,
               -np.pi/2 + a2,   0,  l2,     0,
                np.pi/2 + a3,   0,  l3,     0]
    
    
    dh2fk(dh_table)


def dh2mat(dhparams):
    theta = dhparams(1) # Extracts components
    d = dhparams(2)
    a = dhparams(3)
    alpha = dhparams(4)
    # DH transformation
    transform = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                    [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                    [0, np.sin(alpha), np.cos(alpha), d],
                    [0, 0, 0, 1]])
    return transform


def dh2fk(dhTable):    
    rows = len(dhTable);                          #    Finds number of rows
    retMat = np.array([ [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]);     # Creates identity matrix as base
    for i in range(1,rows):                                      # loops for number of rows
        retMat = np.multiply(retMat, dh2mat(dhTable[i,:]));    # multiplies each transformation matrix
    return retMat
    
