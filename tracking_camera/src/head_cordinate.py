#!/usr/bin/env python3

import numpy as np
from numpy import linalg as la
import scipy.linalg as las

class createHeadCordinate(object):

    def __init__(self, rightear, nose, leftear):   
        
        self.a = rightear
        self.p = nose
        self.b = leftear

    def create_csys(self):

        n = self.b-self.a
        #print(n)
        n = n/la.norm(n,2) #  normalised vector
        #print(n)
        t = (self.a-self.p) - np.multiply(np.dot((self.a-self.p),n),n)
        #print('t')
        #print(t)
        origin = self.p+t
        #print('origin')
        #print(origin)
        x = -t /la.norm(t,2)
        #print(x)
        y = self.b - origin
        y = y / la.norm(y,2) 
        #print(y)
        z = np.cross(x,y)
        #print(z)
        csys = np.transpose(np.reshape(np.array([x,y,z]),(3,3))) # donot transpose! its wrong!
        #print(csys)
        #print(la.matrix_rank(csys))
        csys_basis = las.orth(csys)
        #print('basis')
        #print(csys_basis)
        identity = np.eye(la.matrix_rank(csys_basis))
        #print(la.matrix_rank(csys_basis))
        check = np.matmul(np.transpose(csys_basis),csys_basis)
        #print(check)
        error = la.norm((identity-check),'fro')
        #print(error)

        csys_head = np.eye(4)
        csys_head [0:3, 0:3] = csys[0:3, 0:3]
        csys_head [0,3] = origin[0]
        csys_head [1,3] = origin[1]
        csys_head [2,3] = origin[2]
        #print(csys_head)

        return csys_head, error
    


if __name__ == "__main__":

    v1 = np.array([
                  [-0.9943, -0.0355, 0.1005, 239.80],
                  [-0.1063, 0.2705, -0.9568, 53.6150],
                  [0.0068, -0.9621, -0.2727, 987.82],
                  [0,0,0,1]
                  ])
    v2 = np.array([
                  [-0.2847, 0.3139, -0.9058, 210.12],
                  [0.9215, -0.1710, -0.3488, -51.004],
                  [-0.2644, -0.9339, -0.2406, 1001.3],
                  [0,0,0,1]
                  ])
    v3 = np.array([
                  [.8415, -0.1119, -0.5285, 115.07],
                  [0.5313, -0.0054, 0.8472, 18.9340],
                  [-0.0976, -0.9937, 0.0549, 974.16],
                  [0,0,0,1]
                  ])
    
    
    a = v1[0:3,3] # right ear of the head
    p = v2[0:3,3] # nose of the head
    b = v3[0:3,3] # left ear of the head
    print(a)
    print(p)
    print(b)
    c = createHeadCordinate(a,p,b)
    csys, error = c.create_csys()
    #print(csys)
    print(np.matmul(la.inv(csys),v1))
    print(np.matmul(la.inv(csys),v2))
    print(np.matmul(la.inv(csys),v3))
    
    
    