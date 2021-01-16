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
        csys = np.transpose(np.reshape(np.array([x,y,z]),(3,3)))
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
        csys_head [0:3, 0:3] = csys_basis[0:3, 0:3]
        csys_head [0,3] = origin[0]
        csys_head [1,3] = origin[1]
        csys_head [2,3] = origin[2]
        #print(csys_head)

        return csys_head, error
    


if __name__ == "__main__":

    a = np.array([239.89, 53.615, 987.82]) # right ear of the head
    p = np.array([210.12,-51.004, 1001.3]) # nose of the head
    b = np.array([115.07, 18.934, 974.16]) # left ear of the head
    print(a)
    c = createHeadCordinate(a,p,b)
    csys = c.create_csys()
    print(csys)
    
    
    