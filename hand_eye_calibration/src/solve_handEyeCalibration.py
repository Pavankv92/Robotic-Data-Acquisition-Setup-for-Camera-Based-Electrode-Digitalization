#/usr/bin/env python3

""" computes the solution for handeye calibration """


import numpy as np
from numpy.linalg import inv
from numpy.linalg import multi_dot


def matrixGeneration(M,P):
    N = inv(P)
    a = np.array([[M[0,0],M[0,1],M[0,2]],[M[1,0],M[1,1],M[1,2]],[M[2,0],M[2,1],M[2,2]]])
    A00 = a*N[0,0]
    A01 = a*N[1,0]
    A02 = a*N[2,0]
    A03 = np.zeros(shape=(3,3))
    A10 = a*N[0,1]
    A11 = a*N[1,1]
    A12 = a*N[2,1]
    A13 = np.zeros(shape=(3,3))
    A20 = a*N[0,2]
    A21 = a*N[1,2]
    A22 = a*N[2,2]
    A23 = np.zeros(shape=(3,3))
    A30 = a*N[0,3]
    A31 = a*N[1,3]
    A32 = a*N[2,3]
    A33 = a
    
    A = np.array([[A00[0,0],A00[0,1],A00[0,2],A01[0,0],A01[0,1],A01[0,2],A02[0,0],A02[0,1],A02[0,2],A03[0,0],A03[0,1],A03[0,2]],
                 [A00[1,0],A00[1,1],A00[1,2],A01[1,0],A01[1,1],A01[1,2],A02[1,0],A02[1,1],A02[1,2],A03[1,0],A03[1,1],A03[1,2]],
                 [A00[2,0],A00[2,1],A00[2,2],A01[2,0],A01[2,1],A01[2,2],A02[2,0],A02[2,1],A02[2,2],A03[2,0],A03[2,1],A03[2,2]],
                 [A10[0,0],A10[0,1],A10[0,2],A11[0,0],A11[0,1],A11[0,2],A12[0,0],A12[0,1],A12[0,2],A13[0,0],A13[0,1],A13[0,2]],
                 [A10[1,0],A10[1,1],A10[1,2],A11[1,0],A11[1,1],A11[1,2],A12[1,0],A12[1,1],A12[1,2],A13[1,0],A13[1,1],A13[1,2]],
                 [A10[2,0],A10[2,1],A10[2,2],A11[2,0],A11[2,1],A11[2,2],A12[2,0],A12[2,1],A12[2,2],A13[2,0],A13[2,1],A13[2,2]],
                 [A20[0,0],A20[0,1],A20[0,2],A21[0,0],A21[0,1],A21[0,2],A22[0,0],A22[0,1],A22[0,2],A13[0,0],A23[0,1],A23[0,2]],
                 [A20[1,0],A20[1,1],A20[1,2],A21[1,0],A21[1,1],A21[1,2],A22[1,0],A22[1,1],A22[1,2],A13[1,0],A23[1,1],A23[1,2]],
                 [A20[2,0],A20[2,1],A20[2,2],A21[2,0],A21[2,1],A21[2,2],A22[2,0],A22[2,1],A22[2,2],A13[2,0],A23[2,1],A23[2,2]],
                 [A30[0,0],A30[0,1],A30[0,2],A31[0,0],A31[0,1],A31[0,2],A32[0,0],A32[0,1],A32[0,2],A33[0,0],A33[0,1],A33[0,2]],
                 [A30[1,0],A30[1,1],A30[1,2],A31[1,0],A31[1,1],A31[1,2],A32[1,0],A32[1,1],A32[1,2],A33[1,0],A33[1,1],A33[1,2]],
                 [A30[2,0],A30[2,1],A30[2,2],A31[2,0],A31[2,1],A31[2,2],A32[2,0],A32[2,1],A32[2,2],A33[2,0],A33[2,1],A33[2,2]]])
    
    A = A.reshape(12,12)
    i = -1*np.identity(12)
    Ai = np.concatenate([A,i],axis=1)
    Bi = np.concatenate([np.zeros(shape=(9,1)),-1*M[:3,3:4]],axis=0)
    return Ai,Bi

def finalMatrix(m,n, number):
    A = []
    B = []
    for i in range(number):
        M = m[i]
        N = n[i]
        C, D = matrixGeneration(M,N)
        A.append(C) 
        B.append(D) 
    Arr = np.asarray(A)
    Brr = np.asarray(B)
    return Arr,Brr

def solution(x):
    Xarr = np.concatenate([x[0],x[3],x[6],x[9],
                     x[1],x[4],x[7],x[10],
                     x[2],x[5],x[8],x[11],
                     [0],[0],[0],[1]])
    Yarr = np.concatenate([x[12],x[15],x[18],x[21],
                     x[13],x[16],x[19],x[22],
                     x[14],x[17],x[20],x[23],
                     [0],[0],[0],[1]])
    Xarr = Xarr.reshape(4,4)
    Yarr = Yarr.reshape(4,4)
    return Xarr,Yarr

def solve(m, n, number):

    """ computation of the handeye solution

    Arguements: m = list of ee_T_robot poses
                n = list of kinect_T_chkbd
                number = number poses (usually 40 poses are enough)

    Returns: solution 4X4 matrices
    """
    A,B = finalMatrix(m,n, number)
    A = A.reshape(number*12,24)
    B = B.reshape(number*12,1)
    q,r = np.linalg.qr(A)
    #print(q,r)
    xStar = np.matmul(inv(r), (np.matmul(np.transpose(q), B) ) )
    #xStar = multi_dot([inv(r),q.transpose(),B])
    #print(xStar)
    X,Y = solution(xStar)
    #print(X,Y)
    return(X,Y)
    
if __name__ == "__main__":

    rob = np.load("/home/pavan/catkin_ws/src/project_arbeit/Data/hand_eye/rob.npy")
    cam = np.load("/home/pavan/catkin_ws/src/project_arbeit/Data/hand_eye/cam.npy")
    x,y = solve(rob,cam,40)
    print(x)
    print(y)
   
   
"""
    M1 = np.array([[-0.67, -0.07, -0.73, -6.45],[-0.00419971, -0.99,  0.107, 5.28],[-0.74081168,  0.07511192,  0.6675 , 41.96],[0, 0, 0, 1]])
    M2 = np.array([[17, 18, 19, 20],[21, 22, 23, 24],[25, 26, 27, 28],[0, 0, 0, 1]])
    M3 = np.array([[33, 34, 35, 36],[37, 38, 39, 40],[41, 42, 43, 44],[0, 0, 0, 1]])
    M4 = np.array([[33, 34, 35, 36],[37, 38, 39, 40],[41, 42, 43, 44],[0, 0, 0, 1]])
    N1 = np.array([[-0.67, -0.07, -0.73, -6.45],[-0.00419971, -0.99,  0.107, 5.28],[-0.74081168,  0.07511192,  0.6675 , 41.96],[0, 0, 0, 1]])
    N2 = np.array([[-0.87945513, -0.11204358,  0.46260664, 7.35134058],[0.14218041, -0.98936542,  0.03067237, 6.52867594],[0.45425037,  0.09274858,  0.8860329, 21.77558425],[0, 0, 0, 1]])
    N3 = np.array([[-0.99281396, -0.11358088,  0.03768061, 2.3966946 ],[0.11397872, -0.99344617,  0.00857661, 0.66042136],[0.03645952,  0.01280977,  0.99925303, 24.33955806],[0, 0, 0, 1]])
    N4 = np.array([[-0.99281396, -0.11358088,  0.03768061, 2.3966946 ],[0.11397872, -0.99344617,  0.00857661, 0.66042136],[0.03645952,  0.01280977,  0.99925303, 24.33955806],[0, 0, 0, 1]])
    m = np.array([M1,M2,M3, M4])
    n = np.array([N1,N2,N3, N4])
    print(solve(m,n,4))
"""
    