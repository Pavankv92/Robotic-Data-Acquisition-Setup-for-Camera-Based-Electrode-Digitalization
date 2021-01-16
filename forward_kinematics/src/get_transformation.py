#!/usr/bin/env python

import numpy as np
import math
from multiprocessing import Pool

# TODO Return errors for wrong values

def create_transformation_matrix_from_dh(theta, index=6):
    """
    This function returns the transformation matrix from a given frame to the base 
    using the given Denavit-Hartenberg parameters of the UR3
    
    Parameters
    ----------
    theta: array with the current rotations in rad of the joints
    index: the link for which the transformation is returned, range: 1 to 6 (inclusive)

    Returns
    -------
    a 4x4 array with the transformation
    """
    #UR3
    a = np.array([0, -0.24365, -0.21325, 0, 0, 0])
    alpha = np.array([np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0])
    d = np.array([0.1519, 0, 0, 0.11235, 0.08535, 0.0819])

    #UR5
    #a = np.array([0, -0.425, -0.39225, 0, 0, 0])
    #alpha = np.array([np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0])
    #d = np.array([0.089159, 0, 0, 0.10915, 0.09465, 0.0823])

    trans = np.zeros((index,4,4))
    #create each seperate transformation matrix from fram i-1 to i 
    for i in range(0,index):
        trans[i] = [[np.cos(theta[i]), -np.cos(alpha[i])*np.sin(theta[i]), np.sin(alpha[i])*np.sin(theta[i]), a[i]*np.cos(theta[i])],
                        [np.sin(theta[i]), np.cos(alpha[i])*np.cos(theta[i]), -np.sin(alpha[i])*np.cos(theta[i]), a[i]*np.sin(theta[i])],
                        [0, np.sin(alpha[i]), np.cos(alpha[i]), d[i]],
                        [0, 0, 0, 1]
                ]
    final_trans=trans[0]
    # if our index is > 1, we have to multiply the matrices to get the transformation from base to index
    for i in range(1,index):
        final_trans = np.dot(final_trans, trans[i])
    
    # we only used 15 decimals of pi, so no need to return 1e-25
    return np.around(final_trans, decimals=15)



def translation_from_transformation_matrix(matrix):
    """
    This function returns the translation in x, y, z direction from a given transformation matrix

    Parameters
    ----------
    matrix: a 4x4 array with the transformation

    Returns
    a tuple (x, y, z)
    """
    x = matrix[0][3]
    y = matrix[1][3]
    z = matrix[2][3]
    return (x, y, z)

def rotation_from_transformation_matrix(matrix):
    """
    This function returns the rotation in Euler Angles (Z, Y, X)

    Parameters
    ----------
    matrix: a 4x4 array with the transformation

    Returns
    a tuple with the rotations (rx, ry, rz)

    Source: https://learnopencv.com/rotation-matrix-to-euler-angles/

    """
    sy = np.square(matrix[0,0]**2 + matrix[1,0]**2)
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(matrix[2,1], matrix[2,2])
        y = math.atan2(-matrix[2,0], sy)
        z = math.atan2(matrix[1,0], matrix[0,0])
    else:
        x = math.atan2(-matrix[1,2], matrix[1,1])
        y = math.atan2(-matrix[2,0], sy)
        z = 0
    
    return (x, y, z)


"""
    INVERSE PART IS STARTING HERE
"""

def inverse_coordinates(forward_matrix):
    #UR5
    #d6 = 0.0823
    #d4 = 0.10915
    #a3 = -0.425
    #a4 = -0.39225
    #d1 = 0.089159
    #d5 = 0.09465
    #UR3
    d6 = 0.0819
    d4 = 0.11235
    a3 = -0.24365
    a4 = -0.21325
    d1 = 0.1519
    d5 = 0.08535

    h=[[0],[0],[-d6],[1]]
    P0_5 = np.dot(forward_matrix,h)
    inverseJointAngles = np.zeros((8,6))
    
    for i in range(0,8):
        if i==0:
            shoulderLeft = 1
            wristUp = 1
            elbowUp = 1
        elif i==1:
            shoulderLeft = 1
            wristUp = 1
            elbowUp = -1
        elif i==2:
            shoulderLeft = 1
            wristUp = -1
            elbowUp = 1
        elif i==3:
            shoulderLeft = 1
            wristUp = -1
            elbowUp = -1
        elif i==4:
            shoulderLeft = -1
            wristUp = 1
            elbowUp = 1
        elif i==5:
            shoulderLeft = -1
            wristUp = 1
            elbowUp = -1
        elif i==6:
            shoulderLeft = -1
            wristUp = -1
            elbowUp = 1
        elif i==7:
            shoulderLeft = -1
            wristUp = -1
            elbowUp = -1

        try:
            theta1 = math.atan2(P0_5[1],P0_5[0]) + shoulderLeft * math.acos(d4/math.sqrt(P0_5[0]*P0_5[0]+P0_5[1]*P0_5[1]))+np.pi/2

            theta5 = wristUp * math.acos((forward_matrix[0][3]*math.sin(theta1)-forward_matrix[1][3]*math.cos(theta1)-d4)/d6)

            theta6 = math.atan2((-forward_matrix[0][1]*math.sin(theta1) + forward_matrix[1][1]*math.cos(theta1)) / math.sin(theta5), (forward_matrix[0][0]*math.sin(theta1) - forward_matrix[1][0]*math.cos(theta1)) / math.sin(theta5))

            T1_0 = [[np.cos(theta1), np.sin(theta1), 0, 0],
                    [-np.sin(theta1), np.cos(theta1), 0, 0],
                    [0, 0, 1, -d1],
                    [0, 0, 0, 1]]

            T6_4 = [[np.cos(theta5)*np.cos(theta6), np.sin(theta6), np.sin(theta5)*np.cos(theta6), np.sin(theta6)*d5],
                    [-np.cos(theta5)*np.sin(theta6), np.cos(theta6), -np.sin(theta5)*np.sin(theta6), np.cos(theta6)*d5],
                    [-np.sin(theta5), 0, np.cos(theta5), -d6],
                    [0, 0, 0, 1]]

            T1_4 = np.dot(T1_0, forward_matrix)
            T1_4 = np.dot(T1_4, T6_4)

            theta3 = elbowUp * math.acos((T1_4[0][3]*T1_4[0][3] + T1_4[2][3]*T1_4[2][3]-a3*a3 - a4*a4) / (2*a3*a4))

            theta2 = math.atan2(-T1_4[2][3], -T1_4[0][3]) - math.asin((-a4*math.sin(theta3)) / math.sqrt(T1_4[0][3]*T1_4[0][3] + T1_4[2][3]*T1_4[2][3]))

            T3_1 = [[np.cos(theta2+theta3), 0, np.sin(theta2+theta3), -a3*np.cos(theta3)],
                    [-np.sin(theta3+theta2), 0, np.cos(theta3+theta2), a3*np.sin(theta3)],
                    [0, -1, 0, 0],
                    [0, 0, 0, 1]]

            T3_4 = np.dot(T3_1, T1_4)

            theta4 = math.atan2(T3_4[1][0], T3_4[0][0])
            inverseJointAngles[i] = [theta1, theta2, theta3, theta4, theta5, theta6]

        except:
            print("invalid value")
            inverseJointAngles[i] = [100, 100, 100, 100, 100, 100] #non sense result -> will never be chosen

    return (inverseJointAngles)

def find_Dtheta_and_t(inverse_coord, joint_states):
    v_max = 0.5
    a_current = 1000 #just a big value
    joint_states_des = np.array([0,0,0,0,0,0])
    for i in range(0,8):
        a = (joint_states[0]-inverse_coord[i][0])**2+(joint_states[1]-inverse_coord[i][1])**2+(joint_states[2]-inverse_coord[i][2])**2+(joint_states[3]-inverse_coord[i][3])**2+(joint_states[4]-inverse_coord[i][4])**2+(joint_states[5]-inverse_coord[i][5])**2
        if a < a_current:
            a_current = a
            joint_states_des = np.array([inverse_coord[i][0],inverse_coord[i][1],inverse_coord[i][2],inverse_coord[i][3],inverse_coord[i][4],inverse_coord[i][5]])
    delta_theta = joint_states_des - joint_states
    t = np.amax(delta_theta) / v_max
    Dtheta = delta_theta / t
    #Dtheta_and_t = [Dtheta[0], Dtheta[1], Dtheta[2], Dtheta[3], Dtheta[4], Dtheta[5], t]
    return (Dtheta, t)

def find_theta(inverse_coord, joint_states):
    dist = np.zeros(8)
    for i in range(8):
        dist[i] = np.linalg.norm((joint_states-inverse_coord[i]),2)
    min_ind = np.argmin(dist)
    return inverse_coord[min_ind]


if __name__ == "__main__":
    from timeit import default_timer as timer
#    theta_test = np.array([-24.12, -63.76, -82.19, -4.22, 87.63, 24.9])
#    theta_test = np.array([238.07, -98.96, -126.22, -46.29, 91.39, -1.78])
#    print(create_transformation_matrix_from_dh([theta_test[0]*(np.pi/180), theta_test[1]*(np.pi/180), theta_test[2]*(np.pi/180), theta_test[3]*(np.pi/180), theta_test[4]*(np.pi/180), theta_test[5]*(np.pi/180)], 6))
#    print(inverse_coordinates(create_transformation_matrix_from_dh([theta_test[0]*(np.pi/180), theta_test[1]*(np.pi/180), theta_test[2]*(np.pi/180), theta_test[3]*(np.pi/180), theta_test[4]*(np.pi/180), theta_test[5]*(np.pi/180)], 6)))

    # desired Matrix
    x = 0
    y = 0
    z = 0
    trans_x = 0.1
    trans_y = -0.035
    trans_z = 0.4
    T_x = [[1, 0, 0],
           [0, np.cos(x), -np.sin(x)],
           [0, np.sin(x), np.cos(x)]]

    T_y = [[np.cos(y), 0, np.sin(y)],
           [0, 1, 0],
           [-np.sin(y), 0, np.cos(y)]]
    T_z = [[np.cos(z), -np.sin(z), 0],
           [np.sin(z), np.cos(z), 0],
           [0, 0, 1]]

    T_rot = np.dot(T_z, T_y)
    T_rot = np.dot(T_rot, T_x)
    T_des = [[T_rot[0][0], T_rot[0][1], T_rot[0][2], trans_x],
             [T_rot[1][0], T_rot[1][1], T_rot[1][2], trans_y],
             [T_rot[2][0], T_rot[2][1], T_rot[2][2], trans_z],
             [0, 0, 0, 1]]
    
    start = timer()
    a=inverse_coordinates(T_des)
    end = timer()
    print "time normal:",
    print(end-start)

    #print(find_Dtheta_and_t(inverse_coordinates(T_des),np.array([0,0,0,0,0,0])))
    #print(find_theta(inverse_coordinates(T_des),np.array([3,-3,2,-3,3,-3])))

