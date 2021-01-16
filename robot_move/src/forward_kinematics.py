#!/usr/bin/env python


import numpy as np
from math import cos, sin, pi

# Selection of robot
ROBOT = 'UR5'

# DH Parameters

if ROBOT == 'UR3':

    # d in m
    d1 = 0.1519
    d4 = 0.11235
    d5 = 0.08535
    d6 = 0.0819

    # a in m
    a2 = -0.24365
    a3 = -0.21325

elif ROBOT == 'UR5':

    # d in m
    d1 = 0.089159
    d4 = 0.10915
    d5 = 0.09465
    d6 = 0.0823

    # a in m
    a2 = -0.425
    a3 = -0.39225

d = np.array([d1, 0, 0, d4, d5, d6])  # unit in m
a = np.array([0, a2, a3, 0, 0, 0])  # unit in m
alpha = np.array([(pi / 2), 0, 0, (pi / 2), -(pi / 2), 0])  # unit in radian

"""
    ****** Forward Kinematics ******
"""


def link_tf(i, theta):
    """
        Calculates the link transformation of frame i with respect 
        to previous frame.

        Args:
            i: Frame under consideration
            theta: List of joint angles (unit: radian)

        Returns:
            A_i: Link transformation matrix
            (type: np.mat)
    """
    R_zt = np.mat([[cos(theta[i - 1]), -sin(theta[i - 1]), 0, 0],
                   [sin(theta[i - 1]), cos(theta[i - 1]), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])

    T_zd = np.mat(np.identity(4))
    T_zd[2, 3] = d[i - 1]

    R_xa = np.mat([[1, 0, 0, 0],
                   [0, cos(alpha[i - 1]), -sin(alpha[i - 1]), 0],
                   [0, sin(alpha[i - 1]), cos(alpha[i - 1]), 0],
                   [0, 0, 0, 1]])

    T_xa = np.mat(np.identity(4))
    T_xa[0, 3] = a[i - 1]

    A_i = R_zt * T_zd * T_xa * R_xa

    return A_i


def fwd_kine(theta):
    """
        Calculates the 4x4 homogeneous transformation matrix.

        Args:
            theta: List of joint angles (unit: radian)

        Returns:
            T_06: Homogeneous transform from base to end effector
            (type: np.mat)
    """
    A_1 = link_tf(1, theta)
    A_2 = link_tf(2, theta)
    A_3 = link_tf(3, theta)
    A_4 = link_tf(4, theta)
    A_5 = link_tf(5, theta)
    A_6 = link_tf(6, theta)

    T_06 = A_1 * A_2 * A_3 * A_4 * A_5 * A_6

    return np.around(T_06, decimals=15)