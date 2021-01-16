#!/usr/bin/env python

import rospy
import numpy as np
from numpy import linalg as la
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from time import time
import scipy as sp
from scipy import signal



if __name__ == "__main__":
    

    robot_t_acam = np.array([
                                    [-0.983535,  0.0163488,  0.179979, -0.0669347],
                                    [-0.17461,   0.170792,  -0.969712,    1.81139],
                                    [-0.0465925,  -0.985171,  -0.165125,   0.665069],
                                            [0,         0,         0,         1]
                                                                                ])

    np.save("/home/fasod/fasod_ws/src/project_arbeit/Data/hand_eye/robot_t_acam.npy",robot_t_acam)
    print(np.load("/home/fasod/fasod_ws/src/project_arbeit/Data/hand_eye/robot_t_acam.npy"))

    robot_t_kcam = np.array([
                                    [-0.937068,  -0.019565,   0.348598,  -0.615534],
					                [-0.349144,  0.0484491,  -0.935816,    1.61184],
					                [0.00141995,  -0.998634, -0.0522311,   0.592902],
						            [0,          0,          0,         1]
                                                                                ])

    np.save("/home/fasod/fasod_ws/src/project_arbeit/Data/hand_eye/robot_t_kcam.npy",robot_t_kcam)
    print(np.load("/home/fasod/fasod_ws/src/project_arbeit/Data/hand_eye/robot_t_kcam.npy"))



