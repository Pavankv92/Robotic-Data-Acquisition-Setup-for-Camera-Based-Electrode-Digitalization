#!/usr/bin/env python
"""Class provides methods to estimate the checkerboard pose on the fly using opencv
***no need to undistort the images*** 

Raises:
    ValueError: when the 2D image points weren't found on the checkerboard
                *** it is important to have the emough illumination on the checkerboard***

Returns:
    array -- checkerboard pose in RTSTampedWuthHeader format, check the msg type for more info.
"""

from __future__ import print_function
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, JointState
from rt_msgs.msg import TransformRTStampedWithHeader
from helper_functions import cv2ros, np2ros_RTStampedWithHeader
import numpy as np
import cv2
import rospy
from numpy import linalg as la
import time


class poseEstimation(object):
    def __init__(self): 

        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-16)
        np.set_printoptions(precision=16)
        self.robotSpeed = 0.0

        self.cam_mtx= np.array([[9.68727469e+02,   0.00000000e+00,   1.02687381e+03],
                            [0.00000000e+00,   9.68978580e+02,  7.74022724e+02],
                            [0, 0, 1]], np.float32)

        self.cam_dist= np.array([0.07588704,  0.04285212, -0.00064595,  0.00132813, -0.25794438], np.float32)
        self.objp = self.createObjectpoints(5, 8, 0.04)
        self.pose = TransformRTStampedWithHeader()
        rospy.init_node('chkbd_pose_estimation', anonymous=True)
        self.pub = rospy.Publisher('kin_chkbd_rt_tf', TransformRTStampedWithHeader, queue_size=1)
        rospy.Subscriber('/rgb/image_raw', Image, self.transform_from_image, queue_size = 1)
        #rospy.Subscriber('joint_states', JointState, self.update_speed, queue_size = 3)
        rospy.spin()

    def createObjectpoints(self, height, width, size):
        objp=np.zeros((height*width, 3), np.float32)
        objp[:, :2]=np.indices((width, height)).T.reshape(-1, 2)
        objp *= size
        return np.around(objp, 3)

    def cv2_pose(self, rvecs, tvecs):
        rot3X3=cv2.Rodrigues(rvecs)[0]
        transformation = np.array([[rot3X3[0, 0], rot3X3[0, 1], rot3X3[0, 2], tvecs[0]],
                                    [rot3X3[1, 0], rot3X3[1, 1],rot3X3[1, 2], tvecs[1]],
                                    [rot3X3[2, 0], rot3X3[2, 1],rot3X3[2, 2], tvecs[2]],
                                    [0, 0, 0, 1]], np.float32)
        return transformation
    """
    def update_speed(self, msg):
        speed=msg.velocity
        self.robotSpeed = np.amax(speed)
    """
    def transform_from_image(self, msg):
        bridge=CvBridge()
        """
        while not self.robotSpeed == 0.0:
            time.sleep(10) 
        """
        try:
            cv_image=bridge.imgmsg_to_cv2(msg)
            #cv2.imshow('image', cv_image)
            #cv2.waitKey(250)

        except CvBridgeError as e:
            print(e)
        
        gray_image=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        found, corners=cv2.findChessboardCorners(gray_image, (8, 5), None)

        if found:
            cv2.cornerSubPix(gray_image, corners, (11, 11),(-1, -1), self.criteria)
            # debug
            img = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
            cv2.drawChessboardCorners(img, (8,5), corners, found)
            cv2.imshow('image', img)
            cv2.waitKey(1)

            # solvePnP is used to get the position and orientation of the object
            found, rvecs, tvecs=cv2.solvePnP(self.objp, corners, self.cam_mtx, self.cam_dist)
            # self.pose = cv2ros(rvecs,tvec)
            self.pose_tmp=self.cv2_pose(rvecs, tvecs)
            print(self.pose_tmp)
            self.pose=np2ros_RTStampedWithHeader(self.pose_tmp)
            self.pub.publish(self.pose)

        else:
            print("pattern not found")
            raise ValueError("Could not find the pattern in the image")

        return print(self.pose)


if __name__ == "__main__":
   pose=poseEstimation()