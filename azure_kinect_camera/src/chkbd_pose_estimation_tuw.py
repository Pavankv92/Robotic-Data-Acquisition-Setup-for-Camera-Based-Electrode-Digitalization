#!/usr/bin/env python

from __future__ import print_function
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseStamped
from rt_msgs.msg import TransformRTStampedWithHeader
from helper_functions import cv2ros, np2ros_RTStampedWithHeader
import numpy as np
import cv2
import rospy
from numpy import linalg as la
import time


class poseEstimation(object):
    def __init__(self):
        self.pose = TransformRTStampedWithHeader()
        rospy.init_node('chkbd_pose_estimation', anonymous=True)
        self.pub = rospy.Publisher('kin_chkbd_rt_tf', TransformRTStampedWithHeader, queue_size=1)
        rospy.Subscriber('/pose', PoseStamped, self.transform_from_image, queue_size = 1)
        rospy.spin()

    def transform_from_image(self, msg):
        self.pose.transform.translation.x = msg.pose.position.x
        self.pose.transform.translation.y = msg.pose.position.y
        self.pose.transform.translation.z = msg.pose.position.z
        self.pose.transform.rotation.x = msg.pose.orientation.x
        self.pose.transform.rotation.y = msg.pose.orientation.y
        self.pose.transform.rotation.z = msg.pose.orientation.z
        self.pose.transform.rotation.w = msg.pose.orientation.w
        self.pub.publish(self.pose)

        


if __name__ == "__main__":
   pose=poseEstimation()
