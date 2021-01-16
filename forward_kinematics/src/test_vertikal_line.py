#!/usr/bin/env python

import rospy
from forward_kinematics.msg import float_array

import numpy as np
import time

def talker():
    pub = rospy.Publisher('target_position', float_array, queue_size=3)
    rospy.init_node('test_move')
    time.sleep(10)
    rospy.loginfo("init completed")
    target = np.array([[1, 0, 0, 0.1], [0, 1, 0, -0.1], [0, 0, 1, 0.4], [0, 0, 0, 1]])
    pub.publish(float_array(np.reshape(target, (16))))
    rospy.loginfo("first target")
    time.sleep(2)
    target = np.array([[1, 0, 0, 0.1], [0, 1, 0, -0.1], [0, 0, 1, 0.5], [0, 0, 0, 1]])
    pub.publish(float_array(np.reshape(target, (16))))
    rospy.loginfo("second target")
    time.sleep(2)
    target = np.array([[1, 0, 0, 0.1], [0, 1, 0, -0.1], [0, 0, 1, 0.6], [0, 0, 0, 1]])
    pub.publish(float_array(np.reshape(target, (16))))
    rospy.loginfo("third target")
    time.sleep(2)
    target = np.array([[1, 0, 0, 0.1], [0, 1, 0, 0], [0, 0, 1, 0.6], [0, 0, 0, 1]])
    pub.publish(float_array(np.reshape(target, (16))))
    rospy.loginfo("fourth target")
    time.sleep(2)
    target = np.array([[1, 0, 0, 0.1], [0, 1, 0, 0.1], [0, 0, 1, 0.6], [0, 0, 0, 1]])
    pub.publish(float_array(np.reshape(target, (16))))
    rospy.loginfo("fifth target")
    time.sleep(2)
    target = np.array([[1, 0, 0, 0.1], [0, 1, 0, 0.1], [0, 0, 1, 0.5], [0, 0, 0, 1]])
    pub.publish(float_array(np.reshape(target, (16))))
    rospy.loginfo("sixth target")
    time.sleep(2)
    target = np.array([[1, 0, 0, 0.1], [0, 1, 0, 0.1], [0, 0, 1, 0.4], [0, 0, 0, 1]])
    pub.publish(float_array(np.reshape(target, (16))))
    rospy.loginfo("seventh target")
    time.sleep(2)

if __name__ == "__main__":
    #time.sleep(2)
    talker()
    time.sleep(2)
