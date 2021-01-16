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
    rospy.loginfo("start position")
    time.sleep(10)

    for i in range(1,161):
        if i < 41:
            target = np.array([[1, 0, 0, 0.1],[0, 1, 0, -0.1],[0, 0, 1, 0.4+0.005*i],[0, 0, 0, 1]])
            pub.publish(float_array(np.reshape(target, (16))))
            rospy.loginfo(i)
            time.sleep(0.25)
        elif i < 81:
            target = np.array([[1, 0, 0, 0.1],[0, 1, 0, -0.1+0.005*(i-40)],[0, 0, 1, 0.6],[0, 0, 0, 1]])
            pub.publish(float_array(np.reshape(target, (16))))
            rospy.loginfo(i)
            time.sleep(0.25)
        elif i < 121:
            target = np.array([[1, 0, 0, 0.1],[0, 1, 0, 0.1],[0, 0, 1, 0.6-0.005*(i-80)],[0, 0, 0, 1]])
            pub.publish(float_array(np.reshape(target, (16))))
            rospy.loginfo(i)
            time.sleep(0.25)
        elif i < 162:
            target = np.array([[1, 0, 0, 0.1],[0, 1, 0, 0.1-0.005*(i-120)],[0, 0, 1, 0.4],[0, 0, 0, 1]])
            pub.publish(float_array(np.reshape(target, (16))))
            rospy.loginfo(i)
            time.sleep(0.25)


if __name__ == "__main__":
    #time.sleep(2)
    talker()
    time.sleep(2)
