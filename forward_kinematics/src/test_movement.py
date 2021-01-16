#!/usr/bin/env python

import rospy
from forward_kinematics.msg import float_array

import numpy as np
import time

def publish_targets(pub):
    target = np.array([[1, 0, 0, 0.1], [0, 1, 0, -0.1], [0, 0, 1, 0.4], [0, 0, 0, 1]])
    pub.publish(float_array(np.reshape(target, (16))))
    rospy.loginfo("first target")
    time.sleep(8)

    target = [[0.051757623075540896, -0.776244897884779, -0.6283032762618855, 0.1], [0.048217218432293533, -0.6264703858436924, 0.7779524121098331, -0.05], [-0.9974949866040544, -0.0705600040299336, 0.005003751699777271, 0.6], [0, 0, 0, 1]]
    #target = np.array([[0.0707372016677029, 0.0, 0.9974949866040544, -0.1], [0.0, 1.0, 0.0, -0.3], [-0.9974949866040544, 0.0, 0.0707372016677029, 0.6], [0, 0, 0, 1]])
    pub.publish(float_array(np.reshape(target, (16))))
    rospy.loginfo("second target")
    time.sleep(8)


def talker():
    pub = rospy.Publisher('target_position', float_array, queue_size=3)
    rospy.init_node('test_move')
    time.sleep(5)
    rospy.loginfo("init completed")
    while not rospy.is_shutdown():
        publish_targets(pub)


if __name__ == "__main__":
    #time.sleep(2)
    talker()