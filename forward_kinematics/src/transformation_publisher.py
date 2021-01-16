#!/usr/bin/env python

import sys
import rospy
from forward_kinematics.srv import *
from math import pi
from std_msgs.msg import String

def transformation_client(theta):
    rospy.wait_for_service('transformation_server')
    try:
        transformation_server = rospy.ServiceProxy('transformation_server', DhToPose)
        req = DhToPoseRequest(theta)
        resp1 = transformation_server(req)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: $s"%e


def test():
    theta = [pi/2, -pi/2, pi/4, -pi/2, 0.0, 0.0]
    print transformation_client(theta)

if __name__ == "__main__":
    test()