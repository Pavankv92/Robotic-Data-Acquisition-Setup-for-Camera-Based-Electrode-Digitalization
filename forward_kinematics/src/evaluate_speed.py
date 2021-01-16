#!/usr/bin/env python

import rospy
import time
from sensor_msgs.msg import JointState
from forward_kinematics.msg import float_array
from timeit import default_timer as timer
import numpy as np

class getTime(object):
    def __init__(self):
        rospy.init_node('time_logger')
        rospy.Subscriber('target_position', float_array, self.logTarget)
        rospy.Subscriber('joint_states', JointState, self.logMove)
        self.tTarget = 0
        self.tMove = 0
        self.start = True
        rospy.spin()

    def logTarget(self, msg):
        self.tTarget = rospy.get_time()

    def logMove(self, msg):
        speed = msg.velocity
        robotSpeed = np.amax(speed)
        #print(robotSpeed)
        if self.start:
            if robotSpeed > 1e-4: 
                self.tMove = rospy.get_time()
                self.printTime()
        else:
            if robotSpeed < 1e-4:
                self.start = True
    
    def printTime(self):
            rospy.loginfo(self.tMove-self.tTarget)
            self.start = False


if __name__ == "__main__":
    t = getTime()