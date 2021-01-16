#!/usr/bin/env python

import rospy
from forward_kinematics.srv import inverse_kinematics
from forward_kinematics.msg import float_array
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import numpy as np
import time
import sys


def inverse_client(pose, joints, speed):
    try:
        rospy.wait_for_service('inverse_server', timeout=3)
        inverse_server = rospy.ServiceProxy('inverse_server', inverse_kinematics)
    except rospy.ROSException:
        rospy.logerr("No server")
        exit()
    
    try:
        resp = inverse_server(pose, joints, speed)
        return resp
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: $s"%e)

class Move(object):
    def __init__(self, topic=0, speed=0):
        if speed:
            raise NotImplementedError("this is not working in a save way at this moment")
        self.joints = np.zeros(6)
        self.pose = np.zeros(16)
        self.response = 0
        self.speed = speed
        self.tStart = 0
        rospy.init_node('move_script')

        self.pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=2)
        rospy.Subscriber('joint_states', JointState, self.update_joint_position)
        if topic == 1:
            rospy.Subscriber('target_position', float_array, self.update_target_pose)
        rospy.loginfo("Init complete")
    
    def update_joint_position(self, msg):
        self.joints = msg.position
    
    def update_target_pose(self, msg):
        self.tStart = rospy.get_time()
        if len(msg.data)!=16:
            rospy.logerr("Passed array is not of the right shape")
            ValueError("Passed array is not of the right shape")
        self.pose = msg.data
        rospy.loginfo("got a new target")
        self.run()

    def debug(self):
        goal = np.array([[1, 0, 0, 0.1], [0, 1, 0, -0.1], [0, 0, 1, 0.4], [0, 0, 0, 1]])
        self.pose = np.reshape(goal, (16))
    
    def get_inverse(self):
        rospy.logdebug("Calling the transformation server")
        self.response = inverse_client(self.pose, self.joints, self.speed)

    def run(self):
        self.get_inverse()
        rospy.loginfo("starting the movement")
        if self.speed:
            self.pub.publish("speedj([%f, %f, %f, %f, %f, %f],0.25,%f)"%(self.response.theta[0], self.response.theta[1], self.response.theta[2], self.response.theta[3], self.response.theta[4], self.response.theta[5], self.response.time))
            rospy.loginfo("speedj([%f, %f, %f, %f, %f, %f],0.25,%f)"%(self.response.theta[0], self.response.theta[1], self.response.theta[2], self.response.theta[3], self.response.theta[4], self.response.theta[5], self.response.time))
        else:
            if not self.response.theta[self.response.theta > 20] > 0:
                self.pub.publish("movej([%f, %f, %f, %f, %f, %f],0.5,0.25)"%(self.response.theta[0], self.response.theta[1], self.response.theta[2], self.response.theta[3], self.response.theta[4], self.response.theta[5]))
                rospy.loginfo("movej([%f, %f, %f, %f, %f, %f],0.5,0.25)"%(self.response.theta[0], self.response.theta[1], self.response.theta[2], self.response.theta[3], self.response.theta[4], self.response.theta[5]))
            else:
                rospy.logwarn("can't reach this position, ignoring it")
        end = rospy.get_time()
        rospy.loginfo("Time for processing " + str(end-self.tStart))


if __name__ == "__main__":
    print "##### warning, the robot will move #####"
    myargv = rospy.myargv(argv=sys.argv)
    if len(myargv) < 2:
        print "listening for target pose on topic 'target_position'"
        mov = Move(topic=1, speed=0)
        rospy.loginfo("waiting for a target")
        rospy.spin()
    elif myargv[1] == "--debug":
        print "Using a given target pose"
        mov = Move(topic=0, speed=0)
        mov.debug()
        rospy.loginfo("waiting a moment for subscribers to get ready")
        time.sleep(3)
        mov.run()
        time.sleep(20) # TODO better version, don't return untill move is finished 
    else:
        raise ValueError("unknown argument, only '--debug' or none is supported")
