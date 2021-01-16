#!/usr/bin/env python

import rospy
from forward_kinematics.srv import *
from math import pi
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from forward_kinematics.msg import float_array
import numpy as np

def transformation_client(theta):
    try:
        rospy.wait_for_service('transformation_server', timeout=3)
        transformation_server = rospy.ServiceProxy('transformation_server', DhToPoseMatrix)
    except rospy.ROSException:
        rospy.logerr("No server")
        exit()
        
    try:
        resp = transformation_server(theta)
        return resp
    except rospy.ServiceException, e:
        rospy.logerr("Service call failed: $s"%e)

class Transform(object):
    def __init__(self):
        self.pos = np.zeros(16)
        self.count = 0
        rospy.init_node('pose_publisher')

        self.pub = rospy.Publisher('/forward_kinematics', float_array, queue_size=1) 
        rospy.Subscriber('joint_states', JointState, self.update_value)
        rospy.loginfo("Init complete, ready to publish. Subscribe to topic /forward_kinematics")

    
    def update_value(self, msg):
        if self.count % 25 == 0:
            rospy.logdebug("Calling the transformation server with %.2f, %.2f, %.2f,%.2f, %.2f, %.2f"%(msg.position))
            self.pos = transformation_client(msg.position).pose
        
        self.count += 1
        
    
    def run(self):
        r = rospy.Rate(10)
        count = 0
        while not rospy.is_shutdown():
            self.pub.publish(self.pos)
            #self.pub.publish([self.pos.px, self.pos.py, self.pos.pz, self.pos.rx, self.pos.ry, self.pos.rz])
            r.sleep()

if __name__ == "__main__":
    trans = Transform()
    trans.run()