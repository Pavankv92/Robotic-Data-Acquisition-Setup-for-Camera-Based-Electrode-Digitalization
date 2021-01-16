#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import numpy as np
import time


class recordUR5Pose(object):

    def __init__(self):
        self.joint_list = []
        self.joint_state_tmp = JointState
        self. count = 0
        rospy.Subscriber('/joint_states', JointState, self.callback, queue_size=1)
        
    
    def callback(self,msg):
        self.joint_state = msg.position
        self.joint_state_tmp = self.joint_state 
        #rospy.loginfo(self.joint_state_tmp)
    
    def run(self):
        time.sleep(30)
        print("appending now")
        print(self.joint_state_tmp)
        self.joint_list.append(self.joint_state_tmp)


if __name__ == "__main__":
    
    rospy.init_node('python_test',log_level=rospy.INFO)
    record = recordUR5Pose()
    for i in range(50):
        print("pose", i, "\n")
        record.run()
    print(record.joint_list)
    rospy.spin()

    
        
        



