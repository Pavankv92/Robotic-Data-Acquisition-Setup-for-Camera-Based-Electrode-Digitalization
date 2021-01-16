#!/usr/bin/env python

from get_transformation import *
from forward_kinematics.srv import *
import rospy
import numpy as np


def hadle_request(req):
    rospy.loginfo("[Inverse Server]: calculating the joint angles")
    pose_array = np.asarray(req.pose)
    pose = np.reshape(pose_array, (4,4))
    theta_mult = inverse_coordinates(pose)
    if req.speed:
        rospy.loginfo("using speed")
        theta_real, time = find_Dtheta_and_t(theta_mult, req.joint_states)
        return inverse_kinematicsResponse(theta_real, time)
    else:
        theta_real = find_theta(theta_mult, req.joint_states)
        return inverse_kinematicsResponse(theta_real, 0.)

def move_server():
    rospy.init_node('inverse_server', anonymous=False)
    s = rospy.Service('inverse_server', inverse_kinematics, hadle_request)
    rospy.loginfo("[Inverse Server]: Server is ready")
    rospy.spin()

if __name__ == "__main__":
    move_server()