#!/usr/bin/env python

"""This is the "makeshift" option to do hand eye calibration as cpp node for handeye was 
yielding incorrect results for UR5(its works perfectly for UR3) this node exploits the service 
offered by cpp node.

Arguements: 
TransformArray() -- camera_marker (checker board pose in kinnect frame: chkbdpose_T_kinectcam)
TransformArray() -- base_effector (endeffector in robot base frame ee_T_robot) N and M matrix respectively


Returns:
    arrays -- X and Y matrix
"""

from azure_kinect_camera.msg import TransformArray
import numpy as np 
import rospy
from helper_functions import np2ros_Transform
from azure_kinect_camera.srv import compute_effector_marker_quick_service 


def handeye_client(camera_marker, base_effector, prefix):
    rospy.wait_for_service('compute_effector_marker_quick')
    try:
        handeye_server = rospy.ServiceProxy('compute_effector_marker_quick', compute_effector_marker_quick_service)
        resp = handeye_server(camera_marker, base_effector, prefix)
        return resp
    except rospy.ROSException as error:
        print(error)
        exit()
    
    
if __name__ == "__main__":
    
    mat_rob = np.load("/home/Vishwanath/catkin_ws/src/project_arbeit/Data/hand_eye/ur3/rob.npy")
    mat_cam = np.load("/home/Vishwanath/catkin_ws/src/project_arbeit/Data/hand_eye/ur3/cam.npy")
    camera_marker = TransformArray()
    base_effector = TransformArray()
    prefix = 'kinect_handeye' 
    for i in range(len(mat_rob)):
    
        base_effector.transforms.append(np2ros_Transform(mat_rob[i])) 
    
    for i in range(len(mat_cam)):
    
        camera_marker.transforms.append(np2ros_Transform(mat_cam[i]))   
    
    handeye_client(camera_marker, base_effector, prefix)






