#!/usr/bin/env python

import moveit_commander
import geometry_msgs
import rospy
import sys
import numpy as np
from sensor_msgs.msg import JointState
from helper_functions import ur2ros, ros2np
from robot_move import MoveTheRobot
import time


class generate_UR5_poses(object):
    def __init__(self):
        self.robot = MoveTheRobot()
    
    def move_pose(self):
        list = np.load("/home/Vishwanath/catkin_ws/src/project_arbeit/Data/ur5_pose/UR5_np_pose.npy")
        for i in range(len(list)):
            isMoved = self.robot.moveTo_pose(list[i])
            if isMoved:
                print('robot moved with in tolerance')
            time.sleep(5)
            

    def main_joint(self):
        ur5_pose_list = []
        list = np.load("/home/Vishwanath/catkin_ws/src/project_arbeit/Data/UR5_pose_joint.npy")
        for i in range(len(list)):
            self.robot.moveTo_joint(list[i])
            time.sleep(5)
            self.ros_pose = self.robot.move_group.get_current_pose()
            np_pose = ros2np(self.ros_pose)
            ur5_pose_list.append(np_pose)
            print(np_pose)
        np.save ("/home/Vishwanath/catkin_ws/src/project_arbeit/Data/ur5_pose/UR5_np_pose.npy", ur5_pose_list)

if __name__ == "__main__":
    create_pose = generate_UR5_poses()
    create_pose.move_pose()
    