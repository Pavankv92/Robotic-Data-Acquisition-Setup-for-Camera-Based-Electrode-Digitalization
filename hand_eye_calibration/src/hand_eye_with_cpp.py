#!/usr/bin/env python

import sys
import time

import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from hand_eye_calibration.msg import float_array
from sensor_msgs.msg import Image, JointState

from pose_estimation import transform_from_image

from solve_handEyeCalibration import solve
from evaluate_handeye import Evaluate_handeye
from helper_functions import ros2np


sys.path.append("/home/Vishwanath/catkin_ws/src/project_arbeit/robot_move/src")
from robot_move import MoveTheRobot

class HandEye(object):
    def __init__(self):
        self.posRobot = []
        self.posImage = []
        self.img = Image()
        self.bridge = CvBridge()
        self.robotSpeed = 0
        self.robot = MoveTheRobot()
        self.joint_list = np.load("/home/Vishwanath/catkin_ws/src/project_arbeit/Data/ur5_pose/UR5_np_pose.npy")
        #rospy.init_node('hand_eye_calibration_auto', anonymous=False)
        rospy.Subscriber('/rgb/image_raw', Image, self.update_img)
        rospy.Subscriber('joint_states', JointState, self.update_speed)
        #self.pub = rospy.Publisher('target_position', float_array, queue_size=3)
    
    def update_img(self, msg):
        self.img = msg
    
    def update_speed(self, msg):
        speed = msg.velocity
        self.robotSpeed = np.amax(speed)

    def get_posImage(self):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.img)
        except CvBridgeError as e:
            print(e)
            return

        try:
            tmp_pos = transform_from_image(cv_image)
            rospy.loginfo("printing pose estimation")
            rospy.loginfo(tmp_pos)
        except ValueError as e:
            print(e)
            return

        #rospy.loginfo(str(tmp_pos[0,3]), str(tmp_pos[1,3]), str(tmp_pos[2,3]))
        self.posImage.append(tmp_pos)

    def moveRobot(self, i):
        try:
            
            while not self.robotSpeed == 0.0:     # wait for movement to finish
                time.sleep(0.5)

            time.sleep(0.25)     # we aren't in a hurry, let everyone catch up
            robot_t_ee = ros2np(self.robot.move_group.get_current_pose())
            rospy.loginfo("printing robot_ee")
            rospy.loginfo(robot_t_ee)
            self.posRobot.append(robot_t_ee)
            self.get_posImage()
            rospy.loginfo("pose was saved")
        except KeyboardInterrupt:
            rospy.signal_shutdown("quit")
            exit()

    def run(self):
        for i in range(40):
            self.moveRobot(i)
            rospy.loginfo(i)
        np.save("/home/Vishwanath/catkin_ws/src/project_arbeit/Data/hand_eye/ur3/cam", self.posImage)
        np.save("/home/Vishwanath/catkin_ws/src/project_arbeit/Data/hand_eye/ur3/rob", self.posRobot)
    



if __name__ == "__main__":
    calib = HandEye()
    calib.run()
    
    
        