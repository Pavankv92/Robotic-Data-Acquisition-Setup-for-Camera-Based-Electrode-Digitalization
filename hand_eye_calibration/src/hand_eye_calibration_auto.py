#!/usr/bin/env python
""" this node offers methods to do the hand eye calibration while moving the robot in joint space.
    set of good joint angles (40) have been collected for **UR5**  
"""

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

sys.path.append("/home/fasod/fasod_ws/src/project_arbeit/robot_move/src")
from robot_move import MoveTheRobot

class HandEye(object):

    """ Loads the joint angles to move the robot
        subscribes to /rgb/image_raw to estimate the chkbd pose
        subscribes to the joint_states so that image is captured while robot is not moving
    """
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
    
    def update_img(self, msg):
        self.img = msg
    
    def update_speed(self, msg):
        speed = msg.velocity
        self.robotSpeed = np.amax(speed)

    def get_posImage(self):

        """ saves the 4x4 homogeneous pose of the chkbd to a list 
            *** make sure there is enough illumination on tye chkbd***    
        """
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
        self.posImage.append(tmp_pos)

    def moveRobot(self, i):
        
        """ moves the robot in joint space and after the movement, captures the image and estimates the 
            chkbd pose and saves the chkbd pose along with endeffector pose to a list.
        """
        try:
            target = self.joint_list[i] 
            time.sleep(1)
            isMoved = self.robot.moveTo_pose(target)
            time.sleep(2)
            rospy.loginfo(isMoved)
            rospy.loginfo("driving to new position")
            time.sleep(0.5)
            while not self.robotSpeed == 0.0:     # wait for movement to finish
                time.sleep(0.5)

            time.sleep(0.25)     # we aren't in a hurry, let everyone catch up
            print('robot moved!')
            self.get_posImage()
            self.posRobot.append(target)
            rospy.loginfo("pose was saved")
        except KeyboardInterrupt:
            rospy.signal_shutdown("quit")
            exit()

    def hand_eye(self):

        """ calculates the solution for handeye calibration and saves to data folder
        """
        (X,Y) = solve(self.posRobot, self.posImage, len(self.joint_list))
        rospy.loginfo(X)
        rospy.loginfo(Y)
<<<<<<< HEAD
        np.savetxt("/home/fasod/fasod_ws/src/project_arbeit/Data/hand_eye/hand_X.txt", X)
        np.savetxt("/home/fasod/fasod_ws/src/project_arbeit/Data/hand_eye/hand_Y.txt", Y)
        np.save("/home/fasod/fasod_ws/src/project_arbeit/Data/hand_eye/matx", X)
        np.save("/home/fasod/fasod_ws/src/project_arbeit/Data/hand_eye/maty", Y)
=======
        #np.savetxt("/home/Vishwanath/catkin_ws/src/project_arbeit/Data/hand_eye/hand_X.txt", X)
        #np.savetxt("/home/Vishwanath/catkin_ws/src/project_arbeit/Data/hand_eye/hand_Y.txt", Y)
        np.save("/home/Vishwanath/catkin_ws/src/project_arbeit/Data/hand_eye/matx", X)
        np.save("/home/Vishwanath/catkin_ws/src/project_arbeit/Data/hand_eye/maty", Y)
>>>>>>> 797c049eca1893b0b3cd172cdbc9b9f179f68b8e

    def run(self):

        """ main method to connect all other methods.
            evaluates the obatined handeye solution
        """
        evaluate = Evaluate_handeye()
        for i in range(len(self.joint_list)):
            self.moveRobot(i)
            rospy.loginfo(i)
        np.save("/home/fasod/fasod_ws/src/project_arbeit/Data/hand_eye/cam", self.posImage)
        np.save("/home/fasod/fasod_ws/src/project_arbeit/Data/hand_eye/rob", self.posRobot)
        self.hand_eye()
        evaluate.main()



if __name__ == "__main__":
    calib = HandEye()
<<<<<<< HEAD
    #rospy.logwarn("the robot will move in 3 seconds, stay away and near a stop button")
    #time.sleep(3)
    #calib.run()
    
=======
    
    
>>>>>>> 797c049eca1893b0b3cd172cdbc9b9f179f68b8e
