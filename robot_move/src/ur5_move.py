#!/usr/bin/env python


import rospy
import numpy as np
from numpy import linalg as la
import time
from robot_move import MoveTheRobot
from helper_functions import ros2np
from hand_eye_calibration.msg import float_array
from forward_kinematics import fwd_kine
from sensor_msgs.msg import JointState
import tf


class Move_UR5_record(object):


  def __init__(self):
    self.robot = MoveTheRobot()
    #rospy.init_node('ur5_move', anonymous=True)
    rospy.Subscriber('/joint_states', JointState, self.callback, queue_size=1)
    # variables
    self.robot_t_ee_first = ros2np(self.robot.move_group.get_current_pose()) # just for calculating ee_t_csys
    self.robot_t_acam = np.load("/home/fasod/fasod_ws/src/project_arbeit/Data/hand_eye/robot_t_acam.npy")
    self.acam_t_csys = np.load("/home/fasod/fasod_ws/src/project_arbeit/Data/head_coordinate/acam_t_csys.npy")
    self.robot_t_kcam = np.load("/home/fasod/fasod_ws/src/project_arbeit/Data/hand_eye/cam.npy")
    self.ee_t_csys = self.get_ee_t_csys() 
  
  
  def callback(self,msg):
        self.joint_state_tmp = msg.position
        self.robot_t_ee = fwd_kine(self.joint_state_tmp)
        #rospy.loginfo(self.robot_t_ee)

  # calculate only one time ee_t_csys is constant value
  def get_ee_t_csys(self):
    """method calculates the pose of the head coordinate system (csys) with respect to ee of robot.
        this has to be done only one time as transformation between csys and ee is constant!
    
    Returns:
        np.array -- 4x4 matrix
    """
    self.ee_t_csys = np.matmul(la.inv(self.robot_t_ee_first),np.matmul(self.robot_t_acam, self.acam_t_csys))
    
    return self.ee_t_csys

  def get_kcam_t_csys(self, robot_t_kcam, robot_t_ee,  ee_t_csys):
    self.kcam_t_csys = np.matmul(la.inv(self.robot_t_kcam),np.matmul(robot_t_ee, ee_t_csys))
    return self.kcam_t_csys  
  
  def deg2rad(self,deg):
     return deg/180.*3.1415

  def move_to_jointAngles(self):
    current_joint_angles = self.robot.move_group.get_current_joint_values()
    joint_pose = current_joint_angles
    print(joint_pose)
    # lean towards camera
    #joint_pose[3] = -1.25
    
    #self.robot.moveTo_joint(joint_pose)
    #go to wrist_3 0 position
    #joint_pose[5] = -3.1415
    joint_pose = [self.deg2rad(-30),self.deg2rad(-135),self.deg2rad(-90),self.deg2rad(-45),self.deg2rad(-90),self.deg2rad(-180)]
    print "goal: ", joint_pose
    self.robot.moveTo_joint(joint_pose)
    #then move to start the loop
    response = raw_input('press q to quit')
    while not response == 'q':
      #current_joint_angles = self.robot.move_group.get_current_joint_values()
      if response == 'n':
	      if joint_pose[5] >= 0 :
		joint_pose[5] = -3.1415
	      elif joint_pose[5] < 0:
		joint_pose[5] = 3.1415
	      else :
		joint_pose[5] = 3.1415
      if response == 'm':
      	      if joint_pose[3] >= self.deg2rad(-46) :
		joint_pose[3] = self.deg2rad(-90)
	      elif joint_pose[3] < self.deg2rad(-46):
		joint_pose[3] = self.deg2rad(-45)
	      else :
		joint_pose[3] = self.deg2rad(-45)
      
      print "goal: ", joint_pose
      self.robot.moveTo_joint(joint_pose)
      response = raw_input('press q to quit')

  
  def move_to_headPose(self, pose):
    """method to move the robot along with phantom. poses are given wrt to phantom. 
        phantom poses are coverted to ee pose and robot is moved to this ee pose
    
    Arguments:
        pose {np.array} -- 4x4 matrix
    """
    robot_t_ee = np.matmul(pose,la.inv(self.ee_t_csys))
    print("moving the robot")
    try:
      isMoved = self.robot.moveTo_pose(robot_t_ee)
      if isMoved:
        print("Robot moved to given pose with in tolerance")
      else:
        print("Robot moved failed")
    except ValueError as e:
      print(e)
    
  def kcam_record(self):
    pass

  # move_handeye method is not used during handeye. moving part is included in handeye itself.
  def move_handeye(self, msg):
    try:
      joint_goal = msg.data
      pose_goal = np.reshape(joint_goal, (4,4))
      isMoved = self.robot.moveTo_pose(pose_goal)
      if isMoved:
        print("robot moved to pose goal sucessfully with the tolerance of 0.05")
      else:
        print("robot did not move to the pose goal with in the tolerance of 0.05")
    
    except ValueError as e:
      print (e)

if __name__ == "__main__":
  ur5 = Move_UR5_record()

  
  '''
  robot_t_ee_pose = (ur5.robot.move_group.get_current_pose())
  print(robot_t_ee_pose)
  robot_t_ee_np_pose = ros2np(ur5.robot.move_group.get_current_pose())
  print(robot_t_ee_np_pose)
  '''
  
  
  ur5.move_to_jointAngles()
  #robot_t_ee_np_pose = ros2np(ur5.robot.move_group.get_current_pose())
  #print('from moveit')
  #print(robot_t_ee_np_pose)
  rospy.spin()

  
                        
