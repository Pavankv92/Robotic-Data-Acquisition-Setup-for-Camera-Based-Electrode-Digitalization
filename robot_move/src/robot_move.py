#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from helper_functions import np2ros , ros2np
import numpy as np
import time

class MoveTheRobot(object):
  def __init__(self):
    super(MoveTheRobot, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('MoveTheRobot', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
    
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    #print ("============ Planning frame: %s" % planning_frame)
    
   

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    #print ("============ End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    #print ("============ Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    #print ("============ Printing robot state")
    #print (robot.get_current_state())
    #print ("")
    move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_max_acceleration_scaling_factor(0.1)
    # Misc variables
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
    self.tolerance = 0.01
    
  def all_close(self, goal, actual, tolerance):
    if type(goal) is list:
      for index in range(len(goal)):
        if abs(actual[index] - goal[index]) > tolerance:
          return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
      return self.all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
      return self.all_close(pose_to_list(goal), pose_to_list(actual.pose), tolerance)

    return True
  
  
  def moveTo_joint(self, joint_goal):
    current_joints = self.move_group.get_current_joint_values()
    print("publishing current joint angles before moving")
    print(current_joints)
    self.move_group.go(joint_goal, wait=True)
    self.move_group.stop()
    current_joints = self.move_group.get_current_joint_values()
    print("after moving")
    print(current_joints)
    return self.all_close(joint_goal, current_joints, self.tolerance)
    

  def moveTo_pose(self, pose):
    current_pose = self.move_group.get_current_pose()
    #current_pose_np = ros2np(current_pose)
    #print("printing target pose")
    #print(pose)
    pose_goal_ros = np2ros(pose)
    #print("printing ros pose")
    #print(pose_goal_ros)
    self.move_group.set_pose_target(pose_goal_ros)
    self.move_group.go(wait=True)
    self.move_group.stop()
    self.move_group.clear_pose_targets()
    time.sleep(1)
    #current_pose = self.move_group.get_current_pose()
    #current_pose_np = ros2np(current_pose)
    #print("printing pose after movement")
    #print(current_pose_np)
    return self.all_close(pose_goal_ros, current_pose, self.tolerance )

  def move_along_wayPoints(self, axis, direction, num_iteration, delta, scale=1):
    way_points = []
    wpose = self.move_group.get_current_pose().pose
    for i in range(num_iteration):
      if axis == 'x': 
        
        if direction == '+' :
          wpose.position.x += scale * delta  # First move up (z)  # and sideways (y)
        
        elif direction == '-':
          wpose.position.x -= scale * delta
        way_points.append(copy.deepcopy(wpose))
      
      elif axis == 'y':
        
        if direction == '+':
          wpose.position.y += scale * delta  # First move up (z)  # and sideways (y)
        
        elif direction == '-':
          wpose.position.y -= scale * delta
        way_points.append(copy.deepcopy(wpose))
      
      elif axis == 'z':
        
        if direction == '+':
          wpose.position.z += scale * delta  # First move up (z)  # and sideways (y)
        
        elif direction == '-':
          wpose.position.z -= scale * delta
        way_points.append(copy.deepcopy(wpose))

    (plan, fraction) = self.move_group.compute_cartesian_path(way_points, 0.01, 0.0)
    self.move_group.execute(plan, wait=True)

  def moveTo_position(self, xyz):
    self.move_group.set_position_target(self, xyz)
    self.move_group.go(wait=True)
    self.move_group.stop()
  '''
  def moveTo_rpy(self, rpy):
    self.move_group.set_rpy_target(self,rpy, self.eef_link)
    self.move_group.go(wait=True)
    self.move_group.stop()
  
  def moveTo_shift(self, axis, value):
    self.move_group.shift_pose_target(axis, value)
    self.move_group.go(wait=True)
    self.move_group.stop()
  '''

if __name__ == "__main__":
  robot = MoveTheRobot()
  
  
  """
 
  '
  list = np.load("/home/Vishwanath/catkin_ws/src/project_arbeit/Data/ur5_pose/UR5_pose.npy")
  for i in range(len(list)):
    joint_goal = list[i]
    isMoved = robot.moveTo_pose(joint_goal)
    print(isMoved)
    #time.sleep(10)


  # move to pose
  target = np.array([[ 0.40419128, -0.51453802,  0.75622751,  0.4864   ],
                    [ 0.66279207, -0.40500959, -0.62982053 , 0.126     ],
                    [ 0.630346 ,   0.75578956,  0.17733037,  0.521     ],
                    [ 0.0,          0.0,          0.0,          1.0    ]])
  
  isMoved = robot.moveTo_pose(target)
  print(isMoved)
  target = np.array([[ 0.67947106, -0.45468582,  0.57582974,  0.62178  ],
                    [ 0.57844793, -0.15082472, -0.80165447,  0.086     ],
                    [ 0.45135028,  0.87778853,  0.16053104,  0.5294    ],
                    [ 0.0,          0.0,          0.0,          1.0    ]])
  isMoved = robot.moveTo_pose(target)
  print(isMoved)
  
  target = np.array([[ 0.67890032, -0.45752413,  0.57425258,  0.3726   ],
                    [ 0.58064603, -0.1441355,  -0.80129592,  0.277968  ],
                    [ 0.4493824,   0.87743753,  0.16780593,  0.4974    ],
                    [ 0.0,          0.0,          0.0,          1.0    ]])
  isMoved = robot.moveTo_pose(target)
  print(isMoved)
"""
  


  