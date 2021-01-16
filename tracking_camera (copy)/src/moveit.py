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
import numpy as np
import time

class Move_it(object):
  def __init__(self):
    super(Move_it, self).__init__()
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
    
  

if __name__ == "__main__":
  robot = Move_it()
  
  
 


  