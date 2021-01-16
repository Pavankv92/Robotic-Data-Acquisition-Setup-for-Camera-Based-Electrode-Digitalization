cmake_minimum_required(VERSION 2.8.3)
project(forward_kinematics)
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg geometry_msgs)

## Declare ROS messages and services

add_service_files(FILES DhToPose.srv DhToPoseMatrix.srv inverse_kinematics.srv)
add_message_files(FILES float_array.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Declare a catkin package
catkin_package()
