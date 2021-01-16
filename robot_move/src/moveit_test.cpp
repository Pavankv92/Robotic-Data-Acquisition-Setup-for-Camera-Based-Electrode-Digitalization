/*
 Test code to move the UR3. This code moves the robot
 to 2 poses, but the value of x position does not match
 for either case. They seem to be less than the desired
 pose. 
*/
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

void attachDummyHead(
    moveit::planning_interface::MoveGroupInterface& move_group, 
    const moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    // Define a collision object ROS message
    moveit_msgs::CollisionObject dummy_head;
    dummy_head.header.frame_id = "wrist_3_link";
    dummy_head.id = "box1";    // id of the object used to identify it

    // Define a box to add to the world
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.1;
    primitive.dimensions[1] = 0.1;
    primitive.dimensions[2] = 0.1;

    // Define a pose for the box
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.1;
    box_pose.position.y = 0.1;
    box_pose.position.z = 0.1;

    dummy_head.primitives.push_back(primitive);
    dummy_head.primitive_poses.push_back(box_pose);
    dummy_head.operation = dummy_head.ADD;

    std::vector<moveit_msgs::CollisionObject> dummy_heads;
    dummy_heads.push_back(dummy_head);

    // Add collision object to the world
    ROS_INFO_NAMED("dummy_head", "Add a head into the world");
    planning_scene_interface.addCollisionObjects(dummy_heads);
    
    // Attach collision object to the robot
    ROS_INFO_NAMED("dummy_head", "Attach the head to the robot");
    move_group.attachObject(dummy_head.id);
}

void moveToTarget(double w, double x, double y, double z, 
                  moveit::planning_interface::MoveGroupInterface& move_group)
{
    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = w;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan; // visualizes the trajectory in rviz

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("ur_test", "Visualizing plan 1 (pose goal) %s", success ? "PASS" : "FAILED");

    ROS_INFO_NAMED("ur_test", "Visualizing plan 1 as trajectory line");

    move_group.move(); // moves the robot in gazebo
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ur3_test_interface");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group = 
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    ros::Publisher display_publisher = 
        node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", move_group.getEndEffectorLink().c_str());

    moveToTarget(1.0, 0.4, -0.2, 0.5, move_group);    // pose 1

    moveToTarget(1.0, 0.28, -0.3, 0.4, move_group);    // pose 2

    ros::shutdown();
    return 0;
}