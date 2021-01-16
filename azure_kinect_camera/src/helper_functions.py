#!/usr/bin/env python

from math import sqrt
import tf.transformations as tf
from geometry_msgs.msg import Pose, PoseStamped
from rt_msgs.msg import TransformRTStampedWithHeader
from geometry_msgs.msg import Transform
import numpy as np

def np2ros_Transform(np_pose):
    """Transform pose from np.array format to ROS Pose format.

    Args:
        np_pose: A pose in np.array format (type: np.array)

    Returns:
        An HTM (type: Pose).
    """

    # ROS pose
    ros_pose = Transform()
    # ROS position
    ros_pose.translation.x  = np_pose[0, 3]
    ros_pose.translation.y  = np_pose[1, 3]
    ros_pose.translation.z  = np_pose[2, 3]

    # ROS orientation
    np_q = tf.quaternion_from_matrix(np_pose)
    ros_pose.rotation.x = np_q[0]
    ros_pose.rotation.y = np_q[1]
    ros_pose.rotation.z = np_q[2]
    ros_pose.rotation.w = np_q[3]

    return ros_pose


def np2ros_RTStampedWithHeader(np_pose):
    """Transform pose from np.array format to ROS Pose format.

    Args:
        np_pose: A pose in np.array format (type: np.array)

    Returns:
        An HTM (type: Pose).
    """

    # ROS pose
    ros_pose = TransformRTStampedWithHeader()
    # ROS position
    ros_pose.transform.translation.x  = np_pose[0, 3]
    ros_pose.transform.translation.y  = np_pose[1, 3]
    ros_pose.transform.translation.z  = np_pose[2, 3]

    # ROS orientation
    np_q = tf.quaternion_from_matrix(np_pose)
    ros_pose.transform.rotation.x = np_q[0]
    ros_pose.transform.rotation.y = np_q[1]
    ros_pose.transform.rotation.z = np_q[2]
    ros_pose.transform.rotation.w = np_q[3]

    return ros_pose

def cv2ros(rvecs,tvecs):
    """Transform pose from openCV solvePNP output format to ROS TransformRTStampedWithHeader format.

    Args:
        rvecs: in format [rx, ry, rz] units : radians
        tvecs: in format [tx, ty, tz] units : meters
        (type: list)

    Returns:
        An HTM (type: TransformRTStampedWithHeader)
    """

    # ROS pose
    ros_pose = TransformRTStampedWithHeader()

    # ROS position
    ros_pose.transform.translation.x = tvecs[0]
    ros_pose.transform.translation.y = tvecs[1]
    ros_pose.transform.translation.z = tvecs[2]

    # Ros orientation
    angle = sqrt(rvecs[0] ** 2 + rvecs[1] ** 2 + rvecs[2] ** 2)
    direction = [i / angle for i in rvecs[0:3]]
    np_T = tf.rotation_matrix(angle, direction)
    np_q = tf.quaternion_from_matrix(np_T)
    ros_pose.transform.rotation.x = np_q[0]
    ros_pose.transform.rotation.y = np_q[1]
    ros_pose.transform.rotation.z = np_q[2]
    ros_pose.transform.rotation.w = np_q[3]

    return ros_pose

def ur2ros(ur_pose):
    """Transform pose from UR format to ROS Pose format.

    Args:
        ur_pose: A pose in UR format [px, py, pz, rx, ry, rz] units : meters, radians
        (type: list)

    Returns:
        An HTM (type: Pose).
    """

    # ROS pose
    ros_pose = PoseStamped()

    # ROS position
    ros_pose.pose.position.x = ur_pose[0]
    ros_pose.pose.position.y = ur_pose[1]
    ros_pose.pose.position.z = ur_pose[2]

    # Ros orientation
    angle = sqrt(ur_pose[3] ** 2 + ur_pose[4] ** 2 + ur_pose[5] ** 2)
    direction = [i / angle for i in ur_pose[3:6]]
    np_T = tf.rotation_matrix(angle, direction)
    np_q = tf.quaternion_from_matrix(np_T)
    ros_pose.pose.orientation.x = np_q[0]
    ros_pose.pose.orientation.y = np_q[1]
    ros_pose.pose.orientation.z = np_q[2]
    ros_pose.pose.orientation.w = np_q[3]

    return ros_pose


def ros2np(ros_pose):
    """Transform pose from ROS Pose format to np.array format.

    Args:
        ros_pose: A pose in ROS Pose format (type: PoseStamped)

    Returns:
        An HTM (type: np.array).
    """

    # orientation
    np_pose = tf.quaternion_matrix([ros_pose.pose.orientation.x, ros_pose.pose.orientation.y,
                                    ros_pose.pose.orientation.z, ros_pose.pose.orientation.w])

    # position
    np_pose[0][3] = ros_pose.pose.position.x
    np_pose[1][3] = ros_pose.pose.position.y
    np_pose[2][3] = ros_pose.pose.position.z

    return np_pose


def np2ros(np_pose):
    """Transform pose from np.array format to ROS Pose format.

    Args:
        np_pose: A pose in np.array format (type: np.array)

    Returns:
        An HTM (type: Pose).
    """

    # ROS pose
    ros_pose = Pose()
    # ROS position
    ros_pose.position.x = np_pose[0, 3]
    ros_pose.position.y = np_pose[1, 3]
    ros_pose.position.z = np_pose[2, 3]

    # ROS orientation
    np_q = tf.quaternion_from_matrix(np_pose)
    ros_pose.orientation.x = np_q[0]
    ros_pose.orientation.y = np_q[1]
    ros_pose.orientation.z = np_q[2]
    ros_pose.orientation.w = np_q[3]

    return ros_pose


if __name__ == "__main__":
    rvecs = [1,2,3]
    tvecs = [3,4,5]
    ros_pose = cv2ros(rvecs,tvecs)
    print(ros_pose)

"""
ur_pose_1 = [0.4864, 0.126, .521, 1.5168, 0.1378, 1.2888]
pose_goal_ros = ur2ros(ur_pose_1)
pose_goal_np =  ros2np(pose_goal_ros)
print(pose_goal_np)
print('\n')
ur_pose_2 = [.62178, 0.0860, .5294, 1.4679, 0.1088, 0.90300]
pose_goal_ros = ur2ros(ur_pose_2)
pose_goal_np =  ros2np(pose_goal_ros)
print(pose_goal_np)
print('\n')
ur_pose_3 = [.3726, 0.277968, .4974, 1.46, 0.1086, 0.9029]
pose_goal_ros = ur2ros(ur_pose_3)
pose_goal_np =  ros2np(pose_goal_ros)
print(pose_goal_np)
print('\n')
"""
target = np.array([[ 0.40419128, -0.51453802,  0.75622751,  0.4864   ],
                    [ 0.66279207, -0.40500959, -0.62982053 , 0.126     ],
                    [ 0.630346 ,   0.75578956,  0.17733037,  0.521     ],
                    [ 0.0,          0.0,          0.0,          1.0    ]])
print(np2ros_RTStampedWithHeader(target))
print('printing ros')
print(np2ros(target))
target = np.array([[ 0.67947106, -0.45468582,  0.57582974,  0.62178  ],
                    [ 0.57844793, -0.15082472, -0.80165447,  0.086     ],
                    [ 0.45135028,  0.87778853,  0.16053104,  0.5294    ],
                    [ 0.0,          0.0,          0.0,          1.0    ]])
print(np2ros_RTStampedWithHeader(target)) 
print('printing ros')
print(np2ros(target))
target = np.array([[ 0.67890032, -0.45752413,  0.57425258,  0.3726   ],
                    [ 0.58064603, -0.1441355,  -0.80129592,  0.277968  ],
                    [ 0.4493824,   0.87743753,  0.16780593,  0.4974    ],
                    [ 0.0,          0.0,          0.0,          1.0    ]])
print(np2ros_RTStampedWithHeader(target))
print('printing ros')
print(np2ros(target))