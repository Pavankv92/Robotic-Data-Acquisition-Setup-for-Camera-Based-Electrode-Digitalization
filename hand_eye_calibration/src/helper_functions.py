#!/usr/bin/env python

from math import sqrt
import tf.transformations as tf
from geometry_msgs.msg import Pose, PoseStamped
from rt_msgs.msg import TransformRTStampedWithHeader
import numpy as np

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
        An HTM (type: PoseStamped).
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

def ur2ros_pose(ur_pose):
    """Transform pose from UR format to ROS Pose format.

    Args:
        ur_pose: A pose in UR format [px, py, pz, rx, ry, rz] units : mm, radians
        (type: list)

    Returns:
        An HTM (type: Pose).
    """

    # ROS pose
    ros_pose = PoseStamped()

    # ROS position
    ros_pose.pose.position.x = ur_pose[0]/1000
    ros_pose.pose.position.y = ur_pose[1]/1000
    ros_pose.pose.position.z = ur_pose[2]/1000

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

