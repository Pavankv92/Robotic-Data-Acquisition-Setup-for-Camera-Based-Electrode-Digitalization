#!/usr/bin/env python3

""" this node saves the rgb image to data folder """

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from time import time


def callback(data):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(data)
    except CvBridgeError as e:
        print(e)

    cv2.imshow("Image vindow", cv_image)
    cv2.waitKey(10)
    k = raw_input('press 115 to save and 113 to quit')
    if k == ord('q'):
        cv2.destroyAllWindows()
        rospy.signal_shutdown("quit")
    elif k == ord('s'):
        try:
            print('printing before saving')
            ret = cv2.imwrite("/home/Vishwanath/catkin_ws/src/project_arbeit/Data/images_rgb/images_rgb{}.png".format(time()),cv_image)
        except cv2.error as e:
            print(e)
            return
        if ret:
            rospy.loginfo("Image saved")


def listener():
    """subscribes to rgb/image_raw topic"""
    rospy.init_node('test', anonymous=True)
    rospy.Subscriber('/rgb/image_raw', Image, callback)
    rospy.spin()

if __name__ == "__main__":
    listener()
    cv2.destroyAllWindows()