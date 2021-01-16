#!/usr/bin/env python

from get_transformation import *
from forward_kinematics.srv import *
import rospy
import numpy as np

def handle_transformation(req):
    mat = create_transformation_matrix_from_dh(req.theta)
    # (pos_x, pos_y, pos_z) = translation_from_transformation_matrix(mat)
    # (rot_x, rot_y, rot_z) = rotation_from_transformation_matrix(mat)
    #rospy.loginfo("Sending transformation: Px = %s, Py = %s, Pz= %s"%(pos_x, pos_y, pos_z))
    #return DhToPoseResponse(pos_x, pos_y, pos_z, rot_x, rot_y, rot_z)
    return DhToPoseMatrixResponse(np.reshape(mat, 16))

def transformation_server():
    rospy.init_node('transformation_server')
    s = rospy.Service('transformation_server', DhToPoseMatrix, handle_transformation)
    rospy.loginfo("Transformation server ready")
    rospy.spin()

if __name__ == "__main__":
    transformation_server()
