#!/usr/bin/env python

""" calcualtes the chkbd pose for handeye calibaration """

from __future__ import print_function
import numpy as np
import cv2
import glob



criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-16)
np.set_printoptions(precision=16)

def createObjectpoints(height,width,size):
    objp = np.zeros((height*width, 3), np.float32)
    objp[:,:2] = np.indices((width, height)).T.reshape(-1,2)
    objp *= size
    return np.around(objp, 3)

def transform_from_image(img, show=False):
    objp = createObjectpoints(5,8,0.04)
   
    cam_mtx= np.array([[9.68727469e+02,   0.00000000e+00,   1.02687381e+03],
                       [0.00000000e+00,   9.68978580e+02,  7.74022724e+02],
                       [0, 0, 1]], np.float32)

    cam_dist= np.array([0.07588704,  0.04285212, -0.00064595,  0.00132813, -0.25794438], np.float32)
    gray_image=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray_image, (8,5))

    if found:
        cv2.cornerSubPix(gray_image, corners, (11,11), (-1, -1), criteria)

       #debug
        img = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
        cv2.drawChessboardCorners(img, (8,5), corners, found)
        cv2.imshow('image', img)
        cv2.waitKey(250)


        #solvePnP is used to get the position and orientation of the object
        found, rvecs, tvec = cv2.solvePnP(objp, corners, cam_mtx, cam_dist)
        rot3X3 = cv2.Rodrigues(rvecs)[0]
        
        #print(rotationMatrix_3X3)
        transformation = np.array([ [rot3X3[0,0], rot3X3[0,1], rot3X3[0,2], tvec[0]],
                                    [rot3X3[1,0], rot3X3[1,1], rot3X3[1,2], tvec[1]],
                                    [rot3X3[2,0], rot3X3[2,1], rot3X3[2,2], tvec[2]],
                                    [0, 0, 0, 1]], np.float32)

    else:
        print("pattern not found")
        raise ValueError("Could not find the pattern in the image")

    return transformation


if __name__ == "__main__":
    all_image_names = glob.glob("/home/Vishwanath/catkin_ws/src/project_arbeit/Data/images_rgb/*.png")
        
    for name in all_image_names:
        img = cv2.imread(name)
        #cv2.imshow('image', img)
        #cv2.waitKey(1000)
        gray_image = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        matrix = transform_from_image(gray_image)
        print(matrix)
