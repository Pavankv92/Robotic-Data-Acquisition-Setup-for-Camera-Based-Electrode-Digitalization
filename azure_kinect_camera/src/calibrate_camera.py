#!/usr/bin/env python
"""this class offers methods to calibrate the camera using precaptured images using opencv
    no need to undistort the images as distortion coefficients are being calculated
"""


import numpy as np
import cv2
import glob



class calibrateKinectCamera(object):

    def __init__(self, pattern = (8,5)): # setting the default pattern of the checkerboard
        self.pattern = pattern
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.001)

    def createObjectpoints(self,height,width,size):

        """3D planar object points in world coordinate system for 2D/3D homograpy creation
        
        Arguments:
            height {int} -- number of rows in checker board
            width {int} -- number coulmns in checker board
            size {meter} -- width of the single square on the checker board
        
        Returns:
            array -- array of objects points 
        """
       
        object = []
        for j in range(height):
            for i in range(width):
                A = np.array([i*size,j*size,0])
                object.append(A)
        objp = np.asarray(object,dtype=np.float32)
        return objp

    def calibrate(self):

        """method to calibrate the camera using 2D/3D homography.
        
        Returns:
            manythings -- rms error of the least square solution, camera matrix, distorsion coeeficients,
                          checker board pose(rotation and translation)           
        """
        objp = self.createObjectpoints(5,8,0.04)
        objectpoints = []
        imagepoints = []
    
        all_image_names = glob.glob('/home/pavankv/catkin_ws/src/project_arbeit/Data/images_rgb/*.png')
        
        for name in all_image_names:
            img = cv2.imread(name)
            img = cv2.resize(img,(1366,768))
            #cv2.imshow('image', img)
            #cv2.waitKey(200)
            self.gray_image = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

            found, corners = cv2.findChessboardCorners(self.gray_image, self.pattern,None)

            if found:
                cv2.cornerSubPix(self.gray_image, corners, (11,11), (-1, -1), self.criteria)
                objectpoints.append(objp)
                imagepoints.append(corners)

                #debug
                img = cv2.cvtColor(self.gray_image, cv2.COLOR_GRAY2BGR)
                cv2.drawChessboardCorners(img, self.pattern, corners, found)
                cv2.imshow('image', img)
                cv2.waitKey(1000)
            else:
                print("not found " + name)

        cv2.destroyAllWindows()
        rms, camera_matrix, dist_coeffs, rot_vecs, trans_vecs = cv2.calibrateCamera(objectpoints, imagepoints, self.gray_image.shape[::-1], None, None)
        return rms, camera_matrix, dist_coeffs, rot_vecs, trans_vecs


if __name__ == "__main__":
    cam = calibrateKinectCamera()
    rms, camera_matrix, dist_coeffs, rot_vecs, trans_vecs = cam.calibrate()
    
    print("RMS: ", rms)
    print("camera matrix: \n", camera_matrix)
    print("distortion coefficients: \n", dist_coeffs)
    print("rot: \n", rot_vecs)
    print("translate: \n", trans_vecs)
    print('')
