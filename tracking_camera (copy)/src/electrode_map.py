#!/usr/bin/env python

import tf.transformations as tf
import numpy as np
from numpy import linalg as la
import scipy.linalg as las
from atracsys_client import createPythonClient
from head_cordinate import createHeadCordinate
from moveit import Move_it
import time 
from helper_functions import ros2np
import csv

class electrodeMapping(object):
    
    def __init__(self):
        self.client = createPythonClient('134.28.45.17',5000,'bluestylustip','FORMAT_MATRIXROWWISE')
        self.robot = Move_it()
        self.robot_t_acam = np.load("/home/fasod/fasod_ws/src/project_arbeit/Data/hand_eye/robot_t_acam.npy")
        

    def create_csys(self):
        robot_t_ee = ros2np(self.robot.move_group.get_current_pose())
        print('ee location now /n')
        print(robot_t_ee)
        print('inv(ee) location now /n')
        #print(la.inv(robot_t_ee))
        print('starting the recording process in 10 seconds')
        time.sleep(10)
        # please remeber, its not your right or left. right ear of the phantom
        raw_input('please place the marker on the right ear of the phantom and hit enter') 
        matrix_rightEar = self.client.record_marker()
        print(matrix_rightEar)
        raw_input('please place the marker on the nose of the phantom and hit enter')
        matrix_nose = self.client.record_marker()
        print(matrix_nose)
        # please remeber, its not your right or left. left ear of the phantom
        raw_input('please place the marker on the left ear of the phantom and hit enter')
        matrix_leftEar = self.client.record_marker()
        print(matrix_leftEar)
        csys = createHeadCordinate(np.asarray(matrix_rightEar[0:3,3]), np.asarray(matrix_nose[0:3 , 3]), np.asarray(matrix_leftEar[0:3, 3]))
        self.acam_t_csys, error = csys.create_csys()
        print('Head cordinate system has been created and logged to data folder')
        #print(self.csys_head)
        np.save("/home/fasod/fasod_ws/src/project_arbeit/Data/head_coordinate/acam_t_csys.npy", self.acam_t_csys)
        return self.acam_t_csys
    
    
    def get_ee_t_csys(self):

        """method calculates the pose of the head coordinate system (csys) with respect to ee of robot.
            this has to be done only one time as transformation between csys and ee is constant!
        
        Returns:
            np.array -- 4x4 matrix
        """
        robot_t_ee = ros2np(self.robot.move_group.get_current_pose())
        self.acam_t_csys = np.load("/home/fasod/fasod_ws/src/project_arbeit/Data/head_coordinate/acam_t_csys.npy")
        self.ee_t_csys = np.matmul(la.inv(robot_t_ee),np.matmul(self.robot_t_acam, self.acam_t_csys))
        
        np.save("/home/fasod/fasod_ws/src/project_arbeit/Data/head_coordinate/ee_t_csys.npy", self.ee_t_csys)
    
        return self.ee_t_csys
    
    
    def create_csv_file(self, ee_t_sensor_list, csys_t_sensor_list, ee_t_csys):
        with open('/home/fasod/fasod_ws/src/project_arbeit/Data/head_coordinate/ee_t_sensor_list.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerows(ee_t_sensor_list)
        with open('/home/fasod/fasod_ws/src/project_arbeit/Data/head_coordinate/csys_t_sensor_list.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerows(csys_t_sensor_list)
        with open('/home/fasod/fasod_ws/src/project_arbeit/Data/head_coordinate/ee_t_csys.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerows(ee_t_csys)

    def electrode_map(self):
        self.acam_t_csys = np.load("/home/fasod/fasod_ws/src/project_arbeit/Data/head_coordinate/acam_t_csys.npy")
        print('Lets start the electrode mapping\n')
        csys_t_sensor_list = []
        ee_t_sensor_list = []
        response = ''
        raw_input('place the locator at first electrode and hit enter')
        while not response == 'q':
            self.robot_t_ee = ros2np(self.robot.move_group.get_current_pose())
            print('ee location now /n')
            print(self.robot_t_ee)
            
            acam_t_sensor = self.client.record_marker()
            print('acam_t_sensor now /n')
            print(acam_t_sensor)
            
            ee_t_sensor = np.matmul(la.inv(self.robot_t_ee),np.matmul(self.robot_t_acam, acam_t_sensor))
            print('sensor location wrt ee /n')
            print(ee_t_sensor)
            ee_t_sensor_list.append(ee_t_sensor)

            csys_t_sensor = np.matmul(la.inv(self.acam_t_csys), acam_t_sensor)
            print('sensor location wrt csys /n')
            print(csys_t_sensor)
            csys_t_sensor_list.append(csys_t_sensor)
            
            response = raw_input('move to next location and hit enter to continue! or q to quit')
        np.save("/home/fasod/fasod_ws/src/project_arbeit/Data/head_coordinate/ee_t_sensor_list.npy", ee_t_sensor_list)
        np.save("/home/fasod/fasod_ws/src/project_arbeit/Data/head_coordinate/csys_t_sensor_list.npy", csys_t_sensor_list)
        ee_t_csys = self.get_ee_t_csys()
        self.create_csv_file(ee_t_sensor_list, csys_t_sensor_list, ee_t_csys)
        return ee_t_sensor_list, csys_t_sensor_list, ee_t_csys

    
    def test_transform(self):
        acam_t_sensor_correct = self.client.record_marker()
        robot_t_ee = ros2np(self.robot.move_group.get_current_pose())
        ee_t_sensor = np.array([
                                    [0.04602,   0.98731,   0.14061, 0.0567074],
                                    [0.018159,  0.138306, -0.988365, 0.0992259],
                                    [-0.997217, 0.0465552, -0.013473, -0.043211],
                                            [0,         0,         0,         1]
                                                                                ])


        acam_t_sensor_cacl = np.matmul(la.inv(self.robot_t_acam),np.matmul(robot_t_ee, ee_t_sensor))
        print(self.robot_t_acam)
        print("printing directly from the attracsys")
        print(acam_t_sensor_correct)
        
        print("calculated acam_t_sensor")
        print(acam_t_sensor_cacl)
        q1 = tf.quaternion_from_matrix(acam_t_sensor_correct)
        q2 = tf.quaternion_from_matrix(acam_t_sensor_cacl)
        #angle_error  = deg_to_rad((2*arccos(|q1.q2|))) 
        q1_dot_q2 = np.absolute(np.dot(q1,q2))
        angle_error = (180/np.pi)*2*np.arccos(q1_dot_q2)
        print(angle_error)
         


if __name__ == "__main__":
    map = electrodeMapping()
    
    csys = map.create_csys()
    print('printing csys')
    print(csys)

    ee_t_sensor_list, csys_t_sensor_list, ee_t_csys = map.electrode_map()
    print('priting ee_t_csys')
    print(ee_t_csys)
    print('priting all the electrodes with repect to csys')
    print(csys_t_sensor_list)
    print('priting all the electrodes with repect to ee')
    print(ee_t_sensor_list)
    
    
    

