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
        print('starting the recording process in 10 seconds')
        time.sleep(1)
        # please remeber, its not your right or left. right ear of the phantom
        raw_input('please place the marker on the right ear of the phantom and hit enter') 
        matrix_rightEar = self.client.record_marker()
        np.save("/home/fasod/fasod_ws/src/project_arbeit/Data/head_coordinate/matrix_rightEar.npy", matrix_rightEar)
        raw_input('please place the marker on the nose of the phantom and hit enter')
        matrix_nose = self.client.record_marker()
        np.save("/home/fasod/fasod_ws/src/project_arbeit/Data/head_coordinate/matrix_nose.npy", matrix_nose)
        # please remeber, its not your right or left. left ear of the phantom
        raw_input('please place the marker on the left ear of the phantom and hit enter')
        matrix_leftEar = self.client.record_marker()
        np.save("/home/fasod/fasod_ws/src/project_arbeit/Data/head_coordinate/matrix_leftEar.npy", matrix_leftEar)
        csys = createHeadCordinate(np.asarray(matrix_rightEar[0:3,3]), np.asarray(matrix_nose[0:3 , 3]), np.asarray(matrix_leftEar[0:3, 3]))
        self.acam_t_csys, error = csys.create_csys()
        print('Head cordinate system has been created and logged to data folder')
        np.save("/home/fasod/fasod_ws/src/project_arbeit/Data/head_coordinate/acam_t_csys.npy", self.acam_t_csys)
        self.ee_t_csys = self.get_ee_t_csys()
        print("Right ear")
        csys_t_rightEar = np.matmul(la.inv(self.acam_t_csys), matrix_rightEar)
        print(csys_t_rightEar)
        print("Nose")
        csys_t_nose = np.matmul(la.inv(self.acam_t_csys), matrix_nose)
        print(csys_t_nose)
        print("left ear")
        csys_t_leftEar = np.matmul(la.inv(self.acam_t_csys), matrix_leftEar)
        print(csys_t_leftEar )
        
        with open('/home/fasod/fasod_ws/src/project_arbeit/Data/head_coordinate/matrix_rightEar.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerows(matrix_rightEar)
        with open('/home/fasod/fasod_ws/src/project_arbeit/Data/head_coordinate/matrix_nose.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerows(matrix_nose)
        with open('/home/fasod/fasod_ws/src/project_arbeit/Data/head_coordinate/matrix_leftEar.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerows(matrix_leftEar)
        with open('/home/fasod/fasod_ws/src/project_arbeit/Data/head_coordinate/acam_t_csys.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerows(self.acam_t_csys)
        with open('/home/fasod/fasod_ws/src/project_arbeit/Data/head_coordinate/csys_t_rightEar.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerows(csys_t_rightEar)
        with open('/home/fasod/fasod_ws/src/project_arbeit/Data/head_coordinate/csys_t_nose.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerows(csys_t_nose)
        with open('/home/fasod/fasod_ws/src/project_arbeit/Data/head_coordinate/csys_t_leftEar.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerows(csys_t_leftEar)
        return self.acam_t_csys
    
    
    def get_ee_t_csys(self):

        """method calculates the pose of the head coordinate system (csys) with respect to ee of robot.
            this has to be done only one time as transformation between csys and ee is constant!
        
        Returns:
            np.array -- 4x4 matrix
        """
        robot_t_ee = ros2np(self.robot.move_group.get_current_pose())
        self.acam_t_csys = np.load("/home/fasod/fasod_ws/src/project_arbeit/Data/head_coordinate/acam_t_csys.npy")
        ee_t_csys = np.matmul(la.inv(robot_t_ee),np.matmul(self.robot_t_acam, self.acam_t_csys))
        
        np.save("/home/fasod/fasod_ws/src/project_arbeit/Data/head_coordinate/ee_t_csys.npy", ee_t_csys)

        return ee_t_csys
    
    
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
        #sensor_annotation_list = ["Fp1", "Fp2", "F7", "F3", "Fz", "F4", "F8" , "M1", "T3", "C3", "Cz","C4","T4", "M2", "T5", "P3", "Pz", "P4", "T6", "O1", "O2", "REF" ]
        sensor_annotation_list = ["Fpz", "GND", "Fz", "Cz", "F3", "C3", "REF", "Pz" , "F4", "C4" ]
        while not response == 'q':
            for i in range(len(sensor_annotation_list)):
                raw_input('place the locator at'+ '-' + sensor_annotation_list[i] + '-' + 'and hit enter')
                self.robot_t_ee = ros2np(self.robot.move_group.get_current_pose())
                #print('ee location now /n')
                #print(self.robot_t_ee)
                
                acam_t_sensor = self.client.record_marker()
                #print('acam_t_sensor now /n')
                #print(acam_t_sensor)
                
                ee_t_sensor = np.matmul(la.inv(self.robot_t_ee),np.matmul(self.robot_t_acam, acam_t_sensor))
                #print('sensor location wrt ee /n')
                #print(ee_t_sensor)
                ee_t_sensor_list.append(ee_t_sensor)

                csys_t_sensor = np.matmul(la.inv(self.ee_t_csys), ee_t_sensor)
                #print('sensor location wrt csys /n')
                #print(csys_t_sensor)
                csys_t_sensor_list.append(csys_t_sensor)
            
            #response = raw_input('move to next location and hit enter to continue! or q to quit')
            response = raw_input('enter q to quit')
        np.save("/home/fasod/fasod_ws/src/project_arbeit/Data/head_coordinate/ee_t_sensor_list.npy", ee_t_sensor_list)
        np.save("/home/fasod/fasod_ws/src/project_arbeit/Data/head_coordinate/csys_t_sensor_list.npy", csys_t_sensor_list)
        ee_t_csys = self.get_ee_t_csys()
        self.create_csv_file(ee_t_sensor_list, csys_t_sensor_list, ee_t_csys)
        return ee_t_sensor_list, csys_t_sensor_list, ee_t_csys

    
    def test_head_orientation(self):
        test_ee_t_sensor_list = []
        test_csys_t_sensor_list = []
        for i in range(10):
            raw_input('place the locator at any single electorde and hit enter')
            self.robot_t_ee = ros2np(self.robot.move_group.get_current_pose())
            #print('ee location now /n')
            #print(self.robot_t_ee)
            
            acam_t_sensor = self.client.record_marker()
            #print('acam_t_sensor now /n')
            #print(acam_t_sensor)
            
            ee_t_sensor = np.matmul(la.inv(self.robot_t_ee),np.matmul(self.robot_t_acam, acam_t_sensor))
            #print('sensor location wrt ee /n')
            #print(ee_t_sensor)
            test_ee_t_sensor_list.append(ee_t_sensor)

            csys_t_sensor = np.matmul(la.inv(self.ee_t_csys), ee_t_sensor)
            #print('sensor location wrt csys /n')
            #print(csys_t_sensor)
            test_csys_t_sensor_list.append(csys_t_sensor)
        np.save("/home/fasod/fasod_ws/src/project_arbeit/Data/head_coordinate/test_ee_t_sensor_list.npy", test_ee_t_sensor_list)
        np.save("/home/fasod/fasod_ws/src/project_arbeit/Data/head_coordinate/test_csys_t_sensor_list.npy", test_csys_t_sensor_list)
        with open('/home/fasod/fasod_ws/src/project_arbeit/Data/head_coordinate/test_ee_t_sensor_list.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerows(test_ee_t_sensor_list)
        with open('/home/fasod/fasod_ws/src/project_arbeit/Data/head_coordinate/test_csys_t_sensor_list.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerows(test_csys_t_sensor_list)
         
    def test_stylus_orientation(self):
        test_csys_t_stylus = []
        for i in range(10):
            raw_input('place the stylus on a single electrode and hit enter')
            self.robot_t_ee = ros2np(self.robot.move_group.get_current_pose())
            acam_t_sensor = self.client.record_marker()
            ee_t_sensor = np.matmul(la.inv(self.robot_t_ee),np.matmul(self.robot_t_acam, acam_t_sensor))
            csys_t_sensor = np.matmul(la.inv(self.ee_t_csys), ee_t_sensor)
            test_csys_t_stylus.append(csys_t_sensor)
        with open('/home/fasod/fasod_ws/src/project_arbeit/Data/head_coordinate/test_csys_t_stylus.csv', 'w') as file:
            writer = csv.writer(file)
            writer.writerows(test_csys_t_stylus)

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
    
    """
    map.test_head_orientation()
    map.test_stylus_orientation()
    """
    

