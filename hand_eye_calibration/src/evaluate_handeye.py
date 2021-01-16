#!/usr/bin/env python

import numpy as np
#import quaternion
from numpy import linalg as la
import scipy.linalg as las
import tf.transformations as tf
from util import R_to_axis_angle
#import quaternion




class Evaluate_handeye(object):
    def __init__(self):
        self.mat_x =np.array([  [-0.00327566,  0.00167141, -1.00338648,  0.05718713],
                                [-0.00642112,  1.00284865,  0.0013221 , -0.0715229 ],
                                [ 1.00363757,  0.00604172, -0.00398433, -0.14704855],
                                [ 0.        ,  0.        ,  0.        ,  1.        ]])
                                
        
        self.mat_y = np.array([[-0.05314779, -0.07007525,  0.99940589, -1.57861299],
                            [-1.00210881,  0.02045026, -0.05251911,  0.12344145],
                            [-0.01649032, -1.00024422, -0.071525  ,  0.6297281 ],
                            [ 0.        ,  0.        ,  0.        ,  1.        ]])
        
        
        #self.mat_x = np.load("/home/Vishwanath/catkin_ws/src/project_arbeit/Data/hand_eye/matx.npy")
        #self.mat_y = np.load("/home/Vishwanath/catkin_ws/src/project_arbeit/Data/hand_eye/maty.npy")
        self.mat_m_testData_full = np.load("/home/Vishwanath/catkin_ws/src/project_arbeit/Data/hand_eye/rob.npy")
        self.mat_n_testData_full = np.load("/home/Vishwanath/catkin_ws/src/project_arbeit/Data/hand_eye/cam.npy")

        self.mat_m_testData = self.mat_m_testData_full[39:]
        self.mat_n_testData = self.mat_n_testData_full[39:]
    
        #debug
        #print(self.mat_m_testData)
        #print('------------------------------')
        #print(self.mat_n_testData)
        #print(self.mat_x)
        #print('------------------------------')
        #print(self.mat_y)

    
    def get_translation_2norm(self, matrix):
        vector_tmp = []
        vector_tmp = matrix [0:3, 3]
        norm = la.norm(vector_tmp, ord=2)
        return norm

    def get_orthoNormalisedMat(self, matrix):
        
        """
        fucntion generates orthonormalised basis for given matrix 
        and replaces the 3X3 rotation portion of the given matrix 
        
        Arguments:
            matrix {[np.array]} -- 4X4 matrix
        
        Returns:
            [np.array] -- [matrix, error] 
                          4X4 matrix with 3X3 rotation replaced with orthonormal basis, orthonormalisation error
        """
        
        matrix_tmp = np.eye(3)
        matrix_tmp [0:3, 0:3] = matrix [0:3 , 0:3]
        u,s,v = la.svd(matrix_tmp)
        mat_unitary = np.matmul(u, np.transpose(v))
        matrix [0:3, 0:3] = mat_unitary [0:3, 0:3]
        return matrix
    
    def compute_expected_identity_list(self, mat_m, mat_x, mat_y, mat_n):
        expected_identity_list = []
        for i in range(len(mat_m)):
            expected_identity_1 = np.matmul(mat_y, mat_n[i])
            expected_identity_2 = np.matmul((mat_m[i]),mat_x) 
            expected_identity = np.matmul(la.inv(expected_identity_1), expected_identity_2)
            expected_identity_list.append(expected_identity)
        return expected_identity_list
    
    def evaluate(self, expected_identity_list):
        translation_error_list = []
        rotation_angles_list = []
        for i in range(len(expected_identity_list)):
            translation_error_list.append(self.get_translation_2norm(expected_identity_list[i]))
            
            mat_rot_unitary = self.get_orthoNormalisedMat(expected_identity_list[i])
            
            axis, angle_error = R_to_axis_angle(mat_rot_unitary [0:3, 0:3]) 
            rotation_angles_list.append(angle_error)
        
        translation_norm = la.norm(translation_error_list, ord=2)
        rotation_norm = la.norm(rotation_angles_list, ord=2)

        #debug
        #print(rotation_angles_list)
        return translation_norm, rotation_norm 
            

    def main(self):
        
        #self.mat_x_normalised = self.get_orthoNormalisedMat(self.mat_x)
        #self.mat_y_normalised = self.get_orthoNormalisedMat(self.mat_y)
        self.expected_identity_list = self.compute_expected_identity_list(self.mat_m_testData, self.mat_x, self.mat_y, self.mat_n_testData)
        print(self.expected_identity_list)
        self.translation_norm, self.rotation_norm = self.evaluate(self.expected_identity_list)
        print('trans error')
        print(self.translation_norm)
        print('rot error')
        print(self.rotation_norm)

        #debug
        #print('mat_x : /n')
        #print(self.mat_y)
        #print('mat_x_normalised : /n')
        #print(self.mat_y_normalised)
        
            
if __name__ == "__main__":
    check = Evaluate_handeye()
    check.main()


