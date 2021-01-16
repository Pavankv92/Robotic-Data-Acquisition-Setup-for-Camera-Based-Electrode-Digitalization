#!/usr/bin/env python3
import socket
import warnings
import time
import numpy as np

class createPythonClient(object):

    def __init__(self, address, port, locatorname, format, timeout=25):
        self.address = address
        self.port = port
        self.format = format
        self.locatorname = locatorname
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_address = (self.address, self.port)
        self.timeout = timeout
        self.sock.settimeout(self.timeout)
         

        print('\nTrying to connect to: ' + str(self.address) + 'port: ' + str(self.port))

        try:
            self.sock.connect(self.server_address)
            time.sleep(1)
            msg = ('CM_GETSYSTEM')
            data1 = self.send_recieve(msg)
            print(data1)
            data2 = self.send_recieve(self.locatorname)
            print(data2)
            data3 = self.send_recieve(format)
            print(data3)
            print('Connection successful')

        except socket.timeout:
            print('Server did not answer in ' + str(self.timeout) + ' seconds and timed-out.')
            print('Make sure the server is running and has the IP address and port: ' + str(self.server_address))
            print('Exiting program because no server connection could be established')
            exit(0)
    
    def disconnect(self):
        # disconnect from server
        self.send_string('disconnect')
        self.sock.close()
        print('Disconnected from server!')

    def send_string(self, string):
        # send a string line to server
        self.sock.send((str(string) + '\n').encode())
    
    def receive_line(self, decoding='ascii', chunk_length=8):
        # receive a byte string from server and decode it
        data_str = ''
        try:
            while "\n" not in data_str:
                data = self.sock.recv(chunk_length)
                data_str += str(data.decode(decoding))

        except socket.timeout:
            print('Did not receive full force reading within timeout limit of ' + str(self.timeout) + ' seconds.')
            warnings.warn('Returning zero readings.')
            data_str = ''

        return data_str
    
    def send_recieve(self, string, decoding='ascii', chunk_length=8):
        self.send_string(string)
        time.sleep(0.1)
        data = self.receive_line(decoding,8)
        return data
    
    def get_transformMatrix(self, decoding='ascii', chunk_length=8):  
        matrix = np.eye(4)
        data_str = ''
        try:
            while "\n" not in data_str:
                string = 'CM_NEXTVALUE'
                data = self.send_recieve(string)
                data_str += str(data)
                print(data_str)

        except socket.timeout:
            print('Did not receive full reading within timeout limit of ' + str(self.timeout) + ' seconds.')
            warnings.warn('Returning zero readings.')
        
        data_array = data_str.split()
        #print(data_array)
        matrix[0, 0:4] = data_array[2:6]
        matrix[1, 0:4] = data_array[6:10]
        matrix[2, 0:4] = data_array[10:14]

        matrix [0,3] = matrix [0,3] /1000
        matrix [1,3] = matrix [1,3] /1000
        matrix [2,3] = matrix [2,3] /1000
        return matrix


    def record_marker(self):
        transformation = False
        while not transformation:
            matrix = self.get_transformMatrix()
            if np.array_equal(np.eye(4), matrix):
                raw_input('server is not ready yet, re-place the locator and hit enter')
            else:
                transformation = True
                return matrix
  

        
if __name__ == "__main__":
    s = createPythonClient('134.28.45.17',5000,'bluestylustip','FORMAT_MATRIXROWWISE')
    check = s.record_marker()
    print(check)
    s.disconnect()


