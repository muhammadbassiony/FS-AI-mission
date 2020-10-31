#linear kalman filter
import numpy as np


class KalmanFilter(object):
    
    def __init__(self, F = None, B = None, H = None, Q = None, R = None, P = None, x0 = None):

        self.n = F.shape[1]     #numebr of variables in the system
        self.m = H.shape[1]     #number of measurments

        self.F = F  #prediction matrix
        self.H = H  #sensor matrix
        self.B = 0 if B is None else B  #the control matrix
        
        #np.eye creates an empty matrix with a diagonal of 1s
        self.P = np.eye(self.n) if P is None else P     #covariance matrix
        self.Q = np.eye(self.n) if Q is None else Q     #noise
        
        self.R = np.eye(self.n) if R is None else R     #uncertainity covariance matrix
        
        self.x = np.zeros((self.n, 1)) if x0 is None else x0    #x0 is the inital state/values



    def predict(self, u = 0):
        # u the control vector
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    # z is the sensor readings
    def update(self, z):
        y = z - np.dot(self.H, self.x)  #observations - the old state
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))  #kalman gain
        self.x = self.x + np.dot(K, y) #corrected X
        I = np.eye(self.n) #1 
        self.P = np.dot(np.dot(I - np.dot(K, self.H), self.P), 
        	(I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R), K.T)    #corrected P