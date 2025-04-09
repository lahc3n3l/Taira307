import math
from ulab import numpy as np

# Constants
DEG_TO_RAD = math.pi/180
RAD_TO_DEG = 180/math.pi

class KalmanFilter:
    def __init__(self):
        self.A = np.eye(3)
        self.H = np.array([[1, 0, 0],[0, 1, 0]])
        self.Q = np.eye(3) * (100)**2
        self.R = np.eye(2) * (0.2)**2
        self.P = np.eye(3) * (10)**2
        self.x = np.zeros(3)

    def predict(self, u, dt):
        self.B = np.eye(3)*dt
        self.Q = np.dot(self.B, self.B.transpose()) * (1 *DEG_TO_RAD)**2
        self.x = np.dot(self.A, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.A, self.P), self.A.transpose()) + self.Q

    def update(self, accel, mag):
        mx, my, mz = mag
  
        y = np.array([math.atan2(accel[1], math.sqrt(accel[0]**2 + accel[2]**2)), 
                      math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2))])
        
        S = np.dot(np.dot(self.H, self.P), self.H.transpose()) + self.R
        K = np.dot(np.dot(self.P, self.H.transpose()), np.linalg.inv(S))
        self.x += np.dot(K, (y - np.dot(self.H, self.x)))
        self.P = np.dot((np.eye(3) - np.dot(K, self.H)), self.P)

        
    def get_euler_angles_d(self):
        roll, pitch, yaw = self.x
        return roll*RAD_TO_DEG, pitch*RAD_TO_DEG, yaw*RAD_TO_DEG


