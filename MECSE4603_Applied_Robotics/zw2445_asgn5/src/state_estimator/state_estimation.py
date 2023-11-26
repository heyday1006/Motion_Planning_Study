#!/usr/bin/env python

# Columbia Engineering
# MECS 4603 - Fall 2017

import math
import numpy
import time
import matplotlib.pyplot as plt

# This class pretends to be the real robot. It generates the "true"
# information which is generally unknowable in the real world.
class Robot(object):

    def __init__(self):
        self.mass = 1.1
        self.F = numpy.array([[1,1],[0,1]])
        self.G = numpy.array([[0],[1.0/self.mass]])
        self.H = numpy.array([[0,1]])

        self.x_true = numpy.array([[0],[0]])

        self.model_noise = True
        self.V = numpy.array([[10,0],
                              [0,10]])
        self.sensor_noise = True
        self.W = 100.0

    def reset(self, x):
        self.x_true = x
        
    def step(self, u):
        self.x_true = numpy.dot(self.F, self.x_true) + numpy.dot(self.G, u)
        if self.model_noise:
            self.x_true = self.x_true + numpy.random.multivariate_normal(numpy.zeros(2), self.V)
            
    def sense(self):        
        y = numpy.dot(self.H, self.x_true)
        if self.sensor_noise:
            y = y + numpy.random.normal(0.0, self.W)
        return y

    def get_true_state(self):
        return self.x_true

class Estimator(object):

    def __init__(self):
        # This is our *model* of the robot, which might or might not be an accurate
        # description of the robot above.
        self.mass = 1.0
        self.F = numpy.array([[1,1],[0,1]])
        self.G = numpy.array([[0],[1.0/self.mass]])
        self.H = numpy.array([[0,1]])

        # This is our *estimate* of the characteristics of the noise.
        self.V = numpy.array([[10,0],
                              [0,10]])
        self.W = 100.0

        self.x_est = numpy.array([[0],[0]])
        self.P_est = numpy.array([[0,0],[0,0]])

    def reset(self, x, P):
        self.x_est = x
        self.P_est = P

    # A simple prediction step based on the mode, without an update from sensor data.
    def simple_step(self, u, y):
        x_pred = numpy.dot(self.F, self.x_est) + numpy.dot(self.G, u)
        self.x_est = x_pred
        
    # Prediction and update loop, without a probabilistic framework.
    def step(self, u, y):
        x_pred = numpy.dot(self.F, self.x_est) + numpy.dot(self.G, u)
        
        innov = y - numpy.dot(self.H, x_pred)
        HHT = numpy.dot(self.H, numpy.transpose(self.H))
        HHTinv = numpy.linalg.inv(HHT)
        R = numpy.dot( numpy.transpose(self.H), HHTinv)

        delta_x = numpy.dot(R, innov)
        self.x_est = x_pred + delta_x

    # Full Kalman filter.
    def kalman_step(self, u, y):
        x_pred = numpy.dot(self.F, self.x_est) + numpy.dot(self.G, u)
        P_pred = numpy.dot(self.F, numpy.dot(self.P_est, numpy.transpose(self.F))) + self.V

        innov = y - numpy.dot(self.H, x_pred)
        S = numpy.dot( self.H, numpy.dot(P_pred, numpy.transpose(self.H)) ) + self.W
        R = numpy.dot(P_pred, numpy.transpose(self.H)) * 1.0/S
        
        delta_x = numpy.dot(R, innov)
        self.x_est = x_pred + delta_x
        delta_P = -1.0 * numpy.dot( R, numpy.dot(self.H,P_pred) )
        self.P_est = P_pred + delta_P

    def get_estimate(self):
        return self.x_est

    def get_variance(self):
        return self.P_est

class GUI(object):
    def __init__(self):
        self._fig = plt.figure(figsize=(100,1000))
        self._ax1 = self._fig.add_subplot(1,1,1)
        plt.show(False)

    def plot_state(self, x_true, x_est, P_est):
        self._ax1.clear()
        plt.xlim(-10000,90000)
        plt.ylim(-500,500)
        ptx=[x_true[0,0]]
        pty=[x_true[1,0]]
        self._ax1.scatter(ptx, pty, c='r', linewidths = 10);
        ptx=[x_est[0,0]]
        pty=[x_est[1,0]]
        self._ax1.scatter(ptx, pty, c='b', linewidths = 10);
        if P_est[0,0]!=0.0:
            ptx=[x_est[0,0]-P_est[0,0],
                 x_est[0,0]+P_est[0,0]]
            pty=[x_est[1,0],
                 x_est[1,0]]
            self._ax1.scatter(ptx, pty, c='b', marker='_', linewidths=10);            
            ptx=[x_est[0,0],
                 x_est[0,0]]
            pty=[x_est[1,0] + P_est[1,1],
                 x_est[1,0] - P_est[1,1]]
            self._ax1.scatter(ptx, pty, c='b', marker='|', linewidths=10);            
        self._fig.canvas.draw()

if __name__ == '__main__':
    robot = Robot()
    robot.reset(numpy.array([[0],[0]]))
    estimator = Estimator()
    estimator.reset(numpy.array([[0],[0]]), numpy.array([[0,0],[0,0]]))
    gui = GUI()
    t = math.pi / 2.0
    while True:
        # Command is simply a sinusoid force applied to robot.
        u = numpy.array([[math.sin(t)]])
        robot.step(u)

        y = robot.sense()
        estimator.kalman_step(u, y)
        gui.plot_state(robot.get_true_state(),
                       estimator.get_estimate(),
                       estimator.get_variance())
        t = t + 0.005
        time.sleep(0.01)

# Possible options:
# 1. Perfect robot model, no noise. Simple step tracks well.
# 2. With the addition of model error (wrong mass), simple model tracks poorly
# and error is unknown and unbounded.
# 3. Addition of model noise further worsens performance.
# 4. Going to the prediction-and-update step alleviates problems.
# 5. Adding sensor noise again makes for bad performance.
# 6. Kalman filter has much better tracking in this case. Still, note that
# position uncertainty grows forever (we are never measuring it directly).
# 7. It also works well if we remove modeling error and noise (as expected).
        
