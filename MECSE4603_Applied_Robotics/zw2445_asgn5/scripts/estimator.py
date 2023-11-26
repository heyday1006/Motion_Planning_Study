#!/usr/bin/env python

import numpy
import math

import rospy

from geometry_msgs.msg import Pose2D
from state_estimator.msg import SensorData

class Estimator(object):

    def __init__(self):
        self.time_step = 0.01
        self.F = numpy.zeros((3,3))

        # Initialization
        # noise
        self.V = 0.1 * numpy.identity(3)
        self.theta = 0
        self.x_est = numpy.array([[0],[0],[0]])
        self.P_est = numpy.array([[0,0,0],[0,0,0],[0,0,0]])
        # sensor data
        self.vel_trans = 0
        self.vel_ang = 0
        self.readings = []
        
        self.pub_pose = rospy.Publisher("/robot_pose_estimate", Pose2D, queue_size=1)
        rospy.Subscriber("/sensor_data", SensorData, self.EKF)
    
    # obtain sensor informaiton
    def get_sensor_data(self,msg):
        self.vel_trans = msg.vel_trans
        self.vel_ang = msg.vel_ang
        self.readings = msg.readings
        
    # Extended Kalman filter.
    def EKF(self,msg):
        self.get_sensor_data(msg)
        # Jacobian of f - F
        # self.theta += self.time_step * self.vel_ang
        self.theta = self.x_est[2]
        self.F = numpy.array([[1.0, 0, - self.time_step * self.vel_trans * math.sin(self.theta)],
                              [0, 1.0, self.time_step * self.vel_trans * math.cos(self.theta)],
                              [0, 0, 1.0]])
        print("x_estimate: ", self.x_est)
        # Prediction
        x_pred = numpy.dot(self.F, self.x_est)
        P_pred = numpy.dot(numpy.dot(self.F, self.P_est), numpy.transpose(self.F)) + self.V
        if len(self.readings) == 0:
            # if no nearby landmark, prediction only 
            print("no nearby landmark")
            self.x_est = x_pred
            self.P_est = P_pred
        else:
            # if has nearby landmarks, do predictiona and update
            print("has nearby landmarks: ", len(self.readings))
            # Jacobian of h - H - h1: range, h2: bearing
            self.H = numpy.zeros((2*len(self.readings),3))
            # noise
            self.W = 0.1 * numpy.identity(2 * len(self.readings))
            # sensing information w.r.t receiving sensor data and sensor model on prediction
            y = numpy.zeros((2*len(self.readings),1))
            h = numpy.zeros((2*len(self.readings),1))
            # n nearby landmarks, 2*n sensor data
            for i in range(len(self.readings)):
                denom = (self.readings[i].landmark.x - x_pred[0])**2 + (self.readings[i].landmark.y - x_pred[1])**2
                dh1_x = (x_pred[0] - self.readings[i].landmark.x)/numpy.sqrt(denom)
                dh1_y = (x_pred[1] - self.readings[i].landmark.y)/numpy.sqrt(denom)
                dh2_x = (self.readings[i].landmark.y - x_pred[1])/denom
                dh2_y = -(self.readings[i].landmark.x - x_pred[0])/denom
                h_1 = math.sqrt((x_pred[0] - self.readings[i].landmark.x)**2 + (x_pred[1] - self.readings[i].landmark.y)**2)
                h_2 = math.atan2(self.readings[i].landmark.y - x_pred[1], self.readings[i].landmark.x - x_pred[0]) - x_pred[2]
                # H
                self.H[2*i,:] = [dh1_x,dh1_y,0]
                self.H[2*i+1,:] = [dh2_x,dh2_y,-1]
                # y
                y[2*i,:] = self.readings[i].range
                y[2*i+1,:] = self.readings[i].bearing
                # h
                h[2*i,:] = h_1
                h[2*i+1,:] = h_2   
            # innovation        
            innov = y - h
            S = numpy.dot(numpy.dot(self.H, P_pred), numpy.transpose(self.H)) + self.W
            R = numpy.dot(numpy.dot(P_pred, numpy.transpose(self.H)), numpy.linalg.pinv(S))
            delta_x = numpy.dot(R, innov)
            self.x_est = x_pred + delta_x
            delta_P = -1.0 * numpy.dot( R, numpy.dot(self.H,P_pred) )
            self.P_est = P_pred + delta_P
        # publish the update
        self.publish_pose()

    def publish_pose(self):
        msg = Pose2D()
        msg.x = self.x_est[0]
        msg.y = self.x_est[1]
        msg.theta = self.x_est[2]
        self.pub_pose.publish(msg)

if __name__ == '__main__':
    rospy.init_node('estimator', anonymous=True)
    Estimator()
    rospy.spin()
    
