#!/usr/bin/env python

# Columbia Engineering
# MECS 4603 - Fall 2017

import math
import numpy
import time
import rospy
import random

from geometry_msgs.msg import Pose2D
from state_estimator.msg import SensorData
from state_estimator.msg import Landmark
from state_estimator.msg import LandmarkReading
from state_estimator.msg import LandmarkSet

def create_landmark(x, y):
   l = Landmark()
   l.x = x
   l.y = y
   return l

class Robot(object):

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.vt = 0.0
        self.vrot = 0.0
        self.vt_sens = 0.0
        self.vrot_sens = 0.0

        self.landmarks = []
        self.landmarks.append(create_landmark(5,5))
        self.landmarks.append(create_landmark(5,6))
        self.landmarks.append(create_landmark(6,5))        
        self.landmarks.append(create_landmark(-5,5))
        self.landmarks.append(create_landmark(-5,6))
        self.landmarks.append(create_landmark(-6,5))        
        self.landmarks.append(create_landmark(5,-5))
        self.landmarks.append(create_landmark(5,-6))
        self.landmarks.append(create_landmark(6,-5))        
        self.landmarks.append(create_landmark(-5,-5))
        self.landmarks.append(create_landmark(-5,-6))
        self.landmarks.append(create_landmark(-6,-5))        
        self.landmarks.append(create_landmark(5,0))
        self.landmarks.append(create_landmark(-5,0))
        self.landmarks.append(create_landmark(0,5))
        self.landmarks.append(create_landmark(0,-5))
        self.landmarks.append(create_landmark(1,0))

        self.sensing_range = 3.0 

        self.pub_pose = rospy.Publisher("/robot_pose", Pose2D, queue_size=1)
        self.pub_sens = rospy.Publisher("/sensor_data", SensorData, queue_size=1)
        self.pub_landmarks = rospy.Publisher("/landmarks", LandmarkSet, queue_size=1)

        self.step_size = 0.01
        self.pub_timer = rospy.Timer(rospy.Duration(0.01), self.publish_pose)
        self.pub_data_timer = rospy.Timer(rospy.Duration(0.01), self.publish_sensor_data)
        self.pub_landmarks_timer = rospy.Timer(rospy.Duration(1.0), self.publish_landmarks)
        self.step_timer = rospy.Timer(rospy.Duration(self.step_size), self.step)
        self.rand_vel_timer = rospy.Timer(rospy.Duration(5.0), self.rand_vel_event)
        self.rand_vel()
        
    def get_sensor_data(self):
        sens = SensorData()
        sens.vel_trans = self.vt_sens
        sens.vel_ang = self.vrot_sens
        
        for i in range(0,len(self.landmarks)):
                r = math.sqrt( (self.landmarks[i].x-self.x)*(self.landmarks[i].x-self.x) + 
                               (self.landmarks[i].y-self.y)*(self.landmarks[i].y-self.y) )
                if r < self.sensing_range:
                    reading = LandmarkReading()
                    reading.landmark = self.landmarks[i]
                    reading.range = r
                    reading.bearing = math.atan2( (self.landmarks[i].y - self.y),
                                                  (self.landmarks[i].x - self.x)) - self.theta
                    sens.readings.append(reading)
        return sens

    def step(self, event):
        self.x = self.x + self.step_size * self.vt * math.cos(self.theta)
        self.y = self.y + self.step_size * self.vt * math.sin(self.theta)
        self.theta = self.theta + self.step_size * self.vrot

    def publish_landmarks(self,event):
        msg = LandmarkSet()
        msg.landmarks = self.landmarks
        self.pub_landmarks.publish(msg)
        
    def publish_pose(self, event):
        msg = Pose2D()
        msg.x = self.x
        msg.y = self.y
        msg.theta = self.theta
        self.pub_pose.publish(msg)
    
    def publish_sensor_data(self, event):
        msg = self.get_sensor_data()
        self.pub_sens.publish(msg)

    def rand_vel(self):
      if math.sqrt(self.x*self.x + self.y*self.y) < 7:
         self.vt = 0.5 + random.random() * 1.0
         self.vrot = (-math.pi + random.random() * 2 * math.pi) / 5.0
      else:
         if (self.x * math.cos(self.theta) + self.y * math.sin(self.theta) < 0):
            self.vt = 0.5 + random.random() * 1.0
            self.vrot = 0.0
         else:
            self.vt = 0.0
            self.vrot = (-math.pi + random.random() * 2 * math.pi) / 5.0         
      self.vt_sens = self.vt + numpy.random.normal(0.0, 0.1)
      self.vrot_sens = self.vrot + numpy.random.normal(0.0, 0.05)


        
    def rand_vel_event(self, event):
       self.rand_vel()
        
if __name__ == '__main__':
    rospy.init_node('mobile_robot_sim', anonymous=True)
    robot = Robot()
    rospy.spin()

