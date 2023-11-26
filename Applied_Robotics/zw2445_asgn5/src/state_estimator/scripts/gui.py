#!/usr/bin/env python

# Columbia Engineering
# MECS 4603 - Fall 2017

import math
import numpy
import time
import matplotlib.pyplot as plt
import threading

import rospy

from geometry_msgs.msg import Pose2D
from state_estimator.msg import Landmark
from state_estimator.msg import LandmarkSet
from state_estimator.msg import LandmarkReading
from state_estimator.msg import SensorData

class GUI(object):
    def __init__(self):
        self._fig = plt.figure(figsize=(10,10))
        self._ax1 = self._fig.add_subplot(1,1,1)
        plt.show(False)
        
        self.robot_pose = Pose2D()
        self.robot_pose_est = Pose2D()
        self.landmarks = []
        self.readings = []

        rospy.Subscriber("/robot_pose", Pose2D, self.pose_callback)
        rospy.Subscriber("/robot_pose_estimate", Pose2D, self.est_callback)
        rospy.Subscriber("/landmarks", LandmarkSet, self.landmarks_callback)
        rospy.Subscriber("/sensor_data", SensorData, self.sensor_callback)
    
    def sensor_callback(self, msg):
        self.readings = msg.readings
    
    def pose_callback(self, msg):
        self.robot_pose = msg

    def est_callback(self, msg):
        self.robot_pose_est = msg

    def landmarks_callback(self, msg):
        self.landmarks = msg.landmarks

    def update(self, event):
        self._ax1.clear()
        plt.xlim(-10,10)
        plt.ylim(-10,10)
        ptx=[self.robot_pose.x]
        pty=[self.robot_pose.y]
        self._ax1.scatter(ptx, pty, c='r', s = 64);
        ptx=[self.robot_pose.x+0.2*math.cos(self.robot_pose.theta)]
        pty=[self.robot_pose.y+0.2*math.sin(self.robot_pose.theta)]
        self._ax1.scatter(ptx, pty, c='r', s = 16);
        
        ptx=[self.robot_pose_est.x]
        pty=[self.robot_pose_est.y]
        self._ax1.scatter(ptx, pty, c='b', s = 64);
        ptx=[self.robot_pose_est.x+0.2*math.cos(self.robot_pose_est.theta)]
        pty=[self.robot_pose_est.y+0.2*math.sin(self.robot_pose_est.theta)]
        self._ax1.scatter(ptx, pty, c='b', s = 16);
        
        ptx = []
        pty = []
        for i in range(0,len(self.landmarks)):
            ptx.append(self.landmarks[i].x)
            pty.append(self.landmarks[i].y)
        self._ax1.scatter(ptx, pty, c='b', marker='*', s = 64);
        ptx = []
        pty = []
        for i in range(0,len(self.readings)):
            ptx.append(self.readings[i].landmark.x)
            pty.append(self.readings[i].landmark.y)
        self._ax1.scatter(ptx, pty, c='r', marker='*', s = 64);
        self._fig.canvas.draw()

def my_thread():
    rospy.spin()
    
if __name__ == '__main__':
    rospy.init_node('mobile_robot_gui', anonymous=True)
    gui = GUI()
     
    my_thread = threading.Thread(target=my_thread)
    my_thread.daemon = True
    my_thread.start()
 
    while not rospy.is_shutdown():
        gui.update("foo")
        time.sleep(0.01)

