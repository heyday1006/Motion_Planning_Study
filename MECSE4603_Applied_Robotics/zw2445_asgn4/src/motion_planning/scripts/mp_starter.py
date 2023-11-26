#!/usr/bin/env python

import numpy
import sys

import geometry_msgs.msg
import moveit_msgs.msg
import moveit_msgs.srv
import rospy
import tf

def convert_to_message(T):
    t = geometry_msgs.msg.Pose()
    position = tf.transformations.translation_from_matrix(T)
    orientation = tf.transformations.quaternion_from_matrix(T)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[0]
    t.orientation.y = orientation[1]
    t.orientation.z = orientation[2]
    t.orientation.w = orientation[3]        
    return t


class MoveArm(object):

    def __init__(self):

        # Wait for moveit IK service
        rospy.wait_for_service("compute_ik")
        self.ik_service = rospy.ServiceProxy('compute_ik',  moveit_msgs.srv.GetPositionIK)
        print "IK service ready"

        # Wait for validity check service
        rospy.wait_for_service("check_state_validity")
        self.state_valid_service = rospy.ServiceProxy('check_state_validity',  
                                                      moveit_msgs.srv.GetStateValidity)
        print "State validity service ready"

        # MoveIt parameter
        self.group_name = "lwr_arm"


    """ This function will perform IK for a given transform T of the end-effector. It 
    returns a list q[] of 7 values, which are the result positions for the 7 joints of 
    the KUKA arm, ordered from proximal to distal. If no IK solution is found, it 
    returns an empy list.
    """
    def IK(self, T_goal):
        req = moveit_msgs.srv.GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state = moveit_msgs.msg.RobotState()
        req.ik_request.robot_state.joint_state.name = ["lwr_arm_0_joint",
                                                       "lwr_arm_1_joint",
                                                       "lwr_arm_2_joint",
                                                       "lwr_arm_3_joint",
                                                       "lwr_arm_4_joint",
                                                       "lwr_arm_5_joint",
                                                       "lwr_arm_6_joint"]
        req.ik_request.robot_state.joint_state.position = numpy.zeros(7)
        req.ik_request.robot_state.joint_state.velocity = numpy.zeros(7)
        req.ik_request.robot_state.joint_state.effort = numpy.zeros(7)
        req.ik_request.robot_state.joint_state.header.stamp = rospy.get_rostime()
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped = geometry_msgs.msg.PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = "world_link"
        req.ik_request.pose_stamped.header.stamp = rospy.get_rostime()
        req.ik_request.pose_stamped.pose = convert_to_message(T_goal)
        req.ik_request.timeout = rospy.Duration(3.0)
        res = self.ik_service(req)
        q = []
        if res.error_code.val == res.error_code.SUCCESS:
            q = res.solution.joint_state.position
        return q

    """ This function checks if a set of joint angles q[] creates a valid state, or 
    one that is free of collisions. The values in q[] are assumed to be values for 
    the joints of the KUKA arm, ordered from proximal to distal. 
    """
    def is_state_valid(self, q):
        req = moveit_msgs.srv.GetStateValidityRequest()
        req.group_name = self.group_name
        req.robot_state = moveit_msgs.msg.RobotState()
        req.robot_state.joint_state.name = ["lwr_arm_0_joint",
                                            "lwr_arm_1_joint",
                                            "lwr_arm_2_joint",
                                            "lwr_arm_3_joint",
                                            "lwr_arm_4_joint",
                                            "lwr_arm_5_joint",
                                            "lwr_arm_6_joint"]
        req.robot_state.joint_state.position = q
        req.robot_state.joint_state.velocity = numpy.zeros(7)
        req.robot_state.joint_state.effort = numpy.zeros(7)
        req.robot_state.joint_state.header.stamp = rospy.get_rostime()
        res = self.state_valid_service(req)
        return res.valid


if __name__ == '__main__':
    rospy.init_node('move_arm', anonymous=True)
    ma = MoveArm()
    rospy.spin()

