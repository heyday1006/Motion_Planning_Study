#!/usr/bin/env python

import numpy
import sys

import geometry_msgs.msg
from geometry_msgs.msg import Transform
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import moveit_msgs.msg
import moveit_msgs.srv
import rospy
import tf
import tf2_ros
import math

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
        self.joint_names = ["lwr_arm_0_joint",
                            "lwr_arm_1_joint",
                            "lwr_arm_2_joint",
                            "lwr_arm_3_joint",
                            "lwr_arm_4_joint",
                            "lwr_arm_5_joint",
                            "lwr_arm_6_joint"]
        self.length = 1
        self.current_joint_state = JointState()
        rospy.Subscriber("/joint_states", JointState, self.joint_callback)
        rospy.Subscriber('/motion_planning_goal', Transform, self.rrt)
        self.pub = rospy.Publisher("/joint_trajectory", JointTrajectory, queue_size=1) 

    # The actual callback only stores the received information
    def joint_callback(self, joint_state):
        self.current_joint_state = joint_state
    
    # To obtain current joint values in sequence
    def get_joint_values(self):
        joint_values = []
        for i in range(7):
            index = self.current_joint_state.name.index(self.joint_names[i])
            joint_values.append(self.current_joint_state.position[index])
        joint_values = numpy.array(joint_values)
        return joint_values

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
        #print(req.ik_request.pose_stamped.pose,res.error_code.val, res.error_code.SUCCESS)
        if res.error_code.val == res.error_code.SUCCESS:
            q = res.solution.joint_state.position
        else:
            print("IK unavailable")
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
    
   # discretize to check if a line intersects an obstacle
    def path_free_check(self, q_start, q_end):
        new_point = 0
        update = False
        num_samples = int(math.ceil(numpy.linalg.norm(q_end-q_start)/0.1))
        sample_interval = numpy.true_divide(q_end - q_start,num_samples)
        # economic verison 
        for j in range(1,num_samples+1):
            sample = q_start + j * sample_interval
            if self.is_state_valid(sample) == True:
                update = True
                new_point = sample
            else:
                break
        self.new_point = new_point
        return update
    
    def goal_free_check(self, q_start,q_end):
        update = True
        num_samples = int(math.ceil(numpy.linalg.norm(q_end-q_start)/0.1))
        sample_interval = numpy.true_divide(q_end - q_start,num_samples)
        for j in range(1,num_samples):
            sample = q_start + j * sample_interval
            if self.is_state_valid(sample) == False:
                update = False
                break
        return update

    # implement the Rapidly-exploring Random Trees algorithm 
    # for motion planning in configuration space
    def rrt(self, T_goal):
        path = []
        trajectory = []
        # transform of T_goal
        Tgoal_rot=tf.transformations.quaternion_matrix((T_goal.rotation.x,T_goal.rotation.y,T_goal.rotation.z,T_goal.rotation.w))
        Tgoal_trans=tf.transformations.translation_matrix((T_goal.translation.x,T_goal.translation.y,T_goal.translation.z))
        T_goal=numpy.dot(Tgoal_trans,Tgoal_rot)
        # inverse kinematics
        q_goal = self.IK(T_goal)
        q_goal = numpy.array(q_goal)
        # the information of the starting node
        # use the dictionary to complement new node position with its parent node tag
        q_start = self.get_joint_values()
        node = {"joint_position": q_start, "parent": -1}
        path.append(node.copy())
        # continue until path to goal found
        while True:
            q_rand = numpy.random.uniform(-numpy.pi, numpy.pi, 7)
            # find the closest point around q_rand in vertices
            distance = [numpy.linalg.norm(q_rand - q) for n,q in enumerate(element["joint_position"] for element in path)]
            min_index = distance.index(min(distance))
            q_near = path[min_index].get("joint_position")
            # add branch of predifined length from q_near in the direction of q_rand
            q_new = q_near + self.length * (q_rand-q_near)/numpy.linalg.norm(q_rand-q_near)
            # if the new node /partial of new node is in the free space, add it to vertices
            if self.path_free_check(q_near,q_new) == True:
                node["joint_position"] = self.new_point
                node["parent"] = min_index
                print("new state:",self.is_state_valid(self.new_point))
                path.append(node.copy())
                # if q_new can directly connect to q_goal in the free space, path found
                if self.goal_free_check(self.new_point, q_goal) == True:
                    node["joint_position"] = q_goal
                    node["parent"] = len(path) - 1
                    path.append(node.copy())
                    print("GOAL MET")
                    break
        # trace back to get the trajectory
        trajectory.append(q_goal)
        parent = path[-1].get("parent")
        while True:
            trajectory.append(path[parent].get("joint_position"))
            if parent == 0:
                break
            else:
                parent = path[parent].get("parent")
        # shortcut the original path
        new_path = self.shortcut(trajectory)
        # print("shortcut:",new_path)
        # reverse the list to start at q_start
        new_path.reverse()
        # resample the trajectory in cartesian space
        new_path = self.resampling(new_path)
        #print("new_path",new_path)
        for i in range(len(new_path)):
            print("end state:",self.is_state_valid(new_path[i]))
        # publish the trajectory
        self.motion_planning(new_path)
    
    # publish the trajectory
    def motion_planning(self, path):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        for i in range(len(path)):
            waypoint = JointTrajectoryPoint()
            waypoint.positions = numpy.ndarray.tolist(path[i])
            trajectory.points.append(waypoint)     
        self.pub.publish(trajectory)

    # resample the trajectory in cartesian space
    # no consecutive points on the path are farther than 0.5 to each other
    def resampling(self,path):
        new_path = [path[0]]
        for i in range(1,len(path)):
            num_samples = int(math.ceil(numpy.linalg.norm(path[i]-path[i-1])/0.4))
            sample_interval = numpy.true_divide(path[i] - path[i-1], num_samples)
            sample_list = [(path[i-1] + j * sample_interval) for j in range(1, num_samples+1)]
            new_path += sample_list
        return new_path

    # shortcut the orginal path returned by RRT
    # note the path is in trace back version of trajectory
    def shortcut(self, path):
        new_path = [path[0]]
        i = 0
        while i < len(path)-1:
            shortcut_idx = len(path) - 1
            while i < shortcut_idx:
                if self.goal_free_check(path[i],path[shortcut_idx]) == True:
                    new_path.append(path[shortcut_idx])
                    i = shortcut_idx
                else:
                    shortcut_idx -= 1
        return new_path

if __name__ == '__main__':
    rospy.init_node('move_arm', anonymous=True)
    ma = MoveArm()
    rospy.spin()

