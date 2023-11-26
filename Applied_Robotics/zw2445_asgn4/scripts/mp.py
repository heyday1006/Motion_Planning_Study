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
import time
from urdf_parser_py.urdf import URDF
from angle_axis import*

class MoveArm(object):

    def __init__(self):
  
        # Wait for validity check service
        rospy.wait_for_service("check_state_validity")
        self.state_valid_service = rospy.ServiceProxy('check_state_validity',  
                                                      moveit_msgs.srv.GetStateValidity)
        print "State validity service ready"

        # Loads the robot model, which contains the robot's kinematics information
        self.robot = URDF.from_parameter_server()
        self.get_joint_info()
        # MoveIt parameter
        self.group_name = "lwr_arm"
        self.length = 1
        self.current_joint_state = JointState()
        rospy.Subscriber("/joint_states", JointState, self.joint_callback)
        rospy.Subscriber('/motion_planning_goal', Transform, self.rrt)
        self.pub = rospy.Publisher("/joint_trajectory", JointTrajectory, queue_size=1) 

    # The actual callback only stores the received information
    def joint_callback(self, joint_state):
        self.current_joint_state = joint_state
    
    # To obtain joint information 
    def get_joint_info(self):
        self.num_joints = 0
        self.joint_names = []
        self.joint_axes = []
        link = self.robot.get_root()
        while True:
            if link not in self.robot.child_map: break            
            (joint_name, next_link) = self.robot.child_map[link][0]
            current_joint = self.robot.joint_map[joint_name]        
            if current_joint.type != 'fixed':
                self.num_joints = self.num_joints + 1
                self.joint_names.append(current_joint.name)
                self.joint_axes.append(current_joint.axis)
            link = next_link
    
    # To obtain current joint values in sequence
    def get_joint_values(self):
        joint_values = []
        for i in range(7):
            index = self.current_joint_state.name.index(self.joint_names[i])
            joint_values.append(self.current_joint_state.position[index])
        joint_values = numpy.array(joint_values)
        return joint_values

    # Uses the information passed in under join_state as well as that
    # in self.joint_names and self.joint_axes to compute
    # the list joint_transforms as well as the current end-effector pose x_current.
    def process_link_recursive(self, link, T, joint_values):   
       if link not in self.robot.child_map: 
           self.x_current = T
           return
       for i in range(0,len(self.robot.child_map[link])):
           (joint_name, next_link) = self.robot.child_map[link][i]
           if joint_name not in self.robot.joint_map:
               rospy.logerror("Joint not found in map")
               continue
           current_joint = self.robot.joint_map[joint_name]        

           trans_matrix = tf.transformations.translation_matrix((current_joint.origin.xyz[0], 
                                                                 current_joint.origin.xyz[1],
                                                                 current_joint.origin.xyz[2]))
           rot_matrix = tf.transformations.euler_matrix(current_joint.origin.rpy[0], 
                                                        current_joint.origin.rpy[1],
                                                        current_joint.origin.rpy[2], 'rxyz')
           origin_T = numpy.dot(trans_matrix, rot_matrix)
           current_joint_T = numpy.dot(T, origin_T)
           if current_joint.type != 'fixed':
               if current_joint.name not in joint_values.name:
                   rospy.logerror("Joint not found in list")
                   continue

               self.joint_transforms.append(current_joint_T)
               index = joint_values.name.index(current_joint.name)
               angle = joint_values.position[index]
               joint_rot_T = tf.transformations.rotation_matrix(angle, 
                                                                numpy.asarray(current_joint.axis))
               next_link_T = numpy.dot(current_joint_T, joint_rot_T) 
           else:
               next_link_T = current_joint_T

           self.process_link_recursive(next_link, next_link_T, joint_values) 

    # Forward kinematics used by IK
    def forward_kinematics(self,jointsdata):
        root = self.robot.get_root()
        T = tf.transformations.identity_matrix()
        self.process_link_recursive(root, T, jointsdata)

    # calculate pseudoinverse of Jacobian
    def Jac_inv(self,x_desired,jointsdata):
        self.joint_transforms=[]
        self.forward_kinematics(jointsdata)
        V={}
        J=numpy.zeros((6,self.num_joints))
        
        #desired transform from base to {ee}
        Tb_ee_des = x_desired
        Tb_ee_des_rot=tf.transformations.quaternion_matrix((Tb_ee_des.rotation.x,Tb_ee_des.rotation.y,Tb_ee_des.rotation.z,Tb_ee_des.rotation.w))
        Tb_ee_trans=tf.transformations.translation_matrix((Tb_ee_des.translation.x,Tb_ee_des.translation.y,Tb_ee_des.translation.z))
        Tb_ee_des=numpy.dot(Tb_ee_trans,Tb_ee_des_rot)
        #current transform from base to {ee}
        Tb_ee_cur = self.x_current
        T_cur = self.joint_transforms
        
        Tcur_des=numpy.dot(tf.transformations.inverse_matrix(Tb_ee_cur),Tb_ee_des)
        # desired translation from {ee} to desired one
        trans_des=Tcur_des[:3,3]
        # change in orientation
        angle, axis=rotation_from_matrix(Tcur_des)
        rot_des=axis*angle
        # proportional control on {ee} velocity
        # scaling v_ee such that not exceed 0.1/s for translation and 1rad/s for rotation 
        x_ee=numpy.zeros((6,1))
        # change of x
        x_ee[:3,0]=trans_des
        x_ee[3:,0]=rot_des

        #Jacobian
        for k in range(self.num_joints):
           Tee_j=numpy.dot(tf.transformations.inverse_matrix(Tb_ee_cur),T_cur[k])
           Tj_ee=numpy.dot(tf.transformations.inverse_matrix(T_cur[k]), Tb_ee_cur)
           V_j=numpy.zeros((6,6))
           t=Tj_ee[:3,3]
           R=Tee_j[:3,:3]
           S_t=[[0.0,-t[2],t[1]],[t[2],0.0,-t[0]],[-t[1],t[0],0.0]]
           V_j[:3,:3]=R
           V_j[:3,3:]=-numpy.dot(R,S_t)
           V_j[3:,3:]=R
           
           #given the joint is revolute,only rotation axis taken into account
           if self.joint_axes[k] == [1.0,0.0,0.0]:
                J[:,k]=V_j[:,3]
           elif self.joint_axes[k] == [-1.0,0.0,0.0]:
                J[:,k]=-V_j[:,3]
           elif self.joint_axes[k] == [0.0,1.0,0.0]:
                J[:,k]=V_j[:,4]
           elif self.joint_axes[k] == [0.0,-1.0,0.0]:
                J[:,k]=-V_j[:,4]
           elif self.joint_axes[k] == [0.0,0.0,1.0]:
                J[:,k]=V_j[:,5]
           else:
                J[:,k]=-V_j[:,5]
      
        #pseudoinverse of Jacobian
        J_inv=numpy.linalg.pinv(J)
        
        #pseudoinverse of Jacobian with threshold
        Js_inv=numpy.linalg.pinv(J,1e-2)
        self.x_ee=x_ee
        self.Js_inv=Js_inv

    # In some cases even succeed, robot cannot reach the target as exceed the limit
    # If out out limits, return an empty list, print "IK unavailable"
    def IK(self,ikcommand):
        #print(ikcommand)
        num_tries=0
        max_steps=5
        x_target=ikcommand
        q_data=JointState()
        succeed=False
        q= []
        while succeed==False and num_tries<max_steps:
            q_current=numpy.random.uniform(-numpy.pi, numpy.pi, (7,1))
            start_time=time.time()
            while True:
                # to minimize xd-FWD(qc), use Newton Raphson method
                q_data.name=self.joint_names
                q_data.position=q_current
                self.Jac_inv(x_target,q_data)
                self.forward_kinematics(q_data)
                # difference btw target and current
                xdot=self.x_ee
                # if difference is small, succeed. if time great, break.
                if numpy.linalg.norm(xdot)<0.01:
                    succeed=True
                    print('succeed',q_data.position)
                    break
                elif time.time()-start_time>3:
                    print('time elapse')
                    break 
                else:
                    qdot=numpy.dot(self.Js_inv,xdot)
                    q_current+=qdot
            num_tries+=1
            print('num_tries',num_tries)
            if num_tries>=max_steps:
                print('max_steps met')
        # q_goal is in the range(-pi,pi)
        if succeed == True:
            for i in range(7):
                number = q_data.position[i][0]
                number %= 2*numpy.pi
                if number>numpy.pi:
                    number -= 2*numpy.pi
                elif number< -numpy.pi:
                    number += 2*numpy.pi 
                q.append(number) 
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
        # inverse kinematics
        q_goal = self.IK(T_goal)
        q_goal = numpy.array(q_goal)
        # the information of the starting node
        # use the dictionary to complement new node position with its parent node tag
        q_start = self.get_joint_values()
        print("q_start, q_goal", q_start, q_goal)
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

