#!/usr/bin/env python

import math
import numpy
import time
from threading import Thread, Lock

import rospy
import tf
from geometry_msgs.msg import Transform
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from urdf_parser_py.urdf import URDF

def S_matrix(w):
    S = numpy.zeros((3,3))
    S[0,1] = -w[2]
    S[0,2] =  w[1]
    S[1,0] =  w[2]
    S[1,2] = -w[0]
    S[2,0] = -w[1]
    S[2,1] =  w[0]
    return S

# This is the function that must be filled in as part of the Project.
def cartesian_control(joint_transforms, b_T_ee_current, b_T_ee_desired,
                      red_control, q_current, q0_desired):
    num_joints = len(joint_transforms)
    dq = numpy.zeros(num_joints)
    #-------------------- Fill in your code here ---------------------------
    # compute transform from current end-effector pose to desired one
    w_T_e = b_T_ee_current
    w_T_des = b_T_ee_desired
    c_T_des = numpy.dot( tf.transformations.inverse_matrix(w_T_e), w_T_des )
    # get desired translational velocity in local frame
    dx_e = numpy.zeros(3)
    dx_e[0] = c_T_des[0][3]
    dx_e[1] = c_T_des[1][3]
    dx_e[2] = c_T_des[2][3]
    # normalize to obtain max end-effector velocity of 0.1m/s
    if numpy.linalg.norm(dx_e) > 0.1:
        dx_e = (0.1 / numpy.linalg.norm(dx_e)) * dx_e

    # get desired angular velocity in local frame
    c_R_des = c_T_des[0:3,0:3]
    angle, axis = rotation_from_matrix(c_R_des)
    dw_e = angle * axis
    # normalize to max end-effector angular velocity of 1 rad/s
    if numpy.linalg.norm(dw_e) > 1.0:
        dw_e = (1.0 / numpy.linalg.norm(dw_e)) * dw_e

    #assamble translational and angular velocities
    dv_e = numpy.zeros(6)
    dv_e[0:3] = dx_e
    dv_e[3:6] = dw_e
        
    # assemble Jacobian
    J = numpy.zeros((6,num_joints))
    for i in range(0,len(joint_transforms)):
        # rigid body transfer of joint rotation to end-effector motion
        w_T_j = joint_transforms[i]
        j_T_e = numpy.dot( tf.transformations.inverse_matrix(w_T_j), w_T_e)
        e_T_j = tf.transformations.inverse_matrix(j_T_e)
        e_R_j = e_T_j[0:3,0:3]
        j_trans_e = j_T_e[0:3,3]
        RS = numpy.dot( e_R_j, S_matrix(j_trans_e) )
        # since all joints rotate along local z, we only care about the third column
        J[0][i] = -RS[0][2]
        J[1][i] = -RS[1][2]
        J[2][i] = -RS[2][2]
        J[3][i] = e_R_j[0][2]
        J[4][i] = e_R_j[1][2]
        J[5][i] = e_R_j[2][2]
        
    # compute Jacobian pseudo-inverse, discarding motion for SV's below a threshold
    Jp = numpy.linalg.pinv(J,1.0e-2)
    # multiply by desired end-effector velocity to get joint velocities
    dq = numpy.dot(Jp, dv_e)
    
    if (red_control):
        # assemble desired joint velocity to be projected in null space
        q_des = numpy.zeros(num_joints)
        q_des[0] = q0_desired - q_current[0]
        # build null-space projection matrix
        N = numpy.identity(num_joints) - numpy.dot(Jp, J)
        # multiply by desired joint velocity
        q_null = numpy.dot(N, q_des)
        # add to final resulting velocity
        dq = dq + q_null
    
    # scale joint velocities down to be below limit
    max_dq = 0
    for i in range(0,len(dq)):
        if abs(dq[i]) > max_dq: max_dq = abs(dq[i])
    if max_dq > 1.0:
        for i in range(0,len(dq)): dq[i] = dq[i] / max_dq

    #----------------------------------------------------------------------
    return dq
    
def convert_from_message(t):
    trans = tf.transformations.translation_matrix((t.translation.x,
                                                  t.translation.y,
                                                  t.translation.z))
    rot = tf.transformations.quaternion_matrix((t.rotation.x,
                                                t.rotation.y,
                                                t.rotation.z,
                                                t.rotation.w))
    T = numpy.dot(trans,rot)
    return T

# Returns the angle-axis representation of the rotation contained in the input matrix
# Use like this:
# angle, axis = rotation_from_matrix(R)
def rotation_from_matrix(matrix):
    R = numpy.array(matrix, dtype=numpy.float64, copy=False)
    R33 = R[:3, :3]
    # axis: unit eigenvector of R33 corresponding to eigenvalue of 1
    l, W = numpy.linalg.eig(R33.T)
    i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
    if not len(i):
        raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
    axis = numpy.real(W[:, i[-1]]).squeeze()
    # point: unit eigenvector of R33 corresponding to eigenvalue of 1
    l, Q = numpy.linalg.eig(R)
    i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
    if not len(i):
        raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
    # rotation angle depending on axis
    cosa = (numpy.trace(R33) - 1.0) / 2.0
    if abs(axis[2]) > 1e-8:
        sina = (R[1, 0] + (cosa-1.0)*axis[0]*axis[1]) / axis[2]
    elif abs(axis[1]) > 1e-8:
        sina = (R[0, 2] + (cosa-1.0)*axis[0]*axis[2]) / axis[1]
    else:
        sina = (R[2, 1] + (cosa-1.0)*axis[1]*axis[2]) / axis[0]
    angle = math.atan2(sina, cosa)
    return angle, axis

def same_transform(T1, T2, eps):
    trans1 = tf.transformations.translation_from_matrix(T1)
    trans2 = tf.transformations.translation_from_matrix(T2)
    rot1 = tf.transformations.quaternion_from_matrix(T1)
    rot2 = tf.transformations.quaternion_from_matrix(T2)
    return (abs(trans1 - trans2) < eps).all() and (abs(rot1 - rot2) < eps).all()

class CartesianControl(object):

    #Initialization
    def __init__(self):
        #Loads the robot model, which contains the robot's kinematics information
        self.robot = URDF.from_parameter_server()

        #Subscribes to information about what the current joint values are.
        rospy.Subscriber("/joint_states", JointState, self.joint_callback)

        #Subscribes to command for end-effector pose
        rospy.Subscriber("/cartesian_command", Transform, self.command_callback)

        #Subscribes to command for end-effector pose using numerical IK
        rospy.Subscriber("/ik_command", Transform, self.IK_command_callback)

        #Subscribes to command for redundant dof
        rospy.Subscriber("/redundancy_command", Float32, self.redundancy_callback)

        # Publishes desired joint velocities
        self.pub_vel = rospy.Publisher("/joint_velocities", JointState, queue_size=1)

        # Publishes direct command for positions
        self.pub_command = rospy.Publisher("/joint_command", JointState, queue_size=1)

        #This is where we hold the most recent joint transforms
        self.joint_transforms = []
        self.joint_names = []
        self.q_current = []
        self.x_current = tf.transformations.identity_matrix()
        self.R_base = tf.transformations.identity_matrix()
        self.x_target = tf.transformations.identity_matrix()
        self.q0_desired = 0
        self.last_command_time = 0
        self.last_red_command_time = 0

        # Initialize timer that will trigger callbacks
        self.mutex = Lock()
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def command_callback(self, command):
        self.mutex.acquire()
        self.x_target = convert_from_message(command)
        self.last_command_time = time.time()
        self.mutex.release()

    def redundancy_callback(self, command):
        self.mutex.acquire()
        self.q0_desired = command.data
        self.last_red_command_time = time.time()
        self.mutex.release()        
        
    def timer_callback(self, event):
        msg = JointState()
        if len(self.joint_transforms) == 0: return
        self.mutex.acquire()
        if time.time() - self.last_command_time < 0.5:
            dq = cartesian_control(self.joint_transforms, 
                                   self.x_current, self.x_target,
                                   False, self.q_current, self.q0_desired)
            msg.name = self.joint_names
            msg.velocity = dq
        elif time.time() - self.last_red_command_time < 0.5:
            dq = cartesian_control(self.joint_transforms, 
                                   self.x_current, self.x_current,
                                   True, self.q_current, self.q0_desired)
            msg.name = self.joint_names
            msg.velocity = dq
        else:            
            msg.velocity = numpy.zeros(self.get_num_joints())
        self.mutex.release()
        self.pub_vel.publish(msg)
        
    def joint_callback(self, joint_values):
        root = self.robot.get_root()
        T = tf.transformations.identity_matrix()
        self.mutex.acquire()
        self.joint_transforms = []
        self.joint_names = []
        self.q_current = joint_values.position
        self.process_link_recursive(root, T, joint_values)
        self.mutex.release()

    def align_with_z(self, axis):
        T = tf.transformations.identity_matrix()
        z = numpy.array([0,0,1])
        x = numpy.array([1,0,0])
        dot = numpy.dot(z,axis)
        if dot == 1: return T
        if dot == -1: return tf.transformation.rotation_matrix(math.pi, x)
        rot_axis = numpy.cross(z, axis)
        angle = math.acos(dot)
        return tf.transformations.rotation_matrix(angle, rot_axis)

    def IK_command_callback(self, command):
        self.mutex.acquire()
        self.x_target = convert_from_message(command)
        num_tries = 3
        tries = 0
        while tries < num_tries:
            if self.numerical_IK(2): break
            tries = tries + 1
        self.mutex.release()

    def numerical_IK(self, timeout):
        q_current = numpy.random.rand(self.get_num_joints())
        success = False
        start_time = time.time()
        root = self.robot.get_root()
        joint_state = JointState()
        joint_state.name = self.joint_names
        while True:
            T = tf.transformations.identity_matrix()
            joint_state.position = q_current
            self.joint_transforms = []
            self.joint_names = []
            self.process_link_recursive(root, T, joint_state)
            if same_transform(self.x_current, self.x_target, 1.0e-2):
                success = True
                break
            if time.time() - start_time > timeout:
                break
            dq = cartesian_control(self.joint_transforms,
                                   self.x_current, self.x_target, False, q_current, 0)
            q_current = q_current + dq
        if success:
            msg = JointState()
            msg.name = self.joint_names
            msg.position = q_current
            self.pub_command.publish(msg)
        else:
            print "IK timed out..."
        return success

    def get_num_joints(self):
        num_joints = 0
        link = self.robot.get_root()
        while True:
            if link not in self.robot.child_map: break            
            (joint_name, next_link) = self.robot.child_map[link][0]
            current_joint = self.robot.joint_map[joint_name]        
            if current_joint.type != 'fixed': num_joints = num_joints + 1
            link = next_link
        return num_joints
                    
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
                # compute transform that aligns rotation axis with z
                aligned_joint_T = numpy.dot(current_joint_T, self.align_with_z(current_joint.axis))
                self.joint_transforms.append(aligned_joint_T)
                self.joint_names.append(joint_name)
                index = joint_values.name.index(current_joint.name)
                angle = joint_values.position[index]
                joint_rot_T = tf.transformations.rotation_matrix(angle, 
                                                                 numpy.asarray(current_joint.axis))
                next_link_T = numpy.dot(current_joint_T, joint_rot_T) 
            else:
                next_link_T = current_joint_T

            self.process_link_recursive(next_link, next_link_T, joint_values)
        
if __name__ == '__main__':
    rospy.init_node('cartesian_control', anonymous=True)
    cc = CartesianControl()
    rospy.spin()
