#!/usr/bin/env python

import math
import numpy
import time

import rospy
import tf
from geometry_msgs.msg import Transform
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from urdf_parser_py.urdf import URDF

from cartesian_control.msg import CartesianCommand

def S_matrix(w):
    S = numpy.zeros((3,3))
    S[0,1] = -w[2]
    S[0,2] =  w[1]
    S[1,0] =  w[2]
    S[1,2] = -w[0]
    S[2,0] = -w[1]
    S[2,1] =  w[0]
    return S

def get_column_index(joint_axis):
    column = 0
    mult = 1
    if joint_axis[0] == 1:
        column = 0
        mult = 1
    elif joint_axis[0] == -1:
        column = 0
        mult = -1
    elif joint_axis[1] == 1:
        column = 1
        mult = 1
    elif joint_axis[1] == -1:
        column = 1
        mult = -1
    elif joint_axis[2] == 1:
        column = 2
        mult = 1
    elif joint_axis[2] == -1:
        column = 2
        mult = -1
    else:
        print "Wrong axis!!!"
        print joint_axis
    return (column, mult)
    

# This is the function that must be filled in as part of the Project.
def cartesian_control(joint_transforms, joint_axes, b_T_ee_current, b_T_ee_desired,
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
        # choose the right column to put into Jacobian
        (column, mult) = get_column_index(joint_axes[i])
        J[0][i] = -RS[0][column] * mult
        J[1][i] = -RS[1][column] * mult
        J[2][i] = -RS[2][column] * mult
        J[3][i] = e_R_j[0][column] * mult
        J[4][i] = e_R_j[1][column] * mult
        J[5][i] = e_R_j[2][column] * mult
        
    # compute Jacobian pseudo-inverse, discarding motion for SV's below a threshold
    Jps = numpy.linalg.pinv(J,1.0e-2)
    # multiply by desired end-effector velocity to get joint velocities
    dq = numpy.dot(Jps, dv_e)
    
    if (red_control):
        # assemble desired joint velocity to be projected in null space
        q_des = numpy.zeros(num_joints)
        print "Des: " + str(q0_desired) + "; cur: " + str(q_current[0])
        q_des[0] = 3 * (q0_desired - q_current[0])
        # compute Jacobian pseudo-inverse with all SVDs
        Jp = numpy.linalg.pinv(J)
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
        
        # This is where we hold general info about the joints
        self.joint_names = []
        self.joint_axes = []

        # Prepare general information about the robot
        self.get_joint_info()
        
        #Subscribes to information about what the current joint values are.
        rospy.Subscriber("/joint_states", JointState, self.joint_callback)

        #Subscribes to command for end-effector pose
        rospy.Subscriber("/cartesian_command", CartesianCommand, self.command_callback)

        #Subscribes to command for end-effector pose using numerical IK
        rospy.Subscriber("/ik_command", Transform, self.IK_command_callback)

        # Publishes desired joint velocities
        self.pub_vel = rospy.Publisher("/joint_velocities", JointState, queue_size=1)

        # Publishes direct command for positions
        self.pub_command = rospy.Publisher("/joint_command", JointState, queue_size=1)

        #This is where we hold the most recent joint info
        self.current_joint_state = JointState()

    def command_callback(self, command):
        if len(self.current_joint_state.position) == 0: return
        x_target = convert_from_message(command.x_target)
        (joint_transforms, x_current) = self.get_joint_transforms(self.current_joint_state)
        q_current = self.current_joint_state.position
        dq = cartesian_control(joint_transforms, self.joint_axes, 
                               x_current, x_target,
                               command.secondary_objective, q_current, command.q0_target)
        msg = JointState()
        msg.name = self.joint_names
        msg.velocity = dq
        self.pub_vel.publish(msg)

    def joint_callback(self, joint_state):
        self.current_joint_state = joint_state

    def IK_command_callback(self, command):
        x_target = convert_from_message(command)
        num_tries = 3
        tries = 0
        while tries < num_tries:
            if self.numerical_IK(x_target, 2): break
            tries = tries + 1

    def numerical_IK(self, x_target, timeout):
        q_current = numpy.random.rand(self.num_joints)
        success = False
        start_time = time.time()
        joint_state = JointState()
        joint_state.name = self.joint_names
        while True:
            joint_state.position = q_current
            (joint_transforms, x_current) = self.get_joint_transforms(joint_state)
            if same_transform(x_current, x_target, 1.0e-2):
                success = True
                break
            if time.time() - start_time > timeout:
                break
            dq = cartesian_control(joint_transforms, self.joint_axes,
                                   x_current, x_target, False, q_current, 0)
            q_current = q_current + dq
        if success:
            msg = JointState()
            msg.name = self.joint_names
            msg.position = q_current
            self.pub_command.publish(msg)
        else:
            print "IK timed out..."
        return success

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

    def get_joint_transforms(self, joint_state):
        root = self.robot.get_root()
        T = tf.transformations.identity_matrix()
        joint_transforms = []
        x_current = self.get_joint_transforms_recursive(root, T, joint_state, joint_transforms)
        return (joint_transforms, x_current)
    
    def get_joint_transforms_recursive(self, link, T, joint_state, joint_transforms):
        if link not in self.robot.child_map: 
            return T
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
                if current_joint.name not in joint_state.name:
                    rospy.logerror("Joint not found in list")
                    continue
                joint_transforms.append(current_joint_T)
                index = joint_state.name.index(current_joint.name)
                angle = joint_state.position[index]
                joint_rot_T = tf.transformations.rotation_matrix(angle, 
                                                                 numpy.asarray(current_joint.axis))
                next_link_T = numpy.dot(current_joint_T, joint_rot_T) 
            else:
                next_link_T = current_joint_T

            return self.get_joint_transforms_recursive(next_link, next_link_T, joint_state, joint_transforms)
        
if __name__ == '__main__':
    rospy.init_node('cartesian_control', anonymous=True)
    cc = CartesianControl()
    rospy.spin()
