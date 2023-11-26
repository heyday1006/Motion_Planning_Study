#!/usr/bin/env python
import rospy
import numpy
import tf
import tf2_ros
import time
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from cartesian_control.msg import CartesianCommand
from geometry_msgs.msg import Transform
from urdf_parser_py.urdf import URDF
from angle_axis import*

#gain in translation and orientation
p_1=0.5
p_2=0.6
#p_1=1.0
#p_2=1.0
p_sec=0.6

class CartesianControl(object):

    def __init__(self):
        # Loads the robot model, which contains the robot's kinematics information
        self.robot = URDF.from_parameter_server()
        
        # This is where we hold general info about the joints
        self.num_joints = 0
        self.joint_names = []
        self.joint_axes = []
        self.joint_transforms = []
        # Prepare general information about the robot
        self.get_joint_info()
        #Jacobian
        self.J={}
        self.J_inv={}
        self.Js_inv={}
        self.x_ee=numpy.zeros((6,1))
        self.v_ee=numpy.zeros((6,1))
        
        # This is where we'll hold the most recent joint angle information we receive on the topic
        self.current_joint_state = JointState()

        # Subscribes to information about what the current joint values are.
        rospy.Subscriber("/joint_states", JointState, self.joint_callback)
        rospy.Subscriber('/cartesian_command', CartesianCommand, self.cartesian_control)
        rospy.Subscriber('/ik_command', Transform, self.ik_call)

    # The actual callback only stores the received information
    def joint_callback(self, joint_state):
        self.current_joint_state = joint_state

    # forward kinematics used by cartesian control and IK
    def forward_kinematics(self,jointsdata):
        root = self.robot.get_root()
        T = tf.transformations.identity_matrix()
        self.process_link_recursive(root, T, jointsdata)

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

    # Uses the information passed in under join_state as well as that
    # in self.joint_names and self.joint_axes to compute
    # the list joint_transforms as well as the current end-effector pose x_current.
    # Both Cartesian Control and IK will make use of this.
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
        v_ee=numpy.zeros((6,1))
        # change of x
        x_ee[:3,0]=trans_des
        x_ee[3:,0]=rot_des
        # desired velocity
        v_ee[:3,0]=p_1*x_ee[:3,0]
        v_ee[3:,0]=p_2*x_ee[3:,0]
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
        self.v_ee=v_ee
        self.J=J
        self.J_inv=J_inv
        self.Js_inv=Js_inv

    def cartesian_control(self,cartesiancommand):
        self.Jac_inv(cartesiancommand.x_target,self.current_joint_state)
        qdot_des=numpy.dot(self.Js_inv,self.v_ee)  
        print('velocity: ',self.v_ee)
       
        if cartesiancommand.secondary_objective==True:
           #execute the secondary objective
           q0_desired=cartesiancommand.q0_target
           qdot_sec=numpy.zeros((len(qdot_des),1))   
           q0_current=self.current_joint_state.position[self.current_joint_state.name.index(self.joint_names[0])]
           qdot_sec[0]=p_sec*(q0_desired-q0_current)
           I=numpy.identity(len(qdot_des))
           qdot_null=numpy.dot(I-numpy.dot(self.J_inv,self.J),qdot_sec)
           qdot_des+=qdot_null
           #print('qdot_des: ',qdot_des)
        jointstate=JointState()
        jointstate.name=self.joint_names
        jointstate.velocity=qdot_des
        pub = rospy.Publisher('/joint_velocities', JointState, queue_size=1)
        pub.publish(jointstate)
    
    #in some cases even succeed, robot cannot reach the target as exceed the limit
    def ik_call(self,ikcommand):
        #print(ikcommand)
        num_tries=0
        max_steps=10
        x_target=ikcommand
        q_data=JointState()
        succeed=False
        while succeed==False and num_tries<max_steps:
            q_current=numpy.random.rand(self.num_joints,1)
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
                elif time.time()-start_time>10:
                    print('time elapse')
                    break 
                else:
                    qdot=numpy.dot(self.Js_inv,xdot)
                    q_current+=qdot
                    #print('update',q_data.position)
            num_tries+=1
            print('num_tries',num_tries)
            if num_tries>=max_steps:
                print('max_steps met')
        pub = rospy.Publisher('/joint_command', JointState, queue_size=1)
        pub.publish(q_data)
                        
    
if __name__ == '__main__':
    rospy.init_node('ccik', anonymous=True)
    CartesianControl()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

