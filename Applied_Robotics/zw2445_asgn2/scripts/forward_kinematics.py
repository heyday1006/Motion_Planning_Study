#!/usr/bin/env python
import rospy
import numpy
import tf
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF

def message_from_transform(T):
    msg=geometry_msgs.msg.Transform()
    q=tf.transformations.quaternion_from_matrix(T)
    translation=tf.transformations.translation_from_matrix(T)
    msg.translation.x=translation[0]
    msg.translation.y=translation[1]
    msg.translation.z=translation[2]
    msg.rotation.x=q[0]
    msg.rotation.y=q[1]
    msg.rotation.z=q[2]
    msg.rotation.w=q[3]
    return msg

def callback(data):
    joint_name=data.name
    joint_value=data.position
#initialize each link name, joint object,joint name, link tf, joint rotation matrix
    link,joint,jnt_name,T,link_tf,joint_rot={},{},{},{},{},{}
    k=0 
#create a dictionary to map each joint name with respective joint value
    dic={}
    for i in range(len(joint_name)):
       dic[joint_name[i]]=joint_value[i]
 
    link[0]=robot.get_root()
#while true generate transform matrix from root to each link, otherwise if has keyerror then break
#while last link, there is a keyerror, break
    while True:
       try:           
          (jnt_name[k], link[k+1])=robot.child_map[link[k]][0]
          joint[k]=robot.joint_map[jnt_name[k]]
#translation and rotation of link
          link_tf[k]=numpy.dot(tf.transformations.translation_matrix(joint[k].origin.xyz),tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(joint[k].origin.rpy[0],joint[k].origin.rpy[1],joint[k].origin.rpy[2])))
	  
#fixed joint gives identity matrix, while revolute joint has value
          if joint[k].axis is None:
             joint_rot[k]=numpy.identity(4)
          else:
             joint_rot[k]=tf.transformations.rotation_matrix(dic[jnt_name[k]],joint[k].axis)
#from a link to the next link
          T[k]=tf.transformations.concatenate_matrices(link_tf[k],joint_rot[k])

#from root link to each link in the chain 
          T_t=numpy.identity(4)        
          for n in range(0,k+1):
              T_t=numpy.dot(T_t,T[n])
          publish_transforms(T_t,link[0],joint[k].child)
          k+=1
       except KeyError:
          break
	
def publish_transforms(T,header_frame, child_frame):  
    T1_stamped=geometry_msgs.msg.TransformStamped()
    T1_stamped.header.stamp=rospy.Time.now()
    T1_stamped.header.frame_id=header_frame
    T1_stamped.child_frame_id=child_frame
    T1_stamped.transform=message_from_transform(T)
    br.sendTransform(T1_stamped)
   
  
if __name__ == '__main__':
    rospy.init_node('forward_kinematics', anonymous=True)
    robot=URDF.from_parameter_server(key='robot_description')
    br=tf2_ros.TransformBroadcaster()
    rospy.Subscriber('/joint_states', JointState, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    rospy.sleep(0.1)

