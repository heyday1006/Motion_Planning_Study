#!/usr/bin/env python
import rospy
import numpy
import tf
import tf2_ros
import geometry_msgs.msg

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

def publish_transforms():
    #tranform from base to object
    T1=tf.transformations.concatenate_matrices(tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0.79,0.0,0.79)),tf.transformations.translation_matrix((0.0,1.0,1.0)))
    T1_stamped=geometry_msgs.msg.TransformStamped()
    T1_stamped.header.stamp=rospy.Time.now()
    T1_stamped.header.frame_id="base_frame"
    T1_stamped.child_frame_id="object_frame"
    T1_stamped.transform=message_from_transform(T1)
    br.sendTransform(T1_stamped)
    
    #transform from base to robot
    T2=tf.transformations.concatenate_matrices(tf.transformations.rotation_matrix(1.5,(0.0,0.0,1.0)),tf.transformations.translation_matrix((0.0,-1.0,0.0)))
    T2_stamped=geometry_msgs.msg.TransformStamped()
    T2_stamped.header.stamp=rospy.Time.now()
    T2_stamped.header.frame_id="base_frame"
    T2_stamped.child_frame_id="robot_frame"
    T2_stamped.transform=message_from_transform(T2)
    br.sendTransform(T2_stamped)

    #transform with only translation from robot to camera
    T3_trans=tf.transformations.translation_matrix((0.0,0.1,0.1))
    #inverse transform gives camera to robot
    T3_trans_inv=tf.transformations.inverse_matrix(T3_trans)
    #transform from camera to base
    T_32_trans=numpy.dot(T3_trans_inv,tf.transformations.inverse_matrix(T2))
    #transform from camera to object
    T_31_trans=numpy.dot(T_32_trans,T1)
    #translation from camera to object
    T_31_translation=tf.transformations.translation_from_matrix(T_31_trans)
    #x-axis of camera-(1.0,0.0,0.0)
    #angle of rotation
    #rot=numpy.arctan2(T_31_translation,(1.0,0.0,0.0))
    cos_angle=numpy.dot(T_31_translation,(1.0,0.0,0.0))/numpy.linalg.norm(T_31_translation)
    sin_angle=numpy.linalg.norm(numpy.cross(T_31_translation,(1.0,0.0,0.0)))/numpy.linalg.norm(T_31_translation)
    rot=numpy.arctan2(sin_angle,cos_angle)
    #axis at which the camera rotates
    rot_axis=numpy.cross((1.0,0.0,0.0),T_31_translation)

    #transform from object to camera
    #same as below method, whereas t_x has value instead 0, calculation inaccuracy
    #T_13=tf.transformations.concatenate_matrices(tf.transformations.inverse_matrix(T_31_trans),tf.transformations.rotation_matrix(rot,rot_axis))
    #transform from robot to object
    #T_21=numpy.dot(tf.transformations.inverse_matrix(T2),T1)
    #T3=numpy.dot(T_21,T_13)

    #transform from camera to object
    T3=tf.transformations.concatenate_matrices(T3_trans,tf.transformations.rotation_matrix(rot,rot_axis))
    T3_stamped=geometry_msgs.msg.TransformStamped()
    T3_stamped.header.stamp=rospy.Time.now()
    T3_stamped.header.frame_id="robot_frame"
    T3_stamped.child_frame_id="camera_frame"
    T3_stamped.transform=message_from_transform(T3)
    br.sendTransform(T3_stamped)


if __name__ == '__main__':
    rospy.init_node('solution')
    br=tf2_ros.TransformBroadcaster()
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        publish_transforms()
        rospy.sleep(0.5)
