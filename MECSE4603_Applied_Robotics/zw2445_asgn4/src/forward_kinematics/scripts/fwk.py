#!/usr/bin/env python

import numpy

import geometry_msgs.msg
import rospy
from sensor_msgs.msg import JointState
import tf
import tf.msg
from urdf_parser_py.urdf import URDF

""" Starting from a computed transform T, creates a message that can be
communicated over the ROS wire. In addition to the transform itself, the message
also specifies who is related by this transform, the parent and the child."""
def convert_to_message(T, child, parent):
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = parent
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = child
    translation = tf.transformations.translation_from_matrix(T)
    rotation = tf.transformations.quaternion_from_matrix(T)
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]
    t.transform.rotation.x = rotation[0]
    t.transform.rotation.y = rotation[1]
    t.transform.rotation.z = rotation[2]
    t.transform.rotation.w = rotation[3]        
    return t
    
#Our main class for computing Forward Kinematics
class ForwardKinematics(object):


    #Initialization
    def __init__(self):
        """Announces that it will publish forward kinematics results, in the form of tfMessages.
        "tf" stands for "transform library", it's ROS's way of communicating around information
        about where things are in the world"""
        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=1)

        #Loads the robot model, which contains the robot's kinematics information
        self.robot = URDF.from_parameter_server()

        #Subscribes to information about what the current joint values are.
        rospy.Subscriber("joint_states", JointState, self.callback)


    """This function is called every time the robot publishes its joint values. We must use
    the information we get to compute forward kinematics.

    We will iterate through the entire chain, and publish the transform for each link we find.
    """
    def callback(self, joint_values):
        # We will start at the root
        root = self.robot.get_root()
        # The transform we begin with is the identity
        T = tf.transformations.identity_matrix()
        # This will hold our results, which we will publish at the end
        all_transforms = tf.msg.tfMessage()
        link = root
        while True:
            # Find the joint connected at the end of this link, or its "child"
            # Make sure this link has a child
            if link not in self.robot.child_map:
                break
            # Make sure it has a single child (we don't deal with forks)
            if len(self.robot.child_map[link]) != 1:
                rospy.logerr("Forked kinematic chain!")
                print link
                break
            # Get the name of the child joint, as well as the link it connects to
            (joint_name, next_link) = self.robot.child_map[link][0]
            # Get the actual joint based on its name
            if joint_name not in self.robot.joint_map:
                rospy.logerr("Joint not found in map")
                break;
            joint = self.robot.joint_map[joint_name]

            # We not have all the info we need. Time to update the transform.
            T = self.update_transform_matrix(T, joint, joint_values)

            # Convert the result into a message and prepare it to be published
            transform_msg = convert_to_message(T, next_link, root)
            all_transforms.transforms.append(transform_msg)
            # Move to the next link
            link = next_link

        # Publish all the transforms
        self.pub_tf.publish(all_transforms)


    """ This is the core function for our forward kinematics. Given the transform up to Joint i-1,
    it computes the transform up to and including Joint i.

    Parameters are as follows:
    - T_in is the transform up to and including Joint i-1 (essentially up to the joint we are 
    currently considering)

    - current_joint is the information about Joint i, the one we are considering now. We will use the 
    following fields:
     * current_joint.origin.xyz: the translation from the frame of the previous joint to this one
     * current_joint.origin.rpy: the rotation from the frame of the previous joint to this one, 
       in ROLL-PITCH-YAW XYZ convention
     * current_joint.type: either 'fixed' or 'revolute'. A fixed joint does not move; it is meant to 
       contain a static transform. 
     * current_joint.name: the name of the current joint in the robot description
     * current_joint.axis: (only if type is 'revolute') the axis of rotation of the joint

     - joint_values contains information about the current joint values in the robot. It contains
     information about *all* the joints, so we must find the relevant value for the joint we are now
     considering. We can use the following fields:
      * joint_values.name: a list of the names of *all* the joints in the robot
      * joint_values.position: a list of the current values of *all* the joints in the robot, in the same 
        order as the names in the list above.
     To find the value of the joint we care about, we must find its name in the "name" list, then take
     the value found at the same index in the "position" list.

     We must return the value of T_out, the transform once it has been updated to take into account
     the current joint.
    """    
    def update_transform_matrix(self, T_in, current_joint, joint_values):
        rot_matrix = tf.transformations.euler_matrix(current_joint.origin.rpy[0], 
                                                     current_joint.origin.rpy[1],
                                                     current_joint.origin.rpy[2], 'rxyz')
        trans_matrix = tf.transformations.translation_matrix((current_joint.origin.xyz[0], 
                                                              current_joint.origin.xyz[1],
                                                              current_joint.origin.xyz[2]))
        T = numpy.dot(trans_matrix, rot_matrix)        
        if current_joint.type != 'fixed':
            if current_joint.name not in joint_values.name:
                rospy.logerr("Joint not found in list")
            else:
                index = joint_values.name.index(current_joint.name)
                angle = joint_values.position[index]
                joint_rot_matrix = tf.transformations.rotation_matrix(angle, numpy.asarray(current_joint.axis))
                T = numpy.dot(T, joint_rot_matrix) 
        T_out = numpy.dot(T_in, T)
        return T_out

        
if __name__ == '__main__':
    rospy.init_node('fwk', anonymous=True)
    fwk = ForwardKinematics()
    rospy.spin()
