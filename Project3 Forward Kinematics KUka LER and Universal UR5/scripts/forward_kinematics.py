#!/usr/bin/env python

import numpy
import geometry_msgs.msg
import rospy
from sensor_msgs.msg import JointState
import tf
import tf.transformations
import tf.msg
from urdf_parser_py.urdf import URDF

"""This function will transform a 4x4 transformation matrix T into a ros message 
which can be published. In addition to the transform itself, the message
also specifies who is related by this transform, the parent and the child.
It is an optional function which you may use as you see fit."""
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
        aboutrobot = URDF.from_parameter_server() where things are in the world"""
        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=1)

        #Loads the robot model, which contains the robot's kinematics information
        self.robot = URDF.from_parameter_server()

        #Subscribes to information about what the current joint values are.
        rospy.Subscriber("joint_states", JointState, self.callback)


    """This function is called every time the robot publishes its joint values. You must use
    the information you get to compute forward kinematics.

    The callback you write should iterate through the entire robot chain, and publish 
    the transform for each link you find.
    """
    def callback(self, joint_values):
	root_link = self.robot.get_root()
	(next_joint_name, next_link_name) = self.robot.child_map[root_link][0]
	next_joint = self.robot.joint_map[next_joint_name]	
	Tt = tf.transformations.translation_matrix(next_joint.origin.xyz)
	Tr = tf.transformations.euler_matrix(next_joint.origin.rpy[0], next_joint.origin.rpy[1], next_joint.origin.rpy[2])

	Tb = numpy.dot(Tt, Tr)
	Mb = convert_to_message(Tb, next_joint.child, root_link)
	self.pub_tf.publish(tf.msg.tfMessage([Mb]))
	
	tree = []
	(joint, link)=(0, next_link_name)
	for i in range(len(self.robot.child_map)-1):
		(joint,link) = self.robot.child_map[link][0]
		tree.append(joint)
	
	j=0
	for mapjoint in tree:
		next_joint = self.robot.joint_map[mapjoint]

		if next_joint.type == 'revolute':
		
			Tt = tf.transformations.translation_matrix(next_joint.origin.xyz)
			Tr = tf.transformations.euler_matrix(next_joint.origin.rpy[0], next_joint.origin.rpy[1], next_joint.origin.rpy[2])
			Tv = tf.transformations.rotation_matrix(joint_values.position[j], next_joint.axis)

			Tb = numpy.dot(Tb, numpy.dot(numpy.dot(Tt, Tr), Tv))
			M = convert_to_message(Tb, next_joint.child, root_link)
			self.pub_tf.publish(tf.msg.tfMessage([M]))
			j=j+1
		else:
			Tt = tf.transformations.translation_matrix(next_joint.origin.xyz)
			Tr = tf.transformations.euler_matrix(next_joint.origin.rpy[0], next_joint.origin.rpy[1], next_joint.origin.rpy[2])
		
			Tb = numpy.dot(Tb, numpy.dot(Tt, Tr))
			M = convert_to_message(Tb, next_joint.child, root_link)
			self.pub_tf.publish(tf.msg.tfMessage([M]))
	

if __name__ == '__main__':
    rospy.init_node('fwk', anonymous=True)
    fwk = ForwardKinematics()
  
    rospy.spin()

