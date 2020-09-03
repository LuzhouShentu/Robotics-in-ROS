#!/usr/bin/env python  
import rospy

import numpy
import math
import tf.transformations
import tf
import tf2_ros
import geometry_msgs.msg

def publish_transforms():
    
    T1 = numpy.dot(tf.transformations.euler_matrix(0.64, 0.64, 0.0), 
                   tf.transformations.translation_matrix((1.5,0.8,0.0)))
    
    t1 = geometry_msgs.msg.TransformStamped()
    t1.header.stamp = rospy.Time.now()
    t1.header.frame_id = "base_frame"
    t1.child_frame_id = "object_frame" 
    tr1 = tf.transformations.translation_from_matrix(T1)
    t1.transform.translation.x = tr1[0]
    t1.transform.translation.y = tr1[1]
    t1.transform.translation.z = tr1[2]
    q1 = tf.transformations.quaternion_from_matrix(T1)
    t1.transform.rotation.x = q1[0]
    t1.transform.rotation.y = q1[1]
    t1.transform.rotation.z = q1[2]
    t1.transform.rotation.w = q1[3]
    br.sendTransform(t1)
    
    T2 = numpy.dot(tf.transformations.rotation_matrix(1.5,(0.0,1.0,0.0)), 
                   tf.transformations.translation_matrix((0.0,0.0,-2.0)))
                    
    t2 = geometry_msgs.msg.TransformStamped()
    t2.header.stamp = rospy.Time.now()
    t2.header.frame_id = "base_frame"
    t2.child_frame_id = "robot_frame"
    tr2 = tf.transformations.translation_from_matrix(T2)
    t2.transform.translation.x = tr2[0]
    t2.transform.translation.y = tr2[1]
    t2.transform.translation.z = tr2[2]
    q2 = tf.transformations.quaternion_from_matrix(T2)
    t2.transform.rotation.x = q2[0]
    t2.transform.rotation.y = q2[1]
    t2.transform.rotation.z = q2[2]
    t2.transform.rotation.w = q2[3]
    br.sendTransform(t2)
 
    T2_inverse = tf.transformations.inverse_matrix(T2)
    T3 = tf.transformations.translation_matrix((0.3,0,0.3))
    T3_inverse = tf.transformations.inverse_matrix(T3)
    ob_in_ob = ([0,0,0,1])
    ob_in_ba = numpy.dot(T1,ob_in_ob)
    ob_in_ro = numpy.dot(T2_inverse,ob_in_ba)
    ob_in_ca = numpy.dot(T3_inverse,ob_in_ro)
    ob_in_ca2= ob_in_ca[0:3]
    Xca_in_ca = ([1,0,0])
    Axis_aim =numpy.cross(Xca_in_ca,ob_in_ca2)
    dot_aim = numpy.dot(Xca_in_ca,ob_in_ca2)
    Xca_in_ca_norm = numpy.linalg.norm(Xca_in_ca)
    ob_in_ca2_norm = numpy.linalg.norm(ob_in_ca2)
    Cos_Angle_aim= dot_aim / (Xca_in_ca_norm*ob_in_ca2_norm)
    angle_aim = math.acos(Cos_Angle_aim)
    
    
    t3 = geometry_msgs.msg.TransformStamped()
    t3.header.stamp = rospy.Time.now()
    t3.header.frame_id = "robot_frame"
    t3.child_frame_id = "camera_frame" 
    t3.transform.translation.x = 0.3
    t3.transform.translation.y = 0
    t3.transform.translation.z = 0.3
    q3 = tf.transformations.quaternion_about_axis(angle_aim,(Axis_aim))
    t3.transform.rotation.x = q3[0]
    t3.transform.rotation.y = q3[1]
    t3.transform.rotation.z = q3[2]
    t3.transform.rotation.w = q3[3]
    br.sendTransform(t3)




    
if __name__ == '__main__':
    rospy.init_node('solution')
    br = tf2_ros.TransformBroadcaster()
    rospy.sleep(0.1)

    while not rospy.is_shutdown():
        publish_transforms()
        rospy.sleep(0.1)
