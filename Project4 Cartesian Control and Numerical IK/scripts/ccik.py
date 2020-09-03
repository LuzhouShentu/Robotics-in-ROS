#!/usr/bin/env python

import math
import numpy
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from cartesian_control.msg import CartesianCommand
from urdf_parser_py.urdf import URDF
import random
import tf
from threading import Thread, Lock
import time

'''This is a class which will perform both cartesian control and inverse
   kinematics'''
class CCIK(object):
    def __init__(self):
	#Load robot from parameter server
        self.robot = URDF.from_parameter_server()

	#Subscribe to current joint state of the robot
        rospy.Subscriber('/joint_states', JointState, self.get_joint_state)

	#This will load information about the joints of the robot
        self.num_joints = 0
        self.joint_names = []
        self.q_current = []
        self.joint_axes = []
        self.get_joint_info()

	#This is a mutex
        self.mutex = Lock()

	#Subscribers and publishers for for cartesian control
        rospy.Subscriber('/cartesian_command', CartesianCommand, self.get_cartesian_command)
        self.velocity_pub = rospy.Publisher('/joint_velocities', JointState, queue_size=10)
        self.joint_velocity_msg = JointState()

        #Subscribers and publishers for numerical IK
        rospy.Subscriber('/ik_command', Transform, self.get_ik_command)
        self.joint_command_pub = rospy.Publisher('/joint_command', JointState, queue_size=10)
        self.joint_command_msg = JointState()

    '''This is a function which will collect information about the robot which
       has been loaded from the parameter server. It will populate the variables
       self.num_joints (the number of joints), self.joint_names and
       self.joint_axes (the axes around which the joints rotate)'''
    def get_joint_info(self):
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
   
    '''This is the callback which will be executed when the cartesian control
       recieves a new command. The command will contain information about the
       secondary objective and the target q0. At the end of this callback, 
       you should publish to the /joint_velocities topic.'''
    def get_cartesian_command(self, command):
        self.mutex.acquire()
        
 #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR CARTESIAN CONTROL HERE
	joint_transforms, b_T_ee_cur = self.forward_kinematics(self.q_current)
	trans_xyz = (command.x_target.translation.x, command.x_target.translation.y,command.x_target.translation.z)
	rot_xyzw = (command.x_target.rotation.x, command.x_target.rotation.y, command.x_target.rotation.z, command.x_target.rotation.w)
	
	trans = tf.transformations.translation_matrix(trans_xyz)
	rot = tf.transformations.quaternion_matrix(rot_xyzw)
	b_T_ee_des = numpy.dot(trans, rot)

	ee_cur_T_b = tf.transformations.inverse_matrix(b_T_ee_cur)
	ee_cur_T_ee_des = numpy.dot(ee_cur_T_b, b_T_ee_des)

	ee_cur_T_ee_des_trans = tf.transformations.translation_from_matrix(ee_cur_T_ee_des)
	angle, axis = self.rotation_from_matrix(ee_cur_T_ee_des)
	ee_cur_T_ee_des_rot = numpy.dot(angle, axis)

	delta_x = numpy.zeros((6,1))
	delta_x[0]=ee_cur_T_ee_des_trans[0]	
	delta_x[1]=ee_cur_T_ee_des_trans[1]	
	delta_x[2]=ee_cur_T_ee_des_trans[2]	
	delta_x[3]=ee_cur_T_ee_des_rot[0]	
	delta_x[4]=ee_cur_T_ee_des_rot[1]	
	delta_x[5]=ee_cur_T_ee_des_rot[2]
	
	p=1
	xdot = p*delta_x
	
	#print(xdot)

	J = self.get_jacobian(b_T_ee_cur, joint_transforms)	
	Jp = numpy.linalg.pinv(J,0.01)	
	qdot = numpy.dot(Jp, xdot)
	n=len(self.q_current)
	qsec = numpy.zeros((n,1))
	I = numpy.identity(n)
	if command.secondary_objective == True:
		qsec[0] = 10*(command.q0_target - self.q_current[0])
	
		qnul = numpy.dot(I - numpy.dot(Jp, J),qsec)
		qdot = qdot + qnul
	else:
		qdot = qdot
	#print('qd1',qdot)
	i=0
	while i<n:
		if qdot[i]>=0:
			if qdot[i]>1:
				qdot[i]=1
		if qdot[i]<0:
			if abs(qdot[i])>1:
				qdot[i]=1
		i=i+1
	#print ('qd2',qdot)
    	self.joint_velocity_msg.name = self.joint_names
     	self.joint_velocity_msg.velocity = qdot 
	self.velocity_pub.publish(self.joint_velocity_msg)      #--------------------------------------------------------------------------
        self.mutex.release()
    '''This is a function which will assemble the jacobian of the robot using the
       current joint transforms and the transform from the base to the end
       effector (b_T_ee). Both the cartesian control callback and the
       inverse kinematics callback will make use of this function.
       Usage: J = self.get_jacobian(b_T_ee, joint_transforms)'''
    def get_jacobian(self, b_T_ee, joint_transforms):
        J = numpy.zeros((6,self.num_joints))
    #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR ASSEMBLING THE CURRENT JACOBIAN HERE
	#joint_transforms, b_T_ee = self.forward_kinematics(joint_values)# q_current or joint_values?	
	self.Vj = []
	for j in range(self.num_joints):
			
		b_T_j = joint_transforms[j]
		j_T_b = tf.transformations.inverse_matrix(b_T_j)
		j_T_ee = numpy.dot(j_T_b, b_T_ee)
		ee_T_j = tf.transformations.inverse_matrix(j_T_ee)
		ee_R_j = ee_T_j[:3,:3]
		j_t_ee = tf.transformations.translation_from_matrix(j_T_ee)

		S = numpy.zeros((3,3))
    		S[0,1] = -j_t_ee[2]
    		S[0,2] =  j_t_ee[1]
    		S[1,0] =  j_t_ee[2]
    		S[1,2] = -j_t_ee[0]
    		S[2,0] = -j_t_ee[1]
    		S[2,1] =  j_t_ee[0]

		
		Vj00 = ee_R_j
		Vj01 = -numpy.dot(ee_R_j, S)
		Vj10 = numpy.zeros((3,3))
		Vj11 = ee_R_j
		Vjtop = numpy.append(Vj00, Vj01, axis=1)
		Vjbot = numpy.append(Vj10, Vj11, axis=1)
		Vj = numpy.append(Vjtop, Vjbot, axis=0)
		j_new=[[]]
		a = numpy.matrix([[3],[4],[5]])
		b = numpy.dot(self.joint_axes[j], a)
		c = b[0,0]
		
		if c > 0:
			J[:,j]=Vj[:,int(c)]
		else:
			d=abs(c)
			J[:,j]=-Vj[:,int(d)]
 #--------------------------------------------------------------------------
        return J
  

    '''This is the callback which will be executed when the inverse kinematics
       recieve a new command. The command will contain information about desired
       end effector pose relative to the root of your robot. At the end of this
       callback, you should publish to the /joint_command topic. This should not
       search for a solution indefinitely - there should be a time limit. When
       searching for two matrices which are the same, we expect numerical
       precision of 10e-3.'''
    def get_ik_command(self, command):
        self.mutex.acquire()
        #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR INVERSE KINEMATICS HERE

        m = self.num_joints
	i=0
	qc= numpy.zeros((m,1))
	
	while i < m:
		
		qc[i]=random.uniform(-1*math.pi, math.pi)
		i+=1
	
	#print('st',start_time)
	#print('ft',final_time)
	t=3
	while t>0:
		start_time = time.time()
		final_time = start_time + 10
		while True:
		
			joint_transforms, b_T_ee_cur = self.forward_kinematics(qc)
			trans_xyz = (command.translation.x, command.translation.y,command.translation.z)
			rot_xyzw = (command.rotation.x, command.rotation.y, command.rotation.z, command.rotation.w)
	
			trans = tf.transformations.translation_matrix(trans_xyz)
			rot = tf.transformations.quaternion_matrix(rot_xyzw)
			b_T_ee_des = numpy.dot(trans, rot)

			ee_cur_T_b = tf.transformations.inverse_matrix(b_T_ee_cur)
			ee_cur_T_ee_des = numpy.dot(ee_cur_T_b, b_T_ee_des)

			ee_cur_T_ee_des_trans = tf.transformations.translation_from_matrix(ee_cur_T_ee_des)
			angle, axis = self.rotation_from_matrix(ee_cur_T_ee_des)
			ee_cur_T_ee_des_rot = numpy.dot(angle, axis)
	
			delta_x = numpy.zeros((6,1))
			delta_x[0]=ee_cur_T_ee_des_trans[0]
			delta_x[1]=ee_cur_T_ee_des_trans[1]	
			delta_x[2]=ee_cur_T_ee_des_trans[2]
			delta_x[3]=ee_cur_T_ee_des_rot[0]	
			delta_x[4]=ee_cur_T_ee_des_rot[1]	
			delta_x[5]=ee_cur_T_ee_des_rot[2]
	
			p=1
			deltax = p*delta_x
			J = self.get_jacobian(b_T_ee_cur, joint_transforms)
			run_time = time.time()
		
			if max(abs(deltax)) <0.001:
				t=0
				break
			#sum(numpy.abs(deltax))	
			J = self.get_jacobian(b_T_ee_cur, joint_transforms)
			Jp = numpy.linalg.pinv(J,0.01)
			deltaq = numpy.dot(Jp,deltax)
			qc = qc + deltaq
				
			if run_time > final_time:
				t=t-1
				print('IK times out')
				if t == 0:
					print('IK failed')
				break
	

		
	
	self.joint_command_msg.name = self.joint_names
     	self.joint_command_msg.position = qc
	self.joint_command_pub.publish(self.joint_command_msg)
        #--------------------------------------------------------------------------
        self.mutex.release()

    '''This function will return the angle-axis representation of the rotation
       contained in the input matrix. Use like this: 
       angle, axis = rotation_from_matrix(R)'''
    def rotation_from_matrix(self, matrix):
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

    '''This is the function which will perform forward kinematics for your 
       cartesian control and inverse kinematics functions. It takes as input
       joint values for the robot and will return an array of 4x4 transforms
       from the base to each joint of the robot, as well as the transform from
       the base to the end effector.
       Usage: joint_transforms, b_T_ee = self.forward_kinematics(joint_values)'''
    def forward_kinematics(self, joint_values):
        joint_transforms = []

        link = self.robot.get_root()
        T = tf.transformations.identity_matrix()

        while True:
            if link not in self.robot.child_map:
                break

            (joint_name, next_link) = self.robot.child_map[link][0]
            joint = self.robot.joint_map[joint_name]

            T_l = numpy.dot(tf.transformations.translation_matrix(joint.origin.xyz), tf.transformations.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2]))
            T = numpy.dot(T, T_l)

            if joint.type != "fixed":
                joint_transforms.append(T)
                q_index = self.joint_names.index(joint_name)# index:given element in a list and returns its position.
                T_j = tf.transformations.rotation_matrix(joint_values[q_index], numpy.asarray(joint.axis))#numpy.asarry: conver something in the () into array, there is rotate around an axis
                T = numpy.dot(T, T_j)

            link = next_link
        return joint_transforms, T #where T = b_T_ee

    '''This is the callback which will recieve and store the current robot
       joint states.'''
    def get_joint_state(self, msg):
        self.mutex.acquire()
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])
        self.mutex.release()


if __name__ == '__main__':
    rospy.init_node('cartesian_control_and_IK', anonymous=True)
    CCIK()
    rospy.spin()
