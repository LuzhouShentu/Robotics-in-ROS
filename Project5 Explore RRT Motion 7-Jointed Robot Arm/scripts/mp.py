#!/usr/bin/env python

import numpy
import random
import sys

import geometry_msgs.msg
import moveit_msgs.msg
import moveit_msgs.srv
import rospy
import tf
import moveit_commander
from urdf_parser_py.urdf import URDF
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def convert_to_message(T):
    t = geometry_msgs.msg.Pose()
    position = tf.transformations.translation_from_matrix(T)
    orientation = tf.transformations.quaternion_from_matrix(T)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[0]
    t.orientation.y = orientation[1]
    t.orientation.z = orientation[2]
    t.orientation.w = orientation[3]        
    return t

class MoveArm(object):

    def __init__(self):

        #Loads the robot model, which contains the robot's kinematics information
	self.num_joints = 0
        self.joint_names = []
        self.joint_axes = []
        self.robot = URDF.from_parameter_server()
        self.base = self.robot.get_root()
        self.get_joint_info()

        # Wait for moveit IK service
        rospy.wait_for_service("compute_ik")
        self.ik_service = rospy.ServiceProxy('compute_ik',  moveit_msgs.srv.GetPositionIK)
        print "IK service ready"

        # Wait for validity check service
        rospy.wait_for_service("check_state_validity")
        self.state_valid_service = rospy.ServiceProxy('check_state_validity',  
                                                      moveit_msgs.srv.GetStateValidity)
        print "State validity service ready"

        # MoveIt parameter
        robot_moveit = moveit_commander.RobotCommander()
        self.group_name = robot_moveit.get_group_names()[0]

	#Subscribe to topics
	rospy.Subscriber('/joint_states', JointState, self.get_joint_state)
	rospy.Subscriber('/motion_planning_goal', Transform, self.motion_planning)
        self.current_obstacle = "None"
        rospy.Subscriber('/obstacle', String, self.get_obstacle)

	#Set up publisher
	self.pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)

    '''This callback provides you with the current joint positions of the robot 
     in member variable q_current.'''
    def get_joint_state(self, msg):
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])

    '''This callback provides you with the name of the current obstacle which
    exists in the RVIZ environment. Options are "None", "Simple", "Hard",
    or "Super". '''
    def get_obstacle(self, msg):
        self.current_obstacle = msg.data

    '''This is the callback which will implement your RRT motion planning.
    You are welcome to add other functions to this class (i.e. an
    "is_segment_valid" function will likely come in handy multiple times 
    in the motion planning process and it will be easiest to make this a 
    seperate function and then call it from motion planning). You may also
    create trajectory shortcut and trajectory sample functions if you wish, 
    which will also be called from the motion planning function.'''    
    def motion_planning(self, ee_goal):
	print "Starting motion planning"
	########INSERT YOUR RRT MOTION PLANNING HERE##########
	goal_tran = tf.transformations.translation_matrix((ee_goal.translation.x, ee_goal.translation.y, ee_goal.translation.z))
	
	goal_rot = tf.transformations.quaternion_matrix((ee_goal.rotation.x, ee_goal.rotation.y, ee_goal.rotation.z, ee_goal.rotation.w))
	#print('goal',rotation)
	T_goal = numpy.dot(goal_tran, goal_rot)
	#print('T_goal',T_goal)

	q_goal = numpy.array(self.IK(T_goal))
	
	q_curr = numpy.array(self.q_current)
	self.go=[]
	self.all_q =[]
	self.all_q.append(q_curr)
	self.parents = []
	self.children = []
	self.path = []
	self.go = []
	self.pathi = []
	self.pathn = []
	self.pathq = []
	#d0 = numpy.linalg.norm(numpy.subtract(q_goal, q_curr))
	#print('q_curr',q_curr)
	#print('q_goal',q_goal)
	
	if self.current_obstacle == "None":
		
		self.pathn.append(q_curr)
		self.pathn.append(q_goal)
		self.pathq.append(self.pathn[0])
			
	        ds = numpy.linalg.norm(numpy.subtract(self.pathn[1], self.pathn[0]))
	        unis = numpy.subtract(self.pathn[1],self.pathn[0])/numpy.linalg.norm(numpy.subtract(self.pathn[1],self.pathn[0]))
		
	        ks = ds/25
	        
	        qs = self.pathn[0] + ks*unis
	        self.pathn.append(qs)
	        for i in range(24):
		        qs = qs + ks*unis
		        self.pathq.append(qs)
		self.pathq.append(self.pathn[1])
		print('pathq', self.pathq)
	#print('q_newc',q_newc)
	
	a=0
	k=0
	while k<10000000:
		
		q_new=[]
		q_random=[]
		if self.num_joints == 6:
			for i in range(self.num_joints):
				rand = random.uniform(-2*numpy.pi, 2*numpy.pi)
				q_random.append(rand)
		elif self.num_joints == 7:
	    		for i in range(self.num_joints):
				rand = random.uniform(-numpy.pi, numpy.pi)
				q_random.append(rand)
		#print('q_rand1',q_rand)
		while self.is_state_valid(q_random) == False:
			#print('false')
			q_random=[]
			if self.num_joints == 6:
				for i in range(self.num_joints):
					rand = random.uniform(-2*numpy.pi, 2*numpy.pi)
					q_random.append(rand)
			elif self.num_joints == 7:
	    			for i in range(self.num_joints):
					rand = random.uniform(-numpy.pi, numpy.pi)
					q_random.append(rand)
		q_new = q_random
		q_dis = []
		for i in range(len(self.all_q)):
			distance = numpy.linalg.norm(numpy.subtract(q_new, self.all_q[i]))
			q_dis.append(distance)
			#print('distance', distance)
		#print('qdis',q_dis)
		dis_min = min(q_dis)
		index = q_dis.index(dis_min)
		q_parent =self.all_q[index]	####################################################################################################
		self.parents.append(q_parent)
		
	
		#print('index',index)
		#print('qp',q_parent)
		uv_pc = numpy.subtract(q_new, q_parent)/numpy.linalg.norm(numpy.subtract(q_new, q_parent))
		#print('uv_pc', uv_pc)
		d_pc = numpy.linalg.norm(numpy.subtract(q_new, q_parent))
		k_pc = d_pc/100
		q_child = q_parent
		for i in range(100):
			q_child = q_child + k_pc*uv_pc
		
		#print('q_child', q_child)
			if self.is_state_valid(q_child) == False:
				#print('false')
				q_child -= uv_pc
				break
				#print('q_child2',q_child)
		self.all_q.append(q_child)
		self.children.append(q_child)
		
	
		un_cg = numpy.subtract(q_goal, q_child)/numpy.linalg.norm(numpy.subtract(q_goal, q_child))
		d_cg = numpy.linalg.norm(numpy.subtract(q_goal, q_child))
		k_cg = d_cg/50
		q_to_goal = q_child
		
		for i in range(50):
			q_to_goal = q_to_goal + k_cg*un_cg
			if self.is_state_valid(q_to_goal) == False:
				#print('no')
				break
			elif numpy.linalg.norm(numpy.subtract(q_goal, q_to_goal))<0.0000001:
				#print('yes')
				self.all_q.append(q_goal)
				self.parents.append(q_child)
				self.children.append(q_goal)
				
		
		a+=1
		k+=1
		
		#print('self.children',self.children)
		
		#print('n_parents',len(self.parents))
		#print('n_children',len(self.children))
		print('k',k)
		#print('loop',a)		
		if numpy.linalg.norm(numpy.subtract(q_goal,self.all_q[-1])) == 0:
			break

	q_iterate = q_goal
	num=len(self.children)
	i=0
	while i <num:
		#print('i',i)
		index = i
		if numpy.linalg.norm(numpy.subtract(q_iterate,self.children[index])) == 0:
			self.path.append(self.children[index])
			self.path.append(self.parents[index])
			if numpy.linalg.norm(numpy.subtract(self.parents[index],self.q_current)) == 0:
				#print('finished')
				break
			else:
				#print('continue')
				q_iterate = self.parents[index]
				i=0
			
		else:
			i+=1	
	
	
	self.path.reverse()
	#print ('path',self.path)
	#print('q_goal',q_goal)
	#print('q_curr',numpy.array(self.q_current))
	#print('n_path',len(self.path))
	self.path0 =[]
	
	for i in range(len(self.path)-1):
		if numpy.linalg.norm(numpy.subtract(self.path[i+1],self.path[i])) == 0:
			continue
		else :
			self.path0.append(self.path[i])
	self.path0.append(self.path[-1])
################################################################################################
	self.paths = []
	self.paths.append(self.path0[0])
	n0 =len(self.path0)-2#?????
	print('n0',n0)
	i =0
	f=2
	while i < n0:
		
		while i+f < n0+2:	
			
			uni = numpy.subtract(self.path0[i+f], self.path0[i])/numpy.linalg.norm(numpy.subtract(self.path0[i+f], self.path0[i]))
			#print('uni',uni)
			di = numpy.linalg.norm(numpy.subtract(self.path0[i+f], self.path0[i]))
			ki = di/100
			q_test = self.path0[i]
		
			for j in range(100):
				q_test = q_test + ki*uni
			
				if self.is_state_valid(q_test)==False:
					self.paths.append(self.path0[i+1])
					i=i+1
					break
				if j == 99:
					self.paths.append(self.path0[i+2])
					i=i+2
					break
		
		
		self.paths.append(self.path0[-1])
		print('finished')
		break
	print('n0',self.path0)
	print('path0',len(self.path0))	
	print('paths',self.paths)
	print('ns',len(self.paths))
	print('goal',self.IK(T_goal))
		

	self.pathi= []
	self.pathi= self.path0
	self.pathf=[]
	self.pathf.append(self.pathi[0])
	for i in range(len(self.pathi)-1):
	    	if numpy.linalg.norm(numpy.subtract(self.pathi[i+1],self.pathi[i])) == 0:
	        	continue
	        ds = numpy.linalg.norm(numpy.subtract(self.pathi[i+1], self.pathi[i]))
	        unis = numpy.subtract(self.pathi[i+1],self.pathi[i])/numpy.linalg.norm(numpy.subtract(self.pathi[i+1],self.pathi[i]))
		
	        ks = ds/25
	        
	        qs = self.pathi[i] + ks*unis
	        self.pathf.append(qs)
	        for i in range(24):
		        qs = qs + ks*unis
		        self.pathf.append(qs)
	
	self.pathf.append(self.pathi[-1])
	
	

	#print("final path", self.go)
	if self.current_obstacle == "None":
		j = JointTrajectory()
		j.joint_names = self.joint_names
		pathp = []
		for i in range(len(self.pathq)):
	    		f = JointTrajectoryPoint()
	    		f.positions = self.pathq[i]
	    		pathp.append(f)
			j.points = pathp
			self.pub.publish(j) 
	else:
		j = JointTrajectory()
		j.joint_names = self.joint_names
		pathp = []
		for i in range(len(self.pathf)):
	    		f = JointTrajectoryPoint()
	    		f.positions = self.pathf[i]
	    		pathp.append(f)
			j.points = pathp
			self.pub.publish(j)

    """ This function will perform IK for a given transform T of the end-effector.
    It returns a list q[] of values, which are the result positions for the 
    joints of the robot arm, ordered from proximal to distal. If no IK solution 
    is found, it returns an empy list.
    """
    def IK(self, T_goal):
        req = moveit_msgs.srv.GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state = moveit_msgs.msg.RobotState()
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.velocity = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.effort = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.header.stamp = rospy.get_rostime()
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped = geometry_msgs.msg.PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = self.base
        req.ik_request.pose_stamped.header.stamp = rospy.get_rostime()
        req.ik_request.pose_stamped.pose = convert_to_message(T_goal)
        req.ik_request.timeout = rospy.Duration(3.0)
        res = self.ik_service(req)
        q = []
        if res.error_code.val == res.error_code.SUCCESS:
            q = res.solution.joint_state.position
        return q

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


    """ This function checks if a set of joint angles q[] creates a valid state,
    or one that is free of collisions. The values in q[] are assumed to be values
    for the joints of the KUKA arm, ordered from proximal to distal. 
    """
    def is_state_valid(self, q):
        req = moveit_msgs.srv.GetStateValidityRequest()
        req.group_name = self.group_name
        req.robot_state = moveit_msgs.msg.RobotState()
        req.robot_state.joint_state.name = self.joint_names
        req.robot_state.joint_state.position = q
        req.robot_state.joint_state.velocity = numpy.zeros(self.num_joints)
        req.robot_state.joint_state.effort = numpy.zeros(self.num_joints)
        req.robot_state.joint_state.header.stamp = rospy.get_rostime()
        res = self.state_valid_service(req)
        return res.valid


'''This is a class which you can use to keep track of your tree branches.
It is easiest to do this by appending instances of this class to a list 
(your 'tree'). The class has a parent field and a joint position field (q). 
You can initialize a new branch like this:
RRTBranch(parent, q)
Feel free to keep track of your branches in whatever way you want - this
is just one of many options available to you.'''
class RRTBranch(object):
    def __init__(self, parent, q):
	self.parent = parent
	self.q = q


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm', anonymous=True)
    ma = MoveArm()
    rospy.spin()

