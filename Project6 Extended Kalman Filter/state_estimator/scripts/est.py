#!/usr/bin/env python

# Columbia Engineering
# MECS 4602 - Fall 2018

import math
import numpy
import time

import rospy

from state_estimator.msg import RobotPose
from state_estimator.msg import SensorData

class Estimator(object):
    def __init__(self):

        # Publisher to publish state estimate
        self.pub_est = rospy.Publisher("/robot_pose_estimate", RobotPose, queue_size=1)

        # Initial estimates for the state and the covariance matrix
        self.x = numpy.zeros((3,1))
        self.P = numpy.zeros((3,3))
	
        # Covariance matrix for process (model) noise
        self.V = numpy.zeros((3,3))
        self.V[0,0] = 0.0025
        self.V[1,1] = 0.0025
        self.V[2,2] = 0.005

        self.step_size = 0.01

        # Subscribe to command input and sensory output of robot
        rospy.Subscriber("/sensor_data", SensorData, self.sensor_callback)
        
    # This function gets called every time the robot publishes its control 
    # input and sensory output. You must make use of what you know about 
    # extended Kalman filters to come up with an estimate of the current
    # state of the robot and covariance matrix.
    # The SensorData message contains fields 'vel_trans' and 'vel_ang' for
    # the commanded translational and rotational velocity respectively. 
    # Furthermore, it contains a list 'readings' of the landmarks the
    # robot can currently observe
    def estimate(self, sens):

        #### ----- YOUR CODE GOES HERE ----- ####
        
        xk = self.x[0]
        yk = self.x[1]
        thetak = self.x[2]
        
        t= self.step_size
        vt = sens.vel_trans
        va = sens.vel_ang

        xk1p = numpy.zeros((3,1))
	xk1p[0] = xk + t*vt*math.cos(thetak)
	xk1p[1] = yk + t*vt*math.sin(thetak)
	xk1p[2] = thetak + t*va
	
	F = numpy.eye(3)
	F[0][2] = -t*vt*math.sin(thetak)
	F[1][2] = t*vt*math.cos(thetak)
	
	Pk1p = self.V + numpy.dot(numpy.dot(F,self.P),numpy.transpose(F))
	
	S = 0	
	H = 0
	Wk1 = 0
	R = 0
	nu = 0
	yk1 = 0
	hxk1 = 0
	
	s_num = len(sens.readings)
	if s_num > 0:
		H = numpy.zeros((2*s_num,3))
		Wk1 = numpy.zeros((2*s_num,2*s_num))
		yk1 = numpy.zeros((2*s_num,1))
		hxk1 = numpy.zeros((2*s_num,1))
		
		for i in range(s_num):
			dx = xk1p[0]-sens.readings[i].landmark.x
			dy = xk1p[1]-sens.readings[i].landmark.y
			dxy= math.sqrt(dx**2+dy**2)
			
			if dxy > 0.1:	
				H[i*2][0]= dx/dxy
				H[i*2][1]= dy/dxy
				H[i*2][2]= 0
				H[i*2+1][0]= -dy/(dxy**2)
				H[i*2+1][1]= dx/(dxy**2)
				H[i*2+1][2]= -1

			Wk1[i*2][i*2] = 0.1
			Wk1[i*2+1][i*2+1] = 0.05

			yk1[2*i]= sens.readings[i].range
			yk1[2*i+1] = sens.readings[i].bearing

			hxk1[2*i] = dxy
			hxk1[2*i+1] = math.atan2(-dy, -dx)-xk1p[2]

		nu = yk1 - hxk1
		
		for m in range(s_num):
			if abs(nu[2*m+1]) > numpy.pi:
				if nu[2*m+1]<0:
					nu[2*m+1] += 2*numpy.pi
				else:
					nu[2*m+1] -= 2*numpy.pi
		
		S = Wk1 + numpy.dot(numpy.dot(H, Pk1p), numpy.transpose(H)) 
		R = numpy.dot(numpy.dot(Pk1p, numpy.transpose(H)), numpy.linalg.inv(S))
		
		
	self.x = xk1p + numpy.dot(R, nu)
	self.P = Pk1p - numpy.dot(numpy.dot(R, H), Pk1p)


		


        #### ----- YOUR CODE GOES HERE ----- ####
    
    def sensor_callback(self,sens):
        # Publish state estimate 
        self.estimate(sens)
        est_msg = RobotPose()
        est_msg.header.stamp = sens.header.stamp
        est_msg.pose.x = self.x[0]
        est_msg.pose.y = self.x[1]
        est_msg.pose.theta = self.x[2]
        self.pub_est.publish(est_msg)

if __name__ == '__main__':
    rospy.init_node('state_estimator', anonymous=True)
    est = Estimator()
    rospy.spin()
