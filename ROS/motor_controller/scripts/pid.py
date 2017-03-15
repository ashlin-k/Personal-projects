#! /usr/bin/python

import tty, sys
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sin, cos, pi

class PID:

	def __init__(self, Kp, Ki, Kd):

		self.Kp = Kp
		self.Ki = Ki
		self.Kd = Kd

        	self.integral_dx = 0.0
        	self.integral_dy = 0.0
        	self.integral_dr = 0.0
        	self.prevError_dx = 0.0
        	self.prevError_dy = 0.0
        	self.prevError_dr = 0.0

		self.base_width = 0.012 	# wheel base width in meters
		self.MAX_LIN_VELOCITY = 3.0
		self.MAX_ANG_VELOCITY = pi/2
		self.time_prev = 0.0



	def runPid(self, twist, odom):

		target_dx = 0.0
		target_dy = 0.0
		target_dr = 0.0
		measured_dx = 0.0
		measured_dy = 0.0
		measured_dr = 0.0

		try:
			# print "before get"
			target_dx = twist.linear.x
			target_dy = twist.linear.y
			target_dr = twist.angular.z
			measured_dx = odom.twist.twist.linear.x
			measured_dy = odom.twist.twist.linear.y
			measured_dr = odom.twist.twist.angular.z
			# print "after get"
		except:
			print "Cannot access twist or odom info."

		if (target_dx > self.MAX_LIN_VELOCITY): target_dx = self.MAX_LIN_VELOCITY
		elif (target_dx < -1*self.MAX_LIN_VELOCITY): target_dx = (-1) * self.MAX_LIN_VELOCITY


		error_dx = target_dx - measured_dx
		error_dy = target_dy - measured_dy
		error_dr = target_dr - measured_dr
		time_now = rospy.get_time()
		deltaT = time_now - self.time_prev
		self.time_prev = time_now
		# proportional
	    	proportional_dx = error_dx
	    	proportional_dy = error_dy
	    	proportional_dr = error_dr
	    	# integral
	    	self.integral_dx = self.integral_dx + error_dx * deltaT
	    	self.integral_dy = self.integral_dy + error_dy * deltaT
	    	self.integral_dr = self.integral_dr + error_dr * deltaT
	    	# derivative
	    	derivative_dx = (error_dx - self.prevError_dx) / deltaT
	    	derivative_dy = (error_dy - self.prevError_dy) / deltaT
	    	derivative_dr = (error_dr- self.prevError_dr) / deltaT
	    	self.prevError_dx = error_dx
	    	self.prevError_dy = error_dy
	    	self.prevError_dr = error_dr
	    	# output
	    	output_dx = self.Kp*proportional_dx + self.Ki*self.integral_dx + self.Kd*derivative_dx
	    	output_dy = self.Kp*proportional_dy + self.Ki*self.integral_dy + self.Kd*derivative_dy
	    	output_dr = self.Kp*proportional_dr + self.Ki*self.integral_dr + self.Kd*derivative_dr

	    	if (output_dx > self.MAX_LIN_VELOCITY): output_dx = self.MAX_LIN_VELOCITY
	    	elif (output_dx < -1*self.MAX_LIN_VELOCITY):output_dx = -1 * self.MAX_LIN_VELOCITY 

	    	if (output_dy > self.MAX_LIN_VELOCITY): output_dy = self.MAX_LIN_VELOCITY
	    	elif (output_dy < -1*self.MAX_LIN_VELOCITY):output_dy = -1 * self.MAX_LIN_VELOCITY 

	    	if (output_dr > self.MAX_ANG_VELOCITY): output_dr = self.MAX_ANG_VELOCITY
	    	elif (output_dr < -1*self.MAX_ANG_VELOCITY):output_dr = -1 * self.MAX_ANG_VELOCITY 

	    	rospy.loginfo("PID: self_dx: %f, error dx: %f, output dx: %f, output dr: %f" % (measured_dx, error_dx, output_dx, output_dr))

	    	return output_dx, output_dy, output_dr


