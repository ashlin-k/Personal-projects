#! /usr/bin/python

# from motor import Motor
import time
import tty, sys
import threading
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
from pid import PID
from math import sin, cos, pi

MAX_LINEAR_VELOCITY = 5.0       # m/s
MAX_ANGULAR_VELOCITY = 5.0       # m/s

AXEL_LENGTH = 0.15 #meters

curr_odom = Odometry()

Kp = 1.5
Ki = 0.1
Kd = 1.5

runWithPid = False

def convertVelocitiesToWheelSpeeds(dx, dy, dr, R):
	# takes output from pid (dx, dy, dr) and converts it to wheel speeds

	# original equations
	# 1. (Vr - Vl)/R = dr
	# 2. dx = (R/2) * (Vr + Vl) * cos(theta)
	# 3. dy = (R/2) * (Vr + Vl) * sin(theta)

	euler = tf.transformations.euler_from_quaternion([curr_odom.pose.pose.orientation.x, curr_odom.pose.pose.orientation.y, \
		curr_odom.pose.pose.orientation.z, curr_odom.pose.pose.orientation.w])
	theta = euler[2]

	if (R != 0):
		if (cos(theta) != 0): Vl = dx/(R*cos(theta)) - 0.5*dr*R
		else: Vl = dy/(R*sin(theta)) - 0.5*dr*R
		if (sin(theta) != 0): Vr = dy/(R*sin(theta)) + 0.5*dr*R
		else: Vr = dx/(R*cos(theta)) + 0.5*dr*R
	else:
		Vl = dx
		Vr = dx

	return Vl, Vr

# FOR DEBUGGING WITH FAKE ODOM DATA
pos_x = 0.0
pos_y = 0.0
vel_lin_x = 0.0
vel_ang_z = 0.0

def twistCallback(data):
	# twist data
	# linear velocities are in m/s
	# angular velocities are in rad/s

	global motors 
	global pos_x, pos_y, vel_lin_x, vel_ang_z

	R = 0.0
	rVelocity = 0.0
	lVelocity = 0.0

	####### FAKE ODOM DATA FOR TESTING #######
	odom = Odometry()
	odom_quat = tf.transformations.quaternion_from_euler(0, 0, 0)
	odom.header.stamp = rospy.get_rostime()
	odom.header.frame_id = "odom"
	odom.child_frame_id = "base_link"
	odom.pose.pose.position.x = pos_x
	pos_x = pos_x + 1
	odom.pose.pose.position.y = pos_y
	odom.pose.pose.position.z = 0.0
	# odom.pose.pose.orientation = Quaternion(*odom_quat)
	odom.pose.pose.orientation.x = odom_quat[0]
	odom.pose.pose.orientation.y = odom_quat[1]
	odom.pose.pose.orientation.z = odom_quat[2]
	odom.pose.pose.orientation.w = odom_quat[3]
	
	sign = 1
	if ( vel_lin_x > (data.linear.x + 0.005) ) and (data.linear.x != 0):
		sign = -1
	elif ( vel_lin_x < (data.linear.x - 0.005) ):
		sign = 1
	elif (data.linear.x == 0):
		sign = 0
	vel_lin_x = vel_lin_x + sign * 0.0025

	odom.twist.twist.linear.x = vel_lin_x
	odom.twist.twist.linear.y = 0
	odom.twist.twist.linear.z = 0.0
	odom.twist.twist.angular.x = 0.0
	odom.twist.twist.angular.y = 0.0
	
	if ( vel_ang_z > (data.angular.z + 0.005) ) and (data.angular.z != 0):
		sign = -1
	elif ( vel_ang_z < (data.angular.z - 0.005) ) and (data.angular.z != 0):
		sign = 1
	elif (data.angular.z == 0):
		sign = 0
	vel_ang_z = vel_ang_z + sign * 0.0025

	odom.twist.twist.angular.z = vel_ang_z
	curr_odom = odom
	####### FAKE ODOM DATA FOR TESTING #######

	if (not runWithPid):
		W = data.angular.z
		rVelocity = data.linear.x
		lVelocity = data.linear.x
		if (W != 0):
			R = data.linear.x / W
			rVelocity = (R + AXEL_LENGTH/2.0) - W
			lVelocity = (R - AXEL_LENGTH/2.0) + W
		if (motors != None):
			motors.moveVelocity(rVelocity,lVelocity)

	else:
		new_dx, new_dy, new_dr = pid.runPid(data, curr_odom)
		lVelocity, rVelocity = new_dx, new_dx
		if (new_dr != 0):
			R = data.linear.x / new_dr        
			lVelocity, rVelocity = convertVelocitiesToWheelSpeeds(new_dx, new_dy, new_dr, R)  
		if (motors != None):            
			motors.moveVelocity(lVelocity, rVelocity)
		
		rospy.loginfo("SUB: new_dx: %f, new_dr: %f, Right Motor: %f, Left Motor: %f" % (new_dx, new_dr, rVelocity, lVelocity))


def odometryCallback(data):

	curr_odom = data
	# rospy.loginfo("Recieved odometry information")


	
if __name__ == '__main__':

	global encoder_left, encoder_right, pid, motors

	print "Starting subscriber"

	try:

		motors = None
		# motors = Motor(8,10,12,16,18,22)
		print "...initialized motors..."

		pid = PID(Kp, Ki, Kd)
		print "...created PID..."
		runWithPid = True

		rospy.init_node('motorSubscriber', anonymous=True)
		rospy.Subscriber("cmd_vel", Twist, twistCallback)
		rospy.Subscriber("odom", Odometry, odometryCallback)
		rospy.spin()

	except:
		print "Error: could not start motor or PID."

	


