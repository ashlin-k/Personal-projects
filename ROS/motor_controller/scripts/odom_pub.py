#!/usr/bin/env python

from nav_msgs.msg import Odometry
import rospy
from math import sin, cos, pi
import tf
import time

seq = 0
pos_x = 0.0
pos_y = 0.0
pos_theta = 0.0
vel_lin_x = 0.0
vel_lin_y = 0.0
vel_ang_z = 0.0
R = 0.012			# base width; distance between wheels
time_delay = 0.01


def getOdometry():
	# reads the encoders and writes data to an Odometry object
	# returns the Odometry object

	global encoder_left, encoder_right, pos_x, pos_y, pos_theta, vel_lin_x, vel_lin_y, vel_ang_z, seq

	odom = Odometry()
	d_left = 0.0
	d_right = 0.0
	v_left = 0.0
	v_right = 0.0

	if (encoder_left != None) and (encoder_right != None):
		d_left = encoder_left.getDistance()
		d_right	= encoder_right.getDistance()
		v_left = encoder_left.getVelocity()
		v_right = encoder_right.getVelocity()

	distance = ( d_left + d_right ) / 2
	theta = ( d_left - d_right ) / R

	if (distance != 0):
	        # calculate distance traveled in x and y
        	x = cos( theta ) * distance
        	y = -sin( theta ) * distance
        	# calculate the final position of the robot
        	if( theta != 0): pos_theta = pos_theta + th
        	pos_x = pos_x + ( cos( pos_theta ) * x - sin( pos_theta ) * y )
        	pos_y = pos_y + ( sin( pos_theta ) * x + cos( pos_theta ) * y )


    	vel_lin_x = (R/2) * (v_left + v_right) * cos(pos_theta)
    	vel_lin_y = (R/2) * (v_left + v_right) * sin(pos_theta)
    	vel_ang_z = (v_right - v_left) / R

    	odom_quat = tf.transformations.quaternion_from_euler(0, 0, pos_theta)

    	odom.header.seq = seq
    	seq = seq + 1
    	odom.header.stamp = rospy.get_rostime()
    	odom.header.frame_id = "odom"
    	odom.child_frame_id = "base_link"
    	odom.pose.pose.position.x = pos_x
    	odom.pose.pose.position.y = pos_y
    	odom.pose.pose.position.z = 0.0
    	# odom.pose.pose.orientation = Quaternion(*odom_quat)
    	odom.pose.pose.orientation.x = odom_quat[0]
    	odom.pose.pose.orientation.y = odom_quat[1]
    	odom.pose.pose.orientation.z = odom_quat[2]
    	odom.pose.pose.orientation.w = odom_quat[3]
    	odom.twist.twist.linear.x = vel_lin_x
    	odom.twist.twist.linear.y = vel_lin_y
    	odom.twist.twist.linear.z = 0.0
    	odom.twist.twist.angular.x = 0.0
    	odom.twist.twist.angular.y = 0.0
    	odom.twist.twist.angular.z = vel_ang_z

	return odom


if __name__=="__main__":

	global encoder_left, encoder_right

	rospy.init_node('odometry_node')
	pub = rospy.Publisher('odom', Odometry, queue_size = 1)

	encoder_left = None
	encoder_right = None
	# encoder_left = Encoder(24,26)
	# encoder_right = Encoder(11,13)

	while not rospy.is_shutdown():

		odom = getOdometry()

		pub.publish(odom)

		time.sleep(time_delay)
