#! /usr/bin/python

from motor import Motor
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

Kp = 5.0
Ki = 0.1
Kd = 25.0

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


def twistCallback(data):
    # twist data
    # linear velocities are in m/s
    # angular velocities are in rad/s

    global motors

    R = 0.0
    rVelocity = 0.0
    lVelocity = 0.0

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
        
    rospy.loginfo("SUB: dx: %f, dr: %f, Right Motor: %f, Left Motor: %f" % (data.linear.x, data.angular.z, rVelocity, lVelocity))


def odometryCallback(data):

    curr_odom = data
    # rospy.loginfo("Recieved odometry information")


    
if __name__ == '__main__':

    global encoder_left, encoder_right, pid, motors

    print "Starting subscriber"

    try:

        motors = None
        motors = Motor(8,10,12,16,18,22)
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

    


