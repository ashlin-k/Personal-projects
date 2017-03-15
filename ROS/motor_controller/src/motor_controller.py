# from motor import Motor
# from encoder import Encoder
import time
import tty, sys
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import threading

leftDistance= 0.0
posX = 0.0
posY = 0.0
posZ = 0.0
lin_velX = 0.0
lin_velY = 0.0
lin_velZ = 0.0
ang_velX = 0.0
ang_velY = 0.0
ang_velZ = 0.0

def getCh():

        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

def odometryCallback(data):
    print "called odometryCallback"
    rospy.loginfo(rospy.get_caller_id() + "Getting odometry data.")
    posX = data.pose.pose.x
    posY = data.pose.pose.y
    posZ = data.pose.pose.z
    lin_velX = data.twist.twist.linear.x
    lin_velY = data.twist.twist.linear.y
    lin_velZ = data.twist.twist.linear.z
    ang_velX = data.twist.twist.angular.x
    ang_velY = data.twist.twist.angular.y
    ang_velZ = data.twist.twist.angular.z

def odometryListener():

    print "called odometryListener"
    rospy.init_node('odometrySubscriber', anonymous=True)
    rospy.Subscriber("odom", Odometry, odometryCallback)
    rospy.spin()

def callListener():
    odometryListener()

	
if __name__ == '__main__':

    try:
        # t0 = threading.Thread(target=callListener)
        # t0.start()
        odometryListener()
    except:
        print "Error: could not start listener thread."

    # mainMotors = Motor(8,10,12,16,18,22)
    # left = 0          # need to be determined with data from ROS
    # right = 0

    # encoder_left = Encoder(24,26)
    # encoder_right = Encoder(11,13)
    # while 1:
    #         key = getCh()
    #         if key=='i':
    #                 left+=20
    #         if key == 'k':
    #                 left-=20
            
    #         if key=='o':
    #                 right+=20
    #         if key=='l':
    #                 right-=20

    #         if key == 'q':
    #                 break;

    #         mainMotors.move(left,right)
    #         print left, right
    #         leftDistance = encoder_left.getDistance()
    #         rightDistance = encoder_right.getDistance()
    #         print "Distance left,right:",leftDistance, rightDistance
            
    # encoder_left.stopEncoder()
    # encoder_right.stopEncoder()
