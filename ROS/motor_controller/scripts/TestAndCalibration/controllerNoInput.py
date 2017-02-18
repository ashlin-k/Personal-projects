from motor import Motor
from encoder import Encoder
import time
import tty, sys

leftDistance= 0.0

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


	
mainMotors = Motor(8,10,12,16,18,22)
left = 0
right = 0

encoder_left = Encoder(24,26)
encoder_right = Encoder(11,13)
left = 50
right = 50

count = 0
while 1:
        mainMotors.move(left,right)
        leftVel = encoder_left.getVelocity()
        rightVel = encoder_right.getVelocity()
        print "Velocity left,right:",leftVel, rightVel


	time.sleep(0.5)
	left = count
	right =  count

	count+=1
	if count >= 10:
		break



encoder_left.stopEncoder()
encoder_right.stopEncoder()
