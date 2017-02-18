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


while 1:
        key = getCh()
        if key=='i':
                left+=20
        if key == 'k':
                left-=20
        
        if key=='o':
                right+=20
        if key=='l':
                right-=20

        if key == 'q':
                break;

        mainMotors.move(left,right)
        print left, right
        leftDistance = encoder_left.getDistance()
        rightDistance = encoder_right.getDistance()
	print encoder_left.getVelocity()
        print "Distance left,right:",leftDistance, rightDistance
        
encoder_left.stopEncoder()
encoder_right.stopEncoder()
