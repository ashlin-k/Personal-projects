from motor import Motor
from encoder import Encoder
import time
import tty, sys


mainMotors = Motor(8,10,12,16,18,22)
left = 0
right = 0

encoder_left = Encoder(24,26)
encoder_right = Encoder(11,13)
left = 0
right = 0


dutyCycle = 5
startingSamplingSpeedLeft = 13000
startingSamplingSpeedRight = 13000

left_velocity=0
right_velocity=0
encoder_right.setSamplingSpeed(startingSamplingSpeedRight)
encoder_left.setSamplingSpeed(startingSamplingSpeedLeft)

while 1:
        mainMotors.move(left,right)
	
	for i in range(5):
		time.sleep(0.5)
		left_velocity += encoder_left.getVelocity()
		right_velocity += encoder_right.getVelocity()


	left_velocity /= 5
	right_velocity /= 5

	
	f= file("dutyCycleTableBackward","a+")
	print "Duty: ",dutyCycle, left_velocity, right_velocity
	f.write(str(dutyCycle)+","+str(left_velocity)+","+str(right_velocity)+"\n")
	f.close()


	left_velocity = 0
	right_velocity = 0
	

	left = dutyCycle
	right = dutyCycle

	dutyCycle-=1
	if dutyCycle < -100:
		break
encoder_left.stopEncoder()
