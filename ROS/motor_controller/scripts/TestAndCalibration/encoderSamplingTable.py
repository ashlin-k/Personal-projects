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
startingSamplingSpeedRight = 1000
startingSamplingSpeedLeft = 1000

while 1:
        mainMotors.move(left,right)

	encoder_right.setSamplingSpeed(startingSamplingSpeedRight)
	encoder_left.setSamplingSpeed(startingSamplingSpeedLeft)

	print "Duty: ",dutyCycle

	#Right Motor
	while True:
		startingSamplingSpeedRight +=500
		encoder_right.setSamplingSpeed(startingSamplingSpeedRight)
		time.sleep(3)
		if (encoder_right.getNumOfErrors() < dutyCycle):
		#	print "Break with: ", encoder_right.getNumOfErrors()
			break
		encoder_right.setNumOfErrors(0)

	print "Right: ", startingSamplingSpeedRight,
	
	#Left Motor	
	while True:
		startingSamplingSpeedLeft +=500
		encoder_left.setSamplingSpeed(startingSamplingSpeedLeft)
		time.sleep(3)
		if (encoder_left.getNumOfErrors() < dutyCycle):
		#	print "Break with: ", encoder_right.getNumOfErrors()
			break
		encoder_left.setNumOfErrors(0)
	
	print "Left: ", startingSamplingSpeedLeft

	left = dutyCycle
	right = dutyCycle

	dutyCycle+=5
	if dutyCycle > 100:
		break

encoder_left.stopEncoder()
