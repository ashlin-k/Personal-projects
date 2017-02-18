import RPi.GPIO as GPIO
import time

class Motor:
	#A class that defines the interface to a motor-controller with the Raspberry pi

	#Definitions of the pins
	#Motor 1
	PWMA_pin = 0
	AIN1_pin = 0
	AIN2_pin = 0


	#Motor 2
	PWMB_pin = 0
	BIN1_pin = 0
	BIN2_pin = 0

	#Define motor object https://stackoverflow.com/questions/31467147/how-to-declare-uninitialized-variable-in-class-definition-in-python
	motor1 = None
	motor2 = None

	def __init__(self,PWMA,AIN1,AIN2,PWMB,BIN1,BIN2):
		self.PWMA_pin = PWMA
		self.AIN1_pin = AIN1
		self.AIN2_pin = AIN2

		self.PWMB_pin = PWMB
		self.BIN1_pin = BIN1
		self.BIN2_pin = BIN2

		self.setupPins()

	def setupPins(self):
		GPIO.setmode(GPIO.BOARD) #As indicated a good idea by: https://sourceforge.net/p/raspberry-gpio-python/wiki/BasicUsage/
		outputPins= [self.AIN1_pin, self.AIN2_pin, self.BIN1_pin, self.BIN2_pin]
		pwmPins = [self.PWMA_pin, self.PWMB_pin]

		GPIO.setup(outputPins+pwmPins,GPIO.OUT)

		#initialize the pins with 0 duty cycle
		self.motor1 = GPIO.PWM(self.PWMA_pin, 50) 
		self.motor1.start(0)
		self.motor2 = GPIO.PWM(self.PWMB_pin, 50)
		self.motor2.start(0)

		print "Initialization complete, motor pins:"+str(outputPins)+" PWM: "+str(pwmPins)

	#This function assumes the left motor is wired as Motor1/ M_A and right motor is wired as Motor2/ M_B
	#This function takes the duty cycle as an input
	def move(self,leftMotorDuty,rightMotorDuty):
		#The input values for velocity should be values between -100 to 100, corresponding to specific duty cycle of the motor, and direction of motion
		#If the value for the velocity is +ve, it means the motor is moving forward
		#If the value for the velocity is -ve, it means the motor is moving backward
		if (abs(leftMotorDuty) > 80 or abs(rightMotorDuty) > 80):
			print ("Invalid motor command, values should be between 0 and 80, received:"+str(leftMotorDuty)+","+str(rightMotorDuty))

		#print "Received:"+str(leftMotorDuty)+","+str(rightMotorDuty) + ","+str(self.PWMA_pin)
		#Left Motor
		#Moving Forward
		if (leftMotorDuty > 0):
			GPIO.output(self.AIN1_pin,GPIO.HIGH)
			GPIO.output(self.AIN2_pin,GPIO.LOW)
			self.motor1.ChangeDutyCycle(abs(leftMotorDuty))

		#Moving Backward	
		if (leftMotorDuty < 0):
			GPIO.output(self.AIN1_pin,GPIO.LOW)
			GPIO.output(self.AIN2_pin,GPIO.HIGH)
			self.motor1.ChangeDutyCycle(abs(leftMotorDuty))

		#Right Motor
		#Moving Forward
		if (rightMotorDuty > 0):
			GPIO.output(self.BIN1_pin,GPIO.HIGH)
			GPIO.output(self.BIN2_pin,GPIO.LOW)
			self.motor2.ChangeDutyCycle(abs(rightMotorDuty))
		#Moving Backward	
		if (rightMotorDuty < 0):
			GPIO.output(self.BIN1_pin,GPIO.LOW)
			GPIO.output(self.BIN2_pin,GPIO.HIGH)
			self.motor2.ChangeDutyCycle(abs(rightMotorDuty))

	#This Function will take as input velocity instead of duty cycle
	#A table relating the duty cycle to the input velocity will be used
	#Table is generated by calibration script
	def moveVelocity(self,leftVelocity,rightVelocity):
		#Values taken from calibration data
		leftDuty = (leftVelocity)/0.066
		rightDuty = (rightVelocity)/0.085
		self.move(leftDuty, rightDuty)


	#This function will stop the motors, and will also clean up the pins
	#After this function is called, the object needs to be reinitialized
	#This function should only be called at absolute motor stop, and not during an object avoidance stop
	def stopMotors(self):
		self.motor1.stop()
		self.motor2.stop()
		GPIO.cleanup()

	def test(self):
		import time
		print "Starting Tet"
	#	GPIO.setmode(GPIO.BOARD)
		#GPIO.setup(self.PWMA_pin,GPIO.OUT)
	#	GPIO.setup(8,GPIO.OUT)
		#pwm = GPIO.PWM(self.PWMA_pin,50)
		#pwm = GPIO.PWM(8,50)
		#pwm.start(0)
		for dc in range(0, 101, 5):
			print dc               
			#pwm.ChangeDutyCycle(dc)
			self.motor1.ChangeDutyCycle(dc)
			time.sleep(0.1)
		#pwm.stop()
					




