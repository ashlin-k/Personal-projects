import RPi.GPIO as GPIO
import time
# import threading
from multithreading import Process

class Encoder:
	#A class that defines the interface to a quadrature encoder with the Raspberry pi

	#Deinition of the pins
	ENCAIN_pin = 0  #Encoder A input pin
	ENCBIN_pin = 0  #Encoder B input pin

	oldTime = time.time()
	old_distance = 0    #old encoder position, used to calcualted velocity
	ENC_pos = 0     #Encoder position

	pi = 3.141592653589793238462643383279
	wheelRadius = 0.03 #m
	circ = 2*wheelRadius*pi        #Circumference of gearbox output shaft
	GR = 224        #Gear ratio
	cpr = 12        #Counts per revolution
	cprgb = cpr*GR  #Counts per revolution of the gearbox output shaft

	oldA = None
	oldB = None

	numOfErrors = 0 

	samplingSpeed = 15000 # in Hz
   
	def __init__(self,AIN,BIN):
		self.ENCAIN_pin = AIN
		self.ENCBIN_pin = BIN

		self.setupPins()

		self.oldA = self.readA()
		self.oldB = self.readB()

	#     t0 = threading.Thread(target=self.run_encoders)
	# t0.start()
	p0 = Process(target=self.run_encoders, args=())
	p0.start()
	print "Starting run_encoders thread."



	def setupPins(self):
		GPIO.setmode(GPIO.BOARD) #As indicated a good idea by: https://sourceforge.net/p/raspberry-gpio-python/wiki/BasicUsage/
		inputPins = [self.ENCAIN_pin, self.ENCBIN_pin]

		GPIO.setup(inputPins,GPIO.IN)

		print "Initialization complete, encoder pins: " + str(inputPins)

	def readA(self):
		return GPIO.input(self.ENCAIN_pin)

	def readB(self):
		return GPIO.input(self.ENCBIN_pin)
		
	def rotateEncoder(self):

		#Make sure below isn't None on first run
		
		sigA = self.readA()
		sigB = self.readB()

		if sigA==self.oldB and self.oldA!=sigB:
			# ccw - A following B
			self.ENC_pos = self.ENC_pos+1
		elif sigA!=self.oldB and self.oldA==sigB:
			# cw - B following A
			self.ENC_pos = self.ENC_pos-1

		#if self.oldA==sigA and self.oldB==sigB:
	#   print "Nothing"
		#pass
		if self.oldA!=sigA and self.oldB!=sigB:
		self.numOfErrors +=1
		#print "Error"
		
		self.oldA = sigA
		self.oldB = sigB

	# returns distance in meters
	def getDistance(self):
	   thetaEnc = self.ENC_pos * (2 * self.pi / 12)    # in radians. this is total radians turned since starting and can be greater than 2*pi
	   thetaWheel = thetaEnc / self.GR
	   arcLength = self.wheelRadius * thetaWheel  # this is equivalent to linear distance
	   return  arcLength
	   # return float(self.ENC_pos * self.circ) / self.cprgb

	   
	def getVelocity(self):
	   velocity = (self.getDistance() - self.old_distance)/(time.time() - self.oldTime)
	   self.oldTime = time.time()
	   self.old_distance = self.getDistance()   
	   return velocity

	def setSamplingSpeed(self, speed):
		if (speed > 15000):
		speed = 15000
		print "Sampling speed should not exceed 15000 on the raspberry pi, capping at 15000"
	if (speed <= 0):
		speed = 1000
		print "Sampling speed should be >0, capping at 1000"
		self.samplingSpeed = speed
	 
	def setNumOfErrors(self, num):
		self.numOfErrors = num

	def getNumOfErrors(self):
		return self.numOfErrors

	def stopEncoder(self):
		GPIO.cleanup()

	def run_encoders(self):
	while 1:
			self.rotateEncoder()
		time.sleep(1/self.samplingSpeed)

