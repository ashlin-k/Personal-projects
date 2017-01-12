#include "Arduino.h"
#include "Sumobot.h"
#include "Servo.h"

#define MAX_SPEED 100.
#define MIN_SPEED 10.
#define MAX_TURN_SPEED 200.
#define MIN_TURN_SPEED 5.

unsigned int m1PWMp, m2PWMp, m1DIRp, m2DIRp; 
unsigned int p1p, p2p, p3p, p4p;
unsigned int irp, servop;
unsigned int son1p, son2p, son3p;

Servo motor;
unsigned int mpPin;

double angGoal=0;
double angVel=1.;
unsigned int thresh = 600;
int t90=1000, t180=2000;			//SET

enum State { straight, turning };
State mode = straight;


Sumobot::Sumobot(const unsigned int m1, const unsigned int m3, const unsigned int m, const unsigned int p1, const unsigned int p2, const unsigned int p3, const unsigned int p4, const unsigned int ir, const unsigned int s1, const unsigned int s2, unsigned int s3, const unsigned int servo, unsigned int threshold) {
  
  m1PWMp = m1;    //pwm left wheel 
  //m2PWMp = m2;    //pwm right wheel 
  m1DIRp = m3;    //DIR left wheel 
  mpPin = m;
  p1p = p1;
  p2p = p2;
  p3p = p3;
  p4p = p4;
  irp = ir;
  son1p = s1;		//left front
  son2p = s2;		//left back
  son3p = s3;		//right front
  servop = servo;
  thresh = threshold;

  
  
  //initialize motor pins
  pinMode(m1PWMp, OUTPUT);
  //pinMode(m2PWMp, OUTPUT);
  pinMode(m1DIRp, OUTPUT);
  //pinMode(m2DIRp, OUTPUT); 
  motor.attach(mpPin);

  //initialize phototransistors
	pinMode(p1p, INPUT);
	pinMode(p2p, INPUT);
	pinMode(p3p, INPUT);
	pinMode(p4p, INPUT);

	//initialize sonar and ir sensors
	pinMode(son1p, INPUT);
	pinMode(son3p, INPUT);
	pinMode(irp, INPUT);

  
}

void Sumobot::setAngGoal (double i) {
  angGoal = i;
}

void Sumobot::setThreshold(unsigned int threshold) {
	thresh = threshold;
}

void Sumobot::checkWhiteLine(void) {
  
  /*
                FRONT
          -----------------
          |  P1        P2  |
          |                |
          |                |
          |                |
          |                |
          |  P3        P4  |
          ------------------
                BACK
  
  */
  
  unsigned int p1 = analogRead(p1p); 
  unsigned int p2 = analogRead(p2p); 
  unsigned int p3 = analogRead(p3p); 
  unsigned int p4 = analogRead(p4p); 

  Serial.print("p1 : ");
  Serial.println(p1);
  Serial.print("p2 : ");
  Serial.println(p2);
  Serial.print("p3 : ");
  Serial.println(p3);
  Serial.print("p4 : ");
  Serial.println(p4);

  if (p1 > thresh & p2 > thresh) {
    backward();
    turnRight(t180);
    forward();
	//Serial.println("white line in front");
  }
  
  else if (p3 > thresh & p4 > thresh) {
    forward();
	//Serial.println("white line in back");
  }
  
  else if (p1 > thresh & p3 > thresh) {
    turnRight(t90);
    forward();
	//Serial.println("white line on left");
  }
  
  else if (p2 > thresh & p4 > thresh) {
    turnLeft(t90);
    forward();
	//Serial.println("white line on right");
  }
  
  else {
    
    if (p1 > thresh) {
      backward();
      turnRight(t90);
      forward();
	  //Serial.println("white line p1");
    }
    else if (p2 > thresh) {
      backward();
      turnLeft(t90);
      forward();
	  //Serial.println("white line p2");
    }
    
    else if (p3 > thresh) {
      forward();
      delay(500);
      turnRight(t90);
      forward();
	  //Serial.println("white line p3");
    }
    else if (p4 > thresh) {
      forward();
      delay(500);
      turnLeft(t90);
      forward();
	  //Serial.println( "white line p4");
    }
    else {forward();}
    
  }
  
}

void Sumobot::forward() {

	double vLeft, vRight;
	unsigned int vNorm = 255; 

	if ((angGoal == 0) || (mode == turning)) {

		double s1dist = getSonar(son1p);
		double s3dist = getSonar(son3p);
		double irdist = irCalcDist(analogRead(irp));

		Serial.print("s1 : ");
  Serial.println(s1dist);
  Serial.print("s2 : ");
  Serial.println(s3dist);
  Serial.print("irdist : ");
  Serial.println(irdist);
  Serial.print("p4 : ");

		//turn left
		if (s1dist < 50) {
			vLeft = 0;
			vRight = 255;
			//Serial.println( "robot on the left");
		}

		//turn right
		else if (s3dist < 50) {
			vLeft = 255;
			vRight = 0;
			//Serial.println("robot on the right");
		}

		//go straight
		else {
			vLeft = vNorm;
			vRight = vNorm;
			mode = straight;
			//Serial.println("just go straight");
		}

	}

	else {

		double angNorm = (angGoal-400)/300;

		//set angVel
		if (fabs(angNorm) >= 1) { angVel = MAX_TURN_SPEED; }
		else if (angNorm == 0) { angVel = 0; }
		else { angVel = MAX_TURN_SPEED*fabs(angNorm) + MIN_TURN_SPEED; }
  
		//left turn
		if (angNorm < 0) {
			mode = turning;
			vLeft = fabs(vNorm - angVel);        //inner
			vRight = 2*vNorm - vLeft;			//outer
			if (vRight > 255) { vRight = 255; }
			//Serial.println("pi: go left");
		}
  
		//straight
		else if (angNorm == 0) {
			mode = straight;
			vLeft = vNorm;
			vRight = vNorm;
			//Serial.println("pi: go straight");
		}
  
		//right turn
		else {
			mode = turning;
			vRight = fabs(vNorm - angVel);      //inner
			vLeft = 2*vNorm - vRight;			//outer
			if (vLeft > 255) { vLeft = 255; }
			//Serial.println("pi: go straight");
			}

		Serial.print("vLeft: ");
		Serial.println(vLeft);
		Serial.print("vRight: ");
		Serial.println(vRight);
	}
  
	analogWrite(m1PWMp, vLeft);
	//analogWrite(m2PWMp, vRight);
	digitalWrite(m1DIRp, LOW);
	//digitalWrite(m2DIRp, LOW);
	motor.write(90);
  
}

void Sumobot::backward() {
  
  analogWrite(m1PWMp, 255);
  //analogWrite(m2PWMp, 255);
  digitalWrite(m1DIRp, HIGH);
  //digitalWrite(m2DIRp, HIGH);
  motor.write(0);
 
  delay(1000); 
}

void Sumobot::turnRight (int time) {
    
    analogWrite(m1PWMp, 255);
    analogWrite(m2PWMp, 255);
    digitalWrite(m1DIRp, LOW);
    digitalWrite(m2DIRp, HIGH);
	delay(time);
  
}
  


void Sumobot::turnLeft (int time) {
  
	analogWrite(m1PWMp, 255);
    analogWrite(m2PWMp, 255);
    digitalWrite(m1DIRp, HIGH);
    digitalWrite(m2DIRp, LOW);
	delay(time);
  
}

double Sumobot::getSonar(unsigned int pin) {

	long time;
	double dist;

	pinMode(pin, OUTPUT);
	digitalWrite(pin, LOW);
	delayMicroseconds(2);
	digitalWrite(pin, HIGH);
	delayMicroseconds(5);
	digitalWrite(pin, LOW);
  
	pinMode(pin, INPUT);
	time = pulseIn(pin, HIGH);
	dist = time / 29 / 2;		//convert time (microsec) to distance in cm given the speed of sound is 340 m/s

	return dist;

}

double Sumobot::irCalcDist(unsigned int x) {
  double dist = 23680*pow(x,-1.123);
  return dist;
}


  



