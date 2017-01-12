#include "Arduino.h"
#include "WfBot.h"

#define MAX_SPEED 255.
#define MID_SPEED 220.
#define TOLERANCE 0.5

#define MAX_DIST 37
#define MIN_DIST 35

unsigned int m1PWMpn, m2PWMpn, m1DIRpn, m2DIRpn; 
unsigned int s1p, s2p;

WfBot::WfBot (const unsigned int m1, const unsigned int m2, const unsigned int m3, const unsigned int m4, const unsigned int s1, const unsigned int s2) {

	m1PWMpn = m1;    //pwm left wheel
	m2PWMpn = m2;    //pwm right wheel 
	m1DIRpn = m3;    //dir left wheel
	m2DIRpn = m4;    //dir right wheel
	s1p = s1;		//front sonar
	s2p = s2;		//back sonar
  
	//initialize motor pins
	pinMode(m1PWMpn, OUTPUT);
	pinMode(m2PWMpn, OUTPUT);
	pinMode(m1DIRpn, OUTPUT);
	pinMode(m2DIRpn, OUTPUT);  

}

void WfBot::forward(void) {

	double dist1, dist2, diff;

	dist1 = getS1();
	dist2 = getS2();

	Serial.print("S1: ");
	Serial.println(dist1);
	Serial.print("S2: ");
	Serial.println(dist2);

	diff = fabs(dist1 - dist2);

	//NOT PARALLEL TO WALL
	if (diff > TOLERANCE) {
		// not parallel
		if (dist1 > dist2) {
			setSpeed(MAX_SPEED, MID_SPEED);		//veer right
			Serial.println("not parallel, front wheel far");
		}
		else {
			setSpeed(MID_SPEED, MAX_SPEED);		//veer left
			Serial.println("not parallel, front wheel far");
		}
	}

	//PARALLEL TO WALL
	else {
		//too far
		if ((dist1 > MAX_DIST) | (dist2 > MAX_DIST)) {
			setSpeed(MAX_SPEED, MID_SPEED);		//veer right			
			setSpeed(MID_SPEED, MAX_SPEED);		//veer left
			setSpeed(MAX_SPEED, MAX_SPEED);		//straight
			Serial.println("parallel, too far");
		}
		//too close
		else if ((dist1 < MIN_DIST) | (dist2 < MIN_DIST)) {			
			setSpeed(MID_SPEED, MAX_SPEED);		//veer left
			setSpeed(MAX_SPEED, MID_SPEED);		//veer right
			setSpeed(MAX_SPEED, MAX_SPEED);		//straight
			Serial.println("parallel, too close");
		}
		//on target
		else {
			//go straight
			setSpeed(MAX_SPEED, MAX_SPEED);		//straight
			Serial.println("parallel!!!");
		}
	}

}

double WfBot::getS1(void) {

	long time;
	double dist;

	pinMode(s1p, OUTPUT);
	digitalWrite(s1p, LOW);
	delayMicroseconds(2);
	digitalWrite(s1p, HIGH);
	delayMicroseconds(5);
	digitalWrite(s1p, LOW);
  
	pinMode(s1p, INPUT);
	time = pulseIn(s1p, HIGH);
	dist = time / 29 / 2;		//convert time (microsec) to distance in cm given the speed of sound is 340 m/s

	return dist;

}

double WfBot::getS2(void) {

	long time;
	double dist;

	pinMode(s2p, OUTPUT);
	digitalWrite(s2p, LOW);
	delayMicroseconds(2);
	digitalWrite(s2p, HIGH);
	delayMicroseconds(5);
	digitalWrite(s2p, LOW);
  
	pinMode(s2p, INPUT);
	time = pulseIn(s2p, HIGH);
	dist = time / 29 / 2;		//convert time (microsec) to distance in cm given the speed of sound is 340 m/s

	return dist;

}

void WfBot::setSpeed(unsigned int vLeft, unsigned int vRight) {
	analogWrite(m1PWMpn, vLeft);
	analogWrite(m2PWMpn, vRight);
	digitalWrite(m1DIRpn, LOW);
	digitalWrite(m2DIRpn, LOW);
	delay(1000);
}