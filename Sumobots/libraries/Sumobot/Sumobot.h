/*

This class will provide functions for a sumo-fighting robot.

*/

#ifndef Sumobot_h
#define Sumobot_h

#include "Arduino.h"

class Sumobot {
  
  public:
    Sumobot (const unsigned int m1, const unsigned int m2, const unsigned int m3, const unsigned int p1, const unsigned int p2, 
		const unsigned int p3, const unsigned int p4, const unsigned int ir, const unsigned int s1, const unsigned int s2, const unsigned int s3, const unsigned int servo, unsigned int threshold);
    void forward(); 
    void checkWhiteLine();
    void setAngGoal(double i);
	void setThreshold(unsigned int threshold);
  
  private:
    double angGoal;
    unsigned int m1PWMp, m2PWMp, m1DIRp;                                         
	unsigned int p1p, p2p, p3p, p4p;
	unsigned int irp, servop;
	unsigned int son1p, son2p, son3p;
    unsigned int thresh;
    int t90, t180;
	void backward();
    void turnLeft(int time);
    void turnRight(int time); 
	double getSonar(unsigned int pin);
	double irCalcDist(unsigned int x);
  
};




#endif
