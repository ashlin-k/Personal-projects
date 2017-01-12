/*

This class will provide functions for a sumo-fighting robot.

*/

#ifndef WfBot_h
#define WfBot_h

#include "Arduino.h"

class WfBot {
  
  public:
    WfBot (const unsigned int m1, const unsigned int m2, const unsigned int m3, const unsigned int m4, const unsigned int s1, const unsigned int s2);
    void forward();
  
  private:
    unsigned int m1PWMpn, m2PWMpn, m1DIRpn, m2DIRpn; 
	unsigned int s1p, s2p;														
	double getS1();
	double getS2();
	void setSpeed(unsigned int vLeft, unsigned int vRight);
  
};




#endif