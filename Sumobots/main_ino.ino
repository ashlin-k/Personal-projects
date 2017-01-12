//#include "WfBot.h"
#include "Arduino.h"
#include "Sumobot.h"
#include "Servo.h"

unsigned int threshold = 600;
boolean threshChecked = false;

//Servo myServo;

//buttons
const unsigned int modeButtonPin = 10;

//LEDs
const unsigned int LED1pin = 6;    //green
const unsigned int LED2pin = 7;    //red
const unsigned int LED3pin = 8;    //blue

//photoresistors
const unsigned int p1pin = A4;  //A1
const unsigned int p2pin = A2;  //A2
const unsigned int p3pin = A0;  //A3
const unsigned int p4pin = A3;  //A4

//IR sensors
const unsigned int irpin = A5;  //A0

//servo
const unsigned int servopin = 5;  

//motors
const unsigned int m1PWMpin = 3;
//const unsigned int m2PWMpin = 5;
const unsigned int m1DIRpin = 2;
//const unsigned int m2DIRpin = 4;
const unsigned int mservpin = 6;

//sonar sensors
const unsigned int s1pin = 11;    //left front
const unsigned int s2pin = 12;    //left back
const unsigned int s3pin = 13;    //right

Servo m2;

//WfBot wfbot(m1PWMpin, m2PWMpin, m1DIRpin, m2DIRpin, s1pin, s2pin);
Sumobot sumobot(m1PWMpin, m1DIRpin, mservpin, p1pin, p2pin, p3pin, p4pin, irpin, s1pin, s2pin, s3pin, servopin, threshold);

void setup() {
  
  Serial.begin(9600); 
  
  m2.attach(6);
   
  
  //let button 1 for mode be pin 12, and button 2 for 5 sec delay be pin 13
  pinMode(modeButtonPin,INPUT); 
  pinMode(servopin, OUTPUT); 
  
  pinMode(LED1pin, OUTPUT);    //green
  pinMode(LED2pin, OUTPUT);    //red
  pinMode(LED3pin, OUTPUT);    //blue
  
  
  //WF mode
  if (digitalRead(modeButtonPin) == 0) {
    digitalWrite(LED1pin, HIGH);
    digitalWrite(LED3pin, LOW);
  ]  
  
  
  //sumo mode
  else {    
    
    digitalWrite(LED3pin, HIGH);
    digitalWrite(LED1pin, LOW);
    
    //myServo.attach(servopin);
    
    // 5 SEC DELAY
    digitalWrite(LED2pin, HIGH);
    analogWrite(m1PWMpin, 0);
    //analogWrite(m2PWMpin, 0);
    digitalWrite(m1DIRpin, LOW);
    //digitalWrite(m2DIRpin, LOW);
    analogWrite(mservpin, 50);
    delay(5000);
       
    myServo.write(0);
    delay(500);
    myServo.write(35);
    delay(500);
    myServo.write(0);
    digitalWrite(LED2pin, LOW);
  
    
    //INTERRUPTS FOR RECEIVING <VEL, ANGLE> FROM PI
    // vel and angle from raspberry pi image processing are the target speed and heading to attack another sumobot
    cli(); //stops interrupts
  
    //set timer1 interrupt at 2Hz
    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCNT1  = 0;//initialize counter value to 0
    // set compare match register for 1hz increments
    //CCR = (SysClk)/(PSC*f(desired)) - 1
    OCR1A = 624;// = (16*10^6) / (100*256) - 1 (must be <65536)
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS10 and CS12 bits for 1024 prescaler
    TCCR1B |= (1 << CS12) | (1 << CS10);  
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);
  
    sei();//allow interrupts
  }
  
}

//timer1 interrupt
ISR(TIMER1_COMPA_vect){
  analogWrite(m2PWMpin, 255);
}

void loop () {
  
  //WF mode
  if (digitalRead(modeButtonPin) == 0) {
    wfbot.forward();
  }
  
  //sumo mode
  else {
    
    sumobot.setAngGoal(getYaw());
    
    if (!threshChecked) {
      delay(1000);
      unsigned int t1, t2, t3, t4;
      t1 = analogRead(p1pin);
      t2 = analogRead(p2pin);
      t3 = analogRead(p3pin);
      t4 = analogRead(p4pin);
      unsigned int thresh = (unsigned int)((t1+t2+t3+t4)/4 - 200);
      sumobot.setThreshold(thresh);
      threshChecked = true;
    }  
    
    sumobot.checkWhiteLine();
    sumobot.setAngGoal(getYaw());
    sumobot.forward();
  }
  
}

int getYaw()
{
  if(Serial.available())
  {
  return Serial.read();
  }
  else
  {
    return 0;
  }
}


