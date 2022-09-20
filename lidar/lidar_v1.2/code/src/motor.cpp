#include <iostream>
#include <wiringPi.h>
#include "motor.h"
#include <cmath>
#include "MPU6050.h"
#define PUL	0
#define	DIR	2
#define	ENA	3
#define STP	0.018
MPU6050 acc_device(0x68);
int offset = 3;
int rotate(double degree){
  wiringPiSetup () ;
  pinMode (PUL, OUTPUT) ;
  pinMode (DIR, OUTPUT) ;
  int direction = degree>=0?1:-1;
  degree = fabs(degree);
  if (direction==-1) 
    digitalWrite (DIR, HIGH) ;	// backward
  else if (direction==1)
    digitalWrite (DIR, LOW) ;	// forward
  long steps = degree/STP; 
  for (long i=0;i<steps;i++)
  {
    digitalWrite (PUL, HIGH) ;	// On
    delay (1) ;		// mS
    digitalWrite (PUL, LOW) ;	// Off
    delay (1) ;
  }
  return 0;
}
int enable(){
  wiringPiSetup () ;
  pinMode (ENA, OUTPUT) ;
  digitalWrite (ENA, HIGH) ;
  return 0;
}
int disable(){
  wiringPiSetup () ;
  pinMode (ENA, OUTPUT) ;
  digitalWrite (ENA, LOW) ;
  return 0;
}
int calib(){
  float ax,ay,az; 
  sleep(2);
  acc_device.getAccel(&ax,&ay,&az);
  float degree = ay/0.0174524 + offset;
  rotate(-1 * degree);
  return 0;
}
