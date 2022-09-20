#include <iostream>
#include <wiringPi.h>
#include "motor.h"
#include <cmath>
#define PUL	0
#define	DIR	2
#define STP	0.018
#define ENA	3

int rotate(double degree){
  wiringPiSetup () ;
  pinMode (PUL, OUTPUT) ;
  pinMode (DIR, OUTPUT) ;
  pinMode (ENA, OUTPUT) ;
  int direction = degree>=0?1:-1;
  degree = abs(degree);
  digitalWrite (ENA, HIGH) ;	// Enable
  if (direction==-1) 
    digitalWrite (DIR, HIGH) ;	// backward
  else if (direction==1)
    digitalWrite (DIR, LOW) ;	// forward
  delay (1) ;		// mS
  long steps = degree/STP; 
  std::cout << direction << " " <<steps << std::endl;
  for (long i=0;i<steps;i++)
  {
    digitalWrite (PUL, HIGH) ;	// On
    delay (0.1) ;		// mS
    digitalWrite (PUL, LOW) ;	// Off
    delay (0.1) ;
  }
  digitalWrite (DIR, LOW) ;	// forward
  digitalWrite (ENA, LOW) ;	// Disable
  return 0;
}
int main(int argc,char** argv){
  double dg = atof(argv[1]);
  std::cout << dg << std::endl;
  rotate(dg);
}
