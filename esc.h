//
// ESC.H
//
// This file contains the ESC control for the UAV. Since the UAV uses
// brushless motors, they need an ESC to control it. The ESC is controlled
// just like a regular servo: by sending a PWM signal to it. For this
// the program pi-blaster is used. By sending a string to /dev/pi-blaster
// like "PIN:VALUE" where PIN is the pin number and VALUE is a value between
// 0 and (including) 1. 
//

#ifndef esc
#define esc

#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <stdio.h>
#include <math.h>

#define YAW 0
#define PITCH 1
#define ROLL 2

class ESC {
	private:
	int mEscID[4];
	FILE* PWMOutput;
	
	public:
	ESC();
	
	void init();
	void close();
	void update(float throttle, float PIDOutput[3]);
	void flush();
	
	float EscVals[4];
	
};

#endif