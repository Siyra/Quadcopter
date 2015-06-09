//
// ESC.CPP
//
// This file contains the ESC control for the UAV. Since the UAV uses
// brushless motors, they need an ESC to control it. The ESC is controlled
// just like a regular servo: by sending a PWM signal to it. For this
// the program pi-blaster is used. By sending a string to /dev/pi-blaster
// like "PIN:VALUE" where PIN is the pin number and VALUE is a value between
// 0 and (including) 1. 
//

#include "esc.h"

extern FILE *file;

//
// Default constructor
//
ESC::ESC() {
    // Front left
	mEscID[0] = 24;
    // Front right
	mEscID[1] = 23;
    // Back right
	mEscID[2] = 22;
    // Back left
	mEscID[3] = 25;
	
	PWMOutput = NULL;
}

//
// Initialize the ESC by opening a stream to /dev/pi-blaster
//
void ESC::init() {
	// Open a connection to the device
	PWMOutput = fopen("/dev/pigpio", "w");
	
	if(PWMOutput == NULL) {
		printf("Could not open /dev/pigpio for writing.\n");
		exit(2);
	}
	
	// Initialize all ESC values to 0, since the UAV should start at rest
	for(int i=0; i<4; i++) {
		EscVals[i] = 1000;
	}
	
	flush();
}

//
// Update the ESC values with trottle, pitch, roll and yaw values
//
void ESC::update(float throttle, float PIDOutput[3]) {
	throttle = throttle * 10;

	PIDOutput[YAW] = 0;
	// Set the correct values for each motor, this is assuming the UAV
	// flies in a + orientation, this can be altered so the UAV flies in a X orientation.
	EscVals[0] = 1000 + (throttle + PIDOutput[ROLL] + PIDOutput[PITCH] - PIDOutput[YAW]);
	EscVals[1] = 1000 + (throttle - PIDOutput[ROLL] + PIDOutput[PITCH] + PIDOutput[YAW]);
	EscVals[2] = 1000 + (throttle - PIDOutput[ROLL] - PIDOutput[PITCH] - PIDOutput[YAW]);
	EscVals[3] = 1000 + (throttle + PIDOutput[ROLL] - PIDOutput[PITCH] + PIDOutput[YAW]);
	
	for(int i=0; i<4; i++) {
		if(EscVals[i] > 2000)
			EscVals[i] = 2000;
		else if(EscVals[i] < 1000)
			EscVals[i] = 1000;
	}
	
	//printf("%6.4f, %6.4f, %6.4f, %6.4f, %6.4f, %6.4f, %6.4f\n", PIDOutput[0], PIDOutput[1], PIDOutput[2], EscVals[0], EscVals[1], EscVals[2], EscVals[3]);
	flush();
}

//
// Flush the PWM values to the output pins
//
void ESC::flush() {
	// Write the ESC values to the device
	for(int i=0; i<4; i++) {
        //printf("s %d %g", mEscID[i], EscVals[i]);
		fprintf(PWMOutput, "s %d %g\n", mEscID[i], EscVals[i]);
		fflush(PWMOutput);
	}
}

//
// Close the connection to the pi-blaster
//
void ESC::close() {
	// Set all ESC values to 0
	for(int i=0; i<4; i++) {
		EscVals[i] = 1000;
	}
	
	// Write the values to the device
	flush();
	
	if(PWMOutput == NULL) {
		printf("/dev/pigpio already closed");
		return;
	}
	
	// Close the connection to the device
	fclose(PWMOutput);
	PWMOutput = NULL;
}