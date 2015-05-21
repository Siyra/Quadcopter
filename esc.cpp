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

//
// Default constructor
//
ESC::ESC() {
	mEscID[0] = 0;
	mEscID[1] = 1;
	mEscID[2] = 2;
	mEscID[3] = 3;
	
	PWMOutput = NULL;
}

//
// Initialize the ESC by opening a stream to /dev/pi-blaster
//
void ESC::init() {
	// Open a connection to the device
	PWMOutput = fopen("/dev/pi-blaster", "w");
	
	if(PWMOutput == NULL) {
		printf("Could not open /dev/pi-blaster for writing.\n");
		exit(2);
	}
	
	// Initialize all ESC values to 0, since the UAV should start at rest
	for(int i=0; i<4; i++) {
		EscVals[i] = 0;
	}
	
	flush();
}

//
// Update the ESC values with trottle, pitch, roll and yaw values
//
void ESC::update(float throttle, float PIDOutput[3]) {
	// Set the correct values for each motor, this is assuming the UAV
	// flies in a + orientation, this can be altered so the UAV flies in a X orientation.
	EscVals[0] = (throttle + PIDOutput[PITCH] - PIDOutput[YAW]);
	EscVals[1] = (throttle + PIDOutput[ROLL] + PIDOutput[YAW]);
	EscVals[2] = (throttle - PIDOutput[PITCH] - PIDOutput[YAW]);
	EscVals[3] = (throttle - PIDOutput[ROLL] + PIDOutput[YAW]);
	
	for(int i=0; i<4; i++) {
		if(EscVals[i] > 1)
			EscVals[i] = 1;
		else if(EscVals[i] < 0)
			EscVals[i] = 0;
	}
	
	//printf("%6.4f, %6.4f, %6.4f, %6.4f\n", EscVals[0], EscVals[1], EscVals[2], EscVals[3]);
	//flush();
}

//
// Flush the PWM values to the output pins
//
void ESC::flush() {
	// Write the ESC values to the device
	for(int i=0; i<4; i++) {
		fprintf(PWMOutput, "%d=%f\n", mEscID[i], EscVals[i]);
		fflush(PWMOutput);
	}
}

//
// Close the connection to the pi-blaster
//
void ESC::close() {
	// Set all ESC values to 0
	for(int i=0; i<4; i++) {
		EscVals[i] = 0;
	}
	
	// Write the values to the device
	flush();
	
	if(PWMOutput == NULL) {
		printf("/dev/pi-blaster already closed");
		return;
	}
	
	// Close the connection to the device
	fclose(PWMOutput);
	PWMOutput = NULL;
}