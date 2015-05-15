//
// PID.CPP
//
// This file contains the PID control for the UAV.
//

#include "pid.h"

//
// Default Constructor
//
PID::PID() {
	mKp = 0;
	mKi = 0;
	mKd = 0;
	
	mErr = 0;
	mSumErr = 0;
	mDtErr = 0;
	mLastInput = 0;
	mOutputMin = -100;
	// Temporary max so the quadcopter does not kill itself, probably
	// using a max value of 100 later
	mOutputMax = 100;
}

//
// Function to update the output based on setpoint and sensor inputs
//
float PID::updatePID(float setpoint, float input, float dt) {
	// Calculate the error between setpoint and input
	mErr = setpoint - input;
	
	// Calculate the integral part
	mSumErr += mErr * mKi * dt;
	
	if(mSumErr > mOutputMax)
		mSumErr = mOutputMax;
	else if(mSumErr < mOutputMin)
		mSumErr = mOutputMin;
	
	// Calculate the derivative part
	mDtErr = -(mKd / dt) * (input - mLastInput);
	
	// Sum all parts together to get the output
	mOutput = mKp * mErr + mSumErr + mDtErr;
	
	// Make sure the calculated value is not out of bounds, mSumErr is also
	// reset to make sure the controller doesn't go crazy
	if(mOutput > mOutputMax) {
		mOutput = mOutputMax;
	} else if(mOutput < mOutputMin) {
		mOutput = mOutputMin;
	}
    
    mLastInput = input;
	
	//printf("Error: %6.4f, Output: %6.4f\n", mErr, mOutput);
	
	// Scale the output so it can be used for the ESC (ESC wants it between 0 and 1)
	//mOutput = mOutput / 100;
	mLastInput = input;
	
	return mOutput;
}

//
// Function to set Kp, Ki and Kd
//
void PID::setK(float Kp, float Ki, float Kd) {
	mKp = Kp;
	mKi = Ki;
	mKd = Kd;
	printf("Kp = %4.2f, Kd = %4.2f, Ki = %4.2f\n", mKp, mKd, mKi);
}

//
// Function to set the boundaries of the output
//
void PID::setBounds(float min, float max) {
	mOutputMin = min;
	mOutputMax = max;
	
	if(mOutput > mOutputMax) {
		mSumErr = 0;
		mOutput = mOutputMax;
	} else if(mOutput < mOutputMin) {
		mSumErr = 0;
		mOutput = mOutputMin;
	}
}

void PID::reset() {
	mErr = 0;
	mSumErr = 0;
	mDtErr = 0;
	mLastInput = 0;
}
