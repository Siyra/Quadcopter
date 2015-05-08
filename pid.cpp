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
	mDdtErr = 0;
	mLastInput = 0;
	mOutputMin = 0;
	mOutputMax = 20;
}

//
// Constructor with initial K values
//
PID::PID(float Kp, float Ki, float Kd) {
	mKp = Kp;
	mKi = Ki;
	mKd = Kd;
	
	mErr = 0;
	mSumErr = 0;
	mDdtErr = 0;
	mLastInput = 0;
	mOutputMin = 0;
	mOutputMax = 20;
}

//
// Function to update the output based on setpoint and sensor inputs
//
float PID::UpdatePID(float setpoint, float input, float dt) {
	mErr = setpoint - input;
	
	mSumErr = mErr * mKi * dt;
	
	mDdtErr = -mKd / dt * (input - mLastInput);
	
	mOutput = mKp * mErr + mSumErr + mDdtErr;
	
	if(mOutput > mOutputMax) {
		mSumErr = 0;
		mOutput = mOutputMax;
	} else if(mOutput < mOutputMin) {
		mSumErr = 0;
		mOutput = mOutputMin;
	}
	
	return mOutput;
}

//
// Function to set Kp, Ki and Kd
//
void PID::setK(float Kp, float Ki, float Kd) {
	mKp = Kp;
	mKi = Ki;
	mKd = Kd;
}

//
// Function to set the boundaries of the output
//
void PID::setBounds(float min, float max) {
	mOutputMin = min;
	mOutputMax = max;
}