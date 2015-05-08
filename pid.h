//
// PID.H
//
// This file contains the PID control for the UAV.
//

#ifndef pid
#define pid

#include <iostream>
#include <stdlib.h>
#include <fstream>

class PID {
	private:
	float mKp;
	float mKi;
	float mKd;
	
	float mErr;
	float mSumErr;
	float mDdtErr;
	float mLastInput;
	float mOutputMin;
	float mOutputMax;
	float mOutput;
	
	public:
	PID();
	float updatePID(float setpoint, float input, float dt);
	void setK(float Kp, float Ki, float Kd);
	void setBounds(float min, float max);
	
	float setpoint;
};

#endif