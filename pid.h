//
// PID.H
//
// This file contains the PID control for the UAV.
//

#ifndef pidcontrol_h
#define pidcontrol_h

#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <string.h>
#include <sstream>

class PID {
	private:
	float mKp;
	float mKi;
	float mKd;
	
	float mErr;
	float mSumErr;
	float mDtErr;
	float mLastInput;
	float mOutputMin;
	float mOutputMax;
	float mOutput;
	
	public:
	PID();
	float updatePID(float setpoint, float input, float dt);
	void setK(float Kp, float Ki, float Kd);
	void setBounds(float min, float max);
	void reset();
	
	float setpoint;
};

#endif