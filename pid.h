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
	PID(float,float,float);
	float UpdatePID(float setpoint, float input, float dt);
	void setK(float, float, float);
	
	float setpoint;
};

#endif