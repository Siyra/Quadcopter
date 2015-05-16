//
// FILTER.CPP
//
// This class implements the a filter, now it will only measure position
// based on accelerometer data and GPS positioning.
//

#include "filter.h"

filter::filter(float h0) {
	mState.zeros();
	mMeasure.zeros();
	mTib.zeros();
	mTibTranspose.zeros();
	mOmega.zeros();
	
	mH0 = h0;
}

//
// This function gets the measurements from the sensor data and stores them in mMeasurement
//
void filter::calculateMeasuredPos(RTIMU_DATA& data) {
	mMeasure(7) = mH0 - RTMath::convertPressureToHeight(data.pressure);
}

//
// This function is called whenever there is a new iteration, to calculate the new state
//
void filter::newData(RTIMU_DATA& data, float dt) {
	mAccel << data.accel.x() << data.accel.y() << data.accel.z();
	mGyro << data.gyro.x() << data.gyro.y() << data.gyro.z();
	mDt = dt;
	
	float x,y,z,s;
	
	s = data.fusionQPose.scalar();
	x = data.fusionQPose.x();
	y = data.fusionQPose.y();
	z = data.fusionQPose.z();
	
	mTib(0,0) = 1-2*(y*y+z*z);
	mTib(0,1) = 2*(x*y+s*z);
	mTib(0,2) = 2*(x*z-s*y);
	
	mTib(1,0) = 2*(x*y-s*z);
	mTib(1,1) = 1-2*(x*x+z*z);
	mTib(1,2) = 2*(y*z+s*x);
	
	mTib(2,0) = 2*(x*z+s*y);
	mTib(2,1) = 2*(y*z-s*x);
	mTib(2,2) = 1-2*(x*x+y*y);
	
	mTibTranspose = mTib.t();
	
	mOmega.zeros();
	
	mOmega(0,1) = mGyro(3);
	mOmega(0,2) = -mGyro(2);
	
	mOmega(1,0) = -mGyro(3);
	mOmega(1,2) = mGyro(1);
	
	mOmega(2,0) = mGyro(2);
	mOmega(2,1) = -mGyro(1);
}

//
// This function predicts the next state based on IMU data
//
void filter::predict() {
	// This function will compute the position and velocity based on IMU sensor readings
	
	vec gravity(3);
	gravity << 0 << 0 << GRAV;
	
	// Calculate acceleration first in body
	vec temp = mAccel - (mOmega * mInVel) - (mTib * gravity);
	
	mFk << 1.0 << 0 << 0 << mDt << 0 << 0 << 0.5*mDt*mDt << 0 << 0 << endr
		<< 0 << 1.0 << 0 << 0 << mDt << 0 << 0 << 0.5*mDt*mDt << 0 << endr
		<< 0 << 0 << 1.0 << 0 << 0 << mDt << 0 << 0 << 0.5*mDt*mDt << endr
		<< 0 << 0 << 0 << 1.0 << 0 << 0 << mDt << 0 << 0 << endr
		<< 0 << 0 << 0 << 0 << 1.0 << 0 << 0 << mDt << 0 << endr
		<< 0 << 0 << 0 << 0 << 0 << 1.0 << 0 << 0 << mDt << endr
		<< 0 << 0 << 0 << 0 << 0 << 0 << 1.0 << 0 << 0 << endr
		<< 0 << 0 << 0 << 0 << 0 << 0 << 0 << 1.0 << 0 << endr
		<< 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 1.0;
	
	
	
	
}

// This function corrects the estimated state with data from GPS and barometer
void filter::update() {

}