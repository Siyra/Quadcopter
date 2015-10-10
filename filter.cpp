//
// FILTER.CPP
//
// This class implements the Kalman filter, now it will only measure position
// based on accelerometer data and GPS positioning.
//

#include "filter.h"

filterz::filterz() {
	mBufferSize = 50;
	mIndexBuffer = 0;
	mSensorDataSum = RTVector3(0,0,0);
}

RTVector3 filterz::lowPass(RTVector3 sensorData) {
	// mSensorDataSum -= mBuffer[mIndexBuffer];
	// mBuffer[mIndexBuffer] = sensorData;
	
	// mSensorDataSum += sensorData;
	// mIndexBuffer++;
	
	// if(mIndexBuffer >= mBufferSize)
		// mIndexBuffer = 0;
	
	// if(mIndexBuffer < mBufferSize) {
		// mBuffer[mIndexBuffer++] = sensorData;
		// mSensorDataSum += sensorData;
	// } else {
		// RTVector3 oldest = mBuffer[mIndexBuffer++ % mBufferSize];
		// mSensorDataSum += sensorData;
		// mSensorDataSum -= oldest;
		// mBuffer[mIndexBuffer % mBufferSize] = sensorData;
	// }
	
	//RTVector3 temp(mSensorDataSum.x() / std::min(mBufferSize,mIndexBuffer),mSensorDataSum.y() / std::min(mBufferSize,mIndexBuffer), mSensorDataSum.z() / std::min(mBufferSize,mIndexBuffer));
	// RTVector3 temp(mSensorDataSum.x(),mSensorDataSum.y(), mSensorDataSum.z());
	
	mSensorDataSum = 0.01 * sensorData + 0.99 * mSensorDataSum;
	
	return mSensorDataSum;
}

void filterz::setSize(int size) {
	mBufferSize = size;
}

/*
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
	
	predict();
	
	update();
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
	
	mState(6) = temp(0);
	mState(7) = temp(1);
	mState(8) = temp(2);
	
	// State vector
	// [ pos_x pos_y pos_z  vel_x vel_y vel_z  acc_x acc_y acc_z ]
	mFk << 1.0 << 0 << 0 << mDt << 0 << 0 << 0.5*mDt*mDt << 0 << 0 << endr
		<< 0 << 1.0 << 0 << 0 << mDt << 0 << 0 << 0.5*mDt*mDt << 0 << endr
		<< 0 << 0 << 1.0 << 0 << 0 << mDt << 0 << 0 << 0.5*mDt*mDt << endr
		<< 0 << 0 << 0 << 1.0 << 0 << 0 << mDt << 0 << 0 << endr
		<< 0 << 0 << 0 << 0 << 1.0 << 0 << 0 << mDt << 0 << endr
		<< 0 << 0 << 0 << 0 << 0 << 1.0 << 0 << 0 << mDt << endr
		<< 0 << 0 << 0 << 0 << 0 << 0 << 1.0 << 0 << 0 << endr
		<< 0 << 0 << 0 << 0 << 0 << 0 << 0 << 1.0 << 0 << endr
		<< 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 1.0;
	
	// Measurement state vector:
	// [ GPS_x GPS_y GPS_z  baro_z  sonar_z ]
	mHk << 1.0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << endr
		<< 0 << 1.0 << 0 << 0 << 0 << 0 << 0 << 0 << 0 << endr
		<< 0 << 0 << 1.0 << 0 << 0 << 0 << 0 << 0 << 0 << endr
		<< 0 << 0 << 1.0 << 0 << 0 << 0 << 0 << 0 << 0 << endr
		<< 0 << 0 << 1.0 << 0 << 0 << 0 << 0 << 0 << 0;

	// Project the state ahead
	mState = mFk * mState;
	
	mPkk_1 = mFk * mPkk * mFk.t() + mQ;
}

// This function corrects the estimated state with data from GPS and barometer
void filter::update() {
	mat Sk, SkInverse;
	
	Sk = mHk * mPkk_1 * mHk.t() + mR;
	SkInverse = inv(Sk);
	
	mKk = mPkk_1 * mHk.t() * SkInverse;
	
	mState = mState + mKk * (mMeasure - mHk * mState);
	
	mat ident = eye<mat>(9,9);
	mPkk = (ident - mKk*mHk) * mPkk_1;
}*/
