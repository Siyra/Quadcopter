//
// UKF.H
//
// This class implements the Unscented Kalman Filter, now it will only measure position
// based on accelerometer data and GPS positioning.
//

#ifndef FILTER_H
#define FILTER_H

#include <math.h>
#include <iostream>
#include "RTIMULib/RTMath.h"
#include <algorithm>
//#include <armadillo>

#define GRAV 9.817
#define BUFFER_SIZE 2000
/*
using namespace arma;

typedef struct
{
    vec position;
	vec velocity;
	vec accel;
} STATE;

typedef struct
{
    vec GpsPosition;
	vec GpsVelocity;
	float BaroAltitude;
} MEASUREMENT;
*/
class filterz {
	public:
	filterz();
	
	//void newData(RTIMU_DATA& data, float dt);
	
	RTVector3 lowPass(RTVector3 sensorData);
	void setSize(int size);
	
	protected:
	// Variables for sensor data smoothing
	int mIndexBuffer;
	int mBufferSize;
	RTVector3 mBuffer[BUFFER_SIZE];
	RTVector3 mSensorDataSum;
	
	/*
	void calculateMeasuredPos(RTIMU_DATA& data);
	
	void predict();
	void update();
	
	// The angular accelerations
	vec mAnAccel;
	// The angular velocities
	vec mAnVel;
	// The body accelerations
	vec mInAccel;
	// The body velocities (calculated from the previous step)
	vec mInVel;
	
	// Sensor data
	vec mGyro;
	vec mAccel;
	
	// Transformation matrix from NED to body frame, and from body frame to NED (transpose)
	mat mTib, mTibTranspose;
	
	// Matrix to compensate for centripetal acceleration
	mat mOmega;
	
	// The current state of the system
	vec mState;
	// The current measurements of the GPS and barometer and altitudemeter
	vec mMeasure;
	
	// delta time
	float mDt;
	float mH0;
	
	mat mKk;                                       // the Kalman gain matrix
	mat mPkk_1;                                    // the predicted estimated covariance matrix
	mat mPkk;                                      // the updated estimated covariance matrix
	mat mPDot;                                     // the derivative of the covariance matrix
	mat mQ;                                        // process noise covariance
	mat mFk;                                       // the state transition matrix
	mat mHk;
	mat mRk;                                       // the measurement noise covariance
	*/
};

#endif /* FILTER_H */
