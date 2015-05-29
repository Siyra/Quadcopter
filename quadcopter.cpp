//
// QUADCOPTER.CPP
//
// This is the main source file of the project and contains the main loop of the program.
//
#include "quadcopter.h"
#define TEST
void stack_prefault(void) {
	unsigned char dummy[MAX_SAFE_STACK];

	memset(dummy, 0, MAX_SAFE_STACK);
	return;
}

int main(int argc, char* argv[])
{
	struct timespec t, current_t, last_t;
	struct sched_param param;
	int interval = 10000000; // 10ms = 100 Hz
	//int interval = 50000000; // 50ms = 20 Hz
	float dt = 0.01;
	// Initial starting height
	float h0 = 44000;
	
	bool started = false;
	bool first = true;

	// Declare this as a real time task
	param.sched_priority = MY_PRIORITY;
	if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
		perror("sched_setscheduler failed");
		exit(-1);
	}

	// Lock memory
	if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
		perror("mlockall failed");
		exit(-2);
	}

	// Pre-fault stack
	stack_prefault();
		
	//
	// Initialize the comms
	//
	printf("Initializing Comms..\n");
	Comms MAVLink;
	MAVLink.init("192.168.178.13");

	//
	// Initialize the sensors
	//
	RTIMUSettings imusettings = RTIMUSettings("RTIMULib");

	RTIMU *imu = RTIMU::createIMU(&imusettings);
	RTPressure *pressure = RTPressure::createPressure(&imusettings);

#ifndef TEST		
	// Check if the IMU is detected
	if((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL)) {
		printf("No IMU found, exiting..\n");
		exit(1);
	}

	// Initialize the IMU
	imu->IMUInit();
	
	// Enable the components
	imu->setSlerpPower(0.02);
	imu->setGyroEnable(true);
	imu->setAccelEnable(true);
	imu->setCompassEnable(true);

	//  Set up pressure sensor
	if (pressure != NULL)
		pressure->pressureInit();

	// Read sensor data
	imu->IMURead();
	RTIMU_DATA imuData = imu->getIMUData();

	if (pressure != NULL)
		pressure->pressureRead(imuData);

	printf("Calibrating barometer for height measurement..\n");
	while(h0 > 100) {
		if (pressure != NULL)
			pressure->pressureRead(imuData);

		h0 = RTMath::convertPressureToHeight(imuData.pressure);
		usleep(500000);
	}
#endif /* TEST */

	//filter positionFilter(h0);
	
	//
	// Initialize ESCs
	//
	printf("Initializing ESCs..\n");
	ESC motors;

	motors.init();
	
	// Initialize sensor filters
	filter gyroFilter;

	//
	// Initialize PID Controllers
	//
	printf("Initializing PID Controllers..\n");
	float PIDOutput[3];
	float setpoint[3];
	float attitude[3];
	float gyro[3];
	float throttle = 0;
	
	RTVector3 gyros(1,2,3);
		RTVector3 gyrotemp = gyroFilter.lowPass(gyros);
		gyro[YAW] = gyrotemp.z();
		gyro[PITCH] = gyrotemp.y();
		gyro[ROLL] = gyrotemp.x();

	setpoint[0] = 0;
	setpoint[1] = 0;
	setpoint[2] = 0;

	PID YPRStab[3];
	PID YPRRate[3];

	// The yaw is only controlled using the rate controller, pitch
	// and roll are also controlled using the angle.
	/*YPRStab[PITCH].setK(2,0.035,0.04);
	YPRStab[ROLL].setK(2,0.035,0.04);

	YPRRate[YAW].setK(2,0.1,0.1);
	YPRRate[PITCH].setK(2,0.1,0.1);
	YPRRate[ROLL].setK(2,0.1,0.1);
	*/
	YPRStab[PITCH].setK(0,0,0);
	YPRStab[ROLL].setK(0,0,0);

	YPRRate[YAW].setK(0,0,0);
	YPRRate[PITCH].setK(0,0,0);
	YPRRate[ROLL].setK(0,0,0);
	
	clock_gettime(CLOCK_MONOTONIC, &t);

	// Start after one second
	t.tv_sec++;

	int lastHeartbeat = 0;
		
	printf("Starting process loop with dt: %f\n",dt);
        while(1) {
		// Wait until next shot
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

#ifndef TEST	
		// Read sensor data
		imu->IMURead();
		imuData = imu->getIMUData();

		if (pressure != NULL)
			pressure->pressureRead(imuData);
			
		// Print the IMU data
		//printf("Fusion pose: %s\n", RTMath::displayDegrees("", imuData.fusionPose));
		//printf("Pressure: %4.1f, height above sea level: %4.1f, temperature: %4.1f\n",
		//               imuData.pressure, RTMath::convertPressureToHeight(imuData.pressure), imuData.temperature);

		// This is the attitude angle of the UAV, so yaw, pitch and roll (in degrees)
		attitude[YAW] = imuData.fusionPose.z();
		attitude[PITCH] = imuData.fusionPose.y();
		attitude[ROLL] = imuData.fusionPose.x();

		// These are the angular velocities of the UAV (in rad/s)
		RTVector3 gyrotemp = gyroFilter.lowPass(imuData.gyro);
		gyro[YAW] = gyrotemp.z();
		gyro[PITCH] = gyrotemp.y();
		gyro[ROLL] = gyrotemp.x();

#else
		// This is the attitude angle of the UAV, so yaw, pitch and roll
		attitude[YAW] = 0;
		attitude[PITCH] = 0;
		attitude[ROLL] = 0;

		// These are the angular velocities of the UAV
		RTVector3 gyros(1,2,3);
		RTVector3 gyrotemp = gyroFilter.lowPass(gyros);
		gyro[YAW] = gyrotemp.z();
		gyro[PITCH] = gyrotemp.y();
		gyro[ROLL] = gyrotemp.x();
#endif /* TEST */

		// Here the unscented kalman filter is invoked with the current measurement data
		

		if(started) {
			if(first) {
				first = false;
				clock_gettime(CLOCK_MONOTONIC, &current_t);
				current_t.tv_nsec -= dt*1000000000;
			}
			
			// Calculate the exact delta time
			last_t = current_t;
			
			clock_gettime(CLOCK_MONOTONIC, &current_t);
			dt = ((static_cast <int64_t>(current_t.tv_sec) * 1000000000 +
				static_cast <int64_t>(current_t.tv_nsec)) -
				(static_cast <int64_t>(last_t.tv_sec) * 1000000000 +
				static_cast <int64_t>(last_t.tv_nsec))) / 1000000000.0;
			
			// First the stability control for pitch and roll
			for(int i=1; i<3; i++) {
				PIDOutput[i] = YPRStab[i].updatePID(setpoint[i], attitude[i], dt);
			}
			PIDOutput[0] = setpoint[0];
	
			// Now the rate control for yaw, pitch and roll
			for(int i=0; i<3; i++) {
				PIDOutput[i] = YPRRate[i].updatePID(PIDOutput[i], gyro[i], dt);
			}
			//printf("PIDOutput: %6.4f, %6.4f, %6.4f, %6.4f\n", throttle, PIDOutput[0], PIDOutput[1], PIDOutput[2]);
			// Output to motors
			motors.update(throttle, PIDOutput);
		}
		
		// Send a heartbeat
		if((t.tv_sec - lastHeartbeat) > 1) {
			//printf("Sending heartbeat\n");
			MAVLink.sendHeartbeat();
	
			lastHeartbeat = t.tv_sec;
		}

		// The switch that receives controls from the GCS
		switch(MAVLink.receiveData()) {
			case START:
				started = true;
				break;
	
			case STOP:
				started = false;
				first = true;
				motors.close();
				break;
		
			case SETPOINT:
				MAVLink.parseSetpoint(throttle, setpoint[0], setpoint[1], setpoint[2]);
				printf("throttle = %4.2f, yaw = %4.2f, pitch = %4.2f, roll = %4.2f\n", throttle, setpoint[0], setpoint[1],setpoint[2]);
		
				for(int i=1; i<3; i++) {
					YPRStab[i].reset();
				}
		
				for(int i=0; i<3; i++) {
					YPRRate[i].reset();
				}
				break;
	
			// Set the PID constants for the yaw control (rate only)
			case SETPID_YAW:
				float tempkp = -1;
				float tempkd = -1;
				float tempki = -1;
				MAVLink.parsePID(tempkp,tempkd,tempki);
				YPRRate[YAW].setK(tempkp,tempkd,tempki);
		
				printf("Kp = %4.2f, Kd = %4.2f, Ki = %4.2f\n", tempkp, tempkd,tempki);
				break;
	
			// Set the PID constants for the pitch and roll control (angle only)
			case SETPID_PR_STAB:
				float tempkp = -1;
				float tempkd = -1;
				float tempki = -1;
				MAVLink.parsePID(tempkp,tempkd,tempki);
				YPRStab[PITCH].setK(tempkp,tempkd,tempki);
				YPRStab[ROLL].setK(tempkp,tempkd,tempki);
		
				printf("Kp = %4.2f, Kd = %4.2f, Ki = %4.2f\n", tempkp, tempkd,tempki);
				break;
	
			// Set the PID constants for the pitch and roll control (rate only)
			case SETPID_PR_RATE:
				float tempkp = -1;
				float tempkd = -1;
				float tempki = -1;
				MAVLink.parsePID(tempkp,tempkd,tempki);
				YPRRate[PITCH].setK(tempkp,tempkd,tempki);
				YPRRate[ROLL].setK(tempkp,tempkd,tempki);
		
				printf("Kp = %4.2f, Kd = %4.2f, Ki = %4.2f\n", tempkp, tempkd,tempki);
				break;
	
			default:
				break;
		}

		// Clean the buffer so the message does not get repeated
		MAVLink.purge();
		
		MAVLink.sendStatus(attitude, gyro);

		// Calculate next shot
		t.tv_nsec += interval;

		while (t.tv_nsec >= NSEC_PER_SEC) {
			   t.tv_nsec -= NSEC_PER_SEC;
				t.tv_sec++;
		}
	}
}   
