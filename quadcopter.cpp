//
// QUADCOPTER.CPP
//
// This is the main source file of the project and contains the main loop of the program.
//
#include "quadcopter.h"
void stack_prefault(void) {
	unsigned char dummy[MAX_SAFE_STACK];

	memset(dummy, 0, MAX_SAFE_STACK);
	return;
}
FILE *file;
int main(int argc, char* argv[])
{
	struct timespec t, current_t, last_t;
	struct sched_param param;
	//int interval =  2500000; // 2.5ms = 400 Hz
	//int interval = 5000000; // 5ms = 200 Hz
	int interval = 10000000; // 20ms = 100 Hz
	// int interval = 20000000; // 20ms = 50 Hz
	// int interval = 50000000; // 50ms = 20 Hz
	float dt = (float)interval / 1000000000;
	float cumTime = 0;
	// Initial starting height
	float h0 = 44000;
	
	bool started = true;
	bool first = true;
	file = fopen("recordedData.dat","w");
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
	
	//
	// Initialize ESCs
	//
	printf("Initializing ESCs..\n");
	ESC motors;

	motors.init();
	
	// Initialize sensor filters
	filterz gyroFilter, attiFilter;
	gyroFilter.setSize(100);
	attiFilter.setSize(40);
	
	//
	// Initialize PS3 Controller
	//
	Joystick joystick("/dev/input/js0");
	
	//
	// Initialize PID Controllers
	//
	printf("Initializing PID Controllers..\n");
	float PIDOutput[3];
	float PIDOutput1[3];
	float setpoint[3], bias[3];
	float attitude[3];
	float gyro[3];
	float throttle = 0;
	
	imu->IMURead();
	imuData = imu->getIMUData();

	bias[YAW] = imuData.fusionPose.z();
	bias[PITCH] = imuData.fusionPose.y();
	bias[ROLL] = imuData.fusionPose.x();
	
	setpoint[YAW] = 0;
	setpoint[PITCH] = bias[PITCH];
	setpoint[ROLL] = bias[ROLL];

	PID YPRStab[3];
	PID YPRRate[3];

	// The yaw is only controlled using the rate controller, pitch
	// and roll are also controlled using the angle.
	YPRStab[PITCH].setK(1.8,0,0.01);
	YPRStab[ROLL].setK(1.8,0,0.01);

	//YPRRate[YAW].setK(3,0.1,0.1);
	// YPRRate[PITCH].setK(100,0,0.02);
	// YPRRate[ROLL].setK(100,0,0.02);
	YPRRate[PITCH].setK(8,0,1);
	YPRRate[ROLL].setK(8,0,1);
	// YPRRate[YAW].setK(14,0,0);
	// YPRRate[PITCH].setK(30,0,0);
	// YPRRate[ROLL].setK(30,0,0);
	// YPRRate[YAW].setK(30,0,0);
	
	clock_gettime(CLOCK_MONOTONIC, &t);

	// Start after one second
	t.tv_sec++;

	int lastHeartbeat = 0;
	int lastDataSent = 0;
		
	printf("Starting process loop with dt: %f\n",dt);
	while(1) {
		// Wait until next shot
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

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
		attitude[YAW] = imuData.fusionPose.z() - bias[YAW];
		attitude[PITCH] = imuData.fusionPose.y() - bias[PITCH];
		attitude[ROLL] = imuData.fusionPose.x() - bias[ROLL];

		// These are the angular velocities of the UAV (in rad/s)
		RTVector3 gyrotemp = gyroFilter.lowPass(10*imuData.gyro);
		//RTVector3 attitemp = attiFilter.lowPass(imuData.fusionPose);
		
		//attitude[YAW] = attitemp.z();
		//attitude[PITCH] = attitemp.y() - bias[PITCH];
		//attitude[ROLL] = attitemp.x() - bias[ROLL];
		
		// gyro[YAW] = imuData.gyro.z();
		// gyro[PITCH] = imuData.gyro.y();
		// gyro[ROLL] =  imuData.gyro.x();
		
		gyro[YAW] = gyrotemp.z();
		gyro[PITCH] = gyrotemp.y();
		gyro[ROLL] =  gyrotemp.x();

		// Here the joystick data is processed
		JoystickEvent event;
		if (joystick.sample(&event)) {
			if (event.isAxis() && event.number == 13) {
				throttle = (event.value + 32767) / 100;
				//printf("Set throttle to %g\n", throttle);
			} 
			if (event.isAxis() && event.number == 1) {
				setpoint[PITCH] = (float)event.value / 50767.0;
				//printf("Set pitch to %6.4f\n", setpoint[PITCH]);
			} 
			if (event.isAxis() && event.number == 0) {
				setpoint[ROLL] = (float)event.value / 50767.0;
				//printf("Set roll to %6.4f\n", setpoint[ROLL]);
			}
			if (event.isAxis() && event.number == 2) {
				setpoint[YAW] = (float)event.value / 15767.0;
				//printf("Set yaw to %6.4f\n", setpoint[YAW]);
			}
		}
			

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
			
			
			//printf("DT = %6.4f\n", dt);
			
			// First the stability control for pitch and roll
			for(int i=1; i<3; i++) {
				PIDOutput1[i] = 0;//YPRStab[i].updatePID(setpoint[i], attitude[i], dt);
			}
			PIDOutput1[0] = setpoint[0];
			
			//fprintf(file, "%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f\n", PIDOutput[0], PIDOutput[1], PIDOutput[2], gyro[0], gyro[1], gyro[2]);
	
			// Now the rate control for yaw, pitch and roll
			for(int i=0; i<3; i++) {
				PIDOutput[i] = YPRRate[i].updatePID(PIDOutput1[i], gyro[i], dt);
			}
			//printf("PIDOutput: %6.4f, %6.4f, %6.4f, %6.4f\n", throttle, PIDOutput[0], PIDOutput[1], PIDOutput[2]);
			// Output to motors
			motors.update(throttle, PIDOutput);

			//fprintf(file, "%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f \n", cumTime, imuData.accel.z(), imuData.accel.y(), imuData.accel.x(), gyro[0], gyro[1], gyro[2], PIDOutput[0], PIDOutput[1], PIDOutput[2]);
			//fprintf(file, "%6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f %6.4f \n", cumTime, PIDOutput[0], PIDOutput[1], PIDOutput[2], PIDOutput1[0], PIDOutput1[1], PIDOutput1[2], attitude[0], attitude[1], attitude[2],gyro[0], gyro[1], gyro[2]);
			cumTime += dt;
		}
		
		// Send a heartbeat
		if((t.tv_sec - lastHeartbeat) > 1) {
			//printf("Sending heartbeat\n");
			MAVLink.sendHeartbeat();
	
			lastHeartbeat = t.tv_sec;
		}
		
		if((t.tv_nsec - lastDataSent) >= 100000000) {
			
			lastDataSent = t.tv_nsec;
			float tempkp = -1;
			float tempkd = -1;
			float tempki = -1;
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
					
					MAVLink.parsePID(tempkp,tempkd,tempki);
					YPRRate[YAW].setK(tempkp,tempkd,tempki);
			
					//printf("Kp = %4.2f, Kd = %4.2f, Ki = %4.2f\n", tempkp, tempkd,tempki);
					break;
		
				// Set the PID constants for the pitch and roll control (angle only)
				case SETPID_PR_STAB:
					MAVLink.parsePID(tempkp,tempkd,tempki);
					YPRStab[PITCH].setK(tempkp,tempkd,tempki);
					YPRStab[ROLL].setK(tempkp,tempkd,tempki);
			
					//printf("Kp = %4.2f, Kd = %4.2f, Ki = %4.2f\n", tempkp, tempkd,tempki);
					break;
		
				// Set the PID constants for the pitch and roll control (rate only)
				case SETPID_PR_RATE:
					MAVLink.parsePID(tempkp,tempkd,tempki);
					YPRRate[PITCH].setK(tempkp,tempkd,tempki);
					YPRRate[ROLL].setK(tempkp,tempkd,tempki);
			
					//printf("Kp = %4.2f, Kd = %4.2f, Ki = %4.2f\n", tempkp, tempkd,tempki);
					break;
		
				default:
					break;
			}

			// Clean the buffer so the message does not get repeated
			MAVLink.purge();
			
			MAVLink.sendStatus(attitude, gyro);
		}
		
		

		// Calculate next shot
		t.tv_nsec += interval;

		while (t.tv_nsec >= NSEC_PER_SEC) {
			   t.tv_nsec -= NSEC_PER_SEC;
			   lastDataSent -= NSEC_PER_SEC;
				t.tv_sec++;
		}
	}
}   