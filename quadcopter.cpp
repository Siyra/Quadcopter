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
        struct timespec t;
        struct sched_param param;
        //int interval = 500000; // 500us = 2000 Hz
        int interval = 500000000; // 50ms = 20 Hz
		float dt = 0.5;
		
		bool started = true;

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
		MAVLink.init("10.70.130.58");
		
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
#endif /* TEST */
			
		//
		// Initialize ESCs
		//
		printf("Initializing ESCs..\n");
		ESC motors;
		
		motors.init();
		
		//
		// Initialize PID Controllers
		//
		printf("Initializing PID Controllers..\n");
		float PIDOutput[3];
		float setpoint[3];
		float attitude[3];
		float gyro[3];
		float throttle = 0;
		
		setpoint[0] = 0;
		setpoint[1] = 0;
		setpoint[2] = 0;
		
		RTVector3 accel, prevAccel;
		RTVector3 velocity, prevVelocity;
		RTVector3 position, prevPosition;
		
		PID YPRStab[3];
		PID YPRRate[3];
		
		// The yaw is only controlled using the rate controller, pitch
		// and roll are also controlled using the angle.
		YPRStab[PITCH].setK(1,1,0.2);
		YPRStab[ROLL].setK(1,1,0.2);
		
		YPRRate[YAW].setK(1,0,0);
		YPRRate[PITCH].setK(1,1,0.2);
		YPRRate[ROLL].setK(1,1,0.2);
		
		
        clock_gettime(CLOCK_MONOTONIC, &t);
		
        // Start after one second
        t.tv_sec++;
		
		int lastHeartbeat = 0;
		float tempkp, tempkd, tempki;
		
		
		
		printf("Starting process loop with dt: %f\n",dt);
        while(1) {
			// Wait until next shot
			clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
			
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
					
					printf("Kp = %4.2f, Kd = %4.2f, Ki = %4.2f\n", tempkp, tempkd,tempki);
					break;
				
				// Set the PID constants for the pitch and roll control (angle only)
				case SETPID_PR_STAB:
					MAVLink.parsePID(tempkp,tempkd,tempki);
					YPRStab[PITCH].setK(tempkp,tempkd,tempki);
					YPRStab[ROLL].setK(tempkp,tempkd,tempki);
					
					printf("Kp = %4.2f, Kd = %4.2f, Ki = %4.2f\n", tempkp, tempkd,tempki);
					break;
				
				// Set the PID constants for the pitch and roll control (rate only)
				case SETPID_PR_RATE:
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
			
#ifndef TEST	
			// Read sensor data
			imu->IMURead();
			RTIMU_DATA imuData = imu->getIMUData();
			
			if (pressure != NULL)
                pressure->pressureRead(imuData);
			
			// Print the IMU data
			printf("Fusion pose: %s\n", RTMath::displayDegrees("", imuData.fusionPose));
			printf("Pressure: %4.1f, height above sea level: %4.1f, temperature: %4.1f\n",
                           imuData.pressure, RTMath::convertPressureToHeight(imuData.pressure), imuData.temperature);
			
			// This is the attitude angle of the UAV, so yaw, pitch and roll
			attitude[YAW] = imuData.fusionPose.z();
			attitude[PITCH] = imuData.fusionPose.x();
			attitude[ROLL] = imuData.fusionPose.y();
			
			// These are the angular accelerations of the UAV
			gyro[YAW] = imuData.gyro.z();
			gyro[PITCH] = imuData.gyro.x();
			gyro[ROLL] = imuData.gyro.y();
			
			if(started) {
				// Lets try get the acceleration residuals (without the gravity vector) and integrate these twice
				// to get the position.
				accel = imuData.getAccelResiduals();
				velocity += (accel*dt);
				position += (velocity*dt);
			}
			
			// The position data needs to be fused with GPS / lidar to correct for the poor estimate
			// integrating twice gives
#else
			// This is the attitude angle of the UAV, so yaw, pitch and roll
			attitude[YAW] = 0;
			attitude[PITCH] = 0;
			attitude[ROLL] = 0;
			
			// These are the angular accelerations of the UAV
			gyro[YAW] = 0;
			gyro[PITCH] = 0;
			gyro[ROLL] = 0;
#endif /* TEST */

			MAVLink.sendStatus(attitude, gyro);
			
			if(started) {
			
				// First the stability control for pitch and roll
				for(int i=1; i<3; i++) {
					PIDOutput[i] = YPRStab[i].updatePID(setpoint[i], attitude[i], dt);
				}
				PIDOutput[0] = attitude[0];
				
				// Now the rate control for yaw, pitch and roll
				for(int i=0; i<3; i++) {
					PIDOutput[i] = YPRRate[i].updatePID(PIDOutput[i], gyro[i], dt);
				}
				
				//printf("%lld.%.9ld\n", (long long)t.tv_sec, t.tv_nsec);
				
				// Output to motors
				motors.update(throttle, PIDOutput);
			}
			
			// Calculate next shot
			t.tv_nsec += interval;

			while (t.tv_nsec >= NSEC_PER_SEC) {
				   t.tv_nsec -= NSEC_PER_SEC;
					t.tv_sec++;
			}
   }
}
