#include "quadcopter.h"

#define MY_PRIORITY (49) /* we use 49 as the PRREMPT_RT use 50
                            as the priority of kernel tasklets
                            and interrupt handler by default */

#define MAX_SAFE_STACK (8*1024) /* The maximum stack size which is
                                   guaranteed safe to access without
                                   faulting */

#define NSEC_PER_SEC    (1000000000) /* The number of nsecs per sec. */

#define YAW 0
#define PITCH 1
#define ROLL 2

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
        int interval = 500000000; // 500ms = 2 Hz

        // Declare ourself as a real time task

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
		// Initialize the sensors
		/*
		
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
		
		*/
		
		//
		// Initialize ESCs
		//
		
		
		//
		// Initialize PID Controllers
		//
		float PIDOutput[3];
		
		PID YPRStab[3];
		PID YPRRate[3];
		
		YPRStab[PITCH].setK(1,0,0);
		YPRStab[ROLL].setK(1,0,0);
		
		YPRRate[YAW].setK(1,0,0);
		YPRRate[PITCH].setK(1,0,0);
		YPRRate[ROLL].setK(1,0,0);
		
		
        clock_gettime(CLOCK_MONOTONIC, &t);
		
        // Start after one second
        t.tv_sec++;

        while(1) {
			// Wait until next shot
			clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);

			// Read user input
			
			// Read sensor data
			
			// Get PID control
			printf("Hello World! %d \n", t);

			// Output to motors
			
			// Calculate next shot
			t.tv_nsec += interval;

			while (t.tv_nsec >= NSEC_PER_SEC) {
				   t.tv_nsec -= NSEC_PER_SEC;
					t.tv_sec++;
			}
   }
}
