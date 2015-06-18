//
// QUADCOPTER.H
//
// This is the main header file of the project and contains the main loop of the program.
//

#ifndef MAIN_H
#define MAIN_H

// Some stuff
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <unistd.h>

// The library for reading sensor data
#include "RTIMULib/RTIMULib.h"
#include "pid.h"
#include "esc.h"
#include "comms.h"
#include "filter.h"
#include "joystick.hh"

// We use 49 as the PRREMPT_RT use 50 as the priority of kernel tasklets
// and interrupt handler by default
#define MY_PRIORITY (49)

//The maximum stack size which is guaranteed safe to access without faulting
#define MAX_SAFE_STACK (8*1024)

// The number of nsecs per sec.
#define NSEC_PER_SEC (1000000000)

// For easy identification of angles
#define YAW 0
#define PITCH 1
#define ROLL 2

#define START 1
#define STOP 2
#define SETPOINT 3
#define SETPID_YAW 10
#define SETPID_PR_STAB 11
#define SETPID_PR_RATE 12

#endif
