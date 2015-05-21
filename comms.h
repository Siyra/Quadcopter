//
// COMMS.H
//
// This file contains the implementation of the communication protcol MAVlink
//

#ifndef comms_h
#define comms_h

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <arpa/inet.h>

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <time.h>
#include "MAVLink/mavlink.h"

#define BUFFER_LENGTH 2041

#define YAW 0
#define PITCH 1
#define ROLL 2

#define START 1
#define STOP 2
#define SETPOINT 3
#define SETPID_YAW 10
#define SETPID_PR_STAB 11
#define SETPID_PR_RATE 12


class Comms {
	private:
	struct sockaddr_in mGcAddr; 
	struct sockaddr_in mLocAddr;
	
	ssize_t mRecSize;
	socklen_t mFromLen;

	mavlink_message_t mMsg;
	mavlink_status_t mStatus;
	uint16_t mLen;
	uint8_t mBuf[BUFFER_LENGTH];
	int mBytesSent;
	bool mMavlinkmsg;
	
	std::string mStrMsg;
	
	std::string mTargetIP;
	int mSocket;
	
	public:
	Comms();
	
	void init(std::string targetIP);
	void sendHeartbeat();
	void sendStatus(float attitude[3], float gyro[3]);
	int receiveData();
	void stop();
	
	int getCommand();
	void parsePID(float &kp, float &kd, float &ki);
	void parseSetpoint(float &throttle, float &yaw, float &pitch, float &roll);
	
	void purge();
	uint64_t microsSinceEpoch();
};

#endif