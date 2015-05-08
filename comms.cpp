//
// COMMS.H
//
// This file contains the implementation of the communication protcol MAVlink
//

#include "comms.h"

//
// Default Constructor
//
Comms::Comms() {

}

void Comms::init(std::string targetIP) {
	// Set the IP address of mission control
	mTargetIP = targetIP;
	
	mSocket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
	
	memset(&mLocAddr, 0, sizeof(mLocAddr));
	
	mLocAddr.sin_family = AF_INET;
	mLocAddr.sin_addr.s_addr = INADDR_ANY;
	mLocAddr.sin_port = htons(14551);
	
	// Bind the socket to port 14551, to connect to qgroundcontrol
	if(-1 == bind(mSocket, (struct sockaddr *) &mLocAddr, sizeof(struct sockaddr))) {
		perror("Binding to socket failed, exiting..");
		close(mSocket);
		exit(2);
	}

		
	if(fcntl(mSocket, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
		perror("Error setting non-blocking property, exiting..");
		close(mSocket);
		exit(2);
	}
	
	memset(&mGcAddr, 0, sizeof(mGcAddr));
	mGcAddr.sin_family = AF_INET;
	mGcAddr.sin_addr.s_addr = inet_addr(mTargetIP.c_str());
	mGcAddr.sin_port = htons(14550);
}

void Comms::sendHeartbeat() {
	mavlink_msg_heartbeat_pack(1, 200, &mMsg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
	
	mLen = mavlink_msg_to_send_buffer(mBuf, &mMsg);
	mBytesSent = sendto(mSocket, mBuf, mLen, 0, (struct sockaddr *) &mGcAddr, sizeof(struct sockaddr_in));
}

void Comms::sendStatus() {
	mPos[0] = 1;
	mPos[1] = 2;
	mPos[2] = 3;
	mPos[3] = 4;
	mPos[4] = 5;
	mPos[5] = 6;

	// Sending the status
	mavlink_msg_sys_status_pack(1, 200, &mMsg, 0, 0, 0, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0);
	mLen = mavlink_msg_to_send_buffer(mBuf, &mMsg);
	mBytesSent = sendto(mSocket, mBuf, mLen, 0, (struct sockaddr*)&mGcAddr, sizeof (struct sockaddr_in));
	
	// Sending the position
	mavlink_msg_local_position_ned_pack(1, 200, &mMsg, microsSinceEpoch(), 
									mPos[0], mPos[1], mPos[2],
									mPos[3], mPos[4], mPos[5]);
	mLen = mavlink_msg_to_send_buffer(mBuf, &mMsg);
	mBytesSent = sendto(mSocket, mBuf, mLen, 0, (struct sockaddr*)&mGcAddr, sizeof(struct sockaddr_in));
	
	// Sending the attitude
	mavlink_msg_attitude_pack(1, 200, &mMsg, microsSinceEpoch(), 1.2, 1.7, 3.14, 0.01, 0.02, 0.03);
	mLen = mavlink_msg_to_send_buffer(mBuf, &mMsg);
	mBytesSent = sendto(mSocket, mBuf, mLen, 0, (struct sockaddr*)&mGcAddr, sizeof(struct sockaddr_in));
}

uint64_t Comms::microsSinceEpoch() {
	
	struct timespec time;
	
	uint64_t micros = 0;
	
	clock_gettime(CLOCK_REALTIME, &time);  
	micros = (uint64_t)time.tv_sec * 1000000 + time.tv_nsec/1000;
	
	return micros;
}