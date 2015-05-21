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
	mStrMsg = "";
	memset(mBuf, 0, BUFFER_LENGTH);
	mMavlinkmsg = false;
}

//
// Initialization function for the communications channel
//
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

//
// Send the mandatory heartbeat to the ground station every x seconds
//
void Comms::sendHeartbeat() {
	mavlink_msg_heartbeat_pack(1, 200, &mMsg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
	
	mLen = mavlink_msg_to_send_buffer(mBuf, &mMsg);
	mBytesSent = sendto(mSocket, mBuf, mLen, 0, (struct sockaddr *) &mGcAddr, sizeof(struct sockaddr_in));
	
	memset(mBuf, 0, BUFFER_LENGTH);
}

//
// Function to send status and attitude updates to the ground station
//
void Comms::sendStatus(float attitude[3], float gyro[3]) {
	float pos[6];

	pos[0] = 1;
	pos[1] = 2;
	pos[2] = 3;
	pos[3] = 4;
	pos[4] = 5;
	pos[5] = 6;

	// Sending the status
	mavlink_msg_sys_status_pack(1, 200, &mMsg, 0, 0, 0, 500, 11000, -1, -1, 0, 0, 0, 0, 0, 0);
	mLen = mavlink_msg_to_send_buffer(mBuf, &mMsg);
	mBytesSent = sendto(mSocket, mBuf, mLen, 0, (struct sockaddr*)&mGcAddr, sizeof (struct sockaddr_in));
	
	// Sending the position
	mavlink_msg_local_position_ned_pack(1, 200, &mMsg, microsSinceEpoch(), 
									pos[0], pos[1], pos[2],
									pos[3], pos[4], pos[5]);
	mLen = mavlink_msg_to_send_buffer(mBuf, &mMsg);
	mBytesSent = sendto(mSocket, mBuf, mLen, 0, (struct sockaddr*)&mGcAddr, sizeof(struct sockaddr_in));
	
	// Sending the attitude
	mavlink_msg_attitude_pack(1, 200, &mMsg, microsSinceEpoch(), attitude[YAW], attitude[PITCH], attitude[ROLL], gyro[YAW], gyro[PITCH], gyro[ROLL]);
	mLen = mavlink_msg_to_send_buffer(mBuf, &mMsg);
	mBytesSent = sendto(mSocket, mBuf, mLen, 0, (struct sockaddr*)&mGcAddr, sizeof(struct sockaddr_in));
	
	memset(mBuf, 0, BUFFER_LENGTH);
}

//
// Function that checks if there is any data sent to the quadcopter from the ground station
//
int Comms::receiveData() {
	memset(mBuf, 0, BUFFER_LENGTH);
	mRecSize = recvfrom(mSocket, (void *) mBuf, BUFFER_LENGTH, 0, (struct sockaddr *) &mGcAddr, &mFromLen);
	
	//unsigned int temp = 0;
	mMavlinkmsg = false;
	
	if(mRecSize > 0) {
		//printf("Bytes received: %d\nDatagram: ", (int)mRecSize);
		
		for(int i = 0; i<mRecSize; i++) {
			//temp = mBuf[i];
			//printf("%02x ", (unsigned char)temp);
			
			if(mavlink_parse_char(MAVLINK_COMM_0, mBuf[i], &mMsg, &mStatus) && mMsg.msgid != 0) {
				printf("Received packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", mMsg.sysid, mMsg.compid, mMsg.len, mMsg.msgid);
				mMavlinkmsg = true;
				
			}
		}
		//printf("\n");
		
		if(!mMavlinkmsg) {
			//std::string s (reinterpret_cast<char*>(mBuf), sizeof(mBuf));
			std::string s (reinterpret_cast<char const*>(mBuf));
			printf("Received string: %s\n",s.c_str());
			mStrMsg = s;
		}
	}

	return getCommand();
}

void Comms::purge() {
	memset(mBuf, 0, BUFFER_LENGTH);
	mStrMsg = "";
	mMavlinkmsg = false;
}

int Comms::getCommand() {
	int type = 0;
	
	if(!mMavlinkmsg) {
		std::istringstream ss(mStrMsg);
		
		std::string sub;
		ss >> sub;
		
		do {
			if(sub == "START") {
				type = START;
				break;
			} else if(sub == "STOP") {
				type = STOP;
				break;
			} else if(sub == "SETPID") {
				ss >> sub;
				
				if(sub == "yaw")
					type = SETPID_YAW;
				else if(sub == "pr_stab")
					type = SETPID_PR_STAB;
				else if(sub == "pr_rate")
					type = SETPID_PR_RATE;
					
				break;
			} else if(sub == "SETPOINT") {
				type = SETPOINT;
				break;
			} else { break; }
		} while(ss);
	}
	
	return type;
}

void Comms::parsePID(float &kp, float &kd, float &ki) {
	std::istringstream ss(mStrMsg);

	do {
		std::string sub;
		ss >> sub;
		
		if(sub == "kp") {
			ss >> sub;
			kp = ::atof(sub.c_str());
		} else if(sub == "kd") {
			ss >> sub;
			kd = ::atof(sub.c_str());
		} else if(sub == "ki") {
			ss >> sub;
			ki = ::atof(sub.c_str());
		}
	} while (ss);
	
	//printf("Kp = %4.2f, Kd = %4.2f, Ki = %4.2f\n", kp, kd,ki);
}

void Comms::parseSetpoint(float &throttle, float &yaw, float &pitch, float &roll) {
	std::istringstream ss(mStrMsg);

	do {
		std::string sub;
		ss >> sub;
		
		if(sub == "yaw") {
			ss >> sub;
			yaw = ::atof(sub.c_str());
		} else if(sub == "pitch") {
			ss >> sub;
			pitch = ::atof(sub.c_str());
		} else if(sub == "roll") {
			ss >> sub;
			roll = ::atof(sub.c_str());
		} else if(sub == "throttle") {
			ss >> sub;
			throttle = ::atof(sub.c_str());
		}
	} while (ss);
}

//
// Close the connection
//
void Comms::stop() {
	memset(mBuf, 0, BUFFER_LENGTH);
	close(mSocket);
}

//
// Convenience function to get the number of microseconds since epoch
//
uint64_t Comms::microsSinceEpoch() {
	
	struct timespec time;
	
	uint64_t micros = 0;
	
	clock_gettime(CLOCK_REALTIME, &time);  
	micros = (uint64_t)time.tv_sec * 1000000 + time.tv_nsec/1000;
	
	return micros;
}