/******************************************************************************
PiFortDataTypes.h
Matt Reaves
July 26, 2015
https://github.com/mattmunee/PiFort

This code provides common data types for the PiFort Architecture

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef PIFORTDATATYPES_H
#define PIFORTDATATYPES_H

#include <string.h>

#define MAX_PAYLOAD_DATA_SIZE		100

typedef struct{
	unsigned char nodeID;
	unsigned char gateWayID;
	unsigned char networkID;
	unsigned char numNodes;
}PiFortEEPROM;

enum PiFortNodeType {PF_GATEWAY,PF_NEEDS_A_NAME,PF_INTRUSION_NODE}; // Possible device types: PF_INTRUSION_NODE (accelerometer)
enum PiFortMessageType { PF_MSG_ACCEL_DATA, PF_MSG_NEW_NODE_INFO, PF_MSG_NEW_GATEWAY_INFO, PF_MSG_STATUS };

class Payload{
public:
	Payload(unsigned int dataArraySize){
		payloadSize = sizeof(Payload) - (MAX_PAYLOAD_DATA_SIZE - dataArraySize)*sizeof(float);
	}
	void updateChecksum(void){
		unsigned char* selfPtr = (unsigned char *)this;
		this->crc = 0;
		int bytesOccupied = sizeof(Payload) - (MAX_PAYLOAD_DATA_SIZE - this->numMeas)*sizeof(float);
		// Start at second address (do not include checksum!)
		for (int i = 1; i < bytesOccupied - 1; i++){
			this->crc+=selfPtr[i];
		}
	}

	bool validateChecksum(void){
		unsigned char crcRx = this->crc;
		Payload temp(MAX_PAYLOAD_DATA_SIZE);
		memcpy(&temp, this, sizeof(Payload));
		temp.updateChecksum();
		return temp.crc == crcRx;
	}
	
	unsigned char		crc;							// checksum
	unsigned char		payloadSize;					// size of payload in bytes
	bool				status;							// True (armed), False (standby)
	PiFortMessageType	msgType;						// enum describing message type
	unsigned char		numNodes;						// Number of nodes on network
	unsigned char		nodeID;							// Unique ID for each node on network
	unsigned char		gatewayID;						// Gateway ID
	PiFortNodeType		nodeType;						// Device type
	unsigned long		time;							// Generic time
	float				batteryVoltage;					// Voltage measurement
	unsigned char		numMeas;						// Number of data points to send
	unsigned char		currMeas;						// Position in data array of most current meas (used for circular buffer)
	float				data[MAX_PAYLOAD_DATA_SIZE];	// Fixed size data array
};

#endif //PIFORTDATATYPES_H