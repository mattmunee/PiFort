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

#include <Arduino.h>

#define MAX_PAYLOAD_DATA_SIZE		50

typedef struct{
	byte nodeID;
	byte gateWayID;
	byte networkID;
	byte numNodes;
}PiFortEEPROM;

enum PiFortNodeType {PF_GATEWAY,PF_NEEDS_A_NAME,PF_INTRUSION_NODE}; // Possible device types: PF_INTRUSION_NODE (accelerometer)
enum PiFortMessageType { PF_MSG_ACCEL_DATA, PF_MSG_NEW_NODE_INFO, PF_MSG_NEW_GATEWAY_INFO, PF_MSG_STATUS };

class Payload{
public:
	Payload(byte dataArraySize){
		payloadSize = sizeof(Payload) - (MAX_PAYLOAD_DATA_SIZE - dataArraySize)*sizeof(float);
	}
	byte payloadSize;					// size of payload in bytes
	bool status;						// True (armed), False (standby)
	PiFortMessageType msgType;			// enum describing message type
	byte numNodes;						// Number of nodes on network
	byte nodeID;						// Unique ID for each node on network
	byte gatewayID;						// Gateway ID
	PiFortNodeType nodeType;			// Device type
	byte numMeas;						// Number of data points to send
	byte currMeas;						// Position in data array of most current meas (used for circular buffer)
	float data[MAX_PAYLOAD_DATA_SIZE];	// Fixed size data array
};

#endif //PIFORTDATATYPES_H