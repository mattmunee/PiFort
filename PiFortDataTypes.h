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

enum PiFortNodeType {PF_INTRUSION_NODE}; // Possible device types: PF_INTRUSION_NODE (accelerometer)

class Payload{
public:
	Payload(byte dataArraySize){
		payloadSize = sizeof(Payload) - (MAX_PAYLOAD_DATA_SIZE - dataArraySize)*sizeof(float);
	}
	byte payloadSize;
	byte nodeID;						// Unique ID for each node on network
	PiFortNodeType nodeType;			// Device type
	byte numMeas;						// Number of data points to send
	byte currMeas;						// Position in data array of most current meas (used for circular buffer)
	float data[MAX_PAYLOAD_DATA_SIZE];	// Fixed size data array
};

#endif //PIFORTDATATYPES_H