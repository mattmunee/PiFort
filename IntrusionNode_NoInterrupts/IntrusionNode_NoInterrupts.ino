#include <PiFortDataTypes.h>
#include <ArduinoLED.h>
#include <SPI.h>
#include <Wire.h>
#include <MMA8452Q.h>
#include <RFM69.h>
#include <LowPower.h>

#define SERIAL_BAUD      115200
#define SERIAL_EN
#ifdef SERIAL_EN
#define DEBUG(input)   {Serial.print(input); delay(1);}
#define DEBUGln(input) {Serial.println(input); delay(1);}
#else
#define DEBUG(input);
#define DEBUGln(input);
#endif

#define LED				9            // LED is D9 on Motetino

RFM69 radio;       

#define NODEID			18    //unique for each node on same network
#define NETWORKID		100  //the same on all nodes that talk to each other
#define GATEWAYID		1
#define FREQUENCY		RF69_915MHZ
#define ENCRYPTKEY		"sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define NUM_MEAS_TX		10

MMA8452Q accelerometer;
Payload payload(NUM_MEAS_TX);

ArduinoLED led(LED);

int numints=0;
int measNum = 0;
bool motionInterruptCaught = false;
bool sleepInterruptCaught = false;
const float threshold = 0.6;		//Threshold offset from 1g field (in g's)

void setup()
{
	payload.nodeID = NODEID;
	payload.nodeType = PF_INTRUSION_NODE;
	payload.numMeas = NUM_MEAS_TX;
	Serial.begin(SERIAL_BAUD);
	DEBUGln("Start...");
	
	led.Strobe(5,50);

	// Setup radio
	radio.initialize(FREQUENCY,NODEID,NETWORKID);
	radio.encrypt(ENCRYPTKEY);
	radio.sleep(); // MOTEINO: sleep right away to save power

	// Setup accelerometer
	accelerometer.init(SCALE_2G, ODR_50);

}

void loop()
{
	if (accelerometer.available()){
		accelerometer.read();
		payload.data[measNum] = accelerometer.cx*accelerometer.cx
			+ accelerometer.cy*accelerometer.cy
			+ accelerometer.cz*accelerometer.cz;
	}

	payload.currMeas = measNum;
	if (radio.sendWithRetry(GATEWAYID, (const void*)(&payload), payload.payloadSize)){
		DEBUGln("ACK:OK");
	}
	else{
		DEBUGln("ACK:BAD");
	}	

	measNum++;
	if (measNum > NUM_MEAS_TX)measNum = 0;
}