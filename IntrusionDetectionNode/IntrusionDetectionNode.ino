#include <PiFortDataTypes.h>
#include <ArduinoLED.h>
#include <SPI.h>
#include <Wire.h>
#include <MMA8452Q.h>
#include <RFM69.h>
#include <LowPower.h>
#include <EEPROM.h>

#define SERIAL_BAUD      9600
//#define SERIAL_EN
#ifdef SERIAL_EN
#define DEBUG(input)   {Serial.print(input); delay(5);}
#define DEBUGln(input) {Serial.println(input); delay(5);}
#else
#define DEBUG(input);
#define DEBUGln(input);
#endif

#define LED				9            // LED is D9 on Motetino
#define NETWORKID		100  //the same on all nodes that talk to each other
#define GATEWAYID		1
#define FREQUENCY		RF69_915MHZ
#define ENCRYPTKEY		"sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
#define NUM_MEAS_TX		10
#define RX_PERIOD		5000 //300000 //5 minute intervals

RFM69 radio;
MMA8452Q accelerometer;
Payload payloadTx(NUM_MEAS_TX);
Payload payloadRx(NUM_MEAS_TX);

ArduinoLED led(LED);

int numints=0;
bool motionInterruptCaught = false;
bool sleepInterruptCaught = false;
byte intSource;
accelData data;
PiFortEEPROM eeprom;
signed long prevTime;
signed long currTime;

void setup()
{
	Serial.begin(SERIAL_BAUD);
	DEBUGln("Start...");

	// Read EEPROM
	EEPROM.get(0, eeprom);
	DEBUGln("EEPROM: ");	
	DEBUG("GATEWAY ID: "); DEBUGln(eeprom.gateWayID);
	DEBUG("NETWORK ID: "); DEBUGln(eeprom.networkID);
	DEBUG("NODE ID: "); DEBUGln(eeprom.nodeID);
	DEBUG("NUM NODES: "); DEBUGln(eeprom.numNodes);
	if (eeprom.nodeID == 255){
		DEBUGln("EEPROM Not Set");
		eeprom.gateWayID = GATEWAYID;
		eeprom.networkID = NETWORKID;
		eeprom.numNodes = 0;
		bool setupSuccessful = false;
		radio.initialize(FREQUENCY, 255, NETWORKID);
		radio.encrypt(ENCRYPTKEY);
		payloadTx.nodeID = PF_NEEDS_A_NAME;
		while (!setupSuccessful){
			if (radio.sendWithRetry(GATEWAYID, (const void*)(&payloadTx), payloadTx.payloadSize)){
				DEBUGln("Response from Gateway!");
				payloadRx = *(Payload*)radio.DATA;
				if (payloadRx.msgType == PF_MSG_NEW_NODE_INFO){
					DEBUGln("Received Setup Info.");
					DEBUG("New Node ID: ");
					DEBUGln(payloadRx.nodeID);
					DEBUG("Num Nodes: ");
					DEBUGln(payloadRx.numNodes);
					eeprom.nodeID = payloadRx.nodeID;
					eeprom.numNodes = payloadRx.numNodes;
					EEPROM.put(0, eeprom);
					setupSuccessful = true;
				}
			}
			else{
				DEBUGln("ACK: BAD");
			}
		}
	}

	payloadTx.nodeID = eeprom.nodeID;
	payloadTx.nodeType = PF_INTRUSION_NODE;
	payloadTx.numMeas = NUM_MEAS_TX;

	led.Strobe(5,50);

	// Setup radio
	radio.initialize(FREQUENCY,eeprom.nodeID,NETWORKID);
	radio.encrypt(ENCRYPTKEY);
	radio.sleep(); // MOTEINO: sleep right away to save power

	// Setup accelerometer
	accelerometer.init(SCALE_2G, ODR_100);
	//accelerometer.setupFreefallOrMotionDetection(MOTION, XY, 0.63, 0, INT_PIN1);
	accelerometer.setupFreefallOrMotionDetection(FREEFALL, Y, 0.63, 0, INT_PIN1);
	accelerometer.setupTransientDetection(false, XYZ, 0.23, 0, INT_PIN1);
	accelerometer.setupPortraitLandscapeDetection(2, INT_PIN1);
	accelerometer.setupAutoSleep(ODR_SLEEP_12,LOW_POWER, 0x0F, 4.0,INT_PIN2);
	accelerometer.clearAllInterrupts();

	// Setup Interrupts
	// Use hardware interrupt 1 for motion interrupts (hardware 0 is tied to radio)
	pinMode(INT1, INPUT);
	attachInterrupt(INT1, catchMotionInterrupt, FALLING);

	// Use pin change interrupt to catch sleep/wake interrupt
	pinMode(PC0, INPUT);
	cli();
	PCICR |= 0x02;		//enable pin change interrupt PCI1, ports PCINT[8:14] (PCIE1 is bit 1)
	PCMSK1 |= 0x01;		//enable indiviual pins on vector PCI1, PCINT8 = bit 0 (PCINT20 = PC0 = ADC0 = D14)
	sei();

	prevTime = millis();
}

void loop()
{
	currTime = millis();
	if (currTime - prevTime > RX_PERIOD){
		prevTime = currTime;
		delay(10);
		DEBUGln("Listening...");
		delay(10);
		if (radio.receiveDone()){
			DEBUGln("Doing what I'm told.");
		}
	}
	if (motionInterruptCaught|sleepInterruptCaught){
		intSource = accelerometer.getInterruptSources();
		DEBUGln(F("Interrupt Sources:"));
		if (intSource & 0x01)DEBUGln(F("SRC_DRDY"));
		if (intSource & 0x04)DEBUGln(F("SRC_FF_MT"));
		if (intSource & 0x08)DEBUGln(F("SRC_PULSE"));
		if (intSource & 0x10)DEBUGln(F("SRC_LNDPRT"));
		if (intSource & 0x20)DEBUGln(F("SRC_TRANS"));
		if (intSource & 0x80)DEBUGln(F("SRC_ASLP"));

		byte measNum = 0;
		while (measNum < NUM_MEAS_TX){
			if (accelerometer.available()){
				data=accelerometer.getData();
				payloadTx.data[measNum] = data.scaled.x*data.scaled.x
					+ data.scaled.y*data.scaled.y
					+ data.scaled.z*data.scaled.z;
				measNum++;
				payloadTx.msgType = PF_MSG_DATA;
			}
		}
		for (byte i = 0; i < NUM_MEAS_TX; i++){
			DEBUG(payloadTx.data[i]);
			DEBUG(" ");
		}
		
		if (radio.sendWithRetry(GATEWAYID, (const void*)(&payloadTx), payloadTx.payloadSize)){
			delay(10);
			DEBUGln(F("ACK:OK"));
		}
		else{
			DEBUGln("ACK:BAD");
		}
		delay(10);
		accelerometer.clearAllInterrupts();

		motionInterruptCaught = false;
		sleepInterruptCaught = false;
	}
	else{
		LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
		prevTime -= 1000;//Since we're sleeping for 1s and the timer is turn off
		radio.sleep();
	}

}

void catchMotionInterrupt(){
	motionInterruptCaught = true;
}

ISR(PCINT1_vect)
{
	sleepInterruptCaught = true;
}