#include <PiFortDataTypes.h>
#include <ArduinoLED.h>
#include <SPI.h>
#include <Wire.h>
#include <MMA8452Q.h>
#include <RFM69.h>
#include <LowPower.h>
#include <EEPROM.h>

#define SERIAL_BAUD      19200
#define SERIAL_EN
#ifdef SERIAL_EN
#define DEBUG(input)   {int ms=10;delay(ms);Serial.print(input); delay(ms);}
#define DEBUGln(input) {int ms=10;delay(ms);Serial.println(input); delay(ms);}
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
#define CHECKIN_PERIOD	5000 //300000 //5 minute intervals 
#define BATT_PIN		A1
#define VOLT_SCALE		1.70213			//Voltage divider scaling =(R1+R2)/R2 (where R2 is lower resistor)
#define AREF			3.3				//Reference voltage (3.3V for Moteino)
#define ANLG_SCALE		0.00097751711	//=1/1023

RFM69 radio;
MMA8452Q accelerometer;
Payload payloadTx(NUM_MEAS_TX);
Payload payloadRx(NUM_MEAS_TX);
float battScaleFactor = AREF*VOLT_SCALE*ANLG_SCALE;


ArduinoLED led(LED);

int numints=0;
bool motionInterruptCaught = false;
bool sleepInterruptCaught = false;
byte intSource;
accelData data;
PiFortEEPROM eeprom;
signed long prevTime;
signed long currTime;
bool isArmed;

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
			payloadTx.updateChecksum();
			if (radio.sendWithRetry(GATEWAYID, (const void*)(&payloadTx), payloadTx.payloadSize)){
				DEBUGln("Response from Gateway!");
				payloadRx = *(Payload*)radio.DATA;
				if (payloadRx.validateChecksum()){
					Serial.println("CHECKSUM: OK");
				}
				else{
					Serial.println("CHECKSUM: BAD");
				}
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
	payloadTx.gatewayID = GATEWAYID;
	payloadTx.status = true;
	isArmed = true;

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
	if (accelerometer.isActive()&!isArmed)
		accelerometer.standby();
	if (isArmed&!accelerometer.isActive())
		accelerometer.active();

	currTime = millis();
	if (currTime - prevTime > CHECKIN_PERIOD){
		prevTime = currTime;
		DEBUGln("Checking in...");
		payloadTx.batteryVoltage = battScaleFactor*analogRead(BATT_PIN);
		DEBUG("Battery Voltage: ");
		DEBUGln(payloadTx.batteryVoltage);
		payloadTx.msgType = PF_MSG_STATUS;
		payloadTx.updateChecksum();
		if (radio.sendWithRetry(GATEWAYID, (const void*)(&payloadTx), payloadTx.payloadSize)){
			payloadRx = *(Payload*)radio.DATA;
			if (payloadRx.validateChecksum()){
				Serial.println("CHECKSUM: OK");
			}
			else{
				Serial.println("CHECKSUM: BAD");
			}
			isArmed = payloadRx.status;
			DEBUGln(F("ACK:OK"));
			DEBUG("Status: ");
			DEBUGln(payloadRx.status);
		}
		else{
			DEBUGln("ACK:BAD");
		}
	}

	if (motionInterruptCaught | sleepInterruptCaught){
		if (isArmed & motionInterruptCaught){
			if (motionInterruptCaught)DEBUGln("MOTION");
			if (sleepInterruptCaught)DEBUGln("SLEEP");
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
					data = accelerometer.getData();
					payloadTx.data[measNum] = data.scaled.x*data.scaled.x
						+ data.scaled.y*data.scaled.y
						+ data.scaled.z*data.scaled.z;
					measNum++;
					payloadTx.msgType = PF_MSG_ACCEL_DATA;
				}
			}
			for (byte i = 0; i < NUM_MEAS_TX; i++){
				DEBUG(payloadTx.data[i]);
				DEBUG(" ");
			}

			payloadTx.updateChecksum();
			if (radio.sendWithRetry(GATEWAYID, (const void*)(&payloadTx), payloadTx.payloadSize)){
				payloadRx = *(Payload*)radio.DATA;
				isArmed = payloadRx.status;
				DEBUGln(F("ACK:OK"));
				DEBUG("Status: ");
				DEBUGln(payloadRx.status);
			}
			else{
				DEBUGln("ACK:BAD");
			}

		}

		motionInterruptCaught = false;
		sleepInterruptCaught = false;
		accelerometer.clearAllInterrupts();
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