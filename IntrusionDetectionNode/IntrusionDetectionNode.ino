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
#define DEBUG(input)   {Serial.print(input); delay(5);}
#define DEBUGln(input) {Serial.println(input); delay(5);}
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
bool motionInterruptCaught = false;
bool sleepInterruptCaught = false;
byte intSource;
accelData data;

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
	accelerometer.init(SCALE_2G, ODR_100);
	//accelerometer.setupFreefallOrMotionDetection(MOTION, XY, 0.63, 0, INT_PIN2);
	//accelerometer.setupFreefallOrMotionDetection(FREEFALL, Y, 0.63, 0, INT_PIN2);
	accelerometer.setupPortraitLandscapeDetection(2, INT_PIN2);
	//accelerometer.setupAutoSleep(ODR_SLEEP_12,LOW_POWER, 0x08, 5.0,INT_PIN2);
	accelerometer.clearAllInterrupts();

	// Setup Interrupts
	// Use hardware interrupt 1 for motion interrupt (hardware 0 is tied to radio)
	pinMode(INT1, INPUT);
	attachInterrupt(INT1, catchMotionInterrupt, RISING);

	// Use pin change interrupt to catch sleep/wake interrupt
	pinMode(PC0, INPUT);
	cli();
	PCICR |= 0x02;		//enable pin change interrupt PCI1, ports PCINT[8:14] (PCIE1 is bit 1)
	PCMSK1 |= 0x01;		//enable indiviual pins on vector PCI1, PCINT8 = bit 0 (PCINT20 = PC0 = ADC0 = D14)
	sei();

	DEBUG(F("PL_CFG: "));
	DEBUGln(accelerometer.readRegister(PL_CFG));
	DEBUG(F("PL_COUNT: "));
	DEBUGln(accelerometer.readRegister(PL_COUNT));
	DEBUG(F("CTRL_REG4: "));
	DEBUGln(accelerometer.readRegister(CTRL_REG4));
	DEBUG(F("CTRL_REG5: "));
	DEBUGln(accelerometer.readRegister(CTRL_REG5));
}

void loop()
{
	if (motionInterruptCaught | sleepInterruptCaught){
		intSource = accelerometer.getInterruptSources();
		DEBUGln(F("Interrupt Sources:"));
		if (intSource & 0x01)DEBUGln(F("SRC_DRDY"));
		if (intSource & 0x04)DEBUGln(F("SRC_FF_MT"));
		if (intSource & 0x08)DEBUGln(F("SRC_PULSE"));
		if (intSource & 0x10)DEBUGln(F("SRC_LNDPRT"));
		if (intSource & 0x20)DEBUGln(F("SRC_TRANS"));
		if (intSource & 0x80)DEBUGln(F("SRC_ASLP"));

		if (motionInterruptCaught)DEBUGln(F("Motion"));
		if (sleepInterruptCaught)DEBUGln(F("Sleep"));
		byte measNum = 0;
		while (measNum < NUM_MEAS_TX){
			if (accelerometer.available()){
				data=accelerometer.getData();
				payload.data[measNum] = data.scaled.x*data.scaled.x
					+ data.scaled.y*data.scaled.y
					+ data.scaled.z*data.scaled.z;
				measNum++;
			}
		}
		for (byte i = 0; i < NUM_MEAS_TX; i++){
			DEBUG(payload.data[i]);
			DEBUG(" ");
		}
		DEBUGln(F(""));
		DEBUGln(F(""));
		DEBUG(F("Size Of Payload: "));
		DEBUGln(sizeof(Payload));
		DEBUG(F("Size Of float: "));
		DEBUGln(sizeof(float*));
		DEBUG(F("Size Of PiFortNodeType: "));
		DEBUGln(sizeof(PiFortNodeType));
		DEBUG(F("Size Of Payload: "));
		byte payloadSize = payload.payloadSize;
		DEBUGln(payload.payloadSize);
		if (radio.sendWithRetry(GATEWAYID, (const void*)(&payload), payload.payloadSize)){
			DEBUGln(F("ACK:OK"));
		}
		else{
			DEBUGln("ACK:BAD");
		}

		accelerometer.clearAllInterrupts();

		motionInterruptCaught = false;
		sleepInterruptCaught = false;
	}
	else{
		LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
		radio.sleep();
	}

}

void catchMotionInterrupt(){
	motionInterruptCaught=true;
}

ISR(PCINT1_vect)
{
	sleepInterruptCaught = true;
}