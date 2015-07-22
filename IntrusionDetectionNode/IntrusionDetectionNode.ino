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

#define LED  9            // LED is D9 on Motetino

RFM69 radio;       

#define NODEID        18    //unique for each node on same network
#define NETWORKID     100  //the same on all nodes that talk to each other
#define GATEWAYID     1
#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!

#define NUMMEAS		10

typedef struct{
	byte nodeID;
	byte numMeas;
	float accelMag2[NUMMEAS];
}Payload;
Payload payLoad;

MMA8452Q accelerometer;

ArduinoLED led(LED);

int numints=0;
bool motionInterruptCaught = false;
bool sleepInterruptCaught = false;

void setup()
{
	payLoad.nodeID = NODEID;
	payLoad.numMeas = NUMMEAS;
	Serial.begin(SERIAL_BAUD);
	DEBUGln("Start...");
	
	led.Strobe(5,50);

	// Setup radio
	radio.initialize(FREQUENCY,NODEID,NETWORKID);
	radio.encrypt(ENCRYPTKEY);
	radio.sleep(); // MOTEINO: sleep right away to save power

	// Setup accelerometer
	accelerometer.init(SCALE_2G, ODR_50);
	accelerometer.setupMotionDetection(XY, 0.63, 0, INT_PIN2);
	accelerometer.setupAutoSleep(ODR_SLEEP_1,LOW_POWER, 0x08, 5.0,INT_PIN1);
	accelerometer.clearFFMotionInterrupt();

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

}

void loop()
{
	if (motionInterruptCaught | sleepInterruptCaught){
		DEBUGln(accelerometer.getInterruptSources());
		if (motionInterruptCaught)DEBUGln("Motion");
		if (sleepInterruptCaught)DEBUGln("Sleep");
		byte numMeasurements = 0;
		while (numMeasurements < NUMMEAS){
			if (accelerometer.available()){
				accelerometer.read();
				payLoad.accelMag2[numMeasurements] = accelerometer.cx*accelerometer.cx
					+ accelerometer.cy*accelerometer.cy
					+ accelerometer.cz*accelerometer.cz;
				numMeasurements++;
			}
		}
		for (byte i = 0; i < NUMMEAS; i++){
			DEBUG(payLoad.accelMag2[i]);
			DEBUG(" ");
		}
		DEBUGln("");
		DEBUGln("");

		if (radio.sendWithRetry(GATEWAYID, (const void*)(&payLoad), sizeof(payLoad))){
			DEBUGln("ACK:OK");
		}
		else{
			DEBUGln("ACK:BAD");
		}
		
		delay(100);
		accelerometer.clearFFMotionInterrupt();
		accelerometer.getSystemMode();
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