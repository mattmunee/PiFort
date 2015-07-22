// Sample RFM69 receiver/gateway sketch, with ACK and optional encryption
// Passes through any wireless received messages to the serial port & responds to ACKs
// It also looks for an onboard FLASH chip, if present
// Library and code by Felix Rusu - felix@lowpowerlab.com
// Get the RFM69 and SPIFlash library at: https://github.com/LowPowerLab/

#include <EEPROM.h>
#include <ApplicationMonitor.h>
#include <RFM69.h>    //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>

#define NODEID        1    //unique for each node on same network
#define NETWORKID     100  //the same on all nodes that talk to each other
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
//#define FREQUENCY     RF69_433MHZ
//#define FREQUENCY     RF69_868MHZ
#define FREQUENCY     RF69_915MHZ
#define ENCRYPTKEY    "sampleEncryptKey" //exactly the same 16 characters/bytes on all nodes!
//#define IS_RFM69HW    //uncomment only for RFM69HW! Leave out if you have RFM69W!
#define ACK_TIME      30 // max # of ms to wait for an ack
#define SERIAL_BAUD   115200

//#define SERIAL_EN
#ifdef SERIAL_EN
#define DEBUG(input)   {Serial.print(input); delay(1);}
#define DEBUGln(input) {Serial.println(input); delay(1);}
#else
#define DEBUG(input);
#define DEBUGln(input);
#endif

#ifdef __AVR_ATmega1284P__
	#define LED           15 // Moteino MEGAs have LEDs on D15
	#define FLASH_SS      23 // and FLASH SS on D23
#else
	#define LED           9 // Moteinos have LEDs on D9
	#define FLASH_SS      8 // and FLASH SS on D8
#endif
#define MAXNUMMEAS    100

unsigned long prevLoopTime=0;

typedef struct{
	byte nodeID;
	byte numMeas;
	float accelMag2[MAXNUMMEAS];
}Payload;
Payload payLoad;
char payLoadChar[sizeof(Payload)];

RFM69 radio;
bool promiscuousMode = false; //set to 'true' to sniff all packets on the same network

Watchdog::CApplicationMonitor ApplicationMonitor;
int g_nIterations=0;

void setup() {
	Serial.begin(SERIAL_BAUD);
  
	// Enable Watchdog Timer
	DEBUGln(F("Initializing Watchdog..."));
	ApplicationMonitor.Dump(Serial);
	ApplicationMonitor.EnableWatchdog(Watchdog::CApplicationMonitor::Timeout_8s);


	DEBUGln(F("Initializing Radio..."));
	delay(10);
	radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW
	radio.setHighPower(); //only for RFM69HW!
#endif
	radio.encrypt(ENCRYPTKEY);
	radio.promiscuous(promiscuousMode);
	
	DEBUGln(F("EEPROM Contents:"));
	byte eepromValue;
	for(int i=0;i<512;i++){
		eepromValue = EEPROM.read(i);
		DEBUG(eepromValue);
		DEBUG(F(" "));
		delay(10);
	}

	DEBUGln(F("End Setup!"));
}

byte ackCount=0;
uint32_t packetCount = 0;
void loop() {
	DEBUG(F("Begin Loop..."));
	DEBUGln(g_nIterations);
	unsigned long currTime = millis();
	
	DEBUG(F("Loop Time (ms)"));
	DEBUGln(currTime-prevLoopTime);
	prevLoopTime=currTime;

	// Watchdog Maintenance
	ApplicationMonitor.IAmAlive();
	ApplicationMonitor.SetData(g_nIterations++);
	DEBUGln(F("Watchdog Fed..."));

	//check for any received packets
	DEBUGln(F("Checking radio RX..."));
	if (radio.receiveDone())
	{
		Serial.print(F("Packet Count: "));
		Serial.println(++packetCount);
		Serial.print(F("   Sender ID: "));
		Serial.print(radio.SENDERID, DEC);
		Serial.print(F("   RX_RSSI: "));
		Serial.print(radio.RSSI);
		Serial.print(F("   Data Length: "));
		Serial.print(radio.DATALEN);
    	
		payLoad=*(Payload*)radio.DATA;
		memcpy(&payLoadChar, &payLoad, sizeof(Payload));
		payLoad = *(Payload*)&payLoadChar;

		Serial.println(F("   Data: "));
		for (byte i = 0; i < payLoad.numMeas; i++){
			Serial.print(payLoad.accelMag2[i]);
			Serial.print(" ");
		}
		Serial.println("");
		Serial.println("");
		Serial.println(payLoadChar);
		
		if (promiscuousMode)
		{
		  DEBUG("Target ID: ");
		  DEBUGln(radio.TARGETID);
		}

		DEBUGln(F("Checking ACK Request..."));
		if (radio.ACKRequested())
		{
		  byte theNodeID = radio.SENDERID;
		  radio.sendACK();
		  DEBUG(F(" - ACK sent."));
		}
		DEBUGln();
	}
	DEBUGln(F("End loop!"));
}

void Blink(byte PIN, int DELAY_MS)
{
	pinMode(PIN, OUTPUT);
	digitalWrite(PIN,HIGH);
	delay(DELAY_MS);
	digitalWrite(PIN,LOW);
}
