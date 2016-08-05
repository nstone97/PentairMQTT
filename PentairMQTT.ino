// RS485 Protocol (Sending/Receiving) blatently stollen from SDYoung
// (http://www.sdyoung.com/home/pool-status/how-i-control-the-pool/)
// with some tweaks (probably for the worse) and implemented the MQTT communication

// This sketch sets up 3 telnet servers - 23, 8001, 8002 which all serve different functions
// 23 - General status of the RS485 (Packet output and message information) and MQTT
// 8001 - Debugging steps for the Pentair Packet breakdown
// 8002 - Raw output from the RS485 bus

// I did not use the serial port for any communication as the Feather M0 does NOT handle not having
// a terminal connected well (locks or runs VERY slow).  Also the Terminal locks at times if the buffer
// can't clear fast enough...  I had to telnet into the Feather from a linux box for things to work smoothly as
// PUTTY would block the thing consistently.

#define VER 1.0

#include <SPI.h>
#include <Adafruit_WINC1500.h>
#include <PubSubClient.h>
#include "wifiSettings.h"

//Output LED
int status = 13;

// Setup the WINC1500 Wireless Connection
#define WINC_CS   8
#define WINC_IRQ  7
#define WINC_RST  4
#define WINC_EN   2 
Adafruit_WINC1500 WiFi(WINC_CS, WINC_IRQ, WINC_RST);
int wlstatus = WL_IDLE_STATUS;     // the Wifi radio's status

// Setup the Telenet Servers
Adafruit_WINC1500Server tstatus(23);	// Regular output server
Adafruit_WINC1500Server tdebug(8001);	// Debugging output server
Adafruit_WINC1500Server tstream(8002);	// Raw stream from the RS485 Bus
Adafruit_WINC1500Client cStatus;		// Status client

//Set up the ethernet client
Adafruit_WINC1500Client cMQTT;
IPAddress ipMQTT(192, 168, 1, 6);
PubSubClient mqtt(ipMQTT, 1883, mqttCallback,cMQTT);

long lastReconnectAttempt = 0;
long lastPing = 0;
long lastHeartBeat = millis();
long lastSend = 0;

// RS485 Setup
#define RS485Transmit HIGH
#define RS485Receive LOW
#define TXControl 9

// Setup the RS485 Variables
char pc;								// CHAR on the last cycle
uint8_t buffer[256];
uint8_t* bPointer;
uint8_t bufferOfBytes[256];
uint8_t* bPointerOfBytes;

int bStep = 0;
//Packet Steps
#define header1 1
#define header3 2
#define header4 3
#define bufferData 4
#define calcCheckSum 5
#define saltHead2 6
#define saltTerm 7
#define bufferSaltData 8
int goToCase = header1;
int byteNum;
int sumOfBytesInChkSum;
int bytesOfDataToGet;
int remainingBytes;
int chkSumBits;
int chkSumValue;

// Status Variables
int poolTemp;
int oldPoolTemp;
int airTemp;
int oldAirTemp;
int poolMode;
int featureMode;
int panelHour;
int panelMinute;
int pumpState;
int oldPumpState;
int pumpSet;
int lightState=3;
int oldLightState;
int lightSet;
int waterfallState;
int oldWaterfallState;
int waterfallSet;
int bubblerState;
int oldBubblerState;
int bubblerSet;
int pumpMode;
int pumpRPM=3;
int oldPumpRPM;
int pumpWatts;
int oldPumpWatts;
int chlorDuty;
int oldChlorDuty;
int chlorDutySet;
int saltPct;
int oldSaltPct;

bool frameReceived = false;
bool sendCommand = false;

// MQTT Topics
char mqttWifiStrength[] PROGMEM = "/8230/backyard/pool/controller/wifi";
char mqttWaterTemp[] PROGMEM = "/8230/backyard/pool/watertemp";
char mqttAirTemp[] PROGMEM = "/8230/backyard/pool/airtemp";
char mqttPumpState[] PROGMEM = "/8230/backyard/pool/pump/state";
char mqttLightState[] PROGMEM = "/8230/backyard/pool/light/state";
char mqttWaterfallState[] PROGMEM = "/8230/backyard/pool/waterfall/state";
char mqttBubblerState[] PROGMEM = "/8230/backyard/pool/bubbler/state";
char mqttPumpRPM[] PROGMEM = "/8230/backyard/pool/pump/rpm";
char mqttPumpWatt[] PROGMEM = "/8230/backyard/pool/pump/watt";
char mqttSaltSet[] PROGMEM = "/8230/backyard/pool/chlor/setpoint";
char mqttSaltPct[] PROGMEM = "/8230/backyard/pool/chlor/salinity";
char mqttSubPump[] PROGMEM = "/8230/backyard/pool/pump/set";
char mqttSubLight[] PROGMEM = "/8230/backyard/pool/light/set";
char mqttSubWF[] PROGMEM = "/8230/backyard/pool/waterfall/set";
char mqttSubBub[] PROGMEM = "/8230/backyard/pool/bubbler/set";
char mqttSubChlor[] PROGMEM = "/8230/backyard/pool/chlor/set";

//   PACKET FORMAT				 <------------------NOISE---------------->  <------PREAMBLE------>  Sub   Dest  Src   CFI   Len   Dat1  Dat2  ChkH  ChkL     //Checksum is sum of bytes A5 thru Dat2.
byte pumpOn[] PROGMEM =			{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xA5, 0x01, 0x10, 0x22, 0x86, 0x02, 0x06, 0x01, 0x01, 0x67 };
byte pumpOff[] PROGMEM =		{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xA5, 0x01, 0x10, 0x22, 0x86, 0x02, 0x06, 0x00, 0x01, 0x66 };
byte poolLightOn[] PROGMEM =	{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xA5, 0x01, 0x10, 0x22, 0x86, 0x02, 0x03, 0x01, 0x01, 0x64 };
byte poolLightOff[] PROGMEM =	{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xA5, 0x01, 0x10, 0x22, 0x86, 0x02, 0x03, 0x00, 0x01, 0x63 };
byte waterFallOn[] PROGMEM =	{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xA5, 0x01, 0x10, 0x22, 0x86, 0x02, 0x0B, 0x01, 0x01, 0x6C };
byte waterFallOff[] PROGMEM =	{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xA5, 0x01, 0x10, 0x22, 0x86, 0x02, 0x0B, 0x00, 0x01, 0x6B };
byte bubblerOn[] PROGMEM =		{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xA5, 0x01, 0x10, 0x22, 0x86, 0x02, 0x0C, 0x01, 0x01, 0x6D };
byte bubblerOff[] PROGMEM =		{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xA5, 0x01, 0x10, 0x22, 0x86, 0x02, 0x0C, 0x00, 0x01, 0x6C };
byte chlorSetPct[]  =			{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0xA5, 0x01, 0x10, 0x22, 0x99, 0x04, 0x01, 0x02, 0x00, 0x00, 0x01, 0x7C };

// Return the Free Ram for a printout
extern "C" char *sbrk(int i);
int freeRam() {
	char stack_dummy = 0;
	return &stack_dummy - sbrk(0);
}

void setup()
{
	// Setup the pins
#ifdef WINC_EN
	pinMode(WINC_EN, OUTPUT);
	digitalWrite(WINC_EN, HIGH);
#endif

	pinMode(status, OUTPUT);
	digitalWrite(status, LOW);
	pinMode(TXControl, OUTPUT);
	digitalWrite(TXControl, RS485Receive);

	// check for the presence of the shield:
	if (WiFi.status() == WL_NO_SHIELD) errorLoop();

	// attempt to connect to Wifi network:
	while (WiFi.status() != WL_CONNECTED) {
		wlstatus = WiFi.begin(ssid, pass);

		// wait 10 seconds for connection:
		uint8_t timeout = 10;
		while (timeout && (WiFi.status() != WL_CONNECTED)) {
			timeout--;
			delay(1000);
		}
	}

	tstatus.begin();		// Start the telnet servers
	tdebug.begin();
	tstream.begin();

	Serial1.begin(9600);	// Kick on the Serial Port

	//Setup the Buffer array
	memset(buffer, 0, sizeof(buffer));
	bPointer = buffer;
	memset(bufferOfBytes, 0, sizeof(bufferOfBytes));
	bPointerOfBytes = bufferOfBytes;

	// Setup the MQTT Connection
	lastReconnectAttempt = 0;
	lastPing = 0;

	//Flash the LED to show we are a go
	digitalWrite(status, HIGH);
	delay(500);
	digitalWrite(status, LOW);
	delay(500);
	digitalWrite(status, HIGH);
	delay(500);
	digitalWrite(status, LOW);
	delay(500);
	digitalWrite(status, HIGH);
	delay(500);
	digitalWrite(status, LOW);
	delay(500);
	digitalWrite(status, HIGH);
}

void loop() {
	digitalWrite(TXControl, RS485Receive);

	// Check for Telnet Clients
	Adafruit_WINC1500Client cstatus = tstatus.available();
	Adafruit_WINC1500Client cdebug = tdebug.available();
	Adafruit_WINC1500Client cstream = tstream.available();
	cStatus = cstatus;

	long now = millis();
	if (!mqtt.connected()) {
		if (now - lastReconnectAttempt > 5000) {
			if (cstatus && cstatus.connected()) cstatus.println("*** MQTT Disconnected - trying to connect ***");
			lastReconnectAttempt = now;
			// Attempt to reconnect
			if (mqttConnect()) {
				lastReconnectAttempt = 0;
			}
		}
	}
	else {
		// Client connected
		mqtt.loop();
	}

	if (now - lastHeartBeat > 60000) {
		if (mqttHeartBeat(cstatus)) lastHeartBeat = now;
	}

	// CHeck if we need to send a command - and only do it if we aren't in the middle of buffering a packet
	if (sendCommand && goToCase == 1 && !Serial1.available()) {
		if (now - lastSend > 5000) {					// Send commands every 2 seconds until system responds
			lastSend = now;

			if (pumpSet != pumpState) {						//Send the pump command
				if (cStatus && cStatus.connected()) cStatus.println(F("Sending Pump Command"));
				if (pumpSet == 0) sendRS485(pumpOff, sizeof(pumpOff));
				else if (pumpSet == 1) sendRS485(pumpOn, sizeof(pumpOn));
			}
			else if (lightSet != lightState) {						//Send the pump command
				if (cStatus && cStatus.connected()) cStatus.println(F("Sending Light Command"));
				if (lightSet == 0) sendRS485(poolLightOff, sizeof(poolLightOff));
				else if (lightSet == 1) sendRS485(poolLightOn, sizeof(poolLightOn));
			}
			else if (waterfallSet != waterfallState) {
				if (cStatus && cStatus.connected()) cStatus.println(F("Sending Waterfall Command"));
				if (waterfallSet == 0) sendRS485(waterFallOff, sizeof(waterFallOff));
				else if (waterfallSet == 1) sendRS485(waterFallOn, sizeof(waterFallOn));
			}
			else if (bubblerSet != bubblerState) {
				if (cStatus && cStatus.connected()) cStatus.println(F("Sending Bubbler Command"));
				if (bubblerSet == 0) sendRS485(bubblerOff, sizeof(bubblerOff));
				else if (bubblerSet == 1) sendRS485(bubblerOn, sizeof(bubblerOn));
			}
			else if (chlorDutySet != chlorDuty) {
				if (cStatus && cStatus.connected()) cStatus.println(F("Sending Intelichlor Command"));
				// change the bit for the percentage
				chlorSetPct[17] = chlorDutySet;
				chlorSetPct[21] = 76 + chlorDutySet;
				sendRS485(chlorSetPct, sizeof(chlorSetPct));
				sendCommand = false;
			}
			else {
				sendCommand = false;
			}
		}
	}

	char c;
	while (Serial1.available()) {
		mqtt.loop();
		c = (uint8_t)Serial1.read();
		
		if (cstream && cstream.connected()) {
			if (c < 10) {
				cstream.print(F("0"));
			}
			cstream.print(c, HEX);
			cstream.print(F(" "));
		}
		
		if (cdebug && cdebug.connected()) {
			cdebug.print(goToCase);
			cdebug.print(" ");
		}
		switch (goToCase) {
		case header1:
			if (c == 0xFF) {                               // ignoring leading FF so do nothing, repeat again
				*bPointer++ = (char)c;
				byteNum = 1;
				if (cdebug && cdebug.connected()) cdebug.println(F("step 2"));
				break;
			}
			else if (c == 0x0) {                                 // is this a 0 in byte 2?  could be Pentair packet
				goToCase = header3;
				*bPointer++ = (char)c;
				byteNum++;
				if (cdebug && cdebug.connected()) cdebug.println(F("step 3"));
				break;
			}
			else { //if (c == 0x10)                              // is this an IntelliChlor header?  could be an IntelliChlor packet
				clear485Bus();
				goToCase = saltHead2;
				*bPointer++ = (char)c;
				byteNum = 1;
				if (cdebug && cdebug.connected()) cdebug.println(F("step 4"));
				break;
			}
			if (cdebug && cdebug.connected()) cdebug.println(F("step 5"));
			break;

		case header3:
			*bPointer++ = (char)c;
			if (c == 0xFF) {                               // it's not really the start of a frame, must be deeper into a Pentair packet
				goToCase = header4;
				byteNum++;
				if (cdebug && cdebug.connected()) cdebug.println(F("step 6"));
				break;
			}
			else {
				clear485Bus();
				goToCase = header1;
				if (cdebug && cdebug.connected()) cdebug.println(F("step 7"));
				break;
			}
			if (cdebug && cdebug.connected()) cdebug.println(F("step 8"));
			break;

		case header4:
			if (c == 0xA5) {                              // it's not really the start of a frame, almost have a Pentair preamble match
				goToCase = bufferData;
				sumOfBytesInChkSum += (byte)c, HEX;
				*bPointerOfBytes++ = (char)c;
				*bPointer++ = (char)c;
				byteNum++;
				if (cdebug && cdebug.connected()) cdebug.println(F("step 9"));
				break;
			}
			else {
				clear485Bus();
				goToCase = header1;
				if (cdebug && cdebug.connected()) cdebug.println(F("step 10"));
				break;
			}
			if (cdebug && cdebug.connected()) cdebug.println(F("step 11"));
			break;

		case bufferData:
			*bPointer++ = (char)c;                             // loop until byte 9 is seen
			*bPointerOfBytes++ = (char)c;                      // add up in the checksum bytes
			byteNum++;
			sumOfBytesInChkSum += (byte)c, HEX;
			if (byteNum == 9) {                              // get data payload length of bytes
				bytesOfDataToGet = (c);
				if (cstatus && cstatus.connected()) {
					cstatus.println();
					cstatus.println();
					cstatus.print(F("\n Free RAM = "));
					cstatus.print(freeRam());
					cstatus.println(F(" <-- watch for memory leaks"));
					cstatus.println();
					cstatus.println();
					//digitalClockDisplay();
					//uptime();
					cstatus.println(F("NEW RS-485 FRAMES RECEIVED"));
					cstatus.print(F("Payload bytes to get... "));
					cstatus.println(bytesOfDataToGet);
				}
				if (bytesOfDataToGet < 0 || bytesOfDataToGet > 47) {  //uh oh.....buffer underflow or buffer overflow... Time to GTFO
					clear485Bus();
					if (cdebug && cdebug.connected()) cdebug.println(F("step 12"));
					break;
				}
				if (remainingBytes == bytesOfDataToGet) {
					goToCase = calcCheckSum;
					if (cdebug && cdebug.connected()) cdebug.println(F("step 13"));
					break;
				}
				remainingBytes++;
				if (cdebug && cdebug.connected()) cdebug.println(F("step 14"));
				break;
			}
			if (byteNum >= 10) {
				if (remainingBytes == bytesOfDataToGet) {
					goToCase = calcCheckSum;
					if (cdebug && cdebug.connected()) cdebug.println(F("step 15"));
					break;
				}
				remainingBytes++;
				if (cdebug && cdebug.connected()) cdebug.println(F("step 16"));
				break;
			}
			if (cdebug && cdebug.connected()) cdebug.println(F("step 17"));
			break;

		case calcCheckSum:
			if (chkSumBits < 2) {
				*bPointer++ = (char)c;
				if (chkSumBits == 0) {
					if (cstatus && cstatus.connected()) {
						cstatus.print(F("Checksum high byte..... "));
						cstatus.println(c, HEX);
					}
					chkSumValue = (c * 256);
					if (cdebug && cdebug.connected()) cdebug.println(F("step 18"));
				}
				else if (chkSumBits == 1) {
					if (cstatus && cstatus.connected()) {
						cstatus.print(F("Checksum low byte...... "));
						cstatus.println((byte)c, HEX);
					}
					goToCase = header1;
					byte len = (byte)(bPointer - buffer);
					chkSumValue += (byte)c;
					processPentair(buffer, len, cstatus);
					clear485Bus();
					if (cdebug && cdebug.connected()) cdebug.println(F("step 19"));
					break;
				}
				chkSumBits++;
				if (cdebug && cdebug.connected()) cdebug.println(F("step 20"));
				break;
			}
			if (cdebug && cdebug.connected()) cdebug.println(F("step 21"));
			break;

		case saltHead2:
			if (c == 0x02) {                                  // is this Intellichlor STX header frame 2 ?
				if (cstatus && cstatus.connected()) {
					cstatus.println();
					cstatus.println();
					cstatus.print(F("\n Free RAM = "));
					cstatus.print(freeRam());
					cstatus.println(F(" <-- watch for memory leaks"));
					cstatus.println();
					cstatus.println();
					// digitalClockDisplay();
					cstatus.println(F("NEW RS-485 IntelliChlor FRAMES RECEIVED"));
				}
				goToCase = bufferSaltData;
				*bPointer++ = (char)c;
				byteNum++;
				if (cdebug && cdebug.connected()) cdebug.println(F("step 22"));
				break;
			}
			else {
				clear485Bus();
				goToCase = header1;
				if (cdebug && cdebug.connected()) cdebug.println(F("step 23"));
				break;
			}
			if (cdebug && cdebug.connected()) cdebug.println(F("step 24"));
			break;

		case bufferSaltData:
			if (c != 0x10) {
				*bPointer++ = (char)c;                          // loop until byte value 0x10 is seen
				byteNum++;
				if (cdebug && cdebug.connected()) cdebug.println(F("step 25"));
				break;
			}
			else {                                            // a ha! found a 0x10, we're close
				goToCase = saltTerm;
				*bPointer++ = (char)c;
				byteNum++;
				if (cdebug && cdebug.connected()) cdebug.println(F("step 26"));
				break;
			}
			if (cdebug && cdebug.connected()) cdebug.println(F("step 27"));
			break;

		case saltTerm:
			*bPointer++ = (char)c;
			byteNum++;
			goToCase = header1;
			if (c != 0x03) {
				clear485Bus();
				if (cdebug && cdebug.connected()) cdebug.println(F("step 28"));
				break;
			}
			else {                                            // found an ETX 0x3.  See what we've got
				byte len = (byte)(bPointer - buffer);
				processIntellichlor(buffer, len, cstatus);
				clear485Bus();
				if (cdebug && cdebug.connected()) cdebug.println(F("step 31"));
				break;
			}
			clear485Bus();
			if (cdebug && cdebug.connected()) cdebug.println(F("step 32"));
			break;
		}															// end switch( goToCase )
	}																// while serial available

	//Update the MQTT broker if a Frame had been received and we are connected!
	if (frameReceived && mqtt.connected()) {
		if (cstatus && cstatus.connected()) cstatus.println(F("************ Frame Received, Sending to MQTT ****************"));
		if (mqttUpdate(cstatus)) frameReceived=false;
	}
}

void clear485Bus() {
	memset(buffer, 0, sizeof(buffer));
	bPointer = buffer;
	memset(bufferOfBytes, 0, sizeof(bufferOfBytes));
	bPointerOfBytes = bufferOfBytes;
	byteNum = 0;
	bytesOfDataToGet = 0;
	remainingBytes = 0;
	chkSumBits = 0;
	sumOfBytesInChkSum = 0;
	chkSumValue = 0;
}

void processPentair(uint8_t* buffer, int len, Adafruit_WINC1500Client client) {
	if (client && client.connected()) {
		client.print(F("Sum of bytes........... "));
		client.println(sumOfBytesInChkSum);
		client.print(F("Check sum is........... "));
		client.println(chkSumValue);
		if (sumOfBytesInChkSum == chkSumValue) {
			client.println(F("Check sum result....... GOOD"));
		}
		else {
			client.println(F("Check sum result....... INVALID"));
		}
		int i = 0;
		//while (i < len) {
		//	printByteData(buffer[i++],client);
		//	client.print(" ");
		//}
		//client.println();
	}
	if (sumOfBytesInChkSum == chkSumValue) {
		if (bufferOfBytes[5] == 0x1D) {                            // 29 byte message is for broadcast display updates 
			oldAirTemp = airTemp;
			airTemp = bufferOfBytes[24];
			poolMode = bufferOfBytes[8];
			featureMode = bufferOfBytes[9];
			panelHour = bufferOfBytes[6];
			panelMinute = bufferOfBytes[7];
			oldPumpState = pumpState;
			oldLightState = lightState;
			oldWaterfallState = waterfallState;
			oldBubblerState = bubblerState;

			if (poolMode == 0x20 || poolMode == 0x24) {
				pumpState = 1;

				// Only update the Water Temperature if the pool is Running
				oldPoolTemp = poolTemp;
				poolTemp = bufferOfBytes[20];
			}
			else {
				pumpState = 0;
			}
			if (poolMode == 0x04 || poolMode == 0x24) {
				lightState = 1;
			}
			else {
				lightState = 0;
			}
			if (featureMode == 0x04 || featureMode == 0x0C) {
				waterfallState = 1;
			}
			else {
				waterfallState = 0;
			}
			if (featureMode == 0x08 || featureMode == 0x0C) {
				bubblerState = 1;
			}
			else {
				bubblerState = 0;
			}

			//Look for uncommanded changes
			if (pumpState != pumpSet) pumpSet = pumpState;
			if (lightState != lightSet) lightSet = lightState;
			if (waterfallState != waterfallSet) waterfallSet = waterfallState;
			if (bubblerState != bubblerSet) bubblerSet = bubblerState;

			// Status Output!
			if	(client && client.connected()) {
				client.print(F("Water Temp............. "));
				client.print(poolTemp);
				client.println(F(" degF"));
				client.print(F("Air Temp............... "));
				client.print(airTemp);
				client.println(F(" degF"));
				client.print(F("Panel time............. "));
				client.print(panelHour);
				client.print(F(":"));
				client.println(panelMinute);
				client.print(F("Circulation............ "));
				client.println(pumpState);
				client.print(F("Light.................. "));
				client.println(lightState);
				client.print(F("Waterfall.............. "));
				client.println(waterfallState);
				client.print(F("Bubbler................ "));
				client.println(bubblerState);
			}
			frameReceived = true;
		}
		else if (bufferOfBytes[2] == 0x10 && bufferOfBytes[3] == 0x60 && bufferOfBytes[5] == 0xF) { // 15 byte message is for pump updates
			oldPumpWatts = pumpWatts;
			oldPumpRPM = pumpRPM;
			pumpWatts = ((bufferOfBytes[9] * 256) + bufferOfBytes[10]); //high bit
			pumpRPM = ((bufferOfBytes[11] * 256) + bufferOfBytes[12]);
			pumpMode = bufferOfBytes[18];
			if (client && client.connected()) {
				client.print(F("Pump Mode.............. "));
				if (pumpMode == 0x1) {
					client.println(F("RUN"));
				}
				else if (pumpMode == 0x0B) {
					client.println(F("PRIMING"));
				}
				else {
					client.println(F("OFF"));
				}
				client.print(F("Pump RPM............... "));
				client.println(pumpRPM);
				client.print(F("Pump Watts............. "));
				client.println(pumpWatts);
			}
			frameReceived = true;
		}
	}
}

void processIntellichlor(uint8_t* buffer, int len, Adafruit_WINC1500Client client) {
	if (len == 8) {
		sumOfBytesInChkSum = buffer[2] + buffer[3] + buffer[4] + 18;
		chkSumValue = buffer[5];
	}
	else if (len == 9) {
		sumOfBytesInChkSum = buffer[2] + buffer[3] + buffer[4] + buffer[5] + 18;
		chkSumValue = buffer[6];
	}

	if (client && client.connected()) {
		client.print(F("Sum of bytes........... "));
		client.println(sumOfBytesInChkSum);
		client.print(F("Check sum is........... "));
		client.println(chkSumValue);
		if (sumOfBytesInChkSum == chkSumValue) {
			client.println(F("Check sum result....... GOOD"));
		}
		else {
			client.println(F("Check sum result....... INVALID"));
		}
		int i = 0;
		//while (i < len) {
		//	printByteData(buffer[i++], client);
		//	client.print(" ");
		//}
		//client.println();
	}
	if (sumOfBytesInChkSum == chkSumValue) {
		if (len == 8 && buffer[4] != 0x0) {
			oldChlorDuty = chlorDuty;
			chlorDuty = buffer[4];
			frameReceived = true;
			if (client && client.connected()) {
				client.print(F("Chlor Setpoint......... "));
				client.print(chlorDuty);
				client.println(F("%"));
			}
			frameReceived = true;
		}
		else if (len == 9 && buffer[4] != 0x0) {
			oldSaltPct = saltPct;
			saltPct = buffer[4] * 50;
			if (client && client.connected()) {
				client.print(F("Pool Salinity.......... "));
				client.println(saltPct);
			}
			frameReceived = true;
		}
	}
}

// routine to take binary numbers and show them as two bytes hex
void printByteData(uint8_t Byte, Adafruit_WINC1500Client client) {
	client.print((uint8_t)Byte >> 4, HEX);
	client.print((uint8_t)Byte & 0x0f, HEX);
}

boolean mqttConnect() {
	if (mqtt.connect("PentairCntrl")) {
		// Once connected, publish an announcement...
		mqtt.publish("outTopic", "hello world");
		// ... and resubscribe
		mqtt.subscribe(mqttSubPump);
		mqtt.subscribe(mqttSubLight);
		mqtt.subscribe(mqttSubWF);
		mqtt.subscribe(mqttSubBub);
		mqtt.subscribe(mqttSubChlor);
		//
	}
	return mqtt.connected();
}

bool mqttUpdate(Adafruit_WINC1500Client client) {
	// See what changed!
	if (poolTemp != oldPoolTemp) {
		oldPoolTemp = poolTemp;
		if (client && client.connected()) {
			client.println(F("Sending Water Temperature Info to MQTT "));
		}
		if (!mqttPubInt(mqttWaterTemp, poolTemp, true, client)) return false;
		else {
			if (client && client.connected()) {
				client.println(F("NOT Sending Water temp: Pump is not running"));
			}
		}
	}
	if (airTemp != oldAirTemp) {
		if (client && client.connected()) {
			client.println(F("Sending Air Temperature Info to MQTT "));
		}
		oldAirTemp = airTemp;
		if (!mqttPubInt(mqttAirTemp, airTemp, true, client)) return false;
	}
	if (pumpState != oldPumpState) {
		if (client && client.connected()) {
			client.println(F("Sending Pump State to MQTT "));
		}
		oldPumpState = pumpState;
		if (!mqttPubInt(mqttPumpState, pumpState, true, client)) return false;
	}
	if (lightState != oldLightState) {
		if (client && client.connected()) {
			client.println(F("Sending Light State to MQTT "));
		}
		oldLightState = lightState;
		if (!mqttPubInt(mqttLightState, lightState, true, client)) return false;
	}
	if (waterfallState != oldWaterfallState) {
		if (client && client.connected()) {
			client.println(F("Sending Waterfall State to MQTT "));
		}
		oldWaterfallState = waterfallState;
		if (!mqttPubInt(mqttWaterfallState, waterfallState, true, client)) return false;
	}
	if (bubblerState != oldBubblerState) {
		if (client && client.connected()) {
			client.println(F("Sending Bubbler State to MQTT "));
		}
		oldBubblerState = bubblerState;
		if (!mqttPubInt(mqttBubblerState, bubblerState, true, client)) return false;
	}
	if (pumpRPM != oldPumpRPM) {
		if (client && client.connected()) {
			client.println(F("Sending Pump RPM State to MQTT "));
		}
		oldPumpRPM = pumpRPM;
		if (!mqttPubInt(mqttPumpRPM, pumpRPM, true, client)) return false;
	}
	if (pumpWatts != oldPumpWatts) {
		if (client && client.connected()) {
			client.println(F("Sending Pump Watts to MQTT "));
		}
		oldPumpWatts = pumpWatts;
		if (!mqttPubInt(mqttPumpWatt, pumpWatts, true, client)) return false;
	}
	if (chlorDuty != oldChlorDuty) {
		if (client && client.connected()) {
			client.println(F("Sending Intellichlor Setpoint to MQTT "));
		}
		oldChlorDuty = chlorDuty;
		if (!mqttPubInt(mqttSaltSet, chlorDuty, true, client)) return false;
	}
	if (saltPct != oldSaltPct) {
		if (client && client.connected()) {
			client.println(F("Sending Pool Salinity to MQTT "));
		}
		oldSaltPct = saltPct;
		if (!mqttPubInt(mqttSaltPct, saltPct, true, client)) return false;
	}
	return true;
}

bool mqttHeartBeat(Adafruit_WINC1500Client client) {			// Send a full update to the MQTT server
	if (!mqttPubInt(mqttWifiStrength, WiFi.RSSI(), true, client)) return false;
	if (pumpState == 1) {				// Only send the pool water temperature if the pump is running
		if (!mqttPubInt(mqttWaterTemp, poolTemp, true, client)) return false;
	}
	if (!mqttPubInt(mqttAirTemp, airTemp, true, client)) return false;
	if (!mqttPubInt(mqttPumpState, pumpState, true, client)) return false;
	if (!mqttPubInt(mqttLightState, lightState, true, client)) return false;
	if (!mqttPubInt(mqttWaterfallState, waterfallState, true, client)) return false;
	if (!mqttPubInt(mqttBubblerState, bubblerState, true, client)) return false;
	if (!mqttPubInt(mqttPumpRPM, pumpRPM, true, client)) return false;
	if (!mqttPubInt(mqttPumpWatt, pumpWatts, true, client)) return false;
	if (!mqttPubInt(mqttSaltSet, chlorDuty, true, client)) return false;
	if (!mqttPubInt(mqttSaltPct, saltPct, true, client)) return false;
	return true;
}

bool mqttPubInt(char* Topic, int intMessage, bool retained, Adafruit_WINC1500Client client) {
	char chrMessage[17];
	itoa(intMessage, chrMessage, 10);
	return mqtt.publish(Topic, chrMessage, retained);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
	// handle message arrived
	if (cStatus && cStatus.connected()) {
		cStatus.print("Message arrived [");
		cStatus.print(topic);
		cStatus.print("] ");
		for (int i = 0; i<length; i++) {
			cStatus.print((char)payload[i]);
		}
		cStatus.println();
	}

	if (strcmp(topic, mqttSubPump) == 0) {
		cStatus.println(payload[0], HEX);
		if (payload[0] == 0x30) {
			if (cStatus && cStatus.connected()) cStatus.println("Setting Pump to: OFF");
			pumpSet = 0;
			if (pumpSet != pumpState) sendCommand = true;
		}
		if (payload[0] == 0x31) {
			if (cStatus && cStatus.connected()) cStatus.println("Setting Pump to: ON");
			pumpSet = 1;
			if (pumpSet != pumpState) sendCommand = true;
		}
	}
	else if (strcmp(topic, mqttSubLight) == 0) {
		cStatus.println(payload[0], HEX);
		if (payload[0] == 0x30) {
			if (cStatus && cStatus.connected()) cStatus.println("Setting Light to: OFF");
			lightSet = 0;
			if (lightSet != lightState) sendCommand = true;
		}
		if (payload[0] == 0x31) {
			if (cStatus && cStatus.connected()) cStatus.println("Setting Light to: ON");
			lightSet = 1;
			if (lightSet != lightState) sendCommand = true;
		}
	}
	else if (strcmp(topic, mqttSubWF) == 0) {
		cStatus.println(payload[0], HEX);
		if (payload[0] == 0x30) {
			if (cStatus && cStatus.connected()) cStatus.println("Setting Waterfall to: OFF");
			waterfallSet = 0;
			if (waterfallSet != waterfallState) sendCommand = true;
		}
		if (payload[0] == 0x31) {
			if (cStatus && cStatus.connected()) cStatus.println("Setting Waterfall to: ON");
			waterfallSet = 1;
			if (waterfallSet != waterfallState) sendCommand = true;
		}
	}
	else if (strcmp(topic, mqttSubBub) == 0) {
		cStatus.println(payload[0], HEX);
		if (payload[0] == 0x30) {
			if (cStatus && cStatus.connected()) cStatus.println("Setting Bubbler to: OFF");
			bubblerSet = 0;
			if (bubblerSet != bubblerState) sendCommand = true;
		}
		if (payload[0] == 0x31) {
			if (cStatus && cStatus.connected()) cStatus.println("Setting Bubbler to: ON");
			bubblerSet = 1;
			if (bubblerSet != bubblerState) sendCommand = true;
		}
	}
	else if (strcmp(topic, mqttSubChlor) == 0) {
		char strChlor[10];
		for (int i = 0; i<length; i++) {
			strChlor[i] = payload[i];
		}
		chlorDutySet = atoi(strChlor);
		if (cStatus && cStatus.connected()) {
			cStatus.print("Setting Chlorinator Setpoint to: ");
			cStatus.println(chlorDutySet);
		}
		if (chlorDutySet != chlorDuty) sendCommand = true;
	}

}

void sendRS485(byte *command, int length) {
	if (cStatus && cStatus.connected()) cStatus.println(F("Sending Command down RS485"));
	digitalWrite(TXControl, RS485Transmit);
	for (byte count = 0; count < 2; count++) {
		for (byte i = 0; i < length; i++) {
			Serial1.write(command[i]);
			cStatus.print(command[i], HEX);
			cStatus.print(" ");
		}
		cStatus.println();
	}
}

void errorLoop() {
	while (1) {
		digitalWrite(status, LOW);
		delay(3000);
		digitalWrite(status, HIGH);
		delay(3000);
	}
}