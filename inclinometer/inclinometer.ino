/*

Arduino Nano - https://www.arduino.cc/en/Main/ArduinoBoardNano
MPU6050 (GY521) - http://playground.arduino.cc/Main/MPU-6050
SA56-21GWA 7 segment common anode - http://www.kingbrightusa.com/product.asp?catalog_name=LED&product_id=SA56-21GWA
1 switch
eagle schematic - http://i.imgur.com/zyByg0g.png



*/



#include <EEPROM.h>
#include <math.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;



#define BMED 20
#define BMIN 3
#define BMAX  30

#define CALIB_PRESS_THR 2000
#define WARN_THR 2
#define LEDCOUNT  8
int ledPins[] = {
  8, 12, 11, 5, 6, 7, 9, 10
};

#define CALIB_TIME 500

int symbols[] = {
  0b11111100,
  0b01100000,
  0b11011010,
  0b11110010,
  0b01100110,
  0b10110110,
  0b10111110,
  0b11100100,
  0b11111110,
  0b11110110,
  0b00000001,

  0b10000000,
  0b01000000,
  0b00100000,
  0b00010000,
  0b00001000,
  0b00000100,
  0b00000010,

  0b10010000,
  0b01001000,
  0b00100100,
  0b10010000,
  0b01001000,
  0b00100100,


  0b00000000,
};



void showDigit(int d, bool dot) {
	d = constrain(d, 0, 24);
	int cd = symbols[d];
	if (dot) {
		cd |= symbols[10];
	}

	for (int i = 0; i < LEDCOUNT; i++) {
		if ((cd & 1 << i) == (1 << i))
			digitalWrite(ledPins[i], LOW);
		else
			digitalWrite(ledPins[i], HIGH);
	}
}


void showCircle(int del) {
	showDigit(18, 0);
	delay(del);
	showDigit(19, 0);
	delay(del);
	showDigit(20, 0);
	delay(del);
	showDigit(21, 0);
	delay(del);
	showDigit(22, 0);
	delay(del);
	showDigit(23, 0);
	delay(del);
}


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            G-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat G;    // [x, y, z]            G vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and G vector

VectorFloat G0;

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}

float deltaFWD = 0;
float deltaLEFT = 0;

void setup() {


	Wire.begin();
	mpu.initialize();
	devStatus = mpu.dmpInitialize();
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		dmpReady = true;
		packetSize = mpu.dmpGetFIFOPacketSize();
	}
	else {
		// ERROR!

	}

	for (int i = 0; i < LEDCOUNT; i++) {
		pinMode(ledPins[i], OUTPUT);
	}
	pinMode(3, OUTPUT);
	digitalWrite(3, LOW);
	pinMode(A0, INPUT_PULLUP);



	//deltaFWD = (float)EEPROM.read(0);
	//deltaLEFT = (float)EEPROM.read(1);
	EEPROM.get(0, deltaFWD);
	EEPROM.get(16, deltaLEFT);

	showDigit(4, 0);
#ifdef DBG
	Serial.begin(115200);
	Serial.print("deltaFWD  ");
	Serial.println(deltaFWD);
	Serial.print("deltaLEFT  ");
	Serial.println(deltaLEFT);
#endif
	showCircle(80);
	showDigit(24, 0);
	delay(500);
}





int value = 0;
bool pulse = false;
int bb = 255;
int bs = 10;
int bt = 0;

float angFWD = 0;
float angLEFT = 0;

bool calib = false;
int tCalib = 0;
int calibNum = 0;
void loop() {

	if (!dmpReady) return;

	// wait for MPU interrupt or extra packet(s) available
	while (!mpuInterrupt && fifoCount < packetSize) {
	}

	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		// reset so we can continue cleanly
		mpu.resetFIFO();
#ifdef DBG
		//    Serial.println(F("FIFO overflow!"));
#endif
// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}
	else if (mpuIntStatus & 0x01) {
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;

		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&G, &q);

	}

	doStuff();
}

int prsTime = 0;
bool pressed = false;
float mang = 0;
void doStuff() {

	angFWD = (atan2(G.x, G.z) * 180 / M_PI);
	angLEFT = (atan2(G.y, G.z) * 180 / M_PI);

	mang = max(abs(angFWD - deltaFWD), abs(angLEFT - deltaLEFT));
#ifdef DBG
	Serial.print(angFWD); Serial.print("\t");
	Serial.print(angLEFT); Serial.print("\t");
	Serial.print(mang); Serial.print("\t");
	Serial.println();
#endif
	value = constrain(map(abs(mang), 0, 60, 0, 9), 0, 9);


	if (value > WARN_THR) {
		pulse = true;
	}
	else {
		pulse = false;
	}


	int ar = analogRead(A0);
	if (!pressed && ar < 500 && prsTime == 0) {
		prsTime = millis();
	}
	if (ar > 500) {
		prsTime = 0;
		pressed = false;
	}

	if (ar < 500 && millis() - prsTime > CALIB_PRESS_THR) {
		pressed = true;
	}

	if (!calib &&  pressed) {
		calib = true;
		calibNum = 0;
		tCalib = millis();
	}
	if (calib) {
		if (millis() - tCalib > CALIB_TIME) {
			calib = false;
			tCalib = 0;
		}

		deltaFWD = (deltaFWD * calibNum + angFWD) / (++calibNum);
		deltaLEFT = (deltaLEFT * calibNum + angLEFT) / (++calibNum);
#ifdef DBG
		Serial.print(angFWD);
		Serial.print("\t");
		Serial.print(deltaFWD);
		Serial.print("\t");
		Serial.println(deltaLEFT);
#endif
		//EEPROM.write(0, (byte)(int)deltaFWD);
		//EEPROM.write(1, (byte)(int)deltaLEFT);

		EEPROM.put(0, deltaFWD);
		EEPROM.put(16, deltaLEFT);

	}

	int mm = millis();
	if (pulse) {
		if (mm - bt > 100) {
			bt = mm;
			if (bb == BMAX)
				bb = BMIN;
			else
				bb = BMAX;
		}
	}
	else {
		bb = map(value, 0, WARN_THR, BMIN, BMED);
	}
	analogWrite(3, 255 - bb);
#ifdef DBG
	//  Serial.println(value);
#endif
	showDigit(value, value > WARN_THR ? 1 : 0);


}







