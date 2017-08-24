/**
	Purpose: basic closed loop motor controller

	@author Shintaro Fujita
	@version 0.3 2017-4-20
*/

#include "common.h"
#include <Encoder.h>

Encoder encoderRight(33, 34);
Encoder encoderLeft(35, 36);

long encoderRightCount, encoderLeftCount;

char debugMode = 0;

struct pidSetting{
	float p;
	float i;
	float d;
	float dt;
	float esum;
};

struct PidData {
	float setpoint;
	float measured;
	float Kp;
	float Ki;
	float Kd;
	float DeltaTimeFactor;
	float error;
	float p;
	float i;
	float d;
	float correction;
	float esum;
	float e_sum;
	float e_last;
};

struct pidSetting pidSettings[2];

struct PidData pidDatas[2];

unsigned long last_t = 0L;


char adr = 0xFF;
long counter;


void setup() {
	analogWrite(16, 0);
	analogWrite(17, 0);
	analogWriteResolution(11);
	analogWriteFrequency(16, 29296.875);
	analogWriteFrequency(17, 29296.875);
	
	
	setMotorPWMRight(0);
	setMotorPWMLeft(0);

	pinMode(15, OUTPUT);
	pinMode(14, OUTPUT);

	Serial1.begin(115200); 

	// EEPROM.put(0,pidSettings);
	EEPROM.get(0,pidSettings);


	pidDatas[0].DeltaTimeFactor = 0.02; // 0.02;
	pidDatas[0].Kp = pidSettings[0].p;
	pidDatas[0].Ki = pidSettings[0].i;
	pidDatas[0].Kd = pidSettings[0].d;
	pidDatas[0].esum = pidSettings[0].esum;
	pidDatas[0].setpoint = 0.0;

	pidDatas[1].DeltaTimeFactor = 0.02; // 0.02;
	pidDatas[1].Kp = pidSettings[1].p;
	pidDatas[1].Ki = pidSettings[1].i;
	pidDatas[1].Kd = pidSettings[1].d;
	pidDatas[1].esum = pidSettings[1].esum;
	pidDatas[1].setpoint = 0.0;

	Serial1.println("ready");
}



int debugSerialCounter = 0;
int pidExcutionCounter = 0;

void loop() {
	
	encoderRightCount = encoderRight.read();
	encoderLeftCount = -encoderLeft.read();

	debugSerialCounter++;
	pidExcutionCounter++;

	if(debugSerialCounter>20000) {
		debugSerialCounter = 0;

		switch(debugMode) {
			case '0':
			break;
			case '1':
				debugPID(0);
			break;
			case '2':
				debugPID(1);
			break;
		}
	}

	if(pidExcutionCounter>100) {
		pidExcutionCounter = 0;

		pidDatas[0].measured = (float)encoderRightCount;

		pidDatas[1].measured = (float)encoderLeftCount;

		pid(0);

		pid(1);

		int pwmRight = (int)(pidDatas[0].correction);
		int pwmLeft = (int)(pidDatas[1].correction);

		pwmRight<0 ? digitalWrite(15, HIGH) : digitalWrite(15, LOW);
		pwmLeft<0 ? digitalWrite(14, HIGH) : digitalWrite(14, LOW);
		
		pwmRight = abs(pwmRight);
		pwmLeft = abs(pwmLeft);

		if(pwmRight>1000) { // prevent bullshit value ;)
			pwmRight = 1000;
		}

		if(pwmLeft>1000) { // prevent bullshit value ;)
			pwmLeft = 1000;
		}
			
		setMotorPWMRight(pwmRight);
		setMotorPWMLeft(pwmLeft);

	}

	serialParser();
}

void serialParser() {
	
	uint8_t c;

	
	while(Serial1.available()) {

		c = Serial1.read();
		serialCMDParser(c);
		
	}
}


void setTarget(char pidid, float val) {
	pidDatas[pidid].setpoint = val; // 1.2;
}

void setPidSettings_dt(char pidid, float val) {
	pidDatas[pidid].DeltaTimeFactor = val; // 1.2;
	updateEEPROMPID();
}

void setPidSettings_P(char pidid, float val) {
	pidDatas[pidid].Kp = val; // 1.2;
	updateEEPROMPID();
}

void setPidSettings_I(char pidid, float val) {
	pidDatas[pidid].Ki = val; // 0.01;
	updateEEPROMPID();
}

void setPidSettings_D(char pidid, float val) {
	pidDatas[pidid].Kd = val; // 0.01;
	updateEEPROMPID();
}

void setPidSettings_ESum(char pidid, float val) {
	pidDatas[pidid].esum = val; // 0.01;
	updateEEPROMPID();
}


void setMotorPWMLeft(int val) {
	analogWrite(16, val);
}

void setMotorPWMRight(int val) {
	analogWrite(17, val);
}

void updateEEPROMPID(){
	pidSettings[0].p = pidDatas[0].Kp;
	pidSettings[0].i = pidDatas[0].Ki;
	pidSettings[0].d = pidDatas[0].Kd;
	pidSettings[0].esum = pidDatas[0].esum;
	pidSettings[0].dt = pidDatas[0].DeltaTimeFactor;

	pidSettings[1].p = pidDatas[1].Kp;
	pidSettings[1].i = pidDatas[1].Ki;
	pidSettings[1].d = pidDatas[1].Kd;
	pidSettings[1].esum = pidDatas[1].esum;
	pidSettings[1].dt = pidDatas[1].DeltaTimeFactor;

	EEPROM.put(0, pidSettings);
}

void pid(char id) {

	/*
	unsigned long t = millis();
	unsigned long dt = t - last_t;
	last_t = t;
	*/

	// float _Ta = (float)dt*pidDatas[id].DeltaTimeFactor;

	// id = 0;

	pidDatas[id].error = pidDatas[id].setpoint - pidDatas[id].measured;			//Vergleich

	pidDatas[id].e_sum = (pidDatas[id].e_sum + pidDatas[id].error);				//Integration I-Anteil

	if(pidDatas[id].e_sum>pidDatas[id].esum) pidDatas[id].e_sum = pidDatas[id].esum;
	if(pidDatas[id].e_sum<-pidDatas[id].esum) pidDatas[id].e_sum = -pidDatas[id].esum;

	pidDatas[id].p = pidDatas[id].Kp * pidDatas[id].error;
	
	pidDatas[id].i = pidDatas[id].Ki * pidDatas[id].e_sum;
	pidDatas[id].d = pidDatas[id].Kd * (pidDatas[id].error - pidDatas[id].e_last);
	
	pidDatas[id].correction = pidDatas[id].p + pidDatas[id].i + pidDatas[id].d;	//Reglergleichung
	
	pidDatas[id].e_last = pidDatas[id].error;
}

void debugPID(char pidid) {
	Serial1.print(pidDatas[pidid].measured); // error
	Serial1.print(" ");
	Serial1.print(pidDatas[pidid].setpoint); // error
	Serial1.print(" ");
	Serial1.print(pidDatas[pidid].p);
	Serial1.print(" ");
	Serial1.print(pidDatas[pidid].i);
	Serial1.print(" ");
	Serial1.print(pidDatas[pidid].d);
	Serial1.print(" ");
	Serial1.println(pidDatas[pidid].correction);
}