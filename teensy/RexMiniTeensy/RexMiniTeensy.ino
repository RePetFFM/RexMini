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
};

struct pidSetting pidSettings[2];

struct PidData pidDatas[2];

unsigned long last_t = 0L;
float e_sum = 0.0;
float e_last = 0.0;

float targetPos = 0.0;

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

	targetPos = 0.0;

	pidDatas[0].DeltaTimeFactor = 0.02; // 0.02;
	pidDatas[0].Kp = 1.2;
	pidDatas[0].Ki = 0.01;
	pidDatas[0].Kd = 0.01;
	pidDatas[0].esum = 0.1;

	pidDatas[1].DeltaTimeFactor = 0.02; // 0.02;
	pidDatas[1].Kp = 1.2;
	pidDatas[1].Ki = 0.01;
	pidDatas[1].Kd = 0.01;
	pidDatas[1].esum = 0.1;

	Serial1.println("ready");
}



int debugSerialCounter = 0;
int pidExcutionCounter = 0;

void loop() {
	
	encoderRightCount = encoderRight.read();
	encoderLeftCount = -encoderLeft.read();

	debugSerialCounter++;
	pidExcutionCounter++;

	if(debugSerialCounter>10000) {
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

		/*
		Serial1.print("ER: ");
		Serial1.print(encoderRightCount);
		Serial1.print(" EL: ");
		Serial1.println(encoderLeftCount);
		*/
	}

	if(pidExcutionCounter>100) {
		pidExcutionCounter = 0;

		// pidDatas[0].setpoint = targetPos;
		pidDatas[0].measured = (float)encoderRightCount;

		// pidDatas[1].setpoint = targetPos;
		pidDatas[1].measured = (float)encoderLeftCount;

		pid(&pidDatas[0]);
		pid(&pidDatas[1]);

		int pwmRight = (int)(pidDatas[0].correction);
		int pwmLeft = (int)(pidDatas[1].correction);

		/*
		Serial1.print("R: ");
		Serial1.print(pwmRight);
		Serial1.print(" L: ");
		Serial1.println(pwmLeft);
		*/

		pwmRight = abs(pwmRight);
		pwmLeft = abs(pwmLeft);

		if(pwmRight>1000) { // prevent bullshit value ;)
			pwmRight = 1000;
		}

		if(pwmLeft>1000) { // prevent bullshit value ;)
			pwmLeft = 1000;
		}

		pwmRight<0 ? digitalWrite(14, HIGH) : digitalWrite(14, LOW);
		pwmLeft<0 ? digitalWrite(15, HIGH) : digitalWrite(15, LOW);
			
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
	analogWrite(17, val);
}

void setMotorPWMRight(int val) {
	analogWrite(16, val);
}

void updateEEPROMPID(){
	pidSettings[0].p = pidDatas[0].p;
	pidSettings[0].i = pidDatas[0].i;
	pidSettings[0].d = pidDatas[0].d;
	pidSettings[0].esum = pidDatas[0].esum;
	pidSettings[0].dt = pidDatas[0].DeltaTimeFactor;

	pidSettings[1].p = pidDatas[1].p;
	pidSettings[1].i = pidDatas[1].i;
	pidSettings[1].d = pidDatas[1].d;
	pidSettings[1].esum = pidDatas[1].esum;
	pidSettings[1].dt = pidDatas[1].DeltaTimeFactor;

	EEPROM.put(0, pidSettings);
}

void pid(struct PidData *pidTmp) {

	unsigned long t = millis();
	unsigned long dt = t - last_t;
	last_t = t;

	// float _Ta = (float)dt*pidTmp->DeltaTimeFactor;

	pidTmp->error = pidTmp->setpoint - pidTmp->measured;			//Vergleich

	e_sum = (e_sum + pidTmp->error);				//Integration I-Anteil

	if(e_sum>pidTmp->esum) e_sum = pidTmp->esum;
	if(e_sum<-pidTmp->esum) e_sum = -pidTmp->esum;

	pidTmp->p = pidTmp->Kp * pidTmp->error;
	
	pidTmp->i = pidTmp->Ki * e_sum;
	pidTmp->d = pidTmp->Kd * (pidTmp->error - e_last);
	
	pidTmp->correction = pidTmp->p + pidTmp->i + pidTmp->d;	//Reglergleichung
	
	e_last = pidTmp->error;
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