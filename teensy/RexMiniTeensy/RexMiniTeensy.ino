/**
	Purpose: basic closed loop motor controller

	@author Shintaro Fujita
	@version 0.3 2017-4-20
*/

#include <SerialCommand.h>
#include <Encoder.h>

Encoder encoderRight(33, 34);
Encoder encoderLeft(35, 36);

long encoderRightCount, encoderLeftCount;

SerialCommand SCmd;

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


struct PidData pidRight;
struct PidData pidLeft;

unsigned long last_t = 0L;
float e_sum = 0.0;
float e_last = 0.0;

float targetPos = 0.0;

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

	targetPos = 0.0;

	pidRight.DeltaTimeFactor = 0.02; // 0.02;
	pidRight.Kp = 1.2;
	pidRight.Ki = 0.01;
	pidRight.Kd = 0.01;
	pidRight.esum = 0.1;

	pidLeft.DeltaTimeFactor = 0.02; // 0.02;
	pidLeft.Kp = 1.2;
	pidLeft.Ki = 0.01;
	pidLeft.Kd = 0.01;
	pidLeft.esum = 0.1;

	SCmd.addCommand("MLGOTO",set_motor_left_goto);

	Serial1.println("ready");
}

void set_motor_left_goto() {

  int aNumber;  
  char *arg; 

  arg = SCmd.next(); 
  if (arg != NULL) 
  {
    aNumber=atoi(arg);    // Converts a char string to an integer
    targetPos = (float)aNumber;
    Serial1.println("MLDONE");
  } 
}

int debugSerialCounter = 0;
int pidExcutionCounter = 0;

void loop() {
	encoderRightCount = encoderRight.read();
	encoderLeftCount = -encoderLeft.read();

	debugSerialCounter++;
	pidExcutionCounter++;

	if(debugSerialCounter>1000) {
		debugSerialCounter = 0;

		/*
		Serial1.print("ER: ");
		Serial1.print(encoderRightCount);
		Serial1.print(" EL: ");
		Serial1.println(encoderLeftCount);
		*/
	}

	if(pidExcutionCounter>100) {
		pidExcutionCounter = 0;

		pidRight.setpoint = targetPos;
		pidRight.measured = (float)encoderRightCount;

		pidLeft.setpoint = targetPos;
		pidLeft.measured = (float)encoderLeftCount;

		pid(&pidRight);
		pid(&pidLeft);

		int pwmRight = (int)(pidRight.correction);
		int pwmLeft = (int)(pidLeft.correction);

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
}


void setMotorPWMLeft(int val) {
	analogWrite(17, val);
}

void setMotorPWMRight(int val) {
	analogWrite(16, val);
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