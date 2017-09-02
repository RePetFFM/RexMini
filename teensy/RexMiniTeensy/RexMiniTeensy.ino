/**
	Purpose: basic closed loop motor controller

	@author Shintaro Fujita
	@version 0.3 2017-4-20
*/



#include "common.h"
#include <Encoder.h>


/*
begin: line ccd variables
*/
#include <ADC.h>

#define TSL1401_SI    25
#define TSL1401_CLK   24
#define TSL1401_OUT   A9

// Stores direct data of one line (16 bit)
uint16_t TSL1401_buf16[132];

// Stores contrast adapted data of one line (8 bit)
uint8_t  TSL1401_buf8[132];
uint8_t  TSL1401_buf8_last[132];
uint8_t  TSL1401_buf8_delta[132];

uint16_t TSL1401_valmax;
uint16_t TSL1401_valmin;
float    TSL1401_avgmax;
float    TSL1401_avgmin;

// Stores processed data of one line
uint8_t  Tx1_buf[132];


volatile uint8_t TSL1401_ExposeStarted;
volatile uint8_t TSL1401_ExposeStopped;

IntervalTimer TSL1401_TimerStartExpose;
IntervalTimer TSL1401_TimerStopExpose;
ADC *adc = new ADC(); // adc object;


/*
end: line ccd variables
*/


#define BAT_VOLTAGE	A8
char robotMode = 0;

Encoder encoderRight(33, 34);
Encoder encoderLeft(35, 36);

long encoderRightCount, encoderLeftCount;

char debugMode = 0;

uint8_t center_scan_area = 10;
uint8_t centerline_center;
uint8_t centerline_width;
uint8_t centerline_known_width = 2;
uint8_t centerline_width_tolarance = 2;
uint8_t centerline_detected = 0;

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

float forwardSpeed = 0.0;
float curveSpeed = 100.0;


void setup() {
	analogWrite(16, 0);
	analogWrite(17, 0);
	analogWriteResolution(11);
	analogWriteFrequency(16, 29296.875);
	analogWriteFrequency(17, 29296.875);

	pinMode(BAT_VOLTAGE, INPUT);
	
	
	setMotorPWMRight(0);
	setMotorPWMLeft(0);

	pinMode(15, OUTPUT);
	pinMode(14, OUTPUT);

	// Serial.begin(115200);
	Serial1.begin(230400);  // 115200

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

	Serial1.println("ok");

	TSL1401_setup();
}




uint32_t microsPid;
uint32_t microsDebugOutput;
bool motorControlEnabled = false;

void loop() {

	robotMode_loop();

	if(motorControlEnabled) {
		motorPID_loop();	
	}

	if(robotMode==2) {
		followLine();
	}

	serialParser();

	TSL1401_loop();

	debugSerialOut_loop();

	serialOut_state_loop();
}

void debugSerialOut_loop() {
	if(microsDebugOutput<micros()) {
		microsDebugOutput = micros() + 10000;

		switch(debugMode) {
			case '0':
			break;
			case '1':
			serialOut_PID(0);
			break;
			case '2':
			serialOut_PID(1);
			break;
		}
	}
}

void motorPID_loop() {

	encoderRightCount = encoderRight.read();
	encoderLeftCount = -encoderLeft.read();

	if(microsPid<micros()) {
		microsPid = micros()+100;

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
}

uint8_t currentRobotMode = 0;

void robotMode_loop() {
	if(currentRobotMode!=robotMode) {
		switch (robotMode) {
			case 0:
				// standby mode
				clearMotorRevolution();
				motorHalt();
				motorControlEnabled = false;
				
			  break;
			case 1:
			  // do something
				clearMotorRevolution();
				motorHalt();
				motorControlEnabled = false;
				centerline_known_width = centerline_width;
				robotMode = 0;
			  break;

			case 2:
			  // do something
				clearMotorRevolution();
				motorControlEnabled = true;
				
			  break;
			case 3:
			  // do something
				// clearMotorRevolution();
				motorControlEnabled = true;
			  break;
			default:
			  // do something
				clearMotorRevolution();
				motorHalt();
		}
		currentRobotMode = robotMode;
	}
	
	
}

void motorHalt() {
	setMotorPWMRight(0);
	setMotorPWMLeft(0);	
}

void serialParser() {
	
	uint8_t c;

	
	while(Serial1.available()) {

		c = Serial1.read();
		serialCMDParser(c);
		
	}
}

void clearMotorRevolution() {
	pidDatas[0].setpoint = 0;
	pidDatas[0].measured = 0;
	encoderRight.write(0);
	encoderRightCount = 0;

	pidDatas[1].setpoint = 0;
	pidDatas[1].measured = 0;
	encoderLeft.write(0);
	encoderLeftCount = 0;
}

void setCurvefactor(float val) {
	curveSpeed<5 ? curveSpeed = 5 : false;
	curveSpeed = val; // 1.2;
}

void setForwardSpeed(float val) {
	forwardSpeed = val; // 1.2;
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

void serialOut_PID(char pidid) {
	Serial1.print("dp");
	Serial1.print(" ");
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

uint32_t millisSerialOut_state;

void serialOut_state_loop() {

	if(millisSerialOut_state<millis()) {
		millisSerialOut_state = 1000+millis();
		Serial1.print("0s");
		Serial1.print(robotMode,DEC);
		Serial1.print(" ");
		Serial1.print(analogRead(BAT_VOLTAGE));
		Serial1.println();	
	}

	
}

float motorRightRev = 0.0;
float motorLeftRev = 0.0;

void stear(int stearing) {
	float stearing_float = (float)stearing;
	float curve;
	motorRightRev += forwardSpeed;
	motorLeftRev += forwardSpeed;

	curve = (stearing_float*stearing_float*stearing_float)/600;

	motorRightRev += curve/curveSpeed;
	motorLeftRev += -curve/curveSpeed;

	setTarget(1,motorRightRev);
	setTarget(0,motorLeftRev);
	// setpointLeft(Math.round(motorRightRev));
	// setpointRight(Math.round(motorLeftRev));
}



void detectLine() {
	uint8_t prevValue = 255;
	uint8_t centerline_left_in = 130;
	uint8_t centerline_right_in = 0;

	for(uint8_t i=64-center_scan_area; i<64+center_scan_area; i++) {
		Tx1_buf[i]<prevValue ? centerline_left_in = i : false;
		prevValue = Tx1_buf[i];
	}

	prevValue = 255;

	for(uint8_t i=64+center_scan_area; i>64-center_scan_area; i--) {
		Tx1_buf[i]<prevValue ? centerline_right_in = i : false;
		prevValue = Tx1_buf[i];
	}
	centerline_width = centerline_right_in - centerline_left_in;
	centerline_center = centerline_left_in + (centerline_width/2);

	uint8_t minTolarance;
	centerline_known_width-centerline_width_tolarance<1 ? minTolarance = 1 : minTolarance = centerline_known_width-centerline_width_tolarance;

	if(centerline_width>=minTolarance && centerline_known_width+centerline_width_tolarance>centerline_width) {
		centerline_detected = 1;
	} else {
		centerline_detected = 0;
	}


	
}

uint32_t millisFollowLine;

void followLine() {
	if(centerline_detected==1 && millisFollowLine<millis()) {
		millisFollowLine = millis() + 10;
		forwardSpeed = 2;

		stear(centerline_center-64);
	}
}

/*
begin: line ccd code
*/


void TSL1401_setup() {
	// Serial.begin(115200);

	pinMode(TSL1401_SI, OUTPUT);
	pinMode(TSL1401_CLK, OUTPUT);
	pinMode(TSL1401_OUT, INPUT);

	adc->setReference(ADC_REFERENCE::REF_1V2, ADC_0);
	adc->setAveraging(0, ADC_0); // set number of averages 0,4,8,16,32
	adc->setResolution(16, ADC_0); // set bits of resolution

	// it can be any of the ADC_CONVERSION_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED_16BITS, HIGH_SPEED or VERY_HIGH_SPEED
	// see the documentation for more information
	// additionally the conversion speed can also be ADACK_2_4, ADACK_4_0, ADACK_5_2 and ADACK_6_2,
	// where the numbers are the frequency of the ADC clock in MHz and are independent on the bus speed.
	adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED_16BITS); // change the conversion speed
	// it can be any of the ADC_MED_SPEED enum: VERY_LOW_SPEED, LOW_SPEED, MED_SPEED, HIGH_SPEED or VERY_HIGH_SPEED
	adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed  

	// adc->enablePGA(1, ADC_0); // Gain 1, 2, 4, 8, 16, 32 or 64
	//  adc->printError();
	TSL1401_Init(2000, 20000ul);
}


uint32_t serialOutInterval;

void TSL1401_loop() {
	uint32_t tsDur;
	
	int tmpInt;
	char tmp[2];

	// Exposing is done in interrupt, this flag indicates if exposing is done and
	// picture can be read, reading takes about 1ms
	if(TSL1401_ExposeStopped) {
		tsDur = micros();
		TSL1401_ReadLine();
		TSL1401_PrepareExpose(); 

		if(serialOutInterval<millis()) {
			serialOutInterval = 100+millis();
			Serial1.print("cl");
			for(uint8_t i=0; i<130; i++) {
				sprintf(tmp,"%02X", Tx1_buf[i]);
				Serial1.print(tmp);	
			}
			Serial1.print(" ");
			Serial1.print(center_scan_area);

			detectLine();

			Serial1.print(" ");
			Serial1.print(centerline_center);

			Serial1.print(" ");
			Serial1.print(centerline_width);

			Serial1.print(" ");
			Serial1.print(centerline_detected);

			Serial1.print(" ");
			Serial1.print(centerline_known_width);

			
			
			Serial1.println();	
		}
		
		tsDur = micros() - tsDur;
	}
}





// Step 0: Initialize timers
void TSL1401_Init(int32_t ExposeDuration, int32_t ExposePeriod) {
	uint32_t ts;
	TSL1401_TimerStartExpose.end();
	TSL1401_TimerStopExpose.end();

	digitalWrite(TSL1401_SI, LOW);
	digitalWrite(TSL1401_CLK, LOW);

	// Clkin 170 more clks
	for(int i=0;i<170;i++) {
		digitalWrite(TSL1401_CLK, HIGH);
		digitalWrite(TSL1401_CLK, LOW);    
	}

	TSL1401_PrepareExpose();
	ts = micros();
	TSL1401_TimerStartExpose.begin(TSL1401_StartExpose, ExposePeriod);
	while((uint32_t)((uint32_t)micros() - (uint32_t)ts) < (uint32_t)ExposeDuration) ;
	TSL1401_TimerStopExpose.begin(TSL1401_StopExpose, ExposePeriod);
}

// Step 1
void TSL1401_PrepareExpose() {
	// Clkin SI signal
	digitalWriteFast(TSL1401_CLK, LOW);
	delayMicroseconds(1);
	digitalWriteFast(TSL1401_SI, HIGH);
	delayMicroseconds(1);
	digitalWriteFast(TSL1401_CLK, HIGH);
	delayMicroseconds(1);
	digitalWriteFast(TSL1401_SI, LOW);
	delayMicroseconds(1);
	digitalWriteFast(TSL1401_CLK, LOW);
	delayMicroseconds(1);

	// Clkin 17 more clks
	for(int i=0;i<17;i++) {
		digitalWriteFast(TSL1401_CLK, HIGH);
		delayMicroseconds(1);
		digitalWriteFast(TSL1401_CLK, LOW);  
		delayMicroseconds(1);  
	}  

	noInterrupts();
	TSL1401_ExposeStarted = 0;
	TSL1401_ExposeStopped = 0;
	interrupts();
}

// Step 2: Triggered by interrupt
void TSL1401_StartExpose() {
	if(TSL1401_ExposeStarted == 0) {
		TSL1401_ExposeStarted = 1;
		// Clkin 115 more clks
		for(int i=0;i<115;i++) {
			digitalWriteFast(TSL1401_CLK, HIGH);
			delayMicroseconds(1);
			digitalWriteFast(TSL1401_CLK, LOW);  
			delayMicroseconds(1);  
		}
	}
}

// Step 3: Triggered by interrupt 
void TSL1401_StopExpose() {
	if(TSL1401_ExposeStopped == 0) {
		// Clkin SI signal
		digitalWriteFast(TSL1401_SI, HIGH);
		delayMicroseconds(1);
		digitalWriteFast(TSL1401_CLK, HIGH);
		delayMicroseconds(1);
		digitalWriteFast(TSL1401_SI, LOW);
		delayMicroseconds(1);
		digitalWriteFast(TSL1401_CLK, LOW); 
		delayMicroseconds(1);
		TSL1401_ExposeStopped = 1; 
	}
}

// Step 4: can be started in main loop if TSL1401_ExposeStopped == 1
void TSL1401_ReadLine() {
	uint16_t valadc;
	int16_t  valmapped;
	uint8_t  val8;

	uint8_t delta8;


	adc->startSingleRead(TSL1401_OUT);
	TSL1401_valmax = 0;
	TSL1401_valmin = 65535u;

	// Read analog
	for(int i=0;i<130;i++) {
		while(!adc->isComplete()) ;

		// trigger ADC conversion for next cycle, ADC is running in background 
		// while remaining part of pixel calculation is done
		valadc = adc->readSingle();
		digitalWriteFast(TSL1401_CLK, HIGH);
		delayMicroseconds(1);
		adc->startSingleRead(TSL1401_OUT);

		// store 16-bit value directly
		TSL1401_buf16[i] = valadc;

		// learn max and min value
		if(valadc > TSL1401_valmax) TSL1401_valmax = valadc;
		if(valadc < TSL1401_valmin) TSL1401_valmin = valadc;

		// map to avgmax and avgmin, clip values to 8-bit
		valmapped = map(valadc,TSL1401_avgmin,TSL1401_avgmax,0,255);
		if(valmapped > 255) valmapped = 255;
		if(valmapped < 0) valmapped = 0;
		val8 = valmapped;

		TSL1401_buf8[i] = val8;

		// avoid value 255 for Tx1_buf
		
		if(val8 == 255) val8 = 254;

		val8<100 ? Tx1_buf[i] = 0 : Tx1_buf[i] = 254;
		
		digitalWriteFast(TSL1401_CLK, LOW); 
		delayMicroseconds(1);
	}
	
	// calculate next average from min and max value
	TSL1401_avgmin = 0.99 * TSL1401_avgmin + 0.01 * (float)TSL1401_valmin; 
	TSL1401_avgmax = 0.99 * TSL1401_avgmax + 0.01 * (float)TSL1401_valmax;
}


/*
end: line ccd code
*/



