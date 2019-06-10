#include <ADC.h>

#define TSL1401_SI    25
#define TSL1401_CLK   24
#define TSL1401_OUT   A9

// Stores direct data of one line (16 bit)
uint16_t TSL1401_buf16[132];

// Stores contrast adapted data of one line (8 bit)
uint8_t  TSL1401_buf8[132];

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
    
    // store contrast enhanced 8-bit value
    TSL1401_buf8[i] = val8;
    
    // avoid value 255 for Tx1_buf
    if(val8 == 255) val8 = 254;
    Tx1_buf[i] = val8;
    
    digitalWriteFast(TSL1401_CLK, LOW); 
    delayMicroseconds(1);
  } 
  
  // calculate next average from min and max value
  TSL1401_avgmin = 0.99 * TSL1401_avgmin + 0.01 * (float)TSL1401_valmin; 
  TSL1401_avgmax = 0.99 * TSL1401_avgmax + 0.01 * (float)TSL1401_valmax;
}





void setup() {
  Serial.begin(115200);
  
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


void loop() {
  uint32_t tsDur;
  
  // Exposing is done in interrupt, this flag indicates if exposing is done and
  // picture can be read, reading takes about 1ms
  if(TSL1401_ExposeStopped) {
    tsDur = micros();
    TSL1401_ReadLine();
    TSL1401_PrepareExpose(); 
    
    // insert end mark value and transmit
    Tx1_buf[129] = 255;
    Serial.write(Tx1_buf, 130);

    tsDur = micros() - tsDur;
  }
}

