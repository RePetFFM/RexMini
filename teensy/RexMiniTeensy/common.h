#include "Arduino.h"
#include <EEPROM.h>
#include <avr/io.h>
#include <stdio.h>
#include "serialParser.h"

void clearMotorRevolution();
void setCurvefactor(float val);
void setForwardSpeed(float val);
void setTarget(char pidid, float val);
void setPidSettings_dt(char pidid, float val);
void setPidSettings_P(char pidid, float val);
void setPidSettings_I(char pidid, float val);
void setPidSettings_D(char pidid, float val);
void setPidSettings_ESum(char pidid, float val);
