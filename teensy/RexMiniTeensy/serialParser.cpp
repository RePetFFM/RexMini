#include "common.h"
#include "serialParser.h"


void serialPrintHelp() {
	Serial1.println(F("Available commands"));
	Serial1.println(F("Basic syntax: ##C[*]"));
	Serial1.println(F("## = adress(Hex), C = command, [*] = optional command and/or value(s)"));
	Serial1.println(F("reserved adresses are 00 and FF. 00 = broadcast, FF = default devices adress if not defined."));
	Serial1.println(F("line feed to commit the command"));
	Serial1.println(F("-------------------------------"));
	Serial1.println(F("cmd ? = this help"));
	Serial1.println(F("cmd s = set"));
	Serial1.println(F("cmd r = request"));
	Serial1.println(F("cmd e = execute"));
}

char readSingleHex(char c) {
	char cret = 0x00;
	if(c>='A' && c<='F') {
		cret = c-'A'+10;
	}
	if(c>='a' && c<='f') {
		cret = c-'a'+10;
	}
	if(c>='0' && c<='9') {
		cret = c-'0';
	}
	return cret;

}

void blockingBroadcastDelay() {
	uint8_t d = adr+1;
	delay(adr);
}

void toggleDebugMode() {
	/*
	if(pidDebugMode==0) {
		pidDebugMode = 1;
	} else {
		pidDebugMode = 0;
	}
	Serial1.println("toggle debug");
	Serial1.println(pidDebugMode);
	*/ 
}

void serialParserRequest(char adrReq, char * buf,uint8_t cnt) {
	cnt-=4;
	switch (buf[0]) {
		case 'a':
			if(adrReq==0x00) {
				blockingBroadcastDelay();
				char adrHex[2];
				sprintf(adrHex,"%02X",adr);
				Serial1.println(adrHex);
			}
		break;
		
		
	}
}

void serialParserSet(char adrReq, char * buf,uint8_t cnt) {
	float tmpFloat;
	long tmpLong = 0L;
	cnt-=4;
	switch (buf[0]) {
		case 'g': // goto
			{
				buf[0] = ' ';
				// int tmpa = sscanf((const char*)&buf[1],"%d",&tmpLong);
				tmpLong = atoi((const char*)&buf[1]);
				if((float)tmpLong!=targetPos) {
					setTarget(adrReq,(float)tmpLong);
				}	
			}
		break;
		case 'p': // pid propotional value
			{
				buf[0] = ' ';
				tmpLong = atoi((const char*)&buf[1]);
				// setPidSettings_P((float)tmpLong/1000.0);
				setPidSettings_P(adrReq,(float)tmpLong/10.0);
			}
		break;
		case 'i': // pid propotional value
			{
				buf[0] = ' ';
				tmpLong = atoi((const char*)&buf[1]);
				// setPidSettings_I((float)tmpLong/10000.0);
				setPidSettings_I(adrReq,(float)tmpLong/10.0);
			}
		break;
		case 'd': // pid propotional value
			{
				buf[0] = ' ';
				tmpLong = atoi((const char*)&buf[1]);
				// setPidSettings_D((float)tmpLong/10000.0);
				setPidSettings_D(adrReq,(float)tmpLong/10.0);
			}
		break;
		case 't': // pid propotional value
			{
				buf[0] = ' ';
				tmpLong = atoi((const char*)&buf[1]);
				// setPidSettings_dt((float)tmpLong/1000.0);
				setPidSettings_dt(adrReq,(float)tmpLong/1000.0);
			}
		break;
		case 'e': // pid propotional value
			{
				buf[0] = ' ';
				tmpLong = atoi((const char*)&buf[1]);
				setPidSettings_ESum(adrReq,(float)tmpLong);
			}
		break;

	}
}




void serialParserExecute(char adrReq, char * buf,uint8_t cnt) {
	cnt-=4;

	switch (buf[0]) {
		case 's':
			targetPos = (float)counter;
			// OCR1A = 10;
			/*
			if(true) {
				OCR1A = 10;	
			}
			*/
		break;

		case 'd': // debug mode
			debugMode = buf[1];
			Serial1.print("debugmode:");
			Serial1.println(buf[1]);
			/*
			0 = off
			1 = pid 0 debug
			2 = pid 1 debug
			*/
		break;
	}
}



void serialCMDParser(char c) {
	static char buf[32];
	static uint8_t bufcnt = 0;
	static uint8_t bufcntcmd = 0;
	static char adrReq;
	buf[bufcnt++] = c;
	if(c!=0x0a && c!=0x0d) bufcntcmd++;

	if(bufcnt==2) {
		adrReq = readSingleHex(buf[0])<<4 | readSingleHex(buf[1]);

	}
	
	if((adrReq==0xFF || adrReq==0x00 || adrReq==0x01) && ((c == 0x0a) || (bufcnt >= 30)) || ((bufcnt==2 ) && (c=='?'))) { // (c == 0x0d) ||

		// buf[bufcnt] = 0;
		switch(buf[2]) {
			case '?':
				serialPrintHelp();
			break; 
			case 's': // set
				serialParserSet(adrReq,&buf[3],bufcntcmd);
			break;
			case 'r': // request
				serialParserRequest(adrReq,&buf[3],bufcntcmd);
			break;
			case 'e': // execute
				serialParserExecute(adrReq,&buf[3],bufcntcmd);
			break;
		}

		

		bufcnt = 0;
		bufcntcmd = 0;
	}
}