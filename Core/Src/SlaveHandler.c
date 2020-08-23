/*
 * SlaveHandler.c
 *
 *  Created on: Aug 22, 2020
 *      Author: academic
 */
#include "Adc.h"
#include "Gpio.h"
#include "Irq.h"
#include "Timer.h"
/*
int checkCommand(char *data){
	int command = *(data + 1);
	int command_bar = *(data + 2);

	if((~command) != command_bar){
		return 0;
	}
	return 1;
}
*/
void handleADCSlave(char *data){
	disableIRQ();
	adcEnableEOCInterrupt(adc1);
	adcSetStartRegularConversion(adc1);
	enableIRQ();
}

void handleLEDSlave(char *data){
	disableIRQ();
	char command = *(data+1);
	if(command == 0x10){
		timerDisableInterrupt(timer3 , CC3_INT);
		gpioWriteBit(gpioG, PIN_13, 0);
	}
	else if(command == 0x11){
		timerEnableInterrupt(timer3 ,CC3_INT);
	}
	else if(command == 0x12){
		timerDisableInterrupt(timer3 , CC3_INT);
		gpioWriteBit(gpioG, PIN_13, 1);
	}
	enableIRQ();
}
