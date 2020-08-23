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
#include "SlaveHandler.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <malloc.h>
#include <stdarg.h>
#include <string.h>

extern UsartInfo usartInfo[] ;
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

void handleSerialSlave(char *data){
	disableIRQ();
	int adcValue;
	adcValue = strtol(data+DATA_PACKET, NULL, 10);
	serialSend(PRINT_SLAVE,"adc value is %d \r\n",adcValue);
	enableIRQ();
}

void freeMessage(char * msg){
    if(msg)
        free(msg);
}

void serialSend(UsartPort port,char *message,...){
	disableIRQ();
    int actualLength;
    char* buffer;

    va_list arg;
    va_start(arg, message);

    actualLength = vsnprintf(NULL,0, message, arg);
    buffer =malloc(actualLength + 1);
    vsnprintf(buffer,actualLength + 1, message, arg);
    va_end(arg);
	UsartInfo * info = &usartInfo[port];
    usartSendSerialMessage(info,buffer);
    freeMessage(buffer);
    enableIRQ();
}

void usartSendSerialMessage(UsartInfo * info,char *message){
	disableIRQ();
	usartEnableInterrupt(info->usart,TRANS_COMPLETE);
	info->usartTxBuffer = message;
	info->txTurn = 1;
	info->txCount = 0;
	enableIRQ();
}

