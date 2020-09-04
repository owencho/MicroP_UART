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
#include "PacketMacro.h"
#include "SlaveHandler.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <malloc.h>
#include <Packet.h>
#include <stdarg.h>
#include <string.h>

extern UsartInfo usartInfo[] ;

void handleADCSlave(char *data){
	disableIRQ();
	if(compareReceivedCommand(data)){
		adcEnableEOCInterrupt(adc1);
		adcSetStartRegularConversion(adc1);
	}
	enableIRQ();
}

void handleLEDSlave(char *data){
	disableIRQ();
	char command = *(data+RX_CMD_PACKET);
	if(compareReceivedCommand(data)){
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
	}
	enableIRQ();
}

void handleSerialSlave(char *data){
	disableIRQ();
	int adcValue;
	if(compareReceivedCommand(data)){
		adcValue = *(int*)&data[RX_DATA_PACKET];
		serialSend(PRINT_SLAVE,"adc value is %d \r\n",adcValue);
	}
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

