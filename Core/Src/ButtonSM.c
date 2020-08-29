/*
 * ButtonSM.c
 *
 *  Created on: Aug 20, 2020
 *      Author: academic
 */

#include "ButtonSM.h"
#include "Exti.h"
#include "Usart.h"
#include "UsartDriver.h"
#include "Gpio.h"
#include "String.h"
#include <stdlib.h>

extern UsartInfo usartInfo[] ;
UsartInfo * info = &usartInfo[MASTER];
BlinkyState buttonState = BUTTON_WAIT;
char adcRead [] = {0x21,ADC_ADDRESS,0x21, 0xE};
char ledControl [] = {0x21,LED_ADDRESS , 0x10, 0xE };
char serialSlave [] = {0x21,SERIAL_ADDRESS , 0x10, 0xE };
char adcPacket [4];
char serialPacket [8];
void handleButtonSM(){
	switch(buttonState){
		case BUTTON_WAIT:
			usartSendMessage(MASTER,adcRead,4);
			buttonState = READ_ADC;
		break;

		case READ_ADC:
			usartReceiveMessage(MASTER,7);
			buttonState = WAIT_ADC_VALUE;
		break;

		case WAIT_ADC_VALUE:
			strcpy(adcPacket, (info->usartRxBuffer)+DATA_PACKET);
			if(ledControl[CMD_PACKET] == 0x12){
				ledControl[CMD_PACKET] = 0x10;
			}else{
				ledControl[CMD_PACKET]++;
			}
			usartSendMessage(MASTER,ledControl,4);
			buttonState = SEND_CONTROL_LED;
		break;

		case SEND_CONTROL_LED:
			memcpy(serialPacket, serialSlave, 4);
			memcpy(&serialPacket[4], adcPacket, 4);
			usartSendMessage(MASTER,serialPacket,8);
			buttonState = SEND_STRING;
		break;

		case SEND_STRING:
			extiSetPendingRegister(exti,PIN_0);
			extiSetInterruptMaskRegister(exti,PIN_0,NOT_MASKED);
			buttonState = BUTTON_WAIT;
		break;
	}
}

