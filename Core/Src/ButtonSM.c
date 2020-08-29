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
#include "Irq.h"
#include "String.h"
#include "InvertCommand.h"
#include <stdlib.h>

extern UsartInfo usartInfo[] ;

BlinkyState buttonState = BUTTON_WAIT;
char adcRead [4] = {0x21,ADC_ADDRESS,0x21,0x21};
char ledControl [4] = {0x21,LED_ADDRESS , 0x10,0x10};
char serialPacket [8] = {0x21,SERIAL_ADDRESS , 0x10,0x10};
char adcPacket [4];
int adcValue ;
void handleButtonSM(){
	disableIRQ();
	UsartInfo * info = &usartInfo[MASTER];
	char * buffer = info->usartRxBuffer;
	switch(buttonState){
		case BUTTON_WAIT:
			//adcRead[SEND_INV_CMD_PACKET]=getInvertCommand(adcRead[SEND_CMD_PACKET]);
			usartSendMessage(MASTER,adcRead,4);
			buttonState = READ_ADC;
		break;

		case READ_ADC:
			usartReceiveMessage(MASTER,7);
			buttonState = WAIT_ADC_VALUE;
		break;

		case WAIT_ADC_VALUE:
			strcpy(adcPacket, buffer+DATA_PACKET);
			if(ledControl[SEND_CMD_PACKET] == 0x12){
				ledControl[SEND_CMD_PACKET] = 0x10;
			}else{
				ledControl[SEND_CMD_PACKET]++;
			}
			usartSendMessage(MASTER,ledControl,4);
			buttonState = SEND_CONTROL_LED;
		break;

		case SEND_CONTROL_LED:
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
	enableIRQ();
}

