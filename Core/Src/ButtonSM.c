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

extern UsartInfo usartInfo[] ;
BlinkyState buttonState = BUTTON_WAIT;
int adcValue;
char adcRead [] = {0x21,ADC_ADDRESS,0x21, 0xE};
char ledControl [] = {0x21,LED_ADDRESS , 0x10, 0xE };
void handleButtonSM(){
	switch(buttonState){
		case BUTTON_WAIT:
			usartSendMessage(MASTER,adcRead,4);
			buttonState = READ_ADC;
		break;

		case READ_ADC:
			usartReceiveMessage(MASTER,8);
			buttonState = WAIT_ADC_VALUE;
		break;

		case WAIT_ADC_VALUE:
			usartSendMessage(MASTER,ledControl,4);
			ledControl[CMD_PACKET]++;
			if(ledControl[CMD_PACKET]> 0x12){
				ledControl[CMD_PACKET] = 0x10;
			}
			extiSetInterruptMaskRegister(exti,PIN_0,NOT_MASKED);
			buttonState = BUTTON_WAIT;
		break;

		case SEND_CONTROL_LED:
			buttonState = SEND_STRING;
		break;

		case SEND_STRING:
			buttonState = SEND_STRING;
		break;
	}
}

