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
int ADCvalue;
char adcRead [] = {ADC_ADDRESS , 0x21, 0xE };
void handleButtonSM(){
	switch(buttonState){
		case BUTTON_WAIT:
			usartSendMessage(MASTER,adcRead);
			buttonState = READ_ADC;
		break;

		case READ_ADC:
			buttonState = WAIT_ADC_VALUE;
		break;

		case WAIT_ADC_VALUE:
			adcValue = handleADCvalue();
			usartSendMessage(MASTER,ledControl);
		break;

		case SEND_CONTROL_LED:

		break;

		case SEND_STRING:

		break;
	}
}

