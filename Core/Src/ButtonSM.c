/*
 * ButtonSM.c
 *
 *  Created on: Aug 20, 2020
 *      Author: academic
 */

#include "ButtonSM.h"
#include "Exti.h"
#include "Usart.h"
BlinkyState buttonState = BUTTON_WAIT;
char adcRead [] = {0x12 , 0x21, 0xE };
void ButtonSM(){
	switch(buttonState){
		case BUTTON_WAIT:
			sendPacket();
			buttonState = READ_ADC;
		break;

		case READ_ADC:

		break;

		case WAIT_ADC_VALUE:

		break;

		case SEND_CONTROL_LED:

		break;

		case SEND_STRING:

		break;
	}
}
