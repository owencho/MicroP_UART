/*
 * ButtonSM.c
 *
 *  Created on: Aug 20, 2020
 *      Author: academic
 */

#include <MasterSM.h>
#include <Packet.h>
#include "Exti.h"
#include "Usart.h"
#include "UsartDriver.h"
#include "Gpio.h"
#include "Irq.h"
#include "String.h"
#include <stdlib.h>

extern UsartInfo usartInfo[] ;

MasterState buttonState = BUTTON_WAIT;
char adcReadPacket [4];
char ledControlPacket [4];
char serialPacket [8];
char adcData [4];
int ledMode = 0x10;
char receivedCommand;
void handleMasterSM(){
	disableIRQ();
	UsartInfo * info = &usartInfo[MASTER];
	char * buffer = info->usartRxBuffer;
	switch(buttonState){
		case BUTTON_WAIT:
			assignPacketAddressCommand(adcReadPacket,ADC_ADDRESS,0x21);
			usartSendMessage(MASTER,adcReadPacket,4);
			buttonState = READ_ADC;
		break;

		case READ_ADC:
			usartReceiveMessage(MASTER,7);
			buttonState = WAIT_ADC_VALUE;
		break;

		case WAIT_ADC_VALUE:
			receivedCommand = *(buffer+RX_CMD_PACKET);
			if(receivedCommand == 0x20 && compareReceivedCommand(buffer)){
				strcpy(adcData, buffer+RX_DATA_PACKET);
				if(ledMode == 0x12){
					ledMode = 0x10;
				}else{
					ledMode++;
				}
				assignPacketAddressCommand(ledControlPacket,LED_ADDRESS,ledMode);
				usartSendMessage(MASTER,ledControlPacket,4);
				buttonState = SEND_CONTROL_LED;
			}
			else{
				extiSetPendingRegister(exti,PIN_0);
				extiSetInterruptMaskRegister(exti,PIN_0,NOT_MASKED);
				buttonState = BUTTON_WAIT;
			}
		break;

		case SEND_CONTROL_LED:
			assignPacketAddressCommand(serialPacket , SERIAL_ADDRESS , 0x30);
			memcpy(&serialPacket[4], adcData, 4);
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

