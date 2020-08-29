/*
 * Serial.c
 *
 *  Created on: Aug 13, 2020
 *      Author: academic
 */
#include "Irq.h"
#include "Nvic.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <malloc.h>
#include <stdarg.h>
#include <UsartDriver.h>

#define hasRequestedTxPacket(info) ((info)->requestTxPacket)
#define hasRequestedRxPacket(info) ((info)->requestRxPacket)

int findPacketLength(char * data){
    return (sizeof(data)/sizeof(char));
}

UsartInfo usartInfo[] = {
  [MASTER]={NULL},
  [ADC_SLAVE]={NULL},
  [LED_SLAVE]={NULL},
  [SERIAL_SLAVE]={NULL},
  [PRINT_SLAVE]={NULL},
};

void handleUsartSend(UsartRegs * usart , char * transmitBuffer , int count){
	if(count == TX_ADDRESS_PACKET){
		usartSend(usart,transmitBuffer[count]+ (1<<8));
	}
	else{
		usartSend(usart,transmitBuffer[count]);
	}

}
void usartInfoConfig(UsartPort port,UsartRegs * usart){
    disableIRQ();
    UsartInfo * info =&usartInfo[port];
    info->usart = usart;
    info->usartRxBuffer = malloc(sizeof(char)*64);
    info->txCount = 0;
    info->rxCount = 0;
    info->requestTxPacket = 0;
    info->requestRxPacket = 0;
    enableIRQ();
}

void initUsartInfo(){
	memset(&usartInfo[PRINT_SLAVE],0,sizeof(usartInfo));
	usartInfoConfig(MASTER,usart1);
	usartInfoConfig(ADC_SLAVE,uart5);
	usartInfoConfig(LED_SLAVE,usart6);
	usartInfoConfig(SERIAL_SLAVE,uart4);
	usartInfoConfig(PRINT_SLAVE,uart8);
}
void usartSendMessage(UsartPort port,char * message,int length){
	disableIRQ();
	UsartInfo * info =&usartInfo[port];

    if(!hasRequestedTxPacket(info)){
        info->requestTxPacket = 1;
    	info->usartTxBuffer = message;
    	info->txLength = length;
    	info->txTurn = 1;
    	usartEnableInterrupt(info->usart,TRANS_COMPLETE);
    	usartDisableReceiver(info->usart);
    	usartEnableTransmission(info->usart);
    }
	enableIRQ();
}

void usartReceiveMessage(UsartPort port,int size){
	disableIRQ();
	UsartInfo * info =&usartInfo[port];
	if(!hasRequestedRxPacket(info)){
		info->requestRxPacket = 1;
		info->rxLength = size;
		info->txTurn = 0;
		usartDisableTransmission(info->usart);
		usartEnableReceiver(info->usart);
		usartEnableInterrupt(info->usart,RXNE_INTERRUPT);
		usartSetReceiverWakeupMode(info->usart,MUTE_MODE);
	}
	enableIRQ();
}
