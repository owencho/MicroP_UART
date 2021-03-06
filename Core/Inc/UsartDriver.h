/*
 * Serial.h
 *
 *  Created on: Aug 13, 2020
 *      Author: academic
 */

#ifndef INC_USARTDRIVER_H_
#define INC_USARTDRIVER_H_
#include "Usart.h"
#include "PacketMacro.h"

#define MASTER_ADDRESS 10
#define ADC_ADDRESS 1
#define LED_ADDRESS 2
#define SERIAL_ADDRESS 3


typedef struct UsartInfo UsartInfo;
struct UsartInfo {
	UsartRegs * usart;
	char * usartTxBuffer;
	char * usartRxBuffer;
	int requestTxPacket;
	int requestRxPacket;
	int  txTurn;
	int txLength;
	int rxLength;
	int txCount;
	int rxCount;
};

typedef enum{
	MASTER,
    ADC_SLAVE,
	LED_SLAVE,
	SERIAL_SLAVE,
	PRINT_SLAVE,
} UsartPort;


void initUsartInfo();
void usartReceiveMessage(UsartPort port,int size);
void handleUsartSend(UsartRegs * usart , char * transmitBuffer , int count);
void usartSendMessage(UsartPort port,char * message,int length);
int findPacketLength(char data[]);
#endif /* INC_USARTDRIVER_H_ */
