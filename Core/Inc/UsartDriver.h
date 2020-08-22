/*
 * Serial.h
 *
 *  Created on: Aug 13, 2020
 *      Author: academic
 */

#ifndef INC_USARTDRIVER_H_
#define INC_USARTDRIVER_H_
#include "Usart.h"


#define PACKET_SIZE 0
#define ADDRESS_PACKET 1

#define MASTER_ADDRESS 0
#define ADC_ADDRESS 1
#define LED_ADDRESS 2
#define SEND_ADDRESS 3
#define SERIAL_ADDRESS 4

typedef struct UsartInfo UsartInfo;
struct UsartInfo {
	UsartRegs * usart;
	char * usartTxBuffer;
	char * usartRxBuffer;
	int requestTxPacket;
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
	SEND_SLAVE,
	SERIAL_SLAVE,
} UsartPort;


void initUsartInfo();
void handleUsartSend(UsartRegs * usart , char * transmitBuffer , int count);
void usartSendMessage(UsartPort port,char * message);
int findPacketLength(char data[]);
#endif /* INC_USARTDRIVER_H_ */
