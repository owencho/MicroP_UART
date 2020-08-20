/*
 * Serial.h
 *
 *  Created on: Aug 13, 2020
 *      Author: academic
 */

#ifndef INC_SERIAL_H_
#define INC_SERIAL_H_
#include "Usart.h"

typedef struct UsartInfo UsartInfo;
struct UsartInfo {
	UsartRegs * usart;
	char * usartBufferPtr;
	char * usartTxLen;
	int usartTurn;
	int  txTurn;
};

void freeMessage(char * msg);
void serialSend(UsartRegs * usart,char *message,...);
void usartSendMessage(UsartInfo * info,char *message,int length);
#endif /* INC_SERIAL_H_ */
