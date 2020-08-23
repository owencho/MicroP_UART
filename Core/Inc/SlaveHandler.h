/*
 * SlaveHandler.h
 *
 *  Created on: Aug 23, 2020
 *      Author: academic
 */

#ifndef INC_SLAVEHANDLER_H_
#define INC_SLAVEHANDLER_H_

#include "UsartDriver.h"
void handleADCSlave(char *data);
void handleLEDSlave(char *data);
void handleSerialSlave(char *data);
void serialSend(UsartPort port,char *message,...);
void usartSendSerialMessage(UsartInfo * info,char *message);
#endif /* INC_SLAVEHANDLER_H_ */
