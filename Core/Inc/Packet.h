/*
 * InvertCommand.h
 *
 *  Created on: Aug 29, 2020
 *      Author: academic
 */

#ifndef INC_PACKET_H_
#define INC_PACKET_H_

int getInvertCommand(int command);
void addInvertCommandInPacket(char * data);
int compareReceivedCommand(char *data);
#endif /* INC_PACKET_H_ */
