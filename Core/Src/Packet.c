/*
 * InvertCommand.c
 *
 *  Created on: Aug 29, 2020
 *      Author: academic
 */


#include "Packet.h"
#include <stdio.h>
#include "PacketMacro.h"

int getInvertCommand(int command){
	 int res = 0;
	  __asm ("eor %[result], %[input_cmd],#255"
	    : [result] "=r" (res)
	    : [input_cmd] "r" (command)
	  );
	  return res;
}

void addInvertCommandInPacket(char * data){
	data[TX_INV_CMD_PACKET]=getInvertCommand(data[TX_CMD_PACKET]);
}

int compareReceivedCommand(char * data){
	 return (data[RX_CMD_PACKET] == getInvertCommand(data[RX_INV_CMD_PACKET]));
}
