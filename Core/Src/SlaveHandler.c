/*
 * SlaveHandler.c
 *
 *  Created on: Aug 22, 2020
 *      Author: academic
 */
#include "Adc.h"

int checkCommand(char *data){
	char command = *(data + 1);
	char command_bar = *(data + 2);

	if(command != (!command_bar)){
		return 0;
	}
	return 1;
}

void handleADCSlave(char *data){
	if(!checkCommand(data)){
		return;
	}
	adcSetStartRegularConversion(adc1);
}
