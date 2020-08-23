/*
 * HardwareConfig.h
 *
 *  Created on: Aug 22, 2020
 *      Author: academic
 */

#ifndef INC_HARDWARECONFIG_H_
#define INC_HARDWARECONFIG_H_
#include "Usart.h"
void configureButtonInterrupt();
void configureGpio();
void initUsartSlave(UsartRegs * usart ,int slaveAddress, int nvicNumber);
void configureTimer3();
void configureAdc1();
void initUsart1();
void initUart5();
void initUsart6();
void initUart4();
void initUart8();
#endif /* INC_HARDWARECONFIG_H_ */
