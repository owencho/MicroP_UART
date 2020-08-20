/*
 * InitUsart.c
 *
 *  Created on: Aug 18, 2020
 *      Author: academic
 */
#include "InitUsart.h"
#include "Gpio.h"
#include "Clock.h"
#include "Nvic.h"
#include "Syscfg.h"
#include "Exti.h"
#include "Rcc.h"
#include "Usart.h"
#include "Common.h"
#include "BaseAddress.h"

void initUsart1(){
	  //set alternate function
	  gpioSetAlternateFunction(gpioA ,PIN_8 ,AF7); //set PA8 as USART1_CK
	  gpioSetAlternateFunction(gpioA ,PIN_9 ,AF7); //set PA9 as USART1_TX
	  gpioSetAlternateFunction(gpioA ,PIN_10 ,AF7); //set PA10 as USART1_RX

	  // RCC APB1 and APB2 peripheral clock enable reg
	  enableUSART1();
	  //pg 183 to enable uart clock

	  //set oversampling before baud due to function
	  setUsartOversamplingMode(usart1,OVER_16);
	  //set baud with BRR
	  usartSetBaudRate(usart1,115200);
	  //Parity see first bit11 , and bit 10 of CR1
	  usartEnableParityControl(usart1);
	  setUsartParityMode(usart1,ODD_PARITY);
	  //set Half duplex
	  usartSetHalfDuplexMode(usart1,ENABLE_MODE);
	  //enable TE (bit 3, CR1) for transmitter
	  usartEnableTransmission(usart1);
	  //usartEnableReceiver(usart1);
	  //CR1 bit 13 enable USART for both UART
	  enableUsart(usart1);
}

void initUart5(){
	  //set alternate function
	  gpioSetAlternateFunction(gpioC ,PIN_12 ,AF8); //set PC12 as USART5_TX
	  gpioSetAlternateFunction(gpioD ,PIN_2 ,AF8); //set PD2 as USART5_RX

	  // RCC APB1 and APB2 peripheral clock enable reg
	  enableUART5();
	  //pg 183 to enable uart clock

	  //set oversampling before baud due to function
	  setUsartOversamplingMode(uart5,OVER_16);
	  //set baud with BRR
	  usartSetBaudRate(uart5,115200);
	  //Parity see first bit11 , and bit 10 of CR1
	  usartEnableParityControl(uart5);
	  setUsartParityMode(uart5,ODD_PARITY);
	  // Set Address
	  usartSetUsartAddressNode(uart5,5);
	  // set as address mark for wake up
	  setUsartWakeupMode(uart5,ADDRESS_MARK);
	  //set receiver wakeup mode as mute
	  usartSetReceiverWakeupMode(uart5,MUTE_MODE);
	  //set Half duplex
	  usartSetHalfDuplexMode(uart5,ENABLE_MODE);
	  //enable interrupt
	  nvicEnableInterrupt(53);
	  usartEnableInterrupt(uart5,RXNE_INTERRUPT);
	  //enable RE (bit 2 ,CR1) for receiver
	  //usartEnableTransmission(uart5);
	  usartEnableReceiver(uart5);
	  //CR1 bit 13 enable USART for both UART
	  enableUsart(uart5);
}
void initUsart6(){
	  //set alternate function
	  gpioSetAlternateFunction(gpioG ,PIN_7 ,AF8); //set PG7 as USART6_CK
	  gpioSetAlternateFunction(gpioG ,PIN_14 ,AF8); //set PG14 as USART6_TX
	  gpioSetAlternateFunction(gpioG ,PIN_9 ,AF8); //set PG9 as USART6_RX

	  // RCC APB1 and APB2 peripheral clock enable reg
	  enableUSART6();
	  //set oversampling before baud due to function
	  setUsartOversamplingMode(usart6,OVER_16);
	  //set baud with BRR
	  usartSetBaudRate(usart6,115200);
	  //Parity see first bit11 , and bit 10 of CR1
	  usartEnableParityControl(usart6);
	  setUsartParityMode(usart6,ODD_PARITY);
	  // Set Address
	  usartSetUsartAddressNode(usart6,6);
	  // set as address mark for wake up
	  setUsartWakeupMode(usart6,ADDRESS_MARK);
	  //set receiver wakeup mode as mute
	  usartSetReceiverWakeupMode(usart6,MUTE_MODE);
	  //set Half duplex
	  usartSetHalfDuplexMode(usart6,ENABLE_MODE);
	  //enable interrupt
	  nvicEnableInterrupt(71);
	  usartEnableInterrupt(usart6,RXNE_INTERRUPT);
	  //enable RE (bit 2 ,CR1) for receiver
	 // usartEnableTransmission(usart6);
	  usartEnableReceiver(usart6);
	  //CR1 bit 13 enable USART for both UART
	  enableUsart(usart6);
}
