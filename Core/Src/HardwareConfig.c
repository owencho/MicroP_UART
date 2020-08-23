/*
 * InitUsart.c
 *
 *  Created on: Aug 18, 2020
 *      Author: academic
 */
#include "HardwareConfig.h"
#include "Gpio.h"
#include "Clock.h"
#include "Nvic.h"
#include "Syscfg.h"
#include "Exti.h"
#include "Rcc.h"
#include "Adc.h"
#include "Common.h"
#include "BaseAddress.h"
#include "Timer.h"

void configureButtonInterrupt(){
	  //enable EXTI Line0 interrupt
	  nvicEnableInterrupt(6);
	  extiSetPendingRegister(exti,PIN_0);
	  extiSetInterruptMaskRegister(exti,PIN_0,NOT_MASKED);
	  extiSetRisingTriggerInterrupt(exti,PIN_0,RISING_ENABLED);
	  syscfgExternalInterruptConfig(syscfg,PIN_0, PORT_A);
}
void configureGpio(){
	  //Usart 6
	  enableGpioG();
	  gpioSetMode(gpioG, PIN_7, GPIO_ALT);
	  gpioSetMode(gpioG, PIN_14, GPIO_ALT);
	  gpioSetMode(gpioG, PIN_9, GPIO_ALT);
	  gpioSetPinSpeed(gpioG,PIN_7,HIGH_SPEED);
	  gpioSetPinSpeed(gpioG,PIN_14,HIGH_SPEED);
	  gpioSetPinSpeed(gpioG,PIN_9,HIGH_SPEED);

	  //set LED
	  gpioSetMode(gpioG, PIN_13, GPIO_OUT);
	  gpioSetPinSpeed(gpioG,PIN_13,HIGH_SPEED);

	  enableGpio(PORT_B);
	  gpioSetMode(gpioB, PIN_13, GPIO_OUT);
	  gpioSetPinSpeed(gpioB,PIN_13,HIGH_SPEED);

	  // set pin 1 as input of the ADC signal
	  enableGpioA();
	  gpioSetMode(gpioA, PIN_1, GPIO_ANALOG);
	  gpioSetPinSpeed(gpioA,PIN_1,HIGH_SPEED);

	  //button
	  gpioSetMode(gpioA, PIN_0, GPIO_IN);
	  gpioSetPinSpeed(gpioA,PIN_0,HIGH_SPEED);

	  //Usart1
	  gpioSetMode(gpioA, PIN_8, GPIO_ALT);
	  gpioSetMode(gpioA, PIN_9, GPIO_ALT);
	  gpioSetMode(gpioA, PIN_10, GPIO_ALT);
	  gpioSetPinSpeed(gpioA,PIN_8,HIGH_SPEED);
	  gpioSetPinSpeed(gpioA,PIN_9,HIGH_SPEED);
	  gpioSetPinSpeed(gpioA,PIN_10,HIGH_SPEED);


	  //Uart 5
	  enableGpio(PORT_C);
	  gpioSetMode(gpioC, PIN_12, GPIO_ALT);  //set GpioC as alternate mode
	  gpioSetPinSpeed(gpioC,PIN_12,HIGH_SPEED);

	  enableGpio(PORT_D);
	  gpioSetMode(gpioD, PIN_2, GPIO_ALT);  //set GpioC as alternate mode
	  gpioSetPinSpeed(gpioD,PIN_2,HIGH_SPEED);

	  //uart4
	  gpioSetMode(gpioC, PIN_10, GPIO_ALT);  //uart4_tx
	  gpioSetPinSpeed(gpioC,PIN_10,HIGH_SPEED);

	  gpioSetMode(gpioC, PIN_11, GPIO_ALT);  //uart4_rx
	  gpioSetPinSpeed(gpioC,PIN_11,HIGH_SPEED);

	  //uart8
	  enableGpio(PORT_E);

	  gpioSetMode(gpioE, PIN_0, GPIO_ALT);  //uart8_rx
	  gpioSetPinSpeed(gpioE,PIN_0,HIGH_SPEED);

	  gpioSetMode(gpioE, PIN_1, GPIO_ALT);  //uart8_tx
	  gpioSetPinSpeed(gpioE,PIN_1,HIGH_SPEED);
}
void initUsart1(){
	  //set alternate function
	  gpioSetAlternateFunction(gpioA ,PIN_8 ,AF7); //set PA8 as USART1_CK
	  gpioSetAlternateFunction(gpioA ,PIN_9 ,AF7); //set PA9 as USART1_TX
	  gpioSetAlternateFunction(gpioA ,PIN_10 ,AF7); //set PA10 as USART1_RX

	  //enable usart1 peripheral clock
	  enableUSART1();
	  //pg 183 to enable uart clock
	  //set oversampling before baud due to function
	  setUsartOversamplingMode(usart1,OVER_16);
	  //set baud with BRR
	  usartSetBaudRate(usart1,115200);
	  // Set Address as slave 1
	  usartSetUsartAddressNode(usart1,10);
	  // set as address mark for wake up
	  setUsartWakeupMode(usart1,ADDRESS_MARK);
	  //set receiver wakeup mode as mute
	  usartSetReceiverWakeupMode(usart1,MUTE_MODE);
	  //disable parity control
	  usartDisableParityControl(usart1);
	  //set Half duplex
	  usartSetHalfDuplexMode(usart1,ENABLE_MODE);
	  //set 9 bit
	  setUsartWordLength(usart1,DATA_9_BITS);
	  //enable interrupt
	  nvicEnableInterrupt(37);
	  //enable transmission
	  usartEnableTransmission(usart1);
	  //CR1 bit 13 enable USART for both UART
	  enableUsart(usart1);


}

void initUart5(){
	  //set alternate function
	  gpioSetAlternateFunction(gpioC ,PIN_12 ,AF8); //set PC12 as USART5_TX
	  gpioSetAlternateFunction(gpioD ,PIN_2 ,AF8); //set PD2 as USART5_RX

	  //enable uart5 peripheral clock
	  enableUART5();
	  //configure uart5 as slave
	  initUsartSlave(uart5 ,1, 53);

}
void initUsart6(){
	  //set alternate function
	  gpioSetAlternateFunction(gpioG ,PIN_7 ,AF8); //set PG7 as USART6_CK
	  gpioSetAlternateFunction(gpioG ,PIN_14 ,AF8); //set PG14 as USART6_TX
	  gpioSetAlternateFunction(gpioG ,PIN_9 ,AF8); //set PG9 as USART6_RX

	  // RCC APB1 and APB2 peripheral clock enable reg
	  enableUSART6();
	  //configure uart5 as slave
	  initUsartSlave(usart6 ,2, 71);
}

void initUart4(){
	  //set alternate function
	  gpioSetAlternateFunction(gpioC ,PIN_10 ,AF8); //set PC10 as UART4_TX
	  gpioSetAlternateFunction(gpioC ,PIN_11 ,AF8); //set PC11 as UART4_RX

	  // RCC APB1 and APB2 peripheral clock enable reg
	  enableUART4();
	  //configure uart5 as slave
	  initUsartSlave(uart4 ,3, 52);
}

void initUart8(){
	  //set alternate function
	  gpioSetAlternateFunction(gpioE ,PIN_0 ,AF8); //set PE0 as UART8_RX
	  gpioSetAlternateFunction(gpioE ,PIN_1 ,AF8); //set PE1 as UART8_TX

	  // RCC APB1 and APB2 peripheral clock enable reg
	  enableUART8();
	  //configure uart5 as slave
	  initUsartSlave(uart8 ,4, 83);
}

void initUsartSlave(UsartRegs * usart ,int slaveAddress, int nvicNumber){
	  //pg 183 to enable uart clock
	  //set oversampling before baud due to function
	  setUsartOversamplingMode(usart,OVER_16);
	  //set baud with BRR
	  usartSetBaudRate(usart,115200);
	  // Set Address as slave 1
	  usartSetUsartAddressNode(usart,slaveAddress);
	  // set as address mark for wake up
	  setUsartWakeupMode(usart,ADDRESS_MARK);
	  //set receiver wakeup mode as mute
	  usartSetReceiverWakeupMode(usart,MUTE_MODE);
	  //disable parity control
	  usartDisableParityControl(usart);
	  //set Half duplex
	  usartSetHalfDuplexMode(usart,ENABLE_MODE);
	  //set 9 bit
	  setUsartWordLength(usart,DATA_9_BITS);
	  //enable TE (bit 3, CR1) for transmitter
	  usartEnableReceiver(usart);
	  //enable interrupt
	  nvicEnableInterrupt(nvicNumber);
	  usartEnableInterrupt(usart,RXNE_INTERRUPT);
	  //CR1 bit 13 enable USART for both UART
	  enableUsart(usart);
}

void configureAdc1(){
	  enableAdc1();
	  //enable interrupt
	  //adcEnableEOCInterrupt(adc1);
	  adcSetScanMode(adc1,ENABLE_MODE);
	  nvicEnableInterrupt(18);
	  adcSetADCResolution(adc1,ADC_RES_12_BIT);
	  adcSetRightDataAlignment(adc1);
	  adcSetSingleConvertion(adc1);
	  //adcSetContinousConvertion(adc1);
	  adcSetSamplingTime(adc1,CHANNEL_1,ADC_SAMP_3_CYCLES);
	  adcSetSingleSequenceRegister(adc1,CHANNEL_1,1);
	  adcEnableADCConversion(adc1);
	  //adcSetStartRegularConversion(adc1);
}

void configureTimer3(){
	  enableTimer3();
	  //enable nvic interrupt
	  nvicEnableInterrupt(29);

	  timerSetControlRegister(timer3,(ARR_ENABLE | TIMER_UP_COUNT |
			  	  	  	  	  	  	  TIMER_ONE_PULSE_DISABLE |TIMER_COUNTER_ENABLE |
									  T1_CH1_SELECT| MASTER_MODE_COMP_OC3REF|OC3_OUT_LOW));
	  //ARR disable
	  //ARR reg is buffered
	  //Up count
	  //one pulse mode disabled
	  //counter enabled
	  //CH1 is connected to T1
	  // Master Mode is routed to Output 3
	  timerSetSlaveMasterRegister(timer3,SLAVE_MODE| SMS_DISABLED | TRIGGER_FIL_T1);
	  //slave mode disabled
	  timerSetCompareCaptureModeRegister(timer3,(CC3_OUTPUT |OC3_MODE_TOGGLE));
	  // CC3 channel is configured as toggle mode

	  timerSetCompareCaptureEnableRegister(timer3,(OC3_ENABLE|OC3_ACTIVELOW));

	  //to generate 50hz with 50% duty cycle
	  timerWritePrescaler(timer3,27);
	  timerWriteAutoReloadReg(timer3, 65535);
	  timerWriteCapComReg3(timer3 , 32767);

}
