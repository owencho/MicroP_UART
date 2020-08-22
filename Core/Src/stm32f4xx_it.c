/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "Irq.h"
#include "Adc.h"
#include "Exti.h"
#include "UsartDriver.h"
#include "main.h"
#include "Usart.h"
#include "Common.h"
#include "ButtonSM.h"
#include "Gpio.h"
#include "Rcc.h"
#include "BaseAddress.h"
#include "stm32f4xx_it.h"
#include "ButtonSM.h"
#include <stdlib.h>
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern UsartInfo usartInfo[];
int txCount = 0;
extern int interrupt;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */
char newMsg[8];
void ADC_IRQHandler(void){
	int newAdcValue ;
	char buffer[4];
	char msg[]={0x21,MASTER,0x21,0xE};
	newAdcValue = adcReadRegularDataReg(adc1);
	itoa(newAdcValue,buffer,10);
	memcpy(newMsg, msg, 4);
	memcpy(&newMsg[4], buffer, 4);
	adcDisableEOCInterrupt(adc1);
	usartSendMessage(ADC_SLAVE,newMsg,8);

}

void UART5_IRQHandler(void){
	UsartInfo * info = &usartInfo[ADC_SLAVE];
	UsartRegs * usart = info->usart;
	char * receiveBuffer = info->usartRxBuffer;
	char * transmitBuffer = info->usartTxBuffer;

	if(info->txTurn){
		   if(info->txLength != info->txCount){
			   usartClearTcFlag(usart);
			   handleUsartSend(usart,transmitBuffer,info->txCount);
			   info->txCount++;
		   }else{
			   gpioWriteBit(gpioB, PIN_13, 1);
			   usartDisableInterrupt(usart,TRANS_COMPLETE);
			   usartReceiveMessage(ADC_SLAVE,3);
			   info->txTurn = 0;
			   info->txCount = 0;
			   info->requestTxPacket = 0;

		   }
	}
	else{
		   if(info->rxLength != info->rxCount){
			   receiveBuffer[info->rxCount] = usartReceive(usart);
			   info->rxCount ++;
		   }

		   if(info->rxLength == info->rxCount){
			   if(*receiveBuffer == ADC_ADDRESS){
				   usartDisableReceiver(usart);
				   handleADCSlave(receiveBuffer);
			   }
			   info->rxCount = 0;
		   }
	}
}

void USART6_IRQHandler(void){
	UsartInfo * info = &usartInfo[LED_SLAVE];
	UsartRegs * usart = info->usart;
	char * receiveBuffer = info->usartRxBuffer;

   if(info->rxLength != info->rxCount){
	   receiveBuffer[info->rxCount] = usartReceive(usart);
	   info->rxCount ++;
   }

   if(info->rxLength == info->rxCount){
	   if(*receiveBuffer == LED_ADDRESS){
		   usartDisableReceiver(usart);
		   handleLEDSlave(receiveBuffer);
	   }
	   info->rxCount = 0;
   }
}

void USART1_IRQHandler(void){
	UsartInfo * info = &usartInfo[MASTER];
	UsartRegs * usart = info->usart;
	char * receiveBuffer = info->usartRxBuffer;
	char * transmitBuffer = info->usartTxBuffer;

	if(info->txTurn){
		   if(info->txLength != info->txCount){
			   usartClearTcFlag(usart);
			   handleUsartSend(usart,transmitBuffer,info->txCount);
			   info->txCount++;
		   }else{
			   usartDisableInterrupt(usart,TRANS_COMPLETE);
			   info->txTurn = 0;
			   info->txCount = 0;
			   info->requestTxPacket = 0;
			   handleButtonSM();
		   }
	}
	else{
		   if(info->rxLength != info->rxCount){
			   receiveBuffer[info->rxCount] = usartReceive(usart);
			   info->rxCount ++;
		   }

		   if(info->rxLength == info->rxCount){
			   if(*receiveBuffer == MASTER_ADDRESS){
				   usartEnableTransmission(usart);
				   usartDisableReceiver(usart);
				   handleButtonSM();
			   }

			   info->rxCount = 0;
		   }
	}
}


void EXTI0_IRQHandler(void){
	disableIRQ();
	extiSetPendingRegister(exti,PIN_0);
	extiSetInterruptMaskRegister(exti,PIN_0,MASKED);
	handleButtonSM();
	enableIRQ();
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
