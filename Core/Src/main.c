/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "Gpio.h"
#include "Clock.h"
#include "Nvic.h"
#include "Syscfg.h"
#include "Exti.h"
#include "Rcc.h"
#include "Usart.h"
#include "Common.h"
#include "BaseAddress.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  // RCC APB1 and APB2 peripheral clock enable reg
  enableUSART1();
  enableUSART6();
  enableUART5();
  //pg 183 to enable uart clock
  //set GpioG as alternate mode
  enableGpioG();
  gpioSetMode(gpioG, PIN_7, GPIO_ALT);
  gpioSetMode(gpioG, PIN_14, GPIO_ALT);
  gpioSetMode(gpioG, PIN_9, GPIO_ALT);
  gpioSetPinSpeed(gpioG,PIN_7,HIGH_SPEED);
  gpioSetPinSpeed(gpioG,PIN_14,HIGH_SPEED);
  gpioSetPinSpeed(gpioG,PIN_9,HIGH_SPEED);

  enableGpioA();
  //set GpioA as alternate mode
  gpioSetMode(gpioA, PIN_8, GPIO_ALT);
  gpioSetMode(gpioA, PIN_9, GPIO_ALT);
  gpioSetMode(gpioA, PIN_10, GPIO_ALT);
  gpioSetPinSpeed(gpioA,PIN_8,HIGH_SPEED);
  gpioSetPinSpeed(gpioA,PIN_9,HIGH_SPEED);
  gpioSetPinSpeed(gpioA,PIN_10,HIGH_SPEED);

  enableGpio(PORT_C);
  gpioSetMode(gpioC, PIN_12, GPIO_ALT);  //set GpioC as alternate mode
  gpioSetPinSpeed(gpioC,PIN_12,HIGH_SPEED);

  enableGpio(PORT_D);
  gpioSetMode(gpioD, PIN_2, GPIO_ALT);  //set GpioC as alternate mode
  gpioSetPinSpeed(gpioC,PIN_2,HIGH_SPEED);


  //set alternate function
  gpioSetAlternateFunction(gpioA ,PIN_8 ,AF7); //set PA8 as USART1_CK
  gpioSetAlternateFunction(gpioA ,PIN_9 ,AF7); //set PA9 as USART1_TX
  gpioSetAlternateFunction(gpioA ,PIN_10 ,AF7); //set PA10 as USART1_RX

  gpioSetAlternateFunction(gpioC ,PIN_12 ,AF8); //set PC12 as USART5_TX
  gpioSetAlternateFunction(gpioD ,PIN_2 ,AF8); //set PD2 as USART5_RX

  gpioSetAlternateFunction(gpioG ,PIN_7 ,AF8); //set PG7 as USART6_CK
  gpioSetAlternateFunction(gpioG ,PIN_14 ,AF8); //set PG14 as USART6_TX
  gpioSetAlternateFunction(gpioG ,PIN_9 ,AF8); //set PG9 as USART6_RX

  //set oversampling before baud due to function
  setUsartOversamplingMode(usart1,OVER_16);
  setUsartOversamplingMode(uart5,OVER_16);
  setUsartOversamplingMode(usart6,OVER_16);
  //set baud with BRR
  usartSetBaudRate(usart1,115);
  usartSetBaudRate(uart5,115);
  usartSetBaudRate(usart6,115);
  //Parity see first bit11 , and bit 10 of CR1
  usartEnableParityControl(usart1);
  usartEnableParityControl(uart5);
  usartEnableParityControl(usart6);
  setUsartParityMode(usart1,ODD_PARITY);
  setUsartParityMode(uart5,ODD_PARITY);
  setUsartParityMode(usart6,ODD_PARITY);
  // Set Address
  usartSetUsartAddressNode(usart1,1);
  usartSetUsartAddressNode(uart5,5);
  usartSetUsartAddressNode(usart6,6);
  //set wakeup mode
  // receiver wakeup depends (bit1 ,CR1)
  setUsartWakeupMode(usart1,MUTE_MODE);
  setUsartWakeupMode(uart5,MUTE_MODE);
  setUsartWakeupMode(usart6,MUTE_MODE);
  //set Half duplex
  usartSetHalfDuplexMode(usart1,ENABLE_MODE);
  usartSetHalfDuplexMode(uart5,ENABLE_MODE);
  usartSetHalfDuplexMode(usart6,ENABLE_MODE);
  //enable interrupt
  usartEnableInterrupt(uart5,RXNE_INTERRUPT);
  usartEnableInterrupt(usart6,RXNE_INTERRUPT);
  //enable TE (bit 3, CR1) for transmitter
  usartEnableTransmission(usart1);
  usartEnableTransmission(uart5);
  usartEnableTransmission(usart6);
  //enable RE (bit 2 ,CR1) for receiver
  usartEnableReceiver(usart1);
  usartEnableReceiver(uart5);
  usartEnableReceiver(usart6);
  //CR1 bit 13 enable USART for both UART
  enableUsart(usart1);
  enableUsart(uart5);
  enableUsart(usart6);
  ///CR2
  //LIN mode to detect break
  // STOP bit configure how long is STOP bit
  // CLOCK enable bit not available on UART_4
  //address of usart node, for multiprocessor comm during mute mode

  //CR3
  //mostly interrupt enable
  //enable DMA TR and RX
  // half-duplex selection
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  data = 5;
	  usartSend(usart1,data);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
