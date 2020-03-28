/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
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
#include "main.h"
#include "stm32l4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define char_0 '0'
#define char_9 '9'

#define ARR_MIN_VALUE 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern uint8_t ADCFlag;
extern ADC_HandleTypeDef hadc1;
extern uint8_t USART_Cmp_RX_Flag;
extern uint8_t buffUartRx[BUFFOR_UART_RX_SIZE];
extern uint8_t CounterBuffUartRx;
extern uint8_t StartIndexNewDataInBufforUartRX;
extern uint16_t Arr_Value;
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
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
	/*buffUartRx[CounterBuffUartRx] = LL_USART_ReceiveData8(USART2);
	if (buffUartRx[CounterBuffUartRx] >= CHAR_0 && buffUartRx[CounterBuffUartRx] <= CHAR_9)
		if(++CounterBuffUartRx == BUFFOR_UART_RX_SIZE) CounterBuffUartRx = 0;
	if (CounterBuffUartRx == StartIndexNewDataInBufforUartRX + VALUE_LEN_UART_SIZE) 
		USART_Cmp_RX_Flag = USART_FLAG_RX_SET;*/
	if (CounterBuffUartRx < VALUE_LEN_UART_SIZE)
	{
		buffUartRx[CounterBuffUartRx]= LL_USART_ReceiveData8(USART2);
		if (buffUartRx[CounterBuffUartRx] >= char_0 && buffUartRx[CounterBuffUartRx] <= char_9)
			if (++CounterBuffUartRx > (VALUE_LEN_UART_SIZE-1)) USART_Cmp_RX_Flag = USART_FLAG_RX_SET;
	}
  /* USER CODE END USART2_IRQn 0 */
  /* USER CODE BEGIN USART2_IRQn 1 */
  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void TIM2_IRQHandler(void)
{
	TIM2 ->SR &= ~(TIM_SR_UIF);
	if (Arr_Value > ARR_MIN_VALUE) TIM2 ->ARR =  Arr_Value - 1;
	if (ADCFlag == ADC_FLAG_CLEAN)
	{
		ADCFlag = ADC_FLAG_SET;
	}
	else
	{
		Error_Handler();
	}
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
