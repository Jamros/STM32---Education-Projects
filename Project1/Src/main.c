#include "main.h"
#include <math.h>
#include <stdio.h>

ADC_HandleTypeDef hadc1;

uint32_t ADCconversion = 0;
int32_t tempC = 0;
uint8_t ADCFlag = ADC_FLAG_CLEAN;
uint8_t USART_Cmp_RX_Flag = USART_FLAG_RX_CLEAN;
uint8_t RecData = 0;

uint8_t volatile buffUartRx[BUFFOR_UART_RX_SIZE] = {0};
uint8_t volatile CounterBuffUartRx = 0;

uint16_t Arr_Value = 0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void CMSIS_TIM2_Init(void);

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();

	CMSIS_TIM2_Init();

  while (1)
  {
		if (USART_Cmp_RX_Flag == USART_FLAG_RX_SET)
		{
			Arr_Value = 0;
			Arr_Value = 100*(buffUartRx[0]-ASCII_TO_NUMBERS);
			Arr_Value = Arr_Value + 10*(buffUartRx[1]-ASCII_TO_NUMBERS);
			Arr_Value = Arr_Value + buffUartRx[2]-ASCII_TO_NUMBERS;
			CounterBuffUartRx = 0;
		USART_Cmp_RX_Flag = USART_FLAG_RX_CLEAN;
		}
		if (ADCFlag == ADC_FLAG_SET)
		{
			if (HAL_ADC_Start(&hadc1) != HAL_OK)
			{
				Error_Handler();
			};
			if (HAL_ADC_PollForConversion(&hadc1,100) == HAL_OK)
			{
			ADCconversion = HAL_ADC_GetValue(&hadc1);
			tempC = __LL_ADC_CALC_TEMPERATURE(3300,ADCconversion,LL_ADC_RESOLUTION_12B);
			if(LL_USART_IsActiveFlag_TXE(USART2)) LL_USART_TransmitData8(USART2,(char)tempC);
			ADCFlag = ADC_FLAG_CLEAN;
			}
		}
  }
}

void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  Error_Handler();  
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_MSI_Enable();

   /* Wait till MSI is ready */
  while(LL_RCC_MSI_IsReady() != 1)
  {
    
  }
  LL_RCC_MSI_EnableRangeSelection();
  LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_6);
  LL_RCC_MSI_SetCalibTrimming(0);
  LL_RCC_PLLSAI1_ConfigDomain_ADC(LL_RCC_PLLSOURCE_MSI, LL_RCC_PLLM_DIV_1, 18, LL_RCC_PLLSAI1R_DIV_2);
  LL_RCC_PLLSAI1_EnableDomain_ADC();
  LL_RCC_PLLSAI1_Enable();

   /* Wait till PLLSAI1 is ready */
  while(LL_RCC_PLLSAI1_IsReady() != 1)
  {
    
  }
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_MSI)
  {
  
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_SetSystemCoreClock(4000000);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_HSI);
  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_PLLSAI1);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV12;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
	if (HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  };
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
  
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration  
  PA2   ------> USART2_TX
  PA15 (JTDI)   ------> USART2_RX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_3;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */
	//Wlaczenie przerwan od bufora RX
	LL_USART_EnableIT_RXNE(USART2); 
	
  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = 9600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

}

/* USER CODE BEGIN 4 */
static void CMSIS_TIM2_Init(void)
	{
		//taktowanie timera2
		RCC-> APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

		//taktowanie portu GPIOB;
		RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

		//3 NOZKA NA OUTPUT MODE3 01
		GPIOB-> MODER |= GPIO_MODER_MODE3_1;
		GPIOB-> MODER &= ~(GPIO_MODER_MODE3_0);

		//PUSH-PULL
		GPIOB-> OTYPER &= ~(GPIO_OTYPER_OT3);

		//SZYBKOSC
		GPIOB-> OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR3_0);
		GPIOB-> OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR3_1);

		//DLA POLACZENIA Z TIMEREM, USTAWIENIA FUNCJI ALTERNATYWNEJ
		GPIOB-> AFR[0] |= GPIO_AFRL_AFSEL3_0;

		//ustawienia dzielnika oraz registry z liczbami, do ktorych timer liczy
		TIM2->PSC = SystemCoreClock / 1000UL - 1;
		TIM2->ARR = 1000UL - 1;

		//POROWNANIE
		TIM2->CCR2 = 0;//(TIM2->ARR + 1)/2;

		//Wybranie trybu Compare Output Mode (Toogle) 0011, ustawienia bitu 0 i 1
		TIM2-> CCMR1  |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_0;

		//Polaczenie wyjscia Timer'a z TIM2_CH2 (Pinem)
		TIM2 ->CCER |= TIM_CCER_CC2E;

		//Przerwania na kanale 2 timera 2, przerwania
		TIM2-> DIER |= TIM_DIER_UIE; // |TIM_DIER_CC2IE | TIM_DIER_TIE;

		//wlaczenia przerwania
		NVIC_EnableIRQ(TIM2_IRQn);
		
		//Wyprowadzenie sygnalu Compare do ADC
		TIM2 -> CR2 |= TIM_CR2_MMS_0 | TIM_CR2_MMS_2;

		// Aktywowanie Timera
		TIM2 ->CR1 |= TIM_CR1_CEN;
	}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	while(1);
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
