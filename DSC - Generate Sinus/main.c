/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32l4xx.h"
#include <math.h>

#define SYSCLK_SOURCE_MSI 0
#define SYSCLK_SOURCE_HSI16 1
#define SYSCLK_SOURCE_HSE 2
#define SYSCLK_SOURCE_PLL 3

#define PLL_R_DIVIDER_2 0
#define PLL_R_DIVIDER_4 1
#define PLL_R_DIVIDER_6 2
#define PLL_R_DIVIDER_8 3

#define SINBUFFORSIZE 4096

#define RADMAX 360

static void ConfigDacPin();
static void SystemCoreClock_Config();
static void SetPLLPar(uint8_t PLL_Source,uint8_t N_Multipler,uint8_t M_Divider,uint8_t R_Divider);

uint16_t SinTable[SINBUFFORSIZE];

int main(void)
{
	//Generowanie sygnalu pilo-ksztaltnego
	//uint16_t DAC_Value = 0;

	// Generowanie tablicy dla sinusa
	for (uint16_t index = 0;index < SINBUFFORSIZE; index ++)
	{
		SinTable[index] = (sinf(index*2*M_PI/SINBUFFORSIZE)+1)*((0xFFF+1)/2);
	}

	// Wlaczenie taktowania dla DAC
	SET_BIT(RCC->APB1ENR1,RCC_APB1ENR1_DAC1EN);

	// Wlaczenie taktowania dla PWR
	SET_BIT(RCC->APB1ENR1,RCC_APB1ENR1_PWREN);

	// Konfiguracja pinu DAC (PA4)
	ConfigDacPin();

	// Enable HSI16 and connect to PLL, 4-Wait State, Configure PLL for 80Mhz, Connect PLK to SysCLK
	SystemCoreClock_Config();

	// Kongiuracaj normalnego tryby z wyjœciem na pin DAC_CH1
	MODIFY_REG(DAC->MCR,DAC_MCR_MODE1_Msk,DAC_MCR_MODE1_1);

	// Wlaczenie DAC_CH1
	SET_BIT(DAC->CR,DAC_CR_EN1);
	for(;;)
	{
		//Generowanie sygnalu pilo-ksztaltnego

		/*
		MODIFY_REG(DAC->DHR12R1,DAC_DHR12R1_DACC1DHR_Msk,DAC_Value);
		DAC_Value = ++DAC_Value & DAC_DHR12R1_DACC1DHR_Msk;
		*/

		//Generowanie sinusa
		for (uint16_t index = 0;index < SINBUFFORSIZE; index ++)
		{
			MODIFY_REG(DAC->DHR12R1,DAC_DHR12R1_DACC1DHR_Msk,SinTable[index]);
		}
	}
}

static void ConfigDacPin()
{
	//taktowanie portu GPIOA;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

	//Analog Mode
	GPIOA-> MODER |= GPIO_MODER_MODE4_1;
	GPIOA-> MODER |= GPIO_MODER_MODE4_0;


	//SZYBKOSC maksymalna
	GPIOA-> OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4_0;
	GPIOA-> OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4_1;

}

static void SystemCoreClock_Config()
{
	// Wlaczenie HSI16
	SET_BIT(RCC->CR,RCC_CR_HSION);

	//Oczekiwanie na ustabilizowanie HSI16
	while(!READ_BIT(RCC->CR,RCC_CR_HSIRDY));

	//Set Flash Latency for 4 Wait States
	MODIFY_REG(FLASH->ACR,FLASH_ACR_LATENCY,FLASH_ACR_LATENCY_4WS);

	//Configure PLL with Max Freq
	uint8_t N_Value = 10, M_Value = 1, R_Value = PLL_R_DIVIDER_2, PLL_Source = RCC_PLLCFGR_PLLSRC_HSI;
	SetPLLPar(PLL_Source,N_Value,M_Value,R_Value);

	// Change Source SYSCLK to PLL
	MODIFY_REG(RCC->CFGR,RCC_CFGR_SW_Msk,SYSCLK_SOURCE_PLL);
}

static void SetPLLPar(uint8_t PLL_Source,uint8_t N_Multipler,uint8_t M_Divider,uint8_t R_Divider)
{
	//Wylaczenie PLL
	CLEAR_BIT(RCC->CR,RCC_CR_PLLON);

	//Oczekiwanie na wylaczenie PLL
	while(READ_BIT(RCC->CR,RCC_CR_PLLRDY));

	//Ustawienie zrodla taktowania
	MODIFY_REG(RCC->PLLCFGR,RCC_PLLCFGR_PLLSRC,PLL_Source << RCC_PLLCFGR_PLLSRC_Pos);

	//Zmiana parametrow (N,M,R)

	// Change MULTIPLY N Value
	MODIFY_REG(RCC->PLLCFGR,RCC_PLLCFGR_PLLN,N_Multipler << RCC_PLLCFGR_PLLN_Pos);

	// Change DIVIDER R Value
	MODIFY_REG(RCC->PLLCFGR,RCC_PLLCFGR_PLLR,R_Divider << RCC_PLLCFGR_PLLR_Pos);

	// Change DIVIDER M Value
	MODIFY_REG(RCC->PLLCFGR,RCC_PLLCFGR_PLLM,M_Divider << RCC_PLLCFGR_PLLM_Pos);

	//Wlaczenie PLL
	SET_BIT(RCC->CR,RCC_CR_PLLON);

	//Oczekiwanie na wlaczenie PLL
	while(!READ_BIT(RCC->CR,RCC_CR_PLLRDY));

	//Wlaczenie wyjscia PLLCLK
	SET_BIT(RCC->PLLCFGR,RCC_PLLCFGR_PLLREN);
}
