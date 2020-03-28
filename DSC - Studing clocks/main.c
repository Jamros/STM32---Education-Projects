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

#define MCO_OUTPUT_DISABLE 0
#define MCO_OUTPUT_SYSCLK 1
#define MCO_OUTPUT_MSI 2
#define MCO_OUTPUT_HSI16 3
#define MCO_OUTPUT_HSE 4
#define MCO_OUTPUT_PLL 5
#define MCO_OUTPUT_LSI 6
#define MCO_OUTPUT_LSE 7
#define MCO_OUTPUT_HSI48 8

#define MSI_OUTPUT_100kHz 0
#define MSI_OUTPUT_200kHz 1
#define MSI_OUTPUT_400kHz 2
#define MSI_OUTPUT_800kHz 3
#define MSI_OUTPUT_1MHz 4
#define MSI_OUTPUT_2MHz 5
#define MSI_OUTPUT_4MHz 6
#define MSI_OUTPUT_8MHz 7
#define MSI_OUTPUT_16MHz 8
#define MSI_OUTPUT_24MHz 9
#define MSI_OUTPUT_32MHz 10
#define MSI_OUTPUT_48MHz 11

#define SYSCLK_SOURCE_MSI 0
#define SYSCLK_SOURCE_HSI16 1
#define SYSCLK_SOURCE_HSE 2
#define SYSCLK_SOURCE_PLL 3

#define ENABLE_MCO 1
#define DISABLE_MCO !ENABLE_MCO

#define ACCESS_BRD_DISABLE 0
#define ACCESS_BRD_ENABLE 1

#define PLL_R_DIVIDER_2 0
#define PLL_R_DIVIDER_4 1
#define PLL_R_DIVIDER_6 2
#define PLL_R_DIVIDER_8 3

#define PLL_M_DIVIDER_1 0

#define RTC_SOURCE_DISABLE 0
#define RTC_SOURCE_LSE 1
#define RTC_SOURCE_LSI 2
#define RTC_SOURCE_HSE 3

static void Config_MCO(uint8_t);
static void ChangeMSIFreq(uint8_t);
static void Set_Prescaler_MCO(uint32_t);
static void SetPLLPar(uint8_t PLL_Source,uint8_t N_Multipler,uint8_t M_Divider,uint8_t R_Divider);
static void EnableMSI(void);

uint8_t BitValue;

int main(void)
{

	//-------------------------------------ZAD 2.-------------------------------------



	//-------------------------------------ZAD 2.1-------------------------------------

	//Config MCO PIN
	Config_MCO(ENABLE_MCO);
	//-------------------------------------ZAD 2.1 OK-------------------------------------

	//-------------------------------------ZAD 2.2-------------------------------------
	//SET MCO_OUTPUT_SYSCLK
	MODIFY_REG(RCC->CFGR,RCC_CFGR_MCOSEL_Msk,MCO_OUTPUT_SYSCLK << RCC_CFGR_MCOSEL_Pos);

	//-------------------------------------ZAD 2.2 OK-------------------------------------

	//-------------------------------------ZAD 2.3-------------------------------------
	// Change MSI Freq for 2MHz
	//ChangeMSIFreq(MSI_OUTPUT_8MHz); //Nie dzia³a

	SET_BIT(RCC->CR,RCC_CR_MSIRGSEL | RCC_CR_MSIRANGE_7);
	//-------------------------------------ZAD 2.3 OK-------------------------------------



	//------------------------------------- ZAD 2. OK -------------------------------------




	//-------------------------------------ZAD 3.-------------------------------------



	//-------------------------------------ZAD 3.1-------------------------------------
	// Wlaczenie HSI16
	SET_BIT(RCC->CR,RCC_CR_HSION);


	//Oczekiwanie na ustabilizowanie HSI16
	while(!READ_BIT(RCC->CR,RCC_CR_HSIRDY));
	//-------------------------------------ZAD 3.1 OK-------------------------------------


	//-------------------------------------ZAD 3.2-------------------------------------
	//SET SOURCE SYSCLK to HSI16
	MODIFY_REG(RCC->CFGR,RCC_CFGR_SW_Msk,SYSCLK_SOURCE_HSI16);
	//-------------------------------------ZAD 3.2 OK-------------------------------------


	//-------------------------------------ZAD 3.3-------------------------------------

	//Set Flash Latency for 4 Wait States
	MODIFY_REG(FLASH->ACR,FLASH_ACR_LATENCY,FLASH_ACR_LATENCY_4WS);

	//Configure PLL with Max Freq
	uint8_t N_Value = 10, M_Value = PLL_M_DIVIDER_1, R_Value = PLL_R_DIVIDER_2, PLL_Source = RCC_PLLCFGR_PLLSRC_HSI;
	SetPLLPar(PLL_Source,N_Value,M_Value,R_Value);
	//-------------------------------------ZAD 3.3 OK-------------------------------------

	//-------------------------------------ZAD 3.4-------------------------------------
	// Change Source SYSCLK to PLL
	MODIFY_REG(RCC->CFGR,RCC_CFGR_SW_Msk,SYSCLK_SOURCE_PLL);
	//-------------------------------------ZAD 3.4 OK-------------------------------------


	//-------------------------------------ZAD 3.5-------------------------------------
	//Set Prescaler for MCO
	Set_Prescaler_MCO(RCC_CFGR_MCOPRE_DIV2);

	//-------------------------------------ZAD 3.5 OK------------------------------------- ERR

	// Change Source SYSCLK to MSI
	MODIFY_REG(RCC->CFGR,RCC_CFGR_SW_Msk,SYSCLK_SOURCE_MSI);

	R_Value = PLL_R_DIVIDER_4;
	SetPLLPar(PLL_Source,N_Value,R_Value,M_Value);

	// Change Source SYSCLK to PLL
	MODIFY_REG(RCC->CFGR,RCC_CFGR_SW_Msk,SYSCLK_SOURCE_PLL);
	//-------------------------------------ZAD 3.6-------------------------------------

	//-------------------------------------ZAD 3.6 OK-------------------------------------


	//-------------------------------------ZAD 3 OK-------------------------------------


	//-------------------------------------ZAD 4-------------------------------------

	//-------------------------------------ZAD 4.1-------------------------------------
	//Enable PWR
	SET_BIT(RCC->APB1ENR1,RCC_APB1ENR1_PWREN);

	//Set Voltage scaling range selection
	MODIFY_REG(PWR->CR1,PWR_CR1_VOS,PWR_CR1_VOS_0);

	//Disable BackUp Domain write Protection
	SET_BIT(PWR->CR1,PWR_CR1_DBP);

	//Enable LSE
	SET_BIT(RCC->BDCR,RCC_BDCR_LSEON);

	//Wait for stability LSE
	while(!READ_BIT(RCC->BDCR,RCC_BDCR_LSERDY));

	//-------------------------------------ZAD 4.1 OK-------------------------------------


	//------------------------------------- ZAD 4.2-------------------------------------
	//Set Prescaler for MCO
	Set_Prescaler_MCO(RCC_CFGR_MCOPRE_DIV1);

	//SET MCO_OUTPUT_LSE
	MODIFY_REG(RCC->CFGR,RCC_CFGR_MCOSEL_Msk,MCO_OUTPUT_LSE << RCC_CFGR_MCOSEL_Pos);

	//-------------------------------------ZAD 4.2 OK-------------------------------------

	//-------------------------------------ZAD 4.3-------------------------------------
	//SET MCO_OUTPUT_LSE
	MODIFY_REG(RCC->CFGR,RCC_CFGR_MCOSEL_Msk,MCO_OUTPUT_MSI << RCC_CFGR_MCOSEL_Pos);

	// Enable PLSMSI
	SET_BIT(RCC->CR,RCC_CR_MSIPLLEN);

	//-------------------------------------ZAD 4.3 OK-------------------------------------


	//-------------------------------------ZAD 4 OK-------------------------------------

	// EXTRA ZAD

	//RTC Source
	MODIFY_REG(RCC->BDCR,RCC_BDCR_RTCSEL_Msk,RTC_SOURCE_LSE << RCC_BDCR_RTCSEL_Pos);

	//RTC ON
	SET_BIT(RCC->BDCR,RCC_BDCR_RTCEN);
	for(;;);
}

static void Set_Prescaler_MCO(uint32_t Prescaler)
{
	Config_MCO(DISABLE_MCO);
	MODIFY_REG(RCC->CFGR,RCC_CFGR_MCOPRE_Msk,Prescaler);
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

static void Config_MCO(uint8_t IsEnable)
{
	IsEnable == ENABLE_MCO ? SET_BIT(RCC ->AHB2ENR,RCC_AHB2ENR_GPIOAEN): CLEAR_BIT(RCC ->AHB2ENR,RCC_AHB2ENR_GPIOAEN);

	//3 NOZKA NA OUTPUT MODE3 01 (Alternative Function - Reset value AF0 (MCO))
	SET_BIT(GPIOA-> MODER,GPIO_MODER_MODE8_1);
	CLEAR_BIT(GPIOA-> MODER,GPIO_MODER_MODE8_0);

	//PUSH-PULL
	CLEAR_BIT(GPIOA-> OTYPER,GPIO_OTYPER_OT8);

	//SZYBKOSC MAX 180MHz
	SET_BIT(GPIOA-> OSPEEDR,GPIO_OSPEEDER_OSPEEDR8_0);
	SET_BIT(GPIOA-> OSPEEDR,GPIO_OSPEEDER_OSPEEDR8_1);
}

static void EnableMSI(void)
{
	// Wlaczenie MSI
	SET_BIT(RCC -> CR,RCC_CR_MSION);

	//Test czy MSI stabilne
	while (!READ_BIT(RCC -> CR,RCC_CR_MSIRDY));
}

static void ChangeMSIFreq(uint8_t MsiRange)
{

	while (!READ_BIT(RCC->CR,RCC_CR_MSIRDY) && READ_BIT(RCC->CR,RCC_CR_MSION));

	if (MsiRange < MSI_OUTPUT_8MHz && MsiRange > MSI_OUTPUT_1MHz)
	{
		SET_BIT(RCC->CR,RCC_CR_MSIRGSEL);
		MODIFY_REG(RCC->CR,RCC_CR_MSIRANGE_Msk,MsiRange << RCC_CR_MSIRANGE_Pos);
	}
	else
	{
		SET_BIT(RCC->CR,RCC_CR_MSIRGSEL);
		MODIFY_REG(RCC->CSR,RCC_CSR_MSISRANGE_Msk,MsiRange << RCC_CSR_MSISRANGE_Pos);
	}
}
