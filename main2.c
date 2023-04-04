/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pyro.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* LPTIM Defines */
#define PeriodValue				77
#define PulseValue				5

/* SPI DMA Defines */
#define size    5

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* LPTIM Variables */
LL_LPTIM_InitTypeDef LPTIM_Handle_t;
LL_GPIO_InitTypeDef GPIO_Handle_t;

__IO FlagStatus forcePulseFlag = RESET;
FlagStatus spiFlag = RESET;

/* SPI DMA Variables */
uint16_t bufferTx[size] = {0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA};
uint16_t bufferRx[size];

union PyroSerIn_t SerInCmd;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
	/*Local variables for the state machine */
	LL_GPIO_InitTypeDef GPIO_InitStruct;

	uint8_t u8CommandSerInCount = 0;
	volatile uint8_t vu8TshdCount=THSD_REPEAT;
	SerInCmd.w = 0x01FE0010; // Ensure Operation Mode is set to Forced Readout, bits [8:7]Cmd.b.bit24_Threshold = 1;
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
  MX_DMA_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);			// Enable clock on PORT A

	GPIO_InitStruct.Pin 			= LL_GPIO_PIN_10 | LL_GPIO_PIN_5;
	GPIO_InitStruct.Mode 			= LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed 			= LL_GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Pull 			= LL_GPIO_PULL_NO;
	GPIO_InitStruct.OutputType 		= LL_GPIO_OUTPUT_PUSHPULL;

	LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* SPI DMA Receive Buffer */

	/* SPI DMA Transmit Buffer */
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_3, (uint32_t)(&bufferTx[0]),
			(uint32_t)LL_SPI_DMA_GetRegAddr(SPI1),
			LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3));

	/* LPTIM1 Settings and Initialization */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_LPTIM1);					// Enable the LPTIM1 Clock

	LL_LPTIM_SetClockSource(LPTIM1, LL_LPTIM_CLK_SOURCE_INTERNAL);			// Set clock source to internal
	LL_LPTIM_SetPrescaler(LPTIM1, LL_LPTIM_PRESCALER_DIV1);					// Set Prescaler to 0 (Div 1)
	LL_LPTIM_SetUpdateMode(LPTIM1, LL_LPTIM_UPDATE_MODE_IMMEDIATE);			// Set Update Mode to Immediate
	LL_LPTIM_SetCounterMode(LPTIM1, LL_LPTIM_COUNTER_MODE_INTERNAL);		// Set Counter Mode to Internal
	LL_LPTIM_TrigSw(LPTIM1);												// Set the Trigger to Software
	LL_LPTIM_EnableIT_ARRM(LPTIM1);											// Enable the Auto Reload Register Match Interrupt
	LL_LPTIM_EnableIT_CMPM(LPTIM1);											// Enable the Compare Match Interrupt

	NVIC_SetPriority(LPTIM1_IRQn, 0);										// Set the NVIC Priority to 0
	NVIC_EnableIRQ(LPTIM1_IRQn);											// Enable the Interrupt at NVIC

	LL_LPTIM_Enable(LPTIM1);												// Enable the Low Power Timer 1

	LL_LPTIM_SetAutoReload(LPTIM1, PeriodValue);							// Set the Auto Reload Register value
	while(!LL_LPTIM_IsActiveFlag_ARROK(LPTIM1));							// Wait for flag to be OK
	LL_LPTIM_ClearFlag_ARROK(LPTIM1);										// Clear the OK Flag

	LL_LPTIM_SetCompare(LPTIM1,PulseValue);									// Set the Compare Register
	while(!LL_LPTIM_IsActiveFlag_CMPOK(LPTIM1));							// Wait for flag to be OK
	LL_LPTIM_ClearFlag_CMPOK(LPTIM1);										// Clear the OK Flag

	//will loop for the entire configuration register content
	u8CommandSerInCount = CMD_SERIN_SIZE;
	while(u8CommandSerInCount)
	{
		u8CommandSerInCount--;
		//SERIN HIGH
		PYRO_SERIN_PIN_HIGH();
		//wait for at least tSH time
		TSH();
		//uses the command table to select the given bit state (0 or 1)
		if(SerInCmd.w & BIT(u8CommandSerInCount))
		{
			PYRO_SERIN_PIN_HIGH(); // bit 1
		}
		else
		{
			PYRO_SERIN_PIN_LOW(); // bit 0
		}
		//ensures SERIN tSDH time is respected using polling method
		while (vu8TshdCount)
		{
			TSHD();
			vu8TshdCount--;
		}
		vu8TshdCount=THSD_REPEAT;
		//SERIN LOW
		PYRO_SERIN_PIN_LOW();
		//wait for tSL time
		TSL();
	}
	TSLT();

	/* Change clock Settings */
	LL_RCC_MSI_SetRange(LL_RCC_MSIRANGE_4);

	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);

	LL_LPTIM_StartCounter(LPTIM1, LL_LPTIM_OPERATING_MODE_CONTINUOUS);		// Start the LPTIM1 Counter
	GPIOA->BSRR = LL_GPIO_PIN_5;

	while(forcePulseFlag == RESET);
	while(forcePulseFlag == SET);

//	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);		// Enter in Stop Mode


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		/* Wait for Wake UP on Falling Edge */
		while(!forcePulseFlag);

		/* Start SPI & DMA */


		/* Enter in Low Power Run mode */
//		HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);

		/* Wait for Wake UP on Rising Edge */
		while(forcePulseFlag);

		/* Enter in STOP mode */
//		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    /* USER CODE END WHILE */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPTIM1;
  PeriphClkInit.LptimClockSelection = RCC_LPTIM1CLKSOURCE_LSI;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
	__disable_irq();
	while (1)
	{
	}
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
