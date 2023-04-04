/*
 * PYRO.h
 *
 *  Created on: Nov 1, 2022
 *      Author: pinheird
 */

#ifndef SRC_PYRO_H_
#define SRC_PYRO_H_


/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : pyro.h
 * @brief          : Macros and defines  used for the pyro implementation
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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define THSD_REPEAT 15
#define CMD_SERIN_SIZE  25
#define PYRO_SERIN_PIN  GPIO_PIN_2
#define PYRO_SERIN_PINR	(GPIO_PIN_2<<16)
#define PYRO_SERIN_PORT GPIOC
/* USER CODE END PD */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//union PyrtoSerInCmd_u
//{
//	uint32_t CmdVar;
//	_Bool CmdVector[32];
//	struct CmdBits{

//	}PyroSerIn_t;
//};

//union PyroSerIn_t
//{
//	struct
//	{
//		uint32_t bit25_32_Reserved: 8;
//		uint32_t bit24_Threshold : 1;
//		uint32_t bit23_Threshold : 1;
//		uint32_t bit22_Threshold : 1;
//		uint32_t bit21_Threshold : 1;
//		uint32_t bit20_Threshold : 1;
//		uint32_t bit19_Threshold : 1;
//		uint32_t bit18_Threshold : 1;
//		uint32_t bit17_Threshold : 1;
//		uint32_t bit16_BlindTime : 1;
//		uint32_t bit15_BlindTime : 1;
//		uint32_t bit14_BlindTime : 1;
//		uint32_t bit13_BlindTime : 1;
//		uint32_t bit12_PulseCounter : 1;
//		uint32_t bit11_PulseCounter : 1;
//		uint32_t bit10_WindowTime : 1;
//		uint32_t bit09_WindowTime : 1;
//		uint32_t bit08_OperationMode : 1;
//		uint32_t bit07_OperationMode : 1;
//		uint32_t bit06_SignalSource : 1;
//		uint32_t bit05_SignalSource : 1;
//		uint32_t bit04_Reserved : 1;
//		uint32_t bit03_Reserved : 1;
//		uint32_t bit02_HPFCutOff : 1;
//		uint32_t bit01_Reserved : 1;
//		uint32_t bit00_CountMode : 1;
//	} b;                                /*!< Structure used for bit  access */
//	uint32_t w;                         /*!< Type used for word access */
//	_Bool bitvec[32];					/*!< Type used for bit vector access */
//} ;

uint8_t SerInBuffer[25] = {1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0};
/* USER CODE END PTD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TSL()  asm("NOP")
#define TSH()  asm("NOP")
#define TSHD() asm("NOP");\
		asm("NOP");\
		asm("NOP");\
		asm("NOP");\
		asm("NOP");\
		asm("NOP");\
		asm("NOP");\
		asm("NOP");\
		asm("NOP");\
		asm("NOP")
#define PYRO_SERIN_PIN_LOW()   PYRO_SERIN_PORT->BSRR=PYRO_SERIN_PINR
#define PYRO_SERIN_PIN_HIGH()  PYRO_SERIN_PORT->BSRR=PYRO_SERIN_PIN
/* USER CODE END PM */

#endif /* SRC_PYRO_H_ */
