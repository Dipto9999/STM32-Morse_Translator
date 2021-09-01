/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

enum { NUMBER_CHARACTERS = 36 };

enum LENGTH_RANGES {
  LENGTH_PHRASE_MIN = 1,
  LENGTH_PHRASE_MAX = 9,
  LENGTH_MORSE_MAX = 6,
  LENGTH_PROMPT_MIN = 1,
  LENGTH_PROMPT_MAX = 100
};

enum MORSE_DURATION {
  DOT_DURATION = 100,
  DASH_DURATION = 300,
  SAME_LETTER_DURATION = 100,
  DIFFERENT_LETTER_DURATION = 200,
  DIFFERENT_WORD_DURATION = 200
};

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/*******************/
/* UART Tx and Rx. */
/*******************/

int getPhrase();
void newPhrase();

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

/*****************************/
/* Perform Morse Conversion. */
/*****************************/

int returnMorseForCharacter(char input_character);

/***********************/
/* GPIO Output on LED. */
/***********************/

void outputPhrase();
void outputCharacter(char* morse_conversion, int length_morse);

void outputDot();
void outputDash();

void changeLetter();
void changeWord();

/*****************/
/* Error Message */
/*****************/

void invalidInput();

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define Green_LED_Pin GPIO_PIN_5
#define Green_LED_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define TRUE 1
#define FALSE 0
#define ERROR -1

#define DOT '.'
#define DASH '-'

#define CLEAR_PUTTY "\033[3J"

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
