/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

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
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* Array of Supported Characters for Morse Code. */
static char supported_characters[NUMBER_CHARACTERS] = {
    'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O',
    'P','Q','R','S','T','U','V','W','X','Y','Z',
    '1','2','3','4','5','6','7','8','9','0'
};

/*
 * Array of Morse Code Conversions.
 * "." -> DOT
 * "-" -> DASH
 */
static char morse_code[NUMBER_CHARACTERS][LENGTH_MORSE_MAX] = {
    ".-","-...","-.-.","-..",".","..-.","--.","....","..",".---", "-.-",
    ".-..","--","-.","---",".--.","--.-",".-.","...","-","..-","...-",".--","-..-","-.--","--..",".----",
    "..---","...--","....-",".....","-....","--...","---..","----.","-----"
};


/* Declare Buffer to Receive Data from UART Peripheral. */
static uint8_t* tx_buff;
/* Declare Variable for Message Prompt. */
static char message_prompt[LENGTH_PROMPT_MAX];

/* Declare Buffer to Transmit Data to UART Peripheral. */
static uint8_t rx_buff[LENGTH_PROMPT_MAX];
/* Declare Variable for ASCII String to Convert into Morse Code. */
static char phrase[LENGTH_PHRASE_MAX];

static uint8_t phrase_length = FALSE;

static uint8_t position_phrase = FALSE;
static uint8_t remaining_phrase = TRUE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DMA_Init(void);
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

  /* Note: STMCubeMX will configured peripherals in following order.
           The UART peripheral is unable to receive data using the DMA with
           the following configuration. */

  // MX_GPIO_Init();
  // MX_USART2_UART_Init();
  // MX_DMA_Init();

  /* Note: We will call the configuration functions in the following order to
     initialize the DMA variables prior to the UART variables. */

  // MX_GPIO_Init();
  // MX_DMA_Init();
  // MX_USART2_UART_Init();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  if (getPhrase() == ERROR) Error_Handler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while(TRUE) {
    if (remaining_phrase <= FALSE) {
      outputPhrase();
      newPhrase();
    }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  huart2.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Green_LED_Pin */
  GPIO_InitStruct.Pin = Green_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Green_LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*******************/
/* UART Tx and Rx. */
/******************/

/*
 * Get Phrase From User to Convert into Morse Code.
 *
 * PARAM: huart is a pointer to the UART handle struct; Size is a uint16_t
 *        representing the number of characters sent to the UART peripheral.
 * PRE: UART peripheral is initialized with appropriate parameters for
 *      DMA receive and transfer functionality with IDLE line detection.
 * POST: characters received in rx_buff are retransmitted to UART peripheral
 *       (intended to be in real time);
 *       contents of rx_buff copied into phrase at position_phrase;
 *       UART peripheral ready to receive rest of phrase if incomplete
 * RETURN: TRUE if successful phrase acquisition; otherwise ERROR.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart->Instance == USART2) {
    /* Echo User Input to Terminal in DMA Mode. */
    HAL_UART_Transmit_DMA(
        huart, // UART Handler
        rx_buff, // Data Buffer
        Size // Size
    );
    /* Disable DMA Channel 7 Transfer Half Transfer Complete Interrupt. */
    __HAL_DMA_DISABLE_IT(&hdma_usart2_tx, DMA_IT_HT);

    /* Copy Contents of String Prior to Morse Code Translation. */
    strncpy(
      (phrase + position_phrase), // Destination Buffer
      rx_buff, // Source Buffer
      Size // Size
    );

    /* Update Phrase Position. */
    position_phrase += Size;
    remaining_phrase = phrase_length - position_phrase;

    if (remaining_phrase >= TRUE) {
      /* Restart the DMA to Receive the Rest of String. */
      HAL_UARTEx_ReceiveToIdle_DMA(
          huart, // UART Handler
          rx_buff, // Data Buffer
          (remaining_phrase * sizeof(char)) // Size
      );
      /* Disable DMA Channel 6 Half Transfer Complete Interrupt. */
      __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    }
  }
}

/*
 * Get Phrase From User to Convert into Morse Code.
 *
 * PARAM: VOID
 * PRE: UART peripheral is initialized with appropriate parameters for
 *      DMA receive and transfer with IDLE line detection.
 * POST: instructions sent over to user via UART peripheral and phrase is acquired.
 * RETURN: TRUE if successful phrase acquisition; otherwise ERROR.
 */
int getPhrase() {
  /* Initial Message Prompt. */
  tx_buff = (uint8_t*)(strcpy(message_prompt, "\n\r****Terminal Accepts Only Capital Letters and Arabic Numerals****\n\n\r"));
  /* Transmit Message Prompt in Polling Mode. */
  HAL_UART_Transmit(
      &huart2, // UART Handler
      tx_buff, // Data Buffer
      (strlen(message_prompt) * sizeof(*message_prompt)), // Size
      HAL_MAX_DELAY // Timeout
  );

  /* Modify Message Prompt. */
  tx_buff = (uint8_t*)(strcpy(message_prompt, "\n\rSize of Phrase to Convert into Morse Code\n\r(Must Be Greater Than 0, Less Than 9) : "));
  /* Transmit Message Prompt in Polling Mode. */
  HAL_UART_Transmit(
      &huart2, // UART Handler
      tx_buff, // Data Buffer
      (strlen(message_prompt) * sizeof(*message_prompt)), // Size
      HAL_MAX_DELAY // Timeout
  );

  /* Receive Length of String to Convert into Morse Code in Polling Mode. */
  HAL_UART_Receive(
      &huart2, // UART Handler
      rx_buff, // Data Buffer
      sizeof(char), // Size
      HAL_MAX_DELAY // Timeout
  );
  /* Echo User Input to Terminal in Polling Mode. */
  HAL_UART_Transmit(
      &huart2, // UART Handler
      rx_buff, // Data Buffer
      sizeof(char), // Size
      HAL_MAX_DELAY // Timeout
  );

  /* Check if Input is a Valid Single Digit Positive Integer. */
  phrase_length = atoi(rx_buff);
  if ((phrase_length < LENGTH_PHRASE_MIN) || (phrase_length > LENGTH_PHRASE_MAX)) return ERROR;

  /* Modify Message Prompt. */
  tx_buff = (uint8_t*)(strcpy(message_prompt, "\n\rEnter Phrase : "));
  /* Prompt for String to Convert into Morse Code in Polling Mode. */
  HAL_UART_Transmit(
      &huart2, // UART Handler
      tx_buff, // Data Buffer
      (strlen(message_prompt) * sizeof(*message_prompt)), // Size
      HAL_MAX_DELAY // Timeout
  );

  /* Receive String to Convert into Morse Code. */
  HAL_UARTEx_ReceiveToIdle_DMA(
      &huart2, // UART Handler
      rx_buff, // Data Buffer
      (phrase_length * sizeof(char)) // Size
  );
  /* Disable DMA Channel 6 Half Transfer Complete Interrupt. */
  __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

  return TRUE;
}

/*
 * Prepare to Get New Phrase from UART Peripheral.
 *
 * PARAM: VOID
 * PRE: morse code translation of previous phrase output onto STM32.
 * POST: new phrase is acquired from UART peripheral to translate into morse code.
 * RETURN: VOID
 */
void newPhrase() {
  /* Default Phrase Position. */
  remaining_phrase = TRUE;
  position_phrase = FALSE;

  /* Modify Message Prompt. */
  tx_buff = (uint8_t*)(strcpy(message_prompt, "\n\n\rNew Message...\n\r"));
  /* Transmit Message Prompt in Polling Mode. */
  HAL_UART_Transmit(
      &huart2, // UART Handler
      tx_buff, // Data Buffer
      (strlen(message_prompt) * sizeof(*message_prompt)), // Size
      HAL_MAX_DELAY // Delay
  );

  HAL_Delay(DASH_DURATION);

  /* Clear Screen (Assume PuTTY SSH Client). */
  tx_buff = (uint8_t*)(strcpy(message_prompt, CLEAR_PUTTY));
  /* Transmit Message Prompt in Polling Mode. */
  HAL_UART_Transmit(
      &huart2, // UART Handler
      tx_buff, // Data Buffer
      (strlen(message_prompt) * sizeof(*message_prompt)), // Size
      HAL_MAX_DELAY // Delay
  );

  if (getPhrase() == ERROR) Error_Handler();
}

/*****************************/
/* Perform Morse Conversion. */
/*****************************/

/*
 * Identify Index in Which Character is Stored.
 *
 * PARAM: input_character is a char to be converted to morse code.
 * PRE: input_character is an element of supported_characters array.
 * POST:  VOID
 * RETURN: index in which character is stored in supported_characters array; otherwise ERROR.
 */
int returnMorseForCharacter(char input_character) {
    for (int array_index = 0; array_index < NUMBER_CHARACTERS; array_index++) {
        if (supported_characters[array_index] == input_character) return array_index;
    }
    return ERROR;
}

/***********************/
/* GPIO Output on LED. */
/***********************/

/*
 * Convert Morse Code Conversion of Phrase on STM32 Green LED.
 *
 * PARAM: VOID
 * PRE: LENGTH_PHRASE_MIN <= phrase_length <= LENGTH_PHRASE_MAX
 * POST: morse code translation of phrase is output onto the stm32.
 * RETURN: VOID
 */
void outputPhrase() {
    /*********************************/
    /******** Local Variables ********/
    /*********************************/

    char letter;

    int morse_length = FALSE;

    int phrase_index = FALSE;
    int morse_index = FALSE;

    /* Output Morse Conversion Onto STM32. */
    for (phrase_index = 0; phrase_index < phrase_length; phrase_index++) {
        letter = phrase[phrase_index];

        if (letter == ' ')  {
            changeWord();
        }
        else {
            morse_index = returnMorseForCharacter(letter);

            // Output Morse Code Only for Valid Input.
            if (morse_index == ERROR) Error_Handler();

            morse_length = strlen(morse_code[morse_index]);
            outputCharacter(morse_code[morse_index], morse_length);
        }
    }
}

/*
 * Display the Morse Code Conversion For an ASCII Character on the STM 32 Green LED.
 *
 * PARAM: morse_conversion is a string respresenting the morse message to be displayed;
 *        morse_length is an integer representing the length.
 * PRE: VOID
 * POST: led blinks morse conversion
 * RETURN: VOID
 */
void outputCharacter(char* morse_conversion, int morse_length) {
    for (int morse_index = 0; morse_index < morse_length; morse_index++) {
        if (morse_conversion[morse_index] == DOT) outputDot();
        else if (morse_conversion[morse_index] == DASH) outputDash();
        else Error_Handler();
    }
    // Delay Display after Each Character to Differentiate Letters on LED.
    changeLetter();
}

/*
 * Outputs a Dot on the STM32 Green LED.
 *
 * PARAM: VOID
 * PRE: NULL (no pre-conditions)
 * POST:  led turned on for DOT_DURATION.
 * RETURN:  VOID
 */
void outputDot() {
    HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_SET);
    HAL_Delay(DOT_DURATION);
    HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_RESET);

    /* Follow With a Short Delay Between Dot/Dash in Same Letter. */
    HAL_Delay(SAME_LETTER_DURATION);
}

/*
 * Outputs a Dash on the STM32 Green LED.
 *
 * PARAM: VOID
 * PRE: NULL (no pre-conditions)
 * POST:  led turned on for DASH_DURATION.
 * RETURN:  VOID
 */
void outputDash() {
    HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_SET);
    HAL_Delay(DASH_DURATION);
    HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_RESET);

    /* Follow With a Short Delay Between Dot/Dash in Same Letter. */
    HAL_Delay(SAME_LETTER_DURATION);
}

/*
 * Delay Display on the STM32 Green LED to Indicate Change in Letter.
 *
 * PARAM: VOID
 * PRE: NULL (no pre-conditions)
 * POST:  led turned off for DIFFERENT_LETTER_DURATION.
 * RETURN: VOID
 */
void changeLetter() {
  HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_RESET);
  HAL_Delay(DIFFERENT_LETTER_DURATION);
}

/*
 * Delay Display on the STM32 Green LED to Indicate Change in Word.
 *
 * PARAM: VOID
 * PRE: NULL (no pre-conditions)
 * POST:  NULL (no side-effects)
 * RETURN:  VOID
 */
void changeWord() {
  HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_RESET);
  HAL_Delay(DIFFERENT_WORD_DURATION);
}

/*****************/
/* Error Message */
/*****************/

/*
 * Send an Error Message to the Terminal and Reset System.
 *
 * PARAM: VOID
 * PRE: invalid input received in terminal.
 * POST: stm32 system undergoes software reset.
 * RETURN: VOID
 */
void invalidInput() {
  /*******************/
  /* Local Variables */
  /*******************/

  tx_buff = (uint8_t*)(strcpy(message_prompt, "\n\n\rERROR! Invalid Configuration...\n\rRestarting System...\n\r"));
  /* Transmit Error Message in Polling Mode. */
  HAL_UART_Transmit(
    &huart2, // UART Handler
    tx_buff, // Data Buffer
    (strlen(message_prompt) * sizeof(char)), // Size
    HAL_MAX_DELAY // Timeout
  );

  /* Clear Screen (Assume PuTTY SSH Client). */
  tx_buff = (uint8_t*)(strcpy(message_prompt, CLEAR_PUTTY));
  /* Transmit Message Prompt in Polling Mode. */
  HAL_UART_Transmit(
      &huart2, // UART Handler
      tx_buff, // Data Buffer
      (strlen(message_prompt) * sizeof(*message_prompt)), // Size,
      HAL_MAX_DELAY // Delay
  );

  HAL_Delay(DASH_DURATION);

  /* STM32 System Reset. */
  HAL_NVIC_SystemReset();
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  invalidInput();
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
