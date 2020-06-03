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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
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
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//=====================FOR TRANSFER PRINTF TO UART=======================
int _write(int fd, char *ptr, int len) {
  HAL_UART_Transmit(&huart3, (uint8_t*) ptr, len, 0xFFFF);

  return len;
}
int _read(int fd, char *ptr, int len) {

  *ptr = 0x00; // Flush the character buffer

  HAL_UART_Receive(&huart3, (uint8_t*) ptr, 1, 0xFFFF);

  return 1;
}
////http://www.count-zero.ru/2016/stm32_uart/
//=====================END TRANSFER PRINTF TO UART=======================


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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  uint16_t i = 0;/*вообще это ужасная практика создание переменных, они должны объявляться
  как можно ближе к их спользованию, но так как бывают микроконтроллеры с очень маленькой
  памятью, и ВОЗМОЖНО, чтобы не тратить время на их создание, создают как бы глобальную 
  переменную(в нашем случае видимость i ограничена областью функции main). 
  Имена 'i' и 'j' исторически используются в цыклах, но и в цыклах более целесообразно
  их названия заменять на более понятные имена, чтобы при чтении кода было очень понятно за
  что она отвечает.
  Ниже будет два похожих цыкла, в последнем используется i, разница читабельности на лицо*/
  volatile uint16_t j = 0;
  char sendingStringByUart[255];

  /* вывод в uart значение предделителя системной частоты, значение счетчика периода и 
   * значение для сравнения с четчиком периода.*/
  snprintf(sendingStringByUart, 255,
      "TIM->TIM prescaler register = %lu; "
      "TIM->TIM auto-reload register = %lu;© "
      "TIM3->TIM capture/compare register = %lu;\r\n",
      TIM3->PSC, TIM3->ARR, TIM3->CCR1);
  HAL_UART_Transmit(&huart3, (uint8_t*) sendingStringByUart, strlen(sendingStringByUart), 1000);

  /*вывод в uart значение системной частоты*/
  snprintf(sendingStringByUart, 255, "HAL_RCC_GetSysClockFreq () = %lu; \r\n",
      HAL_RCC_GetSysClockFreq());
  HAL_UART_Transmit(&huart3, (uint8_t*) sendingStringByUart, strlen(sendingStringByUart), 1000);
  
  uint16_t delayForChangingTimersSetting = 5000;/*оспользуется, чтобы создать мигание*/
  int bool = 0;/*используется как логическая переменная для смены параметров таймера*/

  while (1)
  {
    for (uint16_t compareValue = 0; compareValue <= (TIM3->ARR); compareValue++) { 
      // сравнение с CounterPeriod(называют еще auto-reload register)
      /* 
       * переменной 'i' будем задавать ширину заполнения ШИМ сигнала или можно назвать процент
       * заполнения, если рассматривать отношение '100% * i/(TIM3->ARR)'.
       *  ___
       * |   |___... заполнение 50%, тут 'compareValue'равна половине значения содержащегося 
       * в регистре CounterPeriod(auto-reload register) compareValue=(TIM3->ARR)/2
       * 
       *  _______
       * |       ... заполнение 100%
       *
       * */
      TIM3->CCR1 = compareValue;
      
      for (j = 0; j < delayForChangingTimersSetting; j++)
        __NOP();
      /*__NOP(); бездействие, наверно, это плохой способ, так как процессор, наверно, в этот тик
       * не может выполнять полезную работу, если, к примеру у нас все будет работать во FreeRTOS.
       * Во FreeRTOS есть для этого свои функции не блокирующие процессор, чтобы заменить этот цикл*/
    }

    snprintf(sendingStringByUart, 255,
        "TIM->TIM prescaler register = %lu; TIM->TIM auto-reload register = %lu; TIM3->TIM capture/compare register = %lu;\r\n",
        TIM3->PSC, TIM3->ARR, TIM3->CCR1);
    HAL_UART_Transmit(&huart3, (uint8_t*) sendingStringByUart, strlen(sendingStringByUart), 1000);

    for (i = TIM3->ARR; i > 0; i--) {
      TIM3->CCR1 = i;
      for (j = 0; j < delayForChangingTimersSetting; j++)
        __NOP();
    }
    snprintf(sendingStringByUart, 255,
        "TIM->TIM prescaler register = %lu; TIM->TIM auto-reload register = %lu; TIM3->TIM capture/compare register = %lu;\r\n",
        TIM3->PSC, TIM3->ARR, TIM3->CCR1);
    HAL_UART_Transmit(&huart3, (uint8_t*) sendingStringByUart, strlen(sendingStringByUart), 1000);



    if (bool) {
      bool = 0;
      TIM3->PSC = 7199; /*меняем prescaler таймера*/
      TIM3->ARR = 999; /*меняем CounterPeriod(называют еще auto-reload register)*/
      snprintf(sendingStringByUart, 255,
          "TIM->TIM prescaler register = %lu; TIM->TIM auto-reload register = %lu; TIM3->TIM capture/compare register = %lu;\r\n",
          TIM3->PSC, TIM3->ARR, TIM3->CCR1);
      HAL_UART_Transmit(&huart3, (uint8_t*) sendingStringByUart, strlen(sendingStringByUart), 1000);
    } else {
      bool = 1;
      TIM3->PSC = 71; /*меняем prescaler таймера*/
      TIM3->ARR = 999; /*меняем CounterPeriod(называют еще auto-reload register)*/
      snprintf(sendingStringByUart, 255,
          "TIM->TIM prescaler register = %lu; TIM->TIM auto-reload register = %lu; TIM3->TIM capture/compare register = %lu;\r\n",
          TIM3->PSC, TIM3->ARR, TIM3->CCR1);
      HAL_UART_Transmit(&huart3, (uint8_t*) sendingStringByUart, strlen(sendingStringByUart), 1000);
    }

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
