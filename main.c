/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define OUT1 GPIO_PIN_11
#define OUT2 GPIO_PIN_5
#define OUT3 GPIO_PIN_6
#define OUT4 GPIO_PIN_7
#define OUT5 GPIO_PIN_6
#define OUT6 GPIO_PIN_7
#define OUT7 GPIO_PIN_9
#define OUT8 GPIO_PIN_8

uint16_t pins [8] = {OUT1,OUT2,OUT3,OUT4,OUT5,OUT6,OUT7,OUT8};
GPIO_TypeDef* GPIOS[8] = {GPIOB, GPIOA, GPIOA, GPIOA, GPIOB, GPIOC, GPIOA, GPIOA};

typedef enum{FALSE =0,TRUE =! FALSE} bool; // in C type bool is not existing so i have to create it on my own

bool flag = 0;
uint16_t del =200;
int max = sizeof(pins) / sizeof(pins[0]);



#define BUZZER GPIO_PIN_10 // PIN PA4
uint16_t buzzHigh =50;
uint16_t buzzLow =100;




/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

//RGB1
  HAL_Delay(buzzLow);
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
  HAL_Delay(buzzLow);
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);

  HAL_Delay(buzzLow);
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
  HAL_Delay(buzzLow);
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);

  HAL_Delay(buzzLow);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
  HAL_Delay(buzzLow);
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
//RGB2
  HAL_Delay(buzzLow);
  HAL_GPIO_WritePin(RGB2_GREEN_GPIO_Port, RGB2_GREEN_Pin, GPIO_PIN_SET);
  HAL_Delay(buzzLow);
  HAL_GPIO_WritePin(RGB2_GREEN_GPIO_Port, RGB2_GREEN_Pin, GPIO_PIN_RESET);
  HAL_Delay(buzzLow);
  HAL_GPIO_WritePin(RGB2_BLUE_GPIO_Port, RGB2_BLUE_Pin, GPIO_PIN_SET);
  HAL_Delay(buzzLow);
  HAL_GPIO_WritePin(RGB2_BLUE_GPIO_Port, RGB2_BLUE_Pin, GPIO_PIN_RESET);
  HAL_Delay(buzzLow);
  HAL_GPIO_WritePin(RGB2_RED_GPIO_Port, RGB2_RED_Pin, GPIO_PIN_SET);
  HAL_Delay(buzzLow);
  HAL_GPIO_WritePin(RGB2_RED_GPIO_Port, RGB2_RED_Pin, GPIO_PIN_RESET);
//Buzzer
    HAL_Delay(buzzLow);
    HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
    HAL_Delay(buzzHigh);
    HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
    HAL_Delay(buzzLow);
    HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
    HAL_Delay(buzzHigh);
    HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
    HAL_Delay(buzzLow);
    HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
    HAL_Delay(buzzHigh);
    HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {



uint32_t currentTick =HAL_GetTick();

	  if (flag == 0){
	 		  for (int i = 0; i < max; ++i){
	 		  HAL_GPIO_TogglePin(GPIOS[i], pins[i]);
	 		  HAL_Delay(del);
	 		  if (i == max - 1 && HAL_GPIO_ReadPin(GPIOS[i], pins[i]) == GPIO_PIN_SET) {
	 		              flag = 1;
	 		}
	 	  }
	 	}
	 	  else {
	 	      for (int i = max - 1; 0 <= i; --i) {
	 	          HAL_GPIO_TogglePin(GPIOS[i], pins[i]);
	 	          HAL_Delay(del);
	 	          if (i == max - 1 && HAL_GPIO_ReadPin(GPIOS[i], pins[i]) == GPIO_PIN_SET) {
	 	              flag = 0;
	 	          }
	 	      }
	 	  }

if (currentTick >5000){

HAL_Delay(100);
HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
HAL_Delay(100);
HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);

HAL_Delay(110);
HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
HAL_Delay(110);
HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);

HAL_Delay(120);
HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
HAL_Delay(120);
HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
currentTick=0;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|LD3_Pin|LD4_Pin|LD8_Pin
                          |LD7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RGB1_RED_Pin|LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RGB2_GREEN_Pin|RGB2_BLUE_Pin|BUZZER_Pin|LD1_Pin
                          |RGB1_GREEN_Pin|RGB1_BLUE_Pin|RGB2_RED_Pin|LD5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIP_1_Pin DIP_5_Pin */
  GPIO_InitStruct.Pin = DIP_1_Pin|DIP_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LD3_Pin LD4_Pin LD8_Pin
                           LD7_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LD3_Pin|LD4_Pin|LD8_Pin
                          |LD7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RGB1_RED_Pin LD6_Pin */
  GPIO_InitStruct.Pin = RGB1_RED_Pin|LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RGB2_GREEN_Pin RGB2_BLUE_Pin BUZZER_Pin LD1_Pin
                           RGB1_GREEN_Pin RGB1_BLUE_Pin RGB2_RED_Pin LD5_Pin */
  GPIO_InitStruct.Pin = RGB2_GREEN_Pin|RGB2_BLUE_Pin|BUZZER_Pin|LD1_Pin
                          |RGB1_GREEN_Pin|RGB1_BLUE_Pin|RGB2_RED_Pin|LD5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DIP_2_Pin DIP_3_Pin */
  GPIO_InitStruct.Pin = DIP_2_Pin|DIP_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
