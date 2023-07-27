/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#define R_ON HAL_GPIO_WritePin(R_GPIO_Port, R_Pin, 1)
#define G_ON HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, 1)
#define B_ON HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, 1)
#define R_OFF HAL_GPIO_WritePin(R_GPIO_Port, R_Pin, 0)
#define G_OFF HAL_GPIO_WritePin(G_GPIO_Port, G_Pin, 0)
#define B_OFF HAL_GPIO_WritePin(B_GPIO_Port, B_Pin, 0)
#define BUTTON_STATE HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin)
#define DEBOUNCE_TIME 50
#define PRESS 0
#define RELEASE 1
typedef enum
{
	IDLE = 0,
	WAIT_BUTTON_UP,
	WAIT_PRESS_TIMEOUT,
	WAIT_CLICK_TIMEOUT,
	WAIT_HOLD_TIMEOUT
} my_state_t;

my_state_t state = IDLE;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t id_state_LED = 0;
uint8_t button_interrupt = 0;
uint8_t Mode_State = 0;\
uint32_t t_timeout = 0;
uint8_t BT;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void state_test_ON()
{
	R_ON;
	G_ON;
	B_ON;
}

void state_test_OFF()
{
	R_OFF;
	G_OFF;
	B_OFF;
}

void state_run(uint8_t id_state)
{
	switch (id_state)
	{
		case 1:
			R_ON;
			G_OFF;
			B_OFF;
			break;

		case 2:
			G_ON;
			R_OFF;
			B_OFF;
			break;

		case 3:
			B_ON;
			R_OFF;
			G_OFF;
			break;

		default:
			R_OFF;
			G_OFF;
			B_OFF;
			break;
	}
}
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
  /* USER CODE BEGIN 2 */
  HAL_InitTick(TICK_INT_PRIORITY); // Reset HAL_GetTick() to 0
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    switch (state)
    {
      case IDLE:
        if (button_interrupt == 1 && BUTTON_STATE == 0)
        {
          state = WAIT_PRESS_TIMEOUT;
          t_timeout = HAL_GetTick() + 50;
        }
        break;

      case WAIT_PRESS_TIMEOUT:
        if (BUTTON_STATE == 0 && HAL_GetTick() > t_timeout)
        {
          state = WAIT_CLICK_TIMEOUT;
          t_timeout = HAL_GetTick() + 250;
        }
        if (BUTTON_STATE == 1 && HAL_GetTick() <= t_timeout)
        {
          state = IDLE;
          button_interrupt = 0;
        }
        break;

      case WAIT_CLICK_TIMEOUT:
        if (BUTTON_STATE == 1 && HAL_GetTick() <= t_timeout)
        {
          id_state_LED++;
          if (id_state_LED > 3) id_state_LED = 1;
          state_run(id_state_LED);
          state = IDLE;
          button_interrupt = 0;
        }

        if (BUTTON_STATE == 0 && HAL_GetTick() > t_timeout)
        {
          state = WAIT_HOLD_TIMEOUT;
          t_timeout = HAL_GetTick() + 2700;
        }
        break;

      case WAIT_HOLD_TIMEOUT:
        if (BUTTON_STATE == 0 && HAL_GetTick() > t_timeout)
        {
          if (id_state_LED != 0) id_state_LED = 0;
          state_run(id_state_LED);
          state = IDLE;
          button_interrupt = 0;
        }
        break;

      default:
          state = IDLE;
          break;
    }
    BT = BUTTON_STATE;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, G_Pin|B_Pin|R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : G_Pin B_Pin R_Pin */
  GPIO_InitStruct.Pin = G_Pin|B_Pin|R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	button_interrupt = 1;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
