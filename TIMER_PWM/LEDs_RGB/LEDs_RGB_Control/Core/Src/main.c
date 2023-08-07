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
#define BUTTON_STATE HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin)

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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t en_LED = 0;
uint8_t i_LED = 0;
uint8_t di_LED = 0;
uint8_t en_fade_mode = 0;
uint32_t bright = 100;
uint8_t button_interrupt = 0;
uint32_t t_timeout = 0;

uint32_t add_pointer_value = 0;
uint32_t bright_count_R;
uint32_t bright_count_G;
uint32_t bright_count_B;

uint8_t COLOR_TABLE[100] =
{
		10, 0, 0,
		20, 35, 0,
		40, 0, 27,
		60, 10, 0,
		80, 25, 0,
		100, 0, 125,
		120, 255, 0,
		140, 10, 25,
		160, 0, 0,
		180, 25, 0,
		200, 0, 124,
		220, 30, 57,
};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM4)
	{
		en_LED =  1 - en_LED;
	}

	if (htim->Instance == TIM1 && en_fade_mode == 1)
	{
		if (di_LED == 0)
		{
			i_LED++;
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, i_LED*79/99);
			if (i_LED > bright_count_R) di_LED = 1;
		}

		if (di_LED == 1)
		{
			i_LED--;
			  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, i_LED*79/99);
			if (i_LED < 1) di_LED = 0;
		}
	}
}

void __1HZ_Timer_LED_Run()
{
	 if (en_LED == 1) __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	 if (en_LED == 0) __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 80);
}

void __1HZ_PWM_RGB_Setup()
{
	  __HAL_TIM_SET_AUTORELOAD(&htim2, 1999);
	  __HAL_TIM_SET_PRESCALER(&htim2, 3999);

	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 999);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 999);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 999);
}

void __Fade_LED_Setup()
{
	  __HAL_TIM_SET_AUTORELOAD(&htim1, 99);
	  __HAL_TIM_SET_PRESCALER(&htim1, 999);

	  __HAL_TIM_SET_AUTORELOAD(&htim2, 79);
	  __HAL_TIM_SET_PRESCALER(&htim2, 399);

	  en_fade_mode = 1;
}

void __RGB_PWM_Setup(uint32_t freq)
{
	uint32_t PSC = 3999;
	uint32_t max_count = 8000000/(freq*(PSC + 1)) - 1;

	__HAL_TIM_SET_AUTORELOAD(&htim2, max_count);
	__HAL_TIM_SET_PRESCALER(&htim2, PSC);

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, max_count/2);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, max_count/2);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, max_count/2);

}

void __LED_TIMER_Setup(uint32_t freq)
{
	uint32_t PSC = 3999;
	uint32_t max_count = 8000000/((freq*2)*(PSC + 1)) - 1;

	__HAL_TIM_SET_AUTORELOAD(&htim4, max_count);
	__HAL_TIM_SET_PRESCALER(&htim4, PSC);
}

void __LED_TIMER_Run(uint32_t bright)
{
	uint32_t PSC = 399;
	uint32_t freq = 250;  //Hz
	uint32_t max_count = 8000000/(freq*(PSC + 1)) - 1;
	uint32_t bright_count = max_count*bright/100;

	if (en_LED == 1) __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, max_count + 1);
	if (en_LED == 0) __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, max_count - bright_count);
}

void __RGB_Color(uint8_t *Color_Table)
{
	  uint32_t PSC = 99;
	  uint32_t max_count_RGB = 8000000/(250*(PSC + 1)) - 1;
	  bright_count_R = *Color_Table*max_count_RGB/255;
	  bright_count_G = *(Color_Table + 1)*max_count_RGB/255;
	  bright_count_B = *(Color_Table + 2)*max_count_RGB/255;

	  //Set fade freq 80 Hz
	  __HAL_TIM_SET_AUTORELOAD(&htim1, 99);
	  __HAL_TIM_SET_PRESCALER(&htim1, 999);

	  //Set base timer for change pwm RGB - 250 Hz
	  __HAL_TIM_SET_AUTORELOAD(&htim2, max_count_RGB);
	  __HAL_TIM_SET_PRESCALER(&htim2, PSC);

	  //Set PWM for each R,G,B
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, bright_count_R);  //Red
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, bright_count_G);  //Green
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, bright_count_B);  //Blue
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  //Turn on Timer 4 for Control LED period
  HAL_TIM_Base_Start_IT(&htim2);

  //Turn on Timer 3 for LED power
  HAL_TIM_Base_Start_IT(&htim3);

  //Turn on Timer 4 for Control LED period
  HAL_TIM_Base_Start_IT(&htim4);

  //Turn on Timer 1 for Fade LED period
  HAL_TIM_Base_Start_IT(&htim1);

  //Start turn ON PWM
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  //Set start state for LED1, LED2, LED3
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 80);  //OFF
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 80);  //OFF
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 80);  //OFF

  //Set start state for R, G, B
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);  //OFF
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);  //OFF
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);  //OFF

  /* USER CODE END 2 */
  HAL_InitTick(TICK_INT_PRIORITY); // Reset HAL_GetTick() to 0

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	//  __LED_TIMER_Run(bright);
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
	          add_pointer_value = add_pointer_value + 3;
	          if (add_pointer_value > 36) add_pointer_value = 0;
	          __RGB_Color(COLOR_TABLE + add_pointer_value);
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
	          state = IDLE;
	          button_interrupt = 0;
	        }
	        break;

	      default:
	          state = IDLE;
	          break;
	    }


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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 999;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 79;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = 39;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 3999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
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
