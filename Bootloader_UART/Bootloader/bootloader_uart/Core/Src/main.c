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

#include "circular_buffer.h"
#include "flash.h"
#include "fota.h"
#include "uart.h"
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
UART_HandleTypeDef huart1;
DMA_HandleTypeDef  hdma_usart1_rx;
DMA_HandleTypeDef  hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void        SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uart_status status = UART_ERROR;
uint8_t     frame_data[127];

uint8_t aTextInfo[] = "\r\n--------- Start Program ---------\r\n";
uint8_t aRXBufferUser[RX_BUFFER_SIZE];

uint8_t aRXbufferA[RX_BUFFER_SIZE];
uint8_t aRXbufferB[RX_BUFFER_SIZE];

uint8_t *p_buffer_for_reception;
uint8_t *p_buffer_for_user;

__IO uint32_t num_received_chars;

hexa_struct data_frame;

uint8_t buffer[128];

cbuffer_t cb;

void start_reception(void);

uint8_t frame_rx_data[128];

uint16_t high_address = 0x0800;

uint32_t full_address;

uint8_t last_checksum = 0x00;
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  uart_transmit_string((uint8_t *) "Hello\n\r");
  cb_init(&cb, buffer, 128);

  start_reception();
  /* USER CODE END 2 */
  // flash_erase(0x08004000);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    cb_read(&cb, frame_rx_data, cb_data_count(&cb));

    /* USER CODE END WHILE */
    data_frame.start_code = *frame_rx_data;

    data_frame.byte_count = *(frame_rx_data + 1);

    data_frame.address = ((*(frame_rx_data + 2) << 8) & 0xFF00) | (*(frame_rx_data + 3) & 0x00FF);

    data_frame.record_type = *(frame_rx_data + 4);

    if (data_frame.record_type == RECORD_DATA)
    {
      for (uint8_t i = 0; i < data_frame.byte_count; i++)
      {
        data_frame.data[i] = *(frame_rx_data + 5 + i);
      }

      data_frame.checksum = *(frame_rx_data + (uint32_t) (data_frame.byte_count + 5));

      full_address = (uint32_t) (((high_address << 16) & 0xFFFF0000) | (data_frame.address & 0x0000FFFF));

      if (last_checksum != data_frame.checksum)
      {
        // flash_write(full_address, (uint32_t*)data_frame.data, (data_frame.byte_count/4));
        last_checksum = data_frame.checksum;
      }
    }
    else if (data_frame.record_type == RECORD_EX_LIR_ADDRESS)
    {
      high_address        = ((*(frame_rx_data + 5) << 8) & 0xFF00) | (*(frame_rx_data + 6));
      data_frame.checksum = *(frame_rx_data + 7);
    }

    last_checksum = data_frame.checksum;
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
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType =
    RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{
  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance          = USART1;
  huart1.Init.BaudRate     = 115200;
  huart1.Init.WordLength   = UART_WORDLENGTH_8B;
  huart1.Init.StopBits     = UART_STOPBITS_1;
  huart1.Init.Parity       = UART_PARITY_NONE;
  huart1.Init.Mode         = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */
void print_info(UART_HandleTypeDef *huart, uint8_t *String, uint16_t Size)
{
  if (HAL_UART_Transmit(huart, String, Size, 100) != HAL_OK)
  {
    Error_Handler();
  }
}

void start_reception(void)
{
  /* Initializes buffer swap
   */
  p_buffer_for_reception = aRXbufferA;
  p_buffer_for_user      = aRXbufferB;

  /* Print user info on COMPORT*/
  print_info(&huart1, aTextInfo, COUNTOF(aTextInfo));

  /*Initialize RX sequence using Reception To Idle event API */
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, aRXBufferUser, RX_BUFFER_SIZE) != HAL_OK)
  {
    Error_Handler();
  }
}

void user_data_treatment(UART_HandleTypeDef *huart, uint8_t *p_data, uint16_t size)
{
  uint8_t *p_buff = p_data;
  uint8_t  i;

  cb_write(&cb, p_data, size);

  for (i = 0; i < size; i++)
  {
    while (!(__HAL_UART_GET_FLAG(huart, UART_FLAG_TXE)))
    {
    }
    huart->Instance->DR = *p_buff;
    p_buff++;
  }
}

/**
 * @brief User implementation of Reception Event Callback
 *
 * @param huart UART handle
 * @param size Number of data available in application reception buffer
 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  static uint8_t old_pos = 0;
  uint8_t       *p_temp;
  uint8_t        i;

  /*Check if number of received data in reception buffer has changed*/
  if (size != old_pos)
  {
    /*Check if of index in reception buffer has be increased */
    if (size > old_pos)
    {
      /*Current position is higher than previous one*/
      num_received_chars = size - old_pos;
      /*Coppy received data in "User" buffer for evacution*/
      for (i = 0; i < num_received_chars; i++)
      {
        p_buffer_for_user[i] = aRXBufferUser[old_pos + i];
      }
    }
    else
    {
      /*Current position is lower than previous one: end of buffer*/
      /*First coppy data from current potion till end of buffer*/
      num_received_chars = RX_BUFFER_SIZE - old_pos;
      /*Coppy received data in "User" buffer for evacuation*/
      for (i = 0; i < num_received_chars; i++)
      {
        p_buffer_for_user[i] = aRXBufferUser[i + old_pos];
      }

      /*Check and continue with beginning of buffer*/
      if (size > 0)
      {
        for (i = 0; i < size; i++)
        {
          p_buffer_for_user[i] = aRXBufferUser[i];
        }
        num_received_chars = num_received_chars + size;
      }
    }
    /*Process received data*/
    user_data_treatment(huart, p_buffer_for_user, num_received_chars);

    /*Swap buffers for next bytes to be processed*/
    /*p_buffer_for_user <--> p_buffer_for_reception*/
    p_temp                 = p_buffer_for_user;
    p_buffer_for_user      = p_buffer_for_reception;
    p_buffer_for_reception = p_temp;
  }

  /*Update old_pos*/
  old_pos = size;
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

#ifdef USE_FULL_ASSERT
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
