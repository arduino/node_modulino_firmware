/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
IWDG_HandleTypeDef hiwdg;

#define NODE_BUTTONS    0x7C
#define NODE_BUZZER     0x3C


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(uint8_t address);
static void MX_IWDG_Init(void);
static void MX_NVIC_Init(void);
static uint8_t readPinstraps();

static volatile bool dataReceived = false;
static uint8_t i2c_buffer[128];

static uint8_t ADDRESS;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  ADDRESS = readPinstraps();
  MX_I2C1_Init(ADDRESS);
  //MX_IWDG_Init();
  /* Initialize interrupts */
  MX_NVIC_Init();

  configurePins();

  HAL_I2C_EnableListen_IT(&hi2c1);

  /* Infinite loop */
  while (1)
  {
    //HAL_IWDG_Refresh(&hiwdg);
    if (dataReceived) {
      switch (ADDRESS) {
        case NODE_BUTTONS:
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, i2c_buffer[0] == 0 ? GPIO_PIN_RESET: GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, i2c_buffer[1] == 0 ? GPIO_PIN_RESET: GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, i2c_buffer[2] == 0 ? GPIO_PIN_RESET: GPIO_PIN_SET);
          break;
        case NODE_BUZZER:
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, i2c_buffer[0] == 0 ? GPIO_PIN_RESET: GPIO_PIN_SET);
          break;
      }
      dataReceived = false;
    }
  }
}

void configurePins() {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  switch (ADDRESS) {
    case NODE_BUTTONS:
      GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
      break;
    case NODE_BUZZER:
      GPIO_InitStruct.Pin = GPIO_PIN_0;
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
      break;
    }
}

uint8_t populateBuffer() {
  switch (ADDRESS) {
    case NODE_BUTTONS:
      i2c_buffer[0] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
      i2c_buffer[1] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
      i2c_buffer[2] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
      return 3;
      break;
  }
}

uint8_t prepareRx() {
  switch (ADDRESS) {
    case NODE_BUTTONS:
      return 3;
    case NODE_BUZZER:
      return 1;
  }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {

}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  dataReceived = true;
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {

  if (TransferDirection == I2C_DIRECTION_RECEIVE) {
    uint8_t len = populateBuffer();
    HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, i2c_buffer, len, I2C_FIRST_AND_LAST_FRAME);
  } else {
    uint8_t len = prepareRx();
    HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, i2c_buffer, len, I2C_FIRST_AND_LAST_FRAME);
  }
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
    HAL_I2C_EnableListen_IT(&hi2c1);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  /** Error_Handler() function is called when error occurs.
    * 1- When Slave doesn't acknowledge its address, Master restarts communication.
    * 2- When Master doesn't acknowledge the last data transferred, Slave doesn't care
    */
  if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF)
  {
    Error_Handler();
  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV4;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* I2C1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_IRQn);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(uint8_t address)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00100413;
  hi2c1.Init.OwnAddress1 = address;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_ENABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
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
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
}

static uint8_t readPinstraps() {

  // address = PA6 | PA7 | PA8 | PC14 | PC15 | PF2
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  return (
    HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) << 6 |
    HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) << 5 |
    HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) << 4 |
    HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) << 3 |
    HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) << 2 |
    HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_2) << 1);
}


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
