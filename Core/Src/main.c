/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 Arduino SA.
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
#include <string.h>

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
IWDG_HandleTypeDef hiwdg;
TIM_HandleTypeDef htim1;

#define NODE_BUTTONS    0x7C
#define NODE_BUZZER     0x3C
#define NODE_ENCODER    0x76
#define NODE_ENCODER_2  0x74
#define NODE_SMARTLEDS  0x6C
#define NUM_LEDS        8


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(uint8_t address);
static void MX_IWDG_Init(void);
static void MX_NVIC_Init(void);
static void MX_TIM1_Encoder_Init(void);
static void MX_TIM1_PWM_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);
static uint8_t readPinstraps();
static void transfer(uint8_t b);
void configurePins(void);

static volatile bool dataReceived = false;
static uint8_t i2c_buffer[128];

static uint8_t ADDRESS;
static uint8_t PINSTRAP_ADDRESS;

static int16_t encoder_last_reset_status = 0;

void JumpToBootloader (void)
{
  void (*SysMemBootJump)(void);

	uint32_t BootAddr  = 0x1FFF0000;

	/* Disable all interrupts */
	__disable_irq();
	/* Disable Systick timer */
	SysTick->CTRL = 0;
	/* Set the clock to the default state */
	HAL_RCC_DeInit();

	/* Clear Interrupt Enable Register & Interrupt Pending Register */
	NVIC->ICER[0]=0xFFFFFFFF;
	NVIC->ICPR[0]=0xFFFFFFFF;

	/* Re-enable all interrupts */
	__enable_irq();
	/* Set up the jump to boot loader address + 4 */
	SysMemBootJump = (void (*)(void)) (*((uint32_t *) ((BootAddr + 4))));

	/* Set the main stack pointer to the boot loader stack */
	__set_MSP(*(uint32_t *)BootAddr);
	/* Call the function to jump to boot loader location */
	SysMemBootJump();

	/* Jump is done successfully */
	while (1)
	{
		/* Code should never reach this loop */
	}
}

static  bool returnFlashStatus = false;
__attribute__((section(".userdata"))) uint8_t stuff[128];

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
  PINSTRAP_ADDRESS = readPinstraps();
  ADDRESS = PINSTRAP_ADDRESS;

  // read user data in flash
  // if data valid, replace ADDRESS with saved content
  if (stuff[0] != 0xFF && stuff[0] != 0x00 && ((stuff[0] ^ 0x33) == stuff[1])) {
    ADDRESS = stuff[0];
  }

  MX_I2C1_Init(ADDRESS);
  MX_IWDG_Init();
  /* Initialize interrupts */
  MX_NVIC_Init();

  configurePins();

  HAL_I2C_EnableListen_IT(&hi2c1);

  uint32_t endTone = 0;

  /* Infinite loop */
  while (1)
  {
    HAL_IWDG_Refresh(&hiwdg);
    if (endTone != 0 && HAL_GetTick() > endTone) {
      HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
      endTone = 0;
    }

    if (dataReceived) {

      if (i2c_buffer[0] == 'D' && i2c_buffer[1] == 'I' && i2c_buffer[2] == 'E') {
        JumpToBootloader();
      }
      if (i2c_buffer[0] == 'C' && i2c_buffer[1] == 'F') {
        uint8_t new_address = i2c_buffer[2];
        FLASH_EraseInitTypeDef pEraseInit = {
          .TypeErase = FLASH_TYPEERASE_PAGES,
          .Page = 7,
          .NbPages = 1,
        };
        uint32_t PageError;
        HAL_FLASH_Unlock();
        HAL_FLASHEx_Erase(&pEraseInit, &PageError);
        uint8_t data[8] = {new_address, new_address ^ 0x33, new_address, new_address ^ 0x33, new_address, new_address ^ 0x33, new_address, new_address ^ 0x33 };
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (uint32_t)&stuff[0], *((uint64_t*)data));
        returnFlashStatus = true;
        dataReceived = false;
        HAL_FLASH_Lock();
        NVIC_SystemReset();
      }

      switch (PINSTRAP_ADDRESS) {
        case NODE_BUTTONS:
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, i2c_buffer[0] == 0 ? GPIO_PIN_RESET: GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, i2c_buffer[1] == 0 ? GPIO_PIN_RESET: GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, i2c_buffer[2] == 0 ? GPIO_PIN_RESET: GPIO_PIN_SET);
          break;
        case NODE_BUZZER:
          uint32_t frequency;
          uint32_t duration;
          memcpy(&frequency, &i2c_buffer[0], sizeof(frequency));
          memcpy(&duration, &i2c_buffer[4], sizeof(duration));
          endTone = HAL_GetTick() + duration;

          // TODO: make the prescaler precise and configurable
          uint32_t val = 0xFFFF * 180 / frequency;
          TIM1->ARR = val;
          TIM1->CCR1 = val / 2;
          //TIM_OC_InitTypeDef sConfig;
          //HAL_TIM_PWM_ConfigChannel(&htim1, &sConfig, TIM_CHANNEL_1);
          HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
          break;
        case NODE_ENCODER:
        case NODE_ENCODER_2:
          memcpy(&encoder_last_reset_status, &i2c_buffer[0], 2);
          encoder_last_reset_status += __HAL_TIM_GET_COUNTER(&htim1);
          break;
        case NODE_SMARTLEDS:
          show_leds(i2c_buffer);
          break;
      }
      dataReceived = false;
    }
  }
}

void show_leds(uint8_t* data) {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); //DATA
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); //CLOCK
  transfer(0);
  transfer(0);
  transfer(0);
  transfer(0);
  for (int i = 0; i < NUM_LEDS * 4; i++) {
    transfer(data[i]);
  }
  for (int i = 0; i < (NUM_LEDS + 14)/16; i++)
  {
    transfer(0);
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
}

void transfer(uint8_t b) {
  for (int i = 7; i >= 0; i--) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, (b >> i) & 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
  }
}

void configurePins() {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  switch (PINSTRAP_ADDRESS) {
    case NODE_BUTTONS:
      GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      GPIO_InitStruct.Pull = GPIO_PULLUP;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

      GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
      break;
    case NODE_BUZZER:
      MX_TIM1_PWM_Init();
      break;
    case NODE_ENCODER:
    case NODE_ENCODER_2:
      GPIO_InitStruct.Pin = GPIO_PIN_2;
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      GPIO_InitStruct.Pull = GPIO_PULLUP;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
      MX_TIM1_Encoder_Init();
      HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
      break;
    case NODE_SMARTLEDS:
      // TODO: use an SPI like
      // Boost enable
      GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
      memset(i2c_buffer, 0xE0, NUM_LEDS * 4);
      show_leds(i2c_buffer);
    }
}

uint8_t populateBuffer() {
  if (returnFlashStatus) {
    returnFlashStatus = false;
    return 6;
  }
  i2c_buffer[0] = PINSTRAP_ADDRESS;
  switch (PINSTRAP_ADDRESS) {
    case NODE_BUTTONS:
      i2c_buffer[1] = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
      i2c_buffer[2] = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
      i2c_buffer[3] = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
      return 3;
    case NODE_ENCODER:
    case NODE_ENCODER_2:
      int16_t data = __HAL_TIM_GET_COUNTER(&htim1) - encoder_last_reset_status;
      memcpy(&i2c_buffer[1], &data, 2);
      i2c_buffer[3] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 0 ? GPIO_PIN_SET : GPIO_PIN_RESET;
      return 3;
    case NODE_SMARTLEDS:
      return NUM_LEDS * 4;
  }
  return 3;
}

uint8_t prepareRx() {
  switch (PINSTRAP_ADDRESS) {
    case NODE_BUTTONS:
      return 3;
    case NODE_BUZZER:
      return 8;
    case NODE_ENCODER:
    case NODE_ENCODER_2:
      return 4;
    case NODE_SMARTLEDS:
      return NUM_LEDS * 4;
  }
  return 3;
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {

}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  dataReceived = true;
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {

  if (TransferDirection == I2C_DIRECTION_RECEIVE) {
    uint8_t len = populateBuffer();
    HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, i2c_buffer, len + 1, I2C_FIRST_AND_LAST_FRAME);
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


static void MX_TIM1_Encoder_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM1_PWM_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim1);
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
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
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
