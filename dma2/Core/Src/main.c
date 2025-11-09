/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "ssd1306.h"
#include <string.h>
#include <stdio.h>
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
// UART RX buffer for 3-byte command frames from PC
// Frame format: [START=0x88, COMMAND, CHECKSUM]
uint8_t uart_rx_buffer[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct {
    // Hardware configuration
    // ----------------------
    uint16_t pin;                 // GPIO pin number (e.g., GPIO_PIN_1)
    GPIO_TypeDef* port;           // GPIO port (e.g., GPIOA, GPIOB)

    // Protocol configuration
    // ----------------------
    uint8_t cmd_press;            // Command ID for button press (e.g., 0x40)

    // Runtime state
    // -------------
    uint32_t last_press_time;     // Timestamp of last press (for debouncing)

    // DMA buffer
    // ----------
    // CRITICAL: Each button MUST have its own buffer to prevent DMA race conditions
    uint8_t tx_buffer[4];         // Dedicated UART transmit buffer
} Button_t;

#define PROTO_START      0xAA
#define CMD_BTN_HDG_PRESS   0x40
#define CMD_BTN_VS_PRESS   0x41
#define CMD_BTN_APPR_PRESS   0x42
#define CMD_BTN_ALT_PRESS   0x43

#define COUNT_BUTTONS 4
static Button_t standalone_buttons[COUNT_BUTTONS] = {
    {BTN_0_Pin,  BTN_0_GPIO_Port, CMD_BTN_HDG_PRESS,  0, {0}},  // AP button (PB1)
    {BTN_1_Pin,  BTN_1_GPIO_Port, CMD_BTN_VS_PRESS, 0, {0}},  // HDG button (PB2)
    {BTN_2_Pin,  BTN_2_GPIO_Port, CMD_BTN_APPR_PRESS,  0, {0}},  // VS button (PB3)
    {BTN_3_Pin,  BTN_3_GPIO_Port, CMD_BTN_ALT_PRESS, 0, {0}}   // ALT button (PB4) - TODO: configure in CubeMX as BTN_ALT
};

void SendButtonCommand(Button_t* btn, uint8_t cmd, uint8_t operand)
{
    uint8_t* txBuffer = btn->tx_buffer;

    txBuffer[0] = PROTO_START;
    txBuffer[1] = cmd;
    txBuffer[2] = operand;
    txBuffer[3] = PROTO_START ^ cmd ^ operand;

    HAL_UART_Transmit_DMA(&huart2, txBuffer, 4);  // ← DMA transfer
}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//
//{
//
//  if (GPIO_Pin == BTN_0_Pin) {
//	  SendButtonCommand(&standalone_buttons[0], 0x40, 0x00);
//  }
//
//}

/**
 * @brief Update OLED display with received command information
 * @param command: Received command byte
 */
void Display_UpdateCommand(uint8_t command)
{
    char cmd_text[20];
    char code_text[20];

    // Format command name
    if (command == 0x10) {
        snprintf(cmd_text, sizeof(cmd_text), "CMD: LED ON");
    } else if (command == 0x11) {
        snprintf(cmd_text, sizeof(cmd_text), "CMD: LED OFF");
    } else {
        snprintf(cmd_text, sizeof(cmd_text), "CMD: UNKNOWN");
    }

    // Format command code
    snprintf(code_text, sizeof(code_text), "Code: 0x%02X", command);

    // Update display
    SSD1306_Clear();
    SSD1306_SetCursor(0, 0);
    SSD1306_Puts("UART Monitor");
    SSD1306_SetCursor(0, 1);
    SSD1306_Puts(cmd_text);
    SSD1306_SetCursor(0, 2);
    SSD1306_Puts(code_text);
    SSD1306_UpdateScreen();
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  // Initialize SSD1306 OLED Display
  SSD1306_Init(&hi2c1);
  SSD1306_Clear();
  SSD1306_SetCursor(0, 0);
  SSD1306_Puts("UART Monitor");
  SSD1306_SetCursor(0, 1);
  SSD1306_Puts("Ready...");
  SSD1306_UpdateScreen();

  // Start DMA reception for 3-byte command frames from PC
  HAL_UART_Receive_DMA(&huart2, uart_rx_buffer, 3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  for(int i =0; i< COUNT_BUTTONS; i++)
	  {
		  GPIO_PinState state = HAL_GPIO_ReadPin(standalone_buttons[i].port, standalone_buttons[i].pin);
		  if(state == GPIO_PIN_RESET)
		  {
			  SendButtonCommand(&standalone_buttons[i], standalone_buttons[i].cmd_press, 0x00);
		  }
	  }

//	  GPIO_PinState btn0_state = HAL_GPIO_ReadPin(BTN_0_GPIO_Port, BTN_0_Pin);
//	  if(btn0_state == GPIO_PIN_RESET)
//	  {
//		  SendButtonCommand(&standalone_buttons[0], 0x40, 0x00);
//	  }
//
//	  GPIO_PinState btn1_state = HAL_GPIO_ReadPin(BTN_1_GPIO_Port, BTN_1_Pin);
//	  if(btn1_state == GPIO_PIN_RESET)
//	  {
//		  SendButtonCommand(&standalone_buttons[1], 0x41, 0x00);
//	  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  /* DMA1_Ch2_3_DMA2_Ch1_2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch2_3_DMA2_Ch1_2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch2_3_DMA2_Ch1_2_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_0_Pin BTN_1_Pin BTN_2_Pin BTN_3_Pin */
  GPIO_InitStruct.Pin = BTN_0_Pin|BTN_1_Pin|BTN_2_Pin|BTN_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief UART RX Complete Callback
 * @note  Called when DMA has received a complete 3-byte frame
 *        Frame format: [START=0x88, COMMAND, CHECKSUM]
 *        Commands: 0x10=LED ON, 0x11=LED OFF
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    // Extract frame bytes
    uint8_t start = uart_rx_buffer[0];
    uint8_t command = uart_rx_buffer[1];
    uint8_t checksum = uart_rx_buffer[2];

    // Validate frame: check START byte and checksum
    if (start == 0x88 && checksum == (0x88 ^ command))
    {
      // Process valid commands
      if (command == 0x10)
      {
        // Command 0x10: Turn LED ON
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
      }
      else if (command == 0x11)
      {
        // Command 0x11: Turn LED OFF
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
      }
      // Unknown commands are silently ignored

      // Update OLED display with received command
      Display_UpdateCommand(command);
    }
    // Invalid frames (wrong START or checksum) are silently ignored

    // Restart DMA reception for next frame
    HAL_UART_Receive_DMA(&huart2, uart_rx_buffer, 3);
  }
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
