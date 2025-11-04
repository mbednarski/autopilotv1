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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Protocol constants
#define PROTO_START      0xAA
#define CMD_HDG_RESET    0x10
#define CMD_HDG_SET      0x11

/* ============================================================================
 * ENCODER CONFIGURATION
 * ============================================================================
 */

/**
 * @brief Encoder scaling divisor
 *
 * This constant adjusts how many encoder counts equal one HDG:SET unit.
 * It compensates for the encoder's pulses-per-revolution vs detents ratio.
 *
 * HOW TO DETERMINE THE RIGHT VALUE:
 * ---------------------------------
 * 1. Set to 1 initially (no scaling)
 * 2. Rotate encoder one detent (one "click")
 * 3. Observe how many HDG:SET packets are sent
 *    - If 1 packet with delta=+1 → Divisor = 1 (perfect, no change needed)
 *    - If 2 packets with delta=+1 → Divisor = 2
 *    - If 1 packet with delta=+2 → Divisor = 2
 *    - If 1 packet with delta=+4 → Divisor = 4 (most common with TI12 mode)
 * 4. Adjust ENCODER_COUNTS_PER_UNIT to match
 *
 * CURRENT CONFIGURATION: 4 (verified working)
 * -------------------------------------------
 * This encoder produces 4 hardware counts per detent click:
 * - TI12 encoder mode counts all edges on both channels (4× resolution)
 * - One detent = one complete quadrature cycle = 4 edges
 * - Dividing by 4 gives perfect 1:1 mapping (one click = ±1)
 *
 * GOAL: One physical detent click = one HDG:SET(±1) command
 *
 * EXAMPLES FOR OTHER ENCODERS:
 * - Value 1: No scaling (use raw encoder counts)
 * - Value 2: Divide counts by 2 (low-res encoders or TI1 mode)
 * - Value 4: Divide counts by 4 (CURRENT - typical for TI12 mode)
 */
#define ENCODER_COUNTS_PER_UNIT  4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* ============================================================================
 * ENCODER STATE VARIABLES
 * ============================================================================
 * These variables maintain the encoder's state between reads in the main loop.
 * They must be static to preserve values across function calls.
 */

// Encoder position tracking
// --------------------------
// TIM2 is a 32-bit hardware counter that automatically increments/decrements
// as the encoder rotates (increments for CW, decrements for CCW).
// We store the previous counter value to calculate the delta (change) since
// the last read. Using int32_t because TIM2 counter is 32-bit.
static int32_t encoder_last_count = 0;

// Encoder delta accumulator
// -------------------------
// Accumulates encoder movements between UART transmissions. This solves two
// problems:
//
// 1. PREVENTS DUPLICATE SENDS:
//    A single detent click might take 30-80ms mechanically, but the encoder
//    generates multiple electrical pulses during that time. Without accumulation,
//    each pulse would be sent separately (e.g., two +1 packets instead of one +2).
//    The accumulator collects all pulses, then sends once when rate limit allows.
//
// 2. ENABLES SCALING:
//    After accumulating raw counts, we divide by ENCODER_COUNTS_PER_UNIT to
//    convert hardware counts to logical units. This allows one detent click to
//    equal exactly ±1, regardless of the encoder's electrical characteristics.
//
// Example timeline (current encoder with 4 counts per detent):
//   t=0ms:  Detent starts, counter: 100→101, accum: 0→1, last: 101
//   t=10ms: Detent continues, counter: 101→102, accum: 1→2, last: 102
//   t=20ms: Detent continues, counter: 102→103, accum: 2→3, last: 103
//   t=30ms: Detent completes, counter: 103→104, accum: 3→4, last: 104
//   t=40ms: Rate limit OK, send accum/4 = 4/4 = +1, reset accum to 0
//
// Result: One physical detent = one HDG:SET(+1) command
static int32_t encoder_accumulated_delta = 0;

// Rate limiting for UART transmission
// ------------------------------------
// Without rate limiting, rapidly spinning the encoder could flood the UART
// with hundreds of packets per second. This timestamp tracks the last time
// we sent a command. We'll enforce a minimum interval (e.g., 20ms) between
// transmissions to prevent overload while still maintaining responsiveness.
static uint32_t encoder_last_send_time = 0;

// Button debouncing
// -----------------
// Physical buttons "bounce" when pressed - the electrical contact rapidly
// makes/breaks connection for 5-50ms, creating multiple triggers from one
// press. We use a timestamp to ignore subsequent triggers within the
// debounce window (50ms is typical).
static uint32_t button_last_press_time = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* ============================================================================
 * USER FUNCTION PROTOTYPES
 * ============================================================================
 */

/**
 * @brief Process rotary encoder and send HDG:SET commands
 *
 * This function reads the hardware encoder counter (TIM2), calculates the
 * delta (change) since the last read, and sends the appropriate HDG:SET
 * command via UART if the encoder has moved. Includes rate limiting to
 * prevent UART flooding during fast rotation.
 *
 * @note Called from main loop. Hardware timer counts encoder pulses
 *       automatically in background, so this function just reads the result.
 */
void ProcessEncoder(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Old test buffer - no longer needed
// uint8_t UART2_txBuffer[5] = {0xAA, 0x01, 0x32, 0x00, 0xFF};

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* ============================================================================
   * START ENCODER TIMER
   * ============================================================================
   * This starts TIM2 in encoder mode. The timer hardware will now automatically
   * count encoder pulses in the background:
   *
   * - Clockwise rotation: Counter increments (goes up)
   * - Counter-clockwise:  Counter decrements (goes down)
   * - TIM_CHANNEL_ALL:    Enables both Channel 1 (PA0) and Channel 2 (PA1)
   *
   * The counter runs continuously with NO CPU involvement. Our ProcessEncoder()
   * function will simply read the counter value periodically to check for
   * changes. This is why hardware encoder mode is so efficient!
   *
   * Technical details:
   * - TIM2 is configured in TI12 mode (4x resolution)
   * - Counts on all edges of both channels: rising A, falling A, rising B, falling B
   * - 32-bit counter (0 to 4,294,967,295) - virtually impossible to overflow
   * - Direction detection handled automatically by hardware
   */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* ========================================================================
     * MAIN LOOP - ENCODER PROCESSING
     * ========================================================================
     * This is a simple polling loop that continuously checks the encoder.
     *
     * HOW IT WORKS:
     * -------------
     * 1. ProcessEncoder() reads the TIM2 hardware counter (automatic counting)
     * 2. If the counter changed, it calculates delta and sends HDG:SET via UART
     * 3. HAL_Delay(10) paces the loop at ~100 Hz (100 reads per second)
     *
     * WHY 10ms DELAY IS FINE:
     * -----------------------
     * - TIM2 hardware counts encoder pulses continuously in the background
     * - No pulses are ever missed, even during the delay
     * - 100 Hz polling is plenty fast for human input (humans can't spin
     *   encoders faster than ~10 rotations/second)
     * - Lower CPU usage than busy-waiting (CPU can sleep during delay)
     *
     * BUTTON HANDLING:
     * ----------------
     * The encoder button is handled by interrupt (HAL_GPIO_EXTI_Callback),
     * not in this loop. When you press the button, the interrupt fires
     * immediately and sends HDG:RESET. No polling needed!
     *
     * EXPANSION TO 4 ENCODERS:
     * ------------------------
     * When you add more encoders (TIM3, TIM1, GPIO-based), you'll call
     * their respective ProcessEncoderX() functions here:
     *
     *   ProcessEncoder1();  // TIM2 (this one)
     *   ProcessEncoder2();  // TIM3
     *   ProcessEncoder3();  // TIM1
     *   ProcessEncoder4();  // GPIO EXTI
     *   HAL_Delay(10);
     *
     * Each encoder function will read its own timer/GPIO and send its
     * own command (HDG, ALT, SPD, etc.).
     */

    // Process encoder rotation and send HDG:SET commands
    ProcessEncoder();

    // Pace the main loop at ~100 Hz (10ms delay)
    // This is NOT blocking the encoder counting (hardware does that)
    // or button presses (interrupt handles those)
    HAL_Delay(10);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_KNOB1_Pin BTN_KNOB_VS_Pin */
  GPIO_InitStruct.Pin = BTN_KNOB1_Pin|BTN_KNOB_VS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_KNOB_ALT_Pin */
  GPIO_InitStruct.Pin = BTN_KNOB_ALT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_KNOB_ALT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Send HDG:RESET command via UART
 * Resets heading to default position (operand always 0x00)
 */
void SendHdgReset(void)
{
    static uint8_t txBuffer[4];

    txBuffer[0] = PROTO_START;
    txBuffer[1] = CMD_HDG_RESET;
    txBuffer[2] = 0x00;  // Operand always zero for reset
    txBuffer[3] = PROTO_START ^ CMD_HDG_RESET ^ 0x00;  // Checksum

    HAL_UART_Transmit_DMA(&huart2, txBuffer, 4);
}

/**
 * @brief Send HDG:SET command via UART
 * Sets heading delta with signed 8-bit value
 * @param delta: Signed delta value (-128 to +127)
 */
void SendHdgSet(int8_t delta)
{
    static uint8_t txBuffer[4];

    txBuffer[0] = PROTO_START;
    txBuffer[1] = CMD_HDG_SET;
    txBuffer[2] = (uint8_t)delta;  // Cast signed to unsigned for transmission
    txBuffer[3] = PROTO_START ^ CMD_HDG_SET ^ (uint8_t)delta;  // Checksum

    HAL_UART_Transmit_DMA(&huart2, txBuffer, 4);
}

/* ============================================================================
 * ENCODER PROCESSING FUNCTION
 * ============================================================================
 */

/**
 * @brief Process rotary encoder and send HDG:SET commands
 *
 * This function is the heart of the encoder interface. It performs three steps:
 * 1. Read the current TIM2 counter value (hardware counts encoder pulses)
 * 2. Calculate delta (change since last read)
 * 3. Send HDG:SET command if encoder moved (with rate limiting)
 *
 * HOW IT WORKS:
 * -------------
 * TIM2 is configured in encoder mode with TI12 (4x resolution). The hardware
 * automatically monitors PA0 (Channel A) and PA1 (Channel B):
 *
 * - When you rotate clockwise:  Counter increments (e.g., 100 -> 104)
 * - When you rotate counter-CW: Counter decrements (e.g., 100 -> 96)
 * - The hardware handles all the quadrature decoding logic
 *
 * We simply read the counter, compare it to the last value, and the difference
 * is our delta. This delta becomes the operand for the HDG:SET command.
 *
 * RATE LIMITING:
 * --------------
 * If the user spins the encoder very fast, we could generate hundreds of UART
 * packets per second. The rate limiter ensures we send at most one packet
 * every 20ms (50 Hz), which is:
 * - Fast enough for smooth control
 * - Slow enough to not overwhelm the UART/PC receiver
 *
 * WHY DELTA CALCULATION WORKS:
 * ----------------------------
 * Subtracting two int32_t values automatically handles wraparound. Even if
 * the counter wraps from 2^32-1 to 0, the math works correctly due to
 * two's complement arithmetic.
 */
void ProcessEncoder(void)
{
    // Rate limiting configuration
    // ---------------------------
    // Minimum time between UART transmissions in milliseconds.
    // 20ms = 50 Hz maximum update rate. Adjust if needed:
    //   - Lower (e.g., 10ms): More responsive, higher UART traffic
    //   - Higher (e.g., 50ms): Less responsive, lower UART traffic
    const uint32_t MIN_SEND_INTERVAL_MS = 20;

    // =========================================================================
    // STEP 1: Read current encoder position from hardware
    // =========================================================================
    // __HAL_TIM_GET_COUNTER reads the TIM2->CNT register directly.
    // This value changes automatically in hardware as the encoder rotates:
    // - Clockwise rotation: Counter increments
    // - Counter-clockwise:  Counter decrements
    // - No CPU intervention needed!
    //
    // Cast to int32_t for signed arithmetic (encoder can go both directions)
    int32_t current_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);

    // =========================================================================
    // STEP 2: Calculate raw delta (change since last read)
    // =========================================================================
    // This is how many counts the hardware encoder has moved since we last
    // checked. Could be positive (CW), negative (CCW), or zero (no movement).
    int32_t delta = current_count - encoder_last_count;

    // =========================================================================
    // STEP 3: Accumulate delta and update reference position
    // =========================================================================
    // KEY CHANGE: We ALWAYS update encoder_last_count when delta != 0,
    // regardless of whether we send via UART. This prevents seeing the
    // same encoder movement multiple times.
    //
    // WHY THIS MATTERS:
    // A single detent click might take 30-80ms mechanically. During that time,
    // the encoder generates multiple electrical pulses. Without immediate
    // reference update, we would see:
    //   t=0ms:  counter 100→101, delta=+1 → send
    //   t=10ms: counter 101→102, delta=+1 → send again! (duplicate)
    //
    // With immediate update:
    //   t=0ms:  counter 100→101, delta=+1, accum=+1, last=101
    //   t=10ms: counter 101→102, delta=+1, accum=+2, last=102
    //   t=20ms: Rate limit OK, send accum=+2, reset accum=0
    if (delta != 0)
    {
        encoder_accumulated_delta += delta;  // Add to accumulator
        encoder_last_count = current_count;  // Update reference immediately
    }

    // =========================================================================
    // STEP 4: Send accumulated delta when rate limit allows
    // =========================================================================
    // Only transmit when:
    // 1. We have accumulated movement (accumulated_delta != 0)
    // 2. Enough time has passed since last transmission (rate limiting)
    if (encoder_accumulated_delta != 0)
    {
        uint32_t current_time = HAL_GetTick();
        uint32_t time_since_last_send = current_time - encoder_last_send_time;

        if (time_since_last_send >= MIN_SEND_INTERVAL_MS)
        {
            // ================================================================
            // STEP 5: Scale accumulated delta by encoder divisor
            // ================================================================
            // Convert raw encoder counts to logical units.
            // This compensates for encoder PPR vs detent count mismatch.
            //
            // EXAMPLE (current encoder with ENCODER_COUNTS_PER_UNIT = 4):
            //   Physical: User clicks encoder 1 detent
            //   Hardware: Counter increments by 4 (four edges in TI12 mode)
            //   Accumulated: +4
            //   Scaled: +4 / 4 = +1
            //   Sent: HDG:SET(+1) ← One click = one unit!
            //
            // To adjust: Change ENCODER_COUNTS_PER_UNIT constant at top of file
            int32_t scaled_delta = encoder_accumulated_delta / ENCODER_COUNTS_PER_UNIT;

            // ================================================================
            // STEP 6: Only send if scaled delta is non-zero
            // ================================================================
            // After division, very small movements might round to zero.
            // Don't send a packet if there's nothing meaningful to send.
            // The fractional part stays in the accumulator for next time.
            if (scaled_delta != 0)
            {
                // ============================================================
                // STEP 7: Clamp to protocol range (-128 to +127)
                // ============================================================
                // UART protocol uses signed 8-bit operand. If user spins
                // encoder extremely fast, clamp to prevent overflow.
                if (scaled_delta > 127)
                    scaled_delta = 127;
                else if (scaled_delta < -128)
                    scaled_delta = -128;

                // ============================================================
                // STEP 8: Send the HDG:SET command via UART
                // ============================================================
                SendHdgSet((int8_t)scaled_delta);

                // ============================================================
                // STEP 9: Update timestamps and accumulator
                // ============================================================
                // Record send time for rate limiting
                encoder_last_send_time = current_time;

                // Subtract the UNSCALED amount we just sent from accumulator.
                // Why multiply back? Because we divided for sending, but the
                // accumulator tracks raw counts.
                //
                // Example:
                //   accumulated_delta = 5 raw counts
                //   scaled_delta = 5 / 2 = 2 (integer division)
                //   We sent: +2
                //   We used: 2 * 2 = 4 raw counts
                //   Remaining: 5 - 4 = 1 raw count (saved for next time)
                encoder_accumulated_delta -= (scaled_delta * ENCODER_COUNTS_PER_UNIT);
            }
            // If scaled_delta is 0 (very small movement), keep it in
            // accumulator. It will add up with future movements until
            // it's large enough to send.
        }
        // If rate limited, accumulated_delta stays as-is and will be
        // sent on the next cycle when rate limit permits.
    }
}

/* ============================================================================
 * BUTTON INTERRUPT HANDLER
 * ============================================================================
 */

/**
 * @brief GPIO External Interrupt Callback
 *
 * This function is called automatically by the HAL when ANY GPIO pin configured
 * as an external interrupt (EXTI) triggers. We check which pin caused the
 * interrupt and handle it accordingly.
 *
 * INTERRUPT vs POLLING:
 * ---------------------
 * Unlike the encoder (which we poll in the main loop), the button uses an
 * interrupt. This means:
 * - Button press triggers this function IMMEDIATELY (within microseconds)
 * - No need to continuously check button state in main loop
 * - More responsive than polling
 * - Better for infrequent events like button presses
 *
 * WHY DEBOUNCING IS CRITICAL:
 * ---------------------------
 * Physical buttons don't make clean electrical contact. When you press a
 * button, the contacts "bounce" - they make/break contact rapidly for 5-50ms:
 *
 * Physical button:      Press!
 *                         ↓
 * Electrical signal:   __|‾|_|‾|‾‾‾‾‾  (bounce for ~20ms, then stable)
 *                         ↑ ↑ ↑
 *                      Multiple edges = multiple interrupts!
 *
 * Without debouncing, one button press would trigger this interrupt 5-10 times,
 * sending multiple HDG:RESET commands. We prevent this by:
 * 1. Checking the time since the last valid press
 * 2. Ignoring presses that occur within the debounce window (50ms)
 *
 * INTERRUPT CONTEXT CONSIDERATIONS:
 * ----------------------------------
 * This function runs in interrupt context, not main loop context. Rules:
 * - Keep it FAST (microseconds, not milliseconds)
 * - Don't use HAL_Delay() - it blocks ALL interrupts
 * - Don't do heavy processing
 * - OK to call SendHdgReset() because it just triggers DMA (non-blocking)
 *
 * @param GPIO_Pin: The pin number that triggered the interrupt
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // Debounce configuration
    // ----------------------
    // Minimum time between valid button presses in milliseconds.
    // 50ms is typical for mechanical buttons. Adjust if needed:
    //   - Too low (e.g., 10ms): May still get double-triggers from bounce
    //   - Too high (e.g., 200ms): Button feels sluggish, hard to press twice
    const uint32_t DEBOUNCE_TIME_MS = 50;

    // Check if this interrupt was triggered by our encoder button
    // -----------------------------------------------------------
    // Multiple GPIO pins share the same interrupt handler (EXTI4_15 handles
    // pins 4-15). We need to check which specific pin triggered this call.
    //
    // BTN_KNOB1_Pin is defined in main.h as GPIO_PIN_4 (PA4)
    if (GPIO_Pin == BTN_KNOB1_Pin)
    {
        // Get current system time
        // -----------------------
        // HAL_GetTick() returns milliseconds since boot (from SysTick timer).
        // Safe to call from interrupt context (just reads a volatile variable).
        uint32_t current_time = HAL_GetTick();

        // Calculate time elapsed since last valid button press
        // -----------------------------------------------------
        // If this is the first press after boot, button_last_press_time is 0,
        // so time_since_last_press will be equal to current_time (large value),
        // which will pass the debounce check. Perfect!
        uint32_t time_since_last_press = current_time - button_last_press_time;

        // Debounce check
        // --------------
        // Only process this button press if enough time has passed since the
        // last valid press. This filters out the bouncing edges.
        if (time_since_last_press >= DEBOUNCE_TIME_MS)
        {
            // Valid button press - send HDG:RESET command
            // --------------------------------------------
            // Calls the existing SendHdgReset() function which builds the
            // 4-byte protocol packet: [0xAA | 0x10 | 0x00 | checksum]
            //
            // SendHdgReset() uses HAL_UART_Transmit_DMA(), which is non-blocking
            // (just starts the DMA transfer and returns immediately). This is
            // safe to call from interrupt context.
            SendHdgReset();

            // Update debounce timestamp
            // -------------------------
            // Record this press time. Any subsequent interrupts within the next
            // DEBOUNCE_TIME_MS will be ignored.
            button_last_press_time = current_time;
        }
        // If we're still within the debounce window, silently ignore this
        // interrupt. It's just the button bouncing.
    }

    // If the interrupt was from a different GPIO pin (e.g., PC13 - the blue
    // user button on the Nucleo board), we ignore it. You could add handling
    // for other buttons here in the future:
    //
    // else if (GPIO_Pin == B1_Pin) {
    //     // Handle blue button
    // }
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
