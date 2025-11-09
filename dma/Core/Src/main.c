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

/**
 * @brief Encoder state and configuration structure
 *
 * This structure encapsulates everything needed for one rotary encoder:
 * - Hardware configuration (which timer, which button pin)
 * - Protocol configuration (which commands to send)
 * - Runtime state (position tracking, debouncing, rate limiting)
 * - Calibration (counts-per-detent scaling)
 *
 * WHY USE A STRUCTURE?
 * --------------------
 * Instead of having separate variables for each encoder (encoder1_count,
 * encoder2_count, etc.), we group related data together. Benefits:
 *
 * 1. ORGANIZATION: All encoder data in one place
 * 2. REUSABILITY: Same ProcessEncoder() function works for all encoders
 * 3. SCALABILITY: Adding encoder #4 = just add one line to array
 * 4. MAINTAINABILITY: Change structure once, affects all encoders
 *
 * This is how professional embedded code handles multiple similar peripherals!
 */
typedef struct {
    // Hardware configuration
    // ----------------------
    TIM_HandleTypeDef* htim;      // Pointer to hardware timer (htim1/htim2/htim3)
    uint16_t button_pin;          // GPIO pin number for button (e.g., GPIO_PIN_4)
    GPIO_TypeDef* button_port;    // GPIO port for button (e.g., GPIOA, GPIOB)

    // Protocol configuration
    // ----------------------
    uint8_t cmd_reset;            // Command ID for reset (e.g., 0x10 for HDG)
    uint8_t cmd_set;              // Command ID for set delta (e.g., 0x11 for HDG)

    // Runtime state variables
    // -----------------------
    // These track the encoder's current state and are updated by ProcessEncoder()
    int32_t last_count;           // Last timer counter value (for delta calculation)
    int32_t accumulated_delta;    // Accumulated counts before scaling and sending
    uint32_t last_send_time;      // Timestamp of last UART send (for rate limiting)
    uint32_t last_movement_time;  // Timestamp of last encoder movement (for noise rejection)
    uint32_t button_last_press;   // Timestamp of last button press (for debouncing)

    // DMA buffer
    // ----------
    // CRITICAL: Each encoder MUST have its own buffer!
    // If encoders shared one buffer, they would overwrite each other's data
    // while DMA is still transmitting. This caused unreliable transfers.
    uint8_t tx_buffer[4];         // Dedicated UART transmit buffer for this encoder

    // Calibration
    // -----------
    uint8_t counts_per_unit;      // How many hardware counts = 1 command unit
                                  // (e.g., 4 for encoders with 4 counts per detent)
} Encoder_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Protocol constants
#define PROTO_START      0xAA

// Heading control (0x10-0x1F range)
#define CMD_HDG_RESET    0x10
#define CMD_HDG_SET      0x11

// Altitude control (0x20-0x2F range)
#define CMD_ALT_RESET    0x20
#define CMD_ALT_SET      0x21

// Vertical Speed control (0x30-0x3F range)
#define CMD_VS_RESET     0x30
#define CMD_VS_SET       0x31

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
 * ENCODER ARRAY - STRUCTURE-BASED APPROACH
 * ============================================================================
 * Instead of separate variables for each encoder, we use an array of structures.
 * This is a fundamental embedded systems pattern for handling multiple peripherals.
 *
 * ARRAY INITIALIZATION SYNTAX:
 * ----------------------------
 * Each line in the array initializes one Encoder_t structure. The values inside
 * the curly braces correspond to the struct fields in order:
 *
 * { htim, cmd_reset, cmd_set, button_pin, button_port, last_count, accumulated_delta,
 *   last_send_time, button_last_press, counts_per_unit }
 *
 * We initialize state variables to 0 (they'll be updated at runtime), and set
 * configuration values to the appropriate hardware/protocol constants.
 *
 * WHY THIS IS BETTER:
 * -------------------
 * - Before: 3 encoders × 4 state variables = 12 separate variables
 * - After:  1 array with 3 elements = clean, organized, scalable
 * - Bonus:  Same ProcessEncoder() function works for all 3!
 */
static Encoder_t encoders[3] = {
    // Encoder 0: Heading (HDG)
    // ------------------------
    // Timer: TIM2 (PA0/PA1), Button: PA4
    // Commands: 0x10 (reset), 0x11 (set)
    // All state initialized to 0, calibration set to 4 counts/detent
    {
        &htim2,              // Hardware timer handle
        BTN_KNOB1_Pin,       // Button GPIO pin (PA4)
        GPIOA,               // Button GPIO port
        CMD_HDG_RESET,       // Reset command (0x10)
        CMD_HDG_SET,         // Set command (0x11)
        0,                   // last_count (initialized to 0)
        0,                   // accumulated_delta (initialized to 0)
        0,                   // last_send_time (initialized to 0)
        0,                   // last_movement_time (initialized to 0)
        0,                   // button_last_press (initialized to 0)
        {0},                 // tx_buffer (initialized to zeros)
        4                    // counts_per_unit (4 for TI12 mode)
    },

    // Encoder 1: Vertical Speed (VS)
    // -------------------------------
    // Timer: TIM3 (PA6/PA7), Button: PA5
    // Commands: 0x30 (reset), 0x31 (set)
    {
        &htim3,              // Hardware timer handle
        BTN_KNOB_VS_Pin,     // Button GPIO pin (PA5)
        GPIOA,               // Button GPIO port
        CMD_VS_RESET,        // Reset command (0x30)
        CMD_VS_SET,          // Set command (0x31)
        0,                   // last_count
        0,                   // accumulated_delta
        0,                   // last_send_time
        0,                   // last_movement_time
        0,                   // button_last_press
        {0},                 // tx_buffer (initialized to zeros)
        4                    // counts_per_unit
    },

    // Encoder 2: Altitude (ALT)
    // --------------------------
    // Timer: TIM1 (PA8/PA9), Button: PB0
    // Commands: 0x20 (reset), 0x21 (set)
    {
        &htim1,              // Hardware timer handle
        BTN_KNOB_ALT_Pin,    // Button GPIO pin (PB0)
        GPIOB,               // Button GPIO port (NOTE: GPIOB, not GPIOA!)
        CMD_ALT_RESET,       // Reset command (0x20)
        CMD_ALT_SET,         // Set command (0x21)
        0,                   // last_count
        0,                   // accumulated_delta
        0,                   // last_send_time
        0,                   // last_movement_time
        0,                   // button_last_press
        {0},                 // tx_buffer (initialized to zeros)
        4                    // counts_per_unit
    }
};

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
 * @brief Process rotary encoder and send appropriate commands
 *
 * This function reads the specified encoder's hardware timer counter,
 * calculates the delta (change) since the last read, and sends the
 * appropriate command via UART if the encoder has moved. Works with
 * any encoder in the encoders[] array via parameterization.
 *
 * Includes rate limiting, accumulation, and scaling for all encoders.
 *
 * @param enc: Pointer to the encoder structure to process
 * @note Called from main loop for each encoder. Hardware timers count
 *       encoder pulses automatically in background.
 */
void ProcessEncoder(Encoder_t* enc);

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
   * START ALL ENCODER TIMERS
   * ============================================================================
   * We start all 3 hardware timers in encoder mode. Each timer automatically
   * counts its encoder's pulses in the background:
   *
   * - Clockwise rotation: Counter increments (goes up)
   * - Counter-clockwise:  Counter decrements (goes down)
   * - TIM_CHANNEL_ALL:    Enables both channels (A and B inputs)
   *
   * The counters run continuously with NO CPU involvement. Our ProcessEncoder()
   * function will simply read the counter values periodically to check for
   * changes. This is why hardware encoder mode is so efficient!
   *
   * ENCODER ASSIGNMENTS:
   * --------------------
   * TIM2 (32-bit): HDG encoder on PA0/PA1
   * TIM3 (16-bit): VS encoder on PA6/PA7
   * TIM1 (16-bit): ALT encoder on PA8/PA9
   *
   * Technical details:
   * - All configured in TI12 mode (4x resolution)
   * - Count on all edges of both channels: rising A, falling A, rising B, falling B
   * - TIM2 is 32-bit (0 to 4,294,967,295) - virtually impossible to overflow
   * - TIM1/TIM3 are 16-bit (0 to 65,535) - but still plenty for human input speeds
   * - Direction detection handled automatically by hardware quadrature decoder
   */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);  // HDG encoder
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);  // VS encoder
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);  // ALT encoder

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* ========================================================================
     * MAIN LOOP - MULTI-ENCODER PROCESSING
     * ========================================================================
     * This is a simple polling loop that continuously checks ALL encoders.
     *
     * HOW IT WORKS:
     * -------------
     * 1. Call ProcessEncoder() for each encoder in the array
     * 2. Each call reads its timer's hardware counter (automatic counting)
     * 3. If the counter changed, calculates delta and sends appropriate command
     * 4. HAL_Delay(10) paces the loop at ~100 Hz (100 iterations per second)
     *
     * STRUCTURE-BASED APPROACH:
     * -------------------------
     * Instead of ProcessEncoderHDG(), ProcessEncoderVS(), ProcessEncoderALT(),
     * we call the SAME function 3 times with different structure pointers:
     *
     *   ProcessEncoder(&encoders[0]);  // HDG: reads TIM2, sends CMD_HDG_SET
     *   ProcessEncoder(&encoders[1]);  // VS:  reads TIM3, sends CMD_VS_SET
     *   ProcessEncoder(&encoders[2]);  // ALT: reads TIM1, sends CMD_ALT_SET
     *
     * This is elegant! One function, three encoders, minimal code duplication.
     *
     * WHY 10ms DELAY IS FINE:
     * -----------------------
     * - All hardware timers count encoder pulses continuously in the background
     * - No pulses are ever missed, even during the delay
     * - 100 Hz polling is plenty fast for human input (humans can't spin
     *   encoders faster than ~10 rotations/second)
     * - Lower CPU usage than busy-waiting (CPU can sleep during delay)
     *
     * BUTTON HANDLING:
     * ----------------
     * All encoder buttons are handled by interrupt (HAL_GPIO_EXTI_Callback),
     * not in this loop. When you press any button, the interrupt fires
     * immediately and sends the appropriate RESET command. No polling needed!
     *
     * EXPANSION TO 4 ENCODERS:
     * ------------------------
     * To add a 4th encoder, just:
     * 1. Add it to the encoders[] array in the PV section
     * 2. Add ProcessEncoder(&encoders[3]); below
     * 3. Change loop limit in HAL_GPIO_EXTI_Callback from 3 to 4
     *
     * That's it! The structure-based approach scales beautifully.
     */

    // Process all encoder rotations and send appropriate commands
    // ------------------------------------------------------------
    // NOTICE: We pass a pointer to each encoder structure using &encoders[i]
    // The & operator gives us the address of the array element.
    ProcessEncoder(&encoders[0]);  // HDG encoder (TIM2)
    ProcessEncoder(&encoders[1]);  // VS encoder (TIM3)
    ProcessEncoder(&encoders[2]);  // ALT encoder (TIM1)

    // Pace the main loop at ~100 Hz (10ms delay)
    // -------------------------------------------
    // This is NOT blocking:
    // - Encoder counting (hardware timers do that continuously)
    // - Button presses (interrupts handle those immediately)
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
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Send generic command via UART using encoder's dedicated buffer
 *
 * This is a generic function that builds and transmits any 4-byte protocol packet.
 * It calculates the XOR checksum and sends via DMA.
 *
 * WHY EACH ENCODER NEEDS ITS OWN BUFFER:
 * ---------------------------------------
 * CRITICAL FIX: We used to have ONE shared static buffer for all encoders.
 * This caused a race condition:
 *
 * 1. HDG encoder calls SendCommand → fills buffer, starts DMA
 * 2. DMA takes ~35μs to transmit 4 bytes at 115200 baud
 * 3. VS encoder calls SendCommand → OVERWRITES buffer while DMA is reading!
 * 4. Result: Corrupted/mixed packets sent 💥
 *
 * Solution: Each encoder has its own tx_buffer[4] in the structure.
 * Now encoders can't interfere with each other's transmissions.
 *
 * PROTOCOL FORMAT:
 * ----------------
 * Byte 0: START (0xAA) - frame sync marker
 * Byte 1: COMMAND - what action to perform (e.g., 0x11 = HDG:SET)
 * Byte 2: OPERAND - command parameter (e.g., +5 for HDG:SET)
 * Byte 3: CHECKSUM - XOR of first 3 bytes for error detection
 *
 * USAGE EXAMPLES:
 * ---------------
 * SendCommand(&encoders[0], CMD_HDG_RESET, 0);     // Send HDG:RESET
 * SendCommand(&encoders[1], CMD_VS_SET, 5);        // Send VS:SET with +5
 * SendCommand(&encoders[2], CMD_ALT_SET, -10);     // Send ALT:SET with -10
 *
 * @param enc: Pointer to encoder structure (provides dedicated buffer)
 * @param cmd: Command byte (0x00-0xFF)
 * @param operand: Command parameter as signed 8-bit (-128 to +127)
 */
void SendCommand(Encoder_t* enc, uint8_t cmd, int8_t operand)
{
    // Use THIS encoder's dedicated buffer (not a shared static one!)
    // This prevents race conditions when multiple encoders send simultaneously
    uint8_t* txBuffer = enc->tx_buffer;

    // Build the 4-byte packet in the encoder's buffer
    txBuffer[0] = PROTO_START;              // Always 0xAA
    txBuffer[1] = cmd;                      // Command byte (varies)
    txBuffer[2] = (uint8_t)operand;         // Operand (cast signed→unsigned)
    txBuffer[3] = PROTO_START ^ cmd ^ (uint8_t)operand;  // XOR checksum

    // Transmit via DMA (non-blocking)
    // DMA controller will send these 4 bytes in background while CPU continues
    // Each encoder's buffer is independent, so no interference!
    HAL_UART_Transmit_DMA(&huart2, txBuffer, 4);
}

/* ============================================================================
 * ENCODER PROCESSING FUNCTION
 * ============================================================================
 */

/**
 * @brief Process rotary encoder and send appropriate commands
 *
 * THIS IS THE KEY REFACTORING: One function handles ALL encoders!
 * =================================================================
 * Instead of ProcessEncoderHDG(), ProcessEncoderVS(), ProcessEncoderALT(),
 * we have ONE function that accepts a pointer to an Encoder_t structure.
 * The structure tells us which timer to read, which commands to send, etc.
 *
 * This is called "parameterization" - making a function work on different
 * data by passing parameters instead of hardcoding values.
 *
 * USAGE:
 * ------
 * ProcessEncoder(&encoders[0]);  // Process HDG encoder
 * ProcessEncoder(&encoders[1]);  // Process VS encoder
 * ProcessEncoder(&encoders[2]);  // Process ALT encoder
 *
 * HOW IT WORKS:
 * -------------
 * Hardware timers (TIM1/TIM2/TIM3) are configured in encoder mode with TI12
 * (4x resolution). The hardware automatically monitors the A/B channels:
 *
 * - When you rotate clockwise:  Counter increments (e.g., 100 -> 104)
 * - When you rotate counter-CW: Counter decrements (e.g., 100 -> 96)
 * - The hardware handles all the quadrature decoding logic
 *
 * We simply read the counter, compare it to the last value, and the difference
 * is our delta. This delta becomes the operand for the command.
 *
 * STRUCTURE FIELD ACCESS:
 * -----------------------
 * enc->htim              : Which timer to read (htim1, htim2, or htim3)
 * enc->last_count        : Previous counter value for this encoder
 * enc->accumulated_delta : Accumulated counts for this encoder
 * enc->cmd_set           : Which command to send (HDG:SET, VS:SET, or ALT:SET)
 *
 * The -> operator accesses fields through a pointer.
 *
 * @param enc: Pointer to the encoder structure to process
 */
void ProcessEncoder(Encoder_t* enc)
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
    // __HAL_TIM_GET_COUNTER reads the timer's CNT register directly.
    // NOTICE: We use enc->htim, not a hardcoded timer!
    //         This is why the same function works for HDG, VS, and ALT.
    //
    // This value changes automatically in hardware as the encoder rotates:
    // - Clockwise rotation: Counter increments
    // - Counter-clockwise:  Counter decrements
    // - No CPU intervention needed!
    //
    // Cast to int32_t for signed arithmetic (encoder can go both directions)
    int32_t current_count = (int32_t)__HAL_TIM_GET_COUNTER(enc->htim);

    // =========================================================================
    // STEP 2: Calculate raw delta (change since last read)
    // =========================================================================
    // This is how many counts the hardware encoder has moved since we last
    // checked. Could be positive (CW), negative (CCW), or zero (no movement).
    //
    // NOTICE: We use enc->last_count, which is unique to this encoder!
    //         HDG encoder has its own last_count, VS has its own, etc.
    int32_t delta = current_count - enc->last_count;

    // =========================================================================
    // STEP 3: Accumulate delta and update reference position
    // =========================================================================
    // KEY CONCEPT: We ALWAYS update enc->last_count when delta != 0,
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
        enc->accumulated_delta += delta;       // Add to this encoder's accumulator
        enc->last_count = current_count;       // Update this encoder's reference
        enc->last_movement_time = HAL_GetTick();  // Track movement for noise rejection
    }

    // =========================================================================
    // STEP 4: Send accumulated delta when rate limit allows
    // =========================================================================
    // Only transmit when:
    // 1. We have accumulated movement (accumulated_delta != 0)
    // 2. Enough time has passed since last transmission (rate limiting)
    //
    // NOTICE: Each encoder has its own rate limiting timestamp!
    if (enc->accumulated_delta != 0)
    {
        uint32_t current_time = HAL_GetTick();
        uint32_t time_since_last_send = current_time - enc->last_send_time;

        if (time_since_last_send >= MIN_SEND_INTERVAL_MS)
        {
            // ================================================================
            // STEP 5: Scale accumulated delta by encoder divisor
            // ================================================================
            // Convert raw encoder counts to logical units.
            // This compensates for encoder PPR vs detent count mismatch.
            //
            // NOTICE: We use enc->counts_per_unit, which is configurable
            //         per encoder! Each encoder can have different scaling.
            //
            // EXAMPLE (encoder with counts_per_unit = 4):
            //   Physical: User clicks encoder 1 detent
            //   Hardware: Counter increments by 4 (four edges in TI12 mode)
            //   Accumulated: +4
            //   Scaled: +4 / 4 = +1
            //   Sent: XXX:SET(+1) ← One click = one unit!
            //
            // To adjust: Modify counts_per_unit in encoders[] array initialization
            int32_t scaled_delta = enc->accumulated_delta / enc->counts_per_unit;

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
                // STEP 8: Send the appropriate command via UART
                // ============================================================
                // NOTICE: We use enc->cmd_set, which varies by encoder!
                //   - For HDG: enc->cmd_set = CMD_HDG_SET (0x11)
                //   - For VS:  enc->cmd_set = CMD_VS_SET (0x31)
                //   - For ALT: enc->cmd_set = CMD_ALT_SET (0x21)
                //
                // This is the magic that makes one function work for all encoders!
                // Pass enc so SendCommand uses this encoder's dedicated tx_buffer
                SendCommand(enc, enc->cmd_set, (int8_t)scaled_delta);

                // ============================================================
                // STEP 9: Update timestamps and accumulator
                // ============================================================
                // Record send time for rate limiting
                enc->last_send_time = current_time;

                // Subtract the UNSCALED amount we just sent from accumulator.
                // Why multiply back? Because we divided for sending, but the
                // accumulator tracks raw counts.
                //
                // Example:
                //   accumulated_delta = 5 raw counts
                //   counts_per_unit = 2
                //   scaled_delta = 5 / 2 = 2 (integer division)
                //   We sent: +2
                //   We used: 2 * 2 = 4 raw counts
                //   Remaining: 5 - 4 = 1 raw count (saved for next time)
                enc->accumulated_delta -= (scaled_delta * enc->counts_per_unit);
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
 * THIS IS ANOTHER KEY REFACTORING: One handler for ALL encoder buttons!
 * ======================================================================
 * This function is called automatically by the HAL when ANY GPIO pin configured
 * as an external interrupt (EXTI) triggers. Instead of hardcoding button checks,
 * we loop through the encoders array to find which encoder's button was pressed.
 *
 * STRUCTURE-BASED APPROACH:
 * -------------------------
 * We iterate through encoders[0], encoders[1], encoders[2] and check:
 * - Is GPIO_Pin == this encoder's button_pin?
 * - If yes, send this encoder's cmd_reset command
 *
 * This scales perfectly! If we add a 4th encoder to the array, this handler
 * automatically supports it with ZERO code changes.
 *
 * INTERRUPT vs POLLING:
 * ---------------------
 * Unlike the encoder rotation (which we poll in the main loop), buttons use
 * interrupts. This means:
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
 * Without debouncing, one button press would trigger this interrupt 5-10 times.
 * We prevent this using timestamps stored PER ENCODER in the structure.
 *
 * NOISE REJECTION (CRITICAL FOR VS ENCODER):
 * ------------------------------------------
 * When an encoder rotates, it generates electrical noise that can couple into
 * nearby wires (especially the button wire on the same encoder). This causes
 * false button interrupts during rotation.
 *
 * Solution: We reject button presses if the encoder was actively rotating
 * within the last 150ms. This is detected by checking enc->last_send_time.
 * If a command was sent recently, the encoder was moving, so the button
 * interrupt is likely noise and is ignored.
 *
 * INTERRUPT CONTEXT CONSIDERATIONS:
 * ----------------------------------
 * This function runs in interrupt context, not main loop context. Rules:
 * - Keep it FAST (microseconds, not milliseconds)
 * - Don't use HAL_Delay() - it blocks ALL interrupts
 * - Don't do heavy processing
 * - OK to call SendCommand() because it just triggers DMA (non-blocking)
 *
 * @param GPIO_Pin: The pin number that triggered the interrupt
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // Debounce and noise rejection configuration
    // -------------------------------------------
    const uint32_t DEBOUNCE_TIME_MS = 100;  // Increased from 50ms to reject more noise
    const uint32_t ENCODER_QUIET_TIME_MS = 500;  // Increased to 500ms - rejects button when encoder is rotating/settling

    // Get current time once (more efficient than calling HAL_GetTick() repeatedly)
    uint32_t current_time = HAL_GetTick();

    // Loop through all encoders to find which button was pressed
    // -----------------------------------------------------------
    // NOTICE: We use a for loop to iterate through the encoders array.
    // The loop variable 'i' goes from 0 to 2 (3 encoders total).
    //
    // This is the key to scalability: If we add a 4th encoder to the array,
    // we just change 'i < 3' to 'i < 4' (or better, use a #define for count).
    for (int i = 0; i < 3; i++)
    {
        // Get pointer to current encoder for cleaner syntax
        Encoder_t* enc = &encoders[i];

        // Check if THIS encoder's button triggered the interrupt
        // -------------------------------------------------------
        // NOTICE: We check enc->button_pin, which is different for each encoder:
        //   - encoders[0]: BTN_KNOB1_Pin (PA4) for HDG
        //   - encoders[1]: BTN_KNOB_VS_Pin (PA5) for VS
        //   - encoders[2]: BTN_KNOB_ALT_Pin (PB0) for ALT
        if (GPIO_Pin == enc->button_pin)
        {
            // Calculate time since this specific encoder's last button press
            // ---------------------------------------------------------------
            // NOTICE: Each encoder has its own button_last_press timestamp!
            // Pressing HDG button doesn't affect VS button debouncing, etc.
            uint32_t time_since_last_press = current_time - enc->button_last_press;

            // Noise rejection: Ignore button if encoder was recently rotating
            // ----------------------------------------------------------------
            // CRITICAL FIX for VS encoder noise issue:
            // When the encoder rotates, it generates electrical noise that can
            // couple into the button pin and trigger false interrupts.
            //
            // We reject the button press if the encoder MOVED recently
            // (within ENCODER_QUIET_TIME_MS). We check last_movement_time, which
            // updates IMMEDIATELY when the encoder counter changes, not when we
            // send (which is rate-limited). This catches noise even before the
            // first UART packet is sent.
            uint32_t time_since_encoder_activity = current_time - enc->last_movement_time;

            if (time_since_encoder_activity < ENCODER_QUIET_TIME_MS)
            {
                // Encoder was recently rotating - this is likely noise, ignore it
                break;
            }

            // Debounce check
            // --------------
            if (time_since_last_press >= DEBOUNCE_TIME_MS)
            {
                // Valid button press - send appropriate RESET command
                // ----------------------------------------------------
                // NOTICE: We use enc->cmd_reset, which varies by encoder:
                //   - For HDG: CMD_HDG_RESET (0x10)
                //   - For VS:  CMD_VS_RESET (0x30)
                //   - For ALT: CMD_ALT_RESET (0x20)
                //
                // Operand is always 0 for reset commands.
                // Pass enc so SendCommand uses this encoder's dedicated tx_buffer
                SendCommand(enc, enc->cmd_reset, 0);

                // Update THIS encoder's debounce timestamp
                // -----------------------------------------
                enc->button_last_press = current_time;
            }

            // Found matching button, no need to check other encoders
            break;
        }
    }

    // If GPIO_Pin didn't match any encoder button, the interrupt came from
    // a different source (e.g., PC13 - the blue user button on Nucleo board).
    // We simply ignore it. You could add handling for other pins here if needed.
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
