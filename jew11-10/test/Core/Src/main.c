/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "application.h"
#include "app_config.h"
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim4_up;

/* USER CODE BEGIN PV */
drv_t drv;
motor_t motor;
motor_drv_t motor_drv;
Encoder_HandleTypeDef encoder;
application_t app;
volatile uint8_t raw_limit_a, raw_limit_b;
volatile uint32_t control_max_cycles;
static volatile uint8_t application_ready;
static uint32_t previous_control_cycles;
static uint8_t control_tick_started;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  /* The generated DMA/TIM4 peripherals remain configured but are not started.
   * The encoder is sampled once, synchronously, by the TIM1 application tick. */
  HAL_NVIC_DisableIRQ(DMA1_Stream6_IRQn);
  /* Keep timer registers tied to the estimator/controller configuration.
   * Reject a changed clock tree instead of silently generating wrong STEP rates. */
  _Static_assert(CPR > 0u && RES > 0u && CPR <= 65536u / RES,
                 "Encoder count range must fit TIM3");
  _Static_assert(APP_CONTROL_HZ > 0u, "Control frequency must be positive");
  RCC_ClkInitTypeDef clocks;
  uint32_t flash_latency;
  HAL_RCC_GetClockConfig(&clocks, &flash_latency);
  const uint32_t tim1_clock = HAL_RCC_GetPCLK2Freq() *
      (clocks.APB2CLKDivider == RCC_HCLK_DIV1 ? 1u : 2u);
  const uint32_t tim2_clock = HAL_RCC_GetPCLK1Freq() *
      (clocks.APB1CLKDivider == RCC_HCLK_DIV1 ? 1u : 2u);
  const uint32_t control_period = tim1_clock / (htim1.Init.Prescaler + 1u) / APP_CONTROL_HZ;
  if (tim1_clock != APP_TIMER_HZ || tim2_clock != APP_TIMER_HZ ||
      htim2.Init.Prescaler != 0u || control_period == 0u || control_period > 65536u ||
      (uint64_t)control_period * (htim1.Init.Prescaler + 1u) * APP_CONTROL_HZ != tim1_clock)
    Error_Handler();
  htim1.Init.Period = control_period - 1u;
  __HAL_TIM_SET_AUTORELOAD(&htim1, htim1.Init.Period);
  htim1.Instance->EGR = TIM_EGR_UG;
  __HAL_TIM_SET_COUNTER(&htim1, 0u);
  htim3.Init.Period = CPR * RES - 1u;
  __HAL_TIM_SET_AUTORELOAD(&htim3, htim3.Init.Period);
  __HAL_TIM_SET_COUNTER(&htim3, 0u);
  init_Encoder(&encoder, CPR, RES, 1.0f / APP_CONTROL_HZ);
  encoder_set_direction(&encoder, APP_ENCODER_DIRECTION);
  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) Error_Handler();

  drv_init(&drv, APP_MICROSTEPS, M0_GPIO_Port, M0_Pin, M1_GPIO_Port, M1_Pin,
           M2_GPIO_Port, M2_Pin, DIR_GPIO_Port, DIR_Pin, RST_SLP_GPIO_Port, RST_SLP_Pin);
  motor_init(&motor, APP_STEPS_PER_REV, &htim2, TIM_CHANNEL_1,
             APP_MAX_STEP_RATE_HZ, APP_TIMER_HZ, APP_CONTROL_HZ);
  motor_drv_init(&motor_drv, &drv, &motor);
  if (!motor_set_limits(&motor_drv, APP_MAX_STEP_RATE_HZ, APP_MAX_STEP_ACCEL)) Error_Handler();

  motor_homing_config_t homing = motor_homing_default_config();
  homing.a_pressed_level = APP_LIMIT_A_ACTIVE;
  homing.b_pressed_level = APP_LIMIT_B_ACTIVE;
  homing.cw_limit = APP_LIMIT_CW_MASK;
  homing.speed = APP_HOMING_SPEED_HZ;
  homing.latch_speed = APP_HOMING_SPEED_HZ * 0.5f;
  homing.acceleration = APP_HOMING_ACCEL;
  homing.debounce_ms = APP_HOMING_DEBOUNCE_MS;
  homing.backoff_steps = APP_HOMING_BACKOFF_STEPS;
  homing.max_release_steps = APP_HOMING_RELEASE_STEPS;
  homing.min_travel_steps = APP_HOMING_MIN_STEPS;
  homing.max_travel_steps = APP_HOMING_MAX_STEPS;
  homing.release_timeout_ms = APP_HOMING_RELEASE_TIMEOUT_MS;
  homing.phase_timeout_ms = APP_HOMING_TIMEOUT_MS;
  homing.total_timeout_ms = 5u * APP_HOMING_TIMEOUT_MS;
  if (!motor_configure_homing(&motor_drv, &homing)) Error_Handler();

  application_config_t config = {0};
  config.hardware_confirmed = APP_HARDWARE_CONFIRMED;
  config.run_mode = APP_RUN_MODE;
  config.metres_per_step = APP_METRES_PER_STEP;
  config.control = pendulum_default_config();
  config.control.dt_s = 1.0f / APP_CONTROL_HZ;
  config.control.effective_length_m = APP_EFFECTIVE_LENGTH_M;
  config.control.max_speed_m_s = APP_MAX_STEP_RATE_HZ * APP_METRES_PER_STEP;
  config.control.max_accel_m_s2 = APP_MAX_STEP_ACCEL * APP_METRES_PER_STEP;
  config.control.travel_margin_m = APP_TRAVEL_MARGIN_M;
  config.control.swing_timeout_s = APP_SWING_TIMEOUT_S;
  application_init(&app, &motor_drv, &encoder, &config);

  /* SysTick must continue during foreground homing. STEP completion has
   * higher priority than the control tick to preserve pulse accounting. */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  __HAL_DBGMCU_FREEZE_TIM1();
  __HAL_DBGMCU_FREEZE_TIM2();
  application_ready = 1u;
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK) Error_Handler();
/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    application_service(&app);
    HAL_Delay(1);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2399;
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
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 839;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIR_Pin|RST_SLP_Pin|M2_Pin|M1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(M0_GPIO_Port, M0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_Pin RST_SLP_Pin M2_Pin M1_Pin */
  GPIO_InitStruct.Pin = DIR_Pin|RST_SLP_Pin|M2_Pin|M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : M0_Pin */
  GPIO_InitStruct.Pin = M0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(M0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LIMIT_B_Pin LIMIT_A_Pin */
  GPIO_InitStruct.Pin = LIMIT_B_Pin|LIMIT_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* Explicit per-switch bias; defaults retain external-bias assumption. */
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pin = LIMIT_A_Pin;
  GPIO_InitStruct.Pull = APP_LIMIT_A_PULL;
  HAL_GPIO_Init(LIMIT_A_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = LIMIT_B_Pin;
  GPIO_InitStruct.Pull = APP_LIMIT_B_PULL;
  HAL_GPIO_Init(LIMIT_B_GPIO_Port, &GPIO_InitStruct);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM1 || !application_ready) return;
    const uint32_t start_cycles = DWT->CYCCNT;
    const uint32_t period_cycles = SystemCoreClock / (uint32_t)APP_CONTROL_HZ;
    application_check_timing(&app, control_tick_started ?
                             start_cycles - previous_control_cycles : 0u,
                             0u, period_cycles);
    previous_control_cycles = start_cycles;
    control_tick_started = 1u;
    raw_limit_a = (uint8_t)HAL_GPIO_ReadPin(LIMIT_A_GPIO_Port, LIMIT_A_Pin);
    raw_limit_b = (uint8_t)HAL_GPIO_ReadPin(LIMIT_B_GPIO_Port, LIMIT_B_Pin);
    application_tick(&app, (uint16_t)__HAL_TIM_GET_COUNTER(&htim3),
                     HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == APP_START_BUTTON_ACTIVE);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,
                     app.mode == APP_BALANCE ? GPIO_PIN_SET : GPIO_PIN_RESET);
    const uint32_t elapsed = DWT->CYCCNT - start_cycles;
    if (elapsed > control_max_cycles) control_max_cycles = elapsed;
    application_check_timing(&app, 0u, elapsed, period_cycles);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (application_ready && htim->Instance == TIM2 &&
        htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        motor_pulse_finished_isr(&motor_drv);
}

void App_EmergencyStop(void)
{
    /* Also callable from hard-fault paths where HAL/state may be unreliable. */
    TIM2->CCER &= ~TIM_CCER_CC1E;
    TIM2->CR1 &= ~TIM_CR1_CEN;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  App_EmergencyStop();
  __disable_irq();
  while (1) { }
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
