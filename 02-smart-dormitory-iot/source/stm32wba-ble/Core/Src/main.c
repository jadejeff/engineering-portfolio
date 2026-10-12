/* USER CODE BEGIN Header */
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "NanoEdgeAI.h"
#include "p2p_server_app.h"
#include "app_common.h"
#include "stm32_seq.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AI_LEARN_TIMEOUT_MS     30000   /* 学习超时：30秒内未完成学习则强制进入检测 */
#define AI_ALERT_DEBOUNCE_CNT   3       /* 连续 N 帧异常才触发告警（消抖） */
#define AI_ALERT_COOLDOWN_MS    5000    /* 告警冷却：两次告警最小间隔 5 秒 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc4;

CRC_HandleTypeDef hcrc;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef handle_GPDMA1_Channel1;
DMA_HandleTypeDef handle_GPDMA1_Channel0;

RAMCFG_HandleTypeDef hramcfg_SRAM1;

RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN PV */
extern UART_HandleTypeDef huart1;  /* 用于主循环调试打印，不放在 ISR 中 */

/* USART3 变量前置声明（定义在 USER CODE 4，此处为 main() 中 while 循环提供可见性） */
#define USART3_RX_BUF_SIZE 128
extern volatile uint8_t  usart3_rx_data;
extern volatile uint8_t  usart3_rx_buf[USART3_RX_BUF_SIZE];
extern volatile uint16_t usart3_rx_len;
extern volatile uint8_t  usart3_frame_ready;
extern volatile uint8_t  usart3_print_buf[USART3_RX_BUF_SIZE]; /* ISR 快照，main 循环打印 */
extern volatile uint16_t usart3_print_len;                     /* 快照长度 */

/* USART3 TX 中断环形缓冲 */
#define U3_TX_BUF_SIZE 256
extern volatile uint8_t  u3_tx_buf[U3_TX_BUF_SIZE];
extern volatile uint16_t u3_tx_head;
extern volatile uint16_t u3_tx_tail;
extern volatile uint8_t  u3_tx_busy;

/* NanoEdge AI 外部变量（定义于 p2p_server_app.c） */
extern uint8_t  g_ai_learning_done;
extern uint16_t g_ai_learning_count;
extern float    g_ai_input_buf[3];
extern uint8_t  g_ai_alert_buf[4];
extern uint8_t  g_ai_similarity;

/* JSON 解析函数（定义于 app_ble.c） */
int parse_json_for_ai(const uint8_t *json_str, uint16_t len,
                       float *out_t, float *out_c, float *out_p);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void SystemPower_Config(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void USART3_Init_For_ESP8266(void);
void USART3_SendString_IT(uint8_t *str, uint16_t len);
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
  /* Config code for STM32_WPAN (HSE Tuning must be done before system clock configuration) */
  MX_APPE_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* Configure the System Power */
  SystemPower_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_RAMCFG_Init();
  MX_RTC_Init();
  MX_ICACHE_Init();
  MX_LPUART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
USART3_Init_For_ESP8266();
  HAL_UART_Transmit(&huart3, (uint8_t *)"WBA_BOOT\n", 9, 100);

  /* NanoEdge AI 初始化（使用预训练模型） */
  {
    enum neai_state init_st = neai_anomalydetection_init(true);
    char init_msg[64];
    int nn = snprintf(init_msg, sizeof(init_msg),
                      "\r\n[AI INIT] ret=%d learning=%d count=%d/%d\r\n",
                      init_st, g_ai_learning_done, g_ai_learning_count, AI_LEARNING_MIN_SAMPLES);
    HAL_UART_Transmit(&huart1, (uint8_t *)init_msg, nn, 100);
  }
  /* USER CODE END 2 */

  /* Init code for STM32_WPAN */
  MX_APPE_Init(NULL);
  HAL_UART_Transmit(&huart3, (uint8_t *)"BLE_OK\n", 7, 100);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */
    MX_APPE_Process();

    /* USER CODE BEGIN 3 */
    /* 主循环中打印 USART3 帧快照 + NanoEdge AI 推理 */
    if(usart3_frame_ready)
    {
      /* 调试打印到 USART1 */
      HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n-- USART3 RX: ", 17, 50);
      if(usart3_print_len > 0)
      {
        HAL_UART_Transmit(&huart1, (uint8_t *)usart3_print_buf, usart3_print_len, 50);
      }
      HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, 50);

      /* ---- NanoEdge AI 推理 ---- */
      float t, c, p;
      if (parse_json_for_ai((const uint8_t *)usart3_print_buf, usart3_print_len, &t, &c, &p) == 3)
      {
        g_ai_input_buf[0] = t;
        g_ai_input_buf[1] = c;
        g_ai_input_buf[2] = p;

        if (!g_ai_learning_done)
        {
          /* 学习阶段：喂入正常数据建立基线 */
          static uint32_t learn_first_tick = 0;

          neai_anomalydetection_learn(g_ai_input_buf);

          /* 记录第一帧学习数据的时间戳 */
          if (learn_first_tick == 0)
          {
            learn_first_tick = HAL_GetTick();
          }

          if (++g_ai_learning_count >= AI_LEARNING_MIN_SAMPLES)
          {
            g_ai_learning_done = 1;
            HAL_UART_Transmit(&huart1,
                (uint8_t *)"\r\n========== [AI] LEARNING DONE -> DETECTION MODE ==========\r\n",
                61, 100);
          }
          else if ((HAL_GetTick() - learn_first_tick) > AI_LEARN_TIMEOUT_MS)
          {
            /* 学习超时！ESP8266 可能断连，强制进入检测模式 */
            g_ai_learning_done = 1;
            char timeout_msg[80];
            int tn = snprintf(timeout_msg, sizeof(timeout_msg),
                              "\r\n!!! [AI] LEARN TIMEOUT (%dms, only %d/%d frames) -> FORCE DETECTION !!!\r\n",
                              AI_LEARN_TIMEOUT_MS, g_ai_learning_count, AI_LEARNING_MIN_SAMPLES);
            HAL_UART_Transmit(&huart1, (uint8_t *)timeout_msg, tn, 100);
          }
          else
          {
            /* 学习阶段：每帧都打印，方便观察喂入的数据是否真的 "正常" */
            char dbg[64];
            int n = snprintf(dbg, sizeof(dbg),
                             "\r\n[AI LEARN] %d/%d (t=%.1f c=%.3f p=%.1f)\r\n",
                             g_ai_learning_count, AI_LEARNING_MIN_SAMPLES, t, c, p);
            HAL_UART_Transmit(&huart1, (uint8_t *)dbg, n, 100);
          }
        }
        else
        {
          /* 检测阶段：计算相似度 */
          static uint8_t  alert_level = 0;       /* 当前异常状态: 0=正常, 1=异常 */
          static uint8_t  alert_cnt = 0;         /* 连续异常帧计数 */
          static uint32_t last_alert_tick = 0;   /* 上次告警发送时间 */

          enum neai_state st = neai_anomalydetection_detect(g_ai_input_buf, &g_ai_similarity);

          /* 每 10 帧汇报一次相似度 + 跳过统计 */
          {
            static uint16_t detect_cnt = 0;
            if ((++detect_cnt % 10) == 0)
            {
              char dbg[64];
              int n = snprintf(dbg, sizeof(dbg),
                               "\r\n[AI DETECT] #%d sim=%d%% (t=%.1f c=%.3f p=%.1f)\r\n",
                               detect_cnt, g_ai_similarity, t, c, p);
              HAL_UART_Transmit(&huart1, (uint8_t *)dbg, n, 100);
            }
            /* 相似度异常但未到阈值时，每帧打印供校准参考 */
            if (g_ai_similarity < 90)
            {
              char dbg2[64];
              int n2 = snprintf(dbg2, sizeof(dbg2),
                                "\r\n[AI LOW-SIM] #%d sim=%d%% (t=%.1f c=%.3f p=%.1f)\r\n",
                                detect_cnt, g_ai_similarity, t, c, p);
              HAL_UART_Transmit(&huart1, (uint8_t *)dbg2, n2, 100);
            }
          }

          /* ── 消抖 + 冷却逻辑 ── */
          uint8_t this_abnormal = (st == NEAI_OK && g_ai_similarity < 50) ? 1 : 0;

          if (this_abnormal != alert_level)
          {
            /* 状态变化：重置消抖计数 */
            alert_level = this_abnormal;
            alert_cnt = this_abnormal ? 1 : 0;
          }
          else if (this_abnormal)
          {
            alert_cnt++;
          }

          if (this_abnormal
              && alert_cnt >= AI_ALERT_DEBOUNCE_CNT
              && (HAL_GetTick() - last_alert_tick) > AI_ALERT_COOLDOWN_MS)
          {
            last_alert_tick = HAL_GetTick();
            alert_cnt = 0;  /* 重置消抖，避免同一波异常连续告警 */

            /* 异常！发送 "111" 告警 */
            memcpy(g_ai_alert_buf, "111", 3);

            /* AI 验证打印：告警 */
            {
              char alert[64];
              int n = snprintf(alert, sizeof(alert),
                               "\r\n!!! [AI ALERT] sim=%d%% -> SEND '111' to ESP8266 & BLE !!!\r\n",
                               g_ai_similarity);
              HAL_UART_Transmit(&huart1, (uint8_t *)alert, n, 100);
            }

            /* USART3 回传 ESP8266 */
            USART3_SendString_IT(g_ai_alert_buf, 3);
            /* BLE 通知微信小程序 */
            UTIL_SEQ_SetTask(1U << CFG_TASK_AI_ALERT_ID, CFG_SEQ_PRIO_0);
          }
        }
      }
      else
      {
        /* JSON 解析失败计数器（每 50 次损坏汇报一次） */
        static uint16_t parse_skip_cnt = 0;
        if ((++parse_skip_cnt % 50) == 0)
        {
          char skip_msg[40];
          int sn = snprintf(skip_msg, sizeof(skip_msg),
                            "\r\n[AI SKIP] %d bad JSON frames\r\n", parse_skip_cnt);
          HAL_UART_Transmit(&huart1, (uint8_t *)skip_msg, sn, 100);
        }
      }

      usart3_frame_ready = 0;
    }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMLOW);

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEDiv = RCC_HSE_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL1.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK7|RCC_CLOCKTYPE_HCLK5;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB7CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHB5_PLL1_CLKDivider = RCC_SYSCLK_PLL1_DIV1;
  RCC_ClkInitStruct.AHB5_HSEHSI_CLKDivider = RCC_SYSCLK_HSEHSI_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

   /* Select SysTick source clock */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_LSE);

   /* Re-Initialize Tick with new clock source */
  if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RADIOST;
  PeriphClkInit.RadioSlpTimClockSelection = RCC_RADIOSTCLKSOURCE_LSE;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{
  /* WKUP_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(WKUP_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(WKUP_IRQn);
/* USER CODE BEGIN PWR */

/* USER CODE END PWR */
}

/**
  * @brief ADC4 Initialization Function
  * @param None
  * @retval None
  */
void MX_ADC4_Init(void)
{

  /* USER CODE BEGIN ADC4_Init 0 */

  /* USER CODE END ADC4_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC4_Init 1 */

  /* USER CODE END ADC4_Init 1 */

  /** Common config
  */
  hadc4.Instance = ADC4;
  hadc4.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc4.Init.Resolution = ADC_RESOLUTION_12B;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc4.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc4.Init.LowPowerAutoPowerOff = DISABLE;
  hadc4.Init.LowPowerAutonomousDPD = ADC_LP_AUTONOMOUS_DPD_DISABLE;
  hadc4.Init.LowPowerAutoWait = DISABLE;
  hadc4.Init.ContinuousConvMode = DISABLE;
  hadc4.Init.NbrOfConversion = 1;
  hadc4.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc4.Init.DMAContinuousRequests = DISABLE;
  hadc4.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_LOW;
  hadc4.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc4.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_814CYCLES_5;
  hadc4.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc4.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC4_Init 2 */

  /* USER CODE END ADC4_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.GeneratingPolynomial = 7607;
  hcrc.Init.CRCLength = CRC_POLYLENGTH_16B;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_WORDS;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
void MX_GPDMA1_Init(void)
{

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* GPDMA1 interrupt Init */
    HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */
  /* USER CODE END ICACHE_Init 1 */

  /** Full retention for ICACHE in stop mode
  */
  LL_PWR_SetICacheRAMStopRetention(LL_PWR_ICACHERAM_STOP_FULL_RETENTION);

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  LL_ICACHE_SetMode(LL_ICACHE_1WAY);
  LL_ICACHE_Enable();
  /* USER CODE BEGIN ICACHE_Init 2 */
  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 209700;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* FIFO 已关闭 —— HAL_UART_Transmit_IT 状态机兼容 */
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief RAMCFG Initialization Function
  * @param None
  * @retval None
  */
void MX_RAMCFG_Init(void)
{

  /* USER CODE BEGIN RAMCFG_Init 0 */

  /* USER CODE END RAMCFG_Init 0 */

  /* USER CODE BEGIN RAMCFG_Init 1 */

  /* USER CODE END RAMCFG_Init 1 */

  /** Initialize RAMCFG SRAM1
  */
  hramcfg_SRAM1.Instance = RAMCFG_SRAM1;
  if (HAL_RAMCFG_Init(&hramcfg_SRAM1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_RAMCFG_ConfigWaitState(&hramcfg_SRAM1, RAMCFG_WAITSTATE_0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RAMCFG_Init 2 */

  /* USER CODE END RAMCFG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_PrivilegeStateTypeDef privilegeState = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = 31;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  hrtc.Init.BinMode = RTC_BINARY_ONLY;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  privilegeState.rtcPrivilegeFull = RTC_PRIVILEGE_FULL_NO;
  privilegeState.backupRegisterPrivZone = RTC_PRIVILEGE_BKUP_ZONE_NONE;
  privilegeState.backupRegisterStartZone2 = RTC_BKP_DR0;
  privilegeState.backupRegisterStartZone3 = RTC_BKP_DR0;
  if (HAL_RTCEx_PrivilegeModeSet(&hrtc, &privilegeState) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  if (HAL_RTCEx_SetSSRU_IT(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the Alarm A
  */
  sAlarm.BinaryAutoClr = RTC_ALARMSUBSECONDBIN_AUTOCLR_NO;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDBINMASK_NONE;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// ==========================
// USART3 变量定义 (ESP8266)
// ==========================
volatile uint8_t  usart3_rx_data = 0;        // 单字节接收缓冲(ISR写入)
volatile uint8_t  usart3_rx_buf[USART3_RX_BUF_SIZE] = {0}; // 帧工作缓冲区(ISR写入,接收完清空)
volatile uint16_t usart3_rx_len = 0;         // 帧工作长度(ISR写入,接收完清空)
volatile uint8_t  usart3_frame_ready = 0;    // 帧就绪标志(ISR置位,main循环清零)
volatile uint8_t  usart3_print_buf[USART3_RX_BUF_SIZE] = {0}; // 帧快照(ISR在清空前复制)
volatile uint16_t usart3_print_len = 0;      // 快照长度(main循环打印用)

// USART3 TX 中断发送环形缓冲
volatile uint8_t  u3_tx_buf[U3_TX_BUF_SIZE] = {0};
volatile uint16_t u3_tx_head = 0;
volatile uint16_t u3_tx_tail = 0;
volatile uint8_t  u3_tx_busy   = 0;

void USART3_Init_For_ESP8266(void)
{
  HAL_UART_Receive_IT(&huart3, (uint8_t *)&usart3_rx_data, 1);
}

/* 非阻塞中断发送：数据入环形缓冲，直接寄存器写启动（绕过 HAL，避免中断链断裂） */
void USART3_SendString_IT(uint8_t *str, uint16_t len)
{
  for (uint16_t i = 0; i < len; i++)
  {
    uint16_t next = (u3_tx_head + 1) % U3_TX_BUF_SIZE;
    while (next == u3_tx_tail); /* 缓冲满则等待（正常使用不会满） */
    u3_tx_buf[u3_tx_head] = str[i];
    u3_tx_head = next;
  }
  if (!u3_tx_busy)
  {
    u3_tx_busy = 1;
    uint8_t byte = u3_tx_buf[u3_tx_tail];
    u3_tx_tail = (u3_tx_tail + 1) % U3_TX_BUF_SIZE;
    USART3->TDR = byte;                              /* 直接写寄存器，不经过 HAL 状态机 */
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_TXE);      /* 仅使能 TXE 中断 */
  }
}

/* HAL_TxCpltCallback 不再处理 huart3（TX 由 USART3_IRQHandler 手动接管） */

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
  Error_Handler();
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
