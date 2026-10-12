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
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "memorymap.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "dht22.h"
#include "acs712.h"
#include "mq2.h"
#include "door.h"
#include "alarm.h"
#include "app_config.h"
#include "app_context.h"
#include "hmi_protocol.h"
#include "hmi.h"
#include "hmi_pages.h"
#include "logic_iot.h"
#include "sensor_sim.h"
#include "relay.h"
#include "load_buttons.h"
#include "bsp_esp8266.h"
#include "wifi_cmd.h"
#include "data_output.h"
#include "light.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define POWER_SAMPLE_INTERVAL_MS  20U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t digital_sensor_tick;
static uint32_t dht22_task_tick;
static uint32_t power_task_tick;
static uint32_t system_task_tick;
static uint32_t hmi_refresh_tick;
static uint8_t hmi_refresh_page = 1U;
static uint32_t load_button_tick;
volatile uint8_t gRX_BufF[1];
extern volatile uint8_t TcpClosedFlag;
static uint32_t tcp_task_tick;
static uint8_t tcp_send_fail_count;
static uint8_t last_phone_link_state = 0xFFU;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**************************关闭标准库下的半主机模式****************************/
__ASM (".global __use_no_semihosting");
struct FILE
{
  int handle;
};
FILE __stdout;
void _sys_exit(int x)
{
  x = x;
}
void _ttywrch(int ch)
{
  ch = ch;
}
/* USART1 is reserved for structured sample data. Suppress debug printf. */
int fputc(int ch, FILE *f)
{
  (void)f;
  return ch;
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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_UART5_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  AppContext_Init();
  AppConfig_Init();
  HAL_TIM_Base_Start(&htim2);
  Relay_Init();
  Alarm_Init();
  ACS712_Init();
  LoadButtons_Init();
  SensorSim_Init();
  HMI_Protocol_Init();
  HMI_Page0_UpdateOverview();
  HMI_Page1_UpdateAllDevices();
  ESP8266_Init(&huart5, (uint8_t*)gRX_BufF, 115200);
  ESP8266_ConnectStart();
  WiFiCmd_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HMI_Protocol_Task();
    HMI_Task();

    ESP8266_ConnectTask();
    if (ESP8266_IsConnected())
    {
      g_app_ctx.ble_online_count = 1U;
      g_app_ctx.human_present = 1U;
    }
    else
    {
      g_app_ctx.ble_online_count = 0U;
      g_app_ctx.human_present = 0U;
      g_app_ctx.phone_sync_ok = 0U;
    }
    if (g_app_ctx.ble_online_count != last_phone_link_state)
    {
      last_phone_link_state = g_app_ctx.ble_online_count;
      HMI_Page0_UpdatePhoneStatus();
      HMI_Page2_UpdateSafety();
    }
    if (ESP8266_IsConnected())
    {
      WiFiCmd_Task();
    }

    if ((HAL_GetTick() - digital_sensor_tick) >= 100U)
    {
      digital_sensor_tick = HAL_GetTick();
      MQ2_Task();
      Door_Task();
      Light_Task();
    }

    if ((HAL_GetTick() - load_button_tick) >= 10U)
    {
      load_button_tick = HAL_GetTick();
      LoadButtons_Task();
    }

    if ((HAL_GetTick() - power_task_tick) >= POWER_SAMPLE_INTERVAL_MS)
    {
      power_task_tick = HAL_GetTick();
      ACS712_Task();
      DataOutput_SendSample();
    }

    if ((HAL_GetTick() - dht22_task_tick) >= 2000U)
    {
      dht22_task_tick = HAL_GetTick();
      DHT22_Task();
    }

    if ((HAL_GetTick() - system_task_tick) >= 1000U)
    {
      system_task_tick = HAL_GetTick();
      SensorSim_Task();

      LogicIot_Task();

      Alarm_Task();
    }

    if ((HAL_GetTick() - hmi_refresh_tick) >= 250U)
    {
      hmi_refresh_tick = HAL_GetTick();
      /* The global waveform keeps receiving points while other pages are shown. */
      HMI_Page0_PowerTrendSample(1U);
      switch (hmi_refresh_page)
      {
        case 0U: HMI_Page0_UpdateOverview();   break;
        case 1U: HMI_Page1_UpdateAllDevices(); break;
        case 2U: HMI_Page2_UpdateSafety();     break;
        default: HMI_Page3_UpdateMode();       break;
      }
      hmi_refresh_page = (uint8_t)((hmi_refresh_page + 1U) & 0x03U);
    }

    if ((HAL_GetTick() - tcp_task_tick) >= 1000U)
    {
      tcp_task_tick = HAL_GetTick();
      if (ESP8266_IsConnected())
      {
        char tcp_str[256];
        sprintf(tcp_str, "{\"t\":%.1f,\"v\":%.2f,\"c\":%.3f,\"p\":%.1f,\"e\":%.4f,\"s\":%d,\"d\":%d,\"h\":%d,\"l\":%d,\"k\":%d,\"x\":%d,\"z\":%d,\"m\":%d,\"a\":%d,\"u\":%d}\r\n",
                g_app_ctx.temperature_c,
                g_app_ctx.voltage_v,
                g_app_ctx.current_a,
                g_app_ctx.power_w,
                g_app_ctx.energy_kwh,
                g_app_ctx.smoke_detected,
                g_app_ctx.door_unlocked,
                g_app_ctx.human_present,
                g_app_ctx.light_on,
                g_app_ctx.socket_on,
                g_app_ctx.ac_on,
                g_app_ctx.lock_unlocked,
                g_app_ctx.mode,
                g_app_ctx.alarm_active,
                g_app_ctx.ble_online_count);
        if (ESP8266_SendString(ENABLE, tcp_str, strlen(tcp_str), Single_ID_0))
        {
          uint8_t sync_changed = (g_app_ctx.phone_sync_ok == 0U) ? 1U : 0U;
          g_app_ctx.phone_sync_ok = 1U;
          tcp_send_fail_count = 0U;
          if (sync_changed != 0U)
          {
            HMI_Page0_UpdatePhoneStatus();
          }
        }
        else
        {
          uint8_t sync_changed = g_app_ctx.phone_sync_ok;
          g_app_ctx.phone_sync_ok = 0U;
          if (sync_changed != 0U)
          {
            HMI_Page0_UpdatePhoneStatus();
          }
          if (++tcp_send_fail_count >= 3U)
          {
            TcpClosedFlag = 1U;
          }
        }
      }
    }

    if (TcpClosedFlag)
    {
      printf("[INFO] TCP disconnected, reconnecting...\r\n");
      TcpClosedFlag = 0;
      g_app_ctx.ble_online_count = 0U;
      g_app_ctx.human_present = 0U;
      g_app_ctx.phone_sync_ok = 0U;
      tcp_send_fail_count = 0U;
      WiFiCmd_Init();
      ESP8266_ConnectStart();
    }

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV4;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
