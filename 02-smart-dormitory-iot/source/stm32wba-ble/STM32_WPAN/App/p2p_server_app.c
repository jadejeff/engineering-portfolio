/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    p2p_server_app.c
  * @author  MCD Application Team
  * @brief   p2p_server_app application definition.
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
#include "app_common.h"
#include "log_module.h"
#include "app_ble.h"
#include "ll_sys_if.h"
#include "dbg_trace.h"
#include "p2p_server_app.h"
#include "p2p_server.h"
#include "stm32_rtos.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_bsp.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
 typedef struct{
    uint8_t             Device_Led_Selection;
    uint8_t             Led1;
 }P2P_LedCharValue_t;

 typedef struct{
    uint8_t             Device_Button_Selection;
    uint8_t             ButtonStatus;
 }P2P_ButtonCharValue_t;

/* USER CODE END PTD */

typedef enum
{
  Switch_c_NOTIFICATION_OFF,
  Switch_c_NOTIFICATION_ON,
  /* USER CODE BEGIN Service1_APP_SendInformation_t */

  /* USER CODE END Service1_APP_SendInformation_t */
  P2P_SERVER_APP_SENDINFORMATION_LAST
} P2P_SERVER_APP_SendInformation_t;

typedef struct
{
  P2P_SERVER_APP_SendInformation_t     Switch_c_Notification_Status;
  /* USER CODE BEGIN Service1_APP_Context_t */
  P2P_LedCharValue_t              LedControl;
  P2P_ButtonCharValue_t           ButtonControl;

  /* USER CODE END Service1_APP_Context_t */
  uint16_t              ConnectionHandle;
} P2P_SERVER_APP_Context_t;

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern void USART3_SendString_IT(uint8_t *str, uint16_t len);  /* 非阻塞中断发送 */

/* USER CODE END EV */

/* Private macros ------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
static P2P_SERVER_APP_Context_t P2P_SERVER_APP_Context;

uint8_t a_P2P_SERVER_UpdateCharData[247];

/* USER CODE BEGIN PV */
#define G_NOTIFY_BUF_SIZE  128  /* JSON 约 88~120 字节，128 充裕 */
uint8_t g_notify_buf[G_NOTIFY_BUF_SIZE] = "0";

#define G_PHONE_RX_BUF_SIZE  256
uint8_t  g_phone_rx_buf[G_PHONE_RX_BUF_SIZE];  /* 手机发来的数据 */
uint16_t g_phone_rx_len = 0;                    /* 手机发来数据的长度 */

/* ---- NanoEdge AI 全局变量 ---- */
#define AI_LEARNING_MIN_SAMPLES  80     /* 学习帧数要求 (NanoEdge 建议 ≥217, 实测 81 帧达 94.66% 准确率) */
uint8_t  g_ai_learning_done = 0;        /* 0=学习中, 1=检测中 */
uint16_t g_ai_learning_count = 0;       /* 已学习帧数 */
float    g_ai_input_buf[3];             /* {t, c, p} 临时存储 */
uint8_t  g_ai_alert_buf[4];             /* "111\0" 告警消息缓冲 */
uint8_t  g_ai_similarity = 0;           /* 最新相似度 (0-100) */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void P2P_SERVER_Switch_c_SendNotification(void);

/* USER CODE BEGIN PFP */
static void P2P_SERVER_APP_LED_BUTTON_context_Init(void);

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void P2P_SERVER_Notification(P2P_SERVER_NotificationEvt_t *p_Notification)
{
  /* USER CODE BEGIN Service1_Notification_1 */

  /* USER CODE END Service1_Notification_1 */
  switch(p_Notification->EvtOpcode)
  {
    /* USER CODE BEGIN Service1_Notification_Service1_EvtOpcode */

    /* USER CODE END Service1_Notification_Service1_EvtOpcode */

    case P2P_SERVER_LED_C_READ_EVT:
      /* USER CODE BEGIN Service1Char1_READ_EVT */

      /* USER CODE END Service1Char1_READ_EVT */
      break;

    case P2P_SERVER_LED_C_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN Service1Char1_WRITE_NO_RESP_EVT */
    uint8_t *p_data = p_Notification->DataTransfered.p_Payload;
    uint16_t data_len = p_Notification->DataTransfered.Length;
    USART3_SendString_IT(p_data, data_len);  /* 非阻塞中断发送，不阻塞 BLE 事件处理 */

    /* 存储手机发来的数据 */
    if (data_len <= G_PHONE_RX_BUF_SIZE)
    {
      memcpy(g_phone_rx_buf, p_data, data_len);
      g_phone_rx_len = data_len;
    }

    /* 打印存储的数据：HEX + 可读ASCII */
    LOG_INFO_APP("\r\n-- [Phone -> WBA] len=%d\r\n", g_phone_rx_len);
    LOG_INFO_APP("   HEX: ");
    for (int i = 0; i < g_phone_rx_len; i++)
    {
      LOG_INFO_APP("%02X ", g_phone_rx_buf[i]);
    }
    LOG_INFO_APP("\r\n");
    LOG_INFO_APP("   RAW: ");
    for (int i = 0; i < g_phone_rx_len; i++)
    {
      uint8_t b = g_phone_rx_buf[i];
      LOG_INFO_APP("%c", (b >= 0x20 && b <= 0x7E) ? (char)b : '.');
    }
    LOG_INFO_APP("\r\n");

#if 0  // LED 亮灭功能已禁用
    uint8_t cmd = p_Notification->DataTransfered.p_Payload[0];

    if(cmd == 0x01)
    {
      APP_BSP_LED_On(LD1);
      LOG_INFO_APP("-- LED1 ON \r\n");
    }
    else if(cmd == 0x00)
    {
      APP_BSP_LED_Off(LD1);
      LOG_INFO_APP("-- LED1 OFF \r\n");
    }
#endif
      /* USER CODE END Service1Char1_WRITE_NO_RESP_EVT */
      break;

    case P2P_SERVER_SWITCH_C_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN Service1Char2_NOTIFY_ENABLED_EVT */
      P2P_SERVER_APP_Context.Switch_c_Notification_Status = Switch_c_NOTIFICATION_ON;
      LOG_INFO_APP("-- P2P APPLICATION SERVER : NOTIFICATION ENABLED\n");
      LOG_INFO_APP(" \n\r");
			UTIL_SEQ_SetTask(1U << CFG_TASK_SEND_NOTIF_ID, 0);
      /* USER CODE END Service1Char2_NOTIFY_ENABLED_EVT */
      break;

    case P2P_SERVER_SWITCH_C_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN Service1Char2_NOTIFY_DISABLED_EVT */
      P2P_SERVER_APP_Context.Switch_c_Notification_Status = Switch_c_NOTIFICATION_OFF;
      LOG_INFO_APP("-- P2P APPLICATION SERVER : NOTIFICATION DISABLED\n");
      LOG_INFO_APP(" \n\r");
      /* USER CODE END Service1Char2_NOTIFY_DISABLED_EVT */
      break;

    default:
      /* USER CODE BEGIN Service1_Notification_default */

      /* USER CODE END Service1_Notification_default */
      break;
  }
  /* USER CODE BEGIN Service1_Notification_2 */

  /* USER CODE END Service1_Notification_2 */
  return;
}

void P2P_SERVER_APP_EvtRx(P2P_SERVER_APP_ConnHandleNotEvt_t *p_Notification)
{
  /* USER CODE BEGIN Service1_APP_EvtRx_1 */

  /* USER CODE END Service1_APP_EvtRx_1 */

  switch(p_Notification->EvtOpcode)
  {
    /* USER CODE BEGIN Service1_APP_EvtRx_Service1_EvtOpcode */

    /* USER CODE END Service1_APP_EvtRx_Service1_EvtOpcode */
    case P2P_SERVER_CONN_HANDLE_EVT :
      /* USER CODE BEGIN Service1_APP_CONN_HANDLE_EVT */

      /* USER CODE END Service1_APP_CONN_HANDLE_EVT */
      break;

    case P2P_SERVER_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN Service1_APP_DISCON_HANDLE_EVT */

      /* USER CODE END Service1_APP_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN Service1_APP_EvtRx_default */

      /* USER CODE END Service1_APP_EvtRx_default */
      break;
  }

  /* USER CODE BEGIN Service1_APP_EvtRx_2 */

  /* USER CODE END Service1_APP_EvtRx_2 */

  return;
}

void P2P_SERVER_APP_Init(void)
{
  UNUSED(P2P_SERVER_APP_Context);
  P2P_SERVER_Init();

  /* USER CODE BEGIN Service1_APP_Init */
  UTIL_SEQ_RegTask( 1U << CFG_TASK_SEND_NOTIF_ID, UTIL_SEQ_RFU, P2P_SERVER_Switch_c_SendNotification);
  UTIL_SEQ_RegTask( 1U << CFG_TASK_AI_ALERT_ID, UTIL_SEQ_RFU, P2P_SERVER_AiAlert_SendNotification);

  /**
   * Initialize LedButton Service
   */
  P2P_SERVER_APP_Context.Switch_c_Notification_Status= Switch_c_NOTIFICATION_OFF;
  P2P_SERVER_APP_LED_BUTTON_context_Init();

  /* USER CODE END Service1_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* ---- NanoEdge AI 告警发送任务 (BLE 通知手机 "111") ---- */
__USED void P2P_SERVER_AiAlert_SendNotification(void)
{
  P2P_SERVER_Data_t p2p_server_notification_data;

  p2p_server_notification_data.p_Payload = g_ai_alert_buf;
  p2p_server_notification_data.Length = 3;  /* "111" 3 字节 */

  if (P2P_SERVER_APP_Context.Switch_c_Notification_Status == Switch_c_NOTIFICATION_ON)
  {
    P2P_SERVER_UpdateValue(P2P_SERVER_SWITCH_C, &p2p_server_notification_data);
  }
  return;
}

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
__USED void P2P_SERVER_Switch_c_SendNotification(void) /* Property Notification */
{
  P2P_SERVER_APP_SendInformation_t notification_on_off = Switch_c_NOTIFICATION_OFF;
  P2P_SERVER_Data_t p2p_server_notification_data;

  p2p_server_notification_data.p_Payload = (uint8_t*)a_P2P_SERVER_UpdateCharData;
  p2p_server_notification_data.Length = 0;

  /* USER CODE BEGIN Service1Char2_NS_1 */

  /* 发送全局缓冲区 g_notify_buf 给手机（只发实际数据长度） */
  memcpy(a_P2P_SERVER_UpdateCharData, g_notify_buf, G_NOTIFY_BUF_SIZE);
  {
    size_t actual_len = strnlen((const char *)g_notify_buf, G_NOTIFY_BUF_SIZE);
    p2p_server_notification_data.Length = (actual_len > 0) ? (uint16_t)actual_len : G_NOTIFY_BUF_SIZE;
  }

  /* Update notification data length */

  if(P2P_SERVER_APP_Context.Switch_c_Notification_Status == Switch_c_NOTIFICATION_ON)
  {
    notification_on_off = Switch_c_NOTIFICATION_ON;
  }
  else
  {
    LOG_INFO_APP("-- P2P SERVER : NOTIFICATION DISABLED\r\n");
  }

  /* USER CODE END Service1Char2_NS_1 */

  if (notification_on_off != Switch_c_NOTIFICATION_OFF)
  {
    P2P_SERVER_UpdateValue(P2P_SERVER_SWITCH_C, &p2p_server_notification_data);
  }

  /* USER CODE BEGIN Service1Char2_NS_Last */
  /* 不再自动循环调度，由 USART3 收到新数据时触发 */
  /* USER CODE END Service1Char2_NS_Last */

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS */
static void P2P_SERVER_APP_LED_BUTTON_context_Init(void)
{
  APP_BSP_LED_Off(LED_BLUE);
  P2P_SERVER_APP_Context.LedControl.Device_Led_Selection=0x01;        /* select device 01 */
  P2P_SERVER_APP_Context.LedControl.Led1=0x00;                        /* led OFF */
  P2P_SERVER_APP_Context.ButtonControl.Device_Button_Selection=0x01;  /* select device 01 */
  P2P_SERVER_APP_Context.ButtonControl.ButtonStatus=0x00;

  return;
}

/* USER CODE END FD_LOCAL_FUNCTIONS */
