#include "hmi_protocol.h"
#include "alarm.h"
#include "app_config.h"
#include "device_manager.h"
#include "mode_manager.h"
#include "bsp_esp8266.h"
#include "wifi_cmd.h"
#include "hmi.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

extern volatile uint8_t gRX_BufF[1];

#define HMI_FRAME_HEAD 0xA5U
#define HMI_FRAME_TAIL 0x5AU
#define HMI_FRAME_MAX_LEN 5U
#define HMI_RX_QUEUE_SIZE 8U

#define HMI_CMD_DEVICE 0x20U
#define HMI_CMD_MODE   0x30U
#define HMI_CMD_CONFIG 0x31U
#define HMI_CMD_ALARM_ACK    0x40U
#define HMI_CMD_ALARM_DETAIL 0x41U

static uint8_t s_uart3_rx_byte;
static volatile uint32_t s_uart5_rx_retry_tick;
static uint8_t s_rx_buf[HMI_FRAME_MAX_LEN];
static uint8_t s_rx_count;
static uint8_t s_page_response[5];
static uint8_t s_page_response_count;
static volatile uint8_t s_current_page = 0xFFU;

static uint8_t s_frame_queue[HMI_RX_QUEUE_SIZE][HMI_FRAME_MAX_LEN];
static uint8_t s_frame_len[HMI_RX_QUEUE_SIZE];
static volatile uint8_t s_frame_head;
static volatile uint8_t s_frame_tail;
static volatile uint8_t s_frame_count;

static uint8_t HMI_ExpectedLen(uint8_t cmd)
{
    switch (cmd)
    {
        case HMI_CMD_DEVICE:
        case HMI_CMD_CONFIG:
            return 5U;

        case HMI_CMD_MODE:
        case HMI_CMD_ALARM_ACK:
        case HMI_CMD_ALARM_DETAIL:
            return 4U;

        default:
            return 0U;
    }
}

static void HMI_ResetRx(void)
{
    s_rx_count = 0U;
}

static void HMI_PushByte(uint8_t data)
{
    uint8_t expected_len;

    if (s_page_response_count > 0U)
    {
        s_page_response[s_page_response_count++] = data;
        if (s_page_response_count >= sizeof(s_page_response))
        {
            if ((s_page_response[2] == 0xFFU) &&
                (s_page_response[3] == 0xFFU) &&
                (s_page_response[4] == 0xFFU))
            {
                s_current_page = s_page_response[1];
            }
            s_page_response_count = 0U;
        }
        return;
    }

    if ((s_rx_count == 0U) && (data == 0x66U))
    {
        s_page_response[0] = data;
        s_page_response_count = 1U;
        return;
    }

    if (s_rx_count == 0U)
    {
        if (data == HMI_FRAME_HEAD)
        {
            s_rx_buf[s_rx_count++] = data;
        }
        return;
    }

    if (data == HMI_FRAME_HEAD)
    {
        s_rx_buf[0] = data;
        s_rx_count = 1U;
        return;
    }

    if (s_rx_count >= HMI_FRAME_MAX_LEN)
    {
        HMI_ResetRx();
        return;
    }

    s_rx_buf[s_rx_count++] = data;

    if (s_rx_count < 2U)
    {
        return;
    }

    expected_len = HMI_ExpectedLen(s_rx_buf[1]);
    if (expected_len == 0U)
    {
        HMI_ResetRx();
        return;
    }

    if (s_rx_count == expected_len)
    {
        if ((s_rx_buf[0] == HMI_FRAME_HEAD) &&
            (s_rx_buf[expected_len - 1U] == HMI_FRAME_TAIL) &&
            (s_frame_count < HMI_RX_QUEUE_SIZE))
        {
            memcpy(s_frame_queue[s_frame_head], s_rx_buf, expected_len);
            s_frame_len[s_frame_head] = expected_len;
            s_frame_head = (uint8_t)((s_frame_head + 1U) % HMI_RX_QUEUE_SIZE);
            s_frame_count++;
        }

        HMI_ResetRx();
    }
}

static void HMI_PrintFrame(const uint8_t *frame, uint8_t len)
{
    uint8_t i;

    printf("[HMI RX]");
    for (i = 0U; i < len; i++)
    {
        printf(" %02X", frame[i]);
    }
    printf("\r\n");
}

static void HMI_HandleDeviceCmd(const uint8_t *frame, uint8_t len)
{
    AppDevice device;
    AppDeviceState state;

    if (len != 5U)
    {
        return;
    }

    device = (AppDevice)frame[2];
    state = (frame[3] != 0U) ? APP_DEVICE_ON : APP_DEVICE_OFF;

    (void)DeviceManager_SetDevice(device, state, DEVICE_CTRL_SRC_HMI);
}

static void HMI_HandleModeCmd(const uint8_t *frame, uint8_t len)
{
    if (len != 4U)
    {
        return;
    }

    (void)ModeManager_SetMode((AppMode)frame[2]);
}

static void HMI_HandleConfigCmd(const uint8_t *frame, uint8_t len)
{
    if (len != 5U)
    {
        return;
    }

    (void)AppConfig_SetFromHmi(frame[2], frame[3]);
}

static void HMI_HandleAlarmCmd(const uint8_t *frame, uint8_t len)
{
    if (len != 4U)
    {
        return;
    }

    if ((frame[1] == HMI_CMD_ALARM_DETAIL) &&
        ((frame[2] == 0x01U) || (frame[2] == 0x02U)))
    {
        printf("[ALARM] Detail\r\n");
        Alarm_ShowDetail();
    }
    else if ((frame[1] == HMI_CMD_ALARM_ACK) && (frame[2] == 0x00U))
    {
        printf("[ALARM] Confirm\r\n");
        Alarm_Confirm();
    }
}

static void HMI_HandleFrame(const uint8_t *frame, uint8_t len)
{
    HMI_PrintFrame(frame, len);

    switch (frame[1])
    {
        case HMI_CMD_DEVICE:
            HMI_HandleDeviceCmd(frame, len);
            break;

        case HMI_CMD_MODE:
            HMI_HandleModeCmd(frame, len);
            break;

        case HMI_CMD_CONFIG:
            HMI_HandleConfigCmd(frame, len);
            break;

        case HMI_CMD_ALARM_ACK:
        case HMI_CMD_ALARM_DETAIL:
            HMI_HandleAlarmCmd(frame, len);
            break;

        default:
            printf("[CMD] Unsupported 0x%02X\r\n", frame[1]);
            break;
    }
}

void HMI_Protocol_Init(void)
{
    HMI_ResetRx();
    s_frame_head = 0U;
    s_frame_tail = 0U;
    s_frame_count = 0U;
    s_page_response_count = 0U;
    s_current_page = 0xFFU;

    HAL_UART_Receive_IT(&huart3, &s_uart3_rx_byte, 1U);
}

uint8_t HMI_Protocol_GetCurrentPage(void)
{
    return s_current_page;
}

void HMI_Protocol_Task(void)
{
    uint8_t frame[HMI_FRAME_MAX_LEN];
    uint8_t len;
    uint8_t processed = 0U;
    uint32_t primask;

    if (huart3.RxState == HAL_UART_STATE_READY)
    {
        (void)HAL_UART_Receive_IT(&huart3, &s_uart3_rx_byte, 1U);
    }
    if ((huart5.RxState == HAL_UART_STATE_READY) &&
        ((int32_t)(HAL_GetTick() - s_uart5_rx_retry_tick) >= 0))
    {
        (void)HAL_UART_Receive_IT(&huart5, (uint8_t *)gRX_BufF, 1U);
    }

    while ((s_frame_count > 0U) && (processed < 4U))
    {
        primask = __get_PRIMASK();
        __disable_irq();
        len = s_frame_len[s_frame_tail];
        memcpy(frame, s_frame_queue[s_frame_tail], len);
        s_frame_tail = (uint8_t)((s_frame_tail + 1U) % HMI_RX_QUEUE_SIZE);
        s_frame_count--;
        if (primask == 0U)
        {
            __enable_irq();
        }

        HMI_HandleFrame(frame, len);
        processed++;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        HMI_PushByte(s_uart3_rx_byte);
        HAL_UART_Receive_IT(&huart3, &s_uart3_rx_byte, 1U);
    }
    else if (huart->Instance == UART5)
    {
        if (ESP8266_IsConnected() != 0U)
        {
            WiFiCmd_PushRxByteFromISR(gRX_BufF[0]);
        }
        else if (ESP8266_Fram_Record_Struct.InfBit.FramLength < (RX_BUF_MAX_LEN - 1U))
        {
            uint16_t index = ESP8266_Fram_Record_Struct.InfBit.FramLength++;
            ESP8266_Fram_Record_Struct.Data_RX_BUF[index] = (char)gRX_BufF[0];
            ESP8266_Fram_Record_Struct.Data_RX_BUF[index + 1U] = '\0';
        }
        HAL_UART_Receive_IT(&huart5, (uint8_t *)gRX_BufF, 1U);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        HMI_ResetRx();
        HAL_UART_Receive_IT(&huart3, &s_uart3_rx_byte, 1U);
    }
    else if (huart->Instance == UART5)
    {
        /* Back off briefly if the wireless bridge produces framing/overrun errors. */
        s_uart5_rx_retry_tick = HAL_GetTick() + 5U;
    }
}

void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        if (ESP8266_IsConnected() == 0U)
        {
            uint16_t length = ESP8266_Fram_Record_Struct.InfBit.FramLength;
            if (length >= RX_BUF_MAX_LEN)
            {
                length = RX_BUF_MAX_LEN - 1U;
            }
            ESP8266_Fram_Record_Struct.Data_RX_BUF[length] = '\0';
            ESP8266_Fram_Record_Struct.InfBit.FramFinishFlag = 1U;
            TcpClosedFlag = (strstr(ESP8266_Fram_Record_Struct.Data_RX_BUF,
                                    "CLOSED") != NULL) ? 1U : 0U;
        }
        HAL_UART_Receive_IT(&huart5, (uint8_t *)gRX_BufF, 1U);
    }
}
