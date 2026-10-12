#include "wifi_cmd.h"
#include "app_context.h"
#include "device_manager.h"
#include "mode_manager.h"
#include "bsp_esp8266.h"
#include "hmi_pages.h"
#include <stdio.h>
#include <string.h>

#define WIFI_RX_QUEUE_SIZE  128U

static volatile uint8_t s_rx_queue[WIFI_RX_QUEUE_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static volatile uint16_t s_rx_count;
static char s_closed_window[6];
static uint8_t s_pending_cmd;
static uint8_t s_have_pending_cmd;

void WiFiCmd_Init(void)
{
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_count = 0U;
    s_pending_cmd = 0U;
    s_have_pending_cmd = 0U;
    memset(s_closed_window, 0, sizeof(s_closed_window));
}

void WiFiCmd_PushRxByteFromISR(uint8_t data)
{
    uint8_t i;

    if (s_rx_count < WIFI_RX_QUEUE_SIZE)
    {
        s_rx_queue[s_rx_head] = data;
        s_rx_head = (uint16_t)((s_rx_head + 1U) % WIFI_RX_QUEUE_SIZE);
        s_rx_count++;
    }

    for (i = 0U; i < 5U; i++)
    {
        s_closed_window[i] = s_closed_window[i + 1U];
    }
    s_closed_window[5] = (char)data;
    if (memcmp(s_closed_window, "CLOSED", 6U) == 0)
    {
        TcpClosedFlag = 1U;
    }
}

/* Send full 15-field JSON response over TCP */
static void WiFiCmd_SendJsonResponse(void)
{
    char tcp_str[256];
    uint8_t previous_sync_state = g_app_ctx.phone_sync_ok;
    sprintf(tcp_str,
            "{\"t\":%.1f,\"v\":%.2f,\"c\":%.3f,\"p\":%.1f,\"e\":%.4f,"
            "\"s\":%d,\"d\":%d,\"h\":%d,\"l\":%d,\"k\":%d,"
            "\"x\":%d,\"z\":%d,\"m\":%d,\"a\":%d,\"u\":%d}\r\n",
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
        g_app_ctx.phone_sync_ok = 1U;
    }
    else
    {
        g_app_ctx.phone_sync_ok = 0U;
    }

    if (g_app_ctx.phone_sync_ok != previous_sync_state)
    {
        HMI_Page0_UpdatePhoneStatus();
    }
}

/* Dispatch a single 2-byte command (cmd, op) */
static void WiFiCmd_Process(uint8_t cmd, uint8_t op)
{
    AppDeviceState state;

    printf("[WiFi] CMD=0x%02X OP=0x%02X\r\n", cmd, op);

    switch (cmd)
    {
        /* ---- Device commands (A0-A3) ---- */
        case WIFI_CMD_LED:
            state = (op != 0x00U) ? APP_DEVICE_ON : APP_DEVICE_OFF;
            (void)DeviceManager_SetDevice(APP_DEVICE_LIGHT, state,
                                          DEVICE_CTRL_SRC_WIFI);
            break;

        case WIFI_CMD_AC:
            state = (op != 0x00U) ? APP_DEVICE_ON : APP_DEVICE_OFF;
            (void)DeviceManager_SetDevice(APP_DEVICE_AC, state,
                                          DEVICE_CTRL_SRC_WIFI);
            break;

        case WIFI_CMD_SOCKET:
            state = (op != 0x00U) ? APP_DEVICE_ON : APP_DEVICE_OFF;
            (void)DeviceManager_SetDevice(APP_DEVICE_SOCKET, state,
                                          DEVICE_CTRL_SRC_WIFI);
            break;

        case WIFI_CMD_LOCK:
            /* A3 01 = lock (unlocked=OFF), A3 00 = unlock (unlocked=ON) */
            state = (op != 0x00U) ? APP_DEVICE_OFF : APP_DEVICE_ON;
            (void)DeviceManager_SetDevice(APP_DEVICE_LOCK, state,
                                          DEVICE_CTRL_SRC_WIFI);
            break;

        /* ---- Scene commands (B0) ---- */
        case WIFI_CMD_SCENE:
            switch (op)
            {
                case 0x01U: /* SLEEP  — energy mode + force light off */
                    (void)ModeManager_SetMode(APP_MODE_ENERGY);
                    (void)DeviceManager_SetDevice(APP_DEVICE_LIGHT,
                                                  APP_DEVICE_OFF,
                                                  DEVICE_CTRL_SRC_WIFI);
                    break;

                case 0x02U: /* STUDY  — normal mode + force light on */
                    (void)ModeManager_SetMode(APP_MODE_NORMAL);
                    (void)DeviceManager_SetDevice(APP_DEVICE_LIGHT,
                                                  APP_DEVICE_ON,
                                                  DEVICE_CTRL_SRC_WIFI);
                    break;

                case 0x03U: /* REST   — normal mode + force light off */
                    (void)ModeManager_SetMode(APP_MODE_NORMAL);
                    (void)DeviceManager_SetDevice(APP_DEVICE_LIGHT,
                                                  APP_DEVICE_OFF,
                                                  DEVICE_CTRL_SRC_WIFI);
                    break;

                default:
                    return; /* Unknown scene — no response */
            }
            break;

        /* ---- System mode commands (B1) ---- */
        case WIFI_CMD_SYSMODE:
            switch (op)
            {
                case 0x00U: (void)ModeManager_SetMode(APP_MODE_NORMAL);   break;
                case 0x01U: (void)ModeManager_SetMode(APP_MODE_ENERGY);   break;
                case 0x02U: (void)ModeManager_SetMode(APP_MODE_SECURITY); break;
                default:    return; /* Unknown mode — no response */
            }
            break;

        /* ---- Query command (C0) — just send response, no state change ---- */
        case WIFI_CMD_QUERY:
            break;

        default:
            return; /* Unknown command code — no response */
    }

    /* Every supported command gets a full JSON response */
    WiFiCmd_SendJsonResponse();
}

static uint8_t WiFiCmd_IsCommandByte(uint8_t data)
{
    return ((data >= WIFI_CMD_LED) && (data <= WIFI_CMD_LOCK)) ||
           (data == WIFI_CMD_SCENE) ||
           (data == WIFI_CMD_SYSMODE) ||
           (data == WIFI_CMD_QUERY);
}

static uint8_t WiFiCmd_IsValidOperation(uint8_t cmd, uint8_t op)
{
    if ((cmd >= WIFI_CMD_LED) && (cmd <= WIFI_CMD_LOCK))
    {
        return (op <= 0x01U) ? 1U : 0U;
    }

    if (cmd == WIFI_CMD_SCENE)
    {
        return ((op >= 0x01U) && (op <= 0x03U)) ? 1U : 0U;
    }

    if (cmd == WIFI_CMD_SYSMODE)
    {
        return (op <= 0x02U) ? 1U : 0U;
    }

    return (cmd == WIFI_CMD_QUERY) ? 1U : 0U;
}

static uint8_t WiFiCmd_PopRxByte(uint8_t *data)
{
    uint32_t primask;

    if ((data == NULL) || (s_rx_count == 0U))
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (s_rx_count == 0U)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return 0U;
    }

    *data = s_rx_queue[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) % WIFI_RX_QUEUE_SIZE);
    s_rx_count--;
    if (primask == 0U)
    {
        __enable_irq();
    }
    return 1U;
}

void WiFiCmd_Task(void)
{
    uint8_t data;
    uint8_t scanned = 0U;

    while ((scanned < 16U) && (WiFiCmd_PopRxByte(&data) != 0U))
    {
        scanned++;

        if (s_have_pending_cmd == 0U)
        {
            if (WiFiCmd_IsCommandByte(data) != 0U)
            {
                s_pending_cmd = data;
                s_have_pending_cmd = 1U;
            }
            continue;
        }

        if (WiFiCmd_IsValidOperation(s_pending_cmd, data) != 0U)
        {
            uint8_t cmd = s_pending_cmd;
            s_have_pending_cmd = 0U;
            WiFiCmd_Process(cmd, data);
            return;
        }

        s_have_pending_cmd = 0U;
        if (WiFiCmd_IsCommandByte(data) != 0U)
        {
            s_pending_cmd = data;
            s_have_pending_cmd = 1U;
        }
    }
}
