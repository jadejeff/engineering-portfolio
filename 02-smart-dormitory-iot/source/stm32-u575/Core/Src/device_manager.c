#include "device_manager.h"
#include "alarm.h"
#include "hmi_pages.h"
#include "relay.h"
#include <stdio.h>
#include <string.h>

static AppDeviceState *DeviceManager_StatePtr(AppDevice device)
{
    switch (device)
    {
        case APP_DEVICE_LIGHT:
            return &g_app_ctx.light_on;

        case APP_DEVICE_SOCKET:
            return &g_app_ctx.socket_on;

        case APP_DEVICE_AC:
            return &g_app_ctx.ac_on;

        case APP_DEVICE_LOCK:
            return &g_app_ctx.lock_unlocked;

        default:
            return 0;
    }
}

const char *DeviceManager_GetDeviceName(AppDevice device)
{
    switch (device)
    {
        case APP_DEVICE_LIGHT:
            return "灯光";

        case APP_DEVICE_SOCKET:
            return "插座";

        case APP_DEVICE_AC:
            return "空调";

        case APP_DEVICE_LOCK:
            return "门锁";

        default:
            return "未知设备";
    }
}

const char *DeviceManager_GetStateText(AppDevice device, AppDeviceState state)
{
    if (device == APP_DEVICE_AC)
    {
        return (state == APP_DEVICE_ON) ? "运行" : "待机";
    }

    if (device == APP_DEVICE_LOCK)
    {
        return (state == APP_DEVICE_ON) ? "未锁" : "已锁";
    }

    return (state == APP_DEVICE_ON) ? "开启" : "关闭";
}

AppDeviceState DeviceManager_GetDeviceState(AppDevice device)
{
    AppDeviceState *state = DeviceManager_StatePtr(device);

    if (state == 0)
    {
        return APP_DEVICE_OFF;
    }

    if (device == APP_DEVICE_LIGHT)
    {
        *state = LightRelay_Get() ? APP_DEVICE_ON : APP_DEVICE_OFF;
    }
    else if (device == APP_DEVICE_AC)
    {
        *state = AirConditioner_Get() ? APP_DEVICE_ON : APP_DEVICE_OFF;
    }

    return *state;
}

static void DeviceManager_PushLog(const char *line)
{
    uint8_t i;

    for (i = 3U; i > 0U; i--)
    {
        strncpy(g_app_ctx.device_logs[i],
                g_app_ctx.device_logs[i - 1U],
                sizeof(g_app_ctx.device_logs[i]) - 1U);
        g_app_ctx.device_logs[i][sizeof(g_app_ctx.device_logs[i]) - 1U] = '\0';
    }

    strncpy(g_app_ctx.device_logs[0], line, sizeof(g_app_ctx.device_logs[0]) - 1U);
    g_app_ctx.device_logs[0][sizeof(g_app_ctx.device_logs[0]) - 1U] = '\0';
}

static uint8_t DeviceManager_ShouldDeny(AppDevice device,
                                        AppDeviceState state,
                                        DeviceControlSource source)
{
    /* Power overload is alarm-only: it must never auto-close the socket. */
    if ((source == DEVICE_CTRL_SRC_AUTO) &&
        (device == APP_DEVICE_SOCKET) &&
        (state == APP_DEVICE_OFF) &&
        (overload_alarm != 0U) &&
        (smoke_alarm == 0U))
    {
        return 1U;
    }

    if (((source == DEVICE_CTRL_SRC_HMI) || (source == DEVICE_CTRL_SRC_WIFI)) &&
        (device == APP_DEVICE_SOCKET) &&
        (state == APP_DEVICE_ON) &&
        smoke_alarm)
    {
        return 1U;
    }

    return 0U;
}

static void DeviceManager_ReportDeny(AppDevice device, AppDeviceState state)
{
    char log_line[128];

    snprintf(g_app_ctx.last_operation,
             sizeof(g_app_ctx.last_operation),
             "操作：%s %s",
             DeviceManager_GetDeviceName(device),
             DeviceManager_GetStateText(device, state));

    snprintf(g_app_ctx.last_result,
             sizeof(g_app_ctx.last_result),
             "结果：已拒绝");

    snprintf(g_app_ctx.feedback,
             sizeof(g_app_ctx.feedback),
             "反馈：报警状态禁止操作");

    snprintf(log_line,
             sizeof(log_line),
             "[%lu秒] 拒绝 %s %s",
             (unsigned long)(HAL_GetTick() / 1000U),
             DeviceManager_GetDeviceName(device),
             DeviceManager_GetStateText(device, state));
    DeviceManager_PushLog(log_line);

    printf("[CMD] Deny %s %s\r\n",
           DeviceManager_GetDeviceName(device),
           DeviceManager_GetStateText(device, state));

    HMI_Page1_UpdateAllDevices();
    HMI_Page0_UpdateOverview();
}

static AppDeviceState DeviceManager_ApplyHardware(AppDevice device,
                                                  AppDeviceState state)
{
    uint8_t on = (state == APP_DEVICE_ON) ? 1U : 0U;

    switch (device)
    {
        case APP_DEVICE_LIGHT:
            LightRelay_Set(on);
            return LightRelay_Get() ? APP_DEVICE_ON : APP_DEVICE_OFF;

        case APP_DEVICE_AC:
            AirConditioner_Set(on);
            return AirConditioner_Get() ? APP_DEVICE_ON : APP_DEVICE_OFF;

        case APP_DEVICE_SOCKET:
            if (state == APP_DEVICE_OFF)
            {
                HighPowerLoads_ForceOff();
            }
            return state;

        default:
            return state;
    }
}

uint8_t DeviceManager_SetDevice(AppDevice device,
                                AppDeviceState state,
                                DeviceControlSource source)
{
    AppDeviceState *target = DeviceManager_StatePtr(device);
    const char *device_name;
    const char *state_text;
    const char *source_text;
    char log_line[128];
    AppDeviceState actual_state;
    uint8_t operation_ok;

    if (target == 0)
    {
        printf("[CMD] Unknown device 0x%02X\r\n", (unsigned int)device);
        return 0U;
    }

    if (DeviceManager_ShouldDeny(device, state, source))
    {
        DeviceManager_ReportDeny(device, state);
        return 0U;
    }

    actual_state = DeviceManager_ApplyHardware(device, state);
    *target = actual_state;
    operation_ok = (actual_state == state) ? 1U : 0U;
    g_app_ctx.device_action_count++;

    device_name = DeviceManager_GetDeviceName(device);
    state_text = DeviceManager_GetStateText(device, actual_state);
    source_text = (source == DEVICE_CTRL_SRC_AUTO)  ? "自动" :
                   (source == DEVICE_CTRL_SRC_WIFI) ? "无线" : "屏幕";

    snprintf(g_app_ctx.last_operation,
             sizeof(g_app_ctx.last_operation),
             "操作：%s %s",
             device_name,
             state_text);

    snprintf(g_app_ctx.last_result,
             sizeof(g_app_ctx.last_result),
             "结果：%s",
             operation_ok ? "成功" : "失败");

    snprintf(g_app_ctx.feedback,
             sizeof(g_app_ctx.feedback),
             "反馈：%s %s",
             device_name,
             state_text);

    snprintf(log_line,
             sizeof(log_line),
             "[%lu秒] %s %s %s",
             (unsigned long)(HAL_GetTick() / 1000U),
             source_text,
             device_name,
             state_text);
    DeviceManager_PushLog(log_line);

    printf("[CMD] %s %s %s\r\n",
           device_name,
           state_text,
           operation_ok ? "OK" : "FAIL");

    if (device == APP_DEVICE_AC)
    {
        printf("[GPIO] AC_REQ=%u PB12_ODR=%u PB13_ODR=%u PB12_PIN=%u PB13_PIN=%u\r\n",
               (unsigned int)((state == APP_DEVICE_ON) ? 1U : 0U),
               (unsigned int)((GPIOB->ODR & RB12_Pin) ? 1U : 0U),
               (unsigned int)((GPIOB->ODR & RB13_Pin) ? 1U : 0U),
               (unsigned int)HAL_GPIO_ReadPin(RB12_GPIO_Port, RB12_Pin),
               (unsigned int)HAL_GPIO_ReadPin(RB13_GPIO_Port, RB13_Pin));
    }

    HMI_Page1_UpdateAllDevices();
    HMI_Page0_UpdateOverview();
    return operation_ok;
}
