#include "hmi_pages.h"
#include "app_context.h"
#include "hmi.h"
#include "alarm.h"
#include "device_manager.h"
#include "mode_manager.h"
#include <stdio.h>

#define HMI_PIC_OFF 2U
#define HMI_PIC_ON  3U
#define HMI_POWER_TREND_MAX_W 40.0f
#define HMI_POWER_HISTORY_SIZE 192U
#define HMI_POWER_REDRAW_BATCH 24U

static uint8_t s_power_history[HMI_POWER_HISTORY_SIZE];
static uint8_t s_power_history_head;
static uint8_t s_power_history_count;
static uint8_t s_power_redraw_data[HMI_POWER_HISTORY_SIZE];
static uint8_t s_power_redraw_index;
static uint8_t s_power_redraw_count;
static uint32_t s_run_info_start_tick;
static uint8_t s_run_info_start_valid;

static uint8_t HMI_Page0_MapPowerTrendValue(void)
{
    float ratio;

    if (g_app_ctx.power_sensor_ok == 0U)
    {
        return 0U;
    }

    ratio = g_app_ctx.power_w / HMI_POWER_TREND_MAX_W;
    if (ratio < 0.0f)
    {
        ratio = 0.0f;
    }
    else if (ratio > 1.0f)
    {
        ratio = 1.0f;
    }

    /* Lift the low-power region while preserving 40 W as full scale. */
    ratio = (2.0f * ratio) - (ratio * ratio);

    return (uint8_t)(ratio * 255.0f);
}

static void HMI_Page0_SendPowerTrendValue(uint8_t value)
{
    char command[32];

    snprintf(command,
             sizeof(command),
             "add s0.id,0,%u",
             (unsigned int)value);
    HMI_SendCmd(command);
}

void HMI_Page0_PowerTrendSample(uint8_t page_visible)
{
    uint8_t curve_value;
    uint8_t batch_count = 0U;

    if (g_app_ctx.power_sensor_ok == 0U)
    {
        return;
    }

    curve_value = HMI_Page0_MapPowerTrendValue();
    s_power_history[s_power_history_head] = curve_value;
    s_power_history_head =
        (uint8_t)((s_power_history_head + 1U) % HMI_POWER_HISTORY_SIZE);
    if (s_power_history_count < HMI_POWER_HISTORY_SIZE)
    {
        s_power_history_count++;
    }

    if (page_visible == 0U)
    {
        return;
    }

    while ((s_power_redraw_index < s_power_redraw_count) &&
           (batch_count < HMI_POWER_REDRAW_BATCH))
    {
        HMI_Page0_SendPowerTrendValue(
            s_power_redraw_data[s_power_redraw_index++]);
        batch_count++;
    }

    if (s_power_redraw_index >= s_power_redraw_count)
    {
        s_power_redraw_count = 0U;
    }

    if (batch_count == 0U)
    {
        HMI_Page0_SendPowerTrendValue(curve_value);
    }
}

void HMI_Page0_BeginPowerTrendRedraw(void)
{
    uint8_t oldest;
    uint8_t i;

    oldest = (uint8_t)((s_power_history_head + HMI_POWER_HISTORY_SIZE -
                        s_power_history_count) % HMI_POWER_HISTORY_SIZE);
    for (i = 0U; i < s_power_history_count; i++)
    {
        s_power_redraw_data[i] =
            s_power_history[(uint8_t)((oldest + i) % HMI_POWER_HISTORY_SIZE)];
    }

    s_power_redraw_index = 0U;
    s_power_redraw_count = s_power_history_count;
    HMI_SendCmd("cle s0.id,0");
}

static const char *HMI_ModeText(AppMode mode)
{
    switch (mode)
    {
        case APP_MODE_ENERGY:
            return "模式：节能";

        case APP_MODE_SECURITY:
            return "模式：安全";

        case APP_MODE_NORMAL:
        default:
            return "模式：普通";
    }
}

static uint8_t HMI_Page1_RunCount(void)
{
    uint8_t count = 0U;

    if (g_app_ctx.light_on == APP_DEVICE_ON)
    {
        count++;
    }

    if (g_app_ctx.socket_on == APP_DEVICE_ON)
    {
        count++;
    }

    if (g_app_ctx.ac_on == APP_DEVICE_ON)
    {
        count++;
    }

    return count;
}

static uint8_t HMI_HighPowerLoadActive(void)
{
    return (g_app_ctx.high_power_load_on[0] ||
            g_app_ctx.high_power_load_on[1] ||
            g_app_ctx.high_power_load_on[2]) ? 1U : 0U;
}

void HMI_Page0_UpdatePhoneStatus(void)
{
    char text[8];

    snprintf(text, sizeof(text), "%u", g_app_ctx.ble_online_count);
    HMI_SetText("page0.t_onl", text);
    HMI_SetText("page0.t_ble",
                (g_app_ctx.ble_online_count > 0U) ? "蓝牙在线" : "蓝牙离线");
    HMI_SetText("page0.t1", g_app_ctx.phone_sync_ok ?
                "同步成功" :
                ((g_app_ctx.ble_online_count > 0U) ? "等待同步" : "未同步"));
    HMI_SetText("page0.t_phone",
                (g_app_ctx.ble_online_count > 0U) ? "连接正常" : "等待连接");
}

void HMI_Page0_UpdateOverview(void)
{
    char text[32];

    if (g_app_ctx.power_sensor_ok == 0U)
    {
        HMI_SetText("t_pwr", "读取失败");
        HMI_SetText("t_cur", "读取失败");
    }
    else
    {
        snprintf(text, sizeof(text), "%.1f", g_app_ctx.power_w);
        HMI_SetText("t_pwr", text);

        snprintf(text, sizeof(text), "%.3f", g_app_ctx.current_a);
        HMI_SetText("t_cur", text);
    }

    snprintf(text, sizeof(text), "%.2f", g_app_ctx.voltage_v);
    HMI_SetText("t_vol", text);

    snprintf(text, sizeof(text), "%.4f", g_app_ctx.energy_kwh);
    HMI_SetText("t_eng", text);

    if (smoke_alarm || fire_alarm || overload_alarm)
    {
        HMI_SetText("t_alm", "严重");
    }
    else if (door_alarm)
    {
        HMI_SetText("t_alm", "注意");
    }
    else
    {
        HMI_SetText("t_alm", "正常");
    }

    HMI_SetText("t_alm1", smoke_alarm ?
                "烟雾：" HMI_ALARM_GAP "异常" :
                "烟雾：" HMI_ALARM_GAP "正常");
    if (g_app_ctx.temp_sensor_ok == 0U)
    {
        HMI_SetText("t_alm2", "火灾：" HMI_ALARM_GAP "待检测");
    }
    else
    {
        HMI_SetText("t_alm2", fire_alarm ?
                    "火灾：" HMI_ALARM_GAP "有" :
                    "火灾：" HMI_ALARM_GAP "无");
    }
    HMI_SetText("t_alm3", door_alarm ?
                "门锁：" HMI_ALARM_GAP "未锁" :
                "门锁：" HMI_ALARM_GAP "已锁");
    HMI_SetText("t_alm4", overload_alarm ?
                "过载：" HMI_ALARM_GAP "有" :
                "过载：" HMI_ALARM_GAP "无");

    HMI_Page0_UpdatePhoneStatus();

    snprintf(text,
             sizeof(text),
             "灯光 %s",
             DeviceManager_GetStateText(APP_DEVICE_LIGHT,
                                        DeviceManager_GetDeviceState(APP_DEVICE_LIGHT)));
    HMI_SetText("page0.t_light", text);

    snprintf(text,
             sizeof(text),
             "插座 %s",
             DeviceManager_GetStateText(APP_DEVICE_SOCKET, g_app_ctx.socket_on));
    HMI_SetText("page0.t_sock", text);

    snprintf(text,
             sizeof(text),
             "空调 %s",
             DeviceManager_GetStateText(APP_DEVICE_AC,
                                        DeviceManager_GetDeviceState(APP_DEVICE_AC)));
    HMI_SetText("page0.t_ac", text);

    snprintf(text,
             sizeof(text),
             "门锁 %s",
             DeviceManager_GetStateText(APP_DEVICE_LOCK, g_app_ctx.lock_unlocked));
    HMI_SetText("page0.t_lock2", text);

}

void HMI_Page1_UpdateRunInfo(void)
{
    char text[32];
    uint8_t run_count = HMI_Page1_RunCount();
    uint32_t now = HAL_GetTick();
    uint32_t elapsed_s;
    uint32_t hours;
    uint32_t minutes;
    uint32_t seconds;

    if (s_run_info_start_valid == 0U)
    {
        s_run_info_start_tick = now;
        s_run_info_start_valid = 1U;
    }

    HMI_SetText("t_mode", HMI_ModeText(g_app_ctx.mode));

    snprintf(text, sizeof(text), "运行数：%u", run_count);
    HMI_SetText("t_runnum", text);

    HMI_SetText("t_load",
                ((run_count >= 3U) || HMI_HighPowerLoadActive()) ?
                "负载：高" : "负载：正常");
    HMI_SetText("t_lastop", g_app_ctx.last_operation);
    HMI_SetText("t_result", g_app_ctx.last_result);

    elapsed_s = (now - s_run_info_start_tick) / 1000U;
    hours = elapsed_s / 3600U;
    minutes = (elapsed_s / 60U) % 60U;
    seconds = elapsed_s % 60U;
    snprintf(text,
             sizeof(text),
             "时长：%02lu:%02lu:%02lu",
             (unsigned long)hours,
             (unsigned long)minutes,
             (unsigned long)seconds);
    HMI_SetText("t_time", text);

    HMI_SetText("t_fb", g_app_ctx.feedback);
    HMI_SetText("t_log1", g_app_ctx.device_logs[0]);
    HMI_SetText("t_log2", g_app_ctx.device_logs[1]);
    HMI_SetText("t_log3", g_app_ctx.device_logs[2]);
    HMI_SetText("t_log4", g_app_ctx.device_logs[3]);
}

void HMI_Page1_UpdateDevice(AppDevice device)
{
    AppDeviceState state = DeviceManager_GetDeviceState(device);
    uint16_t pic = (state == APP_DEVICE_ON) ? HMI_PIC_ON : HMI_PIC_OFF;

    switch (device)
    {
        case APP_DEVICE_LIGHT:
            HMI_SetText("page1.t_light", DeviceManager_GetStateText(device, state));
            HMI_SetPic("page1.p_light_sw", pic);
            break;

        case APP_DEVICE_SOCKET:
            HMI_SetText("page1.t_sock", DeviceManager_GetStateText(device, state));
            HMI_SetPic("page1.p_sock_sw", pic);
            break;

        case APP_DEVICE_AC:
            HMI_SetText("page1.t_ac", DeviceManager_GetStateText(device, state));
            HMI_SetPic("page1.p_ac_sw", pic);
            break;

        case APP_DEVICE_LOCK:
            HMI_SetText("page1.t_lock", DeviceManager_GetStateText(device, state));
            HMI_SetPic("page1.p_lock_sw", pic);
            break;

        default:
            break;
    }
}

void HMI_Page1_UpdateLight(void)
{
    HMI_Page1_UpdateDevice(APP_DEVICE_LIGHT);
}

void HMI_Page1_UpdateAllDevices(void)
{
    HMI_Page1_UpdateDevice(APP_DEVICE_LIGHT);
    HMI_Page1_UpdateDevice(APP_DEVICE_SOCKET);
    HMI_Page1_UpdateDevice(APP_DEVICE_AC);
    HMI_Page1_UpdateDevice(APP_DEVICE_LOCK);
    HMI_Page1_UpdateRunInfo();
}

void HMI_Page2_UpdateSafety(void)
{
    char text[32];

    HMI_SetText("page2.t_smoke", smoke_alarm ? "异常" : "正常");

    snprintf(text, sizeof(text), "%.1fC", g_app_ctx.temperature_c);
    HMI_SetText("t_temp", text);

    HMI_SetText("page2.t_human", g_app_ctx.human_present ? "有人" : "无人");
    HMI_SetText("page2.t_lock", door_alarm ? "未锁" : "已锁");

    HMI_SetText("t_alarmio", g_app_ctx.alarm_active ? "报警：开启" : "报警：关闭");
    HMI_SetText("t_buzzer", g_app_ctx.buzzer_on ? "蜂鸣器：开启" : "蜂鸣器：关闭");
    HMI_SetText("t_lightio", g_app_ctx.alarm_active ? "警示灯：开启" : "警示灯：关闭");

    HMI_SetText("t_slog1", smoke_alarm ? "烟雾异常" : "烟雾正常");
    HMI_SetText("t_slog2",
                (g_app_ctx.temp_sensor_ok == 0U) ? "温度读取失败" :
                (fire_alarm ? "检测到火灾" : "未检测到火灾"));
    HMI_SetText("t_slog3", door_alarm ? "门锁未锁" : "门锁已锁");
    HMI_SetText("t_slog4", g_app_ctx.human_present ? "房间有人" : "房间无人");
}

void HMI_Page3_UpdateMode(void)
{
    uint8_t normal_on = (g_app_ctx.mode == APP_MODE_NORMAL);
    uint8_t energy_on = (g_app_ctx.mode == APP_MODE_ENERGY);
    uint8_t security_on = (g_app_ctx.mode == APP_MODE_SECURITY);

    HMI_SetPic("pn", normal_on ? HMI_PIC_ON : HMI_PIC_OFF);
    HMI_SetPic("pe", energy_on ? HMI_PIC_ON : HMI_PIC_OFF);
    HMI_SetPic("ps", security_on ? HMI_PIC_ON : HMI_PIC_OFF);

    HMI_SetText("t_normal", normal_on ? "已启用" : "未启用");
    HMI_SetText("t_eco", energy_on ? "已启用" : "未启用");
    HMI_SetText("t_safe", security_on ? "已启用" : "未启用");

    HMI_SetText("t_state3", HMI_ModeText(g_app_ctx.mode));
    HMI_SetText("t_state5", "系统：运行中");
    HMI_SetText("t_set_fb", g_app_ctx.feedback);
    HMI_SetText("t_set_tip", ModeManager_GetModeName(g_app_ctx.mode));
}

void HMI_Page4_ShowSummary(void)
{
    HMI_GotoPage(4U);
    HMI_SetText("page4.t_alarm_msg", g_app_ctx.alarm_summary);
    HMI_SetText("page4.t_alarm_detail", "详情");
    HMI_SetText("page4.t_alarm_ok", "确认");
}

void HMI_Page4_ShowDetail(void)
{
    HMI_SetText("page4.t_alarm_msg", g_app_ctx.alarm_detail);
    HMI_SetText("page4.t_alarm_detail", "详情");
    HMI_SetText("page4.t_alarm_ok", "确认");
}

void HMI_Page4_ReturnToSafety(void)
{
    HMI_GotoPage(2U);
}
