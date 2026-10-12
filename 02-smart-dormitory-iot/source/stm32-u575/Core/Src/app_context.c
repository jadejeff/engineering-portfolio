#include "app_context.h"
#include <string.h>

AppContext g_app_ctx;

void AppContext_Init(void)
{
    g_app_ctx.voltage_v = 0.0f;
    g_app_ctx.current_a = 0.0f;
    g_app_ctx.power_w = 0.0f;
    g_app_ctx.energy_kwh = 0.0f;
    g_app_ctx.temperature_c = 26.0f;

    g_app_ctx.ble_online_count = 0U;
    g_app_ctx.phone_sync_ok = 0U;
    g_app_ctx.human_present = 0U;
    g_app_ctx.smoke_detected = 0U;
    g_app_ctx.door_unlocked = 0U;
    g_app_ctx.power_sensor_ok = 0U;
    g_app_ctx.temp_sensor_ok = 0U;
    g_app_ctx.light_bright = 0U;
    g_app_ctx.buzzer_on = 0U;
    g_app_ctx.high_power_load_on[0] = 0U;
    g_app_ctx.high_power_load_on[1] = 0U;
    g_app_ctx.high_power_load_on[2] = 0U;

    g_app_ctx.light_on = APP_DEVICE_OFF;
    g_app_ctx.socket_on = APP_DEVICE_OFF;
    g_app_ctx.ac_on = APP_DEVICE_OFF;
    g_app_ctx.lock_unlocked = APP_DEVICE_OFF;

    g_app_ctx.mode = APP_MODE_NORMAL;
    g_app_ctx.safety_level = APP_SAFETY_NORMAL;
    g_app_ctx.alarm_active = 0U;
    g_app_ctx.alarm_acknowledged = 0U;
    g_app_ctx.alarm_page_shown = 0U;
    g_app_ctx.alarm_detail_viewed = 0U;
    g_app_ctx.device_action_count = 0U;

    strcpy(g_app_ctx.alarm_summary, "系统正常");
    strcpy(g_app_ctx.alarm_detail, "当前无报警");
    strcpy(g_app_ctx.last_operation, "操作：无");
    strcpy(g_app_ctx.last_result, "结果：就绪");
    strcpy(g_app_ctx.feedback, "反馈：系统就绪");
    strcpy(g_app_ctx.device_logs[0], "[--] 系统已就绪");
    strcpy(g_app_ctx.device_logs[1], "[--] 等待操作");
    strcpy(g_app_ctx.device_logs[2], "");
    strcpy(g_app_ctx.device_logs[3], "");
}
