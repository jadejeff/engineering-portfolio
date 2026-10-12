#include "alarm.h"
#include "app_context.h"
#include "hmi.h"
#include "hmi_pages.h"
#include "app_config.h"
#include <string.h>

uint8_t smoke_alarm = 0U;
uint8_t fire_alarm = 0U;
uint8_t door_alarm = 0U;
uint8_t overload_alarm = 0U;

#define BUZZER_ON_STATE   GPIO_PIN_RESET
#define BUZZER_OFF_STATE  GPIO_PIN_SET

static void Alarm_SetBuzzer(uint8_t on)
{
    HAL_GPIO_WritePin(buzzer_GPIO_Port,
                      buzzer_Pin,
                      on ? BUZZER_ON_STATE : BUZZER_OFF_STATE);
    g_app_ctx.buzzer_on = on ? 1U : 0U;
}

void Alarm_Init(void)
{
    Alarm_SetBuzzer(0U);
}

static const char *Alarm_LevelText(AppSafetyLevel level)
{
    switch (level)
    {
        case APP_SAFETY_CRITICAL:
            return "等级：严重";

        case APP_SAFETY_WARNING:
            return "等级：警告";

        case APP_SAFETY_NOTICE:
            return "等级：注意";

        case APP_SAFETY_NORMAL:
        default:
            return "等级：正常";
    }
}

static void Alarm_CopyText(char *dst, uint32_t dst_size, const char *src)
{
    if (dst_size == 0U)
    {
        return;
    }

    strncpy(dst, src, dst_size - 1U);
    dst[dst_size - 1U] = '\0';
}

static void Alarm_SetContext(AppSafetyLevel level,
                             const char *summary,
                             const char *detail)
{
    uint8_t alarm_changed;

    alarm_changed = (g_app_ctx.safety_level != level) ||
                    (strncmp(g_app_ctx.alarm_summary,
                             summary,
                             sizeof(g_app_ctx.alarm_summary)) != 0);

    g_app_ctx.safety_level = level;
    g_app_ctx.alarm_active = (level != APP_SAFETY_NORMAL) ? 1U : 0U;

    Alarm_CopyText(g_app_ctx.alarm_summary,
                   sizeof(g_app_ctx.alarm_summary),
                   summary);
    Alarm_CopyText(g_app_ctx.alarm_detail,
                   sizeof(g_app_ctx.alarm_detail),
                   detail);

    if (g_app_ctx.alarm_active == 0U)
    {
        Alarm_SetBuzzer(0U);
        g_app_ctx.alarm_acknowledged = 0U;
        g_app_ctx.alarm_page_shown = 0U;
        g_app_ctx.alarm_detail_viewed = 0U;
    }
    else if (alarm_changed)
    {
        g_app_ctx.alarm_acknowledged = 0U;
        g_app_ctx.alarm_page_shown = 0U;
        g_app_ctx.alarm_detail_viewed = 0U;
    }
}

static void Alarm_UpdateSafetyText(void)
{
    HMI_SetText("t_level", Alarm_LevelText(g_app_ctx.safety_level));

    if (fire_alarm)
    {
        HMI_SetText("t_reason", "原因：检测到火灾");
        HMI_SetText("t_suggest", "建议：立即断电撤离");
    }
    else if (smoke_alarm)
    {
        HMI_SetText("t_reason", "原因：烟雾异常");
        HMI_SetText("t_suggest", "建议：立即检查");
    }
    else if (overload_alarm)
    {
        HMI_SetText("t_reason", "原因：电流过载");
        HMI_SetText("t_suggest", "建议：检查负载");
    }
    else if (door_alarm)
    {
        HMI_SetText("t_reason", "原因：门锁未锁");
        HMI_SetText("t_suggest", "建议：及时锁门");
    }
    else
    {
        HMI_SetText("t_reason", "原因：无");
        HMI_SetText("t_suggest", "建议：无");
    }
}

static uint8_t Alarm_ShouldPopup(void)
{
    if ((g_app_ctx.alarm_active == 0U) ||
        (g_app_ctx.alarm_acknowledged != 0U) ||
        (g_app_ctx.alarm_page_shown != 0U))
    {
        return 0U;
    }

    if (g_app_ctx.safety_level == APP_SAFETY_CRITICAL)
    {
        return 1U;
    }

    if ((g_app_ctx.safety_level == APP_SAFETY_WARNING) &&
        (g_app_ctx.mode == APP_MODE_SECURITY))
    {
        return 1U;
    }

    return 0U;
}

void Alarm_Task(void)
{
    fire_alarm = ((g_app_ctx.temp_sensor_ok != 0U) &&
                  (smoke_alarm != 0U) &&
                  (g_app_ctx.temperature_c >=
                   g_app_config.temp_alarm_threshold_c)) ? 1U : 0U;

    if (fire_alarm)
    {
        Alarm_SetContext(APP_SAFETY_CRITICAL,
                         "火灾报警",
                         "烟雾与温度同时超过阈值，请立即断电并安全撤离。");
    }
    else if (smoke_alarm)
    {
        Alarm_SetContext(APP_SAFETY_CRITICAL,
                         "烟雾报警",
                         "检测到烟雾异常，请立即检查空气、电源及房间安全。");
    }
    else if (overload_alarm)
    {
        Alarm_SetContext(APP_SAFETY_CRITICAL,
                         "过载报警",
                         "检测到功率过载，请立即检查插座和用电负载。");
    }
    else if (door_alarm)
    {
        Alarm_SetContext(APP_SAFETY_WARNING,
                         "门锁未锁",
                         "当前门锁未锁，房间无人时请及时锁门。");
    }
    else
    {
        Alarm_SetContext(APP_SAFETY_NORMAL,
                         "系统正常",
                         "当前无报警");
    }

    Alarm_UpdateSafetyText();

    if (Alarm_ShouldPopup())
    {
        Alarm_SetBuzzer(1U);
        HMI_Page4_ShowSummary();
        g_app_ctx.alarm_page_shown = 1U;
    }
}

void Alarm_ShowDetail(void)
{
    if (g_app_ctx.alarm_active != 0U)
    {
        g_app_ctx.alarm_detail_viewed = 1U;
        HMI_Page4_ShowDetail();
    }
}

void Alarm_Confirm(void)
{
    Alarm_SetBuzzer(0U);

    if (g_app_ctx.alarm_active != 0U)
    {
        g_app_ctx.alarm_acknowledged = 1U;
        g_app_ctx.alarm_page_shown = 0U;
        g_app_ctx.alarm_detail_viewed = 0U;
    }

    HMI_Page4_ReturnToSafety();
    HMI_Page2_UpdateSafety();
}
