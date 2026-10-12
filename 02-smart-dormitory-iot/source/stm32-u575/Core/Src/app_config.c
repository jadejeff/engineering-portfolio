#include "app_config.h"
#include "app_context.h"
#include "hmi_pages.h"
#include <stdio.h>

AppConfig g_app_config;

static void AppConfig_Report(const char *name, const char *value_text)
{
    snprintf(g_app_ctx.last_operation,
             sizeof(g_app_ctx.last_operation),
             "设置：%s",
             name);

    snprintf(g_app_ctx.last_result,
             sizeof(g_app_ctx.last_result),
             "结果：成功");

    snprintf(g_app_ctx.feedback,
             sizeof(g_app_ctx.feedback),
             "反馈：%s %s",
             name,
             value_text);

    printf("[CFG] %s %s\r\n", name, value_text);

    HMI_Page1_UpdateRunInfo();
    HMI_Page3_UpdateMode();
}

static void AppConfig_ReportInvalid(uint8_t param, uint8_t value)
{
    snprintf(g_app_ctx.last_operation,
             sizeof(g_app_ctx.last_operation),
             "设置项：0x%02X",
             param);

    snprintf(g_app_ctx.last_result,
             sizeof(g_app_ctx.last_result),
             "结果：已拒绝");

    snprintf(g_app_ctx.feedback,
             sizeof(g_app_ctx.feedback),
             "反馈：参数无效");

    printf("[CFG] Invalid param=0x%02X value=%u\r\n",
           (unsigned int)param,
           (unsigned int)value);

    HMI_Page1_UpdateRunInfo();
    HMI_Page3_UpdateMode();
}

void AppConfig_Init(void)
{
    g_app_config.temp_alarm_threshold_c = 40.0f;
    g_app_config.power_overload_threshold_w = 35.0f;
    g_app_config.energy_auto_off_enable = 1U;
    g_app_config.security_auto_cut_socket_enable = 1U;
}

uint8_t AppConfig_SetFromHmi(uint8_t param, uint8_t value)
{
    char value_text[20];

    switch ((AppConfigParam)param)
    {
        case APP_CONFIG_TEMP_THRESHOLD:
            if ((value < 10U) || (value > 60U))
            {
                AppConfig_ReportInvalid(param, value);
                return 0U;
            }

            g_app_config.temp_alarm_threshold_c = (float)value;
            snprintf(value_text, sizeof(value_text), "%uC", (unsigned int)value);
            AppConfig_Report("火灾温度阈值", value_text);
            return 1U;

        case APP_CONFIG_POWER_THRESHOLD:
            if ((value < 1U) || (value > 50U))
            {
                AppConfig_ReportInvalid(param, value);
                return 0U;
            }

            g_app_config.power_overload_threshold_w = (float)value * 100.0f;
            snprintf(value_text, sizeof(value_text), "%u00W", (unsigned int)value);
            AppConfig_Report("功率阈值", value_text);
            return 1U;

        case APP_CONFIG_ENERGY_AUTO_OFF:
            g_app_config.energy_auto_off_enable = (value != 0U) ? 1U : 0U;
            AppConfig_Report("节能自动关闭", g_app_config.energy_auto_off_enable ? "开启" : "关闭");
            return 1U;

        case APP_CONFIG_SECURITY_AUTO_CUT_SOCKET:
            g_app_config.security_auto_cut_socket_enable = (value != 0U) ? 1U : 0U;
            AppConfig_Report("安全自动断电", g_app_config.security_auto_cut_socket_enable ? "开启" : "关闭");
            return 1U;

        default:
            AppConfig_ReportInvalid(param, value);
            return 0U;
    }
}
