#include "mode_manager.h"
#include "hmi_pages.h"
#include <stdio.h>

static uint8_t s_manual_mode_selected;

const char *ModeManager_GetModeName(AppMode mode)
{
    switch (mode)
    {
        case APP_MODE_ENERGY:
            return "节能模式";

        case APP_MODE_SECURITY:
            return "安全模式";

        case APP_MODE_NORMAL:
            return "普通模式";

        default:
            return "未知模式";
    }
}

static uint8_t ModeManager_ApplyMode(AppMode mode, uint8_t manual)
{
    if ((mode != APP_MODE_NORMAL) &&
        (mode != APP_MODE_ENERGY) &&
        (mode != APP_MODE_SECURITY))
    {
        printf("[MODE] Unknown 0x%02X\r\n", (unsigned int)mode);
        return 0U;
    }

    if ((manual == 0U) && (s_manual_mode_selected != 0U))
    {
        return 0U;
    }

    if (manual != 0U)
    {
        s_manual_mode_selected = 1U;
    }

    if (g_app_ctx.mode == mode)
    {
        HMI_Page1_UpdateRunInfo();
        HMI_Page3_UpdateMode();
        return 1U;
    }

    g_app_ctx.mode = mode;

    snprintf(g_app_ctx.last_operation,
             sizeof(g_app_ctx.last_operation),
             "模式：%s",
             ModeManager_GetModeName(mode));
    snprintf(g_app_ctx.last_result,
             sizeof(g_app_ctx.last_result),
             "结果：成功");
    snprintf(g_app_ctx.feedback,
             sizeof(g_app_ctx.feedback),
             "反馈：切换为%s",
             ModeManager_GetModeName(mode));

    printf("[MODE] %s\r\n", ModeManager_GetModeName(mode));

    HMI_Page1_UpdateRunInfo();
    HMI_Page3_UpdateMode();

    return 1U;
}

uint8_t ModeManager_SetMode(AppMode mode)
{
    return ModeManager_ApplyMode(mode, 1U);
}

uint8_t ModeManager_SetModeAuto(AppMode mode)
{
    return ModeManager_ApplyMode(mode, 0U);
}
