#include "logic_iot.h"
#include "alarm.h"
#include "app_config.h"
#include "app_context.h"
#include "device_manager.h"
#include "light.h"
#include "mode_manager.h"

static uint8_t s_presence_initialized;
static uint8_t s_last_human_present;

static void LogicIot_PresenceModeTask(void)
{
    uint8_t human_present = (g_app_ctx.human_present != 0U) ? 1U : 0U;

    if (s_presence_initialized == 0U)
    {
        s_presence_initialized = 1U;
        s_last_human_present = human_present;
        (void)ModeManager_SetModeAuto(human_present ? APP_MODE_NORMAL :
                                                     APP_MODE_SECURITY);
        return;
    }
    else if (human_present == s_last_human_present)
    {
        return;
    }

    s_last_human_present = human_present;
    (void)ModeManager_SetModeAuto(human_present ? APP_MODE_NORMAL :
                                                 APP_MODE_SECURITY);
}

static void LogicIot_EnergyModeTask(void)
{
    if (g_app_ctx.mode != APP_MODE_ENERGY)
    {
        return;
    }

    if (g_app_config.energy_auto_off_enable == 0U)
    {
        return;
    }

    if ((g_app_ctx.human_present != 0U) &&
        (Light_IsBright() != 0U) &&
        (g_app_ctx.light_on == APP_DEVICE_ON))
    {
        (void)DeviceManager_SetDevice(APP_DEVICE_LIGHT,
                                      APP_DEVICE_OFF,
                                      DEVICE_CTRL_SRC_AUTO);
    }
}

static void LogicIot_SecurityModeTask(void)
{
    if (g_app_ctx.mode != APP_MODE_SECURITY)
    {
        return;
    }

    if (g_app_config.security_auto_cut_socket_enable == 0U)
    {
        return;
    }

    if (smoke_alarm &&
        (g_app_ctx.socket_on == APP_DEVICE_ON))
    {
        (void)DeviceManager_SetDevice(APP_DEVICE_SOCKET,
                                      APP_DEVICE_OFF,
                                      DEVICE_CTRL_SRC_AUTO);
    }
}

void LogicIot_Task(void)
{
    LogicIot_PresenceModeTask();
    LogicIot_EnergyModeTask();
    LogicIot_SecurityModeTask();
}
