#include "sensor_sim.h"
#include "app_context.h"

static uint32_t s_ble_tick;
static uint8_t s_auto_ble_enabled;

void SensorSim_SetBleOnlineCount(uint8_t count)
{
    if (count > 9U)
    {
        count = 9U;
    }

    g_app_ctx.ble_online_count = count;
    g_app_ctx.human_present = (count > 0U) ? 1U : 0U;
}

void SensorSim_Init(void)
{
    s_ble_tick = HAL_GetTick();
    s_auto_ble_enabled = 0U;
    SensorSim_SetBleOnlineCount(0U);
}

void SensorSim_Task(void)
{
    uint32_t now = HAL_GetTick();

    if (s_auto_ble_enabled == 0U)
    {
        return;
    }

    if ((now - s_ble_tick) >= 15000U)
    {
        s_ble_tick = now;

        if (g_app_ctx.ble_online_count == 0U)
        {
            SensorSim_SetBleOnlineCount(1U);
        }
        else
        {
            SensorSim_SetBleOnlineCount(0U);
        }
    }
}
