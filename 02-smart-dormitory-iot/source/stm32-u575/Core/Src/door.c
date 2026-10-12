#include "door.h"
#include "alarm.h"
#include "app_context.h"
#include "hmi.h"
#include <stdio.h>

void Door_Task(void)
{
    static GPIO_PinState last_pin_state = (GPIO_PinState)0xFF;
    GPIO_PinState pin_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
    uint8_t state_changed = (pin_state != last_pin_state) ? 1U : 0U;

    if (state_changed != 0U)
    {
        last_pin_state = pin_state;
        printf("[DOOR] PB2=%u %s\r\n",
               (unsigned int)pin_state,
               (pin_state == GPIO_PIN_RESET) ? "LOCK" : "UNLOCK");
    }

    if(pin_state == GPIO_PIN_RESET)
    {
        door_alarm = 0U;
        g_app_ctx.door_unlocked = 0U;
        g_app_ctx.lock_unlocked = APP_DEVICE_OFF;
    }
    else
    {
        door_alarm = 1U;
        g_app_ctx.door_unlocked = 1U;
        g_app_ctx.lock_unlocked = APP_DEVICE_ON;
    }

    if (state_changed != 0U)
    {
        HMI_SetText("page2.t_lock", door_alarm ? "未锁" : "已锁");
        HMI_SetText("page0.t_alm3", door_alarm ?
                    "门锁：" HMI_ALARM_GAP "未锁" :
                    "门锁：" HMI_ALARM_GAP "已锁");
    }
}
