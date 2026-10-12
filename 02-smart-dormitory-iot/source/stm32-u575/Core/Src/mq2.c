#include "mq2.h"
#include "alarm.h"
#include "app_context.h"
#include "hmi.h"

static uint8_t mq2_state;
static uint8_t s_last_mq2_state = 0xFFU;

void MQ2_Task(void)
{
    mq2_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);

    if(mq2_state == GPIO_PIN_RESET)
    {
        smoke_alarm = 1U;
        g_app_ctx.smoke_detected = 1U;
    }
    else
    {
        smoke_alarm = 0U;
        g_app_ctx.smoke_detected = 0U;
    }

    if (mq2_state != s_last_mq2_state)
    {
        s_last_mq2_state = mq2_state;
        HMI_SetText("page2.t_smoke", smoke_alarm ? "异常" : "正常");
        HMI_SetText("page0.t_alm1", smoke_alarm ?
                    "烟雾：" HMI_ALARM_GAP "异常" :
                    "烟雾：" HMI_ALARM_GAP "正常");
    }
}
