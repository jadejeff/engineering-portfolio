#include "relay.h"
#include "app_context.h"
#include "hmi_pages.h"
#include <stdio.h>

static void Relay_Write(GPIO_TypeDef *port, uint16_t pin, uint8_t on)
{
    HAL_GPIO_WritePin(port, pin, on ? LOAD_ON_STATE : LOAD_OFF_STATE);
}

static uint8_t Relay_Read(GPIO_TypeDef *port, uint16_t pin)
{
    return (HAL_GPIO_ReadPin(port, pin) == LOAD_ON_STATE) ? 1U : 0U;
}

static void OldAcOutputs_ForceOff(void)
{
    HAL_GPIO_WritePin(OLD_AC_FAN_PORT, OLD_AC_FAN_PIN, LOAD_OFF_STATE);
    HAL_GPIO_WritePin(OLD_AC_POWER_PORT, OLD_AC_POWER_PIN, LOAD_OFF_STATE);
}

void HighPowerLoads_ForceOff(void)
{
    HAL_GPIO_WritePin(PWR1_GPIO_Port, PWR1_Pin, LOAD_OFF_STATE);
    HAL_GPIO_WritePin(PWR2_GPIO_Port, PWR2_Pin, LOAD_OFF_STATE);
    HAL_GPIO_WritePin(PWR3_GPIO_Port, PWR3_Pin, LOAD_OFF_STATE);
    g_app_ctx.high_power_load_on[0] = 0U;
    g_app_ctx.high_power_load_on[1] = 0U;
    g_app_ctx.high_power_load_on[2] = 0U;
}

void Relay_Init(void)
{
    LightRelay_Set(0U);
    AirConditioner_Set(0U);
    OldAcOutputs_ForceOff();
    HighPowerLoads_ForceOff();

    g_app_ctx.light_on = APP_DEVICE_OFF;
    g_app_ctx.ac_on = APP_DEVICE_OFF;
    g_app_ctx.high_power_load_on[0] = HighPowerLoad_Get(1U);
    g_app_ctx.high_power_load_on[1] = HighPowerLoad_Get(2U);
    g_app_ctx.high_power_load_on[2] = HighPowerLoad_Get(3U);

    printf("[GPIO] INIT PB12_ODR=%u PB13_ODR=%u PB12_PIN=%u PB13_PIN=%u\r\n",
           (unsigned int)((GPIOB->ODR & AC_FAN_PIN) ? 1U : 0U),
           (unsigned int)((GPIOB->ODR & AC_POWER_PIN) ? 1U : 0U),
           (unsigned int)HAL_GPIO_ReadPin(AC_FAN_PORT, AC_FAN_PIN),
           (unsigned int)HAL_GPIO_ReadPin(AC_POWER_PORT, AC_POWER_PIN));
}

void LightRelay_Set(uint8_t on)
{
    HAL_GPIO_WritePin(LIGHT_PORT,
                      LIGHT_PIN,
                      on ? LIGHT_ON_STATE : LIGHT_OFF_STATE);
}

uint8_t LightRelay_Get(void)
{
    return (HAL_GPIO_ReadPin(LIGHT_PORT, LIGHT_PIN) == LIGHT_ON_STATE) ?
           1U : 0U;
}

void AirConditioner_Set(uint8_t on)
{
    Relay_Write(AC_FAN_PORT, AC_FAN_PIN, on);
    Relay_Write(AC_POWER_PORT, AC_POWER_PIN, on);
}

uint8_t AirConditioner_Get(void)
{
    return (Relay_Read(AC_FAN_PORT, AC_FAN_PIN) &&
            Relay_Read(AC_POWER_PORT, AC_POWER_PIN)) ? 1U : 0U;
}

uint8_t HighPowerLoad_Get(uint8_t index)
{
    switch (index)
    {
        case 1U:
            return (HAL_GPIO_ReadPin(PWR1_GPIO_Port, PWR1_Pin) ==
                    LOAD_ON_STATE) ? 1U : 0U;

        case 2U:
            return (HAL_GPIO_ReadPin(PWR2_GPIO_Port, PWR2_Pin) ==
                    LOAD_ON_STATE) ? 1U : 0U;

        case 3U:
            return (HAL_GPIO_ReadPin(PWR3_GPIO_Port, PWR3_Pin) ==
                    LOAD_ON_STATE) ? 1U : 0U;

        default:
            return 0U;
    }
}

uint8_t HighPowerLoad_Set(uint8_t index, uint8_t on)
{
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t actual_on;

    if ((index < 1U) || (index > 3U))
    {
        return 0U;
    }

    if ((on != 0U) && (g_app_ctx.socket_on != APP_DEVICE_ON))
    {
        snprintf(g_app_ctx.last_operation,
                 sizeof(g_app_ctx.last_operation),
                 "操作：负载%u 开启",
                 (unsigned int)index);
        snprintf(g_app_ctx.last_result,
                 sizeof(g_app_ctx.last_result),
                 "结果：已拒绝");
        snprintf(g_app_ctx.feedback,
                 sizeof(g_app_ctx.feedback),
                 "反馈：插座关闭，负载已禁用");
        HMI_Page1_UpdateRunInfo();
        HMI_Page0_UpdateOverview();
        return 0U;
    }

    switch (index)
    {
        case 1U:
            port = PWR1_GPIO_Port;
            pin = PWR1_Pin;
            break;

        case 2U:
            port = PWR2_GPIO_Port;
            pin = PWR2_Pin;
            break;

        default:
            port = PWR3_GPIO_Port;
            pin = PWR3_Pin;
            break;
    }

    Relay_Write(port, pin, on);
    actual_on = HighPowerLoad_Get(index);
    g_app_ctx.high_power_load_on[index - 1U] = actual_on;
    g_app_ctx.device_action_count++;

    snprintf(g_app_ctx.last_operation,
             sizeof(g_app_ctx.last_operation),
             "操作：负载%u %s",
             (unsigned int)index,
             actual_on ? "开启" : "关闭");
    snprintf(g_app_ctx.last_result,
             sizeof(g_app_ctx.last_result),
             "结果：%s",
             (actual_on == (on ? 1U : 0U)) ? "成功" : "失败");
    snprintf(g_app_ctx.feedback,
             sizeof(g_app_ctx.feedback),
             "反馈：负载%u %s",
             (unsigned int)index,
             actual_on ? "开启" : "关闭");

    printf("[BUTTON] Load%u %s\r\n",
           (unsigned int)index,
           actual_on ? "ON" : "OFF");

    HMI_Page1_UpdateRunInfo();
    HMI_Page0_UpdateOverview();

    return (actual_on == (on ? 1U : 0U)) ? 1U : 0U;
}
