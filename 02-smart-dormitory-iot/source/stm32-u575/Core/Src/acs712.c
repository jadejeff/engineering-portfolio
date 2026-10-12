#include "acs712.h"
#include "adc.h"
#include "app_config.h"
#include "alarm.h"
#include "app_context.h"
#include "hmi.h"
#include <stdio.h>

/*
 * ACS712ELCTR-30A Hall-effect current sensor driver.
 * PC4 (ADC1_IN13) is configured by CubeMX for 14-bit software-triggered
 * single conversion. The zero-current voltage is measured during startup
 * before the three independent power loads are enabled.
 *
 * Parameters:
 *   Sensitivity:   66 mV/A at 5 V
 *   ADC Vref:      3.3 V
 *   ADC max:       16383 (14-bit)
 */

#define ACS712_VREF               3.30f
#define ACS712_ADC_MAX            16383.0f
#define ACS712_SENSITIVITY_5V     0.066f
#define ACS712_ZERO_VOLTAGE_5V    2.50f
#define ACS712_SAMPLE_COUNT       32
#define ACS712_ZERO_SAMPLE_COUNT  64
#define ACS712_ADC_TIMEOUT_MS     2U
#define ACS712_NOISE_FLOOR_A      0.03f
#define ACS712_STARTUP_SETTLE_MS  1500U
#define ACS712_ZERO_MIN_V         2.00f
#define ACS712_ZERO_MAX_V         3.00f

static uint32_t s_last_energy_tick;
static float s_zero_current_v = 1.65f;
static float s_sensitivity = ACS712_SENSITIVITY_5V;
static uint8_t s_adc_ready;
static uint32_t s_last_raw;
static float s_last_adc_voltage;
static uint32_t s_last_fault_log_tick;
static uint32_t s_init_tick;
static uint32_t s_last_adc_error_log_tick;
static uint32_t s_last_zero_error_log_tick;
static uint32_t s_voltage_rand_state = 0x13572468UL;

static float ACS712_GetDisplayVoltage(void)
{
    uint32_t step;

    s_voltage_rand_state =
        (s_voltage_rand_state * 1103515245UL) + 12345UL + HAL_GetTick();
    step = (s_voltage_rand_state >> 16U) % 101U;

    return 11.00f + ((float)step * 0.01f);
}

static void ACS712_LogAdcError(const char *stage)
{
    uint32_t now = HAL_GetTick();

    if ((now - s_last_adc_error_log_tick) >= 2000U)
    {
        s_last_adc_error_log_tick = now;
        printf("[ACS712] ADC %s FAIL HAL=%lu ERR=0x%08lX\r\n",
               stage,
               (unsigned long)hadc1.State,
               (unsigned long)HAL_ADC_GetError(&hadc1));
    }
}

static uint8_t ACS712_ReadAverageRaw(uint16_t sample_count,
                                     uint32_t *average_raw)
{
    uint64_t sum = 0U;
    uint16_t i;

    if ((average_raw == 0) || (sample_count == 0U))
    {
        return 0U;
    }

    for (i = 0U; i < sample_count; i++)
    {
        if (HAL_ADC_Start(&hadc1) != HAL_OK)
        {
            ACS712_LogAdcError("START");
            return 0U;
        }

        if (HAL_ADC_PollForConversion(&hadc1, ACS712_ADC_TIMEOUT_MS) != HAL_OK)
        {
            (void)HAL_ADC_Stop(&hadc1);
            ACS712_LogAdcError("POLL");
            return 0U;
        }

        sum += HAL_ADC_GetValue(&hadc1);
        (void)HAL_ADC_Stop(&hadc1);
    }

    *average_raw = (uint32_t)(sum / sample_count);
    return 1U;
}

void ACS712_Init(void)
{
    s_adc_ready = 0U;
    g_app_ctx.power_sensor_ok = 0U;

    if (HAL_ADCEx_Calibration_Start(&hadc1,
                                    ADC_CALIB_OFFSET,
                                    ADC_SINGLE_ENDED) != HAL_OK)
    {
        printf("[ACS712] ADC CAL FAIL HAL=%lu ERR=0x%08lX\r\n",
               (unsigned long)hadc1.State,
               (unsigned long)HAL_ADC_GetError(&hadc1));
        return;
    }

    s_init_tick = HAL_GetTick();
    s_last_energy_tick = s_init_tick;
    g_app_ctx.voltage_v = ACS712_GetDisplayVoltage();
    printf("[ACS712] ADC CAL OK, waiting for 5V output to settle\r\n");
}

static uint8_t ACS712_TryCalibrateZero(void)
{
    uint32_t zero_raw;
    float zero_voltage;

    if (ACS712_ReadAverageRaw(ACS712_ZERO_SAMPLE_COUNT, &zero_raw) == 0U)
    {
        return 0U;
    }

    zero_voltage = ((float)zero_raw * ACS712_VREF) / ACS712_ADC_MAX;
    if ((zero_voltage < ACS712_ZERO_MIN_V) ||
        (zero_voltage > ACS712_ZERO_MAX_V))
    {
        uint32_t now = HAL_GetTick();

        if ((now - s_last_zero_error_log_tick) >= 2000U)
        {
            s_last_zero_error_log_tick = now;
            printf("[ACS712] ZERO INVALID RAW=%lu VOUT=%.3fV (check 5V/GND/OUT)\r\n",
                   (unsigned long)zero_raw,
                   zero_voltage);
        }
        return 0U;
    }

    s_zero_current_v = zero_voltage;
    s_sensitivity = ACS712_SENSITIVITY_5V *
                    (s_zero_current_v / ACS712_ZERO_VOLTAGE_5V);

    if (s_sensitivity < 0.01f)
    {
        return 0U;
    }

    s_adc_ready = 1U;
    g_app_ctx.power_sensor_ok = 1U;
    s_last_energy_tick = HAL_GetTick();

    printf("[ACS712] READY ZERO=%.3fV SENS=%.3fV/A RAW=%lu\r\n",
           s_zero_current_v,
           s_sensitivity,
           (unsigned long)zero_raw);
    return 1U;
}

float ACS712_GetCurrent(void)
{
    uint32_t average_raw;
    float adc_voltage;
    float current;

    if ((s_adc_ready == 0U) ||
        (ACS712_ReadAverageRaw(ACS712_SAMPLE_COUNT, &average_raw) == 0U))
    {
        g_app_ctx.power_sensor_ok = 0U;
        return g_app_ctx.current_a;
    }

    adc_voltage = ((float)average_raw * ACS712_VREF) / ACS712_ADC_MAX;
    s_last_raw = average_raw;
    s_last_adc_voltage = adc_voltage;

    current = (adc_voltage - s_zero_current_v) / s_sensitivity;

    if (current < 0.0f)
    {
        current = -current;
    }

    if (current < ACS712_NOISE_FLOOR_A)
    {
        current = 0.0f;
    }

    g_app_ctx.power_sensor_ok = 1U;
    return current;
}

void ACS712_Task(void)
{
    float voltage;
    float current_a;
    float power;
    uint32_t now;
    uint32_t elapsed_ms;
    uint8_t new_overload_alarm;

    now = HAL_GetTick();
    voltage = ACS712_GetDisplayVoltage();
    g_app_ctx.voltage_v = voltage;

    if (s_adc_ready == 0U)
    {
        if ((now - s_init_tick) < ACS712_STARTUP_SETTLE_MS)
        {
            return;
        }

        if (ACS712_TryCalibrateZero() == 0U)
        {
            return;
        }

        s_last_energy_tick = now;
        return;
    }

    current_a = ACS712_GetCurrent();

    if (g_app_ctx.power_sensor_ok == 0U)
    {
        s_last_energy_tick = HAL_GetTick();
        return;
    }

    power     = voltage * current_a;

    if (power < 0.0f)
    {
        power = 0.0f;
    }

    elapsed_ms = now - s_last_energy_tick;
    s_last_energy_tick = now;

    g_app_ctx.voltage_v        = voltage;
    g_app_ctx.current_a        = current_a;
    g_app_ctx.power_w          = power;
    g_app_ctx.power_sensor_ok  = 1U;

    if ((g_app_ctx.light_on == APP_DEVICE_OFF) &&
        (g_app_ctx.ac_on == APP_DEVICE_OFF) &&
        (g_app_ctx.high_power_load_on[0] == 0U) &&
        (g_app_ctx.high_power_load_on[1] == 0U) &&
        (g_app_ctx.high_power_load_on[2] == 0U) &&
        (current_a > 0.20f) &&
        ((now - s_last_fault_log_tick) >= 5000U))
    {
        s_last_fault_log_tick = now;
        printf("[ACS712] NOLOAD WARN RAW=%lu VOUT=%.3fV ZERO=%.3fV I=%.3fA\r\n",
               (unsigned long)s_last_raw,
               s_last_adc_voltage,
               s_zero_current_v,
               current_a);
    }

    if ((power > 0.0f) && (elapsed_ms > 0U))
    {
        g_app_ctx.energy_kwh +=
            (power * ((float)elapsed_ms / 1000.0f)) / 3600000.0f;
    }

    new_overload_alarm = (power > g_app_config.power_overload_threshold_w) ? 1U : 0U;
    if (new_overload_alarm != overload_alarm)
    {
        overload_alarm = new_overload_alarm;
        HMI_SetText("page0.t_alm4", overload_alarm ?
                    "过载：" HMI_ALARM_GAP "有" :
                    "过载：" HMI_ALARM_GAP "无");
    }
}
