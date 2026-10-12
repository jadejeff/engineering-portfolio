#ifndef __APP_CONTEXT_H
#define __APP_CONTEXT_H

#include "main.h"

typedef enum
{
    APP_DEVICE_OFF = 0,
    APP_DEVICE_ON = 1
} AppDeviceState;

typedef enum
{
    APP_DEVICE_LIGHT = 0x01,
    APP_DEVICE_SOCKET = 0x02,
    APP_DEVICE_AC = 0x03,
    APP_DEVICE_LOCK = 0x04
} AppDevice;

typedef enum
{
    APP_MODE_NORMAL = 0,
    APP_MODE_ENERGY = 1,
    APP_MODE_SECURITY = 2
} AppMode;

typedef enum
{
    APP_SAFETY_NORMAL = 0,
    APP_SAFETY_NOTICE = 1,
    APP_SAFETY_WARNING = 2,
    APP_SAFETY_CRITICAL = 3
} AppSafetyLevel;

typedef struct
{
    float voltage_v;
    float current_a;
    float power_w;
    float energy_kwh;
    float temperature_c;

    uint8_t ble_online_count;
    uint8_t phone_sync_ok;
    uint8_t human_present;
    uint8_t smoke_detected;
    uint8_t door_unlocked;
    uint8_t power_sensor_ok;
    uint8_t temp_sensor_ok;
    uint8_t light_bright;
    uint8_t buzzer_on;
    uint8_t high_power_load_on[3];

    AppDeviceState light_on;
    AppDeviceState socket_on;
    AppDeviceState ac_on;
    AppDeviceState lock_unlocked;

    AppMode mode;
    AppSafetyLevel safety_level;
    uint8_t alarm_active;
    uint8_t alarm_acknowledged;
    uint8_t alarm_page_shown;
    uint8_t alarm_detail_viewed;
    uint32_t device_action_count;

    char alarm_summary[96];
    char alarm_detail[192];
    char last_operation[96];
    char last_result[48];
    char feedback[128];
    char device_logs[4][128];
} AppContext;

extern AppContext g_app_ctx;

void AppContext_Init(void);

#endif
