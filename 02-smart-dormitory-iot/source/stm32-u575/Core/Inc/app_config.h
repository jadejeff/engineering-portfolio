#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#include "main.h"

typedef enum
{
    APP_CONFIG_TEMP_THRESHOLD = 0x01,
    APP_CONFIG_POWER_THRESHOLD = 0x02,
    APP_CONFIG_ENERGY_AUTO_OFF = 0x03,
    APP_CONFIG_SECURITY_AUTO_CUT_SOCKET = 0x04
} AppConfigParam;

typedef struct
{
    float temp_alarm_threshold_c;
    float power_overload_threshold_w;
    uint8_t energy_auto_off_enable;
    uint8_t security_auto_cut_socket_enable;
} AppConfig;

extern AppConfig g_app_config;

void AppConfig_Init(void);
uint8_t AppConfig_SetFromHmi(uint8_t param, uint8_t value);

#endif
