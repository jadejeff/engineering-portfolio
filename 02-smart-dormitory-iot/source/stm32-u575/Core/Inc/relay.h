#ifndef __RELAY_H
#define __RELAY_H

#include "main.h"

#define LIGHT_PIN          light_Pin
#define LIGHT_PORT         light_GPIO_Port
#define AC_FAN_PIN         RB12_Pin
#define AC_FAN_PORT        RB12_GPIO_Port
#define AC_POWER_PIN       RB13_Pin
#define AC_POWER_PORT      RB13_GPIO_Port
#define OLD_AC_FAN_PIN     air_Pin
#define OLD_AC_FAN_PORT    air_GPIO_Port
#define OLD_AC_POWER_PIN   R_Pin
#define OLD_AC_POWER_PORT  R_GPIO_Port

/* PB3 light uses the measured opposite polarity. */
#define LIGHT_ON_STATE     GPIO_PIN_SET
#define LIGHT_OFF_STATE    GPIO_PIN_RESET

/* Measured relay input behavior: high enables the load, low disables it. */
#define LOAD_ON_STATE      GPIO_PIN_SET
#define LOAD_OFF_STATE     GPIO_PIN_RESET

#define LIGHT_ON()  HAL_GPIO_WritePin(LIGHT_PORT, LIGHT_PIN, LIGHT_ON_STATE)
#define LIGHT_OFF() HAL_GPIO_WritePin(LIGHT_PORT, LIGHT_PIN, LIGHT_OFF_STATE)

void Relay_Init(void);

void LightRelay_Set(uint8_t on);
uint8_t LightRelay_Get(void);

void AirConditioner_Set(uint8_t on);
uint8_t AirConditioner_Get(void);

uint8_t HighPowerLoad_Set(uint8_t index, uint8_t on);
uint8_t HighPowerLoad_Get(uint8_t index);
void HighPowerLoads_ForceOff(void);

#endif
