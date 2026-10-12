#ifndef __SENSOR_SIM_H
#define __SENSOR_SIM_H

#include "main.h"

void SensorSim_Init(void);
void SensorSim_Task(void);
void SensorSim_SetBleOnlineCount(uint8_t count);

#endif
