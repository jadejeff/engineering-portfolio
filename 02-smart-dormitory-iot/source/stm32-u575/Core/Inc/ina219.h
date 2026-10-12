#ifndef __INA219_H
#define __INA219_H

#include "main.h"

void INA219_Init(void);

float INA219_GetBusVoltage(void);

float INA219_GetCurrent(void);
void INA219_Task(void);

#endif
