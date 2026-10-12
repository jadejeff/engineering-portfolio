#ifndef __ACS712_H
#define __ACS712_H

#include "main.h"

#define ACS712_VOLTAGE_FIXED  11.56f

void ACS712_Init(void);
float ACS712_GetCurrent(void);
void ACS712_Task(void);

#endif
