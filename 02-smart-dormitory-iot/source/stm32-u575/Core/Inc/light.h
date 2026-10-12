#ifndef __LIGHT_H
#define __LIGHT_H

#include "main.h"

/* Final hardware assignment: digital light sensor DO -> PC9. */
#define LIGHT_SENSOR_BRIGHT_STATE  GPIO_PIN_RESET

void Light_Task(void);
uint8_t Light_IsBright(void);

#endif
