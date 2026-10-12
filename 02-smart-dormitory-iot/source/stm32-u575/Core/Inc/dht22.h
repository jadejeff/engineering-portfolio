#ifndef DHT22_H
#define DHT22_H

#include "stm32u5xx_hal.h"

#define DHT22_PIN   GPIO_PIN_1
#define DHT22_PORT  GPIOB

uint8_t DHT22_Read(float *hum, float *temp);
void DHT22_Task(void);

#endif
