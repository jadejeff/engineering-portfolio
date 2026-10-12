#ifndef __HMI_PROTOCOL_H
#define __HMI_PROTOCOL_H

#include "main.h"

void HMI_Protocol_Init(void);
void HMI_Protocol_Task(void);
uint8_t HMI_Protocol_GetCurrentPage(void);

#endif
