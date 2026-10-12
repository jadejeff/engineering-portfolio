#ifndef __ALARM_H
#define __ALARM_H

#include "main.h"

extern uint8_t smoke_alarm;
extern uint8_t fire_alarm;
extern uint8_t door_alarm;
extern uint8_t overload_alarm;

void Alarm_Init(void);
void Alarm_Task(void);
void Alarm_ShowDetail(void);
void Alarm_Confirm(void);

#endif
