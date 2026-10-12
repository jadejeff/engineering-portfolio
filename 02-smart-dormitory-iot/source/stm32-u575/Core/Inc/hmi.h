#ifndef __HMI_H
#define __HMI_H

#include "main.h"

/* Page0 alarm rows reserve 18 spaces between the label and its state. */
#define HMI_ALARM_GAP  "                  "

void HMI_SendCmd(const char *cmd);
void HMI_Task(void);
void HMI_SetText(const char *obj, const char *text);
void HMI_SetPic(const char *obj, uint16_t pic_id);
void HMI_GotoPage(uint8_t page);

#endif
