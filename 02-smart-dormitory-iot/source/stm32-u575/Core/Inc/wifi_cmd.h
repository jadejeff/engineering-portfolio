#ifndef __WIFI_CMD_H
#define __WIFI_CMD_H

#include "main.h"

/* WiFi 2-byte command codes */
#define WIFI_CMD_LED      0xA0U
#define WIFI_CMD_AC       0xA1U
#define WIFI_CMD_SOCKET   0xA2U
#define WIFI_CMD_LOCK     0xA3U
#define WIFI_CMD_SCENE    0xB0U
#define WIFI_CMD_SYSMODE  0xB1U
#define WIFI_CMD_QUERY    0xC0U

void WiFiCmd_Init(void);
void WiFiCmd_Task(void);
void WiFiCmd_PushRxByteFromISR(uint8_t data);

#endif
