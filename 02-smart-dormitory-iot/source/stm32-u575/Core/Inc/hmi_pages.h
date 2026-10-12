#ifndef __HMI_PAGES_H
#define __HMI_PAGES_H

#include "main.h"
#include "app_context.h"

void HMI_Page0_UpdateOverview(void);
void HMI_Page0_UpdatePhoneStatus(void);
void HMI_Page0_PowerTrendSample(uint8_t page_visible);
void HMI_Page0_BeginPowerTrendRedraw(void);
void HMI_Page1_UpdateLight(void);
void HMI_Page1_UpdateDevice(AppDevice device);
void HMI_Page1_UpdateRunInfo(void);
void HMI_Page1_UpdateAllDevices(void);
void HMI_Page2_UpdateSafety(void);
void HMI_Page3_UpdateMode(void);
void HMI_Page4_ShowSummary(void);
void HMI_Page4_ShowDetail(void);
void HMI_Page4_ReturnToSafety(void);

#endif
