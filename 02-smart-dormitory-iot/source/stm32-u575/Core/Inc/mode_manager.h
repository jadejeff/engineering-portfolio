#ifndef __MODE_MANAGER_H
#define __MODE_MANAGER_H

#include "app_context.h"

uint8_t ModeManager_SetMode(AppMode mode);
uint8_t ModeManager_SetModeAuto(AppMode mode);
const char *ModeManager_GetModeName(AppMode mode);

#endif
