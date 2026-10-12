#ifndef __DEVICE_MANAGER_H
#define __DEVICE_MANAGER_H

#include "app_context.h"

typedef enum
{
    DEVICE_CTRL_SRC_HMI = 0,
    DEVICE_CTRL_SRC_AUTO = 1,
    DEVICE_CTRL_SRC_WIFI = 2
} DeviceControlSource;

uint8_t DeviceManager_SetDevice(AppDevice device,
                                AppDeviceState state,
                                DeviceControlSource source);

AppDeviceState DeviceManager_GetDeviceState(AppDevice device);
const char *DeviceManager_GetDeviceName(AppDevice device);
const char *DeviceManager_GetStateText(AppDevice device, AppDeviceState state);

#endif
