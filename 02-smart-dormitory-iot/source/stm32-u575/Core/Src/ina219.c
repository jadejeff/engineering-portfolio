#include "ina219.h"
#include "i2c.h"

#define INA219_ADDR (0x40 << 1)

#define REG_CONFIG       0x00
#define REG_BUSVOLTAGE   0x02
#define REG_CURRENT      0x04
#define REG_CALIBRATION  0x05

static void INA219_WriteReg(uint8_t reg,uint16_t data)
{
    uint8_t buf[2];

    buf[0] = data >> 8;
    buf[1] = data & 0xFF;

    HAL_I2C_Mem_Write(
        &hi2c1,
        INA219_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        buf,
        2,
        100);
}

static uint16_t INA219_ReadReg(uint8_t reg)
{
    uint8_t buf[2];

    HAL_I2C_Mem_Read(
        &hi2c1,
        INA219_ADDR,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        buf,
        2,
        100);

    return ((uint16_t)buf[0] << 8) | buf[1];
}

void INA219_Init(void)
{
    INA219_WriteReg(REG_CONFIG,0x399F);

    INA219_WriteReg(REG_CALIBRATION,4096);
}

float INA219_GetBusVoltage(void)
{
    uint16_t raw;

    raw = INA219_ReadReg(REG_BUSVOLTAGE);

    raw >>= 3;

    return raw * 0.004f;
}

float INA219_GetCurrent(void)
{
    int16_t raw;

    raw = (int16_t)INA219_ReadReg(REG_CURRENT);

    return raw * 0.1f;
}