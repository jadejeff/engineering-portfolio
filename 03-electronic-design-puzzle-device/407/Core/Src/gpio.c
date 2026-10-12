/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   GPIO configuration
  ******************************************************************************
  */
/* USER CODE END Header */

#include "gpio.h"

void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO?? */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*
     * PE7  = X_STEP
     * PE8  = X_DIR
     * PE9  = YL_STEP
     * PE10 = YL_DIR
     * PE11 = YR_STEP
     * PE12 = YR_DIR
     * PE13 = Z_STEP
     * PE14 = Z_DIR
     */
    HAL_GPIO_WritePin(
        GPIOE,
        GPIO_PIN_7  |
        GPIO_PIN_8  |
        GPIO_PIN_9  |
        GPIO_PIN_10 |
        GPIO_PIN_11 |
        GPIO_PIN_12 |
        GPIO_PIN_13 |
        GPIO_PIN_14,
        GPIO_PIN_RESET
    );

    GPIO_InitStruct.Pin =
        GPIO_PIN_7  |
        GPIO_PIN_8  |
        GPIO_PIN_9  |
        GPIO_PIN_10 |
        GPIO_PIN_11 |
        GPIO_PIN_12 |
        GPIO_PIN_13 |
        GPIO_PIN_14;

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*
     * PD12~PD15:ULN2003??????
     */
    HAL_GPIO_WritePin(
        GPIOD,
        GPIO_PIN_12 |
        GPIO_PIN_13 |
        GPIO_PIN_14 |
        GPIO_PIN_15,
        GPIO_PIN_RESET
    );

    GPIO_InitStruct.Pin =
        GPIO_PIN_12 |
        GPIO_PIN_13 |
        GPIO_PIN_14 |
        GPIO_PIN_15;

    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*
     * PB0:???MOS??
     */
    HAL_GPIO_WritePin(
        GPIOB,
        GPIO_PIN_0,
        GPIO_PIN_RESET
    );

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}