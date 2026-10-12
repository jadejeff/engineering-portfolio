#include "data_output.h"
#include "app_context.h"
#include "usart.h"
#include <stdio.h>

void DataOutput_SendSample(void)
{
    char line[256];
    int length;

    length = snprintf(line,
                      sizeof(line),
                      "DATA,%lu,%.4f,%.2f\r\n",
                      (unsigned long)HAL_GetTick(),
                      g_app_ctx.current_a,
                      g_app_ctx.power_w);

    if ((length > 0) && ((size_t)length < sizeof(line)))
    {
        (void)HAL_UART_Transmit(&huart1,
                               (uint8_t *)line,
                               (uint16_t)length,
                               20U);
    }
}
