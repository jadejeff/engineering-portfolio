#include "hmi.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

#define HMI_TX_QUEUE_SIZE  64U
#define HMI_TX_FRAME_MAX   260U

typedef struct
{
    uint16_t length;
    uint8_t data[HMI_TX_FRAME_MAX];
} HmiTxFrame;

static HmiTxFrame s_tx_queue[HMI_TX_QUEUE_SIZE];
static volatile uint8_t s_tx_head;
static volatile uint8_t s_tx_tail;
static volatile uint8_t s_tx_count;
static volatile uint8_t s_tx_busy;
static uint32_t s_tx_start_tick;

static void HMI_TryStartTx(void)
{
    HmiTxFrame *frame;

    if ((s_tx_busy != 0U) || (s_tx_count == 0U))
    {
        return;
    }

    frame = &s_tx_queue[s_tx_tail];
    s_tx_busy = 1U;
    s_tx_start_tick = HAL_GetTick();
    if (HAL_UART_Transmit_IT(&huart3, frame->data, frame->length) != HAL_OK)
    {
        s_tx_busy = 0U;
    }
}

void HMI_SendCmd(const char *cmd)
{
    HmiTxFrame *frame;
    uint32_t primask;
    size_t length;

    if (cmd == NULL)
    {
        return;
    }

    length = strlen(cmd);
    if (length > (HMI_TX_FRAME_MAX - 3U))
    {
        length = HMI_TX_FRAME_MAX - 3U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (s_tx_count >= HMI_TX_QUEUE_SIZE)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return;
    }

    frame = &s_tx_queue[s_tx_head];
    memcpy(frame->data, cmd, length);
    frame->data[length] = 0xFFU;
    frame->data[length + 1U] = 0xFFU;
    frame->data[length + 2U] = 0xFFU;
    frame->length = (uint16_t)(length + 3U);
    s_tx_head = (uint8_t)((s_tx_head + 1U) % HMI_TX_QUEUE_SIZE);
    s_tx_count++;
    if (primask == 0U)
    {
        __enable_irq();
    }

    HMI_TryStartTx();
}

void HMI_Task(void)
{
    uint32_t primask;

    if ((s_tx_busy != 0U) &&
        ((HAL_GetTick() - s_tx_start_tick) >= 200U))
    {
        primask = __get_PRIMASK();
        __disable_irq();
        (void)HAL_UART_AbortTransmit(&huart3);
        if ((s_tx_busy != 0U) && (s_tx_count > 0U))
        {
            s_tx_tail = (uint8_t)((s_tx_tail + 1U) % HMI_TX_QUEUE_SIZE);
            s_tx_count--;
        }
        s_tx_busy = 0U;
        if (primask == 0U)
        {
            __enable_irq();
        }
    }

    HMI_TryStartTx();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        if (s_tx_count > 0U)
        {
            s_tx_tail = (uint8_t)((s_tx_tail + 1U) % HMI_TX_QUEUE_SIZE);
            s_tx_count--;
        }
        s_tx_busy = 0U;
    }
}

void HMI_SetText(const char *obj, const char *text)
{
    char cmd[256];

    snprintf(cmd, sizeof(cmd), "%s.txt=\"%s\"", obj, text);
    HMI_SendCmd(cmd);
}

void HMI_SetPic(const char *obj, uint16_t pic_id)
{
    char cmd[40];

    snprintf(cmd, sizeof(cmd), "%s.pic=%u", obj, pic_id);
    HMI_SendCmd(cmd);

    snprintf(cmd, sizeof(cmd), "ref %s", obj);
    HMI_SendCmd(cmd);
}

void HMI_GotoPage(uint8_t page)
{
    char cmd[16];

    snprintf(cmd, sizeof(cmd), "page %u", page);
    HMI_SendCmd(cmd);
}
