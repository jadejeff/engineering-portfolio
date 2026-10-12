#include "load_buttons.h"
#include "relay.h"
#include <stdio.h>

#define LOAD_BUTTON_COUNT        3U
#define LOAD_BUTTON_DEBOUNCE_MS  30U

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState raw_state;
    GPIO_PinState stable_state;
    uint32_t change_tick;
} LoadButton;

static LoadButton s_buttons[LOAD_BUTTON_COUNT] =
{
    {PWR1_BTN_GPIO_Port, PWR1_BTN_Pin, GPIO_PIN_SET, GPIO_PIN_SET, 0U},
    {PWR2_BTN_GPIO_Port, PWR2_BTN_Pin, GPIO_PIN_SET, GPIO_PIN_SET, 0U},
    {PWR3_BTN_GPIO_Port, PWR3_BTN_Pin, GPIO_PIN_SET, GPIO_PIN_SET, 0U}
};

void LoadButtons_Init(void)
{
    uint8_t i;

    for (i = 0U; i < LOAD_BUTTON_COUNT; i++)
    {
        s_buttons[i].raw_state = HAL_GPIO_ReadPin(s_buttons[i].port,
                                                  s_buttons[i].pin);
        s_buttons[i].stable_state = s_buttons[i].raw_state;
        s_buttons[i].change_tick = HAL_GetTick();
    }

    printf("[BUTTON] INIT PC6=%u PC7=%u PC8=%u\r\n",
           (unsigned int)s_buttons[0].stable_state,
           (unsigned int)s_buttons[1].stable_state,
           (unsigned int)s_buttons[2].stable_state);
}

void LoadButtons_Task(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t i;

    for (i = 0U; i < LOAD_BUTTON_COUNT; i++)
    {
        GPIO_PinState raw = HAL_GPIO_ReadPin(s_buttons[i].port,
                                             s_buttons[i].pin);

        if (raw != s_buttons[i].raw_state)
        {
            s_buttons[i].raw_state = raw;
            s_buttons[i].change_tick = now;
            printf("[BUTTON] RAW K%u=%u\r\n",
                   (unsigned int)(i + 1U),
                   (unsigned int)raw);
        }

        if ((raw != s_buttons[i].stable_state) &&
            ((now - s_buttons[i].change_tick) >= LOAD_BUTTON_DEBOUNCE_MS))
        {
            s_buttons[i].stable_state = raw;

            if (raw == GPIO_PIN_RESET)
            {
                uint8_t index = i + 1U;
                uint8_t next_on = HighPowerLoad_Get(index) ? 0U : 1U;
                (void)HighPowerLoad_Set(index, next_on);
            }
        }
    }
}
