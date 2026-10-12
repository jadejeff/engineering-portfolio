#include "main.h"
#include "gpio.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>

/*
 * Rotation diagnostics use existing $STATUS,...# status frames and are
 * disabled in competition builds. They never change task frame parsing.
 */
#ifndef ROTATION_DEBUG_ENABLE
#define ROTATION_DEBUG_ENABLE       0U
#endif

#if (ROTATION_DEBUG_ENABLE != 0U)
#include <stdio.h>
#endif

/* ========================================================================= */
/* ????                                                                  */
/* ========================================================================= */

void SystemClock_Config(void);
void Error_Handler(void);

/* ========================================================================= */
/* ??????                                                              */
/* ========================================================================= */

/* X? */
#define X_STEP_PORT                 GPIOE
#define X_STEP_PIN                  GPIO_PIN_7
#define X_DIR_PORT                  GPIOE
#define X_DIR_PIN                   GPIO_PIN_8

/* Y?? */
#define YL_STEP_PORT                GPIOE
#define YL_STEP_PIN                 GPIO_PIN_9
#define YL_DIR_PORT                 GPIOE
#define YL_DIR_PIN                  GPIO_PIN_10

/* Y?? */
#define YR_STEP_PORT                GPIOE
#define YR_STEP_PIN                 GPIO_PIN_11
#define YR_DIR_PORT                 GPIOE
#define YR_DIR_PIN                  GPIO_PIN_12

/* Z? */
#define Z_STEP_PORT                 GPIOE
#define Z_STEP_PIN                  GPIO_PIN_13
#define Z_DIR_PORT                  GPIOE
#define Z_DIR_PIN                   GPIO_PIN_14

/* ========================================================================= */
/* ????????                                                          */
/* ========================================================================= */

#define ROT_PORT                    GPIOD
#define ROT_IN1                     GPIO_PIN_12
#define ROT_IN2                     GPIO_PIN_13
#define ROT_IN3                     GPIO_PIN_14
#define ROT_IN4                     GPIO_PIN_15

/* ========================================================================= */
/* ???                                                                    */
/* ========================================================================= */

#define MAGNET_PORT                 GPIOB
#define MAGNET_PIN                  GPIO_PIN_0

#define MAGNET_ON_LEVEL             GPIO_PIN_SET
#define MAGNET_OFF_LEVEL            GPIO_PIN_RESET

/* Active-low buzzer module: PC5 -> IN, common GND. */
#define BUZZER_PORT                 GPIOC
#define BUZZER_PIN                  GPIO_PIN_5
#define BUZZER_ON_LEVEL             GPIO_PIN_RESET
#define BUZZER_OFF_LEVEL            GPIO_PIN_SET
#define BUZZER_DONE_TIME_MS         2000U

/* ========================================================================= */
/* ????                                                                  */
/* ========================================================================= */

/*
 * USART2:
 * PA2 = TX,????RX
 * PA3 = RX,????TX
 */
#define VISION_UART_BAUD            115200U
#define UART_FRAME_BUFFER_SIZE      256U

/* HMI serial screen: USART1, PA9=TX, PA10=RX, 9600-8N1. */
#define HMI_UART_BAUD               9600U

/*
 * HMI button protocol:
 *   Requirement 1     : A5 5A 01 01 0D 0A
 *   Requirement 2 (1) : A5 5A 02 01 0D 0A
 *   Requirement 2 (2) : A5 5A 03 01 0D 0A
 *
 * The first two bytes are the header, byte 3 is the mode, byte 4 is START.
 * CR/LF are optional and ignored.
 */
#define HMI_FRAME_HEAD_0            0xA5U
#define HMI_FRAME_HEAD_1            0x5AU
#define HMI_START_COMMAND           0x01U

/* Maximum number of pieces supported by the official task. */
#define MAX_VISION_TASKS            4U

/*
 * Placement coordinates from OpenMV are used directly in all modes.
 * Requirement 1, Requirement 2 (1), and Requirement 2 (2) do not apply
 * any target expansion or center-offset correction in STM32.
 */

typedef enum
{
    PUZZLE_MODE_NONE = 0,
    PUZZLE_MODE_REQ1 = 1,
    PUZZLE_MODE_REQ2_WHITE = 2,
    PUZZLE_MODE_REQ2_CARD = 3
} PuzzleMode;

static UART_HandleTypeDef hvision_uart;
static UART_HandleTypeDef hhmi_uart;

/* ========================================================================= */
/* A4???????????                                                  */
/* ========================================================================= */

/*
 * ??????0.1mm???:
 *
 * A4??:297.0mm -> 2970
 * A4??:210.0mm -> 2100
 */
#define VISION_VERTICAL_MAX_TENTH_MM    2980
#define VISION_HORIZONTAL_MAX_TENTH_MM  2100

/*
 * X???:
 *
 * ??x = 0mm     -> X = 30500
 * ??x = 297mm   -> X = 2800
 */
#define MACHINE_X_AT_RIGHT_EDGE     30500
#define MACHINE_X_AT_LEFT_EDGE      2800

/*
 * Y????????????:
 *
 * ??y = 0mm     -> Y = -40500
 * ??y = 210mm   -> Y = -1600
 */
#define MACHINE_Y_AT_TOP_EDGE      (-40500)
#define MACHINE_Y_AT_BOTTOM_EDGE   (-1600)

/* A4???? */
#define MACHINE_X_MIN               MACHINE_X_AT_LEFT_EDGE
#define MACHINE_X_MAX               MACHINE_X_AT_RIGHT_EDGE

#define MACHINE_Y_MIN               MACHINE_Y_AT_TOP_EDGE
#define MACHINE_Y_MAX               MACHINE_Y_AT_BOTTOM_EDGE

/*
 * ??Reset???
 *
 * Reset?????A4???????
 */
#define RESET_X_STEPS               0
#define RESET_Y_STEPS               0

/* ========================================================================= */
/* Z???                                                                   */
/* ========================================================================= */

/*
 * ??????Z??????????
 *
 * Z=0:????
 * Z=500:??????
 * Z=1500:????
 */
#define Z_CONTACT_STEPS             0
#define Z_RESET_SAFE_STEPS          500
#define Z_TRANSPORT_STEPS           1500

#define Z_MIN_STEPS                 Z_CONTACT_STEPS
#define Z_MAX_STEPS                 Z_TRANSPORT_STEPS

/* ========================================================================= */
/* ?????                                                                */
/* ========================================================================= */

/*
 * ???:
 *
 * 512? = 45?
 * 4096? = 360?
 *
 * ??? = ???
 * ??? = ???
 */
#define ROT_STEPS_PER_45_DEG        512
#define ROT_MAX_ABS_STEPS           2048

/* ========================================================================= */
/* ????                                                                  */
/* ========================================================================= */

/*
 * XY??:
 *
 * 50us? + 50us?
 * ???????10000?/??
 */
#define XY_TICK_HIGH_US            30U
#define XY_TICK_LOW_US             30U

/*
 * Rotation motor scheduling:
 *
 * Rotation is no longer uniformly distributed over the full XY path.
 * It uses an independent M1 acceleration/deceleration profile and stops
 * immediately after the requested number of half-steps is complete.
 *
 * One XY software tick is:
 *     XY_TICK_HIGH_US + XY_TICK_LOW_US
 *
 * Current setting:
 *     start/end: (30us + 30us) * 24 = 1440us per half-step
 *     cruise:    (30us + 30us) * 15 =  900us per half-step
 *
 * Smaller = faster, larger = slower.
 */
#define ROT_FAST_TICKS_PER_STEP     15U
#define ROT_SLOW_TICKS_PER_STEP     24U
#define ROT_RAMP_STEPS              256U

#if (ROT_SLOW_TICKS_PER_STEP < ROT_FAST_TICKS_PER_STEP)
#error "ROT_SLOW_TICKS_PER_STEP must not be faster than ROT_FAST_TICKS_PER_STEP"
#endif

/*
 * Physical direction correction for the PD12-PD15 28BYJ-48 motor:
 *   +1: keep the original coil-sequence direction
 *   -1: reverse the original coil-sequence direction
 *
 * The previously working XY-coupled version rotated physically opposite
 * to the required direction, so this version uses -1.
 */
#define ROT_DIRECTION_SIGN          (-1)

#if ((ROT_DIRECTION_SIGN != 1) && (ROT_DIRECTION_SIGN != -1))
#error "ROT_DIRECTION_SIGN must be +1 or -1"
#endif

/*
 * Z???:
 *
 * 50us? + 50us??
 */
#define Z_STEP_HIGH_US              30U
#define Z_STEP_LOW_US               30U

/* ???????? */
#define DIR_SETUP_US                50U

/* ???? */
#define BOOT_DELAY_MS               1000U

/* ???? */
#define MAGNET_PICK_WAIT_MS         300U

/* ???? */
#define MAGNET_RELEASE_WAIT_MS      400U

/* ========================================================================= */
/* ????                                                                  */
/* ========================================================================= */

/*
 * ??????????????????
 */
#define X_POSITIVE_DIR_LEVEL        GPIO_PIN_SET
#define X_NEGATIVE_DIR_LEVEL        GPIO_PIN_RESET

#define YL_POSITIVE_DIR_LEVEL       GPIO_PIN_SET
#define YL_NEGATIVE_DIR_LEVEL       GPIO_PIN_RESET

#define YR_POSITIVE_DIR_LEVEL       GPIO_PIN_SET
#define YR_NEGATIVE_DIR_LEVEL       GPIO_PIN_RESET

/*
 * Z?????:
 *
 * PE14 SET   -> ??
 * PE14 RESET -> ??
 */
#define Z_UP_DIR_LEVEL              GPIO_PIN_SET
#define Z_DOWN_DIR_LEVEL            GPIO_PIN_RESET

/* ========================================================================= */
/* ??????                                                              */
/* ========================================================================= */

static int32_t current_x_steps = RESET_X_STEPS;
static int32_t current_y_steps = RESET_Y_STEPS;
static int32_t current_z_steps = Z_CONTACT_STEPS;
static int32_t current_rot_steps = 0;

static int8_t current_rot_phase = 0;

/* ========================================================================= */
/* ???????                                                            */
/* ========================================================================= */

/*
 * ????????????10????
 *
 * 213.3mm -> 2133
 * -139?  -> -1390
 */
typedef struct
{
    uint8_t slot;

    int32_t pick_vertical_10;
    int32_t pick_horizontal_10;

    int32_t place_vertical_10;
    int32_t place_horizontal_10;

    int32_t angle_10;

} VisionTask;

/* ========================================================================= */
/* ???????                                                            */
/* ========================================================================= */

static const uint8_t rot_table[8][4] =
{
    {1U, 0U, 0U, 0U},
    {1U, 1U, 0U, 0U},
    {0U, 1U, 0U, 0U},
    {0U, 1U, 1U, 0U},
    {0U, 0U, 1U, 0U},
    {0U, 0U, 1U, 1U},
    {0U, 0U, 0U, 1U},
    {1U, 0U, 0U, 1U}
};

/* ========================================================================= */
/* ??????                                                              */
/* ========================================================================= */

static void Delay_us(uint32_t us)
{
    volatile uint32_t count;

    while (us > 0U)
    {
        count = 84U;

        while (count > 0U)
        {
            __NOP();
            count--;
        }

        us--;
    }
}

static uint32_t AbsInt32(int32_t value)
{
    if (value >= 0)
    {
        return (uint32_t)value;
    }

    return (uint32_t)(-value);
}

static uint32_t MaxU32(uint32_t a, uint32_t b)
{
    return (a > b) ? a : b;
}

static int32_t ClampInt32(
    int32_t value,
    int32_t minimum,
    int32_t maximum
)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}

/* ========================================================================= */
/* USART2????                                                            */
/* ========================================================================= */

static void Vision_UART2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();

    /*
     * PA2 = USART2_TX
     * PA3 = USART2_RX
     */
    GPIO_InitStruct.Pin =
        GPIO_PIN_2 |
        GPIO_PIN_3;

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hvision_uart.Instance = USART2;
    hvision_uart.Init.BaudRate = VISION_UART_BAUD;
    hvision_uart.Init.WordLength = UART_WORDLENGTH_8B;
    hvision_uart.Init.StopBits = UART_STOPBITS_1;
    hvision_uart.Init.Parity = UART_PARITY_NONE;
    hvision_uart.Init.Mode = UART_MODE_TX_RX;
    hvision_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    hvision_uart.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&hvision_uart) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ========================================================================= */
/* USART1: serial HMI start button                                          */
/* ========================================================================= */

static void HMI_UART1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    /*
     * PA9  = USART1_TX -> HMI RX
     * PA10 = USART1_RX <- HMI TX
     */
    GPIO_InitStruct.Pin =
        GPIO_PIN_9 |
        GPIO_PIN_10;

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    hhmi_uart.Instance = USART1;
    hhmi_uart.Init.BaudRate = HMI_UART_BAUD;
    hhmi_uart.Init.WordLength = UART_WORDLENGTH_8B;
    hhmi_uart.Init.StopBits = UART_STOPBITS_1;
    hhmi_uart.Init.Parity = UART_PARITY_NONE;
    hhmi_uart.Init.Mode = UART_MODE_TX_RX;
    hhmi_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    hhmi_uart.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&hhmi_uart) != HAL_OK)
    {
        Error_Handler();
    }
}

/* Discard button bytes received before a fresh vision task is ready.
 * This guarantees that an old/early press cannot start the next task. */
static void HMI_FlushRx(void)
{
    uint8_t byte;

    __HAL_UART_CLEAR_OREFLAG(&hhmi_uart);

    while (HAL_UART_Receive(
               &hhmi_uart,
               &byte,
               1U,
               0U
           ) == HAL_OK)
    {
        /* discard */
    }

    __HAL_UART_CLEAR_OREFLAG(&hhmi_uart);
}

/* Wait for A5 5A 01 01. The trailing 0D 0A sent by the screen is optional. */
static PuzzleMode HMI_WaitForModeButton(void)
{
    uint8_t byte;
    uint8_t state = 0U;
    PuzzleMode pending_mode = PUZZLE_MODE_NONE;

    while (1)
    {
        if (HAL_UART_Receive(
                &hhmi_uart,
                &byte,
                1U,
                HAL_MAX_DELAY
            ) != HAL_OK)
        {
            __HAL_UART_CLEAR_OREFLAG(&hhmi_uart);
            state = 0U;
            pending_mode = PUZZLE_MODE_NONE;
            continue;
        }

        switch (state)
        {
            case 0U:
                if (byte == HMI_FRAME_HEAD_0)
                {
                    state = 1U;
                }
                break;

            case 1U:
                if (byte == HMI_FRAME_HEAD_1)
                {
                    state = 2U;
                }
                else
                {
                    state = (byte == HMI_FRAME_HEAD_0) ? 1U : 0U;
                }
                break;

            case 2U:
                if ((byte >= (uint8_t)PUZZLE_MODE_REQ1) &&
                    (byte <= (uint8_t)PUZZLE_MODE_REQ2_CARD))
                {
                    pending_mode = (PuzzleMode)byte;
                    state = 3U;
                }
                else
                {
                    state = (byte == HMI_FRAME_HEAD_0) ? 1U : 0U;
                    pending_mode = PUZZLE_MODE_NONE;
                }
                break;

            case 3U:
                if (byte == HMI_START_COMMAND)
                {
                    return pending_mode;
                }

                state = (byte == HMI_FRAME_HEAD_0) ? 1U : 0U;
                pending_mode = PUZZLE_MODE_NONE;
                break;

            default:
                state = 0U;
                pending_mode = PUZZLE_MODE_NONE;
                break;
        }
    }
}

static void Vision_SendString(const char *text)
{
    if (text == NULL)
    {
        return;
    }

    HAL_UART_Transmit(
        &hvision_uart,
        (uint8_t *)text,
        (uint16_t)strlen(text),
        200U
    );
}

static void Rotation_DebugLog(
    const char *stage,
    uint8_t slot,
    int32_t angle_10,
    int32_t commanded_steps,
    int32_t current_before,
    int32_t target_steps,
    uint32_t issued_steps,
    int32_t current_after
)
{
#if (ROTATION_DEBUG_ENABLE != 0U)
    /*
     * $STATUS,ROT,stage,slot,angle_10,commanded_steps,
     * current_before,target_steps,issued_steps,current_after#
     *
     * issued_steps is the number of half-step commands verified by the
     * software scheduler. There is no physical rotor-position sensor.
     */
    char message[160];
    int length;

    length = snprintf(
        message,
        sizeof(message),
        "$STATUS,ROT,%s,%u,%ld,%ld,%ld,%ld,%lu,%ld#",
        stage,
        (unsigned int)slot,
        (long)angle_10,
        (long)commanded_steps,
        (long)current_before,
        (long)target_steps,
        (unsigned long)issued_steps,
        (long)current_after
    );

    if ((length > 0) &&
        ((uint32_t)length < (uint32_t)sizeof(message)))
    {
        Vision_SendString(message);
    }
#else
    (void)stage;
    (void)slot;
    (void)angle_10;
    (void)commanded_steps;
    (void)current_before;
    (void)target_steps;
    (void)issued_steps;
    (void)current_after;
#endif
}


static void Vision_SendMode(PuzzleMode mode)
{
    char message[10] = "$MODE,0#";

    if ((mode < PUZZLE_MODE_REQ1) ||
        (mode > PUZZLE_MODE_REQ2_CARD))
    {
        return;
    }

    message[6] = (char)('0' + (uint8_t)mode);
    Vision_SendString(message);
}

static void Vision_SendSlotStatus(
    const char *status,
    uint8_t slot
)
{
    char message[24];
    uint32_t index = 0U;
    const char *p = status;

    message[index++] = '$';

    while ((*p != '\0') &&
           (index < sizeof(message) - 4U))
    {
        message[index++] = *p;
        p++;
    }

    message[index++] = ',';
    message[index++] = (char)('0' + slot);
    message[index++] = '#';
    message[index] = '\0';

    Vision_SendString(message);
}

/*
 * ????:
 *
 * $??#
 *
 * ???buffer????$?#?
 */
static uint8_t Vision_ReadFrame(
    char *buffer,
    uint32_t buffer_size
)
{
    uint8_t received_byte;
    uint32_t index = 0U;
    uint8_t frame_started = 0U;

    if ((buffer == NULL) || (buffer_size < 2U))
    {
        return 0U;
    }

    while (1)
    {
        if (HAL_UART_Receive(
                &hvision_uart,
                &received_byte,
                1U,
                HAL_MAX_DELAY
            ) != HAL_OK)
        {
            return 0U;
        }

        if (frame_started == 0U)
        {
            if (received_byte == '$')
            {
                frame_started = 1U;
                index = 0U;
            }

            continue;
        }

        /*
         * ????$,?????????
         */
        if (received_byte == '$')
        {
            index = 0U;
            continue;
        }

        if (received_byte == '#')
        {
            buffer[index] = '\0';
            return 1U;
        }

        /*
         * ?????
         */
        if ((received_byte == '\r') ||
            (received_byte == '\n'))
        {
            continue;
        }

        if (index >= buffer_size - 1U)
        {
            frame_started = 0U;
            index = 0U;

            Vision_SendString("$ERR,LONG#");

            continue;
        }

        buffer[index++] = (char)received_byte;
    }
}

/* ========================================================================= */
/* ??????                                                              */
/* ========================================================================= */

static void SkipSpaces(const char **cursor)
{
    while ((**cursor == ' ') ||
           (**cursor == '\t'))
    {
        (*cursor)++;
    }
}

/*
 * ???????????10?????
 *
 * 213.3 -> 2133
 * -139  -> -1390
 */
static uint8_t ParseFixed10(
    const char **cursor,
    int32_t *result
)
{
    const char *p;
    int32_t sign = 1;
    int64_t integer_part = 0;
    int32_t decimal_part = 0;
    uint8_t has_digit = 0U;

    p = *cursor;

    SkipSpaces(&p);

    if (*p == '-')
    {
        sign = -1;
        p++;
    }
    else if (*p == '+')
    {
        p++;
    }

    while ((*p >= '0') && (*p <= '9'))
    {
        has_digit = 1U;

        integer_part =
            integer_part * 10 +
            (int64_t)(*p - '0');

        if (integer_part > 1000000)
        {
            return 0U;
        }

        p++;
    }

    if (*p == '.')
    {
        p++;

        if ((*p >= '0') && (*p <= '9'))
        {
            decimal_part = (int32_t)(*p - '0');
            p++;
        }

        /*
         * ???????????
         */
        while ((*p >= '0') && (*p <= '9'))
        {
            p++;
        }
    }

    if (has_digit == 0U)
    {
        return 0U;
    }

    *result =
        sign *
        (int32_t)(
            integer_part * 10 +
            decimal_part
        );

    *cursor = p;

    return 1U;
}

static uint8_t ParseUnsignedInteger(
    const char **cursor,
    uint8_t *result
)
{
    const char *p = *cursor;
    uint32_t value = 0U;
    uint8_t has_digit = 0U;

    SkipSpaces(&p);

    while ((*p >= '0') && (*p <= '9'))
    {
        has_digit = 1U;
        value = value * 10U + (uint32_t)(*p - '0');

        if (value > 255U)
        {
            return 0U;
        }

        p++;
    }

    if (has_digit == 0U)
    {
        return 0U;
    }

    *result = (uint8_t)value;
    *cursor = p;
    return 1U;
}

static uint8_t CoordinateReasonable(
    int32_t value_10,
    int32_t maximum_10
)
{
    /*
     * ???????2mm??,
     * ???????A4???
     */
    if (value_10 < -20)
    {
        return 0U;
    }

    if (value_10 > maximum_10 + 20)
    {
        return 0U;
    }

    return 1U;
}

/*
 * ??????:
 *
 * slot,px,py,sx,sy,ang;
 * slot,px,py,sx,sy,ang;
 * slot,px,py,sx,sy,ang;
 * slot,px,py,sx,sy,ang
 */
static uint8_t ParseVisionFrame(
    const char *frame,
    VisionTask tasks[MAX_VISION_TASKS],
    uint8_t *task_count
)
{
    const char *cursor = frame;
    uint8_t expected_count = 0U;
    uint8_t group_count = 0U;
    uint8_t seen_slots = 0U;

    if ((frame == NULL) || (tasks == NULL) || (task_count == NULL))
    {
        return 0U;
    }

    SkipSpaces(&cursor);

    /*
     * Unified variable-count protocol:
     *   TASK,n;slot,px,py,sx,sy,ang;...
     *
     * Legacy Requirement-1 frames without TASK,n are also accepted:
     *   slot,px,py,sx,sy,ang;...  (normally four groups)
     */
    if (strncmp(cursor, "TASK,", 5U) == 0)
    {
        cursor += 5;

        if (ParseUnsignedInteger(&cursor, &expected_count) == 0U)
        {
            return 0U;
        }

        if ((expected_count < 1U) ||
            (expected_count > MAX_VISION_TASKS))
        {
            return 0U;
        }

        SkipSpaces(&cursor);

        if (*cursor != ';')
        {
            return 0U;
        }

        cursor++;
    }

    while (*cursor != '\0')
    {
        int32_t values[6];
        int32_t slot_value;
        uint8_t field;
        uint8_t slot;

        if (group_count >= MAX_VISION_TASKS)
        {
            return 0U;
        }

        for (field = 0U; field < 6U; field++)
        {
            if (ParseFixed10(&cursor, &values[field]) == 0U)
            {
                return 0U;
            }

            SkipSpaces(&cursor);

            if (field < 5U)
            {
                if (*cursor != ',')
                {
                    return 0U;
                }

                cursor++;
            }
        }

        slot_value = values[0];

        if ((slot_value < 0) ||
            ((slot_value % 10) != 0))
        {
            return 0U;
        }

        slot = (uint8_t)(slot_value / 10);

        if (slot >= MAX_VISION_TASKS)
        {
            return 0U;
        }

        if ((seen_slots & (uint8_t)(1U << slot)) != 0U)
        {
            return 0U;
        }

        if ((CoordinateReasonable(
                 values[1],
                 VISION_VERTICAL_MAX_TENTH_MM
             ) == 0U) ||
            (CoordinateReasonable(
                 values[2],
                 VISION_HORIZONTAL_MAX_TENTH_MM
             ) == 0U) ||
            (CoordinateReasonable(
                 values[3],
                 VISION_VERTICAL_MAX_TENTH_MM
             ) == 0U) ||
            (CoordinateReasonable(
                 values[4],
                 VISION_HORIZONTAL_MAX_TENTH_MM
             ) == 0U))
        {
            return 0U;
        }

        if ((values[5] < -1800) ||
            (values[5] > 1800))
        {
            return 0U;
        }

        /* Store by slot so execution order remains slot 0,1,2,3. */
        tasks[slot].slot = slot;
        tasks[slot].pick_vertical_10 = values[1];
        tasks[slot].pick_horizontal_10 = values[2];
        tasks[slot].place_vertical_10 = values[3];
        tasks[slot].place_horizontal_10 = values[4];
        tasks[slot].angle_10 = values[5];

        seen_slots |= (uint8_t)(1U << slot);
        group_count++;

        SkipSpaces(&cursor);

        if (*cursor == ';')
        {
            cursor++;
            SkipSpaces(&cursor);

            if (*cursor == '\0')
            {
                return 0U;
            }
        }
        else if (*cursor != '\0')
        {
            return 0U;
        }
    }

    if (group_count == 0U)
    {
        return 0U;
    }

    if ((expected_count != 0U) &&
        (group_count != expected_count))
    {
        return 0U;
    }

    /* Require contiguous slot numbers 0..count-1. */
    if (seen_slots != (uint8_t)((1U << group_count) - 1U))
    {
        return 0U;
    }

    *task_count = group_count;
    return 1U;
}

/* ========================================================================= */
/* ??????                                                              */
/* ========================================================================= */

static int32_t VisionHorizontalToMachineXSteps(int32_t horizontal_10)
{
    int64_t numerator;
    int32_t result;

    horizontal_10 = ClampInt32(
        horizontal_10,
        0,
        VISION_HORIZONTAL_MAX_TENTH_MM
    );

    /*
     * x=0mm   -> 30500
     * x=297mm -> 2800
     */
    numerator =
        (int64_t)horizontal_10 *
        (MACHINE_X_AT_LEFT_EDGE -
         MACHINE_X_AT_RIGHT_EDGE);

    /*
     * ????????
     */
    if (numerator >= 0)
    {
        numerator += VISION_HORIZONTAL_MAX_TENTH_MM / 2;
    }
    else
    {
        numerator -= VISION_HORIZONTAL_MAX_TENTH_MM / 2;
    }

    result =
        MACHINE_X_AT_RIGHT_EDGE +
        (int32_t)(
            numerator /
            VISION_HORIZONTAL_MAX_TENTH_MM
        );

    return ClampInt32(
        result,
        MACHINE_X_MIN,
        MACHINE_X_MAX
    );
}

static int32_t VisionVerticalToMachineYSteps(int32_t vertical_10)
{
    int64_t numerator;
    int32_t result;

    vertical_10 = ClampInt32(
        vertical_10,
        0,
        VISION_VERTICAL_MAX_TENTH_MM
    );

    /*
     * ????Y??:
     *
     * y=0mm   -> -40500
     * y=210mm -> -1600
     */
    numerator =
        (int64_t)vertical_10 *
        (MACHINE_Y_AT_BOTTOM_EDGE -
         MACHINE_Y_AT_TOP_EDGE);

    if (numerator >= 0)
    {
        numerator += VISION_VERTICAL_MAX_TENTH_MM / 2;
    }
    else
    {
        numerator -= VISION_VERTICAL_MAX_TENTH_MM / 2;
    }

    result =
        MACHINE_Y_AT_TOP_EDGE +
        (int32_t)(
            numerator /
            VISION_VERTICAL_MAX_TENTH_MM
        );

    return ClampInt32(
        result,
        MACHINE_Y_MIN,
        MACHINE_Y_MAX
    );
}

static int32_t AngleToRotationSteps(int32_t angle_10)
{
    int64_t numerator;

    /*
     * 45?= 450?0.1?= 512??
     */
    numerator =
        (int64_t)angle_10 *
        ROT_STEPS_PER_45_DEG;

    if (numerator >= 0)
    {
        numerator += 225;
    }
    else
    {
        numerator -= 225;
    }

    return (int32_t)(numerator / 450);
}

/* ========================================================================= */
/* ?????                                                                */
/* ========================================================================= */

static void Magnet_On(void)
{
    HAL_GPIO_WritePin(
        MAGNET_PORT,
        MAGNET_PIN,
        MAGNET_ON_LEVEL
    );
}

static void Magnet_Off(void)
{
    HAL_GPIO_WritePin(
        MAGNET_PORT,
        MAGNET_PIN,
        MAGNET_OFF_LEVEL
    );
}

static void Buzzer_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    /*
     * Active-low buzzer on PC5:
     * HIGH = off, LOW = on.
     * Preload the inactive HIGH state before enabling output mode.
     */
    HAL_GPIO_WritePin(
        BUZZER_PORT,
        BUZZER_PIN,
        BUZZER_OFF_LEVEL
    );

    GPIO_InitStruct.Pin = BUZZER_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(
        BUZZER_PORT,
        &GPIO_InitStruct
    );

    HAL_GPIO_WritePin(
        BUZZER_PORT,
        BUZZER_PIN,
        BUZZER_OFF_LEVEL
    );
}

static void Buzzer_On(void)
{
    HAL_GPIO_WritePin(
        BUZZER_PORT,
        BUZZER_PIN,
        BUZZER_ON_LEVEL
    );
}

static void Buzzer_Off(void)
{
    HAL_GPIO_WritePin(
        BUZZER_PORT,
        BUZZER_PIN,
        BUZZER_OFF_LEVEL
    );
}

static void Buzzer_Beep(uint32_t duration_ms)
{
    Buzzer_On();
    HAL_Delay(duration_ms);
    Buzzer_Off();
}

/* ========================================================================= */
/* ????                                                                  */
/* ========================================================================= */

static void X_SetDirection(uint8_t positive)
{
    HAL_GPIO_WritePin(
        X_DIR_PORT,
        X_DIR_PIN,
        (positive != 0U) ?
        X_POSITIVE_DIR_LEVEL :
        X_NEGATIVE_DIR_LEVEL
    );
}

static void Y_SetDirection(uint8_t positive)
{
    HAL_GPIO_WritePin(
        YL_DIR_PORT,
        YL_DIR_PIN,
        (positive != 0U) ?
        YL_POSITIVE_DIR_LEVEL :
        YL_NEGATIVE_DIR_LEVEL
    );

    HAL_GPIO_WritePin(
        YR_DIR_PORT,
        YR_DIR_PIN,
        (positive != 0U) ?
        YR_POSITIVE_DIR_LEVEL :
        YR_NEGATIVE_DIR_LEVEL
    );
}

static void Z_SetDirection(uint8_t upward)
{
    HAL_GPIO_WritePin(
        Z_DIR_PORT,
        Z_DIR_PIN,
        (upward != 0U) ?
        Z_UP_DIR_LEVEL :
        Z_DOWN_DIR_LEVEL
    );
}

/* ========================================================================= */
/* ?????                                                                */
/* ========================================================================= */

static void Rotate_OutputPhase(uint8_t phase)
{
    HAL_GPIO_WritePin(
        ROT_PORT,
        ROT_IN1,
        (rot_table[phase][0] != 0U) ?
        GPIO_PIN_SET :
        GPIO_PIN_RESET
    );

    HAL_GPIO_WritePin(
        ROT_PORT,
        ROT_IN2,
        (rot_table[phase][1] != 0U) ?
        GPIO_PIN_SET :
        GPIO_PIN_RESET
    );

    HAL_GPIO_WritePin(
        ROT_PORT,
        ROT_IN3,
        (rot_table[phase][2] != 0U) ?
        GPIO_PIN_SET :
        GPIO_PIN_RESET
    );

    HAL_GPIO_WritePin(
        ROT_PORT,
        ROT_IN4,
        (rot_table[phase][3] != 0U) ?
        GPIO_PIN_SET :
        GPIO_PIN_RESET
    );
}

/*
 * direction > 0:???
 * direction < 0:???
 */
static void Rotate_OneStep(int8_t direction)
{
    if (direction > 0)
    {
        current_rot_phase++;

        if (current_rot_phase >= 8)
        {
            current_rot_phase = 0;
        }
    }
    else
    {
        current_rot_phase--;

        if (current_rot_phase < 0)
        {
            current_rot_phase = 7;
        }
    }

    Rotate_OutputPhase(
        (uint8_t)current_rot_phase
    );
}

/* ========================================================================= */
/* Z???                                                                   */
/* ========================================================================= */

static uint8_t Z_MoveTo(int32_t target_z_steps)
{
    int32_t delta;
    uint32_t pulse_count;
    uint32_t i;

    if ((target_z_steps < Z_MIN_STEPS) ||
        (target_z_steps > Z_MAX_STEPS))
    {
        return 0U;
    }

    delta =
        target_z_steps -
        current_z_steps;

    if (delta == 0)
    {
        return 1U;
    }

    if (delta > 0)
    {
        Z_SetDirection(1U);
        pulse_count = (uint32_t)delta;
    }
    else
    {
        Z_SetDirection(0U);
        pulse_count = (uint32_t)(-delta);
    }

    Delay_us(DIR_SETUP_US);

    for (i = 0U; i < pulse_count; i++)
    {
        HAL_GPIO_WritePin(
            Z_STEP_PORT,
            Z_STEP_PIN,
            GPIO_PIN_SET
        );

        Delay_us(Z_STEP_HIGH_US);

        HAL_GPIO_WritePin(
            Z_STEP_PORT,
            Z_STEP_PIN,
            GPIO_PIN_RESET
        );

        Delay_us(Z_STEP_LOW_US);
    }

    current_z_steps =
        target_z_steps;

    return 1U;
}

/* ========================================================================= */
/* XY????                                                                */
/* ========================================================================= */

static uint8_t MachinePointInsideA4(
    int32_t x_steps,
    int32_t y_steps
)
{
    if ((x_steps < MACHINE_X_MIN) ||
        (x_steps > MACHINE_X_MAX))
    {
        return 0U;
    }

    if ((y_steps < MACHINE_Y_MIN) ||
        (y_steps > MACHINE_Y_MAX))
    {
        return 0U;
    }

    return 1U;
}

static uint8_t IsResetPoint(
    int32_t x_steps,
    int32_t y_steps
)
{
    if ((x_steps == RESET_X_STEPS) &&
        (y_steps == RESET_Y_STEPS))
    {
        return 1U;
    }

    return 0U;
}

/* ========================================================================= */
/* XY???????                                                          */
/* ========================================================================= */

/*
 * M1 rotation profile:
 *   - first ROT_RAMP_STEPS half-steps: 24 -> 15 ticks/step;
 *   - middle section:                  15 ticks/step;
 *   - last ROT_RAMP_STEPS half-steps:  15 -> 24 ticks/step.
 *
 * Short moves automatically use half their travel as the ramp length. The
 * fastest existing rate is retained, while starts, stops and reversals no
 * longer apply the full rate in one step.
 */
static uint32_t Rotation_StepIntervalTicks(
    uint32_t step_index,
    uint32_t total_steps
)
{
    uint32_t ramp_steps;
    uint32_t steps_from_start;
    uint32_t steps_from_end;
    uint32_t edge_distance;
    uint32_t interval_range;

    if ((total_steps == 0U) || (step_index >= total_steps))
    {
        return ROT_SLOW_TICKS_PER_STEP;
    }

    ramp_steps = ROT_RAMP_STEPS;
    if (ramp_steps > (total_steps / 2U))
    {
        ramp_steps = total_steps / 2U;
    }

    if (ramp_steps == 0U)
    {
        return ROT_SLOW_TICKS_PER_STEP;
    }

    steps_from_start = step_index;
    steps_from_end = total_steps - 1U - step_index;
    edge_distance =
        (steps_from_start < steps_from_end) ?
        steps_from_start :
        steps_from_end;

    if (edge_distance >= ramp_steps)
    {
        return ROT_FAST_TICKS_PER_STEP;
    }

    interval_range =
        ROT_SLOW_TICKS_PER_STEP -
        ROT_FAST_TICKS_PER_STEP;

    return
        ROT_SLOW_TICKS_PER_STEP -
        (interval_range * edge_distance) /
        ramp_steps;
}


static uint32_t Rotation_TotalTicks(uint32_t total_steps)
{
    uint32_t step_index;
    uint32_t total_ticks = 0U;

    for (step_index = 0U;
         step_index < total_steps;
         step_index++)
    {
        total_ticks += Rotation_StepIntervalTicks(
            step_index,
            total_steps
        );
    }

    return total_ticks;
}

/*
 * XY?????????
 *
 * ???????????????,
 * ??Z?????????????
 */
static uint8_t XYRot_MoveTo(
    int32_t target_x_steps,
    int32_t target_y_steps,
    int32_t target_rot_steps
)
{
    int32_t delta_x;
    int32_t delta_y;
    int32_t delta_rot;

    uint32_t abs_x;
    uint32_t abs_y;
    uint32_t abs_rot;

    uint32_t xy_total_ticks;
    uint32_t rotation_total_ticks;
    uint32_t total_ticks;

    uint32_t accumulator_x = 0U;
    uint32_t accumulator_y = 0U;

    uint32_t rotation_tick_counter = 0U;
    uint32_t rotation_steps_done = 0U;

    uint32_t tick;

    /*
     * Target must be inside the calibrated A4 working range.
     * The defined reset point is also allowed.
     */
    if ((MachinePointInsideA4(
             target_x_steps,
             target_y_steps
         ) == 0U) &&
        (IsResetPoint(
             target_x_steps,
             target_y_steps
         ) == 0U))
    {
        return 0U;
    }

    if ((target_rot_steps < -ROT_MAX_ABS_STEPS) ||
        (target_rot_steps > ROT_MAX_ABS_STEPS))
    {
        return 0U;
    }

    delta_x =
        target_x_steps -
        current_x_steps;

    delta_y =
        target_y_steps -
        current_y_steps;

    delta_rot =
        target_rot_steps -
        current_rot_steps;

    abs_x = AbsInt32(delta_x);
    abs_y = AbsInt32(delta_y);
    abs_rot = AbsInt32(delta_rot);

    if ((abs_x == 0U) &&
        (abs_y == 0U) &&
        (abs_rot == 0U))
    {
        return 1U;
    }

    if (delta_x > 0)
    {
        X_SetDirection(1U);
    }
    else if (delta_x < 0)
    {
        X_SetDirection(0U);
    }

    if (delta_y > 0)
    {
        Y_SetDirection(1U);
    }
    else if (delta_y < 0)
    {
        Y_SetDirection(0U);
    }

    Delay_us(DIR_SETUP_US);

    /*
     * XY uses its own total tick count, so its speed is not reduced by
     * a long rotation command.
     */
    xy_total_ticks =
        MaxU32(abs_x, abs_y);

    /* Rotation duration is the exact sum of the M1 ramp intervals. */
    rotation_total_ticks = Rotation_TotalTicks(abs_rot);

    /*
     * The loop only ends after both independent schedules are complete.
     *
     * - If rotation finishes first, it stops and XY continues.
     * - If XY finishes first, XY stops and rotation continues.
     */
    total_ticks =
        MaxU32(
            xy_total_ticks,
            rotation_total_ticks
        );

    for (tick = 0U;
         tick < total_ticks;
         tick++)
    {
        uint16_t pulse_pins = 0U;

        /*
         * X/Y DDA is based on xy_total_ticks, not total_ticks.
         * Therefore rotation duration cannot slow the XY axes.
         */
        if ((tick < xy_total_ticks) &&
            (abs_x > 0U))
        {
            accumulator_x += abs_x;

            if (accumulator_x >= xy_total_ticks)
            {
                accumulator_x -= xy_total_ticks;
                pulse_pins |= X_STEP_PIN;
            }
        }

        if ((tick < xy_total_ticks) &&
            (abs_y > 0U))
        {
            accumulator_y += abs_y;

            if (accumulator_y >= xy_total_ticks)
            {
                accumulator_y -= xy_total_ticks;

                pulse_pins |=
                    YL_STEP_PIN |
                    YR_STEP_PIN;
            }
        }

        /* Independent M1 ramp; it is not spread over the XY path. */
        if (rotation_steps_done < abs_rot)
        {
            uint32_t rotation_step_interval;

            rotation_tick_counter++;
            rotation_step_interval = Rotation_StepIntervalTicks(
                rotation_steps_done,
                abs_rot
            );

            if (rotation_tick_counter >=
                rotation_step_interval)
            {
                rotation_tick_counter = 0U;

                Rotate_OneStep(
                    (int8_t)(
                        ((delta_rot > 0) ? 1 : -1) *
                        ROT_DIRECTION_SIGN
                    )
                );

                rotation_steps_done++;
            }
        }

        if (pulse_pins != 0U)
        {
            HAL_GPIO_WritePin(
                GPIOE,
                pulse_pins,
                GPIO_PIN_SET
            );
        }

        Delay_us(XY_TICK_HIGH_US);

        if (pulse_pins != 0U)
        {
            HAL_GPIO_WritePin(
                GPIOE,
                pulse_pins,
                GPIO_PIN_RESET
            );
        }

        Delay_us(XY_TICK_LOW_US);
    }

    /*
     * Software completion checks. Reaching here means the requested pulse
     * counts for X, Y and rotation have all been issued.
     */
    if (rotation_steps_done != abs_rot)
    {
        return 0U;
    }

    current_x_steps =
        target_x_steps;

    current_y_steps =
        target_y_steps;

    current_rot_steps =
        target_rot_steps;

    return 1U;
}

/* ========================================================================= */
/* ??????                                                              */
/* ========================================================================= */

static uint8_t ExecuteTask(
    const VisionTask *task,
    uint8_t is_last_task
)
{
    int32_t pick_x_steps;
    int32_t pick_y_steps;

    int32_t place_x_steps;
    int32_t place_y_steps;

    int32_t rotation_steps;
    int32_t rotation_before_steps;
    uint32_t rotation_issued_steps;

    pick_x_steps =
        VisionHorizontalToMachineXSteps(
            task->pick_horizontal_10
        );

    pick_y_steps =
        VisionVerticalToMachineYSteps(
            task->pick_vertical_10
        );

    place_x_steps =
        VisionHorizontalToMachineXSteps(
            task->place_horizontal_10
        );

    place_y_steps =
        VisionVerticalToMachineYSteps(
            task->place_vertical_10
        );

    rotation_steps =
        AngleToRotationSteps(
            task->angle_10
        );

    Rotation_DebugLog(
        "RX",
        task->slot,
        task->angle_10,
        rotation_steps,
        current_rot_steps,
        current_rot_steps,
        0U,
        current_rot_steps
    );

    Vision_SendSlotStatus(
        "RUN",
        task->slot
    );

    /*
     * 1. ?Reset???????
     * ??Z??500??????
     */
    rotation_before_steps = current_rot_steps;

    if (XYRot_MoveTo(
            pick_x_steps,
            pick_y_steps,
            0
        ) == 0U)
    {
        return 0U;
    }

    rotation_issued_steps = AbsInt32(
        0 - rotation_before_steps
    );

    Rotation_DebugLog(
        "PICK_ZERO",
        task->slot,
        task->angle_10,
        rotation_steps,
        rotation_before_steps,
        0,
        rotation_issued_steps,
        current_rot_steps
    );

    /*
     * 2. ?????,???????
     */
    Magnet_On();

    if (Z_MoveTo(
            Z_CONTACT_STEPS
        ) == 0U)
    {
        Magnet_Off();
        return 0U;
    }

    /*
     * ??????????
     */
    HAL_Delay(
        MAGNET_PICK_WAIT_MS
    );

    /*
     * 3. ???1500??????
     */
    if (Z_MoveTo(
            Z_TRANSPORT_STEPS
        ) == 0U)
    {
        Magnet_Off();
        return 0U;
    }

    /*
     * 4. XY?????,?????
     */
    rotation_before_steps = current_rot_steps;

    if (XYRot_MoveTo(
            place_x_steps,
            place_y_steps,
            rotation_steps
        ) == 0U)
    {
        Magnet_Off();
        return 0U;
    }

    rotation_issued_steps = AbsInt32(
        rotation_steps - rotation_before_steps
    );

    Rotation_DebugLog(
        "PLACE",
        task->slot,
        task->angle_10,
        rotation_steps,
        rotation_before_steps,
        rotation_steps,
        rotation_issued_steps,
        current_rot_steps
    );

    /*
     * ??XY???????,
     * ???Z???
     */
    if (Z_MoveTo(
            Z_CONTACT_STEPS
        ) == 0U)
    {
        Magnet_Off();
        return 0U;
    }

    /*
     * 5. ?????,?????
     */
    Magnet_Off();

    /*
     * Completion indication starts at the real scoring event: the final
     * piece has reached the placement surface and magnetic force is removed.
     * Keep the existing two-second duration and do not beep for earlier slots.
     */
    if (is_last_task != 0U)
    {
        Buzzer_Beep(BUZZER_DONE_TIME_MS);
    }

    HAL_Delay(
        MAGNET_RELEASE_WAIT_MS
    );

    /*
     * 6. Z??????????????
     */
    if (Z_MoveTo(
            Z_TRANSPORT_STEPS
        ) == 0U)
    {
        return 0U;
    }

    Vision_SendSlotStatus(
        "DONE",
        task->slot
    );

    return 1U;
}

/* ========================================================================= */
/* ???                                                                    */
/* ========================================================================= */

int main(void)
{
    VisionTask tasks[MAX_VISION_TASKS];
    char frame_buffer[UART_FRAME_BUFFER_SIZE];
    PuzzleMode selected_mode;
    uint8_t task_count;
    uint8_t slot;
    uint8_t task_ready;

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    Buzzer_GPIO_Init();
    Vision_UART2_Init();
    HMI_UART1_Init();

    current_x_steps = RESET_X_STEPS;
    current_y_steps = RESET_Y_STEPS;
    current_z_steps = Z_CONTACT_STEPS;
    current_rot_steps = 0;

    Magnet_Off();
    Buzzer_Off();

    HAL_GPIO_WritePin(
        GPIOE,
        X_STEP_PIN |
        YL_STEP_PIN |
        YR_STEP_PIN |
        Z_STEP_PIN,
        GPIO_PIN_RESET
    );

    current_rot_phase = 0;
    Rotate_OutputPhase((uint8_t)current_rot_phase);

    HAL_Delay(BOOT_DELAY_MS);

    if (Z_MoveTo(Z_TRANSPORT_STEPS) == 0U)
    {
        Error_Handler();
    }

    /* OpenMV may boot before or after STM32. This announces the control side. */
    Vision_SendString("$READY#");

    while (1)
    {
        task_count = 0U;
        task_ready = 0U;

        /*
         * One official test starts here. The HMI press selects and starts the
         * mode at the same time. There is no second confirmation button.
         */
        selected_mode = HMI_WaitForModeButton();

        Vision_SendMode(selected_mode);
        Vision_SendString("$WAIT_DATA#");

        /*
         * OpenMV now runs the selected algorithm while the camera cover is
         * being removed. STM32 remains here until a valid task frame arrives.
         */
        while (task_ready == 0U)
        {
            if (Vision_ReadFrame(
                    frame_buffer,
                    sizeof(frame_buffer)
                ) == 0U)
            {
                Vision_SendString("$ERR,RX#");
                continue;
            }

            /* If OpenMV reset after the button press, resend the locked mode. */
            if (strcmp(frame_buffer, "READY") == 0)
            {
                Vision_SendMode(selected_mode);
                Vision_SendString("$WAIT_DATA#");
                continue;
            }

            /* Status frames are informative and are not task data. */
            if ((strncmp(frame_buffer, "MODE_ACK", 8U) == 0) ||
                (strncmp(frame_buffer, "STATUS", 6U) == 0) ||
                (strcmp(frame_buffer, "WAITING") == 0))
            {
                continue;
            }

            if (ParseVisionFrame(
                    frame_buffer,
                    tasks,
                    &task_count
                ) == 0U)
            {
                Vision_SendString("$ERR,FRAME#");
                continue;
            }

            task_ready = 1U;
        }

        /*
         * Requirement 2 (1): expand the exact solved assembly by 1.5% around
         * its configured rectangle centre. Pick coordinates and angles remain
         * unchanged. Requirement 1 and Requirement 2 (2) are not expanded.
         */
                Vision_SendString("$ACK#");
        Vision_SendString("$STARTED#");

        if (Z_MoveTo(Z_TRANSPORT_STEPS) == 0U)
        {
            Magnet_Off();
            Vision_SendString("$ERR,MOTION#");
            Error_Handler();
        }

        for (slot = 0U; slot < task_count; slot++)
        {
            if (ExecuteTask(
                    &tasks[slot],
                    ((slot + 1U) == task_count) ? 1U : 0U
                ) == 0U)
            {
                Magnet_Off();
                Vision_SendString("$ERR,MOTION#");
                Error_Handler();
            }
        }

        if (XYRot_MoveTo(
                RESET_X_STEPS,
                RESET_Y_STEPS,
                0
            ) == 0U)
        {
            Magnet_Off();
            Vision_SendString("$ERR,MOTION#");
            Error_Handler();
        }

        if (Z_MoveTo(Z_RESET_SAFE_STEPS) == 0U)
        {
            Magnet_Off();
            Vision_SendString("$ERR,MOTION#");
            Error_Handler();
        }

        Magnet_Off();

        /* Completion frame follows the existing full return-to-safe sequence. */
        Vision_SendString("$ALL_DONE#");

        /* Return to the single-button waiting state for the next test. */
        Vision_SendString("$READY#");
    }
}

/* ========================================================================= */
/* ????:HSE 8MHz -> 168MHz                                               */
/* ========================================================================= */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(
        PWR_REGULATOR_VOLTAGE_SCALE1
    );

    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSE;

    RCC_OscInitStruct.HSEState =
        RCC_HSE_ON;

    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_ON;

    RCC_OscInitStruct.PLL.PLLSource =
        RCC_PLLSOURCE_HSE;

    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP =
        RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;

    if (HAL_RCC_OscConfig(
            &RCC_OscInitStruct
        ) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1 |
        RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_PLLCLK;

    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV4;

    RCC_ClkInitStruct.APB2CLKDivider =
        RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(
            &RCC_ClkInitStruct,
            FLASH_LATENCY_5
        ) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ========================================================================= */
/* ????                                                                  */
/* ========================================================================= */

void Error_Handler(void)
{
    __disable_irq();

    /*
     * ??????
     */
    HAL_GPIO_WritePin(
        MAGNET_PORT,
        MAGNET_PIN,
        MAGNET_OFF_LEVEL
    );

    HAL_GPIO_WritePin(
        BUZZER_PORT,
        BUZZER_PIN,
        BUZZER_OFF_LEVEL
    );

    /*
     * ?????????
     */
    HAL_GPIO_WritePin(
        GPIOE,
        X_STEP_PIN |
        YL_STEP_PIN |
        YR_STEP_PIN |
        Z_STEP_PIN,
        GPIO_PIN_RESET
    );

    /*
     * ???????????
     */
    HAL_GPIO_WritePin(
        ROT_PORT,
        ROT_IN1 |
        ROT_IN2 |
        ROT_IN3 |
        ROT_IN4,
        GPIO_PIN_RESET
    );

    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT

void assert_failed(
    uint8_t *file,
    uint32_t line
)
{
    (void)file;
    (void)line;
}

#endif
