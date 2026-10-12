#include "dht22.h"

static void delay_us(uint32_t us)
{
  uint32_t clk = SystemCoreClock / 1000000;
  while (us--)
    for (uint32_t i = 0; i < clk; i++) __NOP();
}

static void DHT22_OUT(void)
{
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = DHT22_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(DHT22_PORT, &gpio);
}

static void DHT22_IN(void)
{
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = DHT22_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT22_PORT, &gpio);
}

static uint8_t DHT22_Start(void)
{
  DHT22_OUT();
  HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, GPIO_PIN_RESET);
  delay_us(1000);
  HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, GPIO_PIN_SET);
  delay_us(30);

  DHT22_IN();
  uint32_t t = 1000;
  while (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN) && t--) delay_us(1);
  if (t == 0) return 0;

  t = 1000;
  while (!HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN) && t--) delay_us(1);
  if (t == 0) return 0;

  return 1;
}

static uint8_t DHT22_ReadByte(void)
{
  uint8_t dat = 0;
  for (int i = 0; i < 8; i++)
  {
    while (!HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN));
    delay_us(30);
    if (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN)) dat |= (1 << (7 - i));
    while (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN));
  }
  return dat;
}

uint8_t DHT22_Read(float *hum, float *temp)
{
  uint8_t buf[5];
  if (!DHT22_Start()) return 0;

  for (int i = 0; i < 5; i++)
    buf[i] = DHT22_ReadByte();

  if ((buf[0] + buf[1] + buf[2] + buf[3]) != buf[4]) return 0;

  *hum = ((buf[0] << 8) | buf[1]) / 10.0f;
  *temp = ((buf[2] & 0x7F) << 8 | buf[3]) / 10.0f;
  if (buf[2] & 0x80) *temp = -(*temp);

  return 1;
}