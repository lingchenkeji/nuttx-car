/****************************************************************************
 * apps/myprojects/modules/ws2812/ws2812_api.c
 *
 * WS2812B RGB LED Driver Implementation
 *
 * Uses bit-banging on PB8 (TIM4 CH3) to send data to WS2812B.
 * Timing is calibrated for STM32F103 @ 72MHz.
 *
 * WS2812B-2020 Timing (800kHz):
 *   T0H 220-380ns   T0L 580ns-1us
 *   T1H 580ns-1us   T1L 580ns-1us
 *   RES >= 280us
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/
/****************************************************************************
 * WS2812 氛围灯的颜色和闪烁。状态含义见 include/ws2812_api.h。
 ****************************************************************************/


/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>

#include <nuttx/kmalloc.h>
#include <nuttx/irq.h>

#ifdef CONFIG_ARCH_CHIP_STM32
#  include <stm32_gpio.h>
#endif

#include "../../include/ws2812_api.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* WS2812B-2020 Timing @ 72MHz (13.89ns per cycle)
 *
 * Datasheet specs:
 *   T0H  220-380ns    T0L  580ns-1us
 *   T1H  580ns-1us    T1L  580ns-1us
 *   RES  >= 280us
 *   Total bit >= 1.25us
 *
 * The while-loop + PIN_SET/CLR adds ~8 cycles of overhead to the
 * measured HIGH time.  The cycle counts below are the DWT threshold
 * values; actual pin-HIGH time is approximately (cycles + 8) * 13.89ns.
 */

#define WS2812_BIT_CYCLES  90
#define WS2812_T0H_CYCLES  14   /* (14+8)*13.89 ≈ 306ns  [spec 220-380] */
#define WS2812_T1H_CYCLES  42   /* (42+8)*13.89 ≈ 694ns  [spec 580-1000] */
#define WS2812_RESET_US    300  /* >= 280us required by WS2812B-2020 */

/* Cortex-M DWT cycle counter registers */

#define ARM_DEMCR           (*(volatile uint32_t *)0xe000edfc)
#define ARM_DWT_CTRL        (*(volatile uint32_t *)0xe0001000)
#define ARM_DWT_CYCCNT      (*(volatile uint32_t *)0xe0001004)
#define ARM_DEMCR_TRCENA    (1u << 24)
#define ARM_DWT_CYCCNTENA   (1u << 0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ws2812_dev_s
{
  struct ws2812_pins_s pins;
  struct ws2812_color_s current_color;
  bool initialized;
  uint8_t brightness;  /* 0-100, default 15 */
  uint32_t set_regaddr; /* GPIOx_BSRR address */
  uint32_t clr_regaddr; /* GPIOx_BRR address (F1 series) */
  uint32_t bitmask;     /* 1 << pin */
};

/****************************************************************************
 * Private Inline Functions
 ****************************************************************************/

static inline void ws2812_timing_init(void)
{
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CYCCNT = 0;
  ARM_DWT_CTRL |= ARM_DWT_CYCCNTENA;
}

/* Pin access as macros to guarantee zero function-call overhead */

#define WS2812_PIN_SET(dev) \
  (*(volatile uint32_t *)(dev)->set_regaddr = (dev)->bitmask)
#define WS2812_PIN_CLR(dev) \
  (*(volatile uint32_t *)(dev)->clr_regaddr = (dev)->bitmask)
#define WS2812_DWT_READ() ARM_DWT_CYCCNT
#define WS2812_DWT_ELAPSED(start) ((uint32_t)(ARM_DWT_CYCCNT - (start)))

/* Send a single bit to WS2812B.
 * t0 is sampled BEFORE PIN_SET so that both edges have symmetric
 * pipeline delay relative to the DWT measurement, giving ±1-cycle
 * accuracy on the HIGH pulse width.
 */

static inline void ws2812_send_bit(FAR const struct ws2812_dev_s *dev,
                                   bool bit)
{
  uint32_t t0;
  uint32_t th = bit ? WS2812_T1H_CYCLES : WS2812_T0H_CYCLES;

  t0 = WS2812_DWT_READ();
  WS2812_PIN_SET(dev);
  while (WS2812_DWT_ELAPSED(t0) < th);
  WS2812_PIN_CLR(dev);
  while (WS2812_DWT_ELAPSED(t0) < WS2812_BIT_CYCLES);
}

/* Send a byte to WS2812B (MSB first) */

static inline void ws2812_send_byte(FAR const struct ws2812_dev_s *dev,
                                    uint8_t byte)
{
  int i;
  for (i = 7; i >= 0; i--)
    {
      ws2812_send_bit(dev, (byte >> i) & 1);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ws2812_register
 ****************************************************************************/

FAR struct ws2812_dev_s *ws2812_register(
  FAR const struct ws2812_pins_s *pins)
{
  FAR struct ws2812_dev_s *dev;
  uint32_t port;
  uint32_t pin;

  if (pins == NULL)
    {
      return NULL;
    }

  dev = (FAR struct ws2812_dev_s *)kmm_zalloc(
    sizeof(struct ws2812_dev_s));
  if (dev == NULL)
    {
      printf("WS2812: ERROR kmm_zalloc failed!\n");
      return NULL;
    }

  memcpy(&dev->pins, pins, sizeof(struct ws2812_pins_s));

#ifdef CONFIG_ARCH_CHIP_STM32
  /* Configure DIN pin as push-pull output, initial LOW */

  port = (dev->pins.dinpin & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  pin  = (dev->pins.dinpin & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  dev->bitmask = (1u << pin);

  switch (port)
    {
      case 0: /* GPIOA */
        dev->set_regaddr = STM32_GPIOA_BSRR;
        dev->clr_regaddr = STM32_GPIOA_BRR;
        break;
      case 1: /* GPIOB */
        dev->set_regaddr = STM32_GPIOB_BSRR;
        dev->clr_regaddr = STM32_GPIOB_BRR;
        break;
      case 2: /* GPIOC */
        dev->set_regaddr = STM32_GPIOC_BSRR;
        dev->clr_regaddr = STM32_GPIOC_BRR;
        break;
      default:
        kmm_free(dev);
        return NULL;
    }

  stm32_configgpio(dev->pins.dinpin);
  stm32_gpiowrite(dev->pins.dinpin, false);
  ws2812_timing_init();
#else
  kmm_free(dev);
  return NULL;
#endif

  /* Initialize color to black (off) */

  dev->current_color.r = 0;
  dev->current_color.g = 0;
  dev->current_color.b = 0;
  dev->brightness = 15;
  dev->initialized = true;

  printf("WS2812: registered (pin=0x%08lx)\n",
         (unsigned long)dev->pins.dinpin);
  return dev;
}

/****************************************************************************
 * Name: ws2812_unregister
 ****************************************************************************/

int ws2812_unregister(FAR struct ws2812_dev_s *dev)
{
  if (dev == NULL)
    {
      return -EINVAL;
    }

  /* Turn off LED before unregistering */

  ws2812_off(dev);

  kmm_free(dev);
  return OK;
}

/****************************************************************************
 * Name: ws2812_send_raw
 ****************************************************************************/

int ws2812_send_raw(FAR struct ws2812_dev_s *dev,
                    uint8_t b1, uint8_t b2, uint8_t b3)
{
  irqstate_t flags;

  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();

  ws2812_send_byte(dev, b1);
  ws2812_send_byte(dev, b2);
  ws2812_send_byte(dev, b3);

  WS2812_PIN_CLR(dev);
  leave_critical_section(flags);
  usleep(WS2812_RESET_US);

  return OK;
}

/****************************************************************************
 * Name: ws2812_set_color
 ****************************************************************************/

int ws2812_set_color(FAR struct ws2812_dev_s *dev,
                      const struct ws2812_color_s *color)
{
  if (dev == NULL || !dev->initialized || color == NULL)
    {
      return -EINVAL;
    }

  memcpy(&dev->current_color, color, sizeof(struct ws2812_color_s));

  return ws2812_send_raw(dev, color->g, color->r, color->b);
}

/****************************************************************************
 * Name: ws2812_set_rgb
 ****************************************************************************/

int ws2812_set_rgb(FAR struct ws2812_dev_s *dev,
                    uint8_t r, uint8_t g, uint8_t b)
{
  struct ws2812_color_s color;
  uint8_t br;

  if (dev == NULL)
    {
      return -EINVAL;
    }

  br = dev->brightness;
  if (br > 100)
    {
      br = 100;
    }

  color.r = (uint8_t)((uint16_t)r * br / 100);
  color.g = (uint8_t)((uint16_t)g * br / 100);
  color.b = (uint8_t)((uint16_t)b * br / 100);

  return ws2812_set_color(dev, &color);
}

/****************************************************************************
 * Name: ws2812_off
 ****************************************************************************/

int ws2812_off(FAR struct ws2812_dev_s *dev)
{
  struct ws2812_color_s black = {0, 0, 0};
  return ws2812_set_color(dev, &black);
}

/****************************************************************************
 * Name: ws2812_startup_indicator
 ****************************************************************************/

int ws2812_startup_indicator(FAR struct ws2812_dev_s *dev)
{
  return ws2812_blink(dev, 0, 255, 0, 80, 80, 3);
}

/****************************************************************************
 * Name: ws2812_set_brightness
 ****************************************************************************/

int ws2812_set_brightness(FAR struct ws2812_dev_s *dev, uint8_t pct)
{
  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

  if (pct > 100)
    {
      pct = 100;
    }

  dev->brightness = pct;
  return OK;
}

/****************************************************************************
 * Name: ws2812_blink
 ****************************************************************************/

int ws2812_blink(FAR struct ws2812_dev_s *dev,
                 uint8_t r, uint8_t g, uint8_t b,
                 uint32_t on_ms, uint32_t off_ms, int count)
{
  int i;

  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

  for (i = 0; i < count; i++)
    {
      ws2812_set_rgb(dev, r, g, b);
      usleep(on_ms * 1000);
      ws2812_off(dev);
      if (i < count - 1 || off_ms > 0)
        {
          usleep(off_ms * 1000);
        }
    }

  return OK;
}
