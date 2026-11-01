/****************************************************************************
 * apps/myprojects/modules/func_led/func_led_api.c
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
 * 功能灯的开关。何时点亮、何时熄灭由 project1_main.c 决定。
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

#ifdef CONFIG_ARCH_CHIP_STM32
#  include <stm32_gpio.h>
#endif

#include "../../include/func_led_api.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct func_led_dev_s
{
  struct func_led_pins_s pins;
  bool is_on;
  bool initialized;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct func_led_dev_s *func_led_register(
  FAR const struct func_led_pins_s *pins)
{
  FAR struct func_led_dev_s *dev;

  if (pins == NULL)
    {
      return NULL;
    }

  dev = (FAR struct func_led_dev_s *)kmm_zalloc(
    sizeof(struct func_led_dev_s));
  if (dev == NULL)
    {
      printf("FUNC_LED: ERROR kmm_zalloc failed!\n");
      return NULL;
    }

  memcpy(&dev->pins, pins, sizeof(struct func_led_pins_s));

#ifdef CONFIG_ARCH_CHIP_STM32
  /* Configure LED pin as push-pull output, initial LOW (LED off) */

  stm32_configgpio(dev->pins.ledpin);
  stm32_gpiowrite(dev->pins.ledpin, false);
#else
  kmm_free(dev);
  return NULL;
#endif

  dev->is_on = false;
  dev->initialized = true;

  printf("FUNC_LED: registered (pin=0x%08lx)\n",
         (unsigned long)dev->pins.ledpin);
  return dev;
}

int func_led_unregister(FAR struct func_led_dev_s *dev)
{
  if (dev == NULL)
    {
      return -EINVAL;
    }

#ifdef CONFIG_ARCH_CHIP_STM32
  /* Turn LED off before unregistering */

  stm32_gpiowrite(dev->pins.ledpin, false);
#endif

  kmm_free(dev);
  return OK;
}

int func_led_on(FAR struct func_led_dev_s *dev)
{
  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

#ifdef CONFIG_ARCH_CHIP_STM32
  stm32_gpiowrite(dev->pins.ledpin, true);
#endif

  dev->is_on = true;
  return OK;
}

int func_led_off(FAR struct func_led_dev_s *dev)
{
  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

#ifdef CONFIG_ARCH_CHIP_STM32
  stm32_gpiowrite(dev->pins.ledpin, false);
#endif

  dev->is_on = false;
  return OK;
}

int func_led_toggle(FAR struct func_led_dev_s *dev)
{
  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

  if (dev->is_on)
    {
      return func_led_off(dev);
    }
  else
    {
      return func_led_on(dev);
    }
}

bool func_led_is_on(FAR struct func_led_dev_s *dev)
{
  if (dev == NULL || !dev->initialized)
    {
      return false;
    }

  return dev->is_on;
}

int func_led_blink(FAR struct func_led_dev_s *dev,
                    uint8_t count,
                    uint32_t on_ms,
                    uint32_t off_ms)
{
  int i;

  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

  for (i = 0; i < count; i++)
    {
      func_led_on(dev);
      usleep(on_ms * 1000u);

      func_led_off(dev);

      /* Don't delay after the last blink */

      if (i < count - 1)
        {
          usleep(off_ms * 1000u);
        }
    }

  return OK;
}

int func_led_timed_on(FAR struct func_led_dev_s *dev,
                       uint32_t duration_ms)
{
  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

  func_led_on(dev);
  usleep(duration_ms * 1000u);
  func_led_off(dev);

  return OK;
}
