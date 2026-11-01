/****************************************************************************
 * apps/myprojects/include/func_led_api.h
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
 * 功能灯。读到卡并且这次允许亮灯时点亮，主循环按 led_time 熄灭。
 * 它不是氛围灯。氛围灯在 ws2812_api.h，用来表示开机成功或故障。
 ****************************************************************************/


#ifndef __APPS_MYPROJECTS_INCLUDE_FUNC_LED_API_H
#define __APPS_MYPROJECTS_INCLUDE_FUNC_LED_API_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Functional LED pin configuration */

struct func_led_pins_s
{
  uint32_t ledpin;     /* GPIO pin driving the S8050 base (active-high) */
};

/* Functional LED device handle */

struct func_led_dev_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: func_led_register
 *
 * Description:
 *   Register and initialize the functional LED driver.
 *
 * Input Parameters:
 *   pins - GPIO pin configuration
 *
 * Returned Value:
 *   Device handle on success, NULL on failure.
 *
 ****************************************************************************/

EXTERN FAR struct func_led_dev_s *func_led_register(
  FAR const struct func_led_pins_s *pins);

/****************************************************************************
 * Name: func_led_unregister
 *
 * Description:
 *   Unregister the functional LED driver and release resources.
 *
 ****************************************************************************/

EXTERN int func_led_unregister(FAR struct func_led_dev_s *dev);

/****************************************************************************
 * Name: func_led_on
 *
 * Description:
 *   Turn the LED on (GPIO HIGH -> S8050 conducts -> LED lights up).
 *
 ****************************************************************************/

EXTERN int func_led_on(FAR struct func_led_dev_s *dev);

/****************************************************************************
 * Name: func_led_off
 *
 * Description:
 *   Turn the LED off (GPIO LOW -> S8050 off -> LED off).
 *
 ****************************************************************************/

EXTERN int func_led_off(FAR struct func_led_dev_s *dev);

/****************************************************************************
 * Name: func_led_toggle
 *
 * Description:
 *   Toggle the LED state.
 *
 ****************************************************************************/

EXTERN int func_led_toggle(FAR struct func_led_dev_s *dev);

/****************************************************************************
 * Name: func_led_is_on
 *
 * Description:
 *   Query current LED state.
 *
 * Returned Value:
 *   true if LED is on, false if off.
 *
 ****************************************************************************/

EXTERN bool func_led_is_on(FAR struct func_led_dev_s *dev);

/****************************************************************************
 * Name: func_led_blink
 *
 * Description:
 *   Blink the LED for a specified number of times.
 *   This is a BLOCKING call (uses usleep internally).
 *
 * Input Parameters:
 *   dev       - Device handle
 *   count     - Number of blink cycles
 *   on_ms     - On duration per cycle in milliseconds
 *   off_ms    - Off duration per cycle in milliseconds
 *
 ****************************************************************************/

EXTERN int func_led_blink(FAR struct func_led_dev_s *dev,
                           uint8_t count,
                           uint32_t on_ms,
                           uint32_t off_ms);

/****************************************************************************
 * Name: func_led_timed_on
 *
 * Description:
 *   Turn LED on for a specified duration, then turn off automatically.
 *   This is a BLOCKING call (uses usleep internally).
 *   Ideal for simple notification scenarios like "RFID detected → LED on
 *   for 3 seconds → LED off".
 *
 *   For non-blocking usage in threads, call func_led_on() and schedule
 *   func_led_off() via your own timer/thread instead.
 *
 * Input Parameters:
 *   dev       - Device handle
 *   duration_ms - Duration in milliseconds to keep LED on
 *
 ****************************************************************************/

EXTERN int func_led_timed_on(FAR struct func_led_dev_s *dev,
                              uint32_t duration_ms);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_MYPROJECTS_INCLUDE_FUNC_LED_API_H */
