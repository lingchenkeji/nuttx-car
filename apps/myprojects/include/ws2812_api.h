/****************************************************************************
 * apps/myprojects/include/ws2812_api.h
 *
 * WS2812B RGB LED Driver API
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
 * 氛围灯 WS2812。主程序用它报告状态：
 * 正在初始化是琥珀色，模块都成功后慢闪绿色，有模块失败就快闪红色。
 * 亮不亮、有多亮，由参数 rgb_enable 和 rgb_brightness 决定。
 ****************************************************************************/


#ifndef __APPS_MYPROJECTS_INCLUDE_WS2812_API_H
#define __APPS_MYPROJECTS_INCLUDE_WS2812_API_H

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
 * Pre-processor Definitions
 ****************************************************************************/

/* WS2812B pin configuration (PB8 = TIM4 CH3) */

struct ws2812_pins_s
{
  uint32_t dinpin;     /* GPIO pin for WS2812B DIN (PB8 = TIM4 CH3) */
};

/* RGB color structure */

struct ws2812_color_s
{
  uint8_t r;           /* Red (0-255) */
  uint8_t g;           /* Green (0-255) */
  uint8_t b;           /* Blue (0-255) */
};

/* WS2812B device handle */

struct ws2812_dev_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ws2812_register
 *
 * Description:
 *   Register and initialize the WS2812B RGB LED driver.
 *
 * Input Parameters:
 *   pins - GPIO pin configuration (PB8 = TIM4 CH3)
 *
 * Returned Value:
 *   Device handle on success, NULL on failure.
 *
 ****************************************************************************/

EXTERN FAR struct ws2812_dev_s *ws2812_register(
  FAR const struct ws2812_pins_s *pins);

/****************************************************************************
 * Name: ws2812_unregister
 *
 * Description:
 *   Unregister the WS2812B driver and release resources.
 *
 ****************************************************************************/

EXTERN int ws2812_unregister(FAR struct ws2812_dev_s *dev);

/****************************************************************************
 * Name: ws2812_set_color
 *
 * Description:
 *   Set the RGB color of the WS2812B LED.
 *
 * Input Parameters:
 *   dev   - Device handle
 *   color - RGB color
 *
 ****************************************************************************/

EXTERN int ws2812_send_raw(FAR struct ws2812_dev_s *dev,
                            uint8_t b1, uint8_t b2, uint8_t b3);

EXTERN int ws2812_set_color(FAR struct ws2812_dev_s *dev,
                             const struct ws2812_color_s *color);

/****************************************************************************
 * Name: ws2812_set_rgb
 *
 * Description:
 *   Set the RGB color using individual R, G, B values.
 *
 * Input Parameters:
 *   dev - Device handle
 *   r   - Red value (0-255)
 *   g   - Green value (0-255)
 *   b   - Blue value (0-255)
 *
 ****************************************************************************/

EXTERN int ws2812_set_rgb(FAR struct ws2812_dev_s *dev,
                           uint8_t r, uint8_t g, uint8_t b);

/****************************************************************************
 * Name: ws2812_off
 *
 * Description:
 *   Turn off the LED (set to black/off).
 *
 ****************************************************************************/

EXTERN int ws2812_off(FAR struct ws2812_dev_s *dev);

/****************************************************************************
 * Name: ws2812_startup_indicator
 *
 * Description:
 *   Display a startup indicator sequence (green blink).
 *   This is a BLOCKING call.
 *
 * Input Parameters:
 *   dev - Device handle
 *
 ****************************************************************************/

EXTERN int ws2812_startup_indicator(FAR struct ws2812_dev_s *dev);

/****************************************************************************
 * Name: ws2812_set_brightness
 *
 * Description:
 *   Set global brightness (0-100%). Applied to subsequent set_color calls.
 *
 ****************************************************************************/

EXTERN int ws2812_set_brightness(FAR struct ws2812_dev_s *dev, uint8_t pct);

/****************************************************************************
 * Name: ws2812_blink
 *
 * Description:
 *   Blink with given RGB color. BLOCKING call.
 *   count<0 means infinite (not recommended).
 *
 ****************************************************************************/

EXTERN int ws2812_blink(FAR struct ws2812_dev_s *dev,
                        uint8_t r, uint8_t g, uint8_t b,
                        uint32_t on_ms, uint32_t off_ms, int count);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_MYPROJECTS_INCLUDE_WS2812_API_H */
