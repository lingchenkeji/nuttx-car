/****************************************************************************
 * apps/myprojects/include/drv8212_api.h
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
 * 电机 H 桥 DRV8212。
 *
 * 速度用 0~100 的百分比，方向由方向开关决定。主循环大约每 500 毫秒
 * 调用一次 drv8212_check_direction()。
 * 全速键不走这个百分比，中断里直接把输出拉满；松手后恢复按百分比调速。
 ****************************************************************************/


#ifndef __APPS_MYPROJECTS_INCLUDE_DRV8212_API_H
#define __APPS_MYPROJECTS_INCLUDE_DRV8212_API_H

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

/* DRV8212 drive mode */

enum drv8212_mode_e
{
  DRV8212_MODE_COAST = 0, /* IN1=0, IN2=0 */
  DRV8212_MODE_FORWARD,   /* IN1=1, IN2=0 */
  DRV8212_MODE_REVERSE,   /* IN1=0, IN2=1 */
  DRV8212_MODE_BRAKE      /* IN1=1, IN2=1 */
};

/* DRV8212 GPIO pin configuration */

struct drv8212_pins_s
{
  uint32_t in1pin;     /* H-bridge input pin 1 */
  uint32_t in2pin;     /* H-bridge input pin 2 */
  uint32_t nsleeppin;  /* Sleep pin (optional, 0 means unused) */
  uint32_t dirpin;     /* Direction switch pin (optional, 0 means unused) */
};

/* DRV8212 device handle */

struct drv8212_dev_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

EXTERN FAR struct drv8212_dev_s *drv8212_register(
  FAR const struct drv8212_pins_s *pins);
EXTERN int drv8212_unregister(FAR struct drv8212_dev_s *dev);

EXTERN int drv8212_forward(FAR struct drv8212_dev_s *dev);
EXTERN int drv8212_reverse(FAR struct drv8212_dev_s *dev);
EXTERN int drv8212_brake(FAR struct drv8212_dev_s *dev);
EXTERN int drv8212_coast(FAR struct drv8212_dev_s *dev);

EXTERN int drv8212_set_speed(FAR struct drv8212_dev_s *dev,
                             uint8_t speed_percent);
EXTERN int drv8212_sleep(FAR struct drv8212_dev_s *dev);
EXTERN int drv8212_wakeup(FAR struct drv8212_dev_s *dev);
EXTERN int drv8212_get_state(FAR struct drv8212_dev_s *dev,
                             FAR enum drv8212_mode_e *mode,
                             FAR uint8_t *speed_percent,
                             FAR bool *sleeping);
EXTERN void drv8212_debug_dump(FAR struct drv8212_dev_s *dev);

/****************************************************************************
 * Name: drv8212_check_direction
 *
 * Description:
 *   Read the external direction switch (dirpin) and set motor direction.
 *   HIGH = forward, LOW = reverse.
 *   Designed to be called from a worker thread or periodic timer.
 *
 * Returned Value:
 *   OK on success, or negative errno on failure.
 *   Returns 1 if direction changed, 0 if unchanged.
 *
 ****************************************************************************/

EXTERN int drv8212_check_direction(FAR struct drv8212_dev_s *dev);

/****************************************************************************
 * Name: drv8212_force_fullspeed
 *
 * Description:
 *   Force constant DC output (no PWM switching) for maximum motor drive.
 *   Uses TIM "Force Active/Inactive" mode to guarantee true constant
 *   logic levels on IN1/IN2.  ISR-safe (register writes only).
 *   Call drv8212_set_speed() afterwards to return to normal PWM control.
 *
 ****************************************************************************/

EXTERN int drv8212_force_fullspeed(FAR struct drv8212_dev_s *dev);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_MYPROJECTS_INCLUDE_DRV8212_API_H */
