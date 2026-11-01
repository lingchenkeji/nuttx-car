/****************************************************************************
 * apps/myprojects/modules/motor_drv/drv8212_main.c
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
 * 电机的命令行试转入口。运行时的加减速在 project1_main.c 的主循环中。
 ****************************************************************************/


/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>

#ifdef CONFIG_NSH_LIBRARY
#  ifdef CONFIG_ARCH_CHIP_STM32
#    include <nuttx/arch.h>
#    include "stm32_gpio.h"

/* DRV8212 GPIO / Timer mapping:
 *   EN(nSLEEP) -> PA3  (GPIO push-pull, active-high enable)
 *   IN1        -> PA0  (TIM2_CH1, alternate-function push-pull for HW PWM)
 *   IN2        -> PA1  (TIM2_CH2, alternate-function push-pull for HW PWM)
 *
 * PA0/PA1 are configured as AFPP so TIM2 drives the PWM waveform directly.
 * No AFIO remap needed — default TIM2 mapping: CH1=PA0, CH2=PA1.
 */

#    define GPIO_DRV8212_IN1     (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz| \
                                  GPIO_PORTA|GPIO_PIN0)
#    define GPIO_DRV8212_IN2     (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz| \
                                  GPIO_PORTA|GPIO_PIN1)
#    define GPIO_DRV8212_EN      (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz| \
                                  GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN3)
#    define GPIO_DRV8212_DIR     (GPIO_INPUT|GPIO_CNF_INPULLUP|GPIO_MODE_INPUT| \
                                  GPIO_PORTA|GPIO_PIN2)
#  endif

#  include "drv8212_api.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct drv8212_dev_s *g_drv8212_dev;

static const struct drv8212_pins_s g_drv8212_pins =
{
  .in1pin = GPIO_DRV8212_IN1,
  .in2pin = GPIO_DRV8212_IN2,
  .nsleeppin = GPIO_DRV8212_EN,
  .dirpin = GPIO_DRV8212_DIR,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void drv8212_cmd_help(void)
{
  printf("\nUsage: drv8212 <command> [options]\n");
  printf("\nCommands:\n");
  printf("  init           - Initialize DRV8212 (auto-test fwd@50%%)\n");
  printf("  deinit         - Deinitialize DRV8212 motor driver\n");
  printf("  fwd            - Motor forward\n");
  printf("  rev            - Motor reverse\n");
  printf("  brake          - Motor brake\n");
  printf("  coast          - Motor coast (free run)\n");
  printf("  speed <0-100>  - Set PWM duty cycle (hardware TIM2 output)\n");
  printf("  sleep          - Enter low-power sleep mode (nSLEEP=0)\n");
  printf("  wakeup         - Exit sleep mode (nSLEEP=1)\n");
  printf("  status         - Print current driver state\n");
  printf("  dir            - Read direction switch and apply (PA2)\n");
  printf("  dump           - Dump TIM2/GPIO registers for debugging\n");
  printf("  help           - Show this help\n");
  printf("\n");
  printf("Hardware PWM: TIM2 CH1(PA0)=IN1, CH2(PA1)=IN2, 20kHz\n");
  printf("Truth table (nSLEEP=1): 00=coast 01=rev 10=fwd 11=brake\n");
  printf("Note: speed 100%% = constant HIGH (no waveform on scope!)\n");
  printf("      Use speed 10-90%% to see PWM waveform on oscilloscope.\n");
  printf("\n");
}

static int drv8212_check_inited(void)
{
  if (g_drv8212_dev == NULL)
    {
      printf("ERROR: DRV8212 not initialized. Use 'drv8212 init' first\n");
      return -ENODEV;
    }

  return OK;
}

static int drv8212_cmd_init(void)
{
  int ret;

  if (g_drv8212_dev != NULL)
    {
      printf("DRV8212 already initialized\n");
      return OK;
    }

  g_drv8212_dev = drv8212_register(&g_drv8212_pins);
  if (g_drv8212_dev == NULL)
    {
      printf("ERROR: Failed to register DRV8212\n");
      return ERROR;
    }

  printf("DRV8212 initialized OK\n");

  /* Auto-test: start forward at 50% so user can immediately see
   * a 20kHz 50% duty-cycle PWM waveform on the oscilloscope (PA0).
   */

  printf("DRV8212: Auto-test -> forward @ 50%% speed\n");
  ret = drv8212_set_speed(g_drv8212_dev, 50);
  if (ret < 0)
    {
      printf("  WARNING: set_speed failed: %d\n", ret);
    }

  ret = drv8212_forward(g_drv8212_dev);
  if (ret < 0)
    {
      printf("  WARNING: forward failed: %d\n", ret);
    }

  printf("DRV8212: Check PA0 with oscilloscope — expect 20kHz 50%% PWM\n");
  return OK;
}

static int drv8212_cmd_deinit(void)
{
  int ret;

  if (g_drv8212_dev == NULL)
    {
      printf("DRV8212 not initialized\n");
      return ERROR;
    }

  ret = drv8212_unregister(g_drv8212_dev);
  if (ret < 0)
    {
      printf("ERROR: drv8212_unregister failed: %d\n", ret);
      return ERROR;
    }

  g_drv8212_dev = NULL;
  printf("DRV8212 deinitialized\n");
  return OK;
}

static int drv8212_cmd_status(void)
{
  enum drv8212_mode_e mode;
  uint8_t speed;
  bool sleeping;
  int ret;
  FAR const char *mode_name = "unknown";

  ret = drv8212_check_inited();
  if (ret < 0)
    {
      return ERROR;
    }

  ret = drv8212_get_state(g_drv8212_dev, &mode, &speed, &sleeping);
  if (ret < 0)
    {
      printf("ERROR: drv8212_get_state failed: %d\n", ret);
      return ERROR;
    }

  switch (mode)
    {
      case DRV8212_MODE_COAST:
        mode_name = "coast";
        break;
      case DRV8212_MODE_FORWARD:
        mode_name = "forward";
        break;
      case DRV8212_MODE_REVERSE:
        mode_name = "reverse";
        break;
      case DRV8212_MODE_BRAKE:
        mode_name = "brake";
        break;
    }

  printf("DRV8212 state: mode=%s speed=%u%% sleeping=%s\n",
         mode_name, speed, sleeping ? "yes" : "no");
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int ret;

  if (argc < 2)
    {
      drv8212_cmd_help();
      return ERROR;
    }

  if (strcmp(argv[1], "init") == 0)
    {
      return drv8212_cmd_init();
    }
  else if (strcmp(argv[1], "deinit") == 0)
    {
      return drv8212_cmd_deinit();
    }
  else if (strcmp(argv[1], "fwd") == 0)
    {
      if (drv8212_check_inited() < 0)
        {
          return ERROR;
        }

      ret = drv8212_forward(g_drv8212_dev);
      if (ret < 0)
        {
          printf("ERROR: drv8212_forward failed: %d\n", ret);
          return ERROR;
        }
    }
  else if (strcmp(argv[1], "rev") == 0)
    {
      if (drv8212_check_inited() < 0)
        {
          return ERROR;
        }

      ret = drv8212_reverse(g_drv8212_dev);
      if (ret < 0)
        {
          printf("ERROR: drv8212_reverse failed: %d\n", ret);
          return ERROR;
        }
    }
  else if (strcmp(argv[1], "brake") == 0)
    {
      if (drv8212_check_inited() < 0)
        {
          return ERROR;
        }

      ret = drv8212_brake(g_drv8212_dev);
      if (ret < 0)
        {
          printf("ERROR: drv8212_brake failed: %d\n", ret);
          return ERROR;
        }
    }
  else if (strcmp(argv[1], "coast") == 0)
    {
      if (drv8212_check_inited() < 0)
        {
          return ERROR;
        }

      ret = drv8212_coast(g_drv8212_dev);
      if (ret < 0)
        {
          printf("ERROR: drv8212_coast failed: %d\n", ret);
          return ERROR;
        }
    }
  else if (strcmp(argv[1], "speed") == 0)
    {
      int speed;

      if (drv8212_check_inited() < 0)
        {
          return ERROR;
        }

      if (argc < 3)
        {
          printf("ERROR: Missing speed value, usage: drv8212 speed <0-100>\n");
          return ERROR;
        }

      speed = atoi(argv[2]);
      if (speed < 0)
        {
          speed = 0;
        }
      else if (speed > 100)
        {
          speed = 100;
        }

      ret = drv8212_set_speed(g_drv8212_dev, (uint8_t)speed);
      if (ret < 0)
        {
          printf("ERROR: drv8212_set_speed failed: %d\n", ret);
          return ERROR;
        }
    }
  else if (strcmp(argv[1], "sleep") == 0)
    {
      if (drv8212_check_inited() < 0)
        {
          return ERROR;
        }

      ret = drv8212_sleep(g_drv8212_dev);
      if (ret < 0)
        {
          printf("ERROR: drv8212_sleep failed: %d\n", ret);
          return ERROR;
        }
    }
  else if (strcmp(argv[1], "wakeup") == 0)
    {
      if (drv8212_check_inited() < 0)
        {
          return ERROR;
        }

      ret = drv8212_wakeup(g_drv8212_dev);
      if (ret < 0)
        {
          printf("ERROR: drv8212_wakeup failed: %d\n", ret);
          return ERROR;
        }
    }
  else if (strcmp(argv[1], "status") == 0)
    {
      return drv8212_cmd_status();
    }
  else if (strcmp(argv[1], "dir") == 0)
    {
      if (drv8212_check_inited() < 0)
        {
          return ERROR;
        }

      ret = drv8212_check_direction(g_drv8212_dev);
      if (ret < 0)
        {
          printf("ERROR: drv8212_check_direction failed: %d\n", ret);
          return ERROR;
        }

      printf("Direction switch: %s\n",
             ret == 1 ? "CHANGED" : "no change");
    }
  else if (strcmp(argv[1], "dump") == 0)
    {
      if (drv8212_check_inited() < 0)
        {
          return ERROR;
        }

      drv8212_debug_dump(g_drv8212_dev);
      return OK;
    }
  else if (strcmp(argv[1], "help") == 0)
    {
      drv8212_cmd_help();
      return OK;
    }
  else
    {
      printf("ERROR: Unknown command: %s\n", argv[1]);
      drv8212_cmd_help();
      return ERROR;
    }

  return drv8212_cmd_status();
}

#endif /* CONFIG_NSH_LIBRARY */
