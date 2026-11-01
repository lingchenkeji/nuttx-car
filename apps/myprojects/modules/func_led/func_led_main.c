/****************************************************************************
 * apps/myprojects/modules/func_led/func_led_main.c
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
 * 功能灯的命令行试灯入口。小车运行时不从这里控制灯。
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
#    include "stm32_gpio.h"

/* Functional LED GPIO mapping:
 *   LED -> PB11  (GPIO push-pull output, active-high)
 *   PB11 -> S8050 NPN base -> LED (5V)
 *   HIGH = LED ON, LOW = LED OFF
 */

#    define GPIO_FUNC_LED   (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz| \
                             GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN11)
#  endif

#  include "func_led_api.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct func_led_dev_s *g_func_led_dev;

static const struct func_led_pins_s g_func_led_pins =
{
  .ledpin = GPIO_FUNC_LED,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void func_led_cmd_help(void)
{
  printf("\nUsage: func_led <command> [options]\n");
  printf("\nCommands:\n");
  printf("  init              - Initialize functional LED driver\n");
  printf("  deinit            - Deinitialize and release LED\n");
  printf("  on                - Turn LED on\n");
  printf("  off               - Turn LED off\n");
  printf("  toggle            - Toggle LED state\n");
  printf("  blink [N] [on] [off] - Blink N times (on/off in ms)\n");
  printf("                        default: 3 times, 500ms on, 500ms off\n");
  printf("  timed <ms>        - Turn on for <ms> milliseconds then off\n");
  printf("  status            - Show current LED state\n");
  printf("  help              - Show this help\n");
  printf("\n");
  printf("Hardware: PB11 -> S8050 NPN -> 5V LED (active-high)\n");
  printf("\n");
}

static int func_led_check_inited(void)
{
  if (g_func_led_dev == NULL)
    {
      printf("ERROR: func_led not initialized. "
             "Use 'func_led init' first\n");
      return -ENODEV;
    }

  return OK;
}

static int func_led_cmd_init(void)
{
  if (g_func_led_dev != NULL)
    {
      printf("func_led already initialized\n");
      return OK;
    }

  g_func_led_dev = func_led_register(&g_func_led_pins);
  if (g_func_led_dev == NULL)
    {
      printf("ERROR: Failed to register func_led\n");
      return ERROR;
    }

  printf("func_led initialized (PB11, active-high via S8050)\n");
  return OK;
}

static int func_led_cmd_deinit(void)
{
  int ret;

  if (g_func_led_dev == NULL)
    {
      printf("func_led not initialized\n");
      return ERROR;
    }

  ret = func_led_unregister(g_func_led_dev);
  if (ret < 0)
    {
      printf("ERROR: func_led_unregister failed: %d\n", ret);
      return ERROR;
    }

  g_func_led_dev = NULL;
  printf("func_led deinitialized\n");
  return OK;
}

static int func_led_cmd_status(void)
{
  if (func_led_check_inited() < 0)
    {
      return ERROR;
    }

  printf("func_led state: %s\n",
         func_led_is_on(g_func_led_dev) ? "ON" : "OFF");
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
      func_led_cmd_help();
      return ERROR;
    }

  if (strcmp(argv[1], "init") == 0)
    {
      return func_led_cmd_init();
    }
  else if (strcmp(argv[1], "deinit") == 0)
    {
      return func_led_cmd_deinit();
    }
  else if (strcmp(argv[1], "on") == 0)
    {
      if (func_led_check_inited() < 0)
        {
          return ERROR;
        }

      ret = func_led_on(g_func_led_dev);
      if (ret < 0)
        {
          printf("ERROR: func_led_on failed: %d\n", ret);
          return ERROR;
        }

      printf("func_led: ON\n");
    }
  else if (strcmp(argv[1], "off") == 0)
    {
      if (func_led_check_inited() < 0)
        {
          return ERROR;
        }

      ret = func_led_off(g_func_led_dev);
      if (ret < 0)
        {
          printf("ERROR: func_led_off failed: %d\n", ret);
          return ERROR;
        }

      printf("func_led: OFF\n");
    }
  else if (strcmp(argv[1], "toggle") == 0)
    {
      if (func_led_check_inited() < 0)
        {
          return ERROR;
        }

      ret = func_led_toggle(g_func_led_dev);
      if (ret < 0)
        {
          printf("ERROR: func_led_toggle failed: %d\n", ret);
          return ERROR;
        }

      printf("func_led: %s\n",
             func_led_is_on(g_func_led_dev) ? "ON" : "OFF");
    }
  else if (strcmp(argv[1], "blink") == 0)
    {
      uint8_t count = 3;
      uint32_t on_ms = 500;
      uint32_t off_ms = 500;

      if (func_led_check_inited() < 0)
        {
          return ERROR;
        }

      if (argc >= 3)
        {
          count = (uint8_t)atoi(argv[2]);
        }

      if (argc >= 4)
        {
          on_ms = (uint32_t)atoi(argv[3]);
        }

      if (argc >= 5)
        {
          off_ms = (uint32_t)atoi(argv[4]);
        }

      printf("func_led: blinking %u times (%lums on, %lums off)\n",
             count, (unsigned long)on_ms, (unsigned long)off_ms);

      ret = func_led_blink(g_func_led_dev, count, on_ms, off_ms);
      if (ret < 0)
        {
          printf("ERROR: func_led_blink failed: %d\n", ret);
          return ERROR;
        }

      printf("func_led: blink done\n");
    }
  else if (strcmp(argv[1], "timed") == 0)
    {
      uint32_t duration_ms;

      if (func_led_check_inited() < 0)
        {
          return ERROR;
        }

      if (argc < 3)
        {
          printf("ERROR: Missing duration. "
                 "Usage: func_led timed <ms>\n");
          return ERROR;
        }

      duration_ms = (uint32_t)atoi(argv[2]);

      printf("func_led: ON for %lums...\n", (unsigned long)duration_ms);

      ret = func_led_timed_on(g_func_led_dev, duration_ms);
      if (ret < 0)
        {
          printf("ERROR: func_led_timed_on failed: %d\n", ret);
          return ERROR;
        }

      printf("func_led: OFF (timed done)\n");
    }
  else if (strcmp(argv[1], "status") == 0)
    {
      return func_led_cmd_status();
    }
  else if (strcmp(argv[1], "help") == 0)
    {
      func_led_cmd_help();
      return OK;
    }
  else
    {
      printf("ERROR: Unknown command: %s\n", argv[1]);
      func_led_cmd_help();
      return ERROR;
    }

  return OK;
}

#endif /* CONFIG_NSH_LIBRARY */
