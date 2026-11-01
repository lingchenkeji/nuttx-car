/****************************************************************************
 * apps/myprojects/modules/vtx/vtx_main.c
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
 * 语音模块的命令行试播入口。小车正常播报不从这里进，
 * 从 project1_main.c 里读到卡内容之后调用 vtx_speak_fast()。
 ****************************************************************************/


/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#ifdef CONFIG_NSH_LIBRARY
#  include <nuttx/arch.h>

#  ifdef CONFIG_ARCH_CHIP_STM32
#    include "stm32_gpio.h"

/* VTX316 GPIO pin definitions */

#    define GPIO_VTX_RST    (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                             GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN8)
#    define GPIO_VTX_WP     (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                             GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN15)
#    define GPIO_VTX_BAUD0  (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                             GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN14)
#    define GPIO_VTX_BAUD1  (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                             GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN13)
#    define GPIO_VTX_BUSY   (GPIO_INPUT|GPIO_CNF_INFLOAT|\
                             GPIO_PORTB|GPIO_PIN12)
#  endif

#  include "vtx_api.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VTX_DEFAULT_DEVPATH  "/dev/ttyS0"

/****************************************************************************
 * Pre-defined Chinese Phrases (GBK encoded)
 ****************************************************************************/

struct vtx_phrase_s
{
  int id;
  FAR const char *desc;
  FAR const char *text;
};

/* GBK encoded Chinese phrases - VTX316 uses GBK (0x01) encoding */

static const struct vtx_phrase_s g_vtx_phrases[] =
{
  /* Common greetings */

  {1,  "ni hao (Hello)",
   "\xC4\xE3\xBA\xC3"},                                        /* 你好 */
  {2,  "huan ying guang lin (Welcome)",
   "\xBB\xB6\xD3\xAD\xB9\xE2\xC1\xD9"},                      /* 欢迎光临 */
  {3,  "xie xie (Thank you)",
   "\xD0\xBB\xD0\xBB"},                                        /* 谢谢 */
  {4,  "zai jian (Goodbye)",
   "\xD4\xD9\xBC\xFB"},                                        /* 再见 */

  /* Card operations */

  {5,  "qing shua ka (Please swipe card)",
   "\xC7\xEB\xCB\xA2\xBF\xA8"},                                /* 请刷卡 */
  {6,  "shua ka cheng gong (Card swipe OK)",
   "\xCB\xA2\xBF\xA8\xB3\xC9\xB9\xA6"},                      /* 刷卡成功 */
  {7,  "shua ka shi bai (Card swipe fail)",
   "\xCB\xA2\xBF\xA8\xCA\xA7\xB0\xDC"},                      /* 刷卡失败 */

  /* Status messages */

  {8,  "cao zuo cheng gong (Operation OK)",
   "\xB2\xD9\xD7\xF7\xB3\xC9\xB9\xA6"},                      /* 操作成功 */
  {9,  "cao zuo shi bai (Operation fail)",
   "\xB2\xD9\xD7\xF7\xCA\xA7\xB0\xDC"},                      /* 操作失败 */
  {10, "qing shao hou (Please wait)",
   "\xC7\xEB\xC9\xD4\xBA\xF2"},                                /* 请稍候 */

  /* System messages */

  {11, "xi tong qi dong (System startup)",
   "\xCF\xB5\xCD\xB3\xC6\xF4\xB6\xAF"},                      /* 系统启动 */
  {12, "yu yin bo bao ce shi zheng chang "
       "(Voice test normal)",
   "\xD3\xEF\xD2\xF4\xB2\xA5\xB1\xA8"
   "\xB2\xE2\xCA\xD4\xD5\xFD\xB3\xA3"},                      /* 语音播报测试正常 */
  {13, "qing shu ru mi ma (Please enter password)",
   "\xC7\xEB\xCA\xE4\xC8\xEB\xC3\xDC\xC2\xEB"},              /* 请输入密码 */
  {14, "wen du guo gao qing zhu yi "
       "(Temperature too high)",
   "\xCE\xC2\xB6\xC8\xB9\xFD\xB8\xDF"
   "\xC7\xEB\xD7\xA2\xD2\xE2"},                                /* 温度过高请注意 */
  {15, "dian liang bu zu (Low battery)",
   "\xB5\xE7\xC1\xBF\xB2\xBB\xD7\xE3"},                      /* 电量不足 */
};

#define VTX_NUM_PHRASES  (sizeof(g_vtx_phrases) / sizeof(g_vtx_phrases[0]))

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct vtx_dev_s *g_vtx_dev = NULL;

static const struct vtx_pins_s g_vtx_pins =
{
  .rstpin   = GPIO_VTX_RST,
  .wppin    = GPIO_VTX_WP,
  .baud0pin = GPIO_VTX_BAUD0,
  .baud1pin = GPIO_VTX_BAUD1,
  .busypin  = GPIO_VTX_BUSY,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vtx_hex_to_bytes
 *
 * Description:
 *   Convert a hex string to binary bytes.
 *   e.g., "C4E3BAC3" -> {0xC4, 0xE3, 0xBA, 0xC3}
 *
 ****************************************************************************/

static int vtx_hex_to_bytes(FAR const char *hex,
                             FAR uint8_t *out, int maxlen)
{
  int hexlen = strlen(hex);
  int i;
  int outlen = 0;

  if (hexlen % 2 != 0)
    {
      return -1;
    }

  for (i = 0; i < hexlen && outlen < maxlen; i += 2)
    {
      char bs[3];
      char *endp;
      unsigned long val;

      bs[0] = hex[i];
      bs[1] = hex[i + 1];
      bs[2] = '\0';

      val = strtoul(bs, &endp, 16);
      if (*endp != '\0' || val > 0xff)
        {
          return -1;
        }

      out[outlen++] = (uint8_t)val;
    }

  return outlen;
}

/****************************************************************************
 * Name: vtx_cmd_help
 ****************************************************************************/

static void vtx_cmd_help(void)
{
  printf("\nUsage: vtx <command> [options]\n");
  printf("\nCommands:\n");
  printf("  init [devpath]   - Initialize VTX316 device\n");
  printf("                     devpath: UART device (default: %s)\n",
         VTX_DEFAULT_DEVPATH);
  printf("  deinit           - Deinitialize VTX316 device\n");
  printf("  say <text...>    - Speak text (GBK encoding)\n");
  printf("  hex <hexdata>    - Speak GBK hex bytes\n");
  printf("                     e.g: vtx hex C4E3BAC3 (= ni hao)\n");
  printf("  phrase <id>      - Speak predefined Chinese phrase\n");
  printf("  phrase list      - List all predefined phrases\n");
  printf("  stop             - Stop current speech\n");
  printf("  pause            - Pause current speech\n");
  printf("  resume           - Resume paused speech\n");
  printf("  status           - Query chip status\n");
  printf("  busy             - Check busy pin status\n");
  printf("  reset            - Hardware reset chip\n");
  printf("  sleep            - Enter deep sleep mode\n");
  printf("  wakeup           - Wake up from deep sleep\n");
  printf("  volume <0-9>     - Set volume level\n");
  printf("  speed <0-29>     - Set speech speed\n");
  printf("  speaker <id>     - Set speaker voice\n");
  printf("  test             - Run quick self-test\n");
  printf("  help             - Show this help message\n");
  printf("\nNotes:\n");
  printf("  - Use 'vtx phrase list' to see Chinese phrases\n");
  printf("  - Use 'vtx hex' to send GBK hex for Chinese\n");
  printf("  - Control tags: [n1]num [v5] [s10] [p200]\n");
  printf("  - English is read letter by letter (chip design)\n");
  printf("\n");
}

/****************************************************************************
 * Name: vtx_cmd_init
 ****************************************************************************/

static int vtx_cmd_init(int argc, FAR char *argv[])
{
  FAR const char *devpath = VTX_DEFAULT_DEVPATH;

  if (g_vtx_dev != NULL)
    {
      printf("VTX device already initialized\n");
      return OK;
    }

  if (argc > 1)
    {
      devpath = argv[1];
    }

  printf("Initializing VTX316 on %s\n", devpath);

  g_vtx_dev = vtx_register(devpath, &g_vtx_pins, VTX_BAUDRATE_115200);

  if (g_vtx_dev == NULL)
    {
      printf("ERROR: Failed to register VTX316 device\n");
      return ERROR;
    }

  printf("VTX316 device initialized successfully\n");
  return OK;
}

/****************************************************************************
 * Name: vtx_cmd_deinit
 ****************************************************************************/

static int vtx_cmd_deinit(int argc, FAR char *argv[])
{
  if (g_vtx_dev == NULL)
    {
      printf("VTX device not initialized\n");
      return ERROR;
    }

  vtx_unregister(g_vtx_dev);
  g_vtx_dev = NULL;

  printf("VTX316 device deinitialized\n");
  return OK;
}

/****************************************************************************
 * Name: vtx_cmd_say
 *
 * Description:
 *   Speak text with UTF-8 encoding (supports Chinese if terminal is UTF-8).
 *   Concatenates all arguments with spaces so multi-word text works.
 *   Control tags like [n1], [v5], [s10], [p200] can be included in text.
 *
 ****************************************************************************/

static int vtx_cmd_say(int argc, FAR char *argv[])
{
  char textbuf[128];
  int offset = 0;
  int i;
  int ret;

  if (g_vtx_dev == NULL)
    {
      printf("ERROR: VTX device not initialized. Use 'vtx init' first\n");
      return ERROR;
    }

  if (argc < 2)
    {
      printf("ERROR: Missing text. Usage: vtx say <text...>\n");
      return ERROR;
    }

  /* Concatenate all arguments after "say" with spaces */

  for (i = 1; i < argc && offset < (int)sizeof(textbuf) - 1; i++)
    {
      int len = strlen(argv[i]);

      if (i > 1 && offset < (int)sizeof(textbuf) - 1)
        {
          textbuf[offset++] = ' ';
        }

      if (offset + len >= (int)sizeof(textbuf))
        {
          len = sizeof(textbuf) - 1 - offset;
        }

      memcpy(&textbuf[offset], argv[i], len);
      offset += len;
    }

  textbuf[offset] = '\0';

  printf("Speaking (%d bytes, GBK): %s\n", offset, textbuf);

  /* Use GBK encoding (VTX316 chip works reliably with GBK) */

  ret = vtx_speak_and_wait(g_vtx_dev, textbuf,
                            VTX_ENCODING_GBK, 30000);
  if (ret < 0)
    {
      printf("ERROR: vtx_speak_and_wait failed: %d\n", ret);
      return ERROR;
    }

  printf("Speech complete\n");
  return OK;
}

/****************************************************************************
 * Name: vtx_cmd_hex
 *
 * Description:
 *   Send raw hex-encoded bytes to VTX316 as GBK text.
 *   Useful when terminal cannot input Chinese directly.
 *
 *   Example: vtx hex C4E3BAC3  (GBK for "你好")
 *
 ****************************************************************************/

static int vtx_cmd_hex(int argc, FAR char *argv[])
{
  uint8_t binbuf[128];
  int binlen;
  int ret;

  if (g_vtx_dev == NULL)
    {
      printf("ERROR: VTX device not initialized. Use 'vtx init' first\n");
      return ERROR;
    }

  if (argc < 2)
    {
      printf("ERROR: Missing hex data.\n");
      printf("Usage: vtx hex <hex_bytes>\n");
      printf("Example: vtx hex C4E3BAC3  (GBK for 'ni hao')\n");
      return ERROR;
    }

  binlen = vtx_hex_to_bytes(argv[1], binbuf, sizeof(binbuf) - 1);
  if (binlen <= 0)
    {
      printf("ERROR: Invalid hex string (must be even-length hex)\n");
      return ERROR;
    }

  /* Null-terminate for vtx_speak (GBK doesn't contain 0x00) */

  binbuf[binlen] = '\0';

  printf("Sending %d bytes (GBK hex)...\n", binlen);

  ret = vtx_speak_and_wait(g_vtx_dev, (FAR const char *)binbuf,
                            VTX_ENCODING_GBK, 30000);
  if (ret < 0)
    {
      printf("ERROR: vtx_speak_and_wait failed: %d\n", ret);
      return ERROR;
    }

  printf("Speech complete\n");
  return OK;
}

/****************************************************************************
 * Name: vtx_cmd_phrase
 *
 * Description:
 *   Speak a predefined Chinese phrase by ID.
 *   Use "vtx phrase list" to show all available phrases.
 *
 ****************************************************************************/

static int vtx_cmd_phrase(int argc, FAR char *argv[])
{
  int id;
  int i;
  int ret;

  if (g_vtx_dev == NULL)
    {
      printf("ERROR: VTX device not initialized. Use 'vtx init' first\n");
      return ERROR;
    }

  if (argc < 2)
    {
      printf("ERROR: Missing phrase ID. Usage: vtx phrase <id|list>\n");
      return ERROR;
    }

  /* Handle "list" sub-command */

  if (strcmp(argv[1], "list") == 0)
    {
      printf("\nPredefined Chinese Phrases:\n");
      printf("%-4s  %s\n", "ID", "Description");
      printf("----  -----------\n");

      for (i = 0; i < (int)VTX_NUM_PHRASES; i++)
        {
          printf("%-4d  %s\n",
                 g_vtx_phrases[i].id,
                 g_vtx_phrases[i].desc);
        }

      printf("\nUsage: vtx phrase <id>\n\n");
      return OK;
    }

  /* Parse phrase ID */

  id = atoi(argv[1]);

  for (i = 0; i < (int)VTX_NUM_PHRASES; i++)
    {
      if (g_vtx_phrases[i].id == id)
        {
          printf("Phrase %d: %s\n",
                 id, g_vtx_phrases[i].desc);

          ret = vtx_speak_and_wait(g_vtx_dev,
                                    g_vtx_phrases[i].text,
                                    VTX_ENCODING_GBK, 30000);
          if (ret < 0)
            {
              printf("ERROR: vtx_speak_and_wait failed: %d\n", ret);
              return ERROR;
            }

          printf("Speech complete\n");
          return OK;
        }
    }

  printf("ERROR: Unknown phrase ID %d. Use 'vtx phrase list'\n", id);
  return ERROR;
}

/****************************************************************************
 * Name: vtx_cmd_stop
 ****************************************************************************/

static int vtx_cmd_stop(int argc, FAR char *argv[])
{
  if (g_vtx_dev == NULL)
    {
      printf("ERROR: VTX device not initialized. Use 'vtx init' first\n");
      return ERROR;
    }

  return vtx_stop(g_vtx_dev);
}

/****************************************************************************
 * Name: vtx_cmd_pause
 ****************************************************************************/

static int vtx_cmd_pause(int argc, FAR char *argv[])
{
  if (g_vtx_dev == NULL)
    {
      printf("ERROR: VTX device not initialized. Use 'vtx init' first\n");
      return ERROR;
    }

  return vtx_pause(g_vtx_dev);
}

/****************************************************************************
 * Name: vtx_cmd_resume
 ****************************************************************************/

static int vtx_cmd_resume(int argc, FAR char *argv[])
{
  if (g_vtx_dev == NULL)
    {
      printf("ERROR: VTX device not initialized. Use 'vtx init' first\n");
      return ERROR;
    }

  return vtx_resume(g_vtx_dev);
}

/****************************************************************************
 * Name: vtx_cmd_status
 ****************************************************************************/

static int vtx_cmd_status(int argc, FAR char *argv[])
{
  int status;

  if (g_vtx_dev == NULL)
    {
      printf("ERROR: VTX device not initialized. Use 'vtx init' first\n");
      return ERROR;
    }

  status = vtx_query_status(g_vtx_dev);
  if (status < 0)
    {
      printf("ERROR: Failed to query status: %d\n", status);
      return ERROR;
    }

  switch (status)
    {
      case VTX_STATUS_IDLE:
        printf("VTX316 Status: IDLE\n");
        break;
      case VTX_STATUS_BUSY:
        printf("VTX316 Status: BUSY (speaking)\n");
        break;
      case VTX_STATUS_ERROR:
        printf("VTX316 Status: ERROR\n");
        break;
      default:
        printf("VTX316 Status: UNKNOWN (0x%02x)\n", status);
        break;
    }

  return OK;
}

/****************************************************************************
 * Name: vtx_cmd_busy
 ****************************************************************************/

static int vtx_cmd_busy(int argc, FAR char *argv[])
{
  if (g_vtx_dev == NULL)
    {
      printf("ERROR: VTX device not initialized. Use 'vtx init' first\n");
      return ERROR;
    }

  printf("VTX316 R/B pin: %s\n",
         vtx_is_busy(g_vtx_dev) ? "BUSY" : "IDLE");
  return OK;
}

/****************************************************************************
 * Name: vtx_cmd_reset
 ****************************************************************************/

static int vtx_cmd_reset(int argc, FAR char *argv[])
{
  if (g_vtx_dev == NULL)
    {
      printf("ERROR: VTX device not initialized. Use 'vtx init' first\n");
      return ERROR;
    }

  printf("Resetting VTX316...\n");
  return vtx_reset(g_vtx_dev);
}

/****************************************************************************
 * Name: vtx_cmd_sleep
 ****************************************************************************/

static int vtx_cmd_sleep(int argc, FAR char *argv[])
{
  if (g_vtx_dev == NULL)
    {
      printf("ERROR: VTX device not initialized. Use 'vtx init' first\n");
      return ERROR;
    }

  printf("Entering deep sleep...\n");
  return vtx_sleep(g_vtx_dev);
}

/****************************************************************************
 * Name: vtx_cmd_wakeup
 ****************************************************************************/

static int vtx_cmd_wakeup(int argc, FAR char *argv[])
{
  if (g_vtx_dev == NULL)
    {
      printf("ERROR: VTX device not initialized. Use 'vtx init' first\n");
      return ERROR;
    }

  printf("Waking up from deep sleep...\n");
  return vtx_wakeup(g_vtx_dev);
}

/****************************************************************************
 * Name: vtx_cmd_volume
 ****************************************************************************/

static int vtx_cmd_volume(int argc, FAR char *argv[])
{
  int vol;

  if (g_vtx_dev == NULL)
    {
      printf("ERROR: VTX device not initialized. Use 'vtx init' first\n");
      return ERROR;
    }

  if (argc < 2)
    {
      printf("ERROR: Missing volume. Usage: vtx volume <0-9>\n");
      return ERROR;
    }

  vol = atoi(argv[1]);
  printf("Setting volume to %d\n", vol);
  return vtx_set_volume(g_vtx_dev, (uint8_t)vol);
}

/****************************************************************************
 * Name: vtx_cmd_speed
 ****************************************************************************/

static int vtx_cmd_speed(int argc, FAR char *argv[])
{
  int spd;

  if (g_vtx_dev == NULL)
    {
      printf("ERROR: VTX device not initialized. Use 'vtx init' first\n");
      return ERROR;
    }

  if (argc < 2)
    {
      printf("ERROR: Missing speed. Usage: vtx speed <0-29>\n");
      return ERROR;
    }

  spd = atoi(argv[1]);
  printf("Setting speed to %d\n", spd);
  return vtx_set_speed(g_vtx_dev, (uint8_t)spd);
}

/****************************************************************************
 * Name: vtx_cmd_speaker
 ****************************************************************************/

static int vtx_cmd_speaker(int argc, FAR char *argv[])
{
  int id;

  if (g_vtx_dev == NULL)
    {
      printf("ERROR: VTX device not initialized. Use 'vtx init' first\n");
      return ERROR;
    }

  if (argc < 2)
    {
      printf("ERROR: Missing speaker ID. Usage: vtx speaker <id>\n");
      printf("  Available speakers:\n");
      printf("    3  - XiaoLing (female)\n");
      printf("    51 - Yin XiaoJian (male)\n");
      printf("    52 - Yi XiaoQiang (male)\n");
      printf("    53 - Tian BeiBei (female)\n");
      printf("    54 - Donald Duck (effect)\n");
      printf("    55 - Xiao YanZi (girl)\n");
      printf("    56 - Bei Tong (boy)\n");
      printf("    57 - Xiao Ke (boy)\n");
      return ERROR;
    }

  id = atoi(argv[1]);
  printf("Setting speaker to %d\n", id);
  return vtx_set_speaker(g_vtx_dev, (uint8_t)id);
}

/****************************************************************************
 * Name: vtx_cmd_test
 ****************************************************************************/

static int vtx_cmd_test(int argc, FAR char *argv[])
{
  int ret;

  if (g_vtx_dev == NULL)
    {
      printf("ERROR: VTX device not initialized. Use 'vtx init' first\n");
      return ERROR;
    }

  printf("=== VTX316 Self-Test ===\n");

  /* Test 1: Check busy pin */

  printf("[1] Busy pin: %s\n",
         vtx_is_busy(g_vtx_dev) ? "BUSY" : "IDLE");

  /* Test 2: Query status */

  ret = vtx_query_status(g_vtx_dev);
  printf("[2] Query status: %d\n", ret);

  /* Test 3: Speak Chinese test phrase */

  printf("[3] Speaking Chinese test phrase...\n");

  /* GBK: "语音播报测试正常" */

  ret = vtx_speak_and_wait(g_vtx_dev,
                            "\xD3\xEF\xD2\xF4\xB2\xA5\xB1\xA8"
                            "\xB2\xE2\xCA\xD4\xD5\xFD\xB3\xA3",
                            VTX_ENCODING_GBK, 10000);
  printf("[3] Speak result: %d\n", ret);

  printf("=== Test Complete ===\n");
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: main (vtx_main)
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  if (argc < 2)
    {
      vtx_cmd_help();
      return ERROR;
    }

  if (strcmp(argv[1], "init") == 0)
    {
      return vtx_cmd_init(argc - 1, &argv[1]);
    }
  else if (strcmp(argv[1], "deinit") == 0)
    {
      return vtx_cmd_deinit(argc - 1, &argv[1]);
    }
  else if (strcmp(argv[1], "say") == 0)
    {
      return vtx_cmd_say(argc - 1, &argv[1]);
    }
  else if (strcmp(argv[1], "hex") == 0)
    {
      return vtx_cmd_hex(argc - 1, &argv[1]);
    }
  else if (strcmp(argv[1], "phrase") == 0)
    {
      return vtx_cmd_phrase(argc - 1, &argv[1]);
    }
  else if (strcmp(argv[1], "stop") == 0)
    {
      return vtx_cmd_stop(argc - 1, &argv[1]);
    }
  else if (strcmp(argv[1], "pause") == 0)
    {
      return vtx_cmd_pause(argc - 1, &argv[1]);
    }
  else if (strcmp(argv[1], "resume") == 0)
    {
      return vtx_cmd_resume(argc - 1, &argv[1]);
    }
  else if (strcmp(argv[1], "status") == 0)
    {
      return vtx_cmd_status(argc - 1, &argv[1]);
    }
  else if (strcmp(argv[1], "busy") == 0)
    {
      return vtx_cmd_busy(argc - 1, &argv[1]);
    }
  else if (strcmp(argv[1], "reset") == 0)
    {
      return vtx_cmd_reset(argc - 1, &argv[1]);
    }
  else if (strcmp(argv[1], "sleep") == 0)
    {
      return vtx_cmd_sleep(argc - 1, &argv[1]);
    }
  else if (strcmp(argv[1], "wakeup") == 0)
    {
      return vtx_cmd_wakeup(argc - 1, &argv[1]);
    }
  else if (strcmp(argv[1], "volume") == 0)
    {
      return vtx_cmd_volume(argc - 1, &argv[1]);
    }
  else if (strcmp(argv[1], "speed") == 0)
    {
      return vtx_cmd_speed(argc - 1, &argv[1]);
    }
  else if (strcmp(argv[1], "speaker") == 0)
    {
      return vtx_cmd_speaker(argc - 1, &argv[1]);
    }
  else if (strcmp(argv[1], "test") == 0)
    {
      return vtx_cmd_test(argc - 1, &argv[1]);
    }
  else if (strcmp(argv[1], "help") == 0)
    {
      vtx_cmd_help();
      return OK;
    }
  else
    {
      printf("ERROR: Unknown command: %s\n", argv[1]);
      vtx_cmd_help();
      return ERROR;
    }
}

#endif /* CONFIG_NSH_LIBRARY */
