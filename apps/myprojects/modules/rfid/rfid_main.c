/****************************************************************************
 * apps/myprojects/modules/rfid/rfid_main.c
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
 * 读卡模块自己的命令行入口，给板端单独试读卡用。
 * 小车正常运行不从这里进，从 project1_main.c 的主循环进。
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
#  include <nuttx/spi/spi.h>
#  include <arch/board/board.h>

#  ifdef CONFIG_ARCH_CHIP_STM32
#    include <nuttx/arch.h>
#    include "stm32_spi.h"
#    include "stm32_gpio.h"

/* RFID RST pin: PB1 (for MFRC522 or CV520) */
#    define GPIO_RFID_RST     (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                               GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN1)
#  endif

#  include "rfid_api.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RFID_DEFAULT_SPI_BUS    1

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct rfid_dev_s *g_rfid_dev = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rfid_cmd_help
 ****************************************************************************/

static void rfid_cmd_help(void)
{
  printf("\nUsage: rfid <command> [options]\n");
  printf("\nCommands:\n");
  printf("  init [spi_bus]  - Initialize RFID device\n");
  printf("                    spi_bus: SPI bus number (default: 1)\n");
  printf("                    RST pin: PB1 (hardcoded)\n");
  printf("  deinit          - Deinitialize RFID device\n");
  printf("  read            - Read card UID\n");
  printf("  readblk [blk]   - Read MIFARE block (default blk=4, key=FFFFFFFFFFFF)\n");
  printf("  request         - Request card presence\n");
  printf("  diag            - Dump all key registers for debugging\n");
  printf("  spitest         - Raw SPI bus diagnostic (no init needed)\n");
  printf("  txtest [count]  - Continuous REQA TX test (default 50)\n");
  printf("  help            - Show this help message\n");
  printf("\n");
}

static int rfid_cmd_readblk(int argc, FAR char *argv[])
{
  struct rfid_uid_s uid;
  uint8_t key[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
  uint8_t data[16];
  int block = 4;
  int pass;
  int ret;
  int i;

  if (g_rfid_dev == NULL)
    {
      printf("ERROR: RFID device not initialized. Use 'rfid init' first\n");
      return ERROR;
    }

  if (argc > 1)
    {
      block = atoi(argv[1]);
      if (block < 0 || block > 63)
        {
          printf("ERROR: Invalid block %d (0..63)\n", block);
          return ERROR;
        }
    }

  printf("Reading block %d (KeyA=FFFFFFFFFFFF)...\n", block);

  /* For block operations, require strict flow:
   * request -> anticoll -> select -> auth -> read
   * Retry the whole chain a few times to improve robustness. */

  for (pass = 0; pass < 5; pass++)
    {
      ret = rfid_request(g_rfid_dev);
      if (ret < 0)
        {
          usleep(5000);
          continue;
        }

      ret = rfid_anticoll(g_rfid_dev, &uid);
      if (ret < 0)
        {
          usleep(5000);
          continue;
        }

      ret = rfid_select(g_rfid_dev, &uid);
      if (ret < 0)
        {
          usleep(5000);
          continue;
        }

      ret = rfid_authenticate_block(g_rfid_dev, &uid, (uint8_t)block, key, false);
      if (ret < 0)
        {
          usleep(5000);
          continue;
        }

      ret = rfid_read_block(g_rfid_dev, (uint8_t)block, data);
      if (ret == OK)
        {
          break;
        }

      usleep(5000);
    }

  if (ret < 0)
    {
      printf("ERROR: Read block %d failed after retries (error: %d)\n", block, ret);
      return ERROR;
    }

  printf("Block %d data: ", block);
  for (i = 0; i < 16; i++)
    {
      printf("%02X", data[i]);
      if (i < 15)
        {
          printf(" ");
        }
    }

  printf("\nASCII: ");
  for (i = 0; i < 16; i++)
    {
      uint8_t c = data[i];
      printf("%c", (c >= 32 && c <= 126) ? c : '.');
    }

  printf("\n");

  /* Stop Crypto1 after MIFARE operation */
  (void)rfid_stop_crypto(g_rfid_dev);
  return OK;
}

/****************************************************************************
 * Name: rfid_cmd_init
 ****************************************************************************/

static int rfid_cmd_init(int argc, FAR char *argv[])
{
  FAR struct spi_dev_s *spi;
  int spi_bus = RFID_DEFAULT_SPI_BUS;
  uint32_t rst_gpio = GPIO_RFID_RST;     /* Default: PB1 */

  if (g_rfid_dev != NULL)
    {
      printf("RFID device already initialized\n");
      return OK;
    }

  if (argc > 1)
    {
      spi_bus = atoi(argv[1]);
    }

  printf("Initializing RFID on SPI%d, RST pin: PB1\n", spi_bus);
  printf("DEBUG: rst_gpio = 0x%08lx\n", (unsigned long)rst_gpio);

#ifdef CONFIG_ARCH_CHIP_STM32
  printf("DEBUG: Calling stm32_spibus_initialize(%d)...\n", spi_bus);
  spi = stm32_spibus_initialize(spi_bus);
  printf("DEBUG: stm32_spibus_initialize returned %p\n", spi);
#else
  printf("ERROR: Unsupported architecture\n");
  return ERROR;
#endif

  if (spi == NULL)
    {
      printf("ERROR: Failed to initialize SPI%d\n", spi_bus);
      return ERROR;
    }

  printf("DEBUG: Calling rfid_register(spi=%p, rst_gpio=0x%08lx)...\n", 
         spi, (unsigned long)rst_gpio);
  g_rfid_dev = rfid_register(spi, rst_gpio);
  printf("DEBUG: rfid_register returned %p\n", g_rfid_dev);
  
  if (g_rfid_dev == NULL)
    {
      printf("ERROR: Failed to register RFID device\n");
      return ERROR;
    }

  printf("RFID device initialized successfully\n");
  return OK;
}

/****************************************************************************
 * Name: rfid_cmd_deinit
 ****************************************************************************/

static int rfid_cmd_deinit(int argc, FAR char *argv[])
{
  if (g_rfid_dev == NULL)
    {
      printf("RFID device not initialized\n");
      return ERROR;
    }

  rfid_unregister(g_rfid_dev);
  g_rfid_dev = NULL;

  printf("RFID device deinitialized\n");
  return OK;
}

/****************************************************************************
 * Name: rfid_cmd_read
 ****************************************************************************/

static int rfid_cmd_read(int argc, FAR char *argv[])
{
  struct rfid_uid_s uid;
  int ret;
  int i;

  if (g_rfid_dev == NULL)
    {
      printf("ERROR: RFID device not initialized. Use 'rfid init' first\n");
      return ERROR;
    }

  printf("Reading card...\n");

  ret = rfid_read_card(g_rfid_dev, &uid);
  if (ret < 0)
    {
      printf("ERROR: Failed to read card (error: %d)\n", ret);
      return ERROR;
    }

  printf("Card detected!\n");
  printf("UID size: %d bytes\n", uid.size);
  printf("UID: ");

  for (i = 0; i < uid.size; i++)
    {
      printf("%02X", uid.uid[i]);
      if (i < uid.size - 1)
        {
          printf(":");
        }
    }

  printf("\n");
  return OK;
}

/****************************************************************************
 * Name: rfid_cmd_request
 ****************************************************************************/

static int rfid_cmd_request(int argc, FAR char *argv[])
{
  int ret;

  if (g_rfid_dev == NULL)
    {
      printf("ERROR: RFID device not initialized. Use 'rfid init' first\n");
      return ERROR;
    }

  printf("Requesting card...\n");

  ret = rfid_request(g_rfid_dev);
  if (ret < 0)
    {
      printf("ERROR: No card detected (error: %d)\n", ret);
      return ERROR;
    }

  printf("Card detected!\n");
  return OK;
}

/****************************************************************************
 * Name: rfid_cmd_diag
 ****************************************************************************/

static int rfid_cmd_diag(int argc, FAR char *argv[])
{
  if (g_rfid_dev == NULL)
    {
      printf("ERROR: RFID device not initialized. Use 'rfid init' first\n");
      return ERROR;
    }

  return rfid_dump_regs(g_rfid_dev);
}

/****************************************************************************
 * Name: rfid_cmd_txtest
 *
 * Description:
 *   Continuously send REQA for TX modulation analysis on oscilloscope.
 *
 ****************************************************************************/

static int rfid_cmd_txtest(int argc, FAR char *argv[])
{
  int count = 50;

  if (g_rfid_dev == NULL)
    {
      printf("ERROR: RFID device not initialized. Use 'rfid init' first\n");
      return ERROR;
    }

  if (argc > 1)
    {
      count = atoi(argv[1]);
      if (count <= 0 || count > 1000)
        {
          count = 50;
        }
    }

  return rfid_txtest(g_rfid_dev, count);
}

/****************************************************************************
 * Name: rfid_cmd_spitest
 *
 * Description:
 *   Raw SPI diagnostic - bypasses driver, directly tests SPI bus.
 *   Helps diagnose: CS pin, MISO line, SPI mode, chip presence.
 *
 ****************************************************************************/

static int rfid_cmd_spitest(int argc, FAR char *argv[])
{
  FAR struct spi_dev_s *spi;
  uint8_t tx;
  uint8_t rx;
  int mode;

  printf("=== RAW SPI DIAGNOSTIC ===\n\n");

  spi = stm32_spibus_initialize(RFID_DEFAULT_SPI_BUS);
  if (spi == NULL)
    {
      printf("ERROR: SPI init failed\n");
      return ERROR;
    }

  /* Test 1: Read VersionReg(0x37) with Mode0 - normal operation */

  printf("[Test 1] Read VersionReg with CS active (Mode0, 500kHz)\n");
  SPI_LOCK(spi, true);
  SPI_SETFREQUENCY(spi, 500000);
  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  SPI_SELECT(spi, SPIDEV_CONTACTLESS(0), true);
  tx = 0x80 | (0x37 << 1);  /* VersionReg read addr = 0xEE */
  rx = SPI_SEND(spi, tx);
  printf("  TX=0x%02X(addr) -> RX=0x%02X (ignore, addr phase)\n", tx, rx);
  rx = SPI_SEND(spi, 0x00);
  printf("  TX=0x00(dummy) -> RX=0x%02X (this is VersionReg)\n", rx);
  SPI_SELECT(spi, SPIDEV_CONTACTLESS(0), false);
  SPI_LOCK(spi, false);

  /* Test 2: Same read WITHOUT CS active - if result is same, CS has no
   * effect (wiring issue or chip not connected) */

  printf("\n[Test 2] Read VersionReg WITHOUT CS (should differ from Test 1)\n");
  SPI_LOCK(spi, true);
  SPI_SETFREQUENCY(spi, 500000);
  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);
  /* NOT calling SPI_SELECT - CS stays HIGH (inactive) */
  tx = 0x80 | (0x37 << 1);
  rx = SPI_SEND(spi, tx);
  printf("  TX=0x%02X(addr) -> RX=0x%02X\n", tx, rx);
  rx = SPI_SEND(spi, 0x00);
  printf("  TX=0x00(dummy) -> RX=0x%02X\n", rx);
  SPI_LOCK(spi, false);

  /* Test 3: Try all 4 SPI modes */

  printf("\n[Test 3] VersionReg read with all SPI modes:\n");
  for (mode = 0; mode < 4; mode++)
    {
      SPI_LOCK(spi, true);
      SPI_SETFREQUENCY(spi, 500000);
      SPI_SETMODE(spi, mode);
      SPI_SETBITS(spi, 8);
      SPI_SELECT(spi, SPIDEV_CONTACTLESS(0), true);
      SPI_SEND(spi, 0x80 | (0x37 << 1));
      rx = SPI_SEND(spi, 0x00);
      SPI_SELECT(spi, SPIDEV_CONTACTLESS(0), false);
      SPI_LOCK(spi, false);
      printf("  Mode%d: VersionReg = 0x%02X %s\n", mode, rx,
             (rx != 0x00 && rx != 0xFF) ? "<-- non-trivial!" : "");
    }

  /* Restore Mode0 */

  SPI_LOCK(spi, true);
  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_LOCK(spi, false);

  /* Test 4: Write-readback test on TModeReg (0x2A)
   * Reset value = 0x00, try writing 0xAA then read back */

  printf("\n[Test 4] Write-readback TModeReg(0x2A): write 0xAA, readback\n");
  SPI_LOCK(spi, true);
  SPI_SETFREQUENCY(spi, 500000);
  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);

  /* Write 0xAA to TModeReg */

  SPI_SELECT(spi, SPIDEV_CONTACTLESS(0), true);
  tx = 0x00 | (0x2A << 1);  /* Write addr for TModeReg */
  SPI_SEND(spi, tx);
  SPI_SEND(spi, 0xAA);
  SPI_SELECT(spi, SPIDEV_CONTACTLESS(0), false);

  usleep(1000);

  /* Read back TModeReg */

  SPI_SELECT(spi, SPIDEV_CONTACTLESS(0), true);
  tx = 0x80 | (0x2A << 1);  /* Read addr for TModeReg */
  SPI_SEND(spi, tx);
  rx = SPI_SEND(spi, 0x00);
  SPI_SELECT(spi, SPIDEV_CONTACTLESS(0), false);
  SPI_LOCK(spi, false);

  printf("  Wrote 0xAA, read back 0x%02X %s\n", rx,
         (rx == 0xAA) ? "OK!" : "FAIL");

  /* Test 5: Read multiple different registers to check for stuck bus */

  printf("\n[Test 5] Read multiple registers (check for stuck bus):\n");
  {
    static const struct { uint8_t reg; const char *name; } regs[] =
    {
      {0x01, "Command "},
      {0x04, "ComIRQ  "},
      {0x06, "Error   "},
      {0x07, "Status1 "},
      {0x08, "Status2 "},
      {0x0A, "FIFOLvl "},
      {0x0C, "Control "},
      {0x11, "Mode    "},
      {0x14, "TxCtrl  "},
      {0x26, "RFCfg   "},
      {0x37, "Version "},
    };

    int i;
    bool all_same = true;
    uint8_t first_val = 0;

    SPI_LOCK(spi, true);
    SPI_SETFREQUENCY(spi, 500000);
    SPI_SETMODE(spi, SPIDEV_MODE0);
    SPI_SETBITS(spi, 8);

    for (i = 0; i < 11; i++)
      {
        SPI_SELECT(spi, SPIDEV_CONTACTLESS(0), true);
        SPI_SEND(spi, 0x80 | (regs[i].reg << 1));
        rx = SPI_SEND(spi, 0x00);
        SPI_SELECT(spi, SPIDEV_CONTACTLESS(0), false);

        printf("  [0x%02X] %s = 0x%02X\n", regs[i].reg, regs[i].name, rx);
        if (i == 0)
          {
            first_val = rx;
          }
        else if (rx != first_val)
          {
            all_same = false;
          }
      }

    SPI_LOCK(spi, false);

    if (all_same)
      {
        printf("\n  WARNING: All registers return 0x%02X!\n", first_val);
        if (first_val == 0xFF)
          {
            printf("  => MISO stuck HIGH. Check: MISO wiring, chip power,\n");
            printf("     CS connection, NRSTPD not shorted to GND.\n");
          }
        else if (first_val == 0x00)
          {
            printf("  => MISO stuck LOW. Check: MISO wiring, chip power,\n");
            printf("     solder joints, crystal connection.\n");
          }
      }
    else
      {
        printf("\n  Registers have different values - SPI is communicating!\n");
      }
  }

  printf("\n=== SPI DIAGNOSTIC DONE ===\n");
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rfid_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  if (argc < 2)
    {
      rfid_cmd_help();
      return ERROR;
    }

  if (strcmp(argv[1], "init") == 0)
    {
      return rfid_cmd_init(argc - 1, &argv[1]);
    }
  else if (strcmp(argv[1], "deinit") == 0)
    {
      return rfid_cmd_deinit(argc - 1, &argv[1]);
    }
  else if (strcmp(argv[1], "read") == 0)
    {
      return rfid_cmd_read(argc - 1, &argv[1]);
    }
  else if (strcmp(argv[1], "request") == 0)
    {
      return rfid_cmd_request(argc - 1, &argv[1]);
    }
  else if (strcmp(argv[1], "readblk") == 0)
    {
      return rfid_cmd_readblk(argc - 1, &argv[1]);
    }
  else if (strcmp(argv[1], "diag") == 0)
    {
      return rfid_cmd_diag(argc - 1, &argv[1]);
    }
  else if (strcmp(argv[1], "spitest") == 0)
    {
      return rfid_cmd_spitest(argc - 1, &argv[1]);
    }
  else if (strcmp(argv[1], "txtest") == 0)
    {
      return rfid_cmd_txtest(argc - 1, &argv[1]);
    }
  else if (strcmp(argv[1], "help") == 0)
    {
      rfid_cmd_help();
      return OK;
    }
  else
    {
      printf("ERROR: Unknown command: %s\n", argv[1]);
      rfid_cmd_help();
      return ERROR;
    }
}

#endif /* CONFIG_NSH_LIBRARY */
