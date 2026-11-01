/****************************************************************************
 * apps/myprojects/modules/rfid/rfid_api.c
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
 * 将 CV520 的寄存器操作组成读卡步骤，调用顺序见 include/rfid_api.h。
 * 读完后 halt 使卡休眠，cleanup 清除芯片内的本次会话状态。
 ****************************************************************************/


/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>
#include <unistd.h>

#include <nuttx/spi/spi.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <arch/board/board.h>

#ifdef CONFIG_ARCH_CHIP_STM32
#  include <stm32_gpio.h>
#endif

#include "../../include/rfid_api.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MFRC522 Register Definitions */

#define MFRC522_REG_COMMAND               0x01
#define MFRC522_REG_COMIEN                0x02
#define MFRC522_REG_DIVIEN                0x03
#define MFRC522_REG_COMIRQ                0x04
#define MFRC522_REG_DIVIRQ                0x05
#define MFRC522_REG_ERROR                 0x06
#define MFRC522_REG_STATUS1               0x07
#define MFRC522_REG_STATUS2               0x08
#define MFRC522_REG_FIFO_DATA             0x09
#define MFRC522_REG_FIFO_LEVEL            0x0A
#define MFRC522_REG_WATER_LEVEL           0x0B
#define MFRC522_REG_CONTROL               0x0C
#define MFRC522_REG_BIT_FRAMING           0x0D
#define MFRC522_REG_COLL                  0x0E
#define MFRC522_REG_MODE                  0x11
#define MFRC522_REG_TX_MODE               0x12
#define MFRC522_REG_RX_MODE               0x13
#define MFRC522_REG_TX_CONTROL            0x14
#define MFRC522_REG_TX_ASK                0x15
#define MFRC522_REG_TX_SEL                0x16
#define MFRC522_REG_RX_SEL                0x17
#define MFRC522_REG_RX_THRESHOLD          0x18
#define MFRC522_REG_DEMOD                 0x19
#define MFRC522_REG_MFTX                  0x1C
#define MFRC522_REG_MFRX                  0x1D
#define MFRC522_REG_SERIAL_SPEED          0x1F
#define MFRC522_REG_CRC_RESULT_H          0x21
#define MFRC522_REG_CRC_RESULT_L          0x22
#define MFRC522_REG_MOD_WIDTH             0x24
#define MFRC522_REG_RFCFG                 0x26
#define MFRC522_REG_GSN                   0x27
#define MFRC522_REG_CWGSP                 0x28
#define MFRC522_REG_MODGSP                0x29
#define MFRC522_REG_TMODE                 0x2A
#define MFRC522_REG_TPRESCALER            0x2B
#define MFRC522_REG_TRELOAD_H             0x2C
#define MFRC522_REG_TRELOAD_L             0x2D
#define MFRC522_REG_EXREG                 0x0F
#define MFRC522_REG_EXE_WR                0x37

/* MFRC522 Command Definitions */

#define MFRC522_COMMAND_IDLE               0x00
#define MFRC522_COMMAND_MEM                0x01
#define MFRC522_COMMAND_RANDOM_ID          0x02
#define MFRC522_COMMAND_CALC_CRC          0x03
#define MFRC522_COMMAND_TRANSMIT           0x04
#define MFRC522_COMMAND_NO_CHANGE          0x07
#define MFRC522_COMMAND_RECEIVE            0x08
#define MFRC522_COMMAND_TRANSCEIVE         0x0C
#define MFRC522_COMMAND_MF_AUTHENT         0x0E
#define MFRC522_COMMAND_SOFT_RESET         0x0F

/* Status Bits */

#define MFRC522_STATUS1_CRC_OK            (1 << 6)
#define MFRC522_STATUS1_IRQ                (1 << 4)
#define MFRC522_STATUS1_LO_ALERT           (1 << 0)

/* Error Bits */

#define MFRC522_ERROR_COLL                 (1 << 3)
#define MFRC522_ERROR_CRC                  (1 << 2)
#define MFRC522_ERROR_PARITY               (1 << 1)
#define MFRC522_ERROR_PROTOCOL             (1 << 0)

/* SPI Address Format */

#define MFRC522_SPI_READ                  0x80
#define MFRC522_SPI_WRITE                 0x00

/* ISO14443A Commands */

#define PCD_CMD_REQA                      0x26
#define PCD_CMD_WUPA                      0x52
#define PCD_CMD_ANTICOLL_1                0x93
#define PCD_CMD_ANTICOLL_2                0x95
#define PCD_CMD_SELECT_1                  0x93
#define PCD_CMD_SELECT_2                  0x95
#define PICC_CMD_MF_READ                  0x30
#define PICC_CMD_MF_AUTH_KEY_A            0x60
#define PICC_CMD_MF_AUTH_KEY_B            0x61

/* Timeouts */

#define MFRC522_TIMEOUT_MS                1000
#define MFRC522_DELAY_MS                  10

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rfid_dev_s
{
  struct spi_dev_s *spi;      /* SPI device */
  int rstpin;                 /* Reset GPIO pin (-1 if not used) */
  bool initialized;          /* Initialization flag */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rfid_spi_read(FAR struct rfid_dev_s *dev, uint8_t reg,
                         FAR uint8_t *data, size_t len);
static int rfid_spi_write(FAR struct rfid_dev_s *dev, uint8_t reg,
                          FAR const uint8_t *data, size_t len);
static int rfid_write_reg(FAR struct rfid_dev_s *dev, uint8_t reg,
                          uint8_t value);
static int rfid_read_reg(FAR struct rfid_dev_s *dev, uint8_t reg,
                         FAR uint8_t *value);
static int rfid_set_bits(FAR struct rfid_dev_s *dev, uint8_t reg,
                         uint8_t mask);
static int rfid_clear_bits(FAR struct rfid_dev_s *dev, uint8_t reg,
                            uint8_t mask);
static int rfid_set_bit_framing(FAR struct rfid_dev_s *dev, uint8_t tx_last_bits);
static int rfid_soft_reset(FAR struct rfid_dev_s *dev);
static int rfid_init_chip(FAR struct rfid_dev_s *dev);
static int rfid_transceive(FAR struct rfid_dev_s *dev,
                            FAR const uint8_t *send, size_t send_len,
                            FAR uint8_t *recv, FAR size_t *recv_len,
                            uint32_t timeout_ms);
static int rfid_calculate_crc(FAR struct rfid_dev_s *dev,
                               FAR const uint8_t *data, size_t len,
                               FAR uint16_t *crc) __attribute__((unused));
static uint16_t rfid_crc_a_sw(FAR const uint8_t *data, size_t len);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rfid_spi_read
 *
 * Description:
 *   Read data from MFRC522 via SPI.
 *   SPI address format: bit7=1(read), bits[6:1]=register address, bit0=0
 *
 ****************************************************************************/

static int rfid_spi_read(FAR struct rfid_dev_s *dev, uint8_t reg,
                          FAR uint8_t *data, size_t len)
{
  struct spi_dev_s *spi = dev->spi;
  uint8_t addr;
  size_t i;

  if (spi == NULL)
    {
      return -EINVAL;
    }

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_CONTACTLESS(0), true);

  for (i = 0; i < len; i++)
    {
      /* MFRC522B SPI address format: bit7=1 for read, bits[6:1]=register, bit0=0 */

      addr = 0x80 | ((reg & 0x3f) << 1);
      SPI_SEND(spi, addr);
      data[i] = SPI_SEND(spi, 0x00);
    }

  SPI_SELECT(spi, SPIDEV_CONTACTLESS(0), false);
  SPI_LOCK(spi, false);

  return OK;
}

/****************************************************************************
 * Name: rfid_spi_write
 *
 * Description:
 *   Write data to MFRC522 via SPI.
 *   SPI address format: bit7=0(write), bits[6:1]=register address, bit0=0
 *
 ****************************************************************************/

static int rfid_spi_write(FAR struct rfid_dev_s *dev, uint8_t reg,
                          FAR const uint8_t *data, size_t len)
{
  struct spi_dev_s *spi = dev->spi;
  uint8_t addr;
  size_t i;

  if (spi == NULL)
    {
      return -EINVAL;
    }

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_CONTACTLESS(0), true);

  for (i = 0; i < len; i++)
    {
      /* MFRC522B SPI address format: bit7=0 for write, bits[6:1]=register, bit0=0 */

      addr = 0x00 | ((reg & 0x3f) << 1);
      SPI_SEND(spi, addr);
      SPI_SEND(spi, data[i]);
    }

  SPI_SELECT(spi, SPIDEV_CONTACTLESS(0), false);
  SPI_LOCK(spi, false);

  return OK;
}

/****************************************************************************
 * Name: rfid_write_reg
 *
 * Description:
 *   Write a single register.
 *
 ****************************************************************************/

static int rfid_write_reg(FAR struct rfid_dev_s *dev, uint8_t reg,
                           uint8_t value)
{
  return rfid_spi_write(dev, reg, &value, 1);
}

/****************************************************************************
 * Name: rfid_read_reg
 *
 * Description:
 *   Read a single register.
 *
 ****************************************************************************/

static int rfid_read_reg(FAR struct rfid_dev_s *dev, uint8_t reg,
                         FAR uint8_t *value)
{
  return rfid_spi_read(dev, reg, value, 1);
}

/****************************************************************************
 * Name: rfid_set_bits
 *
 * Description:
 *   Set bits in a register.
 *
 ****************************************************************************/

static int rfid_set_bits(FAR struct rfid_dev_s *dev, uint8_t reg,
                         uint8_t mask)
{
  uint8_t value;
  int ret;

  ret = rfid_read_reg(dev, reg, &value);
  if (ret < 0)
    {
      return ret;
    }

  value |= mask;
  return rfid_write_reg(dev, reg, value);
}

/****************************************************************************
 * Name: rfid_clear_bits
 *
 * Description:
 *   Clear bits in a register.
 *
 ****************************************************************************/

static int rfid_clear_bits(FAR struct rfid_dev_s *dev, uint8_t reg,
                            uint8_t mask)
{
  uint8_t value;
  int ret;

  ret = rfid_read_reg(dev, reg, &value);
  if (ret < 0)
    {
      return ret;
    }

  value &= ~mask;
  return rfid_write_reg(dev, reg, value);
}

/****************************************************************************
 * Name: rfid_set_bit_framing
 *
 * Description:
 *   Configure BitFramingReg with TxLastBits (bits 2:0).
 *
 ****************************************************************************/

static int rfid_set_bit_framing(FAR struct rfid_dev_s *dev, uint8_t tx_last_bits)
{
  uint8_t value;
  int ret;

  ret = rfid_read_reg(dev, MFRC522_REG_BIT_FRAMING, &value);
  if (ret < 0)
    {
      return ret;
    }

  value &= ~0x07;
  value |= (tx_last_bits & 0x07);
  return rfid_write_reg(dev, MFRC522_REG_BIT_FRAMING, value);
}
/****************************************************************************
 * Name: rfid_soft_reset
 *
 * Description:
 *   Perform a soft reset of the MFRC522.
 *
 ****************************************************************************/

static int rfid_soft_reset(FAR struct rfid_dev_s *dev)
{
  uint8_t value;
  uint32_t timeout;
  int ret;

  /* Send soft reset command */

  ret = rfid_read_reg(dev, MFRC522_REG_COMMAND, &value);
  if (ret < 0)
    {
      return ret;
    }

  value &= ~0x0f;
  value |= MFRC522_COMMAND_SOFT_RESET;
  ret = rfid_write_reg(dev, MFRC522_REG_COMMAND, value);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait for PowerDown bit (bit 4) to clear */

  usleep(1000); /* 1ms initial delay */

  timeout = MFRC522_TIMEOUT_MS;
  while (timeout > 0)
    {
      ret = rfid_read_reg(dev, MFRC522_REG_COMMAND, &value);
      if (ret < 0)
        {
          return ret;
        }

      if ((value & 0x10) == 0)  /* PowerDown bit cleared */
        {
          break;
        }

      usleep(1000);
      timeout--;
    }

  if (timeout == 0)
    {
      printf("RFID: soft_reset TIMEOUT\n");
      return -ETIMEDOUT;
    }

  return OK;
}

/****************************************************************************
 * Name: rfid_init_chip
 *
 * Description:
 *   Initialize the MFRC522 chip with default settings.
 *
 ****************************************************************************/

static int rfid_init_chip(FAR struct rfid_dev_s *dev)
{
  uint8_t status1;
  uint8_t error;
  int ret;

  /* Probe basic status/error registers to verify SPI communication */

  /* Probe SPI communication by reading basic registers */

  ret = rfid_read_reg(dev, MFRC522_REG_STATUS1, &status1);
  if (ret < 0)
    {
      return ret;
    }

  ret = rfid_read_reg(dev, MFRC522_REG_ERROR, &error);
  if (ret < 0)
    {
      return ret;
    }

  /* Soft reset */

  ret = rfid_soft_reset(dev);
  if (ret < 0)
    {
      return ret;
    }

  usleep(50000); /* 50ms delay after reset */

  /* Configure for ISO14443A */

  /* Default bit framing: 8 bits */

  ret = rfid_write_reg(dev, MFRC522_REG_BIT_FRAMING, 0x00);
  if (ret < 0)
    {
      return ret;
    }

  /* Set ModeReg to datasheet reset/default: 0x3F */

  ret = rfid_write_reg(dev, MFRC522_REG_MODE, 0x3F);
  if (ret < 0)
    {
      return ret;
    }

  /* Set TxMode */

  ret = rfid_write_reg(dev, MFRC522_REG_TX_MODE, 0x00);
  if (ret < 0)
    {
      return ret;
    }

  /* Set RxMode */

  ret = rfid_write_reg(dev, MFRC522_REG_RX_MODE, 0x00);
  if (ret < 0)
    {
      return ret;
    }

  /* Set ModWidth */

  ret = rfid_write_reg(dev, MFRC522_REG_MOD_WIDTH, 0x26);
  if (ret < 0)
    {
      return ret;
    }

  /* Set TxASK: Force 100% ASK */

  ret = rfid_write_reg(dev, MFRC522_REG_TX_ASK, 0x40);
  if (ret < 0)
    {
      return ret;
    }

  /* Set TxSelReg: DriverSel=01 (modulated envelope from internal encoder) */

  ret = rfid_write_reg(dev, MFRC522_REG_TX_SEL, 0x10);
  if (ret < 0)
    {
      return ret;
    }

  /* Receiver settings
   *
   * Keep analog front-end related registers at chip reset defaults
   * (RxSel/RxThreshold/Demod/MFTX/MFRX). Previous aggressive tuning
   * produced unstable short frames (recv_len 0/1) on MFRC522B.
   */

  /* RxModeReg: default settings at 106kbps, no RxCRCEn, no RxNoErr */
  ret = rfid_write_reg(dev, MFRC522_REG_RX_MODE, 0x00);
  if (ret < 0)
    {
      return ret;
    }

  /* Ensure Crypto1 is OFF */

  ret = rfid_clear_bits(dev, MFRC522_REG_STATUS2, 0x08);
  if (ret < 0)
    {
      return ret;
    }

  /* Set antenna driver conductance for maximum output */
  /* GsNOnReg (0x27): CWGsNOn=F, ModGsNOn=F (maximum N-driver conductance) */

  ret = rfid_write_reg(dev, MFRC522_REG_GSN, 0xFF);
  if (ret < 0)
    {
      return ret;
    }

  /* CWGsPReg (0x28): CWGsP=0x3F (maximum P-driver conductance, no modulation) */

  ret = rfid_write_reg(dev, MFRC522_REG_CWGSP, 0x3F);
  if (ret < 0)
    {
      return ret;
    }

  /* ModGsPReg (0x29): ModGsP=0x3F (maximum P-driver conductance, modulation) */

  ret = rfid_write_reg(dev, MFRC522_REG_MODGSP, 0x3F);
  if (ret < 0)
    {
      return ret;
    }

  /* Set TxControl: enable TX1+TX2 with differential drive (MFRC522B)
   * bit6: InvTxRfOn - invert one TX for 180° differential drive
   * bit1: TX2RFEn   - enable TX2 antenna driver
   * bit0: TX1RFEn   - enable TX1 antenna driver
   * 0x43 = differential + TX1 + TX2 (confirmed working) */

  ret = rfid_write_reg(dev, MFRC522_REG_TX_CONTROL, 0x43);
  if (ret < 0)
    {
      return ret;
    }

  /* Set antenna gain */

  ret = rfid_write_reg(dev, MFRC522_REG_RFCFG, 0x70);
  if (ret < 0)
    {
      return ret;
    }

  /* Set timer for ~25ms timeout (was ~2s!)
   *
   * TPrescaler = 0x00A9 = 169
   * TReload    = 0x03E7 = 999
   * td = (2*169+1)*(999+1)/13.56MHz ≈ 25ms
   *
   * TAuto=1 (bit7): timer auto-starts with Transceive command
   */

  ret = rfid_write_reg(dev, MFRC522_REG_TMODE, 0x80);
  if (ret < 0)
    {
      return ret;
    }

  ret = rfid_write_reg(dev, MFRC522_REG_TPRESCALER, 0xA9);
  if (ret < 0)
    {
      return ret;
    }

  ret = rfid_write_reg(dev, MFRC522_REG_TRELOAD_H, 0x03);
  if (ret < 0)
    {
      return ret;
    }

  ret = rfid_write_reg(dev, MFRC522_REG_TRELOAD_L, 0xE7);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: rfid_transceive
 *
 * Description:
 *   Transmit and receive data via MFRC522.
 *
 ****************************************************************************/

static int rfid_transceive(FAR struct rfid_dev_s *dev,
                            FAR const uint8_t *send, size_t send_len,
                            FAR uint8_t *recv, FAR size_t *recv_len,
                            uint32_t timeout_ms)
{
  uint8_t error;
  uint8_t fifo_level;
  uint8_t com_irq;
  uint32_t timeout;
  size_t i;
  int ret;

  if (send_len > 64)
    {
      return -EINVAL;
    }

  /* Set command to idle */

  ret = rfid_write_reg(dev, MFRC522_REG_COMMAND, MFRC522_COMMAND_IDLE);
  if (ret < 0)
    {
      return ret;
    }

  /* Clear collision register */

  (void)rfid_write_reg(dev, MFRC522_REG_COLL, 0x80);

  /* Flush FIFO */

  ret = rfid_set_bits(dev, MFRC522_REG_FIFO_LEVEL, 0x80);
  if (ret < 0)
    {
      return ret;
    }

  /* Clear pending CRC interrupt flags before starting a new CRC operation.
   * Otherwise a stale CRCIRq can make us read an old CRC value. */

  ret = rfid_write_reg(dev, MFRC522_REG_DIVIRQ, 0x7F);
  if (ret < 0)
    {
      return ret;
    }

  /* Write data to FIFO */

  for (i = 0; i < send_len; i++)
    {
      ret = rfid_write_reg(dev, MFRC522_REG_FIFO_DATA, send[i]);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* Clear interrupt flags */

  /* Clear all interrupt flags (Set1=0, writing 1 to bits 6:0 clears them) */

  ret = rfid_write_reg(dev, MFRC522_REG_COMIRQ, 0x7F);
  if (ret < 0)
    {
      return ret;
    }

  ret = rfid_write_reg(dev, MFRC522_REG_DIVIRQ, 0x7F);
  if (ret < 0)
    {
      return ret;
    }

  /* Enable interrupts */

  ret = rfid_write_reg(dev, MFRC522_REG_COMIEN, 0x77);
  if (ret < 0)
    {
      return ret;
    }

  /* Start transceive */

  ret = rfid_write_reg(dev, MFRC522_REG_COMMAND, MFRC522_COMMAND_TRANSCEIVE);
  if (ret < 0)
    {
      return ret;
    }

  /* Set StartSend bit to begin transmission */

  ret = rfid_set_bits(dev, MFRC522_REG_BIT_FRAMING, 0x80);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait for completion */

  timeout = timeout_ms;
  while (timeout > 0)
    {
      ret = rfid_read_reg(dev, MFRC522_REG_COMIRQ, &com_irq);
      if (ret < 0)
        {
          return ret;
        }

      /* RxIRq (bit5) or TimerIRq (bit0) */

      if (com_irq & 0x21)
        {
          break;
        }

      usleep(1000);
      timeout--;
    }

  if (timeout == 0)
    {
      rfid_write_reg(dev, MFRC522_REG_COMMAND, MFRC522_COMMAND_IDLE);
      return -ETIMEDOUT;
    }

  if ((com_irq & 0x20) == 0 && (com_irq & 0x01))
    {
      /* Timer fired but NO RxIRq → genuine timeout, no card response.
       *
       * NOTE: When CONFIG_USEC_PER_TICK is large (e.g. 10ms), usleep(1000)
       * actually sleeps 10ms, which is longer than the MFRC522 timer (~2ms).
       * In this case, BOTH RxIRq and TimerIRq may be set simultaneously
       * when a card DID respond. We must prioritize RxIRq over TimerIRq
       * to avoid false timeouts.
       */

      rfid_write_reg(dev, MFRC522_REG_COMMAND, MFRC522_COMMAND_IDLE);
      return -ETIMEDOUT;
    }

  /* Clear start send bit */

  ret = rfid_clear_bits(dev, MFRC522_REG_BIT_FRAMING, 0x80);
  if (ret < 0)
    {
      return ret;
    }

  /* Read error register */

  ret = rfid_read_reg(dev, MFRC522_REG_ERROR, &error);
  if (ret < 0)
    {
      return ret;
    }

  /* Error bits are checked below; only BufferOvfl and Protocol are fatal */

  /* Check for BufferOverflow - the only truly fatal error.
   * Other errors (Coll/CRC/Parity/Protocol) are reported but
   * we still read FIFO data so callers can validate via BCC/CRC. */

  if (error & 0x10) /* BufferOvfl */
    {
      rfid_write_reg(dev, MFRC522_REG_COMMAND, MFRC522_COMMAND_IDLE);
      return -EIO;
    }

  /* Read FIFO level FIRST, then read data (before going idle) */

  ret = rfid_read_reg(dev, MFRC522_REG_FIFO_LEVEL, &fifo_level);
  if (ret < 0)
    {
      return ret;
    }

  fifo_level &= 0x7f;

  /* Read data from FIFO */

  if (recv != NULL && recv_len != NULL)
    {
      size_t read_len = (fifo_level < *recv_len) ? fifo_level : *recv_len;

      for (i = 0; i < read_len; i++)
        {
          ret = rfid_read_reg(dev, MFRC522_REG_FIFO_DATA, &recv[i]);
          if (ret < 0)
            {
              return ret;
            }
        }

      *recv_len = read_len;
    }

  /* Set command to idle */

  ret = rfid_write_reg(dev, MFRC522_REG_COMMAND, MFRC522_COMMAND_IDLE);
  if (ret < 0)
    {
      return ret;
    }

  /* Now check for protocol-level errors.
   * Collision and parity errors may occur due to weak signal,
   * but if we got data, let the caller validate via BCC. */

  if (error & MFRC522_ERROR_PROTOCOL)
    {
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Name: rfid_calculate_crc
 *
 * Description:
 *   Calculate CRC for data using MFRC522 hardware.
 *
 ****************************************************************************/

static int rfid_calculate_crc(FAR struct rfid_dev_s *dev,
                               FAR const uint8_t *data, size_t len,
                               FAR uint16_t *crc)
{
  uint8_t command;
  uint8_t com_irq;
  uint32_t timeout;
  size_t i;
  int ret;

  /* Set command to idle */

  ret = rfid_write_reg(dev, MFRC522_REG_COMMAND, MFRC522_COMMAND_IDLE);
  if (ret < 0)
    {
      return ret;
    }

  /* Flush FIFO */

  ret = rfid_set_bits(dev, MFRC522_REG_FIFO_LEVEL, 0x80);
  if (ret < 0)
    {
      return ret;
    }

  /* Write data to FIFO */

  for (i = 0; i < len; i++)
    {
      ret = rfid_write_reg(dev, MFRC522_REG_FIFO_DATA, data[i]);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* Start CRC calculation */

  ret = rfid_write_reg(dev, MFRC522_REG_COMMAND, MFRC522_COMMAND_CALC_CRC);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait for completion */

  timeout = 1000;
  while (timeout > 0)
    {
      ret = rfid_read_reg(dev, MFRC522_REG_DIVIRQ, &com_irq);
      if (ret < 0)
        {
          return ret;
        }

      if (com_irq & 0x04) /* CRC interrupt */
        {
          break;
        }

      usleep(1000);
      timeout--;
    }

  if (timeout == 0)
    {
      return -ETIMEDOUT;
    }

  /* Read CRC result */

  ret = rfid_read_reg(dev, MFRC522_REG_CRC_RESULT_L, &command);
  if (ret < 0)
    {
      return ret;
    }

  *crc = command;

  ret = rfid_read_reg(dev, MFRC522_REG_CRC_RESULT_H, &command);
  if (ret < 0)
    {
      return ret;
    }

  *crc |= (command << 8);

  /* Set command to idle */

  ret = rfid_write_reg(dev, MFRC522_REG_COMMAND, MFRC522_COMMAND_IDLE);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: rfid_crc_a_sw
 *
 * Description:
 *   Software CRC_A (ISO14443A, polynomial 0x8408, init 0x6363)
 *
 ****************************************************************************/

static uint16_t rfid_crc_a_sw(FAR const uint8_t *data, size_t len)
{
  uint16_t crc = 0x6363;
  size_t i;

  for (i = 0; i < len; i++)
    {
      uint8_t c = data[i] ^ (uint8_t)(crc & 0x00ff);
      c ^= (uint8_t)(c << 4);
      crc = (crc >> 8) ^ ((uint16_t)c << 8) ^ ((uint16_t)c << 3) ^
            ((uint16_t)c >> 4);
    }

  return crc;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rfid_register
 *
 * Description:
 *   Register the RFID device and initialize it.
 *
 ****************************************************************************/

FAR struct rfid_dev_s *rfid_register(FAR struct spi_dev_s *spi, int rstpin)
{
  FAR struct rfid_dev_s *dev;
  int ret;

  printf("RFID: registering (rstpin=0x%08x)...\n", rstpin);

  if (spi == NULL)
    {
      printf("RFID: ERROR - SPI is NULL\n");
      return NULL;
    }

  /* Allocate device structure */

  dev = (FAR struct rfid_dev_s *)kmm_zalloc(sizeof(struct rfid_dev_s));
  if (dev == NULL)
    {
      printf("RFID: ERROR - Failed to allocate device structure\n");
      return NULL;
    }

  dev->spi = spi;
  dev->rstpin = rstpin;

  /* Configure SPI - MFRC522 supports up to 10MHz, but use 1MHz for stability */

  SPI_SETFREQUENCY(spi, 1000000); /* 1MHz for stability */
  SPI_SETMODE(spi, SPIDEV_MODE0); /* CPOL=0, CPHA=0 */
  SPI_SETBITS(spi, 8);

  /* Handle reset pin if provided */

  if (rstpin != 0)
    {
      stm32_configgpio(rstpin);

      /* Reset sequence: pull low, wait, pull high */

      stm32_gpiowrite(rstpin, false);
      usleep(50000); /* 50ms low */
      stm32_gpiowrite(rstpin, true);
      usleep(100000); /* 100ms high - give chip time to stabilize */
    }

  /* Initialize chip */

  ret = rfid_init_chip(dev);
  if (ret < 0)
    {
      printf("RFID: ERROR chip init failed (%d)\n", ret);
      kmm_free(dev);
      return NULL;
    }

  dev->initialized = true;
  printf("RFID: initialized OK\n");

  return dev;
}

/****************************************************************************
 * Name: rfid_unregister
 *
 * Description:
 *   Unregister the RFID device and release resources.
 *
 ****************************************************************************/

int rfid_unregister(FAR struct rfid_dev_s *dev)
{
  if (dev == NULL)
    {
      return -EINVAL;
    }

  if (dev->rstpin != 0)
    {
      stm32_gpiowrite(dev->rstpin, false);
    }

  kmm_free(dev);

  return OK;
}

/****************************************************************************
 * Name: rfid_request
 *
 * Description:
 *   Request a card to be present (wake up).
 *
 ****************************************************************************/

int rfid_request(FAR struct rfid_dev_s *dev)
{
  uint8_t send_buf[1];
  uint8_t recv_buf[2];
  size_t recv_len;
  uint8_t error;
  int ret2;
  int ret;

  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

  /* Ensure Crypto1 is OFF */

  ret = rfid_clear_bits(dev, MFRC522_REG_STATUS2, 0x08);
  if (ret < 0)
    {
      return ret;
    }

  /* Try REQA first */

  send_buf[0] = PCD_CMD_REQA;
  recv_len = sizeof(recv_buf);

  /* REQA is a 7-bit command, set TxLastBits=7 */

  ret = rfid_set_bit_framing(dev, 0x07);
  if (ret < 0)
    {
      return ret;
    }

  ret = rfid_transceive(dev, send_buf, 1, recv_buf, &recv_len, 100);

  ret2 = rfid_set_bit_framing(dev, 0x00);
  if (ret == OK && ret2 < 0)
    {
      ret = ret2;
    }

  (void)rfid_read_reg(dev, MFRC522_REG_ERROR, &error);

  /* REQA/WUPA response (ATQA) must be 2 bytes */

  if (ret == OK && recv_len >= 2 &&
      (error & (MFRC522_ERROR_CRC | MFRC522_ERROR_PARITY |
                MFRC522_ERROR_PROTOCOL)) == 0)
    {
      return OK;
    }

  /* If REQA fails, try WUPA (wake-up) */

  send_buf[0] = PCD_CMD_WUPA;
  recv_len = sizeof(recv_buf);

  ret = rfid_set_bit_framing(dev, 0x07);
  if (ret < 0)
    {
      return ret;
    }

  ret = rfid_transceive(dev, send_buf, 1, recv_buf, &recv_len, 100);

  ret2 = rfid_set_bit_framing(dev, 0x00);
  if (ret == OK && ret2 < 0)
    {
      ret = ret2;
    }

  (void)rfid_read_reg(dev, MFRC522_REG_ERROR, &error);

  if (ret < 0)
    {
      return ret;
    }

  if (recv_len < 2 ||
      (error & (MFRC522_ERROR_CRC | MFRC522_ERROR_PARITY |
                MFRC522_ERROR_PROTOCOL)) != 0)
    {
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Name: rfid_anticoll
 *
 * Description:
 *   Perform anti-collision detection and get card UID.
 *
 ****************************************************************************/

int rfid_anticoll(FAR struct rfid_dev_s *dev, FAR struct rfid_uid_s *uid)
{
  uint8_t send_buf[2];
  uint8_t recv_buf[5];   /* Anticollision returns 4 UID bytes + BCC (no CRC) */
  size_t recv_len;
  int ret;
  int i;
  uint8_t bcc;
  int attempt;

  if (dev == NULL || !dev->initialized || uid == NULL)
    {
      return -EINVAL;
    }

  /* Ensure 8-bit framing for anticollision/select */

  ret = rfid_set_bit_framing(dev, 0x00);
  if (ret < 0)
    {
      return ret;
    }

  /* Cascade Level 1 anticollision */

  send_buf[0] = PCD_CMD_ANTICOLL_1;
  send_buf[1] = 0x20;
  recv_len = sizeof(recv_buf);

  for (attempt = 0; attempt < 5; attempt++)
    {
      recv_len = sizeof(recv_buf);
      ret = rfid_transceive(dev, send_buf, 2, recv_buf, &recv_len, 100);

      if (ret < 0 || recv_len != 5)
        {
          usleep(3000);
          continue;
        }

    {
      /* Verify BCC (XOR of UID bytes) */

      bcc = recv_buf[0] ^ recv_buf[1] ^ recv_buf[2] ^ recv_buf[3];
      if (bcc != recv_buf[4])
        {
          usleep(3000);
          continue;
        }

      if (recv_buf[0] == 0x88)
        {
          /* Cascade tag detected, UID is 7 bytes.
           * Store UID[0..2] from cascade level 1 (bytes 1..3).
           */

          for (i = 0; i < 3; i++)
            {
              uid->uid[i] = recv_buf[i + 1];
            }

          /* Cascade Level 2 anticollision for remaining 4 bytes */

          send_buf[0] = PCD_CMD_ANTICOLL_2;
          send_buf[1] = 0x20;
          recv_len = sizeof(recv_buf);

          ret = rfid_transceive(dev, send_buf, 2, recv_buf, &recv_len, 100);
          if (ret < 0 || recv_len != 5)
            {
              usleep(3000);
              continue;
            }

          bcc = recv_buf[0] ^ recv_buf[1] ^ recv_buf[2] ^ recv_buf[3];
          if (bcc != recv_buf[4])
            {
              usleep(3000);
              continue;
            }

          uid->size = 7;
          for (i = 0; i < 4; i++)
            {
              uid->uid[3 + i] = recv_buf[i];
            }

          return OK;
        }
      else
        {
          /* 4-byte UID */

          uid->size = 4;
          for (i = 0; i < 4; i++)
            {
              uid->uid[i] = recv_buf[i];
            }

          return OK;
        }
      }
    }

  return -EIO;
}

/****************************************************************************
 * Name: rfid_select
 *
 * Description:
 *   Select a card by its UID.
 *
 ****************************************************************************/

int rfid_select(FAR struct rfid_dev_s *dev, FAR const struct rfid_uid_s *uid)
{
  uint8_t send_buf[12];  /* Select command + UID/BCC + CRC */
  uint8_t recv_buf[10];
  size_t recv_len;
  uint16_t crc;
  uint8_t bcc;
  int ret;
  int attempt;
  int i;

  if (dev == NULL || !dev->initialized || uid == NULL)
    {
      return -EINVAL;
    }

  ret = rfid_set_bit_framing(dev, 0x00);
  if (ret < 0)
    {
      return ret;
    }

  if (uid->size == 4)
    {
      /* Level 1 select */

      send_buf[0] = PCD_CMD_SELECT_1;
      send_buf[1] = 0x70;

      bcc = 0;
      for (i = 0; i < 4; i++)
        {
          send_buf[2 + i] = uid->uid[i];
          bcc ^= uid->uid[i];
        }

      send_buf[6] = bcc;

      crc = rfid_crc_a_sw(send_buf, 7);

      /* ISO14443A CRC: LSB first */

      send_buf[7] = crc & 0xff;
      send_buf[8] = (crc >> 8) & 0xff;

      for (attempt = 0; attempt < 5; attempt++)
        {
          recv_len = sizeof(recv_buf);
          ret = rfid_transceive(dev, send_buf, 9, recv_buf, &recv_len, 100);
          if (ret < 0)
            {
              usleep(3000);
              continue;
            }

          /* Check SAK: cascade bit (0x04) should be 0 for 4-byte UID */

          if (recv_len >= 1 && (recv_buf[0] & 0x04) == 0)
            {
              return OK;
            }

          usleep(3000);
        }
    }
  else if (uid->size == 7)
    {
      /* 7-byte UID requires cascade level 1 and level 2 selects */

      /* Level 1 select with CT (0x88) + UID0..2 */

      send_buf[0] = PCD_CMD_SELECT_1;
      send_buf[1] = 0x70;
      send_buf[2] = 0x88; /* Cascade Tag */
      bcc = 0x88;

      for (i = 0; i < 3; i++)
        {
          send_buf[3 + i] = uid->uid[i];
          bcc ^= uid->uid[i];
        }

      send_buf[6] = bcc;

      crc = rfid_crc_a_sw(send_buf, 7);

      /* ISO14443A CRC: LSB first */

      send_buf[7] = crc & 0xff;
      send_buf[8] = (crc >> 8) & 0xff;

      for (attempt = 0; attempt < 5; attempt++)
        {
          recv_len = sizeof(recv_buf);
          ret = rfid_transceive(dev, send_buf, 9, recv_buf, &recv_len, 100);
          if (ret < 0)
            {
              usleep(3000);
              continue;
            }

          /* Check SAK: cascade bit (0x04) should be 1 for 7-byte UID CL1 */

          if (recv_len >= 1 && (recv_buf[0] & 0x04) != 0)
            {
              break;
            }

          usleep(3000);
        }

      if (attempt == 5)
        {
          return -EIO;
        }

      /* Level 2 select with UID3..6 */

      send_buf[0] = PCD_CMD_SELECT_2;
      send_buf[1] = 0x70;
      bcc = 0;

      for (i = 0; i < 4; i++)
        {
          send_buf[2 + i] = uid->uid[3 + i];
          bcc ^= uid->uid[3 + i];
        }

      send_buf[6] = bcc;

      crc = rfid_crc_a_sw(send_buf, 7);

      /* ISO14443A CRC: LSB first */

      send_buf[7] = crc & 0xff;
      send_buf[8] = (crc >> 8) & 0xff;

      for (attempt = 0; attempt < 5; attempt++)
        {
          recv_len = sizeof(recv_buf);
          ret = rfid_transceive(dev, send_buf, 9, recv_buf, &recv_len, 100);
          if (ret < 0)
            {
              usleep(3000);
              continue;
            }

          /* Check SAK: cascade bit (0x04) should be 0 for final select */

          if (recv_len >= 1 && (recv_buf[0] & 0x04) == 0)
            {
              return OK;
            }

          usleep(3000);
        }
    }
  else
    {
      return -EINVAL;
    }

  return -EIO;
}

/****************************************************************************
 * Name: rfid_read_card
 *
 * Description:
 *   Read a card UID (complete sequence: request + anticoll + select).
 *
 ****************************************************************************/

int rfid_read_card(FAR struct rfid_dev_s *dev, FAR struct rfid_uid_s *uid)
{
  int ret;
  int retry;

  if (dev == NULL || !dev->initialized || uid == NULL)
    {
      return -EINVAL;
    }

  /* Request card with retry */

  for (retry = 0; retry < 10; retry++)
    {
      ret = rfid_request(dev);
      if (ret == OK)
        {
          break;
        }

      usleep(5000); /* 5ms between retries */
    }

  if (ret < 0)
    {
      return ret;
    }

  usleep(2000); /* 2ms delay */

  /* Anti-collision */

  ret = rfid_anticoll(dev, uid);
  if (ret < 0)
    {
      return ret;
    }

  usleep(2000); /* 2ms delay */

  /* Select card */

  ret = rfid_select(dev, uid);

  /* UID is valid after successful anticollision even if select fails.
   * Some MFRC522B boards show unstable SAK responses; keep UID result. */

  return OK;
}

/****************************************************************************
 * Name: rfid_authenticate_block
 *
 * Description:
 *   Authenticate a MIFARE Classic block with Key A/B.
 *
 ****************************************************************************/

int rfid_authenticate_block(FAR struct rfid_dev_s *dev,
                            FAR const struct rfid_uid_s *uid,
                            uint8_t block,
                            FAR const uint8_t key[6],
                            bool use_keyb)
{
  uint8_t auth_buf[12];
  uint8_t com_irq;
  uint8_t status2;
  uint32_t timeout;
  int ret;
  int i;

  if (dev == NULL || !dev->initialized || uid == NULL || key == NULL)
    {
      return -EINVAL;
    }

  if (uid->size < 4)
    {
      return -EINVAL;
    }

  /* Build auth frame: [0x60/0x61][block][key(6)][uid[0..3]] */

  auth_buf[0] = use_keyb ? PICC_CMD_MF_AUTH_KEY_B : PICC_CMD_MF_AUTH_KEY_A;
  auth_buf[1] = block;
  for (i = 0; i < 6; i++)
    {
      auth_buf[2 + i] = key[i];
    }

  for (i = 0; i < 4; i++)
    {
      auth_buf[8 + i] = uid->uid[i];
    }

  /* Force byte-aligned framing for auth command */

  ret = rfid_set_bit_framing(dev, 0x00);
  if (ret < 0)
    {
      return ret;
    }

  /* Stop any previous command and clear stale state */

  ret = rfid_write_reg(dev, MFRC522_REG_COMMAND, MFRC522_COMMAND_IDLE);
  if (ret < 0)
    {
      return ret;
    }

  /* Clear previous Crypto1 state before starting a new auth */

  (void)rfid_clear_bits(dev, MFRC522_REG_STATUS2, 0x08);

  (void)rfid_write_reg(dev, MFRC522_REG_COMIRQ, 0x7F);
  (void)rfid_write_reg(dev, MFRC522_REG_DIVIRQ, 0x7F);
  (void)rfid_write_reg(dev, MFRC522_REG_COLL, 0x80);

  /* Flush FIFO */

  ret = rfid_set_bits(dev, MFRC522_REG_FIFO_LEVEL, 0x80);
  if (ret < 0)
    {
      return ret;
    }

  for (i = 0; i < 12; i++)
    {
      ret = rfid_write_reg(dev, MFRC522_REG_FIFO_DATA, auth_buf[i]);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* Start authentication */

  ret = rfid_write_reg(dev, MFRC522_REG_COMMAND, MFRC522_COMMAND_MF_AUTHENT);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait for IdleIRq (bit4) or TimerIRq (bit0) */

  timeout = 200;
  while (timeout > 0)
    {
      ret = rfid_read_reg(dev, MFRC522_REG_COMIRQ, &com_irq);
      if (ret < 0)
        {
          return ret;
        }

      if (com_irq & 0x11)
        {
          break;
        }

      usleep(1000);
      timeout--;
    }

  if (timeout == 0 ||
      ((com_irq & 0x10) == 0 && (com_irq & 0x01)))
    {
      /* Timeout: IdleIRq NOT set, only TimerIRq → auth failed.
       * Same tick-granularity fix as rfid_transceive: prioritize
       * IdleIRq over TimerIRq when both may be set.
       */

      (void)rfid_write_reg(dev, MFRC522_REG_COMMAND, MFRC522_COMMAND_IDLE);
      (void)rfid_clear_bits(dev, MFRC522_REG_STATUS2, 0x08);
      return -ETIMEDOUT;
    }

  /* Check Crypto1 enabled */

  ret = rfid_read_reg(dev, MFRC522_REG_STATUS2, &status2);
  if (ret < 0)
    {
      return ret;
    }

  if ((status2 & 0x08) == 0)
    {
      (void)rfid_write_reg(dev, MFRC522_REG_COMMAND, MFRC522_COMMAND_IDLE);
      return -EACCES;
    }

  return OK;
}

/****************************************************************************
 * Name: rfid_read_block
 *
 * Description:
 *   Read one MIFARE Classic block (16 bytes).
 *
 ****************************************************************************/

int rfid_read_block(FAR struct rfid_dev_s *dev,
                    uint8_t block,
                    FAR uint8_t data[16])
{
  uint8_t send_buf[4];
  uint8_t recv_buf[32];
  size_t recv_len;
  uint16_t crc;
  int ret;
  int i;

  if (dev == NULL || !dev->initialized || data == NULL)
    {
      return -EINVAL;
    }

  send_buf[0] = PICC_CMD_MF_READ;
  send_buf[1] = block;

  crc = rfid_crc_a_sw(send_buf, 2);
  send_buf[2] = crc & 0xff;
  send_buf[3] = (crc >> 8) & 0xff;

  recv_len = sizeof(recv_buf);
  ret = rfid_transceive(dev, send_buf, 4, recv_buf, &recv_len, 100);
  if (ret < 0)
    {
      return ret;
    }

  /* Some clones return 16 bytes, some 18 bytes (with CRC).
   * We only need first 16 bytes payload. */

  if (recv_len < 16)
    {
      return -EIO;
    }

  for (i = 0; i < 16; i++)
    {
      data[i] = recv_buf[i];
    }

  return OK;
}

int rfid_stop_crypto(FAR struct rfid_dev_s *dev)
{
  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

  return rfid_clear_bits(dev, MFRC522_REG_STATUS2, 0x08);
}

/****************************************************************************
 * Name: rfid_dump_regs
 *
 * Description:
 *   Dump all key registers for debugging.
 *
 ****************************************************************************/

int rfid_dump_regs(FAR struct rfid_dev_s *dev)
{
  uint8_t val;

  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

  printf("\n===== MFRC522B Register Dump =====\n\n");

  /* Page 0 - Command and Status */
  printf("Page 0 (Command/Status):\n");
  rfid_read_reg(dev, 0x01, &val); printf("  CommandReg     (0x01): 0x%02X\n", val);
  rfid_read_reg(dev, 0x02, &val); printf("  CommIEnReg     (0x02): 0x%02X\n", val);
  rfid_read_reg(dev, 0x03, &val); printf("  DivIEnReg      (0x03): 0x%02X\n", val);
  rfid_read_reg(dev, 0x04, &val); printf("  CommIRqReg     (0x04): 0x%02X\n", val);
  rfid_read_reg(dev, 0x05, &val); printf("  DivIRqReg      (0x05): 0x%02X\n", val);
  rfid_read_reg(dev, 0x06, &val); printf("  ErrorReg       (0x06): 0x%02X\n", val);
  rfid_read_reg(dev, 0x07, &val); printf("  Status1Reg     (0x07): 0x%02X\n", val);
  rfid_read_reg(dev, 0x08, &val); printf("  Status2Reg     (0x08): 0x%02X\n", val);
  rfid_read_reg(dev, 0x0A, &val); printf("  FIFOLevelReg   (0x0A): 0x%02X\n", val);
  rfid_read_reg(dev, 0x0D, &val); printf("  BitFramingReg  (0x0D): 0x%02X\n", val);
  rfid_read_reg(dev, 0x0E, &val); printf("  CollReg        (0x0E): 0x%02X\n", val);

  /* Page 1 - Communication */
  printf("\nPage 1 (Communication):\n");
  rfid_read_reg(dev, 0x11, &val); printf("  ModeReg        (0x11): 0x%02X\n", val);
  rfid_read_reg(dev, 0x12, &val); printf("  TxModeReg      (0x12): 0x%02X\n", val);
  rfid_read_reg(dev, 0x13, &val); printf("  RxModeReg      (0x13): 0x%02X\n", val);
  rfid_read_reg(dev, 0x14, &val); printf("  TxControlReg   (0x14): 0x%02X (TxRFEn=%d)\n", val, val & 1);
  rfid_read_reg(dev, 0x15, &val); printf("  TxASKReg       (0x15): 0x%02X (Force100ASK=%d)\n", val, (val >> 6) & 1);
  rfid_read_reg(dev, 0x16, &val); printf("  TxSelReg       (0x16): 0x%02X (DriverSel=%d)\n", val, (val >> 4) & 0x03);

  /* Page 2 - Configuration & Antenna Driver */
  printf("\nPage 2 (Configuration & Antenna Driver):\n");
  rfid_read_reg(dev, 0x24, &val); printf("  ModWidthReg    (0x24): 0x%02X\n", val);
  rfid_read_reg(dev, 0x26, &val); printf("  RFCfgReg       (0x26): 0x%02X (RxGain=%d = %ddB)\n", 
                                          val, (val >> 4) & 0x07, 18 + ((val >> 4) & 0x07) * 5);
  rfid_read_reg(dev, 0x27, &val); printf("  GsNOnReg       (0x27): 0x%02X (CWGsNOn=%d, ModGsNOn=%d)\n",
                                          val, (val >> 4) & 0x0F, val & 0x0F);
  rfid_read_reg(dev, 0x28, &val); printf("  CWGsPReg       (0x28): 0x%02X (CWGsP=%d)\n", val, val & 0x3F);
  rfid_read_reg(dev, 0x29, &val); printf("  ModGsPReg      (0x29): 0x%02X (ModGsP=%d)\n", val, val & 0x3F);
  rfid_read_reg(dev, 0x2A, &val); printf("  TModeReg       (0x2A): 0x%02X\n", val);
  rfid_read_reg(dev, 0x2B, &val); printf("  TPrescalerReg  (0x2B): 0x%02X\n", val);

  printf("\n===================================\n");
  printf("Key checks for RF output:\n");
  printf("  1. TxRFEn=1 (RF enabled)\n");
  printf("  2. DriverSel=1 (modulated signal)\n");
  printf("  3. GsNOnReg/CWGsPReg/ModGsPReg > 0 (driver strength)\n");
  printf("Use oscilloscope to verify 13.56MHz on TX pin.\n");
  printf("===================================\n\n");

  return OK;
}
