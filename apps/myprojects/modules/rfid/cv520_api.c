/****************************************************************************
 * apps/myprojects/modules/rfid/cv520_api.c
 *
 * CV520 Contactless Reader IC Driver
 *
 * CV520 is register-compatible with MFRC522, supporting ISO/IEC 14443
 * Type A and MIFARE protocols via SPI interface.
 *
 * Key CV520 features (from datasheet):
 *   - 13.56MHz contactless communication
 *   - ISO/IEC 14443 Type A / MIFARE support
 *   - SPI up to 10Mbit/s
 *   - 64-byte FIFO buffer
 *   - 2.5~3.6V operating voltage
 *   - 27.12MHz crystal oscillator
 *   - Supports MIFARE Mini/1K/4K/Ultralight/DESFire EV1
 *   - Up to 848kbit/s data transfer rate
 *   - Hardware CRC and parity check
 *
 * SPI Address Format (from CV520 datasheet Section 8.1.1.3):
 *   Bit 7:    1 = read, 0 = write
 *   Bits 6:1: register address
 *   Bit 0:    0 (fixed)
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.
 *
 ****************************************************************************/
/****************************************************************************
 * CV520 的寄存器读写和射频帧。
 * 读卡之后的业务在 include/rfid_api.h 与 project1_main.c。
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
#include <time.h>

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

/* CV520 Register Definitions (from CV520 datasheet Section 9.2)
 *
 * Page 0: Command and Status Registers (0x00 - 0x0F)
 * Page 1: Communication Registers      (0x10 - 0x1F)
 * Page 2: Configuration Registers      (0x20 - 0x2F)
 * Page 3: Test Registers               (0x30 - 0x3F)
 */

/* Page 0: Command and Status */

#define CV520_REG_RESERVED00              0x00  /* Reserved */
#define CV520_REG_COMMAND                 0x01  /* Starts/stops commands */
#define CV520_REG_COMIEN                  0x02  /* Interrupt enable control */
#define CV520_REG_DIVIEN                  0x03  /* Interrupt enable control */
#define CV520_REG_COMIRQ                  0x04  /* Interrupt request bits */
#define CV520_REG_DIVIRQ                  0x05  /* Interrupt request bits */
#define CV520_REG_ERROR                   0x06  /* Error status of last cmd */
#define CV520_REG_STATUS1                 0x07  /* Communication status */
#define CV520_REG_STATUS2                 0x08  /* Receiver/transmitter status */
#define CV520_REG_FIFO_DATA               0x09  /* 64-byte FIFO I/O */
#define CV520_REG_FIFO_LEVEL              0x0A  /* Number of bytes in FIFO */
#define CV520_REG_WATER_LEVEL             0x0B  /* FIFO over/underflow warn */
#define CV520_REG_CONTROL                 0x0C  /* Miscellaneous control */
#define CV520_REG_BIT_FRAMING             0x0D  /* Bit-oriented frame adjust */
#define CV520_REG_COLL                    0x0E  /* Collision detection */
#define CV520_REG_RESERVED0F              0x0F  /* Reserved */

/* Page 1: Communication */

#define CV520_REG_RESERVED10              0x10  /* Reserved */
#define CV520_REG_MODE                    0x11  /* General mode settings */
#define CV520_REG_TX_MODE                 0x12  /* Transmit data rate */
#define CV520_REG_RX_MODE                 0x13  /* Receive data rate */
#define CV520_REG_TX_CONTROL              0x14  /* TX1/TX2 antenna control */
#define CV520_REG_TX_ASK                  0x15  /* TX modulation setting */
#define CV520_REG_TX_SEL                  0x16  /* Internal signal source sel */
#define CV520_REG_RX_SEL                  0x17  /* Internal receiver setting */
#define CV520_REG_RX_THRESHOLD            0x18  /* Bit decoder threshold */
#define CV520_REG_DEMOD                   0x19  /* Demodulator setting */
#define CV520_REG_RESERVED1A              0x1A  /* Reserved */
#define CV520_REG_RESERVED1B              0x1B  /* Reserved */
#define CV520_REG_MFTX                    0x1C  /* MIFARE TX parameters */
#define CV520_REG_MFRX                    0x1D  /* MIFARE RX parameters */
#define CV520_REG_RESERVED1E              0x1E  /* Reserved */
#define CV520_REG_RESERVED1F              0x1F  /* Reserved */

/* Page 2: Configuration */

#define CV520_REG_RESERVED20              0x20  /* Reserved */
#define CV520_REG_CRC_RESULT_H            0x21  /* CRC result MSB */
#define CV520_REG_CRC_RESULT_L            0x22  /* CRC result LSB */
#define CV520_REG_RESERVED23              0x23  /* Reserved */
#define CV520_REG_MOD_WIDTH               0x24  /* Modulation width */
#define CV520_REG_RESERVED25              0x25  /* Reserved */
#define CV520_REG_RFCFG                   0x26  /* Receiver gain config */
#define CV520_REG_GSN                     0x27  /* N-driver conductance */
#define CV520_REG_CWGSP                   0x28  /* P-driver CW conductance */
#define CV520_REG_MODGSP                  0x29  /* P-driver mod conductance */
#define CV520_REG_TMODE                   0x2A  /* Timer settings */
#define CV520_REG_TPRESCALER              0x2B  /* Timer prescaler */
#define CV520_REG_TRELOAD_H               0x2C  /* Timer reload MSB */
#define CV520_REG_TRELOAD_L               0x2D  /* Timer reload LSB */
#define CV520_REG_TCOUNTER_H              0x2E  /* Timer counter MSB */
#define CV520_REG_TCOUNTER_L              0x2F  /* Timer counter LSB */

/* Page 3: Test Registers */

#define CV520_REG_RESERVED30              0x30  /* Reserved */
#define CV520_REG_TESTSEL1                0x31  /* Test signal config */
#define CV520_REG_TESTSEL2                0x32  /* Test signal config / PRBS */
#define CV520_REG_TESTPINEN               0x33  /* D1-D7 output driver en */
#define CV520_REG_TESTPINVALUE            0x34  /* D1-D7 pin level def */
#define CV520_REG_TESTBUS                 0x35  /* Internal test bus status */
#define CV520_REG_AUTOTEST                0x36  /* Digital self-test control */
#define CV520_REG_VERSION                 0x37  /* Software version */
#define CV520_REG_ANALOGTEST              0x38  /* AUX1/AUX2 control */
#define CV520_REG_TESTDAC1                0x39  /* TestDAC1 test value */
#define CV520_REG_TESTDAC2                0x3A  /* TestDAC2 test value */
#define CV520_REG_TESTADC                 0x3B  /* ADC I/Q channel value */

/* CV520 Command Definitions (from datasheet Section 10.3)
 *
 * CommandReg[3:0] activates the corresponding command.
 */

#define CV520_CMD_IDLE                    0x00  /* No action, cancel cmd */
#define CV520_CMD_MEM                     0x01  /* Store 25 bytes to buf */
#define CV520_CMD_RANDOM_ID               0x02  /* Generate random ID */
#define CV520_CMD_CALC_CRC                0x03  /* Activate CRC coprocessor */
#define CV520_CMD_TRANSMIT                0x04  /* Transmit data from FIFO */
#define CV520_CMD_NO_CHANGE               0x07  /* No command change */
#define CV520_CMD_RECEIVE                 0x08  /* Activate receiver */
#define CV520_CMD_TRANSCEIVE              0x0C  /* Transmit + receive */
#define CV520_CMD_MF_AUTHENT              0x0E  /* MIFARE authentication */
#define CV520_CMD_SOFT_RESET              0x0F  /* Software reset */

/* Error Register Bits (ErrorReg 0x06) */

#define CV520_ERR_WRERR                   (1 << 7)  /* Write error */
#define CV520_ERR_TEMPERR                 (1 << 6)  /* Temperature error */
#define CV520_ERR_BUFFEROVFL              (1 << 4)  /* Buffer overflow */
#define CV520_ERR_COLL                    (1 << 3)  /* Collision detected */
#define CV520_ERR_CRC                     (1 << 2)  /* CRC error */
#define CV520_ERR_PARITY                  (1 << 1)  /* Parity error */
#define CV520_ERR_PROTOCOL                (1 << 0)  /* Protocol error (SOF) */

/* Status1Reg Bits */

#define CV520_STATUS1_CRC_OK              (1 << 6)
#define CV520_STATUS1_IRQ                 (1 << 4)
#define CV520_STATUS1_LO_ALERT            (1 << 0)

/* Status2Reg Bits */

#define CV520_STATUS2_MF_CRYPTO1_ON       (1 << 3)

/* SPI Address Format (Section 8.1.1.3)
 * Bit 7:    1=read, 0=write
 * Bits 6:1: register address
 * Bit 0:    0 (fixed)
 */

#define CV520_SPI_READ                    0x80
#define CV520_SPI_WRITE                   0x00

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

#define CV520_TIMEOUT_MS                  1000
#define CV520_DELAY_MS                    10

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rfid_dev_s
{
  struct spi_dev_s *spi;      /* SPI device */
  int rstpin;                 /* Reset GPIO pin (-1 if not used) */
  bool initialized;           /* Initialization flag */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int cv520_spi_read(FAR struct rfid_dev_s *dev, uint8_t reg,
                           FAR uint8_t *data, size_t len);
static int cv520_spi_write(FAR struct rfid_dev_s *dev, uint8_t reg,
                            FAR const uint8_t *data, size_t len);
static int cv520_write_reg(FAR struct rfid_dev_s *dev, uint8_t reg,
                            uint8_t value);
static int cv520_read_reg(FAR struct rfid_dev_s *dev, uint8_t reg,
                           FAR uint8_t *value);
static int cv520_set_bits(FAR struct rfid_dev_s *dev, uint8_t reg,
                           uint8_t mask);
static int cv520_clear_bits(FAR struct rfid_dev_s *dev, uint8_t reg,
                             uint8_t mask);
static int cv520_set_bit_framing(FAR struct rfid_dev_s *dev,
                                  uint8_t tx_last_bits);
static int cv520_soft_reset(FAR struct rfid_dev_s *dev);
static int cv520_init_chip(FAR struct rfid_dev_s *dev);
static int cv520_transceive(FAR struct rfid_dev_s *dev,
                             FAR const uint8_t *send, size_t send_len,
                             FAR uint8_t *recv, FAR size_t *recv_len,
                             uint32_t timeout_ms);
static int cv520_calculate_crc(FAR struct rfid_dev_s *dev,
                                FAR const uint8_t *data, size_t len,
                                FAR uint16_t *crc) __attribute__((unused));
static uint16_t cv520_crc_a_sw(FAR const uint8_t *data, size_t len);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cv520_elapsed_ms
 *
 * Description:
 *   Returns milliseconds elapsed since the given start timestamp.
 *   Used to implement real-time-based timeout polling instead of relying
 *   on usleep() count (which is inaccurate due to NuttX tick granularity).
 *
 ****************************************************************************/

static uint32_t cv520_elapsed_ms(FAR const struct timespec *start)
{
  struct timespec now;

  clock_gettime(CLOCK_MONOTONIC, &now);
  return (uint32_t)((now.tv_sec - start->tv_sec) * 1000
                  + (now.tv_nsec - start->tv_nsec) / 1000000);
}

/****************************************************************************
 * Name: cv520_spi_read
 *
 * Description:
 *   Read data from CV520 via SPI.
 *   CV520 SPI address format (Section 8.1.1.3):
 *     bit7=1(read), bits[6:1]=register address, bit0=0
 *   Data is clocked MSB first (Section 8.1.1.1).
 *
 ****************************************************************************/

static int cv520_spi_read(FAR struct rfid_dev_s *dev, uint8_t reg,
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
      /* CV520 SPI read: bit7=1, bits[6:1]=register, bit0=0 */

      addr = CV520_SPI_READ | ((reg & 0x3f) << 1);
      SPI_SEND(spi, addr);
      data[i] = SPI_SEND(spi, 0x00);
    }

  SPI_SELECT(spi, SPIDEV_CONTACTLESS(0), false);
  SPI_LOCK(spi, false);

  return OK;
}

/****************************************************************************
 * Name: cv520_spi_write
 *
 * Description:
 *   Write data to CV520 via SPI.
 *   CV520 SPI address format (Section 8.1.1.3):
 *     bit7=0(write), bits[6:1]=register address, bit0=0
 *   Data is clocked MSB first (Section 8.1.1.2).
 *
 ****************************************************************************/

static int cv520_spi_write(FAR struct rfid_dev_s *dev, uint8_t reg,
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
      /* CV520 SPI write: bit7=0, bits[6:1]=register, bit0=0 */

      addr = CV520_SPI_WRITE | ((reg & 0x3f) << 1);
      SPI_SEND(spi, addr);
      SPI_SEND(spi, data[i]);
    }

  SPI_SELECT(spi, SPIDEV_CONTACTLESS(0), false);
  SPI_LOCK(spi, false);

  return OK;
}

/****************************************************************************
 * Name: cv520_write_reg / cv520_read_reg
 ****************************************************************************/

static int cv520_write_reg(FAR struct rfid_dev_s *dev, uint8_t reg,
                            uint8_t value)
{
  return cv520_spi_write(dev, reg, &value, 1);
}

static int cv520_read_reg(FAR struct rfid_dev_s *dev, uint8_t reg,
                           FAR uint8_t *value)
{
  return cv520_spi_read(dev, reg, value, 1);
}

/****************************************************************************
 * Name: cv520_set_bits / cv520_clear_bits
 ****************************************************************************/

static int cv520_set_bits(FAR struct rfid_dev_s *dev, uint8_t reg,
                           uint8_t mask)
{
  uint8_t value;
  int ret;

  ret = cv520_read_reg(dev, reg, &value);
  if (ret < 0)
    {
      return ret;
    }

  value |= mask;
  return cv520_write_reg(dev, reg, value);
}

static int cv520_clear_bits(FAR struct rfid_dev_s *dev, uint8_t reg,
                             uint8_t mask)
{
  uint8_t value;
  int ret;

  ret = cv520_read_reg(dev, reg, &value);
  if (ret < 0)
    {
      return ret;
    }

  value &= ~mask;
  return cv520_write_reg(dev, reg, value);
}

/****************************************************************************
 * Name: cv520_set_bit_framing
 *
 * Description:
 *   Configure BitFramingReg (0x0D) TxLastBits[2:0].
 *   Per CV520 datasheet Section 9.3.1.14.
 *
 ****************************************************************************/

static int cv520_set_bit_framing(FAR struct rfid_dev_s *dev,
                                  uint8_t tx_last_bits)
{
  uint8_t value;
  int ret;

  ret = cv520_read_reg(dev, CV520_REG_BIT_FRAMING, &value);
  if (ret < 0)
    {
      return ret;
    }

  value &= ~0x07;
  value |= (tx_last_bits & 0x07);
  return cv520_write_reg(dev, CV520_REG_BIT_FRAMING, value);
}

/****************************************************************************
 * Name: cv520_soft_reset
 *
 * Description:
 *   Perform a soft reset of CV520.
 *   Per CV520 datasheet Section 8.6.2:
 *     - Set CommandReg Command[3:0] = 0x0F (SoftReset)
 *     - Wait for PowerDown bit (bit4) to clear
 *     - After reset, 1024 clock cycles needed before ready
 *
 ****************************************************************************/

static int cv520_soft_reset(FAR struct rfid_dev_s *dev)
{
  uint8_t value;
  int ret;

  /* Read current CommandReg to preserve upper bits */

  ret = cv520_read_reg(dev, CV520_REG_COMMAND, &value);
  if (ret < 0)
    {
      return ret;
    }

  value &= ~0x0f;
  value |= CV520_CMD_SOFT_RESET;
  ret = cv520_write_reg(dev, CV520_REG_COMMAND, value);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait for PowerDown bit (bit 4) to clear per Section 8.6.2 */

  usleep(1000); /* 1ms initial delay */

  {
    struct timespec ts_start;
    clock_gettime(CLOCK_MONOTONIC, &ts_start);

    while (cv520_elapsed_ms(&ts_start) < CV520_TIMEOUT_MS)
      {
        ret = cv520_read_reg(dev, CV520_REG_COMMAND, &value);
        if (ret < 0)
          {
            return ret;
          }

        if ((value & 0x10) == 0)  /* PowerDown bit cleared */
          {
            return OK;
          }

        usleep(1000);
      }
  }

  printf("CV520: soft_reset TIMEOUT\n");
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: cv520_init_chip
 *
 * Description:
 *   Initialize CV520 chip with settings for ISO14443A/MIFARE operation.
 *
 *   Register configuration based on CV520 datasheet:
 *     - ModeReg (0x11): CRC preset 6363h (ISO14443A)
 *     - TxModeReg (0x12): 106kbps
 *     - RxModeReg (0x13): 106kbps
 *     - TxControlReg (0x14): Enable TX1+TX2 antenna drivers
 *     - TxASKReg (0x15): Force 100% ASK modulation
 *     - RFCfgReg (0x26): Maximum receiver gain
 *     - GsNReg (0x27): N-driver conductance
 *     - CWGsPReg (0x28): P-driver CW conductance
 *     - ModGsPReg (0x29): P-driver modulation conductance
 *     - TModeReg (0x2A) / TPrescalerReg (0x2B): Timer config
 *     - TReloadReg (0x2C-0x2D): Timer reload value
 *
 ****************************************************************************/

static int cv520_write_reg_verify(FAR struct rfid_dev_s *dev,
                                   uint8_t reg, uint8_t value,
                                   FAR const char *name)
{
  uint8_t readback;
  int ret;

  ret = cv520_write_reg(dev, reg, value);
  if (ret < 0)
    {
      printf("CV520: WRITE FAIL %s (0x%02X) = 0x%02X\n", name, reg, value);
      return ret;
    }

  ret = cv520_read_reg(dev, reg, &readback);
  if (ret < 0)
    {
      printf("CV520: READBACK FAIL %s (0x%02X)\n", name, reg);
      return ret;
    }

  if (readback != value)
    {
      printf("CV520: VERIFY FAIL %s (0x%02X): wrote 0x%02X, read 0x%02X\n",
             name, reg, value, readback);
    }

  return OK;
}

static int cv520_init_chip(FAR struct rfid_dev_s *dev)
{
  uint8_t version;
  uint8_t val;
  int ret;
  int retry;

  ret = cv520_soft_reset(dev);
  if (ret < 0)
    {
      printf("CV520: soft reset failed (%d)\n", ret);
    }

  usleep(5000); /* 5ms for oscillator (spec: ~38us + margin) */

  version = 0x00;
  for (retry = 0; retry < 3; retry++)
    {
      ret = cv520_read_reg(dev, CV520_REG_VERSION, &version);
      if (ret == OK && version != 0x00)
        {
          break;
        }

      usleep(5000);
    }

  if (version == 0x00)
    {
      cv520_write_reg(dev, CV520_REG_COMMAND, 0x0F);
      usleep(50000);

      cv520_read_reg(dev, CV520_REG_VERSION, &version);
    }

  if (version == 0x00 || version == 0xff)
    {
      printf("CV520: ERROR - chip not detected (ver=0x%02X)\n", version);
      return -ENODEV;
    }

  /* BitFramingReg: default 8-bit framing */

  cv520_write_reg_verify(dev, CV520_REG_BIT_FRAMING, 0x00,
                         "BitFramingReg");

  /* ModeReg (0x11): 0x3F
   * - TxWaitRF=1, CRCPreset=11 (6363h for ISO14443A)
   */

  cv520_write_reg_verify(dev, CV520_REG_MODE, 0x3F, "ModeReg");

  /* TxModeReg: 106kbps */

  cv520_write_reg_verify(dev, CV520_REG_TX_MODE, 0x00, "TxModeReg");

  /* RxModeReg: 106kbps */

  cv520_write_reg_verify(dev, CV520_REG_RX_MODE, 0x00, "RxModeReg");

  /* ModWidthReg: Modulation width */

  cv520_write_reg_verify(dev, CV520_REG_MOD_WIDTH, 0x26, "ModWidthReg");

  /* TxASKReg: Force 100% ASK modulation (bit6=1) */

  cv520_write_reg_verify(dev, CV520_REG_TX_ASK, 0x40, "TxASKReg");

  /* TxSelReg: DriverSel=01 (modulated envelope) */

  cv520_write_reg_verify(dev, CV520_REG_TX_SEL, 0x10, "TxSelReg");

  /* Ensure Crypto1 is OFF */

  cv520_clear_bits(dev, CV520_REG_STATUS2, CV520_STATUS2_MF_CRYPTO1_ON);

  /* GsNReg: N-driver max conductance
   * 0xFF = maximum conductance (ensures complete ground connection for 100% ASK)
   * Configuration: CWGsPReg=0x3F, ModGsPReg=0x00, GsNReg=0xFF
   * Combined with ModGsPReg=0x00, this should achieve true 100% ASK modulation depth
   */

  cv520_write_reg_verify(dev, CV520_REG_GSN, 0xFF, "GsNReg");

  /* CWGsPReg: P-driver CW conductance (0x3F = max, per NXP test conditions) */

  cv520_write_reg_verify(dev, CV520_REG_CWGSP, 0x3F, "CWGsPReg");

  /* ModGsPReg: P-driver modulation minimum conductance (testing)
   * 0x00 = minimum conductance (P-driver off during modulation)
   * Testing: With GsNReg=0x80, modulation level was 2.61V (modulation depth ~21%)
   * Setting ModGsPReg=0x00 should allow N-driver to fully pull down to 0V
   * This should achieve true 100% ASK modulation depth
   */

  cv520_write_reg_verify(dev, CV520_REG_MODGSP, 0x20, "ModGsPReg");

  /* Receiver settings - write chip power-on defaults explicitly
   * CV520 datasheet shows these reserved registers have specific reset values
   * that may affect analog behavior. Write them to ensure correctness.
   */

  /* RxSelReg (0x17): reset value = 0x84 (UARTSel=10 internal analog) */

  cv520_write_reg(dev, 0x17, 0x84);

  /* RxThresholdReg (0x18): bit decoder 最小门限 (MinLevel/CollLevel)
   * 与 RFCfg(接收增益) 的直觉关系(经验):
   * - 弱信号/远距: 更低 MinLevel(更敏感) 常有助于抓弱回波，但抗噪更差
   * - 强场/高增益(易过载/饱和): 往往需要更高 MinLevel(更“钝”) 以压制噪声/误码
   * 不保证“gain↑ 就 threshold↑(或↓)”; 以整机实测为准
   * 稳定基线: 0x64
   * MinLevel=6 (0x60), CollLevel=4 (0x04)
   */

  cv520_write_reg(dev, 0x18, 0x44);

  /* DemodReg (0x19): reset value = 0x4D */

  cv520_write_reg(dev, 0x19, 0x4D);

  /* Reserved (0x23): reset value = 0x88 - may affect analog config! */

  cv520_write_reg(dev, 0x23, 0x88);

  /* Reserved (0x25): reset value = 0x87 - may affect analog config! */

  cv520_write_reg(dev, 0x25, 0x87);

  /* RFCfgReg: 稳定基线
   * 0x40 = 33dB. 0x50 在整机上天线/耦合下反而整体变差(常见为过载/饱和，不是简单“阈值+1 档能救”)
   * 如以后再试 0x50, 可同步微调 RxThresholdReg (MinLevel) ——见上方注释。
   */

  cv520_write_reg_verify(dev, CV520_REG_RFCFG, 0x58, "RFCfgReg");

  /* AutoTestReg (0x36) bit6 = AmpRcv: non-linear receiver processing.
   * NXP datasheet: "increases the operating distance at 106 kbit/s" */

  cv520_write_reg(dev, CV520_REG_AUTOTEST, 0x40);

  /* Timer configuration
   *
   * Old values (TModeReg=0x8D, TPrescaler=0x3E, TReload=0x0FFF) gave
   * a ~2 second timeout per failed REQA — way too slow for card detection!
   *
   * New values for ~25ms timeout:
   *   TPrescaler = 0x00A9 = 169
   *   TReload    = 0x03E7 = 999
   *   td = (2*169+1)*(999+1)/13.56MHz = 339*1000/13.56MHz ≈ 25ms
   *
   * TAuto=1 (bit7): timer starts automatically with Transceive command
   */

  cv520_write_reg_verify(dev, CV520_REG_TMODE, 0x80, "TModeReg");
  cv520_write_reg_verify(dev, CV520_REG_TPRESCALER, 0xA9, "TPrescalerReg");
  cv520_write_reg_verify(dev, CV520_REG_TRELOAD_H, 0x03, "TReloadH");
  cv520_write_reg_verify(dev, CV520_REG_TRELOAD_L, 0xE7, "TReloadL");

  /* Step 5: Enable antenna - TxControlReg (0x14)
   * NXP MFRC522 reset value keeps InvTx2RFOn set (0x80). Enable both
   * antenna drivers on top of that, yielding complementary TX1/TX2 drive.
   *   bit7: InvTx2RFOn = 1 (NXP reset/default complementary side)
   *   bit1: Tx2RFEn = 1    (enable TX2 antenna driver)
   *   bit0: Tx1RFEn = 1    (enable TX1 antenna driver)
   *   0x83 = NXP-aligned differential TX1+TX2
   */

  cv520_write_reg_verify(dev, CV520_REG_TX_CONTROL, 0x83, "TxControlReg");

  cv520_read_reg(dev, CV520_REG_TX_CONTROL, &val);
  if ((val & 0x83) != 0x83)
    {
      cv520_set_bits(dev, CV520_REG_TX_CONTROL, 0x83);
    }

  return OK;
}

/****************************************************************************
 * Name: cv520_transceive
 *
 * Description:
 *   Transmit data from FIFO and receive response via CV520.
 *   Uses Transceive command (0x0C) per CV520 datasheet Section 10.3.
 *
 *   Flow:
 *     1. Set Idle command
 *     2. Flush FIFO
 *     3. Write TX data to FIFO
 *     4. Clear interrupt flags
 *     5. Start Transceive command
 *     6. Set StartSend bit in BitFramingReg
 *     7. Wait for RxIRq or TimerIRq
 *     8. Check errors
 *     9. Read received data from FIFO
 *
 ****************************************************************************/

/* Debug flag for transceive - set by txtest command */

static bool g_cv520_trx_debug = false;

static int cv520_transceive(FAR struct rfid_dev_s *dev,
                             FAR const uint8_t *send, size_t send_len,
                             FAR uint8_t *recv, FAR size_t *recv_len,
                             uint32_t timeout_ms)
{
  uint8_t error;
  uint8_t fifo_level;
  uint8_t com_irq;
  uint8_t val;
  size_t i;
  int ret;

  if (send_len > 64)
    {
      return -EINVAL;
    }

  /* Set command to idle */

  ret = cv520_write_reg(dev, CV520_REG_COMMAND, CV520_CMD_IDLE);
  if (ret < 0)
    {
      return ret;
    }

  /* Clear collision register */

  (void)cv520_write_reg(dev, CV520_REG_COLL, 0x80);

  /* Flush FIFO (set FlushBuffer bit in FIFOLevelReg) */

  ret = cv520_set_bits(dev, CV520_REG_FIFO_LEVEL, 0x80);
  if (ret < 0)
    {
      return ret;
    }

  /* Clear pending CRC interrupt flags */

  ret = cv520_write_reg(dev, CV520_REG_DIVIRQ, 0x7F);
  if (ret < 0)
    {
      return ret;
    }

  /* Write data to FIFO */

  for (i = 0; i < send_len; i++)
    {
      ret = cv520_write_reg(dev, CV520_REG_FIFO_DATA, send[i]);
      if (ret < 0)
        {
          return ret;
        }
    }

  if (g_cv520_trx_debug)
    {
      cv520_read_reg(dev, CV520_REG_FIFO_LEVEL, &val);
      cv520_read_reg(dev, CV520_REG_BIT_FRAMING, &fifo_level);
      printf("  TRX: FIFO=%d BitFrame=0x%02X send[0]=0x%02X len=%d\n",
             val & 0x7f, fifo_level, send[0], (int)send_len);
    }

  /* Clear all interrupt flags (Set1=0, writing 1 to bits 6:0 clears) */

  ret = cv520_write_reg(dev, CV520_REG_COMIRQ, 0x7F);
  if (ret < 0)
    {
      return ret;
    }

  ret = cv520_write_reg(dev, CV520_REG_DIVIRQ, 0x7F);
  if (ret < 0)
    {
      return ret;
    }

  /* Enable interrupts: TxIRq, RxIRq, IdleIRq, HiAlertIRq, LoAlertIRq,
   *                     ErrIRq, TimerIRq */

  ret = cv520_write_reg(dev, CV520_REG_COMIEN, 0x77);
  if (ret < 0)
    {
      return ret;
    }

  /* Start Transceive command */

  ret = cv520_write_reg(dev, CV520_REG_COMMAND, CV520_CMD_TRANSCEIVE);
  if (ret < 0)
    {
      return ret;
    }

  /* Set StartSend bit (bit7) in BitFramingReg to begin transmission */

  ret = cv520_set_bits(dev, CV520_REG_BIT_FRAMING, 0x80);
  if (ret < 0)
    {
      return ret;
    }

  if (g_cv520_trx_debug)
    {
      cv520_read_reg(dev, CV520_REG_COMMAND, &val);
      printf("  TRX: Cmd=0x%02X (expect 0x0C) StartSend set\n", val);
      cv520_read_reg(dev, CV520_REG_STATUS2, &val);
      printf("  TRX: Status2=0x%02X ModemState=%d\n", val, val & 0x07);
    }

  /* Wait for RxIRq (bit5) or TimerIRq (bit0) */

  com_irq = 0;
  {
    struct timespec ts_start;
    bool got_irq = false;

    clock_gettime(CLOCK_MONOTONIC, &ts_start);

    while (cv520_elapsed_ms(&ts_start) < timeout_ms)
      {
        ret = cv520_read_reg(dev, CV520_REG_COMIRQ, &com_irq);
        if (ret < 0)
          {
            return ret;
          }

        if (com_irq & 0x21)  /* RxIRq | TimerIRq */
          {
            got_irq = true;
            break;
          }

        usleep(1000);
      }

    if (g_cv520_trx_debug)
      {
        cv520_read_reg(dev, CV520_REG_ERROR, &val);
        printf("  TRX: ComIRq=0x%02X Error=0x%02X\n", com_irq, val);
      }

    if (!got_irq)
      {
        cv520_write_reg(dev, CV520_REG_COMMAND, CV520_CMD_IDLE);
        return -ETIMEDOUT;
      }
  }

  if ((com_irq & 0x20) == 0 && (com_irq & 0x01))
    {
      /* Timer fired but NO RxIRq → genuine timeout */

      cv520_write_reg(dev, CV520_REG_COMMAND, CV520_CMD_IDLE);
      return -ETIMEDOUT;
    }

  /* Clear StartSend bit */

  ret = cv520_clear_bits(dev, CV520_REG_BIT_FRAMING, 0x80);
  if (ret < 0)
    {
      return ret;
    }

  /* Read error register */

  ret = cv520_read_reg(dev, CV520_REG_ERROR, &error);
  if (ret < 0)
    {
      return ret;
    }

  /* Check for BufferOverflow - fatal error */

  if (error & CV520_ERR_BUFFEROVFL)
    {
      cv520_write_reg(dev, CV520_REG_COMMAND, CV520_CMD_IDLE);
      return -EIO;
    }

  /* Read FIFO level */

  ret = cv520_read_reg(dev, CV520_REG_FIFO_LEVEL, &fifo_level);
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
          ret = cv520_read_reg(dev, CV520_REG_FIFO_DATA, &recv[i]);
          if (ret < 0)
            {
              return ret;
            }
        }

      *recv_len = read_len;
    }

  /* Set command to idle */

  ret = cv520_write_reg(dev, CV520_REG_COMMAND, CV520_CMD_IDLE);
  if (ret < 0)
    {
      return ret;
    }

  /* Check protocol error */

  if (error & CV520_ERR_PROTOCOL)
    {
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Name: cv520_calculate_crc
 *
 * Description:
 *   Calculate CRC using CV520 hardware CRC coprocessor.
 *   Per Section 8.2.5: polynomial x^16+x^12+x^5+1, preset 6363h.
 *
 ****************************************************************************/

static int cv520_calculate_crc(FAR struct rfid_dev_s *dev,
                                FAR const uint8_t *data, size_t len,
                                FAR uint16_t *crc)
{
  uint8_t command;
  uint8_t div_irq;
  size_t i;
  int ret;

  /* Set command to idle */

  ret = cv520_write_reg(dev, CV520_REG_COMMAND, CV520_CMD_IDLE);
  if (ret < 0)
    {
      return ret;
    }

  /* Flush FIFO */

  ret = cv520_set_bits(dev, CV520_REG_FIFO_LEVEL, 0x80);
  if (ret < 0)
    {
      return ret;
    }

  /* Write data to FIFO */

  for (i = 0; i < len; i++)
    {
      ret = cv520_write_reg(dev, CV520_REG_FIFO_DATA, data[i]);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* Start CRC calculation */

  ret = cv520_write_reg(dev, CV520_REG_COMMAND, CV520_CMD_CALC_CRC);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait for CRCIRq (bit2 in DivIrqReg) */

  {
    struct timespec ts_start;
    bool got_irq = false;

    clock_gettime(CLOCK_MONOTONIC, &ts_start);

    while (cv520_elapsed_ms(&ts_start) < 1000)
      {
        ret = cv520_read_reg(dev, CV520_REG_DIVIRQ, &div_irq);
        if (ret < 0)
          {
            return ret;
          }

        if (div_irq & 0x04) /* CRC interrupt */
          {
            got_irq = true;
            break;
          }

        usleep(1000);
      }

    if (!got_irq)
      {
        return -ETIMEDOUT;
      }
  }

  /* Read CRC result from CRCResultReg (0x21=MSB, 0x22=LSB) */

  ret = cv520_read_reg(dev, CV520_REG_CRC_RESULT_L, &command);
  if (ret < 0)
    {
      return ret;
    }

  *crc = command;

  ret = cv520_read_reg(dev, CV520_REG_CRC_RESULT_H, &command);
  if (ret < 0)
    {
      return ret;
    }

  *crc |= (command << 8);

  /* Set command to idle */

  ret = cv520_write_reg(dev, CV520_REG_COMMAND, CV520_CMD_IDLE);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: cv520_crc_a_sw
 *
 * Description:
 *   Software CRC_A (ISO14443A, polynomial 0x8408, init 0x6363)
 *
 ****************************************************************************/

static uint16_t cv520_crc_a_sw(FAR const uint8_t *data, size_t len)
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
 *   Register the CV520 RFID device and initialize it.
 *   CV520 supports SPI up to 10Mbit/s (Section 8.1.1).
 *   Uses SPI Mode 0 (CPOL=0, CPHA=0) per timing spec Section 14.1.
 *
 ****************************************************************************/

FAR struct rfid_dev_s *rfid_register(FAR struct spi_dev_s *spi, int rstpin)
{
  FAR struct rfid_dev_s *dev;
  int ret;

  if (spi == NULL)
    {
      return NULL;
    }

  dev = (FAR struct rfid_dev_s *)kmm_zalloc(sizeof(struct rfid_dev_s));
  if (dev == NULL)
    {
      return NULL;
    }

  dev->spi = spi;
  dev->rstpin = rstpin;

  SPI_SETFREQUENCY(spi, 500000);
  SPI_SETMODE(spi, SPIDEV_MODE0);
  SPI_SETBITS(spi, 8);

  /* Handle NRSTPD (reset/power-down) pin
   * Per CV520 datasheet Section 8.8.1:
   *   - Reset signal low for at least 100ns to trigger reset
   *   - Reset triggered on rising edge of NRSTPD
   *   - After rising edge, wait for oscillator startup (Tstartup)
   *     plus 1024 clock cycles (~38us)
   */

  if (rstpin != 0)
    {
      stm32_configgpio(rstpin);

      stm32_gpiowrite(rstpin, false);
      usleep(1000);   /* 1ms low (spec: >100ns) */

      stm32_gpiowrite(rstpin, true);
      usleep(50000);  /* 50ms for crystal startup */
    }

  /* Initialize chip */

  ret = cv520_init_chip(dev);
  if (ret < 0)
    {
      printf("CV520: ERROR chip init failed (%d)\n", ret);
      kmm_free(dev);
      return NULL;
    }

  dev->initialized = true;
  printf("CV520: OK\n");

  return dev;
}

/****************************************************************************
 * Name: rfid_unregister
 ****************************************************************************/

int rfid_unregister(FAR struct rfid_dev_s *dev)
{
  if (dev == NULL)
    {
      return -EINVAL;
    }

  /* Pull NRSTPD low to enter hard power-down (Section 8.6.1) */

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
 *   Send REQA command to detect ISO14443A cards.
 *   REQA (0x26) is a 7-bit short frame per ISO14443-3.
 *   Expected response: 2-byte ATQA.
 *   Simplified: Only REQA, no WUPA fallback (for stable waveform).
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

  ret = cv520_clear_bits(dev, CV520_REG_STATUS2,
                         CV520_STATUS2_MF_CRYPTO1_ON);
  if (ret < 0)
    {
      return ret;
    }

  /* Send WUPA command (wakes IDLE + HALT + some ACTIVE cards) */

  send_buf[0] = PCD_CMD_WUPA;
  recv_len = sizeof(recv_buf);

  /* WUPA is a 7-bit command, set TxLastBits=7 */

  ret = cv520_set_bit_framing(dev, 0x07);
  if (ret < 0)
    {
      return ret;
    }

  ret = cv520_transceive(dev, send_buf, 1, recv_buf, &recv_len, 100);

  ret2 = cv520_set_bit_framing(dev, 0x00);
  if (ret == OK && ret2 < 0)
    {
      ret = ret2;
    }

  (void)cv520_read_reg(dev, CV520_REG_ERROR, &error);

  /* ATQA must be 2 bytes */

  if (ret == OK && recv_len >= 2 &&
      (error & (CV520_ERR_CRC | CV520_ERR_PARITY |
                CV520_ERR_PROTOCOL)) == 0)
    {
      return OK;
    }

  /* REQA failed — no WUPA fallback here to keep TX waveform clean.
   * WUPA is only used inside p1_try_read_block retry paths. */

  return -EIO;
}

/****************************************************************************
 * Name: rfid_anticoll
 *
 * Description:
 *   Perform ISO14443-3 anti-collision to get card UID.
 *   Supports 4-byte and 7-byte (cascade) UIDs.
 *
 ****************************************************************************/

int rfid_anticoll(FAR struct rfid_dev_s *dev, FAR struct rfid_uid_s *uid)
{
  uint8_t send_buf[2];
  uint8_t recv_buf[5];
  size_t recv_len;
  int ret;
  int i;
  uint8_t bcc;
  int attempt;

  if (dev == NULL || !dev->initialized || uid == NULL)
    {
      return -EINVAL;
    }

  /* Ensure 8-bit framing for anticollision */

  ret = cv520_set_bit_framing(dev, 0x00);
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
      ret = cv520_transceive(dev, send_buf, 2, recv_buf, &recv_len, 100);

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
          /* Cascade tag detected, UID is 7 bytes */

          for (i = 0; i < 3; i++)
            {
              uid->uid[i] = recv_buf[i + 1];
            }

          /* Cascade Level 2 anticollision */

          send_buf[0] = PCD_CMD_ANTICOLL_2;
          send_buf[1] = 0x20;
          recv_len = sizeof(recv_buf);

          ret = cv520_transceive(dev, send_buf, 2, recv_buf, &recv_len,
                                 100);
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
 *   Select a card by its UID (ISO14443-3 SELECT command).
 *
 ****************************************************************************/

int rfid_select(FAR struct rfid_dev_s *dev, FAR const struct rfid_uid_s *uid)
{
  uint8_t send_buf[12];
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

  ret = cv520_set_bit_framing(dev, 0x00);
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

      crc = cv520_crc_a_sw(send_buf, 7);
      send_buf[7] = crc & 0xff;
      send_buf[8] = (crc >> 8) & 0xff;

      for (attempt = 0; attempt < 5; attempt++)
        {
          recv_len = sizeof(recv_buf);
          ret = cv520_transceive(dev, send_buf, 9, recv_buf, &recv_len,
                                 100);
          if (ret < 0)
            {
              usleep(3000);
              continue;
            }

          if (recv_len >= 1 && (recv_buf[0] & 0x04) == 0)
            {
              return OK;
            }

          usleep(3000);
        }
    }
  else if (uid->size == 7)
    {
      /* 7-byte UID: cascade level 1 and 2 */

      /* Level 1 select with CT (0x88) */

      send_buf[0] = PCD_CMD_SELECT_1;
      send_buf[1] = 0x70;
      send_buf[2] = 0x88;
      bcc = 0x88;

      for (i = 0; i < 3; i++)
        {
          send_buf[3 + i] = uid->uid[i];
          bcc ^= uid->uid[i];
        }

      send_buf[6] = bcc;

      crc = cv520_crc_a_sw(send_buf, 7);
      send_buf[7] = crc & 0xff;
      send_buf[8] = (crc >> 8) & 0xff;

      for (attempt = 0; attempt < 5; attempt++)
        {
          recv_len = sizeof(recv_buf);
          ret = cv520_transceive(dev, send_buf, 9, recv_buf, &recv_len,
                                 100);
          if (ret < 0)
            {
              usleep(3000);
              continue;
            }

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

      crc = cv520_crc_a_sw(send_buf, 7);
      send_buf[7] = crc & 0xff;
      send_buf[8] = (crc >> 8) & 0xff;

      for (attempt = 0; attempt < 5; attempt++)
        {
          recv_len = sizeof(recv_buf);
          ret = cv520_transceive(dev, send_buf, 9, recv_buf, &recv_len,
                                 100);
          if (ret < 0)
            {
              usleep(3000);
              continue;
            }

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
 *   Read card UID (complete: request + anticoll + select).
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

  /* Request card with retry
   * Each rfid_request sends REQA+WUPA, hardware timer ~25ms per attempt.
   * 10 retries × ~50ms = ~500ms total polling window.
   */

  for (retry = 0; retry < 10; retry++)
    {
      ret = rfid_request(dev);
      if (ret == OK)
        {
          break;
        }

      usleep(5000); /* 5ms between retries (was 30ms) */
    }

  if (ret < 0)
    {
      return ret;
    }

  usleep(2000);

  /* Anti-collision */

  ret = rfid_anticoll(dev, uid);
  if (ret < 0)
    {
      return ret;
    }

  usleep(2000);

  /* Select card */

  ret = rfid_select(dev, uid);

  /* UID valid after anticollision even if select fails */

  return OK;
}

/****************************************************************************
 * Name: rfid_authenticate_block
 *
 * Description:
 *   Authenticate MIFARE Classic block with Key A/B.
 *   Uses MFAuthent command (0x0E) per CV520 datasheet.
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

  /* Force byte-aligned framing */

  ret = cv520_set_bit_framing(dev, 0x00);
  if (ret < 0)
    {
      return ret;
    }

  /* Stop previous command and clear state */

  ret = cv520_write_reg(dev, CV520_REG_COMMAND, CV520_CMD_IDLE);
  if (ret < 0)
    {
      return ret;
    }

  /* Clear previous Crypto1 state */

  (void)cv520_clear_bits(dev, CV520_REG_STATUS2,
                         CV520_STATUS2_MF_CRYPTO1_ON);

  (void)cv520_write_reg(dev, CV520_REG_COMIRQ, 0x7F);
  (void)cv520_write_reg(dev, CV520_REG_DIVIRQ, 0x7F);
  (void)cv520_write_reg(dev, CV520_REG_COLL, 0x80);

  /* Flush FIFO */

  ret = cv520_set_bits(dev, CV520_REG_FIFO_LEVEL, 0x80);
  if (ret < 0)
    {
      return ret;
    }

  for (i = 0; i < 12; i++)
    {
      ret = cv520_write_reg(dev, CV520_REG_FIFO_DATA, auth_buf[i]);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* Start authentication */

  ret = cv520_write_reg(dev, CV520_REG_COMMAND, CV520_CMD_MF_AUTHENT);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait for IdleIRq (bit4) or TimerIRq (bit0) */

  com_irq = 0;
  {
    struct timespec ts_start;
    bool got_irq = false;

    clock_gettime(CLOCK_MONOTONIC, &ts_start);

    while (cv520_elapsed_ms(&ts_start) < 200)
      {
        ret = cv520_read_reg(dev, CV520_REG_COMIRQ, &com_irq);
        if (ret < 0)
          {
            return ret;
          }

        if (com_irq & 0x11)
          {
            got_irq = true;
            break;
          }

        usleep(1000);
      }

    if (!got_irq ||
        ((com_irq & 0x10) == 0 && (com_irq & 0x01)))
      {
        (void)cv520_write_reg(dev, CV520_REG_COMMAND, CV520_CMD_IDLE);
        (void)cv520_clear_bits(dev, CV520_REG_STATUS2,
                               CV520_STATUS2_MF_CRYPTO1_ON);
        return -ETIMEDOUT;
      }
  }

  /* Check Crypto1 enabled (MFCrypto1On bit in Status2Reg) */

  ret = cv520_read_reg(dev, CV520_REG_STATUS2, &status2);
  if (ret < 0)
    {
      return ret;
    }

  if ((status2 & CV520_STATUS2_MF_CRYPTO1_ON) == 0)
    {
      (void)cv520_write_reg(dev, CV520_REG_COMMAND, CV520_CMD_IDLE);
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

  crc = cv520_crc_a_sw(send_buf, 2);
  send_buf[2] = crc & 0xff;
  send_buf[3] = (crc >> 8) & 0xff;

  recv_len = sizeof(recv_buf);
  ret = cv520_transceive(dev, send_buf, 4, recv_buf, &recv_len, 100);
  if (ret < 0)
    {
      return ret;
    }

  if (recv_len < 16)
    {
      return -EIO;
    }

  /* Validate CRC_A when card returns 18 bytes (16 data + 2 CRC).
   * If CRC mismatch, reject the data to prevent corrupted playback. */

  if (recv_len >= 18)
    {
      uint16_t expected_crc = cv520_crc_a_sw(recv_buf, 16);
      uint16_t received_crc = recv_buf[16] | ((uint16_t)recv_buf[17] << 8);

      if (expected_crc != received_crc)
        {
          return -EIO;
        }
    }

  for (i = 0; i < 16; i++)
    {
      data[i] = recv_buf[i];
    }

  return OK;
}

/****************************************************************************
 * Name: rfid_stop_crypto
 ****************************************************************************/

int rfid_stop_crypto(FAR struct rfid_dev_s *dev)
{
  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

  return cv520_clear_bits(dev, CV520_REG_STATUS2,
                          CV520_STATUS2_MF_CRYPTO1_ON);
}

/****************************************************************************
 * Name: rfid_halt
 *
 * Description:
 *   Send HLTA command to put the card into HALT state.
 *   This ensures the card returns to a known state and will respond
 *   to WUPA on the next polling cycle.
 *
 ****************************************************************************/

int rfid_halt(FAR struct rfid_dev_s *dev)
{
  uint8_t send_buf[4];
  uint8_t recv_buf[4];
  size_t recv_len;
  uint16_t crc;

  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

  send_buf[0] = 0x50;  /* HLTA command */
  send_buf[1] = 0x00;

  crc = cv520_crc_a_sw(send_buf, 2);
  send_buf[2] = crc & 0xff;
  send_buf[3] = (crc >> 8) & 0xff;

  recv_len = sizeof(recv_buf);

  /* HLTA: card should NOT respond (goes silent). Timeout is expected. */

  (void)cv520_transceive(dev, send_buf, 4, recv_buf, &recv_len, 50);

  return OK;
}

/****************************************************************************
 * Name: rfid_cleanup
 *
 * Description:
 *   Thoroughly reset the chip's internal state machine after a read cycle.
 *   Clears: Crypto1 engine, FIFO, all IRQ flags, collision state,
 *   and BitFraming register. Does NOT toggle RF field.
 *
 *   Call after every completed or failed read-block cycle to prevent
 *   cumulative Crypto1/state-machine corruption that causes the
 *   "distance degrades after many reads" symptom.
 *
 ****************************************************************************/

int rfid_cleanup(FAR struct rfid_dev_s *dev)
{
  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

  /* 1. Force Idle */

  cv520_write_reg(dev, CV520_REG_COMMAND, CV520_CMD_IDLE);

  /* 2. Clear Crypto1 */

  cv520_clear_bits(dev, CV520_REG_STATUS2, CV520_STATUS2_MF_CRYPTO1_ON);

  /* 3. Flush FIFO */

  cv520_set_bits(dev, CV520_REG_FIFO_LEVEL, 0x80);

  /* 4. Clear all interrupt flags */

  cv520_write_reg(dev, CV520_REG_COMIRQ, 0x7F);
  cv520_write_reg(dev, CV520_REG_DIVIRQ, 0x7F);

  /* 5. Clear collision register */

  cv520_write_reg(dev, CV520_REG_COLL, 0x80);

  /* 6. Reset bit framing to default */

  cv520_write_reg(dev, CV520_REG_BIT_FRAMING, 0x00);

  return OK;
}

/****************************************************************************
 * Name: rfid_dump_regs
 *
 * Description:
 *   Dump all key CV520 registers for debugging.
 *
 ****************************************************************************/

int rfid_dump_regs(FAR struct rfid_dev_s *dev)
{
  uint8_t val;

  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

  printf("\n===== CV520 Register Dump =====\n\n");

  /* Page 0 - Command and Status */

  printf("Page 0 (Command/Status):\n");
  cv520_read_reg(dev, 0x01, &val);
  printf("  CommandReg     (0x01): 0x%02X\n", val);
  cv520_read_reg(dev, 0x02, &val);
  printf("  CommIEnReg     (0x02): 0x%02X\n", val);
  cv520_read_reg(dev, 0x03, &val);
  printf("  DivIEnReg      (0x03): 0x%02X\n", val);
  cv520_read_reg(dev, 0x04, &val);
  printf("  CommIRqReg     (0x04): 0x%02X\n", val);
  cv520_read_reg(dev, 0x05, &val);
  printf("  DivIRqReg      (0x05): 0x%02X\n", val);
  cv520_read_reg(dev, 0x06, &val);
  printf("  ErrorReg       (0x06): 0x%02X\n", val);
  cv520_read_reg(dev, 0x07, &val);
  printf("  Status1Reg     (0x07): 0x%02X\n", val);
  cv520_read_reg(dev, 0x08, &val);
  printf("  Status2Reg     (0x08): 0x%02X\n", val);
  cv520_read_reg(dev, 0x0A, &val);
  printf("  FIFOLevelReg   (0x0A): 0x%02X\n", val);
  cv520_read_reg(dev, 0x0D, &val);
  printf("  BitFramingReg  (0x0D): 0x%02X\n", val);
  cv520_read_reg(dev, 0x0E, &val);
  printf("  CollReg        (0x0E): 0x%02X\n", val);

  /* Page 1 - Communication */

  printf("\nPage 1 (Communication):\n");
  cv520_read_reg(dev, 0x11, &val);
  printf("  ModeReg        (0x11): 0x%02X\n", val);
  cv520_read_reg(dev, 0x12, &val);
  printf("  TxModeReg      (0x12): 0x%02X\n", val);
  cv520_read_reg(dev, 0x13, &val);
  printf("  RxModeReg      (0x13): 0x%02X\n", val);
  cv520_read_reg(dev, 0x14, &val);
  printf("  TxControlReg   (0x14): 0x%02X (TxRFEn=%d)\n", val, val & 1);
  cv520_read_reg(dev, 0x15, &val);
  printf("  TxASKReg       (0x15): 0x%02X (Force100ASK=%d)\n",
         val, (val >> 6) & 1);
  cv520_read_reg(dev, 0x16, &val);
  printf("  TxSelReg       (0x16): 0x%02X (DriverSel=%d)\n",
         val, (val >> 4) & 0x03);

  /* Page 2 - Configuration & Antenna Driver */

  printf("\nPage 2 (Configuration):\n");
  cv520_read_reg(dev, 0x24, &val);
  printf("  ModWidthReg    (0x24): 0x%02X\n", val);
  cv520_read_reg(dev, 0x26, &val);
  printf("  RFCfgReg       (0x26): 0x%02X (RxGain=%d = %ddB)\n",
         val, (val >> 4) & 0x07, 18 + ((val >> 4) & 0x07) * 5);
  cv520_read_reg(dev, 0x27, &val);
  printf("  GsNReg         (0x27): 0x%02X (CWGsN=%d, ModGsN=%d)\n",
         val, (val >> 4) & 0x0F, val & 0x0F);
  cv520_read_reg(dev, 0x28, &val);
  printf("  CWGsPReg       (0x28): 0x%02X (CWGsP=%d)\n", val, val & 0x3F);
  cv520_read_reg(dev, 0x29, &val);
  printf("  ModGsPReg      (0x29): 0x%02X (ModGsP=%d)\n", val, val & 0x3F);
  cv520_read_reg(dev, 0x2A, &val);
  printf("  TModeReg       (0x2A): 0x%02X\n", val);
  cv520_read_reg(dev, 0x2B, &val);
  printf("  TPrescalerReg  (0x2B): 0x%02X\n", val);

  /* Page 3 - Version */

  printf("\nPage 3 (Test/Version):\n");
  cv520_read_reg(dev, 0x37, &val);
  printf("  VersionReg     (0x37): 0x%02X\n", val);

  printf("\n===================================\n");
  printf("CV520 Key checks for RF output:\n");
  printf("  1. TxRFEn=1 (RF field enabled)\n");
  printf("  2. DriverSel=1 (modulated signal)\n");
  printf("  3. GsN/CWGsP/ModGsP > 0 (driver strength)\n");
  printf("  4. Force100ASK=1 (100%% ASK modulation)\n");
  printf("===================================\n\n");

  return OK;
}

/****************************************************************************
 * Name: rfid_txtest
 *
 * Description:
 *   Continuously send REQA commands for oscilloscope analysis.
 *   Helps diagnose TX modulation issues.
 *   First few iterations print detailed transceive debug.
 *
 ****************************************************************************/

int rfid_txtest(FAR struct rfid_dev_s *dev, int count)
{
  uint8_t send_buf[1];
  uint8_t recv_buf[2];
  size_t recv_len;
  uint8_t val;
  int ok_count = 0;
  int timeout_count = 0;
  int err_count = 0;
  int ret;
  int i;

  if (dev == NULL || !dev->initialized)
    {
      printf("ERROR: RFID not initialized\n");
      return -EINVAL;
    }

  printf("=== TX TEST: sending %d REQA commands ===\n", count);

  /* Dump critical registers before test */

  cv520_read_reg(dev, CV520_REG_COMMAND, &val);
  printf("Pre-test: Cmd=0x%02X", val);
  cv520_read_reg(dev, CV520_REG_TX_CONTROL, &val);
  printf(" TxCtrl=0x%02X", val);
  cv520_read_reg(dev, CV520_REG_TX_ASK, &val);
  printf(" TxASK=0x%02X", val);
  cv520_read_reg(dev, CV520_REG_TX_SEL, &val);
  printf(" TxSel=0x%02X", val);
  cv520_read_reg(dev, CV520_REG_BIT_FRAMING, &val);
  printf(" BitFrame=0x%02X", val);
  cv520_read_reg(dev, CV520_REG_STATUS2, &val);
  printf(" Stat2=0x%02X\n", val);

  /* Enable verbose debug for first 3 iterations */

  g_cv520_trx_debug = true;

  for (i = 0; i < count; i++)
    {
      if (i == 3)
        {
          g_cv520_trx_debug = false;
          printf("(debug off, continuing silently...)\n");
        }

      /* Clear Crypto1 */

      cv520_clear_bits(dev, CV520_REG_STATUS2, CV520_STATUS2_MF_CRYPTO1_ON);

      /* REQA = 7-bit short frame */

      cv520_set_bit_framing(dev, 0x07);

      send_buf[0] = PCD_CMD_REQA;
      recv_len = sizeof(recv_buf);

      ret = cv520_transceive(dev, send_buf, 1, recv_buf, &recv_len, 200);

      cv520_set_bit_framing(dev, 0x00);

      if (ret == OK)
        {
          ok_count++;
          if (i < 3)
            {
              printf("[%d] ATQA received: 0x%02X 0x%02X\n",
                     i, recv_buf[0], recv_buf[1]);
            }
        }
      else if (ret == -ETIMEDOUT)
        {
          timeout_count++;
        }
      else
        {
          err_count++;
          if (i < 3)
            {
              printf("[%d] Error: %d\n", i, ret);
            }
        }

      usleep(10000); /* 10ms between each REQA */
    }

  g_cv520_trx_debug = false;

  printf("\n=== TX TEST RESULT: %d REQA sent ===\n", count);
  printf("  OK (card responded): %d\n", ok_count);
  printf("  Timeout (no card):   %d\n", timeout_count);
  printf("  Error:               %d\n", err_count);

  /* Final register state */

  cv520_read_reg(dev, CV520_REG_TX_CONTROL, &val);
  printf("Post-test: TxCtrl=0x%02X", val);
  cv520_read_reg(dev, CV520_REG_COMMAND, &val);
  printf(" Cmd=0x%02X", val);
  cv520_read_reg(dev, CV520_REG_ERROR, &val);
  printf(" Err=0x%02X\n", val);

  return OK;
}
