/****************************************************************************
 * apps/myprojects/include/rfid_api.h
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
 * 这是读卡器对主程序露出的接口，芯片是 CV520，卡是 MIFARE。
 *
 * project1_main.c 里每次读卡的调用顺序是：
 *   rfid_request             唤醒天线场里的卡
 *   rfid_anticoll            防冲突，得到这张卡的 UID
 *   rfid_select              选中这张卡
 *   rfid_authenticate_block  认证要读的那一块。主程序用的是 Key A
 *   rfid_read_block          读出 16 字节。播报和点位匹配都用这 16 字节
 *   rfid_stop_crypto         结束这次加密会话
 *   rfid_halt                让卡休眠。不做这一步，它下一轮常不再应答唤醒
 *   rfid_cleanup             清芯片内部状态，不清天线场
 *
 * 寄存器细节在 modules/rfid/cv520_api.c，业务封装在 rfid_api.c。
 ****************************************************************************/


#ifndef __APPS_MYPROJECTS_INCLUDE_RFID_API_H
#define __APPS_MYPROJECTS_INCLUDE_RFID_API_H

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

/* RFID card UID structure */

struct rfid_uid_s
{
  uint8_t size;           /* UID size (4 or 7 bytes) */
  uint8_t uid[10];        /* UID data */
};

/* RFID device handle */

struct rfid_dev_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rfid_register
 *
 * Description:
 *   Register the RFID device and initialize it.
 *
 * Input Parameters:
 *   spi - Pointer to SPI device (obtained from stm32_spibus_initialize())
 *   rstpin - Reset GPIO pin number (or -1 if not used)
 *
 * Returned Value:
 *   Pointer to RFID device handle on success; NULL on failure.
 *
 ****************************************************************************/

EXTERN struct rfid_dev_s *rfid_register(FAR struct spi_dev_s *spi, int rstpin);

/****************************************************************************
 * Name: rfid_unregister
 *
 * Description:
 *   Unregister the RFID device and release resources.
 *
 * Input Parameters:
 *   dev - Pointer to RFID device handle
 *
 * Returned Value:
 *   Zero (OK) on success; negative error code on failure.
 *
 ****************************************************************************/

EXTERN int rfid_unregister(FAR struct rfid_dev_s *dev);

/****************************************************************************
 * Name: rfid_request
 *
 * Description:
 *   Request a card to be present (wake up).
 *
 * Input Parameters:
 *   dev - Pointer to RFID device handle
 *
 * Returned Value:
 *   Zero (OK) if card detected; negative error code on failure.
 *
 ****************************************************************************/

EXTERN int rfid_request(FAR struct rfid_dev_s *dev);

/****************************************************************************
 * Name: rfid_anticoll
 *
 * Description:
 *   Perform anti-collision detection and get card UID.
 *
 * Input Parameters:
 *   dev - Pointer to RFID device handle
 *   uid - Pointer to UID structure to store the result
 *
 * Returned Value:
 *   Zero (OK) on success; negative error code on failure.
 *
 ****************************************************************************/

EXTERN int rfid_anticoll(FAR struct rfid_dev_s *dev,
                          FAR struct rfid_uid_s *uid);

/****************************************************************************
 * Name: rfid_select
 *
 * Description:
 *   Select a card by its UID.
 *
 * Input Parameters:
 *   dev - Pointer to RFID device handle
 *   uid - Pointer to UID structure
 *
 * Returned Value:
 *   Zero (OK) on success; negative error code on failure.
 *
 ****************************************************************************/

EXTERN int rfid_select(FAR struct rfid_dev_s *dev,
                       FAR const struct rfid_uid_s *uid);

/****************************************************************************
 * Name: rfid_read_card
 *
 * Description:
 *   Read a card UID (complete sequence: request + anticoll + select).
 *
 * Input Parameters:
 *   dev - Pointer to RFID device handle
 *   uid - Pointer to UID structure to store the result
 *
 * Returned Value:
 *   Zero (OK) on success; negative error code on failure.
 *
 ****************************************************************************/

EXTERN int rfid_read_card(FAR struct rfid_dev_s *dev,
                           FAR struct rfid_uid_s *uid);

/****************************************************************************
 * Name: rfid_authenticate_block
 *
 * Description:
 *   Authenticate a MIFARE Classic block with Key A/B.
 *
 * Input Parameters:
 *   dev      - Pointer to RFID device handle
 *   uid      - Pointer to UID structure (uses first 4 bytes for auth)
 *   block    - Block number
 *   key      - 6-byte key
 *   use_keyb - false for Key A (0x60), true for Key B (0x61)
 *
 * Returned Value:
 *   Zero (OK) on success; negative error code on failure.
 *
 ****************************************************************************/

EXTERN int rfid_authenticate_block(FAR struct rfid_dev_s *dev,
                                   FAR const struct rfid_uid_s *uid,
                                   uint8_t block,
                                   FAR const uint8_t key[6],
                                   bool use_keyb);

/****************************************************************************
 * Name: rfid_read_block
 *
 * Description:
 *   Read a 16-byte MIFARE Classic block.
 *
 * Input Parameters:
 *   dev   - Pointer to RFID device handle
 *   block - Block number
 *   data  - Output buffer, at least 16 bytes
 *
 * Returned Value:
 *   Zero (OK) on success; negative error code on failure.
 *
 ****************************************************************************/

EXTERN int rfid_read_block(FAR struct rfid_dev_s *dev,
                           uint8_t block,
                           FAR uint8_t data[16]);

/****************************************************************************
 * Name: rfid_stop_crypto
 *
 * Description:
 *   Disable Crypto1 session after MIFARE authenticate/read/write.
 *
 ****************************************************************************/

EXTERN int rfid_stop_crypto(FAR struct rfid_dev_s *dev);

/****************************************************************************
 * Name: rfid_halt
 *
 * Description:
 *   Send HLTA to put card into HALT state (card won't respond to REQA,
 *   only to WUPA on next polling cycle).
 *
 ****************************************************************************/

EXTERN int rfid_halt(FAR struct rfid_dev_s *dev);

/****************************************************************************
 * Name: rfid_cleanup
 *
 * Description:
 *   Thoroughly clear chip internal state (Crypto1, FIFO, IRQs, collision,
 *   BitFraming). Does NOT toggle RF field. Call after every read cycle.
 *
 ****************************************************************************/

EXTERN int rfid_cleanup(FAR struct rfid_dev_s *dev);

/****************************************************************************
 * Name: rfid_dump_regs
 *
 * Description:
 *   Dump all key registers for debugging.
 *
 * Input Parameters:
 *   dev - Pointer to RFID device handle
 *
 * Returned Value:
 *   Zero (OK) on success; negative error code on failure.
 *
 ****************************************************************************/

EXTERN int rfid_dump_regs(FAR struct rfid_dev_s *dev);

/****************************************************************************
 * Name: rfid_txtest
 *
 * Description:
 *   Continuously send REQA commands for TX modulation testing.
 *
 ****************************************************************************/

EXTERN int rfid_txtest(FAR struct rfid_dev_s *dev, int count);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_MYPROJECTS_INCLUDE_RFID_API_H */
