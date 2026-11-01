/****************************************************************************
 * apps/myprojects/include/vtx_api.h
 *
 * VTX Voice Module API -- via F8U6 (CH32V006 + VTX316) custom protocol.
 *
 * The F8U6 module handles VTX316 hardware (RST, BAUD, BUSY, PA) internally.
 * The host MCU communicates with the F8U6 through a simple UART protocol:
 *
 *   Request:  [0xAA][0x55][LEN_H][LEN_L][CMD][PARAM...][CHK]
 *   Response: [0x55][0xAA][LEN_H][LEN_L][CMD][STATUS][DATA...][CHK]
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.
 *
 ****************************************************************************/
/****************************************************************************
 * 语音播报。文本经串口送出，编码为 GBK。
 *
 * 上电提示调用 vtx_speak()。读卡后的播报调用 vtx_speak_fast()，
 * 数据写完即返回，不等待模块应答。限时内写不进发送缓冲区时返回。
 * vtx_speak_and_wait() 要等到本次播放结束后才返回。
 *
 * 音量 0~10，语速 0~30。发音人编号见 VTX_SPEAKER_*。
 ****************************************************************************/


#ifndef __APPS_MYPROJECTS_INCLUDE_VTX_API_H
#define __APPS_MYPROJECTS_INCLUDE_VTX_API_H

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

/* Text Encoding Identifiers (used by callers of vtx_speak) */

#define VTX_ENCODING_GB2312           0x00
#define VTX_ENCODING_GBK              0x01
#define VTX_ENCODING_BIG5             0x02
#define VTX_ENCODING_UNICODE          0x03
#define VTX_ENCODING_UTF8             0x04

/* Baud Rate Configurations (host UART to F8U6, typically 115200) */

#define VTX_BAUDRATE_9600             0
#define VTX_BAUDRATE_57600            1
#define VTX_BAUDRATE_115200           2

/* Speaker IDs */

#define VTX_SPEAKER_XIAOLING          3     /* [m3]  Female "XiaoLing" */
#define VTX_SPEAKER_YIN_XIAOJIAN      51    /* [m51] Male "Yin XiaoJian" */
#define VTX_SPEAKER_YI_XIAOQIANG      52    /* [m52] Male "Yi XiaoQiang" */
#define VTX_SPEAKER_TIAN_BEIBEI       53    /* [m53] Female "Tian BeiBei" */
#define VTX_SPEAKER_DONALD_DUCK       54    /* [m54] Effect "Donald Duck" */
#define VTX_SPEAKER_XIAO_YANZI        55    /* [m55] Girl "Xiao YanZi" */
#define VTX_SPEAKER_BEI_TONG          56    /* [m56] Boy "Bei Tong" */
#define VTX_SPEAKER_XIAO_KE           57    /* [m57] Boy "Xiao Ke" */

/* Volume range: 0-10 */

#define VTX_VOLUME_MIN                0
#define VTX_VOLUME_MAX                10

/* Speed range: 0-30 */

#define VTX_SPEED_MIN                 0
#define VTX_SPEED_MAX                 30

/* Tone range: 0-10 */

#define VTX_TONE_MIN                  0
#define VTX_TONE_MAX                  10

/* Maximum text length per synthesis (in bytes) */

#define VTX_MAX_TEXT_LEN              4000

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* VTX chip status */

enum vtx_status_e
{
  VTX_STATUS_IDLE = 0,
  VTX_STATUS_BUSY,
  VTX_STATUS_ERROR,
  VTX_STATUS_UNKNOWN
};

/* VTX GPIO pin configuration (reserved for backward compatibility;
 * all fields should be set to 0 when using F8U6 module) */

struct vtx_pins_s
{
  uint32_t rstpin;
  uint32_t wppin;
  uint32_t baud0pin;
  uint32_t baud1pin;
  uint32_t busypin;
};

/* VTX device handle (opaque) */

struct vtx_dev_s;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

EXTERN FAR struct vtx_dev_s *vtx_register(FAR const char *devpath,
                                           FAR const struct vtx_pins_s *pins,
                                           int baudrate);

EXTERN int vtx_unregister(FAR struct vtx_dev_s *dev);

EXTERN int vtx_speak(FAR struct vtx_dev_s *dev, FAR const char *text,
                     uint8_t encoding);

EXTERN int vtx_speak_fast(FAR struct vtx_dev_s *dev, FAR const char *text,
                          uint8_t encoding);

EXTERN int vtx_speak_and_wait(FAR struct vtx_dev_s *dev,
                               FAR const char *text,
                               uint8_t encoding,
                               uint32_t timeout_ms);

EXTERN int vtx_stop(FAR struct vtx_dev_s *dev);

EXTERN int vtx_pause(FAR struct vtx_dev_s *dev);

EXTERN int vtx_resume(FAR struct vtx_dev_s *dev);

EXTERN int vtx_query_status(FAR struct vtx_dev_s *dev);

EXTERN bool vtx_is_busy(FAR struct vtx_dev_s *dev);

EXTERN int vtx_reset(FAR struct vtx_dev_s *dev);

EXTERN int vtx_sleep(FAR struct vtx_dev_s *dev);

EXTERN int vtx_wakeup(FAR struct vtx_dev_s *dev);

EXTERN int vtx_set_volume(FAR struct vtx_dev_s *dev, uint8_t volume);

EXTERN int vtx_set_speed(FAR struct vtx_dev_s *dev, uint8_t speed);

EXTERN int vtx_set_speaker(FAR struct vtx_dev_s *dev, uint8_t speaker);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_MYPROJECTS_INCLUDE_VTX_API_H */
