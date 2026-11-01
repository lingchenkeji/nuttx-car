/****************************************************************************
 * apps/myprojects/modules/vtx/vtx_api.c
 *
 * VTX Voice Module Driver -- F8U6 (CH32V006 + VTX316) protocol.
 *
 * Communicates with the F8U6 module via its custom UART protocol:
 *   Request:  [0xAA][0x55][LEN_H][LEN_L][CMD][PARAM...][CHK]
 *   Response: [0x55][0xAA][LEN_H][LEN_L][CMD][STATUS][DATA...][CHK]
 *   CHK = XOR of all bytes from LEN_H to last PARAM/DATA byte.
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.
 *
 ****************************************************************************/
/****************************************************************************
 * 语音模块的串口协议：组帧、校验并发送。
 * vtx_speak_fast() 用于读卡后的播报，帧发送后返回，不等待模块应答。
 * 发送缓冲区在约 20 毫秒内无法写入时放弃本次发送。
 * 上电等待就绪信号 1.8 秒；未收到时再查询状态，查询等待 0.2 秒。
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
#include <stdlib.h>
#include <errno.h>
#include <debug.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <poll.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>

#include "../../include/vtx_api.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VTX_RESP_TIMEOUT_MS    3000
#define VTX_INIT_TIMEOUT_MS    1800
#define VTX_INIT_QUERY_MS      200
#define VTX_DEVPATH_MAX        32

/* Card-path TTS only. Each spin is a 500us backoff when the UART
 * cannot accept bytes. This bounds a silent voice module; it is not
 * an ACK/echo wait. 40 * 500us = 20ms, enough to drain a full TX
 * buffer at 115200 and far below the old response timeout.
 */

#define VTX_SPEAK_FAST_TX_SPINS 40

/* F8U6 Protocol Frame Headers */

#define F8U6_REQ_H1           0xAA
#define F8U6_REQ_H2           0x55
#define F8U6_RSP_H1           0x55
#define F8U6_RSP_H2           0xAA

/* F8U6 Commands */

#define F8U6_CMD_TTS_GBK      0x01
#define F8U6_CMD_TTS_UTF8     0x02
#define F8U6_CMD_TTS_UNICODE  0x03
#define F8U6_CMD_TTS_RAW      0x04
#define F8U6_CMD_STOP         0x10
#define F8U6_CMD_PAUSE        0x11
#define F8U6_CMD_RESUME       0x12
#define F8U6_CMD_QUERY        0x20
#define F8U6_CMD_VERSION      0x21
#define F8U6_CMD_VOLUME       0x30
#define F8U6_CMD_SPEED        0x31
#define F8U6_CMD_SPEAKER      0x32
#define F8U6_CMD_TONE         0x33
#define F8U6_CMD_SLEEP        0x40
#define F8U6_CMD_WAKE         0x41
#define F8U6_CMD_RESET        0x42
#define F8U6_CMD_PA           0x43

/* F8U6 Events (unsolicited Board → Host) */

#define F8U6_EVT_READY        0xF0
#define F8U6_EVT_DONE         0xF1
#define F8U6_EVT_ERROR        0xF2

/* F8U6 Response Status */

#define F8U6_STS_OK           0x00
#define F8U6_STS_ERR          0x01
#define F8U6_STS_BUSY         0x02

#define F8U6_RSP_DATA_MAX     64

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct f8u6_rsp_s
{
  uint8_t  cmd;
  uint8_t  status;
  uint8_t  data[F8U6_RSP_DATA_MAX];
  uint16_t data_len;
};

struct vtx_dev_s
{
  char devpath[VTX_DEVPATH_MAX];
  int baudcfg;
  struct vtx_pins_s pins;
  bool initialized;
  uint8_t volume;
  uint8_t speed;
  uint8_t speaker;
};

/****************************************************************************
 * Private Functions -- UART helpers
 ****************************************************************************/

static speed_t vtx_get_termios_baud(int baudcfg)
{
  switch (baudcfg)
    {
      case VTX_BAUDRATE_9600:
        return B9600;

      case VTX_BAUDRATE_57600:
        return B57600;

      case VTX_BAUDRATE_115200:
      default:
        return B115200;
    }
}

static int vtx_uart_open(FAR const char *devpath, int baudcfg)
{
  int fd;
  struct termios tio;
  speed_t baud;

  fd = open(devpath, O_RDWR | O_NONBLOCK);
  if (fd < 0)
    {
      return -errno;
    }

  if (tcgetattr(fd, &tio) < 0)
    {
      close(fd);
      return -errno;
    }

  cfmakeraw(&tio);

  baud = vtx_get_termios_baud(baudcfg);
  cfsetispeed(&tio, baud);
  cfsetospeed(&tio, baud);

  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;
  tio.c_cflag &= ~PARENB;
  tio.c_cflag &= ~CSTOPB;
  tio.c_cflag &= ~CRTSCTS;
  tio.c_cflag |= (CLOCAL | CREAD);

  tio.c_cc[VMIN]  = 0;
  tio.c_cc[VTIME] = 1;

  if (tcsetattr(fd, TCSANOW, &tio) < 0)
    {
      close(fd);
      return -errno;
    }

  return fd;
}

static int vtx_read_byte(int fd, FAR uint8_t *byte, uint32_t timeout_ms)
{
  struct pollfd fds;
  ssize_t ret;

  fds.fd      = fd;
  fds.events  = POLLIN;
  fds.revents = 0;

  ret = poll(&fds, 1, timeout_ms);
  if (ret < 0)
    {
      return -errno;
    }

  if (ret == 0)
    {
      return -ETIMEDOUT;
    }

  if (fds.revents & POLLIN)
    {
      ret = read(fd, byte, 1);
      if (ret == 1)
        {
          return OK;
        }

      if (ret < 0)
        {
          return -errno;
        }
    }

  return -EIO;
}

static int vtx_uart_write_all(int fd, FAR const uint8_t *data, size_t len,
                              int spin_budget)
{
  size_t written = 0;
  ssize_t ret;
  int spins = 0;

  while (written < len)
    {
      ret = write(fd, data + written, len - written);
      if (ret < 0)
        {
          if (errno == EAGAIN || errno == EINTR)
            {
              if (spin_budget >= 0 && spins >= spin_budget)
                {
                  return -EAGAIN;
                }

              spins++;
              usleep(500);
              continue;
            }

          return -errno;
        }

      if (ret == 0)
        {
          /* A zero-length write must not spin forever on the
           * fire-and-forget path. Unlimited callers keep retrying.
           */

          if (spin_budget >= 0 && spins >= spin_budget)
            {
              return -EAGAIN;
            }

          spins++;
          if (spin_budget >= 0)
            {
              usleep(500);
            }

          continue;
        }

      written += (size_t)ret;
    }

  return OK;
}

static void vtx_flush_rx(int fd)
{
  uint8_t byte;

  while (vtx_read_byte(fd, &byte, 10) == OK);
}

/****************************************************************************
 * Private Functions -- F8U6 Protocol
 ****************************************************************************/

/****************************************************************************
 * Name: f8u6_send_request
 *
 * Description:
 *   Build and send an F8U6 request frame.
 *   [0xAA][0x55][LEN_H][LEN_L][CMD][PARAM...][CHK]
 *   LEN = 1 + param_len
 *   CHK = XOR(LEN_H, LEN_L, CMD, PARAM[0..n])
 *
 ****************************************************************************/

static int f8u6_send_request(int fd, uint8_t cmd,
                             FAR const uint8_t *param, uint16_t param_len,
                             int spin_budget)
{
  uint8_t frame[8];
  uint16_t len;
  uint8_t chk;
  uint16_t i;
  int ret;

  len = 1 + param_len;

  frame[0] = F8U6_REQ_H1;
  frame[1] = F8U6_REQ_H2;
  frame[2] = (len >> 8) & 0xFF;
  frame[3] = len & 0xFF;
  frame[4] = cmd;

  chk  = frame[2];
  chk ^= frame[3];
  chk ^= cmd;

  /* Send header + cmd */

  ret = vtx_uart_write_all(fd, frame, 5, spin_budget);
  if (ret < 0)
    {
      return ret;
    }

  /* Send parameters */

  if (param != NULL && param_len > 0)
    {
      ret = vtx_uart_write_all(fd, param, param_len, spin_budget);
      if (ret < 0)
        {
          return ret;
        }

      for (i = 0; i < param_len; i++)
        {
          chk ^= param[i];
        }
    }

  /* Send checksum */

  ret = vtx_uart_write_all(fd, &chk, 1, spin_budget);
  return ret;
}

/****************************************************************************
 * Name: f8u6_recv_response
 *
 * Description:
 *   Read and parse an F8U6 response frame.
 *   [0x55][0xAA][LEN_H][LEN_L][CMD][STATUS][DATA...][CHK]
 *   LEN = 2 + data_len
 *   CHK = XOR(LEN_H, LEN_L, CMD, STATUS, DATA[0..n])
 *
 ****************************************************************************/

static int f8u6_recv_response(int fd, FAR struct f8u6_rsp_s *rsp,
                              uint32_t timeout_ms)
{
  uint8_t byte;
  uint32_t elapsed = 0;
  uint32_t step = 20;

  enum
  {
    S_H1 = 0,
    S_H2,
    S_LENH,
    S_LENL,
    S_PAYLOAD,
    S_CHK
  } state = S_H1;

  uint8_t  len_h = 0;
  uint8_t  len_l = 0;
  uint16_t payload_len = 0;
  uint16_t payload_idx = 0;
  uint8_t  payload_buf[F8U6_RSP_DATA_MAX + 4];
  uint8_t  chk_calc = 0;
  int      ret;

  while (elapsed < timeout_ms)
    {
      ret = vtx_read_byte(fd, &byte, step);
      if (ret == -ETIMEDOUT)
        {
          elapsed += step;
          continue;
        }

      if (ret < 0)
        {
          return ret;
        }

      switch (state)
        {
          case S_H1:
            if (byte == F8U6_RSP_H1)
              {
                state = S_H2;
              }
            break;

          case S_H2:
            if (byte == F8U6_RSP_H2)
              {
                chk_calc = 0;
                state = S_LENH;
              }
            else
              {
                state = S_H1;
              }
            break;

          case S_LENH:
            len_h = byte;
            chk_calc ^= byte;
            state = S_LENL;
            break;

          case S_LENL:
            len_l = byte;
            chk_calc ^= byte;
            payload_len = ((uint16_t)len_h << 8) | len_l;
            if (payload_len < 2 ||
                payload_len > (F8U6_RSP_DATA_MAX + 4))
              {
                state = S_H1;
                break;
              }
            payload_idx = 0;
            state = S_PAYLOAD;
            break;

          case S_PAYLOAD:
            if (payload_idx < sizeof(payload_buf))
              {
                payload_buf[payload_idx] = byte;
              }
            chk_calc ^= byte;
            payload_idx++;
            if (payload_idx >= payload_len)
              {
                state = S_CHK;
              }
            break;

          case S_CHK:
            if (chk_calc != byte)
              {
                printf("VTX: F8U6 checksum err (got 0x%02X expect 0x%02X)\n",
                       byte, chk_calc);
                state = S_H1;
                break;
              }

            /* Frame valid -- extract fields */

            rsp->cmd    = payload_buf[0];
            rsp->status = (payload_len >= 2) ? payload_buf[1] : 0xFF;
            rsp->data_len = (payload_len > 2) ? (payload_len - 2) : 0;

            if (rsp->data_len > F8U6_RSP_DATA_MAX)
              {
                rsp->data_len = F8U6_RSP_DATA_MAX;
              }

            if (rsp->data_len > 0)
              {
                memcpy(rsp->data, &payload_buf[2], rsp->data_len);
              }

            return OK;
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: f8u6_transact
 *
 * Description:
 *   Send a request and wait for matching response (skipping events).
 *
 ****************************************************************************/

static int f8u6_transact(int fd, uint8_t cmd,
                         FAR const uint8_t *param, uint16_t param_len,
                         FAR struct f8u6_rsp_s *rsp,
                         uint32_t timeout_ms)
{
  uint32_t elapsed = 0;
  int ret;

  ret = f8u6_send_request(fd, cmd, param, param_len, -1);
  if (ret < 0)
    {
      return ret;
    }

  while (elapsed < timeout_ms)
    {
      ret = f8u6_recv_response(fd, rsp, timeout_ms - elapsed);
      if (ret < 0)
        {
          return ret;
        }

      if (rsp->cmd == cmd)
        {
          return OK;
        }

      /* Received event rather than our response -- continue waiting */

      elapsed += 100;
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: vtx_enc_to_cmd
 *
 * Description:
 *   Map VTX_ENCODING_xxx to the corresponding F8U6 TTS command byte.
 *
 ****************************************************************************/

static uint8_t vtx_enc_to_cmd(uint8_t encoding)
{
  switch (encoding)
    {
      case VTX_ENCODING_GB2312:
      case VTX_ENCODING_GBK:
      case VTX_ENCODING_BIG5:
        return F8U6_CMD_TTS_GBK;

      case VTX_ENCODING_UTF8:
        return F8U6_CMD_TTS_UTF8;

      case VTX_ENCODING_UNICODE:
        return F8U6_CMD_TTS_UNICODE;

      default:
        return F8U6_CMD_TTS_GBK;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vtx_register
 *
 * Description:
 *   Register the F8U6 voice module. Open UART, wait for EVT_VTX_READY or
 *   query status to confirm module is online.
 *
 ****************************************************************************/

FAR struct vtx_dev_s *vtx_register(FAR const char *devpath,
                                    FAR const struct vtx_pins_s *pins,
                                    int baudrate)
{
  FAR struct vtx_dev_s *dev;
  struct f8u6_rsp_s rsp;
  int fd;
  int ret;

  if (devpath == NULL)
    {
      printf("VTX: invalid param\n");
      return NULL;
    }

  dev = (FAR struct vtx_dev_s *)kmm_zalloc(sizeof(struct vtx_dev_s));
  if (dev == NULL)
    {
      printf("VTX: alloc failed\n");
      return NULL;
    }

  strncpy(dev->devpath, devpath, VTX_DEVPATH_MAX - 1);
  dev->devpath[VTX_DEVPATH_MAX - 1] = '\0';
  dev->baudcfg  = baudrate;
  dev->volume   = 5;
  dev->speed    = 5;
  dev->speaker  = VTX_SPEAKER_XIAOLING;

  if (pins != NULL)
    {
      memcpy(&dev->pins, pins, sizeof(struct vtx_pins_s));
    }

  fd = vtx_uart_open(devpath, baudrate);
  if (fd < 0)
    {
      printf("VTX: UART open failed\n");
      kmm_free(dev);
      return NULL;
    }

  vtx_flush_rx(fd);

  /* Wait for EVT_VTX_READY from F8U6 (sent after VTX316 init) */

  printf("VTX: waiting for F8U6 ready...\n");
  ret = f8u6_recv_response(fd, &rsp, VTX_INIT_TIMEOUT_MS);

  if (ret == OK && rsp.cmd == F8U6_EVT_READY)
    {
      printf("VTX: F8U6 ready (EVT 0xF0)\n");
    }
  else
    {
      /* Fallback: query status to check if module is already running */

      printf("VTX: no EVT_READY, trying status query...\n");
      vtx_flush_rx(fd);
      ret = f8u6_transact(fd, F8U6_CMD_QUERY, NULL, 0,
                           &rsp, VTX_INIT_QUERY_MS);
      if (ret < 0 || rsp.status != F8U6_STS_OK)
        {
          printf("VTX: F8U6 not detected\n");
          close(fd);
          kmm_free(dev);
          return NULL;
        }

      printf("VTX: F8U6 online (query OK)\n");
    }

  close(fd);

  dev->initialized = true;
  printf("VTX: OK\n");

  return dev;
}

/****************************************************************************
 * Name: vtx_unregister
 ****************************************************************************/

int vtx_unregister(FAR struct vtx_dev_s *dev)
{
  if (dev == NULL)
    {
      return -EINVAL;
    }

  dev->initialized = false;
  kmm_free(dev);
  printf("VTX: unregistered\n");

  return OK;
}

/****************************************************************************
 * Name: vtx_speak
 *
 * Description:
 *   Send text to F8U6 for synthesis and playback (non-blocking).
 *
 ****************************************************************************/

int vtx_speak(FAR struct vtx_dev_s *dev, FAR const char *text,
              uint8_t encoding)
{
  struct f8u6_rsp_s rsp;
  uint16_t textlen;
  uint8_t cmd;
  int fd;
  int ret;

  if (dev == NULL || !dev->initialized || text == NULL)
    {
      return -EINVAL;
    }

  textlen = strlen(text);
  if (textlen == 0)
    {
      return -EINVAL;
    }

  if (textlen > VTX_MAX_TEXT_LEN)
    {
      textlen = VTX_MAX_TEXT_LEN;
    }

  cmd = vtx_enc_to_cmd(encoding);

  fd = vtx_uart_open(dev->devpath, dev->baudcfg);
  if (fd < 0)
    {
      return fd;
    }

  vtx_flush_rx(fd);

  ret = f8u6_transact(fd, cmd, (FAR const uint8_t *)text, textlen,
                       &rsp, VTX_RESP_TIMEOUT_MS);
  close(fd);

  if (ret < 0)
    {
      printf("VTX: speak failed (ret=%d)\n", ret);
      return ret;
    }

  if (rsp.status == F8U6_STS_BUSY)
    {
      printf("VTX: busy\n");
      return -EBUSY;
    }

  if (rsp.status != F8U6_STS_OK)
    {
      printf("VTX: speak err (0x%02X)\n", rsp.status);
      return -EIO;
    }

  printf("VTX: speak OK (%u bytes)\n", textlen);
  return OK;
}

/****************************************************************************
 * Name: vtx_speak_fast
 *
 * Description:
 *   Queue a TTS frame and return without reading the F8U6 ACK/echo.
 *   Used after an RFID block is read. A missing speaker never produces
 *   that response; waiting for it froze the main loop.
 *
 ****************************************************************************/

int vtx_speak_fast(FAR struct vtx_dev_s *dev, FAR const char *text,
                   uint8_t encoding)
{
  uint16_t textlen;
  uint8_t cmd;
  int fd;
  int ret;

  if (dev == NULL || !dev->initialized || text == NULL)
    {
      return -EINVAL;
    }

  textlen = strlen(text);
  if (textlen == 0)
    {
      return -EINVAL;
    }

  if (textlen > VTX_MAX_TEXT_LEN)
    {
      textlen = VTX_MAX_TEXT_LEN;
    }

  cmd = vtx_enc_to_cmd(encoding);

  fd = vtx_uart_open(dev->devpath, dev->baudcfg);
  if (fd < 0)
    {
      return fd;
    }

  /* Send only. Do not call f8u6_recv_response / f8u6_transact. */

  ret = f8u6_send_request(fd, cmd, (FAR const uint8_t *)text, textlen,
                          VTX_SPEAK_FAST_TX_SPINS);

  /* Let the UART shift the frame out. close() on O_NONBLOCK does not
   * drain, and this sleep is not a wait for the module echo.
   */

  usleep(5000);
  close(fd);

  return ret;
}

/****************************************************************************
 * Name: vtx_speak_and_wait
 *
 * Description:
 *   Send text and wait until playback finishes (EVT_PLAY_DONE).
 *
 ****************************************************************************/

int vtx_speak_and_wait(FAR struct vtx_dev_s *dev,
                        FAR const char *text,
                        uint8_t encoding,
                        uint32_t timeout_ms)
{
  struct f8u6_rsp_s rsp;
  uint16_t textlen;
  uint8_t cmd;
  int fd;
  int ret;
  uint32_t elapsed;

  if (dev == NULL || !dev->initialized || text == NULL)
    {
      return -EINVAL;
    }

  textlen = strlen(text);
  if (textlen == 0)
    {
      return -EINVAL;
    }

  if (textlen > VTX_MAX_TEXT_LEN)
    {
      textlen = VTX_MAX_TEXT_LEN;
    }

  if (timeout_ms == 0)
    {
      timeout_ms = 60000;
    }

  cmd = vtx_enc_to_cmd(encoding);

  fd = vtx_uart_open(dev->devpath, dev->baudcfg);
  if (fd < 0)
    {
      return fd;
    }

  vtx_flush_rx(fd);

  /* Send TTS command */

  ret = f8u6_transact(fd, cmd, (FAR const uint8_t *)text, textlen,
                       &rsp, VTX_RESP_TIMEOUT_MS);
  if (ret < 0 || rsp.status != F8U6_STS_OK)
    {
      printf("VTX: speak_and_wait: send failed\n");
      close(fd);
      return (ret < 0) ? ret : -EIO;
    }

  /* Wait for EVT_PLAY_DONE */

  elapsed = 0;
  while (elapsed < timeout_ms)
    {
      ret = f8u6_recv_response(fd, &rsp, 200);
      if (ret == OK)
        {
          if (rsp.cmd == F8U6_EVT_DONE)
            {
              printf("VTX: speech done\n");
              close(fd);
              return OK;
            }

          if (rsp.cmd == F8U6_EVT_ERROR)
            {
              printf("VTX: speech error event\n");
              close(fd);
              return -EIO;
            }
        }

      elapsed += 200;
    }

  printf("VTX: speech timeout\n");
  close(fd);
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: vtx_stop
 ****************************************************************************/

int vtx_stop(FAR struct vtx_dev_s *dev)
{
  struct f8u6_rsp_s rsp;
  int fd;
  int ret;

  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

  fd = vtx_uart_open(dev->devpath, dev->baudcfg);
  if (fd < 0)
    {
      return fd;
    }

  ret = f8u6_transact(fd, F8U6_CMD_STOP, NULL, 0,
                       &rsp, VTX_RESP_TIMEOUT_MS);
  close(fd);
  return (ret < 0) ? ret : OK;
}

/****************************************************************************
 * Name: vtx_pause
 ****************************************************************************/

int vtx_pause(FAR struct vtx_dev_s *dev)
{
  struct f8u6_rsp_s rsp;
  int fd;
  int ret;

  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

  fd = vtx_uart_open(dev->devpath, dev->baudcfg);
  if (fd < 0)
    {
      return fd;
    }

  ret = f8u6_transact(fd, F8U6_CMD_PAUSE, NULL, 0,
                       &rsp, VTX_RESP_TIMEOUT_MS);
  close(fd);
  return (ret < 0) ? ret : OK;
}

/****************************************************************************
 * Name: vtx_resume
 ****************************************************************************/

int vtx_resume(FAR struct vtx_dev_s *dev)
{
  struct f8u6_rsp_s rsp;
  int fd;
  int ret;

  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

  fd = vtx_uart_open(dev->devpath, dev->baudcfg);
  if (fd < 0)
    {
      return fd;
    }

  ret = f8u6_transact(fd, F8U6_CMD_RESUME, NULL, 0,
                       &rsp, VTX_RESP_TIMEOUT_MS);
  close(fd);
  return (ret < 0) ? ret : OK;
}

/****************************************************************************
 * Name: vtx_query_status
 *
 * Description:
 *   Query VTX316 status via F8U6. Response data: [vtx_state, hw_busy].
 *
 ****************************************************************************/

int vtx_query_status(FAR struct vtx_dev_s *dev)
{
  struct f8u6_rsp_s rsp;
  int fd;
  int ret;

  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

  fd = vtx_uart_open(dev->devpath, dev->baudcfg);
  if (fd < 0)
    {
      return fd;
    }

  vtx_flush_rx(fd);

  ret = f8u6_transact(fd, F8U6_CMD_QUERY, NULL, 0,
                       &rsp, VTX_RESP_TIMEOUT_MS);
  close(fd);

  if (ret < 0)
    {
      return ret;
    }

  /* data[0] = vtx_state, data[1] = hw_busy */

  if (rsp.data_len >= 2 && rsp.data[1])
    {
      return VTX_STATUS_BUSY;
    }

  return VTX_STATUS_IDLE;
}

/****************************************************************************
 * Name: vtx_is_busy
 *
 * Description:
 *   Check if VTX316 is currently playing via CMD_QUERY_STATUS.
 *
 ****************************************************************************/

bool vtx_is_busy(FAR struct vtx_dev_s *dev)
{
  struct f8u6_rsp_s rsp;
  int fd;
  int ret;

  if (dev == NULL || !dev->initialized)
    {
      return false;
    }

  fd = vtx_uart_open(dev->devpath, dev->baudcfg);
  if (fd < 0)
    {
      return false;
    }

  vtx_flush_rx(fd);

  ret = f8u6_transact(fd, F8U6_CMD_QUERY, NULL, 0, &rsp, 500);
  close(fd);

  if (ret < 0)
    {
      return false;
    }

  /* data[0] = vtx_state, data[1] = hw_busy */

  if (rsp.data_len >= 2)
    {
      return rsp.data[1] != 0;
    }

  return (rsp.status == F8U6_STS_BUSY);
}

/****************************************************************************
 * Name: vtx_reset
 ****************************************************************************/

int vtx_reset(FAR struct vtx_dev_s *dev)
{
  struct f8u6_rsp_s rsp;
  int fd;
  int ret;

  if (dev == NULL)
    {
      return -EINVAL;
    }

  fd = vtx_uart_open(dev->devpath, dev->baudcfg);
  if (fd < 0)
    {
      return fd;
    }

  ret = f8u6_transact(fd, F8U6_CMD_RESET, NULL, 0,
                       &rsp, VTX_RESP_TIMEOUT_MS);
  close(fd);
  return (ret < 0) ? ret : OK;
}

/****************************************************************************
 * Name: vtx_sleep
 ****************************************************************************/

int vtx_sleep(FAR struct vtx_dev_s *dev)
{
  struct f8u6_rsp_s rsp;
  int fd;
  int ret;

  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

  fd = vtx_uart_open(dev->devpath, dev->baudcfg);
  if (fd < 0)
    {
      return fd;
    }

  ret = f8u6_transact(fd, F8U6_CMD_SLEEP, NULL, 0,
                       &rsp, VTX_RESP_TIMEOUT_MS);
  close(fd);
  return (ret < 0) ? ret : OK;
}

/****************************************************************************
 * Name: vtx_wakeup
 ****************************************************************************/

int vtx_wakeup(FAR struct vtx_dev_s *dev)
{
  struct f8u6_rsp_s rsp;
  int fd;
  int ret;

  if (dev == NULL)
    {
      return -EINVAL;
    }

  fd = vtx_uart_open(dev->devpath, dev->baudcfg);
  if (fd < 0)
    {
      return fd;
    }

  ret = f8u6_transact(fd, F8U6_CMD_WAKE, NULL, 0,
                       &rsp, VTX_RESP_TIMEOUT_MS);
  close(fd);
  return (ret < 0) ? ret : OK;
}

/****************************************************************************
 * Name: vtx_set_volume
 *
 * Description:
 *   Set TTS volume (0-10) via F8U6 CMD_SET_VOLUME.
 *
 ****************************************************************************/

int vtx_set_volume(FAR struct vtx_dev_s *dev, uint8_t volume)
{
  struct f8u6_rsp_s rsp;
  int fd;
  int ret;

  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

  if (volume > VTX_VOLUME_MAX)
    {
      volume = VTX_VOLUME_MAX;
    }

  fd = vtx_uart_open(dev->devpath, dev->baudcfg);
  if (fd < 0)
    {
      return fd;
    }

  ret = f8u6_transact(fd, F8U6_CMD_VOLUME, &volume, 1,
                       &rsp, VTX_RESP_TIMEOUT_MS);
  close(fd);

  if (ret == OK && rsp.status == F8U6_STS_OK)
    {
      dev->volume = volume;
      printf("VTX: volume=%u\n", volume);
      return OK;
    }

  return (ret < 0) ? ret : -EIO;
}

/****************************************************************************
 * Name: vtx_set_speed
 *
 * Description:
 *   Set TTS speed (0-10) via F8U6 CMD_SET_SPEED.
 *
 ****************************************************************************/

int vtx_set_speed(FAR struct vtx_dev_s *dev, uint8_t speed)
{
  struct f8u6_rsp_s rsp;
  int fd;
  int ret;

  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

  if (speed > VTX_SPEED_MAX)
    {
      speed = VTX_SPEED_MAX;
    }

  fd = vtx_uart_open(dev->devpath, dev->baudcfg);
  if (fd < 0)
    {
      return fd;
    }

  ret = f8u6_transact(fd, F8U6_CMD_SPEED, &speed, 1,
                       &rsp, VTX_RESP_TIMEOUT_MS);
  close(fd);

  if (ret == OK && rsp.status == F8U6_STS_OK)
    {
      dev->speed = speed;
      printf("VTX: speed=%u\n", speed);
      return OK;
    }

  return (ret < 0) ? ret : -EIO;
}

/****************************************************************************
 * Name: vtx_set_speaker
 *
 * Description:
 *   Set TTS speaker (voice person) via F8U6 CMD_SET_SPEAKER.
 *
 ****************************************************************************/

int vtx_set_speaker(FAR struct vtx_dev_s *dev, uint8_t speaker)
{
  struct f8u6_rsp_s rsp;
  int fd;
  int ret;

  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

  fd = vtx_uart_open(dev->devpath, dev->baudcfg);
  if (fd < 0)
    {
      return fd;
    }

  ret = f8u6_transact(fd, F8U6_CMD_SPEAKER, &speaker, 1,
                       &rsp, VTX_RESP_TIMEOUT_MS);
  close(fd);

  if (ret == OK && rsp.status == F8U6_STS_OK)
    {
      dev->speaker = speaker;
      printf("VTX: speaker=%u\n", speaker);
      return OK;
    }

  return (ret < 0) ? ret : -EIO;
}
