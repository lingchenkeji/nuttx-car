/****************************************************************************
 * apps/myprojects/include/cdc_service_api.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/
/****************************************************************************
 * 电脑和板子的调试口。
 *
 * 物理线路是 USB 虚拟串口。一条 JSON 请求，回一条 JSON。
 * 命令入口是 p1_cdc_handle_json()，命令表写在
 * projects/project1/CDC_JSON_PROTOCOL.md，板上用 NSH 命令 p1ctl 发送。
 *
 * rfid_pause 为真时，主循环先不读卡，避免调试命令和主循环同时操作读卡芯片。
 * 这个接口会改内存里的参数；要掉电仍保留，还得再保存到 Flash。
 ****************************************************************************/


#ifndef __APPS_MYPROJECTS_INCLUDE_CDC_SERVICE_API_H
#define __APPS_MYPROJECTS_INCLUDE_CDC_SERVICE_API_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "rfid_api.h"
#include "vtx_api.h"
#include "param_api.h"
#include "drv8212_api.h"

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct p1_runtime_cfg_s
{
  volatile uint32_t *rfid_target_block0;
  volatile uint32_t *rfid_target_block1;
  volatile uint32_t *rfid_target_block2;
  volatile uint32_t *led_time_s;
  volatile uint32_t *rgb_enable;
  volatile uint32_t *rgb_brightness;
  volatile uint32_t *motor_softstart;
  volatile uint32_t *motor_ramp_ms;
  volatile uint32_t *motor_ramp_step;
  volatile uint32_t *rfid_cooldown_ms;
  volatile uint32_t *vtx_dead_zone_ms;
  volatile uint32_t *match_threshold;
  volatile uint32_t *gear_speed[10];
};

struct p1_cdc_context_s
{
  FAR struct rfid_dev_s *rfid;
  FAR struct vtx_dev_s *vtx;
  FAR struct drv8212_dev_s *motor;
  int encoder_gear;
  FAR struct p1_runtime_cfg_s *runtime;
  volatile bool *rfid_pause;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int p1_cdc_service_init(FAR const struct p1_cdc_context_s *ctx);
int p1_cdc_handle_json(FAR const char *req,
                       FAR char *resp,
                       size_t resp_size);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_MYPROJECTS_INCLUDE_CDC_SERVICE_API_H */

