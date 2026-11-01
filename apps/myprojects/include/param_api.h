/****************************************************************************
 * apps/myprojects/include/param_api.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/
/****************************************************************************
 * 掉电后仍保留的参数。上电时先填默认值，再由 Flash 中的记录覆盖。
 *
 * 电脑通过 USB 虚拟串口改的是内存里的这份。调用 p1_param_save() 之后
 * 才会写入 Flash，重启还在。点位不在这个结构里，点位有自己的 Flash 页。
 *
 * led_time 的单位是 100 毫秒。vtx_dead_zone_ms 是同一张卡的重复播报窗口。
 * gear_speed[0] 到 gear_speed[9] 对应旋钮 10 个档的速度百分比。
 ****************************************************************************/


#ifndef __APPS_MYPROJECTS_INCLUDE_PARAM_API_H
#define __APPS_MYPROJECTS_INCLUDE_PARAM_API_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct p1_params_s
{
  uint32_t rfid_target_block0;  /* primary block (0~63, default 4) */
  uint32_t rfid_target_block1;  /* fallback 1    (0~63, default 5) */
  uint32_t rfid_target_block2;  /* fallback 2    (0~63, default 6) */
  uint32_t led_time;            /* LED熄灭阈值, 100ms units (UI seconds = value/10) */
  uint32_t rgb_enable;          /* 0=off, 1=on */
  uint32_t rgb_brightness;      /* 0~100 (percent) */
  uint32_t motor_softstart;     /* 0=off, 1=on */
  uint32_t motor_ramp_ms;       /* ramp step interval in ms (10~5000) */
  uint32_t motor_ramp_step;     /* ramp step increment in % (1~20) */
  uint32_t rfid_cooldown_ms;    /* card-absent threshold ms (200~10000) */
  uint32_t vtx_dead_zone_ms;    /* 语音播报阈值, same-UID window (0~60000ms) */
  uint32_t match_threshold;     /* waypoint match similarity percent (0~100, default 60) */
  uint32_t gear_speed[10];      /* gear 0~9 speed %, default 55,60,...,100 */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int p1_param_init(void);
int p1_param_load(FAR struct p1_params_s *out);
int p1_param_save(FAR const struct p1_params_s *in);
void p1_param_factory_default(FAR struct p1_params_s *out);

int p1_param_get_u32(FAR const struct p1_params_s *cfg,
                     FAR const char *key,
                     FAR uint32_t *value);
int p1_param_set_u32(FAR struct p1_params_s *cfg,
                     FAR const char *key,
                     uint32_t value);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_MYPROJECTS_INCLUDE_PARAM_API_H */

