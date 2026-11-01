/****************************************************************************
 * apps/myprojects/include/waypoint_api.h
 *
 * Waypoint (point-of-interest) table for project1.
 *
 * A waypoint associates a piece of RFID card content (GBK bytes read from
 * the target block) with three per-point actions:
 *   - led      : light the functional LED when matched
 *   - announce : speak the card content when matched
 *   - speed    : override the motor target speed when matched
 *   - restore  : optionally return to the encoder gear speed after a delay
 *
 * The table is persisted in a dedicated pair of flash pages, completely
 * separate from the general parameter table, so this feature stays
 * self-contained.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/
/****************************************************************************
 * 点位表。最多 40 条，单独占两页 Flash，和普通参数不混在一起。
 *
 * 每条点位保存一段卡内容（最多 16 字节，就是读卡读出来的那一块）。
 * 匹配成功后三件事各自有开关：
 *   led       亮功能灯
 *   announce  把卡里的内容念出来
 *   speed     把电机目标速度改成 speed（百分比）
 * restore_ms 为 0 表示改完速度就保持；不是 0 就在这么多毫秒后
 * 回到旋钮当前档位的速度。
 *
 * mode 为 0 时按相似度模糊匹配，分数要达到参数 match_threshold。
 * mode 为 1 时必须逐字节相同。
 * len 为 0 的槽位是空的。
 ****************************************************************************/


#ifndef __APPS_MYPROJECTS_INCLUDE_WAYPOINT_API_H
#define __APPS_MYPROJECTS_INCLUDE_WAYPOINT_API_H

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
 * Pre-processor Definitions
 ****************************************************************************/

#define P1_WP_MAX          40   /* maximum number of configurable points */
#define P1_WP_UID_MAXLEN   16   /* one MIFARE block of content */
#define P1_WP_RESTORE_MAX_MS 60000u /* 0 = disabled, otherwise 1~60 seconds */

/* Match modes */

#define P1_WP_MODE_FUZZY   0    /* Jaccard byte-set similarity */
#define P1_WP_MODE_EXACT   1    /* byte-for-byte identical */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A single waypoint entry.  len == 0 means the slot is empty/disabled.
 * Total size is 24 bytes (kept word-aligned for flash storage).
 */

struct p1_waypoint_s
{
  uint8_t uid[P1_WP_UID_MAXLEN]; /* content to match (GBK bytes) */
  uint8_t len;                   /* valid length of uid[], 0 = empty */
  uint8_t mode;                  /* P1_WP_MODE_FUZZY / P1_WP_MODE_EXACT */
  uint8_t led;                   /* 1 = light LED on match */
  uint8_t announce;              /* 1 = announce content on match */
  uint8_t speed_en;              /* 1 = override motor speed on match */
  uint8_t speed;                 /* target speed percent (0~100) */
  uint16_t restore_ms;           /* 0 = keep speed, else delay then gear speed */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Load the table from flash (or start empty).  Call once at boot. */

int p1_waypoint_init(void);

/* Read/replace a single slot (0 <= index < P1_WP_MAX). Operates on the
 * in-RAM cache; call p1_waypoint_save() to persist.
 */

int p1_waypoint_get(int index, FAR struct p1_waypoint_s *out);
int p1_waypoint_set(int index, FAR const struct p1_waypoint_s *in);

/* Clear one slot, or all slots when index < 0. */

int p1_waypoint_clear(int index);

/* Persist the current in-RAM table to flash (double-buffered). */

int p1_waypoint_save(void);

/* Number of non-empty slots. */

int p1_waypoint_count(void);

/* Match card content against the table.
 *
 *   data      : card block content
 *   len       : valid length of data (bytes before padding)
 *   threshold : minimum similarity percent (0~100) for a fuzzy match
 *   out       : receives the best-matching waypoint (may be NULL)
 *   score     : receives the match percent 0~100 (may be NULL)
 *
 * Returns the matched slot index (>= 0), or -1 when nothing matches.
 */

int p1_waypoint_match(FAR const uint8_t *data, uint8_t len,
                      uint8_t threshold,
                      FAR struct p1_waypoint_s *out,
                      FAR uint8_t *score);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_MYPROJECTS_INCLUDE_WAYPOINT_API_H */
