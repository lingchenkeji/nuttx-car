/****************************************************************************
 * apps/myprojects/modules/waypoint/waypoint_api.c
 *
 * Waypoint (point-of-interest) table with dedicated flash persistence.
 *
 * Storage layout (STM32F103, 1KB erase pages, 128 pages total):
 *   - param table  : pages nblocks-2, nblocks-1  (managed by param module)
 *   - waypoint table: pages nblocks-4, nblocks-3 (managed here)
 * The two features never share a page, so they stay independent.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/
/****************************************************************************
 * 点位的匹配和 Flash 存取。比对的是卡里读出的内容。
 * 模糊匹配看两段内容的字节集合有多像，精确匹配必须逐字节相同。
 * 表存在参数区旁边独立的两页 Flash 里，保存点位不会把普通参数冲掉。
 ****************************************************************************/


/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>

#include <nuttx/progmem.h>

#include "../../include/waypoint_api.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define P1_WP_MAGIC     0x50315750u  /* "P1WP" */
#define P1_WP_VERSION   1u

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* On-flash record.  Size = 16 header + 40*24 + 4 crc = 980 bytes (< 1KB). */

struct p1_wp_record_s
{
  uint32_t magic;
  uint32_t version;
  uint32_t seq;
  uint32_t count;                        /* informational */
  struct p1_waypoint_s wp[P1_WP_MAX];
  uint32_t crc32;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool     g_wp_ready;
static uint32_t g_wp_seq;
static size_t   g_wp_block[2];
static size_t   g_wp_addr[2];

/* Single in-RAM record acts as the working cache. */

static struct p1_wp_record_s g_wp_rec;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t p1_wp_crc32(FAR const uint8_t *data, size_t len)
{
  uint32_t crc = 0xffffffffu;
  size_t i;
  int b;

  for (i = 0; i < len; i++)
    {
      crc ^= data[i];
      for (b = 0; b < 8; b++)
        {
          uint32_t mask = (crc & 1u) ? 0xedb88320u : 0u;
          crc = (crc >> 1) ^ mask;
        }
    }

  return ~crc;
}

static uint32_t p1_wp_calc_crc(FAR const struct p1_wp_record_s *rec)
{
  return p1_wp_crc32((FAR const uint8_t *)rec,
                     sizeof(struct p1_wp_record_s) - sizeof(uint32_t));
}

/* Validate a flash slot; if valid, copy it into the RAM cache. */

static bool p1_wp_read_slot(int slot)
{
  FAR const struct p1_wp_record_s *raw;
  uint32_t crc;

  raw = (FAR const struct p1_wp_record_s *)(uintptr_t)g_wp_addr[slot];

  if (raw->magic != P1_WP_MAGIC ||
      raw->version != P1_WP_VERSION)
    {
      return false;
    }

  crc = p1_wp_crc32((FAR const uint8_t *)raw,
                    sizeof(struct p1_wp_record_s) - sizeof(uint32_t));
  if (crc != raw->crc32)
    {
      return false;
    }

  memcpy(&g_wp_rec, raw, sizeof(struct p1_wp_record_s));
  return true;
}

static void p1_wp_reset_ram(void)
{
  memset(&g_wp_rec, 0, sizeof(g_wp_rec));
  g_wp_rec.magic   = P1_WP_MAGIC;
  g_wp_rec.version = P1_WP_VERSION;
  g_wp_rec.seq     = 0;
  g_wp_rec.count   = 0;
}

static int p1_wp_write_slot(int slot)
{
  ssize_t ret;
  size_t block = g_wp_block[slot];
  size_t addr  = g_wp_addr[slot];

  ret = up_progmem_eraseblock(block);
  if (ret < 0)
    {
      return (int)ret;
    }

  ret = up_progmem_write(addr, &g_wp_rec, sizeof(struct p1_wp_record_s));
  if (ret < 0)
    {
      return (int)ret;
    }

  if ((size_t)ret != sizeof(struct p1_wp_record_s))
    {
      return -EIO;
    }

  if (memcmp((FAR const void *)(uintptr_t)addr, &g_wp_rec,
             sizeof(struct p1_wp_record_s)) != 0)
    {
      return -EIO;
    }

  return OK;
}

/* Build a 256-bit presence bitmap (32 bytes) over the byte values. */

static void p1_wp_bitmap(FAR uint8_t bm[32],
                         FAR const uint8_t *data, uint8_t len)
{
  uint8_t i;

  memset(bm, 0, 32);
  for (i = 0; i < len; i++)
    {
      bm[data[i] >> 3] |= (uint8_t)(1u << (data[i] & 7u));
    }
}

static uint8_t p1_wp_popcount(uint8_t v)
{
  uint8_t c = 0;
  while (v)
    {
      c += (uint8_t)(v & 1u);
      v >>= 1;
    }

  return c;
}

/* Jaccard similarity (percent 0~100) over the byte-value sets. */

static uint8_t p1_wp_similarity(FAR const uint8_t *a, uint8_t alen,
                                FAR const uint8_t *b, uint8_t blen)
{
  uint8_t ba[32];
  uint8_t bb[32];
  uint32_t inter = 0;
  uint32_t uni   = 0;
  int k;

  if (alen == 0 || blen == 0)
    {
      return 0;
    }

  p1_wp_bitmap(ba, a, alen);
  p1_wp_bitmap(bb, b, blen);

  for (k = 0; k < 32; k++)
    {
      inter += p1_wp_popcount((uint8_t)(ba[k] & bb[k]));
      uni   += p1_wp_popcount((uint8_t)(ba[k] | bb[k]));
    }

  if (uni == 0)
    {
      return 0;
    }

  return (uint8_t)((inter * 100u) / uni);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int p1_waypoint_init(void)
{
  size_t nblocks;

  nblocks = up_progmem_neraseblocks();
  if (nblocks < 4)
    {
      p1_wp_reset_ram();
      return -ENOSPC;
    }

  /* Two dedicated pages just below the parameter pages. */

  g_wp_block[0] = nblocks - 4;
  g_wp_block[1] = nblocks - 3;
  g_wp_addr[0]  = up_progmem_getaddress(g_wp_block[0]);
  g_wp_addr[1]  = up_progmem_getaddress(g_wp_block[1]);

  if (g_wp_addr[0] == SIZE_MAX || g_wp_addr[1] == SIZE_MAX)
    {
      p1_wp_reset_ram();
      return -EFAULT;
    }

  /* Load the newest valid slot. */

  {
    struct p1_wp_record_s tmp;
    bool va;
    bool vb;
    uint32_t sa = 0;
    uint32_t sb = 0;

    /* Peek both sequence numbers without disturbing the cache. */

    va = false;
    vb = false;

    if (((FAR const struct p1_wp_record_s *)(uintptr_t)g_wp_addr[0])->magic
        == P1_WP_MAGIC)
      {
        FAR const struct p1_wp_record_s *r0 =
          (FAR const struct p1_wp_record_s *)(uintptr_t)g_wp_addr[0];
        if (r0->version == P1_WP_VERSION &&
            p1_wp_crc32((FAR const uint8_t *)r0,
                        sizeof(tmp) - sizeof(uint32_t)) == r0->crc32)
          {
            va = true;
            sa = r0->seq;
          }
      }

    if (((FAR const struct p1_wp_record_s *)(uintptr_t)g_wp_addr[1])->magic
        == P1_WP_MAGIC)
      {
        FAR const struct p1_wp_record_s *r1 =
          (FAR const struct p1_wp_record_s *)(uintptr_t)g_wp_addr[1];
        if (r1->version == P1_WP_VERSION &&
            p1_wp_crc32((FAR const uint8_t *)r1,
                        sizeof(tmp) - sizeof(uint32_t)) == r1->crc32)
          {
            vb = true;
            sb = r1->seq;
          }
      }

    if (va && vb)
      {
        if (sa >= sb)
          {
            p1_wp_read_slot(0);
            g_wp_seq = sa;
          }
        else
          {
            p1_wp_read_slot(1);
            g_wp_seq = sb;
          }
      }
    else if (va)
      {
        p1_wp_read_slot(0);
        g_wp_seq = sa;
      }
    else if (vb)
      {
        p1_wp_read_slot(1);
        g_wp_seq = sb;
      }
    else
      {
        p1_wp_reset_ram();
        g_wp_seq = 0;
      }
  }

  g_wp_ready = true;
  return OK;
}

int p1_waypoint_get(int index, FAR struct p1_waypoint_s *out)
{
  if (out == NULL || index < 0 || index >= P1_WP_MAX)
    {
      return -EINVAL;
    }

  *out = g_wp_rec.wp[index];
  return OK;
}

int p1_waypoint_set(int index, FAR const struct p1_waypoint_s *in)
{
  struct p1_waypoint_s e;

  if (in == NULL || index < 0 || index >= P1_WP_MAX)
    {
      return -EINVAL;
    }

  e = *in;

  if (e.len > P1_WP_UID_MAXLEN)
    {
      return -ERANGE;
    }

  if (e.mode > P1_WP_MODE_EXACT)
    {
      return -ERANGE;
    }

  if (e.speed > 100)
    {
      return -ERANGE;
    }

  if (e.restore_ms > P1_WP_RESTORE_MAX_MS)
    {
      return -ERANGE;
    }

  e.led      = e.led ? 1 : 0;
  e.announce = e.announce ? 1 : 0;
  e.speed_en = e.speed_en ? 1 : 0;
  if (!e.speed_en)
    {
      e.restore_ms = 0;
    }

  g_wp_rec.wp[index] = e;
  return OK;
}

int p1_waypoint_clear(int index)
{
  int i;

  if (index >= P1_WP_MAX)
    {
      return -EINVAL;
    }

  if (index < 0)
    {
      for (i = 0; i < P1_WP_MAX; i++)
        {
          memset(&g_wp_rec.wp[i], 0, sizeof(struct p1_waypoint_s));
        }
    }
  else
    {
      memset(&g_wp_rec.wp[index], 0, sizeof(struct p1_waypoint_s));
    }

  return OK;
}

int p1_waypoint_count(void)
{
  int i;
  int n = 0;

  for (i = 0; i < P1_WP_MAX; i++)
    {
      if (g_wp_rec.wp[i].len > 0)
        {
          n++;
        }
    }

  return n;
}

int p1_waypoint_save(void)
{
  int slot;
  int ret;

  if (!g_wp_ready)
    {
      return -EAGAIN;
    }

  g_wp_rec.magic   = P1_WP_MAGIC;
  g_wp_rec.version = P1_WP_VERSION;
  g_wp_rec.seq     = g_wp_seq + 1;
  g_wp_rec.count   = (uint32_t)p1_waypoint_count();
  g_wp_rec.crc32   = p1_wp_calc_crc(&g_wp_rec);

  slot = (int)((g_wp_seq + 1) & 1u);

  ret = p1_wp_write_slot(slot);
  if (ret < 0)
    {
      return ret;
    }

  g_wp_seq = g_wp_rec.seq;
  return OK;
}

int p1_waypoint_match(FAR const uint8_t *data, uint8_t len,
                      uint8_t threshold,
                      FAR struct p1_waypoint_s *out,
                      FAR uint8_t *score)
{
  int i;
  int best = -1;
  uint8_t best_score = 0;

  if (data == NULL || len == 0)
    {
      return -1;
    }

  if (threshold == 0)
    {
      threshold = 1;   /* never match on an all-empty comparison */
    }

  for (i = 0; i < P1_WP_MAX; i++)
    {
      FAR const struct p1_waypoint_s *e = &g_wp_rec.wp[i];
      uint8_t s;

      if (e->len == 0)
        {
          continue;
        }

      if (e->mode == P1_WP_MODE_EXACT)
        {
          s = (e->len == len &&
               memcmp(e->uid, data, len) == 0) ? 100u : 0u;
        }
      else
        {
          s = p1_wp_similarity(e->uid, e->len, data, len);
        }

      if (s >= threshold && s > best_score)
        {
          best_score = s;
          best = i;
        }
    }

  if (best < 0)
    {
      return -1;
    }

  if (out != NULL)
    {
      *out = g_wp_rec.wp[best];
    }

  if (score != NULL)
    {
      *score = best_score;
    }

  return best;
}
