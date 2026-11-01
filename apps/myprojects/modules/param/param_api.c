/****************************************************************************
 * apps/myprojects/modules/param/param_api.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/
/****************************************************************************
 * 参数的默认值、按名字读写，以及双页 Flash 保存。
 * 改内存里的值不会自动落盘，要调用保存函数才写入。字段含义见 param_api.h。
 ****************************************************************************/


/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>

#include <nuttx/progmem.h>

#include "../../include/param_api.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define P1_PARAM_MAGIC         0x50315041u  /* "P1PA" */
#define P1_PARAM_VERSION       8u   /* bump: added match_threshold */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct p1_param_record_s
{
  uint32_t magic;
  uint32_t version;
  uint32_t seq;
  uint32_t length;
  struct p1_params_s cfg;
  uint32_t crc32;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_param_ready;
static uint32_t g_param_seq;
static size_t g_slot_block[2];
static size_t g_slot_addr[2];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t p1_crc32(FAR const uint8_t *data, size_t len)
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

static uint32_t p1_param_calc_crc(FAR const struct p1_param_record_s *rec)
{
  return p1_crc32((FAR const uint8_t *)rec,
                  sizeof(struct p1_param_record_s) - sizeof(uint32_t));
}

static int p1_param_read_slot(int slot,
                              FAR struct p1_param_record_s *out,
                              FAR bool *valid)
{
  FAR const struct p1_param_record_s *raw;
  uint32_t crc;

  raw = (FAR const struct p1_param_record_s *)(uintptr_t)g_slot_addr[slot];
  memcpy(out, raw, sizeof(struct p1_param_record_s));

  *valid = false;
  if (out->magic != P1_PARAM_MAGIC ||
      out->version != P1_PARAM_VERSION ||
      out->length != sizeof(struct p1_params_s))
    {
      return OK;
    }

  crc = p1_param_calc_crc(out);
  if (crc != out->crc32)
    {
      return OK;
    }

  *valid = true;
  return OK;
}

static int p1_param_write_slot(int slot, FAR const struct p1_param_record_s *rec)
{
  ssize_t ret;
  size_t block = g_slot_block[slot];
  size_t addr = g_slot_addr[slot];
  struct p1_param_record_s verify;

  ret = up_progmem_eraseblock(block);
  if (ret < 0)
    {
      return (int)ret;
    }

  ret = up_progmem_write(addr, rec, sizeof(struct p1_param_record_s));
  if (ret < 0)
    {
      return (int)ret;
    }

  if ((size_t)ret != sizeof(struct p1_param_record_s))
    {
      return -EIO;
    }

  memcpy(&verify, (FAR const void *)(uintptr_t)addr, sizeof(verify));
  if (memcmp(&verify, rec, sizeof(verify)) != 0)
    {
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void p1_param_factory_default(FAR struct p1_params_s *out)
{
  int i;

  if (out == NULL)
    {
      return;
    }

  out->rfid_target_block0 = 4;
  out->rfid_target_block1 = 5;
  out->rfid_target_block2 = 6;
  out->led_time           = 5;   /* 5 * 100ms = 0.5s */
  out->rgb_enable         = 1;
  out->rgb_brightness     = 15;
  out->motor_softstart    = 1;
  out->motor_ramp_ms      = 500;
  out->motor_ramp_step    = 5;
  out->rfid_cooldown_ms   = 800;
  out->vtx_dead_zone_ms   = 1000;
  out->match_threshold    = 60;

  for (i = 0; i < 10; i++)
    {
      out->gear_speed[i] = (uint32_t)(55 + i * 5);
    }
}

int p1_param_init(void)
{
  size_t nblocks;
  struct p1_param_record_s a;
  struct p1_param_record_s b;
  bool valid_a;
  bool valid_b;

  nblocks = up_progmem_neraseblocks();
  if (nblocks < 2)
    {
      return -ENOSPC;
    }

  g_slot_block[0] = nblocks - 2;
  g_slot_block[1] = nblocks - 1;
  g_slot_addr[0] = up_progmem_getaddress(g_slot_block[0]);
  g_slot_addr[1] = up_progmem_getaddress(g_slot_block[1]);

  if (g_slot_addr[0] == SIZE_MAX || g_slot_addr[1] == SIZE_MAX)
    {
      return -EFAULT;
    }

  p1_param_read_slot(0, &a, &valid_a);
  p1_param_read_slot(1, &b, &valid_b);

  if (valid_a && valid_b)
    {
      g_param_seq = (a.seq >= b.seq) ? a.seq : b.seq;
    }
  else if (valid_a)
    {
      g_param_seq = a.seq;
    }
  else if (valid_b)
    {
      g_param_seq = b.seq;
    }
  else
    {
      g_param_seq = 0;
    }

  g_param_ready = true;
  return OK;
}

int p1_param_load(FAR struct p1_params_s *out)
{
  struct p1_param_record_s a;
  struct p1_param_record_s b;
  bool valid_a;
  bool valid_b;

  if (out == NULL)
    {
      return -EINVAL;
    }

  if (!g_param_ready)
    {
      return -EAGAIN;
    }

  p1_param_read_slot(0, &a, &valid_a);
  p1_param_read_slot(1, &b, &valid_b);

  if (valid_a && valid_b)
    {
      if (a.seq >= b.seq)
        {
          *out = a.cfg;
          g_param_seq = a.seq;
        }
      else
        {
          *out = b.cfg;
          g_param_seq = b.seq;
        }

      return OK;
    }

  if (valid_a)
    {
      *out = a.cfg;
      g_param_seq = a.seq;
      return OK;
    }

  if (valid_b)
    {
      *out = b.cfg;
      g_param_seq = b.seq;
      return OK;
    }

  p1_param_factory_default(out);
  return -ENOENT;
}

int p1_param_save(FAR const struct p1_params_s *in)
{
  struct p1_param_record_s rec;
  int slot;
  int ret;

  if (in == NULL)
    {
      return -EINVAL;
    }

  if (!g_param_ready)
    {
      return -EAGAIN;
    }

  slot = (int)((g_param_seq + 1) & 1u);
  memset(&rec, 0xff, sizeof(rec));
  rec.magic = P1_PARAM_MAGIC;
  rec.version = P1_PARAM_VERSION;
  rec.seq = g_param_seq + 1;
  rec.length = sizeof(struct p1_params_s);
  rec.cfg = *in;
  rec.crc32 = p1_param_calc_crc(&rec);

  ret = p1_param_write_slot(slot, &rec);
  if (ret < 0)
    {
      return ret;
    }

  g_param_seq = rec.seq;
  return OK;
}

int p1_param_get_u32(FAR const struct p1_params_s *cfg,
                     FAR const char *key,
                     FAR uint32_t *value)
{
  if (cfg == NULL || key == NULL || value == NULL)
    {
      return -EINVAL;
    }

  if (strcmp(key, "rfid_target_block0") == 0)
    {
      *value = cfg->rfid_target_block0;
    }
  else if (strcmp(key, "rfid_target_block1") == 0)
    {
      *value = cfg->rfid_target_block1;
    }
  else if (strcmp(key, "rfid_target_block2") == 0)
    {
      *value = cfg->rfid_target_block2;
    }
  else if (strcmp(key, "led_time") == 0)
    {
      *value = cfg->led_time;
    }
  else if (strcmp(key, "rgb_enable") == 0)
    {
      *value = cfg->rgb_enable;
    }
  else if (strcmp(key, "rgb_brightness") == 0)
    {
      *value = cfg->rgb_brightness;
    }
  else if (strcmp(key, "motor_softstart") == 0)
    {
      *value = cfg->motor_softstart;
    }
  else if (strcmp(key, "motor_ramp_ms") == 0)
    {
      *value = cfg->motor_ramp_ms;
    }
  else if (strcmp(key, "motor_ramp_step") == 0)
    {
      *value = cfg->motor_ramp_step;
    }
  else if (strcmp(key, "rfid_cooldown_ms") == 0)
    {
      *value = cfg->rfid_cooldown_ms;
    }
  else if (strcmp(key, "vtx_dead_zone_ms") == 0)
    {
      *value = cfg->vtx_dead_zone_ms;
    }
  else if (strcmp(key, "match_threshold") == 0)
    {
      *value = cfg->match_threshold;
    }
  else if (strncmp(key, "gear_speed_", 11) == 0 &&
           key[11] >= '0' && key[11] <= '9' && key[12] == '\0')
    {
      *value = cfg->gear_speed[key[11] - '0'];
    }
  else
    {
      return -ENOENT;
    }

  return OK;
}

int p1_param_set_u32(FAR struct p1_params_s *cfg,
                     FAR const char *key,
                     uint32_t value)
{
  if (cfg == NULL || key == NULL)
    {
      return -EINVAL;
    }

  if (strcmp(key, "rfid_target_block0") == 0)
    {
      if (value > 63)
        {
          return -ERANGE;
        }

      cfg->rfid_target_block0 = value;
      return OK;
    }

  if (strcmp(key, "rfid_target_block1") == 0)
    {
      if (value > 63)
        {
          return -ERANGE;
        }

      cfg->rfid_target_block1 = value;
      return OK;
    }

  if (strcmp(key, "rfid_target_block2") == 0)
    {
      if (value > 63)
        {
          return -ERANGE;
        }

      cfg->rfid_target_block2 = value;
      return OK;
    }

  if (strcmp(key, "led_time") == 0)
    {
      if (value < 1 || value > 300)
        {
          return -ERANGE;
        }

      cfg->led_time = value;
      return OK;
    }

  if (strcmp(key, "rgb_enable") == 0)
    {
      if (value > 1)
        {
          return -ERANGE;
        }

      cfg->rgb_enable = value;
      return OK;
    }

  if (strcmp(key, "rgb_brightness") == 0)
    {
      if (value > 100)
        {
          return -ERANGE;
        }

      cfg->rgb_brightness = value;
      return OK;
    }

  if (strcmp(key, "motor_softstart") == 0)
    {
      if (value > 1)
        {
          return -ERANGE;
        }

      cfg->motor_softstart = value;
      return OK;
    }

  if (strcmp(key, "motor_ramp_ms") == 0)
    {
      if (value < 10 || value > 5000)
        {
          return -ERANGE;
        }

      cfg->motor_ramp_ms = value;
      return OK;
    }

  if (strcmp(key, "motor_ramp_step") == 0)
    {
      if (value < 1 || value > 20)
        {
          return -ERANGE;
        }

      cfg->motor_ramp_step = value;
      return OK;
    }

  if (strcmp(key, "rfid_cooldown_ms") == 0)
    {
      if (value < 200 || value > 10000)
        {
          return -ERANGE;
        }

      cfg->rfid_cooldown_ms = value;
      return OK;
    }

  if (strcmp(key, "vtx_dead_zone_ms") == 0)
    {
      if (value > 60000)
        {
          return -ERANGE;
        }

      cfg->vtx_dead_zone_ms = value;
      return OK;
    }

  if (strcmp(key, "match_threshold") == 0)
    {
      if (value > 100)
        {
          return -ERANGE;
        }

      cfg->match_threshold = value;
      return OK;
    }

  if (strncmp(key, "gear_speed_", 11) == 0 &&
      key[11] >= '0' && key[11] <= '9' && key[12] == '\0')
    {
      if (value > 100)
        {
          return -ERANGE;
        }

      cfg->gear_speed[key[11] - '0'] = value;
      return OK;
    }

  return -ENOENT;
}

