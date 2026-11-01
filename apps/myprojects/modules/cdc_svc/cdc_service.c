/****************************************************************************
 * apps/myprojects/modules/cdc_svc/cdc_service.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/
/****************************************************************************
 * 解析电脑发来的一行 JSON，改参数、读写点位、试读卡、试播报。
 * 命令名字和字段看 projects/project1/CDC_JSON_PROTOCOL.md。
 * 这里改的是运行中的参数；点位和参数各自再保存，重启后才还在。
 ****************************************************************************/


/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>

#include <sys/boardctl.h>

#include "../../include/cdc_service_api.h"
#include "../../include/waypoint_api.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define P1_RESP_OK_FMT        "{\"id\":%lu,\"ok\":true,\"code\":0,\"msg\":\"OK\",%s}"
#define P1_RESP_ERR_FMT       "{\"id\":%lu,\"ok\":false,\"code\":%d,\"msg\":\"%s\"}"
#define P1_VTX_TEST_TIMEOUT_MS 10000

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct p1_cdc_context_s g_ctx;
static struct p1_params_s g_params;
static bool g_ctx_ready;
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR const char *p1_find_key(FAR const char *json, FAR const char *key)
{
  char pat[32];
  size_t len = strlen(key);

  if (len + 3 >= sizeof(pat))
    {
      return NULL;
    }

  pat[0] = '"';
  memcpy(&pat[1], key, len);
  pat[len + 1] = '"';
  pat[len + 2] = '\0';
  return strstr(json, pat);
}

static int p1_json_get_string(FAR const char *json,
                              FAR const char *key,
                              FAR char *out,
                              size_t out_len)
{
  FAR const char *p;
  FAR const char *s;
  FAR const char *e;
  size_t n;

  if (json == NULL || key == NULL || out == NULL || out_len < 2)
    {
      return -EINVAL;
    }

  p = p1_find_key(json, key);
  if (p == NULL)
    {
      return -ENOENT;
    }

  p = strchr(p, ':');
  if (p == NULL)
    {
      return -EINVAL;
    }

  p++;
  while (*p == ' ' || *p == '\t')
    {
      p++;
    }

  if (*p != '"')
    {
      return -EINVAL;
    }

  s = p + 1;
  e = strchr(s, '"');
  if (e == NULL)
    {
      return -EINVAL;
    }

  n = (size_t)(e - s);
  if (n >= out_len)
    {
      n = out_len - 1;
    }

  memcpy(out, s, n);
  out[n] = '\0';
  return OK;
}

static int p1_json_get_u32(FAR const char *json,
                           FAR const char *key,
                           FAR uint32_t *value)
{
  FAR const char *p;
  char *endp;
  unsigned long v;

  if (json == NULL || key == NULL || value == NULL)
    {
      return -EINVAL;
    }

  p = p1_find_key(json, key);
  if (p == NULL)
    {
      return -ENOENT;
    }

  p = strchr(p, ':');
  if (p == NULL)
    {
      return -EINVAL;
    }

  p++;
  while (*p == ' ' || *p == '\t')
    {
      p++;
    }

  if (*p < '0' || *p > '9')
    {
      return -EINVAL;
    }

  v = strtoul(p, &endp, 10);
  if (endp == p)
    {
      return -EINVAL;
    }

  *value = (uint32_t)v;
  return OK;
}

static unsigned long p1_json_get_id(FAR const char *json)
{
  uint32_t id;
  if (p1_json_get_u32(json, "id", &id) < 0)
    {
      return 0;
    }

  return (unsigned long)id;
}

static int p1_resp_ok(FAR char *resp, size_t resp_size,
                      unsigned long id, FAR const char *data_kv)
{
  return snprintf(resp, resp_size, P1_RESP_OK_FMT, id, data_kv);
}

static int p1_resp_err(FAR char *resp, size_t resp_size,
                       unsigned long id, int code, FAR const char *msg)
{
  return snprintf(resp, resp_size, P1_RESP_ERR_FMT, id, code, msg);
}

static int p1_cmd_sys_info(FAR char *resp, size_t resp_size, unsigned long id)
{
  char kv[128];

  snprintf(kv, sizeof(kv),
           "\"data\":{\"name\":\"project1\",\"transport\":\"cdcacm+nsh\","
           "\"protocol\":\"json-line\",\"dfu\":true}");
  return p1_resp_ok(resp, resp_size, id, kv);
}

static int p1_cmd_vtx_test(FAR const char *req,
                           FAR char *resp,
                           size_t resp_size,
                           unsigned long id)
{
  char text[64];
  uint32_t encoding = VTX_ENCODING_GBK;
  uint32_t timeout_ms = P1_VTX_TEST_TIMEOUT_MS;
  int ret;

  if (g_ctx.vtx == NULL)
    {
      return p1_resp_err(resp, resp_size, id, -ENODEV, "VTX not ready");
    }

  if (p1_json_get_string(req, "text", text, sizeof(text)) < 0)
    {
      strncpy(text, "VTX TEST", sizeof(text) - 1);
      text[sizeof(text) - 1] = '\0';
    }

  if (p1_json_get_u32(req, "encoding", &encoding) < 0)
    {
      encoding = VTX_ENCODING_GBK;
    }

  (void)p1_json_get_u32(req, "timeout_ms", &timeout_ms);

  ret = vtx_speak_and_wait(g_ctx.vtx, text, (uint8_t)encoding, timeout_ms);
  if (ret < 0)
    {
      return p1_resp_err(resp, resp_size, id, ret, "vtx_speak_and_wait failed");
    }

  return p1_resp_ok(resp, resp_size, id, "\"data\":{\"vtx_test\":\"ok\"}");
}

/* ── RFID retry parameters ── */

#define P1_RFID_RETRY_COUNT     5
#define P1_RFID_RETRY_DELAY_US  5000  /* 5ms between retries */

static void p1_rfid_pause_main(void)
{
  if (g_ctx.rfid_pause)
    {
      *g_ctx.rfid_pause = true;
      usleep(250000);
    }
}

static void p1_rfid_resume_main(void)
{
  if (g_ctx.rfid_pause)
    {
      *g_ctx.rfid_pause = false;
    }
}

/* ── RFID: request (card presence check) ── */

static int p1_cmd_rfid_request(FAR char *resp, size_t resp_size,
                               unsigned long id)
{
  int ret;
  int i;

  if (g_ctx.rfid == NULL)
    {
      return p1_resp_err(resp, resp_size, id, -ENODEV, "RFID not ready");
    }

  for (i = 0; i < P1_RFID_RETRY_COUNT; i++)
    {
      ret = rfid_request(g_ctx.rfid);
      if (ret == OK)
        {
          return p1_resp_ok(resp, resp_size, id,
                            "\"data\":{\"card\":true}");
        }

      usleep(P1_RFID_RETRY_DELAY_US);
    }

  return p1_resp_ok(resp, resp_size, id,
                    "\"data\":{\"card\":false}");
}

/* ── RFID: read UID (with retry) ── */

static int p1_cmd_rfid_read(FAR char *resp, size_t resp_size,
                            unsigned long id)
{
  struct rfid_uid_s uid;
  char kv[96];
  int ret;
  int i;

  if (g_ctx.rfid == NULL)
    {
      return p1_resp_err(resp, resp_size, id, -ENODEV, "RFID not ready");
    }

  for (i = 0; i < P1_RFID_RETRY_COUNT; i++)
    {
      ret = rfid_request(g_ctx.rfid);
      if (ret < 0)
        {
          usleep(P1_RFID_RETRY_DELAY_US);
          continue;
        }

      ret = rfid_anticoll(g_ctx.rfid, &uid);
      if (ret == OK)
        {
          snprintf(kv, sizeof(kv),
                   "\"data\":{\"uid\":\"%02X:%02X:%02X:%02X\",\"size\":%d}",
                   uid.uid[0], uid.uid[1], uid.uid[2], uid.uid[3],
                   (int)uid.size);
          return p1_resp_ok(resp, resp_size, id, kv);
        }

      usleep(P1_RFID_RETRY_DELAY_US);
    }

  return p1_resp_err(resp, resp_size, id, -ENOENT, "no card");
}

/* ── RFID: readblk (authenticate + read block, with retry) ── */

static int p1_cmd_rfid_readblk(FAR const char *req,
                               FAR char *resp,
                               size_t resp_size,
                               unsigned long id)
{
  struct rfid_uid_s uid;
  uint8_t key[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
  uint8_t data[16];
  uint32_t block = g_params.rfid_target_block0;
  int ret;
  int i;
  char kv[128];

  if (g_ctx.rfid == NULL)
    {
      return p1_resp_err(resp, resp_size, id, -ENODEV, "RFID not ready");
    }

  (void)p1_json_get_u32(req, "block", &block);

  for (i = 0; i < P1_RFID_RETRY_COUNT; i++)
    {
      ret = rfid_request(g_ctx.rfid);
      if (ret < 0)
        {
          usleep(P1_RFID_RETRY_DELAY_US);
          continue;
        }

      ret = rfid_anticoll(g_ctx.rfid, &uid);
      if (ret < 0)
        {
          usleep(P1_RFID_RETRY_DELAY_US);
          continue;
        }

      ret = rfid_select(g_ctx.rfid, &uid);
      if (ret < 0)
        {
          usleep(P1_RFID_RETRY_DELAY_US);
          continue;
        }

      ret = rfid_authenticate_block(g_ctx.rfid, &uid,
                                    (uint8_t)block, key, false);
      if (ret < 0)
        {
          (void)rfid_stop_crypto(g_ctx.rfid);
          usleep(P1_RFID_RETRY_DELAY_US);
          continue;
        }

      ret = rfid_read_block(g_ctx.rfid, (uint8_t)block, data);
      (void)rfid_stop_crypto(g_ctx.rfid);
      if (ret == OK)
        {
          snprintf(kv, sizeof(kv),
                   "\"data\":{\"uid\":\"%02X%02X%02X%02X\",\"block\":%lu,"
                   "\"hex\":\"%02X%02X%02X%02X%02X%02X%02X%02X"
                         "%02X%02X%02X%02X%02X%02X%02X%02X\"}",
                   uid.uid[0], uid.uid[1], uid.uid[2], uid.uid[3],
                   (unsigned long)block,
                   data[0], data[1], data[2], data[3],
                   data[4], data[5], data[6], data[7],
                   data[8], data[9], data[10], data[11],
                   data[12], data[13], data[14], data[15]);
          return p1_resp_ok(resp, resp_size, id, kv);
        }

      usleep(P1_RFID_RETRY_DELAY_US);
    }

  return p1_resp_err(resp, resp_size, id, -ENOENT, "no card");
}

/* ── VTX: speak hex data (GBK bytes) ── */

static int p1_cmd_vtx_hex(FAR const char *req,
                          FAR char *resp,
                          size_t resp_size,
                          unsigned long id)
{
  char hex[65];
  uint8_t buf[33];
  int i;
  int len;
  int ret;

  if (g_ctx.vtx == NULL)
    {
      return p1_resp_err(resp, resp_size, id, -ENODEV, "VTX not ready");
    }

  if (p1_json_get_string(req, "hex", hex, sizeof(hex)) < 0)
    {
      return p1_resp_err(resp, resp_size, id, -EINVAL, "missing hex");
    }

  /* Parse hex string (space or continuous, e.g. "B2E2CAD4" or "B2 E2") */

  len = 0;
  for (i = 0; hex[i] != '\0' && len < 32; )
    {
      uint8_t hi;
      uint8_t lo;
      char c;

      while (hex[i] == ' ')
        {
          i++;
        }

      if (hex[i] == '\0')
        {
          break;
        }

      c = hex[i++];
      if (c >= '0' && c <= '9')       hi = c - '0';
      else if (c >= 'A' && c <= 'F')  hi = c - 'A' + 10;
      else if (c >= 'a' && c <= 'f')  hi = c - 'a' + 10;
      else break;

      c = hex[i++];
      if (c >= '0' && c <= '9')       lo = c - '0';
      else if (c >= 'A' && c <= 'F')  lo = c - 'A' + 10;
      else if (c >= 'a' && c <= 'f')  lo = c - 'a' + 10;
      else break;

      buf[len++] = (hi << 4) | lo;
    }

  if (len == 0)
    {
      return p1_resp_err(resp, resp_size, id, -EINVAL, "empty hex");
    }

  buf[len] = '\0';
  ret = vtx_speak_and_wait(g_ctx.vtx, (FAR const char *)buf,
                           VTX_ENCODING_GBK, 10000);
  if (ret < 0)
    {
      return p1_resp_err(resp, resp_size, id, ret, "vtx hex failed");
    }

  return p1_resp_ok(resp, resp_size, id, "\"data\":{\"vtx_hex\":\"ok\"}");
}

/* ── VTX: stop ── */

static int p1_cmd_vtx_stop(FAR char *resp, size_t resp_size,
                           unsigned long id)
{
  if (g_ctx.vtx == NULL)
    {
      return p1_resp_err(resp, resp_size, id, -ENODEV, "VTX not ready");
    }

  vtx_stop(g_ctx.vtx);
  return p1_resp_ok(resp, resp_size, id, "\"data\":{\"stopped\":true}");
}

/* ── VTX: volume ── */

static int p1_cmd_vtx_volume(FAR const char *req,
                             FAR char *resp,
                             size_t resp_size,
                             unsigned long id)
{
  uint32_t vol;
  int ret;

  if (g_ctx.vtx == NULL)
    {
      return p1_resp_err(resp, resp_size, id, -ENODEV, "VTX not ready");
    }

  ret = p1_json_get_u32(req, "value", &vol);
  if (ret < 0)
    {
      return p1_resp_err(resp, resp_size, id, -EINVAL, "missing value");
    }

  ret = vtx_set_volume(g_ctx.vtx, (uint8_t)vol);
  if (ret < 0)
    {
      return p1_resp_err(resp, resp_size, id, ret, "set volume failed");
    }

  return p1_resp_ok(resp, resp_size, id, "\"data\":{\"volume_set\":true}");
}

/* ── VTX: status ── */

static int p1_cmd_vtx_status(FAR char *resp, size_t resp_size,
                             unsigned long id)
{
  int status;
  bool busy;
  char kv[64];

  if (g_ctx.vtx == NULL)
    {
      return p1_resp_err(resp, resp_size, id, -ENODEV, "VTX not ready");
    }

  status = vtx_query_status(g_ctx.vtx);
  busy = vtx_is_busy(g_ctx.vtx);

  snprintf(kv, sizeof(kv),
           "\"data\":{\"status\":%d,\"busy\":%s}",
           status, busy ? "true" : "false");
  return p1_resp_ok(resp, resp_size, id, kv);
}

/* ── VTX: speed ── */

static int p1_cmd_vtx_speed(FAR const char *req,
                            FAR char *resp,
                            size_t resp_size,
                            unsigned long id)
{
  uint32_t spd;
  int ret;

  if (g_ctx.vtx == NULL)
    {
      return p1_resp_err(resp, resp_size, id, -ENODEV, "VTX not ready");
    }

  ret = p1_json_get_u32(req, "value", &spd);
  if (ret < 0)
    {
      return p1_resp_err(resp, resp_size, id, -EINVAL, "missing value");
    }

  ret = vtx_set_speed(g_ctx.vtx, (uint8_t)spd);
  if (ret < 0)
    {
      return p1_resp_err(resp, resp_size, id, ret, "set speed failed");
    }

  return p1_resp_ok(resp, resp_size, id, "\"data\":{\"speed_set\":true}");
}

/* ── VTX: speaker ── */

static int p1_cmd_vtx_speaker(FAR const char *req,
                              FAR char *resp,
                              size_t resp_size,
                              unsigned long id)
{
  uint32_t spk;
  int ret;

  if (g_ctx.vtx == NULL)
    {
      return p1_resp_err(resp, resp_size, id, -ENODEV, "VTX not ready");
    }

  ret = p1_json_get_u32(req, "value", &spk);
  if (ret < 0)
    {
      return p1_resp_err(resp, resp_size, id, -EINVAL, "missing value");
    }

  ret = vtx_set_speaker(g_ctx.vtx, (uint8_t)spk);
  if (ret < 0)
    {
      return p1_resp_err(resp, resp_size, id, ret, "set speaker failed");
    }

  return p1_resp_ok(resp, resp_size, id, "\"data\":{\"speaker_set\":true}");
}

static void p1_apply_runtime(FAR const struct p1_params_s *cfg)
{
  int i;

  if (g_ctx.runtime == NULL || cfg == NULL)
    {
      return;
    }

  *g_ctx.runtime->rfid_target_block0 = cfg->rfid_target_block0;
  *g_ctx.runtime->rfid_target_block1 = cfg->rfid_target_block1;
  *g_ctx.runtime->rfid_target_block2 = cfg->rfid_target_block2;
  *g_ctx.runtime->led_time_s         = cfg->led_time;
  *g_ctx.runtime->rgb_enable         = cfg->rgb_enable;
  *g_ctx.runtime->rgb_brightness     = cfg->rgb_brightness;
  *g_ctx.runtime->motor_softstart    = cfg->motor_softstart;
  *g_ctx.runtime->motor_ramp_ms      = cfg->motor_ramp_ms;
  *g_ctx.runtime->motor_ramp_step    = cfg->motor_ramp_step;
  *g_ctx.runtime->rfid_cooldown_ms   = cfg->rfid_cooldown_ms;
  *g_ctx.runtime->vtx_dead_zone_ms   = cfg->vtx_dead_zone_ms;

  if (g_ctx.runtime->match_threshold != NULL)
    {
      *g_ctx.runtime->match_threshold = cfg->match_threshold;
    }

  for (i = 0; i < 10; i++)
    {
      *g_ctx.runtime->gear_speed[i] = cfg->gear_speed[i];
    }
}

static int p1_cmd_param_list(FAR char *resp, size_t resp_size, unsigned long id)
{
  static char kv[640];

  snprintf(kv, sizeof(kv),
           "\"data\":{"
           "\"rfid_target_block0\":%lu,"
           "\"rfid_target_block1\":%lu,"
           "\"rfid_target_block2\":%lu,"
           "\"led_time\":%lu,"
           "\"rgb_enable\":%lu,"
           "\"rgb_brightness\":%lu,"
           "\"motor_softstart\":%lu,"
           "\"motor_ramp_ms\":%lu,"
           "\"motor_ramp_step\":%lu,"
           "\"rfid_cooldown_ms\":%lu,"
           "\"vtx_dead_zone_ms\":%lu,"
           "\"match_threshold\":%lu,"
           "\"gear_speed_0\":%lu,"
           "\"gear_speed_1\":%lu,"
           "\"gear_speed_2\":%lu,"
           "\"gear_speed_3\":%lu,"
           "\"gear_speed_4\":%lu,"
           "\"gear_speed_5\":%lu,"
           "\"gear_speed_6\":%lu,"
           "\"gear_speed_7\":%lu,"
           "\"gear_speed_8\":%lu,"
           "\"gear_speed_9\":%lu}",
           (unsigned long)g_params.rfid_target_block0,
           (unsigned long)g_params.rfid_target_block1,
           (unsigned long)g_params.rfid_target_block2,
           (unsigned long)g_params.led_time,
           (unsigned long)g_params.rgb_enable,
           (unsigned long)g_params.rgb_brightness,
           (unsigned long)g_params.motor_softstart,
           (unsigned long)g_params.motor_ramp_ms,
           (unsigned long)g_params.motor_ramp_step,
           (unsigned long)g_params.rfid_cooldown_ms,
           (unsigned long)g_params.vtx_dead_zone_ms,
           (unsigned long)g_params.match_threshold,
           (unsigned long)g_params.gear_speed[0],
           (unsigned long)g_params.gear_speed[1],
           (unsigned long)g_params.gear_speed[2],
           (unsigned long)g_params.gear_speed[3],
           (unsigned long)g_params.gear_speed[4],
           (unsigned long)g_params.gear_speed[5],
           (unsigned long)g_params.gear_speed[6],
           (unsigned long)g_params.gear_speed[7],
           (unsigned long)g_params.gear_speed[8],
           (unsigned long)g_params.gear_speed[9]);
  return p1_resp_ok(resp, resp_size, id, kv);
}

static int p1_cmd_param_get(FAR const char *req,
                            FAR char *resp,
                            size_t resp_size,
                            unsigned long id)
{
  char key[32];
  uint32_t value;
  char kv[96];
  int ret;

  ret = p1_json_get_string(req, "key", key, sizeof(key));
  if (ret < 0)
    {
      return p1_resp_err(resp, resp_size, id, -EINVAL, "missing key");
    }

  ret = p1_param_get_u32(&g_params, key, &value);
  if (ret < 0)
    {
      return p1_resp_err(resp, resp_size, id, ret, "unknown key");
    }

  snprintf(kv, sizeof(kv), "\"data\":{\"key\":\"%s\",\"value\":%lu}",
           key, (unsigned long)value);
  return p1_resp_ok(resp, resp_size, id, kv);
}

static int p1_cmd_param_set(FAR const char *req,
                            FAR char *resp,
                            size_t resp_size,
                            unsigned long id)
{
  char key[32];
  uint32_t value;
  int ret;

  ret = p1_json_get_string(req, "key", key, sizeof(key));
  if (ret < 0)
    {
      return p1_resp_err(resp, resp_size, id, -EINVAL, "missing key");
    }

  ret = p1_json_get_u32(req, "value", &value);
  if (ret < 0)
    {
      return p1_resp_err(resp, resp_size, id, -EINVAL, "missing value");
    }

  ret = p1_param_set_u32(&g_params, key, value);
  if (ret < 0)
    {
      return p1_resp_err(resp, resp_size, id, ret, "param range/key error");
    }

  p1_apply_runtime(&g_params);
  return p1_resp_ok(resp, resp_size, id, "\"data\":{\"pending\":true}");
}

static int p1_cmd_param_save(FAR char *resp, size_t resp_size, unsigned long id)
{
  int ret = p1_param_save(&g_params);
  if (ret < 0)
    {
      return p1_resp_err(resp, resp_size, id, ret, "param save failed");
    }

  return p1_resp_ok(resp, resp_size, id, "\"data\":{\"saved\":true}");
}

static int p1_cmd_param_reset(FAR char *resp, size_t resp_size, unsigned long id)
{
  int ret;

  p1_param_factory_default(&g_params);
  p1_apply_runtime(&g_params);
  ret = p1_param_save(&g_params);
  if (ret < 0)
    {
      return p1_resp_err(resp, resp_size, id, ret, "param reset save failed");
    }

  return p1_resp_ok(resp, resp_size, id, "\"data\":{\"reset\":true}");
}

/* ── Waypoint (point-of-interest) commands ── */

static int p1_parse_hex(FAR const char *hex, FAR uint8_t *buf, int max)
{
  int i;
  int len = 0;

  for (i = 0; hex[i] != '\0' && len < max; )
    {
      uint8_t hi;
      uint8_t lo;
      char c;

      while (hex[i] == ' ')
        {
          i++;
        }

      if (hex[i] == '\0')
        {
          break;
        }

      c = hex[i++];
      if (c >= '0' && c <= '9')       hi = c - '0';
      else if (c >= 'A' && c <= 'F')  hi = c - 'A' + 10;
      else if (c >= 'a' && c <= 'f')  hi = c - 'a' + 10;
      else break;

      c = hex[i++];
      if (c >= '0' && c <= '9')       lo = c - '0';
      else if (c >= 'A' && c <= 'F')  lo = c - 'A' + 10;
      else if (c >= 'a' && c <= 'f')  lo = c - 'a' + 10;
      else break;

      buf[len++] = (uint8_t)((hi << 4) | lo);
    }

  return len;
}

static int p1_cmd_wp_count(FAR char *resp, size_t resp_size, unsigned long id)
{
  char kv[64];

  snprintf(kv, sizeof(kv), "\"data\":{\"count\":%d,\"max\":%d}",
           p1_waypoint_count(), P1_WP_MAX);
  return p1_resp_ok(resp, resp_size, id, kv);
}

static int p1_cmd_wp_get(FAR const char *req,
                         FAR char *resp, size_t resp_size,
                         unsigned long id)
{
  struct p1_waypoint_s wp;
  uint32_t index;
  char kv[224];
  char hex[2 * P1_WP_UID_MAXLEN + 1];
  int i;
  int ret;

  if (p1_json_get_u32(req, "index", &index) < 0)
    {
      return p1_resp_err(resp, resp_size, id, -EINVAL, "missing index");
    }

  ret = p1_waypoint_get((int)index, &wp);
  if (ret < 0)
    {
      return p1_resp_err(resp, resp_size, id, ret, "bad index");
    }

  if (wp.len > P1_WP_UID_MAXLEN)
    {
      wp.len = P1_WP_UID_MAXLEN;
    }

  for (i = 0; i < wp.len && i < P1_WP_UID_MAXLEN; i++)
    {
      snprintf(&hex[i * 2], 3, "%02X", wp.uid[i]);
    }

  hex[wp.len * 2] = '\0';

  snprintf(kv, sizeof(kv),
           "\"data\":{\"index\":%lu,\"len\":%u,\"mode\":%u,\"led\":%u,"
           "\"announce\":%u,\"speed_en\":%u,\"speed\":%u,"
           "\"restore_ms\":%u,\"hex\":\"%s\"}",
           (unsigned long)index, wp.len, wp.mode, wp.led,
           wp.announce, wp.speed_en, wp.speed, wp.restore_ms, hex);
  return p1_resp_ok(resp, resp_size, id, kv);
}

static int p1_cmd_wp_set(FAR const char *req,
                         FAR char *resp, size_t resp_size,
                         unsigned long id)
{
  struct p1_waypoint_s wp;
  uint32_t index;
  uint32_t v;

  /* Room for 16 bytes as space-separated hex ("B5 D8 ..." = 47 chars). */

  char hex[3 * P1_WP_UID_MAXLEN + 2];
  int ret;

  if (p1_json_get_u32(req, "index", &index) < 0)
    {
      return p1_resp_err(resp, resp_size, id, -EINVAL, "missing index");
    }

  memset(&wp, 0, sizeof(wp));

  if (p1_json_get_string(req, "hex", hex, sizeof(hex)) < 0)
    {
      return p1_resp_err(resp, resp_size, id, -EINVAL, "missing hex");
    }

  wp.len = (uint8_t)p1_parse_hex(hex, wp.uid, P1_WP_UID_MAXLEN);
  if (wp.len == 0)
    {
      return p1_resp_err(resp, resp_size, id, -EINVAL, "empty hex");
    }

  wp.mode     = (p1_json_get_u32(req, "mode", &v) == 0)     ? (uint8_t)v : 0;
  wp.led      = (p1_json_get_u32(req, "led", &v) == 0)      ? (uint8_t)v : 0;
  wp.announce = (p1_json_get_u32(req, "announce", &v) == 0) ? (uint8_t)v : 0;
  wp.speed_en = (p1_json_get_u32(req, "speed_en", &v) == 0) ? (uint8_t)v : 0;
  wp.speed    = (p1_json_get_u32(req, "speed", &v) == 0)    ? (uint8_t)v : 0;
  if (p1_json_get_u32(req, "restore_ms", &v) == 0)
    {
      if (v > P1_WP_RESTORE_MAX_MS)
        {
          return p1_resp_err(resp, resp_size, id, -ERANGE,
                             "restore_ms out of range");
        }

      wp.restore_ms = (uint16_t)v;
    }

  ret = p1_waypoint_set((int)index, &wp);
  if (ret < 0)
    {
      return p1_resp_err(resp, resp_size, id, ret, "wp set range/index");
    }

  return p1_resp_ok(resp, resp_size, id, "\"data\":{\"pending\":true}");
}

static int p1_cmd_wp_clear(FAR const char *req,
                           FAR char *resp, size_t resp_size,
                           unsigned long id)
{
  uint32_t index;
  int idx = -1;
  int ret;

  if (p1_json_get_u32(req, "index", &index) == 0)
    {
      idx = (int)index;
    }

  ret = p1_waypoint_clear(idx);
  if (ret < 0)
    {
      return p1_resp_err(resp, resp_size, id, ret, "wp clear failed");
    }

  return p1_resp_ok(resp, resp_size, id, "\"data\":{\"pending\":true}");
}

static int p1_cmd_wp_save(FAR char *resp, size_t resp_size, unsigned long id)
{
  int ret = p1_waypoint_save();
  if (ret < 0)
    {
      return p1_resp_err(resp, resp_size, id, ret, "wp save failed");
    }

  return p1_resp_ok(resp, resp_size, id, "\"data\":{\"saved\":true}");
}

/* ── Motor helpers ── */

static FAR const char *p1_mode_str(enum drv8212_mode_e m)
{
  switch (m)
    {
      case DRV8212_MODE_FORWARD: return "forward";
      case DRV8212_MODE_REVERSE: return "reverse";
      case DRV8212_MODE_BRAKE:   return "brake";
      default:                   return "coast";
    }
}

static int p1_cmd_motor_status(FAR char *resp, size_t resp_size,
                               unsigned long id)
{
  enum drv8212_mode_e mode;
  uint8_t spd;
  bool sleeping;
  char kv[192];
  int gear;
  unsigned long gear_spd;

  if (g_ctx.motor == NULL)
    {
      return p1_resp_err(resp, resp_size, id, -ENODEV, "motor not ready");
    }

  drv8212_get_state(g_ctx.motor, &mode, &spd, &sleeping);
  gear = g_ctx.encoder_gear;
  if (gear < 0)  gear = 0;
  if (gear > 9)  gear = 9;
  gear_spd = g_ctx.runtime ? *g_ctx.runtime->gear_speed[gear] : 0;

  snprintf(kv, sizeof(kv),
           "\"data\":{\"speed\":%d,\"mode\":\"%s\",\"sleeping\":%s,"
           "\"gear\":%d,\"gear_speed\":%lu,\"softstart\":%lu,"
           "\"ramp_ms\":%lu,\"ramp_step\":%lu}",
           (int)spd, p1_mode_str(mode),
           sleeping ? "true" : "false",
           gear, gear_spd,
           (unsigned long)(g_ctx.runtime ? *g_ctx.runtime->motor_softstart : 0),
           (unsigned long)(g_ctx.runtime ? *g_ctx.runtime->motor_ramp_ms : 0),
           (unsigned long)(g_ctx.runtime ? *g_ctx.runtime->motor_ramp_step : 0));
  return p1_resp_ok(resp, resp_size, id, kv);
}

static int p1_cmd_motor_speed(FAR const char *req,
                              FAR char *resp, size_t resp_size,
                              unsigned long id)
{
  uint32_t val;
  int ret;

  if (g_ctx.motor == NULL)
    {
      return p1_resp_err(resp, resp_size, id, -ENODEV, "motor not ready");
    }

  ret = p1_json_get_u32(req, "value", &val);
  if (ret < 0)
    {
      return p1_resp_err(resp, resp_size, id, -EINVAL, "missing value");
    }

  ret = drv8212_set_speed(g_ctx.motor, (uint8_t)val);
  if (ret < 0)
    {
      return p1_resp_err(resp, resp_size, id, ret, "set speed failed");
    }

  return p1_resp_ok(resp, resp_size, id, "\"data\":{\"speed_set\":true}");
}

static int p1_cmd_motor_mode(FAR const char *req,
                             FAR char *resp, size_t resp_size,
                             unsigned long id)
{
  char mode[16];
  int ret;

  if (g_ctx.motor == NULL)
    {
      return p1_resp_err(resp, resp_size, id, -ENODEV, "motor not ready");
    }

  ret = p1_json_get_string(req, "mode", mode, sizeof(mode));
  if (ret < 0)
    {
      return p1_resp_err(resp, resp_size, id, -EINVAL, "missing mode");
    }

  if (strcmp(mode, "forward") == 0)
    {
      drv8212_forward(g_ctx.motor);
    }
  else if (strcmp(mode, "reverse") == 0)
    {
      drv8212_reverse(g_ctx.motor);
    }
  else if (strcmp(mode, "brake") == 0)
    {
      drv8212_brake(g_ctx.motor);
    }
  else if (strcmp(mode, "coast") == 0)
    {
      drv8212_coast(g_ctx.motor);
    }
  else
    {
      return p1_resp_err(resp, resp_size, id, -EINVAL, "bad mode");
    }

  return p1_resp_ok(resp, resp_size, id, "\"data\":{\"mode_set\":true}");
}

static int p1_cmd_reboot(FAR const char *req,
                         FAR char *resp,
                         size_t resp_size,
                         unsigned long id)
{
#ifdef CONFIG_BOARDCTL_RESET
  uint32_t to_dfu = 0;
  (void)p1_json_get_u32(req, "to_dfu", &to_dfu);
  (void)to_dfu;
  (void)boardctl(BOARDIOC_RESET, 0);
  return p1_resp_ok(resp, resp_size, id, "\"data\":{\"reboot\":true}");
#else
  return p1_resp_err(resp, resp_size, id, -ENOSYS, "reset not enabled");
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int p1_cdc_service_init(FAR const struct p1_cdc_context_s *ctx)
{
  struct p1_params_s loaded;
  int ret;

  if (ctx == NULL)
    {
      return -EINVAL;
    }

  pthread_mutex_lock(&g_lock);

  g_ctx = *ctx;

  ret = p1_param_init();
  if (ret < 0)
    {
      pthread_mutex_unlock(&g_lock);
      return ret;
    }

  /* Load the waypoint table (independent flash pages). A failure here is
   * non-fatal: the table simply starts empty.
   */

  (void)p1_waypoint_init();

  ret = p1_param_load(&loaded);
  if (ret < 0 && ret != -ENOENT)
    {
      pthread_mutex_unlock(&g_lock);
      return ret;
    }

  g_params = loaded;
  p1_apply_runtime(&g_params);
  g_ctx_ready = true;
  pthread_mutex_unlock(&g_lock);
  return OK;
}

int p1_cdc_handle_json(FAR const char *req, FAR char *resp, size_t resp_size)
{
  char cmd[32];
  unsigned long id;
  int ret;
  struct timespec ts;

  if (req == NULL || resp == NULL || resp_size < 64)
    {
      return -EINVAL;
    }

  id = p1_json_get_id(req);

  /* Use timed lock (3s) to avoid blocking forever on deadlock */

  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_sec += 3;
  ret = pthread_mutex_timedlock(&g_lock, &ts);
  if (ret != 0)
    {
      p1_resp_err(resp, resp_size, id, -EBUSY,
                  "lock timeout - service may be initializing");
      return -EBUSY;
    }

  if (!g_ctx_ready)
    {
      pthread_mutex_unlock(&g_lock);
      p1_resp_err(resp, resp_size, id, -EAGAIN, "service not ready");
      return -EAGAIN;
    }

  ret = p1_json_get_string(req, "cmd", cmd, sizeof(cmd));
  if (ret < 0)
    {
      pthread_mutex_unlock(&g_lock);
      p1_resp_err(resp, resp_size, id, -EINVAL, "missing cmd");
      return -EINVAL;
    }

  if (strcmp(cmd, "sys.ping") == 0)
    {
      p1_resp_ok(resp, resp_size, id, "\"data\":{\"pong\":true}");
      ret = OK;
    }
  else if (strcmp(cmd, "sys.info") == 0)
    {
      ret = p1_cmd_sys_info(resp, resp_size, id);
    }
  else if (strcmp(cmd, "rfid.test") == 0 ||
           strcmp(cmd, "rfid.readblk") == 0)
    {
      p1_rfid_pause_main();
      ret = p1_cmd_rfid_readblk(req, resp, resp_size, id);
      p1_rfid_resume_main();
    }
  else if (strcmp(cmd, "rfid.request") == 0)
    {
      p1_rfid_pause_main();
      ret = p1_cmd_rfid_request(resp, resp_size, id);
      p1_rfid_resume_main();
    }
  else if (strcmp(cmd, "rfid.read") == 0)
    {
      p1_rfid_pause_main();
      ret = p1_cmd_rfid_read(resp, resp_size, id);
      p1_rfid_resume_main();
    }
  else if (strcmp(cmd, "vtx.test") == 0)
    {
      ret = p1_cmd_vtx_test(req, resp, resp_size, id);
    }
  else if (strcmp(cmd, "vtx.hex") == 0)
    {
      ret = p1_cmd_vtx_hex(req, resp, resp_size, id);
    }
  else if (strcmp(cmd, "vtx.stop") == 0)
    {
      ret = p1_cmd_vtx_stop(resp, resp_size, id);
    }
  else if (strcmp(cmd, "vtx.volume") == 0)
    {
      ret = p1_cmd_vtx_volume(req, resp, resp_size, id);
    }
  else if (strcmp(cmd, "vtx.status") == 0)
    {
      ret = p1_cmd_vtx_status(resp, resp_size, id);
    }
  else if (strcmp(cmd, "vtx.speed") == 0)
    {
      ret = p1_cmd_vtx_speed(req, resp, resp_size, id);
    }
  else if (strcmp(cmd, "vtx.speaker") == 0)
    {
      ret = p1_cmd_vtx_speaker(req, resp, resp_size, id);
    }
  else if (strcmp(cmd, "param.list") == 0)
    {
      ret = p1_cmd_param_list(resp, resp_size, id);
    }
  else if (strcmp(cmd, "param.get") == 0)
    {
      ret = p1_cmd_param_get(req, resp, resp_size, id);
    }
  else if (strcmp(cmd, "param.set") == 0)
    {
      ret = p1_cmd_param_set(req, resp, resp_size, id);
    }
  else if (strcmp(cmd, "param.save") == 0)
    {
      ret = p1_cmd_param_save(resp, resp_size, id);
    }
  else if (strcmp(cmd, "param.reset") == 0)
    {
      ret = p1_cmd_param_reset(resp, resp_size, id);
    }
  else if (strcmp(cmd, "wp.count") == 0)
    {
      ret = p1_cmd_wp_count(resp, resp_size, id);
    }
  else if (strcmp(cmd, "wp.get") == 0)
    {
      ret = p1_cmd_wp_get(req, resp, resp_size, id);
    }
  else if (strcmp(cmd, "wp.set") == 0)
    {
      ret = p1_cmd_wp_set(req, resp, resp_size, id);
    }
  else if (strcmp(cmd, "wp.clear") == 0)
    {
      ret = p1_cmd_wp_clear(req, resp, resp_size, id);
    }
  else if (strcmp(cmd, "wp.save") == 0)
    {
      ret = p1_cmd_wp_save(resp, resp_size, id);
    }
  else if (strcmp(cmd, "motor.status") == 0)
    {
      ret = p1_cmd_motor_status(resp, resp_size, id);
    }
  else if (strcmp(cmd, "motor.speed") == 0)
    {
      ret = p1_cmd_motor_speed(req, resp, resp_size, id);
    }
  else if (strcmp(cmd, "motor.mode") == 0)
    {
      ret = p1_cmd_motor_mode(req, resp, resp_size, id);
    }
  else if (strcmp(cmd, "sys.reboot") == 0)
    {
      ret = p1_cmd_reboot(req, resp, resp_size, id);
    }
  else
    {
      p1_resp_err(resp, resp_size, id, -ENOENT, "unknown cmd");
      ret = -ENOENT;
    }

  pthread_mutex_unlock(&g_lock);
  return ret;
}

