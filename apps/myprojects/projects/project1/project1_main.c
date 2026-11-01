/****************************************************************************
 * apps/myprojects/projects/project1/project1_main.c
 *
 * Project 1 Main Application - Single-Loop Architecture
 *
 * This application coordinates all hardware modules in a single main loop
 * (super-loop pattern) to minimize RAM usage on STM32F103C8T6 (20KB RAM).
 *
 * Modules:
 *   - DRV8212 motor driver (PA0/PA1/PA2/PA3)
 *   - CV520   RFID reader  (SPI1 + PB1 RST)
 *   - VTX316  voice TTS    (USART1 + PA8/PB12-15)
 *   - Functional LED        (PB11 via S8050)
 *   - WS2812B RGB LED       (PB8 = TIM4 CH3)
 *
 * Architecture:
 *   NO extra threads. Everything runs in one main loop:
 *   - RFID polling every ~200ms
 *   - VTX announce: non-blocking send + poll busy in main loop
 *   - LED: timestamp-based on/off in main loop
 *   - Motor direction: polled in main loop
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.
 *
 ****************************************************************************/
/****************************************************************************
 * 导读
 *
 * 这是小车的主程序，芯片是 STM32F103，内存很小，所以全程只有这一个循环，
 * 不再开别的线程。可执行语句和原来的当前版本一样，这里只补充中文说明。
 *
 * 建议按这个顺序看：
 *
 * 1. 开机先读旋钮，得到 0~9 档。每一档的速度在 g_cfg_gear_speed[] 里，
 *    默认从 55% 每隔一档加 5%，直到 100%。
 * 2. 再从 Flash 装参数。没存过就用上面的默认值。参数包括读哪一块卡、
 *    灯亮多久、同一张卡多久内不重复播报、点位要多像才算对上。
 * 3. USB 调试口（p1ctl）尽量早打开。语音模块上电可能要等好几秒，
 *    不能让电脑在这段时间里发命令一直失败。
 * 4. 电机驱动是 DRV8212。全速键在中断里处理，按下就直接 100%，
 *    不跟在这个循环后面排队。
 * 5. 读卡器是 CV520。循环里每次真读卡都走完这一串，顺序不要改：
 *    唤醒 → 防冲突拿到 UID → 选卡 → 用密钥认证 → 读 16 字节 →
 *    关闭加密 → 让卡休眠 → 清理芯片状态。
 *    少了“休眠”和“清理”，同一张卡下一轮经常不再应答。
 * 6. 点位表比对的是卡里读出的内容，不是 UID。对上以后可以分别决定
 *    亮功能灯、语音播报、把电机改到某个速度。没配过的卡默认亮灯并播报。
 *    防止短时间重复播报看的才是 UID：同一张卡还在静默窗口里就跳过声音和灯。
 * 7. 点位如果带了恢复时间，到点后速度回到旋钮这一档，不是回到上一张卡的速度。
 * 8. 氛围灯：初始化时琥珀灯，全部成功后慢闪绿色，有模块失败就快闪红色。
 *
 * 模块目录：
 *   modules/rfid/       读卡
 *   modules/vtx/        语音
 *   modules/motor_drv/  电机
 *   modules/waypoint/   点位
 *   modules/param/      掉电参数
 *   modules/cdc_svc/    电脑发来的 JSON
 *   modules/func_led/   功能灯
 *   modules/ws2812/     氛围灯
 *
 * 引脚以本文件里的 GPIO_* 宏为准，不要只看文件最上面的英文简介。
 ****************************************************************************/


/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>

#include <nuttx/spi/spi.h>
#include <nuttx/arch.h>

#ifdef CONFIG_ARCH_CHIP_STM32
#  include "stm32_spi.h"
#  include "stm32_gpio.h"
#  include "stm32_exti.h"
#endif

/* Module APIs */

#include "drv8212_api.h"
#include "rfid_api.h"
#include "vtx_api.h"
#include "func_led_api.h"
#include "ws2812_api.h"
#include "param_api.h"
#include "cdc_service_api.h"
#include "waypoint_api.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ---- DRV8212 Motor Pins ---- */

#define GPIO_DRV8212_IN1     (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz| \
                              GPIO_PORTA|GPIO_PIN0)
#define GPIO_DRV8212_IN2     (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz| \
                              GPIO_PORTA|GPIO_PIN1)
#define GPIO_DRV8212_EN      (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz| \
                              GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN3)
#define GPIO_DRV8212_DIR     (GPIO_INPUT|GPIO_CNF_INPULLUP|GPIO_MODE_INPUT| \
                              GPIO_PORTA|GPIO_PIN2)

/* ---- RFID (CV520) Pins ---- */

#define RFID_SPI_BUS          1
#define GPIO_CV520_RST       (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz| \
                              GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN1)

/* ---- F8U6 Voice Module (UART only, no GPIO needed) ---- */

#define VTX_DEVPATH           "/dev/ttyS0"

/* ---- Functional LED Pin ---- */

#define GPIO_FUNC_LED        (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz| \
                              GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN9)

/* ---- WS2812B RGB LED Pin (PB8 = TIM4 CH3, used as GPIO) ---- */

#define GPIO_WS2812_DIN      (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz| \
                              GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN8)

/* ---- Full-Speed Key (PC13, active low with internal pullup) ---- */

#define GPIO_FULLSPEED_KEY   (GPIO_INPUT|GPIO_CNF_INPULLUP|GPIO_MODE_INPUT| \
                              GPIO_PORTC|GPIO_PIN13)

/* ---- Rotary Encoder Pins (ESR-16H BCD on PB4/PB5/PB6/PB7) ---- */

#define GPIO_ENCODER_SHA  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT| \
                           GPIO_PORTB|GPIO_PIN4)
#define GPIO_ENCODER_SHB  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT| \
                           GPIO_PORTB|GPIO_PIN6)
#define GPIO_ENCODER_SHC  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT| \
                           GPIO_PORTB|GPIO_PIN5)
#define GPIO_ENCODER_SHD  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT| \
                           GPIO_PORTB|GPIO_PIN7)

/* ---- Timing Constants ---- */

#define RFID_TARGET_BLOCK         4      /* MIFARE block to read */
#define RFID_POLL_INTERVAL_MS     200    /* RFID poll period */
#define LED_NOTIFY_DURATION_MS    500    /* LED on duration after card */
#define MOTOR_DIR_CHECK_MS        500    /* Motor direction poll period */
#define MAIN_LOOP_TICK_MS         50     /* Main loop sleep tick */
#define HEARTBEAT_INTERVAL_MS     10000  /* Heartbeat print period */
#define RGB_SLOW_ON_MS            300    /* Slow blink on phase */
#define RGB_SLOW_OFF_MS           1700   /* Slow blink off phase (2s total) */
#define RGB_FAST_ON_MS            80     /* Fast blink on phase (fault) */
#define RGB_FAST_OFF_MS           80     /* Fast blink off phase (fault) */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Announce state machine */

enum announce_state_e
{
  ANNOUNCE_IDLE = 0,    /* Nothing to announce */
  ANNOUNCE_SPEAKING,    /* VTX is speaking, LED is on */
  ANNOUNCE_LED_WAIT,    /* VTX done, LED still on (waiting for timer) */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Module device handles */

static FAR struct drv8212_dev_s   *g_motor_dev;
static FAR struct rfid_dev_s     *g_rfid_dev;
static FAR struct vtx_dev_s      *g_vtx_dev;
static FAR struct func_led_dev_s *g_led_dev;
static FAR struct ws2812_dev_s   *g_rgb_led_dev;

static volatile bool g_rfid_pause = false;

/* Pin configurations */

static const struct drv8212_pins_s g_motor_pins =
{
  .in1pin    = GPIO_DRV8212_IN1,
  .in2pin    = GPIO_DRV8212_IN2,
  .nsleeppin = GPIO_DRV8212_EN,
  .dirpin    = GPIO_DRV8212_DIR,
};

static const struct vtx_pins_s g_vtx_pins =
{
  .rstpin   = 0,
  .wppin    = 0,
  .baud0pin = 0,
  .baud1pin = 0,
  .busypin  = 0,
};

static const struct func_led_pins_s g_led_pins =
{
  .ledpin = GPIO_FUNC_LED,
};

static const struct ws2812_pins_s g_rgb_led_pins =
{
  .dinpin = GPIO_WS2812_DIN,
};

/* RFID deduplication state (UID-based) */

static struct rfid_uid_s g_last_uid;
static bool    g_has_last_uid;
static uint32_t g_rfid_absent_since;   /* Timestamp when card first went absent */
static bool    g_rfid_absent_timing;   /* True if we're counting absence */

/* Announce dead-zone: track the most recently *announced* UID so a same UID
 * presented again within g_cfg_vtx_dead_zone_ms is silently suppressed.
 * A different UID, or the dead-zone elapsing, allows announce again. */

static struct rfid_uid_s g_last_played_uid;
static bool     g_has_last_played_uid;
static uint32_t g_last_played_tick;

/* RFID_NOCARD_COOLDOWN_MS: how long (ms) of continuous no-card before
 * UID state is cleared. Uses the rfid_cooldown_ms parameter at runtime,
 * but we define a compile-time fallback here. */

#define RFID_NOCARD_COOLDOWN_DEFAULT_MS  2000

/* Announce state */

static enum announce_state_e g_announce_state;
static uint32_t g_led_on_tick;       /* Tick when LED was turned on */

/* Flag: a new card arrived while VTX was still speaking.
 * When VTX finishes, main loop will probe card: if still present,
 * announce g_pending_data; if gone, skip. */

static bool g_new_card_waiting;
static uint8_t g_pending_data[16];

/* Running flag */

static volatile bool g_running;

/* Runtime-configurable parameters (written by CDC param.set) */

static volatile uint32_t g_cfg_rfid_target_block0;
static volatile uint32_t g_cfg_rfid_target_block1;
static volatile uint32_t g_cfg_rfid_target_block2;
static volatile uint32_t g_cfg_led_time_s;
static volatile uint32_t g_cfg_rgb_enable;
static volatile uint32_t g_cfg_rgb_brightness;
static volatile uint32_t g_cfg_motor_softstart;
static volatile uint32_t g_cfg_motor_ramp_ms;
static volatile uint32_t g_cfg_motor_ramp_step;
static volatile uint32_t g_cfg_rfid_cooldown_ms;
static volatile uint32_t g_cfg_vtx_dead_zone_ms;
static volatile uint32_t g_cfg_match_threshold;
static volatile uint32_t g_cfg_gear_speed[10];

/* Encoder reading (read once at boot) */

static int g_encoder_gear;

/* Motor soft-start ramp state */

static uint8_t  g_motor_current_speed;
static uint8_t  g_motor_target_speed;
static bool     g_motor_ramping;
static bool     g_motor_fullspeed_active;
static uint32_t g_motor_ramp_last_ms;
static bool     g_motor_restore_pending;
static uint8_t  g_motor_restore_target;
static uint32_t g_motor_restore_deadline_ms;

/* Init result: true = all OK (green blink), false = fault (red blink) */

static bool g_all_init_ok;

/****************************************************************************
 * Private Helper: get_elapsed_ms
 *
 * Description:
 *   Returns milliseconds elapsed since system boot (wraps at ~49 days).
 *   Uses CLOCK_MONOTONIC for accuracy.
 *
 ****************************************************************************/

static uint32_t get_tick_ms(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

/****************************************************************************
 * Name: fullspeed_key_isr
 *
 * Description:
 *   EXTI interrupt handler for PC13 (full-speed key).
 *   Fires on both edges — falling = pressed (active low), rising = released.
 *   Only touches TIM CCR registers via drv8212_set_speed(), safe in ISR.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_CHIP_STM32
static int fullspeed_key_isr(int irq, FAR void *context, FAR void *arg)
{
  bool pressed = !stm32_gpioread(GPIO_FULLSPEED_KEY);

  if (g_motor_dev == NULL)
    {
      return OK;
    }

  if (pressed && !g_motor_fullspeed_active)
    {
      g_motor_fullspeed_active = true;
      g_motor_ramping = false;
      g_motor_current_speed = 100;
      drv8212_set_speed(g_motor_dev, 100);
    }

  /* Release is intentionally ignored: speed stays at 100% until power-off. */

  return OK;
}
#endif

/****************************************************************************
 * Private Functions - Module Init
 ****************************************************************************/

/****************************************************************************
 * Name: p1_read_encoder
 *
 * Description:
 *   Read the ESR-16H 9-position BCD rotary encoder.
 *   Corrected mapping based on hardware measurement:
 *     SH_D (PB7) = BCD bit0 (weight 1)
 *     SH_B (PB6) = BCD bit1 (weight 2)
 *     SH_C (PB5) = BCD bit2 (weight 4)
 *     SH_A (PB4) = BCD bit3 (weight 8)
 *
 ****************************************************************************/

static int p1_read_encoder(void)
{
  int val = 0;

  stm32_configgpio(GPIO_ENCODER_SHA);
  stm32_configgpio(GPIO_ENCODER_SHB);
  stm32_configgpio(GPIO_ENCODER_SHC);
  stm32_configgpio(GPIO_ENCODER_SHD);

  if (stm32_gpioread(GPIO_ENCODER_SHD))
    {
      val |= 1;  /* PB7 = bit0, weight 1 */
    }

  if (stm32_gpioread(GPIO_ENCODER_SHB))
    {
      val |= 2;  /* PB6 = bit1, weight 2 */
    }

  if (stm32_gpioread(GPIO_ENCODER_SHC))
    {
      val |= 4;  /* PB5 = bit2, weight 4 */
    }

  if (stm32_gpioread(GPIO_ENCODER_SHA))
    {
      val |= 8;  /* PB4 = bit3, weight 8 */
    }

  if (val > 9)
    {
      val = 9;
    }

  return val;
}

static int p1_init_motor(void)
{
  int ret;

  g_motor_dev = drv8212_register(&g_motor_pins);
  if (g_motor_dev == NULL)
    {
      printf("[P1] ERR: Motor fail\n");
      return -EIO;
    }

  g_encoder_gear = p1_read_encoder();

  /* Look up target speed from gear table */

  g_motor_target_speed = (uint8_t)g_cfg_gear_speed[g_encoder_gear];
  if (g_motor_target_speed > 100)
    {
      g_motor_target_speed = 100;
    }

  g_motor_current_speed = 0;
  drv8212_set_speed(g_motor_dev, 0);

  ret = drv8212_check_direction(g_motor_dev);
  if (ret < 0)
    {
      drv8212_forward(g_motor_dev);
    }

  printf("[P1] Motor: OK gear=%d target=%u%%\n",
         (int)g_encoder_gear, (unsigned)g_motor_target_speed);
  return OK;
}

/* Speed selected by the encoder at boot, from the current gear table.
 * Waypoint recovery returns here.  It is not the previous point's speed.
 */

static uint8_t p1_encoder_base_speed(void)
{
  int gear = g_encoder_gear;
  uint32_t speed;

  if (gear < 0 || gear > 9)
    {
      gear = 0;
    }

  speed = g_cfg_gear_speed[gear];
  if (speed > 100u)
    {
      speed = 100u;
    }

  return (uint8_t)speed;
}

static int p1_init_rfid(void)
{
  FAR struct spi_dev_s *spi;

#ifdef CONFIG_ARCH_CHIP_STM32
  spi = stm32_spibus_initialize(RFID_SPI_BUS);
#else
  spi = NULL;
#endif

  if (spi == NULL)
    {
      printf("[P1] ERR: SPI fail\n");
      return -EIO;
    }

  g_rfid_dev = rfid_register(spi, GPIO_CV520_RST);
  if (g_rfid_dev == NULL)
    {
      printf("[P1] ERR: RFID fail\n");
      return -EIO;
    }

  printf("[P1] RFID: OK\n");
  return OK;
}

static int p1_init_vtx(void)
{
  g_vtx_dev = vtx_register(VTX_DEVPATH, &g_vtx_pins, VTX_BAUDRATE_115200);
  if (g_vtx_dev == NULL)
    {
      printf("[P1] ERR: VTX fail\n");
      return -EIO;
    }

  printf("[P1] VTX: OK\n");
  return OK;
}

static int p1_init_led(void)
{
  g_led_dev = func_led_register(&g_led_pins);
  if (g_led_dev == NULL)
    {
      printf("[P1] ERR: LED fail\n");
      return -EIO;
    }

  return OK;
}

static int p1_init_rgb_led(void)
{
  g_rgb_led_dev = ws2812_register(&g_rgb_led_pins);
  if (g_rgb_led_dev == NULL)
    {
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Private Functions - RFID Operations
 ****************************************************************************/

/* RFID state machine states */

enum rfid_state_e
{
  RFID_STATE_IDLE = 0,   /* No card, sending WUPA every cycle */
  RFID_STATE_READ,       /* Card found, reading block data */
  RFID_STATE_HOLD,       /* Card read done, monitoring presence */
};

static enum rfid_state_e g_rfid_state;

static bool p1_is_block_empty(FAR const uint8_t data[16])
{
  int i;

  for (i = 0; i < 16; i++)
    {
      if (data[i] != 0x00)
        {
          return false;
        }
    }

  return true;
}

static bool p1_is_same_uid(FAR const struct rfid_uid_s *a,
                           FAR const struct rfid_uid_s *b)
{
  if (a->size != b->size)
    {
      return false;
    }

  return (memcmp(a->uid, b->uid, a->size) == 0);
}

/****************************************************************************
 * Private Functions - Announce (Non-blocking state machine)
 ****************************************************************************/

/****************************************************************************
 * Name: p1_start_announce
 *
 * Description:
 *   Begin announcing card data. Queues the VTX frame and returns
 *   without waiting for the voice-module ACK/echo. LED timing is
 *   handled by the main loop.
 *
 ****************************************************************************/

static void p1_start_announce(FAR const uint8_t data[16])
{
  uint8_t gbkbuf[17];
  int gbklen = 0;
  int i;

  for (i = 0; i < 16; i++)
    {
      if (data[i] == 0x00)
        {
          break;
        }

      gbkbuf[gbklen++] = data[i];
    }

  gbkbuf[gbklen] = '\0';

  if (gbklen > 0 && g_vtx_dev != NULL)
    {
      vtx_speak_fast(g_vtx_dev, (FAR const char *)gbkbuf,
                     VTX_ENCODING_GBK);
    }

  g_announce_state = ANNOUNCE_LED_WAIT;
}

/****************************************************************************
 * Name: p1_poll_announce
 *
 * Description:
 *   Called from main loop to advance the announce state machine.
 *   Checks VTX busy and LED timer, transitions states.
 *
 ****************************************************************************/

static void p1_poll_announce(uint32_t now_ms)
{
  switch (g_announce_state)
    {
      case ANNOUNCE_IDLE:
        break;

      case ANNOUNCE_SPEAKING:
        /* No longer used — fall through to LED_WAIT */

        g_announce_state = ANNOUNCE_LED_WAIT;
        break;

      case ANNOUNCE_LED_WAIT:
        if ((get_tick_ms() - g_led_on_tick) >= (g_cfg_led_time_s * 100))
          {
            if (g_led_dev != NULL)
              {
                func_led_off(g_led_dev);
              }

            g_announce_state = ANNOUNCE_IDLE;
          }
        break;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: main (project1_main)
 *
 * Description:
 *   Project 1 entry point. Single-loop architecture.
 *   All modules are polled from one main loop — no extra threads.
 *
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  uint32_t now_ms;
  uint32_t last_dir_ms       = 0;
  uint32_t last_heartbeat_ms = 0;
  uint32_t rgb_toggle_ms     = 0;
  bool     rgb_on_phase      = false;
  uint8_t  rfid_data[16];
  int ret;

  struct p1_params_s params;
  struct p1_runtime_cfg_s runtime_cfg;
  struct p1_cdc_context_s cdc_ctx;

  printf("=== Project 1 Starting ===\n");

  g_running            = true;
  g_has_last_uid       = false;
  g_rfid_absent_timing = false;
  g_rfid_absent_since  = 0;
  g_rfid_state         = RFID_STATE_IDLE;
  g_announce_state     = ANNOUNCE_IDLE;
  g_new_card_waiting   = false;
  g_has_last_played_uid = false;
  g_last_played_tick   = 0;
  memset(g_pending_data, 0, sizeof(g_pending_data));
  memset(&g_last_uid, 0, sizeof(g_last_uid));
  memset(&g_last_played_uid, 0, sizeof(g_last_played_uid));

  /* ---- Load Parameters ---- */

  {
    int gi;

    g_cfg_rfid_target_block0 = RFID_TARGET_BLOCK;
    g_cfg_rfid_target_block1 = 5;
    g_cfg_rfid_target_block2 = 6;
    g_cfg_led_time_s         = LED_NOTIFY_DURATION_MS / 100;
    g_cfg_rgb_enable         = 1;
    g_cfg_rgb_brightness     = 15;
    g_cfg_motor_softstart    = 1;
    g_cfg_motor_ramp_ms      = 500;
    g_cfg_motor_ramp_step    = 5;
    g_cfg_rfid_cooldown_ms   = RFID_NOCARD_COOLDOWN_DEFAULT_MS;
    g_cfg_vtx_dead_zone_ms   = 1000;
    g_cfg_match_threshold    = 60;

    for (gi = 0; gi < 10; gi++)
      {
        g_cfg_gear_speed[gi] = (uint32_t)(55 + gi * 5);
      }

    ret = p1_param_init();
    if (ret == OK)
      {
        ret = p1_param_load(&params);
        if (ret == OK)
          {
            g_cfg_rfid_target_block0 = params.rfid_target_block0;
            g_cfg_rfid_target_block1 = params.rfid_target_block1;
            g_cfg_rfid_target_block2 = params.rfid_target_block2;
            g_cfg_led_time_s         = params.led_time;
            g_cfg_rgb_enable         = params.rgb_enable;
            g_cfg_rgb_brightness     = params.rgb_brightness;
            g_cfg_motor_softstart    = params.motor_softstart;
            g_cfg_motor_ramp_ms      = params.motor_ramp_ms;
            g_cfg_motor_ramp_step    = params.motor_ramp_step;
            g_cfg_rfid_cooldown_ms   = params.rfid_cooldown_ms;
            g_cfg_vtx_dead_zone_ms   = params.vtx_dead_zone_ms;
            g_cfg_match_threshold    = params.match_threshold;

            for (gi = 0; gi < 10; gi++)
              {
                g_cfg_gear_speed[gi] = params.gear_speed[gi];
              }

            printf("[P1] Params loaded\n");
          }
        else
          {
            printf("[P1] No saved params, using defaults\n");
          }
      }
    else
      {
        printf("[P1] WARN: param init fail %d\n", ret);
      }
  }

  /* Start the parameter service before the potentially slow peripheral
   * probes below.  USB/NSH becomes usable before VTX's ready wait (up to
   * several seconds), so a host never sees a valid p1ctl command return
   * "service not ready" merely because the voice module is still booting.
   *
   * The device handles are NULL at this point, which is intentional:
   * param.* and wp.* are already safe to use.  The second init below,
   * after all peripherals are registered, refreshes those handles for
   * rfid.*, vtx.* and motor.* commands.
   */

  {
    int gi;

    runtime_cfg.rfid_target_block0 = &g_cfg_rfid_target_block0;
    runtime_cfg.rfid_target_block1 = &g_cfg_rfid_target_block1;
    runtime_cfg.rfid_target_block2 = &g_cfg_rfid_target_block2;
    runtime_cfg.led_time_s         = &g_cfg_led_time_s;
    runtime_cfg.rgb_enable         = &g_cfg_rgb_enable;
    runtime_cfg.rgb_brightness     = &g_cfg_rgb_brightness;
    runtime_cfg.motor_softstart    = &g_cfg_motor_softstart;
    runtime_cfg.motor_ramp_ms      = &g_cfg_motor_ramp_ms;
    runtime_cfg.motor_ramp_step    = &g_cfg_motor_ramp_step;
    runtime_cfg.rfid_cooldown_ms   = &g_cfg_rfid_cooldown_ms;
    runtime_cfg.vtx_dead_zone_ms   = &g_cfg_vtx_dead_zone_ms;
    runtime_cfg.match_threshold    = &g_cfg_match_threshold;

    for (gi = 0; gi < 10; gi++)
      {
        runtime_cfg.gear_speed[gi] = &g_cfg_gear_speed[gi];
      }
  }

  cdc_ctx.rfid         = NULL;
  cdc_ctx.vtx          = NULL;
  cdc_ctx.motor        = NULL;
  cdc_ctx.encoder_gear = 0;
  cdc_ctx.runtime      = &runtime_cfg;
  cdc_ctx.rfid_pause   = &g_rfid_pause;

  ret = p1_cdc_service_init(&cdc_ctx);
  if (ret < 0)
    {
      printf("[P1] WARN: early CDC svc init %d\n", ret);
    }

  /* ---- Configure full-speed key GPIO (earliest possible) ---- */

#ifdef CONFIG_ARCH_CHIP_STM32
  stm32_configgpio(GPIO_FULLSPEED_KEY);
#endif
  g_motor_fullspeed_active = false;

  /* ---- Motor FIRST: register early so full-speed key works at boot ---- */

  ret = p1_init_motor();
  if (ret < 0)
    {
      printf("[P1] WARN: Motor fail\n");
    }

  /* Register EXTI interrupt on PC13 (both edges) — from this point on,
   * any key press/release is handled instantly by fullspeed_key_isr(),
   * even while VTX init blocks for ~2 seconds. */

#ifdef CONFIG_ARCH_CHIP_STM32
  if (g_motor_dev != NULL)
    {
      stm32_gpiosetevent(GPIO_FULLSPEED_KEY,
                         true,                /* rising edge  (released) */
                         true,                /* falling edge (pressed)  */
                         true,                /* event */
                         fullspeed_key_isr,
                         NULL);

      /* If key already held at power-on, ISR won't fire (no edge).
       * Do a one-time level check. */

      if (!stm32_gpioread(GPIO_FULLSPEED_KEY))
        {
          g_motor_fullspeed_active = true;
          g_motor_ramping = false;
          g_motor_current_speed = 100;
          drv8212_set_speed(g_motor_dev, 100);
          printf("[P1] Full-speed key held at boot -> 100%%\n");
        }
    }
#endif

  /* ---- Module Initialization with RGB status indication ---- */

  ret = p1_init_rgb_led();
  if (ret < 0)
    {
      printf("[P1] WARN: RGB LED fail, continue\n");
    }
  else
    {
      ws2812_set_brightness(g_rgb_led_dev, (uint8_t)g_cfg_rgb_brightness);
    }

  /* Amber pulse: initializing */

  if (g_rgb_led_dev)
    {
      ws2812_set_rgb(g_rgb_led_dev, 255, 40, 0);
    }

  {
    bool all_ok = true;
    bool motor_ok = (g_motor_dev != NULL);

    ret = p1_init_rfid();
    if (ret < 0)
      {
        printf("[P1] WARN: RFID fail\n");
        all_ok = false;
      }

    ret = p1_init_vtx();
    if (ret < 0)
      {
        printf("[P1] WARN: VTX fail\n");
        all_ok = false;
      }

    ret = p1_init_led();
    if (ret < 0)
      {
        printf("[P1] WARN: LED fail\n");
        all_ok = false;
      }

    if (!motor_ok)
      {
        all_ok = false;
      }

    g_all_init_ok = all_ok;

    if (g_rgb_led_dev)
      {
        if (all_ok)
          {
            ws2812_blink(g_rgb_led_dev, 0, 255, 0, 120, 80, 2);
          }
        else
          {
            ws2812_blink(g_rgb_led_dev, 255, 0, 0,
                         RGB_FAST_ON_MS, RGB_FAST_OFF_MS, 3);
          }
      }
  }

  /* ---- CDC Service Init ---- */

  {
    int gi;

    runtime_cfg.rfid_target_block0 = &g_cfg_rfid_target_block0;
    runtime_cfg.rfid_target_block1 = &g_cfg_rfid_target_block1;
    runtime_cfg.rfid_target_block2 = &g_cfg_rfid_target_block2;
    runtime_cfg.led_time_s         = &g_cfg_led_time_s;
    runtime_cfg.rgb_enable         = &g_cfg_rgb_enable;
    runtime_cfg.rgb_brightness     = &g_cfg_rgb_brightness;
    runtime_cfg.motor_softstart    = &g_cfg_motor_softstart;
    runtime_cfg.motor_ramp_ms      = &g_cfg_motor_ramp_ms;
    runtime_cfg.motor_ramp_step    = &g_cfg_motor_ramp_step;
    runtime_cfg.rfid_cooldown_ms   = &g_cfg_rfid_cooldown_ms;
    runtime_cfg.vtx_dead_zone_ms   = &g_cfg_vtx_dead_zone_ms;
    runtime_cfg.match_threshold    = &g_cfg_match_threshold;

    for (gi = 0; gi < 10; gi++)
      {
        runtime_cfg.gear_speed[gi] = &g_cfg_gear_speed[gi];
      }
  }

  cdc_ctx.rfid         = g_rfid_dev;
  cdc_ctx.vtx          = g_vtx_dev;
  cdc_ctx.motor        = g_motor_dev;
  cdc_ctx.encoder_gear = g_encoder_gear;
  cdc_ctx.runtime      = &runtime_cfg;
  cdc_ctx.rfid_pause   = &g_rfid_pause;

  ret = p1_cdc_service_init(&cdc_ctx);
  if (ret < 0)
    {
      printf("[P1] WARN: CDC svc init %d\n", ret);
    }

  /* Non-blocking startup announcement */

  if (g_vtx_dev != NULL)
    {
      vtx_speak(g_vtx_dev,
                "\xCF\xB5\xCD\xB3\xC6\xF4\xB6\xAF",
                VTX_ENCODING_GBK);
    }

  /* Start motor ramp (skip if full-speed key already activated) */

  if (g_motor_dev != NULL && !g_motor_fullspeed_active)
    {
      if (g_cfg_motor_softstart)
        {
          g_motor_ramping = true;
          g_motor_ramp_last_ms = get_tick_ms();
        }
      else
        {
          g_motor_current_speed = g_motor_target_speed;
          drv8212_set_speed(g_motor_dev, g_motor_current_speed);
          g_motor_ramping = false;
        }
    }

  rgb_toggle_ms = get_tick_ms() - RGB_SLOW_OFF_MS;

  printf("=== P1 Ready ===\n");

  /* ----------------------------------------------------------------
   * 主循环每一圈做六件事，圈末尾睡 5 毫秒，把 CPU 让出去：
   * 1. 读卡状态机。没卡就尝试唤醒；读成功就匹配点位；卡还在场内就保持，不重复播。
   * 2. 看语音说完没有，并按亮灯时间关功能灯。
   * 3. 看电机方向开关。
   * 4. 点位速度到时恢复，以及电机缓慢加减速。全速键不在这里，在中断里。
   * 5. 氛围灯按成功或故障闪。
   * 6. 每 10 秒打一行 alive，用来确认板子还在跑。
   * ---------------------------------------------------------------- */

  /* ================================================================
   * MAIN SUPER LOOP
   *
   * High-frequency RFID state machine + time-based peripheral checks.
   * No fixed tick — runs as fast as possible with 5ms yield.
   * ================================================================ */

  while (g_running)
    {
      now_ms = get_tick_ms();

      /* ---- 1. RFID state machine (runs every cycle) ---- */

      if (g_rfid_dev != NULL && !g_rfid_pause)
        {
          switch (g_rfid_state)
            {
              case RFID_STATE_IDLE:
                {
                  /* Send WUPA. If card responds, read in one shot. */

                  memset(rfid_data, 0, 16);

                  int r = rfid_request(g_rfid_dev);
                  if (r == OK)
                    {
                      /* Card awake. Get UID + select + auth + read in one shot. */

                      struct rfid_uid_s uid;
                      uint8_t key[6] = {0xff,0xff,0xff,0xff,0xff,0xff};
                      uint8_t blk = (uint8_t)g_cfg_rfid_target_block0;

                      r = rfid_anticoll(g_rfid_dev, &uid);
                      if (r < 0)
                        {
                          rfid_cleanup(g_rfid_dev);
                          break;
                        }

                      r = rfid_select(g_rfid_dev, &uid);
                      if (r < 0)
                        {
                          rfid_halt(g_rfid_dev);
                          rfid_cleanup(g_rfid_dev);
                          break;
                        }

                      r = rfid_authenticate_block(g_rfid_dev, &uid, blk,
                                                  key, false);
                      if (r < 0)
                        {
                          rfid_stop_crypto(g_rfid_dev);
                          rfid_halt(g_rfid_dev);
                          rfid_cleanup(g_rfid_dev);
                          break;
                        }

                      r = rfid_read_block(g_rfid_dev, blk, rfid_data);
                      rfid_stop_crypto(g_rfid_dev);
                      rfid_halt(g_rfid_dev);
                      rfid_cleanup(g_rfid_dev);

                      if (r == OK)
                        {
                          /* Success! Always update UID and enter HOLD.
                           * But suppress LED retrigger + announce while
                           * the LED-on window from the previous card is
                           * still active.
                           */

                          struct p1_waypoint_s wp;
                          bool wp_hit      = false;
                          uint8_t clen     = 0;

                          memcpy(&g_last_uid, &uid, sizeof(uid));
                          g_has_last_uid = true;
                          g_rfid_absent_timing = false;

                          /* Waypoint matching runs on EVERY successful
                           * read: the speed action is a control function
                           * and must never be missed, even while the
                           * LED/announce window of the previous card is
                           * still active.  LED + announce stay gated by
                           * the announce state machine below.
                           */

                          while (clen < 16 && rfid_data[clen] != 0x00)
                            {
                              clen++;
                            }

                          if (clen > 0 &&
                              p1_waypoint_match(rfid_data, clen,
                                  (uint8_t)g_cfg_match_threshold,
                                  &wp, NULL) >= 0)
                            {
                              wp_hit = true;

                              if (wp.speed_en &&
                                  !g_motor_fullspeed_active &&
                                  g_motor_dev != NULL)
                                {
                                  uint8_t sv = wp.speed;
                                  if (sv > 100)
                                    {
                                      sv = 100;
                                    }

                                  g_motor_target_speed = sv;
                                  g_motor_ramping       = true;
                                  g_motor_ramp_last_ms  = now_ms;

                                  /* A new speed action supersedes an older
                                   * pending recovery.  Recovery returns to
                                   * the encoder gear speed, not the speed
                                   * left by the previous point. */

                                  if (wp.restore_ms > 0)
                                    {
                                      g_motor_restore_target =
                                        p1_encoder_base_speed();
                                      g_motor_restore_deadline_ms =
                                        now_ms + wp.restore_ms;
                                      g_motor_restore_pending = true;
                                    }
                                  else
                                    {
                                      g_motor_restore_pending = false;
                                    }
                                }
                            }

                          if (g_announce_state == ANNOUNCE_IDLE)
                            {
                              /* Dead-zone gate: if the SAME UID was just
                               * announced within g_cfg_vtx_dead_zone_ms,
                               * skip both LED and audio.  A different UID,
                               * or expiry of the window, lets us through.
                               */

                              bool dead_zone_active = false;

                              if (g_has_last_played_uid &&
                                  g_cfg_vtx_dead_zone_ms > 0 &&
                                  p1_is_same_uid(&uid, &g_last_played_uid) &&
                                  (now_ms - g_last_played_tick)
                                      < g_cfg_vtx_dead_zone_ms)
                                {
                                  dead_zone_active = true;
                                }

                              if (!dead_zone_active)
                                {
                                  /* Matched point decides LED / announce.
                                   * No configured point (or no match)
                                   * falls back to light + announce.
                                   */

                                  bool do_led      = wp_hit ?
                                                     (wp.led != 0) : true;
                                  bool do_announce = wp_hit ?
                                                     (wp.announce != 0) : true;

                                  if (do_led && g_led_dev != NULL)
                                    {
                                      func_led_on(g_led_dev);
                                    }

                                  g_led_on_tick = get_tick_ms();
                                  g_announce_state = ANNOUNCE_LED_WAIT;

                                  if (clen > 0)
                                    {
                                      memcpy(&g_last_played_uid, &uid,
                                             sizeof(uid));
                                      g_has_last_played_uid = true;
                                      g_last_played_tick    = now_ms;
                                    }

                                  if (do_announce &&
                                      !p1_is_block_empty(rfid_data))
                                    {
                                      p1_start_announce(rfid_data);
                                    }
                                }
                            }

                          g_rfid_state = RFID_STATE_HOLD;
                        }
                      /* else: read failed, stay in IDLE, try next cycle */
                    }
                }
                break;

              case RFID_STATE_HOLD:
                {
                  /* Confirm card presence: WUPA + anticoll to verify UID.
                   * If WUPA fails or anticoll fails or UID differs:
                   * start 500ms timer → clear UID → back to IDLE. */

                  int r = rfid_request(g_rfid_dev);
                  if (r == OK)
                    {
                      struct rfid_uid_s uid;

                      r = rfid_anticoll(g_rfid_dev, &uid);
                      if (r == OK && p1_is_same_uid(&uid, &g_last_uid))
                        {
                          /* Same card confirmed — refresh LED-off timer
                           * so the LED stays lit for the full configured
                           * duration AFTER the card actually leaves the
                           * field, instead of starting the countdown from
                           * the moment the card was first detected.
                           */

                          g_rfid_absent_timing = false;

                          if (g_announce_state == ANNOUNCE_LED_WAIT)
                            {
                              g_led_on_tick = now_ms;
                            }
                        }
                      else if (r == OK)
                        {
                          /* Different card — immediate reset */

                          g_has_last_uid = false;
                          g_rfid_absent_timing = false;
                          g_rfid_state = RFID_STATE_IDLE;
                        }
                      else
                        {
                          /* anticoll failed - start RFID cooldown timer */

                          if (!g_rfid_absent_timing)
                            {
                              g_rfid_absent_timing = true;
                              g_rfid_absent_since = now_ms;
                            }
                          else if ((now_ms - g_rfid_absent_since) >=
                                   g_cfg_rfid_cooldown_ms)
                            {
                              g_has_last_uid = false;
                              g_rfid_absent_timing = false;
                              g_rfid_state = RFID_STATE_IDLE;
                            }
                        }
                    }
                  else
                    {
                      /* WUPA failed - start RFID cooldown timer */

                      if (!g_rfid_absent_timing)
                        {
                          g_rfid_absent_timing = true;
                          g_rfid_absent_since = now_ms;
                        }
                      else if ((now_ms - g_rfid_absent_since) >=
                               g_cfg_rfid_cooldown_ms)
                        {
                          g_has_last_uid = false;
                          g_rfid_absent_timing = false;
                          g_rfid_state = RFID_STATE_IDLE;
                        }
                    }
                }
                break;

              default:
                g_rfid_state = RFID_STATE_IDLE;
                break;
            }
        }

      /* ---- 2. Announce state machine (VTX busy + LED timer) ---- */

      p1_poll_announce(now_ms);

      /* ---- 3. Motor direction switch (every 500ms) ---- */

      if (g_motor_dev != NULL &&
          (now_ms - last_dir_ms) >= MOTOR_DIR_CHECK_MS)
        {
          last_dir_ms = now_ms;
          drv8212_check_direction(g_motor_dev);
        }

      /* ---- 4. Timed waypoint speed recovery + motor ramp ---- */
      /* Full-speed key is handled by EXTI ISR (zero-latency).
       * Main loop only drives the gradual ramp-up when active. */

      if (g_motor_restore_pending &&
          (uint32_t)(now_ms - g_motor_restore_deadline_ms) < 0x80000000u)
        {
          g_motor_restore_pending = false;

          if (g_motor_dev != NULL && !g_motor_fullspeed_active)
            {
              g_motor_target_speed = g_motor_restore_target;
              g_motor_ramping = true;
              g_motor_ramp_last_ms = now_ms;
            }
        }

      if (g_motor_dev != NULL && !g_motor_fullspeed_active &&
          g_motor_ramping &&
          (now_ms - g_motor_ramp_last_ms) >= g_cfg_motor_ramp_ms)
        {
          g_motor_ramp_last_ms = now_ms;

          if (g_motor_current_speed < g_motor_target_speed)
            {
              g_motor_current_speed += (uint8_t)g_cfg_motor_ramp_step;
              if (g_motor_current_speed > g_motor_target_speed)
                {
                  g_motor_current_speed = g_motor_target_speed;
                }
            }
          else if (g_motor_current_speed > g_motor_target_speed)
            {
              /* Smooth deceleration (e.g. a waypoint that lowers speed). */

              uint8_t step = (uint8_t)g_cfg_motor_ramp_step;

              if (g_motor_current_speed > step)
                {
                  g_motor_current_speed -= step;
                }
              else
                {
                  g_motor_current_speed = 0;
                }

              if (g_motor_current_speed < g_motor_target_speed)
                {
                  g_motor_current_speed = g_motor_target_speed;
                }
            }

          drv8212_set_speed(g_motor_dev, g_motor_current_speed);

          if (g_motor_current_speed == g_motor_target_speed)
            {
              g_motor_ramping = false;
            }
        }

      /* ---- 5. RGB status blink ---- */

      if (g_rgb_led_dev != NULL)
        {
          ws2812_set_brightness(g_rgb_led_dev,
                                (uint8_t)g_cfg_rgb_brightness);

          if (!g_all_init_ok)
            {
              /* Fault: red fast blink non-stop */

              if (rgb_on_phase)
                {
                  if ((now_ms - rgb_toggle_ms) >= RGB_FAST_ON_MS)
                    {
                      ws2812_off(g_rgb_led_dev);
                      rgb_on_phase = false;
                      rgb_toggle_ms = now_ms;
                    }
                }
              else
                {
                  if ((now_ms - rgb_toggle_ms) >= RGB_FAST_OFF_MS)
                    {
                      ws2812_set_rgb(g_rgb_led_dev, 255, 0, 0);
                      rgb_on_phase = true;
                      rgb_toggle_ms = now_ms;
                    }
                }
            }
          else if (g_cfg_rgb_enable == 0)
            {
              if (rgb_on_phase)
                {
                  ws2812_off(g_rgb_led_dev);
                  rgb_on_phase = false;
                }
            }
          else if (rgb_on_phase)
            {
              if ((now_ms - rgb_toggle_ms) >= RGB_SLOW_ON_MS)
                {
                  ws2812_off(g_rgb_led_dev);
                  rgb_on_phase = false;
                  rgb_toggle_ms = now_ms;
                }
            }
          else
            {
              if ((now_ms - rgb_toggle_ms) >= RGB_SLOW_OFF_MS)
                {
                  ws2812_set_rgb(g_rgb_led_dev, 0, 255, 0);
                  rgb_on_phase = true;
                  rgb_toggle_ms = now_ms;
                }
            }
        }

      /* ---- 6. Heartbeat (every 10s) ---- */

      if ((now_ms - last_heartbeat_ms) >= HEARTBEAT_INTERVAL_MS)
        {
          last_heartbeat_ms = now_ms;
          printf("[P1] alive\n");
        }

      /* ---- Yield CPU briefly ---- */

      usleep(5000);
    }

  /* ---- Cleanup ---- */

  printf("[P1] Shutdown\n");

  if (g_motor_dev != NULL)
    {
      drv8212_brake(g_motor_dev);
      drv8212_unregister(g_motor_dev);
      g_motor_dev = NULL;
    }

  if (g_led_dev != NULL)
    {
      func_led_off(g_led_dev);
      func_led_unregister(g_led_dev);
      g_led_dev = NULL;
    }

  if (g_rgb_led_dev != NULL)
    {
      ws2812_unregister(g_rgb_led_dev);
      g_rgb_led_dev = NULL;
    }

  if (g_vtx_dev != NULL)
    {
      vtx_unregister(g_vtx_dev);
      g_vtx_dev = NULL;
    }

  if (g_rfid_dev != NULL)
    {
      rfid_unregister(g_rfid_dev);
      g_rfid_dev = NULL;
    }

  return OK;
}
