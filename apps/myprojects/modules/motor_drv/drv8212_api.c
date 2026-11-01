/****************************************************************************
 * apps/myprojects/modules/motor_drv/drv8212_api.c
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
 * DRV8212 的引脚、PWM 调速和方向。速度百分比最终变成定时器脉宽。
 * 全速是另一条路，输出恒定电平，不走 PWM。接口说明在 include/drv8212_api.h。
 ****************************************************************************/


/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>

#ifdef CONFIG_ARCH_CHIP_STM32
#  include <stm32_gpio.h>
#endif

#include "../../include/drv8212_api.h"

/****************************************************************************
 * Pre-processor Definitions — TIM2 Hardware PWM
 *
 * DRV8212P supports PWM on IN1/IN2 up to 100 kHz (datasheet Section 7.3).
 * PA0 = TIM2_CH1, PA1 = TIM2_CH2 (default mapping, no remap needed).
 *
 * STM32F103 clocks:
 *   SYSCLK = 72 MHz, APB1 prescaler = /2 => PCLK1 = 36 MHz
 *   TIM2 clock = 2 × PCLK1 = 72 MHz (because APB1 prescaler > 1)
 *
 * Chosen PWM frequency: 20 kHz
 *   PSC  = 0            (no prescaling, counter runs at 72 MHz)
 *   ARR  = 72 000 000 / 20 000 − 1 = 3599
 *   CCRx = (ARR+1) × duty% / 100
 *
 * PWM Mode 1 (OCxM = 110): output HIGH while counter < CCRx.
 ****************************************************************************/

#ifdef CONFIG_ARCH_CHIP_STM32

/* ---- Register access -------------------------------------------------- */

#define DRV_GETREG32(a)       (*(volatile uint32_t *)(a))
#define DRV_PUTREG32(v, a)    (*(volatile uint32_t *)(a) = (v))

/* ---- RCC --------------------------------------------------------------- */

#define DRV_RCC_BASE          0x40021000u
#define DRV_RCC_APB1RSTR      (DRV_RCC_BASE + 0x10u)  /* APB1 peripheral reset */
#define DRV_RCC_APB1ENR       (DRV_RCC_BASE + 0x1cu)  /* APB1 peripheral clock enable */
#define DRV_RCC_TIM2EN        (1u << 0)
#define DRV_RCC_TIM2RST       (1u << 0)

/* ---- GPIOA registers (for debug read-back) ----------------------------- */

#define DRV_GPIOA_CRL         0x40010800u  /* GPIOA control register low */

/* ---- TIM2 registers (base 0x4000 0000) -------------------------------- */

#define DRV_TIM2_BASE         0x40000000u
#define DRV_TIM2_CR1          (DRV_TIM2_BASE + 0x00u)
#define DRV_TIM2_CR2          (DRV_TIM2_BASE + 0x04u)
#define DRV_TIM2_SMCR         (DRV_TIM2_BASE + 0x08u)
#define DRV_TIM2_DIER         (DRV_TIM2_BASE + 0x0cu)
#define DRV_TIM2_SR           (DRV_TIM2_BASE + 0x10u)
#define DRV_TIM2_EGR          (DRV_TIM2_BASE + 0x14u)
#define DRV_TIM2_CCMR1        (DRV_TIM2_BASE + 0x18u)
#define DRV_TIM2_CCER         (DRV_TIM2_BASE + 0x20u)
#define DRV_TIM2_CNT          (DRV_TIM2_BASE + 0x24u)
#define DRV_TIM2_PSC          (DRV_TIM2_BASE + 0x28u)
#define DRV_TIM2_ARR          (DRV_TIM2_BASE + 0x2cu)
#define DRV_TIM2_CCR1         (DRV_TIM2_BASE + 0x34u)
#define DRV_TIM2_CCR2         (DRV_TIM2_BASE + 0x38u)

/* CR1 bits */

#define DRV_CR1_CEN           (1u << 0)
#define DRV_CR1_ARPE          (1u << 7)

/* CCMR1 bits – output compare */

#define DRV_CCMR1_OC1PE       (1u << 3)
#define DRV_CCMR1_OC1M_SHIFT  4u
#define DRV_CCMR1_OC2PE       (1u << 11)
#define DRV_CCMR1_OC2M_SHIFT  12u
#define DRV_CCMR_PWM1         6u          /* PWM mode 1 */

/* CCMR1 OC mode values */

#define DRV_CCMR_FORCE_LO     4u          /* Force inactive (output LOW)  */
#define DRV_CCMR_FORCE_HI     5u          /* Force active   (output HIGH) */

/* CCER bits */

#define DRV_CCER_CC1E         (1u << 0)
#define DRV_CCER_CC2E         (1u << 4)

/* EGR bits */

#define DRV_EGR_UG            (1u << 0)

/* ---- PWM parameters --------------------------------------------------- */

#define DRV_TIM_CLK           72000000u
#define DRV_PWM_FREQ          20000u
#define DRV_PWM_ARR           ((DRV_TIM_CLK / DRV_PWM_FREQ) - 1u)  /* 3599 */

#endif /* CONFIG_ARCH_CHIP_STM32 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct drv8212_dev_s
{
  struct drv8212_pins_s pins;
  enum drv8212_mode_e mode;
  enum drv8212_mode_e saved_mode;   /* mode saved before sleep */
  uint8_t speed_percent;
  bool has_sleep_pin;
  bool has_dir_pin;
  bool last_dir;                    /* last direction pin state (for change detection) */
  bool sleeping;
  bool initialized;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_ARCH_CHIP_STM32

/****************************************************************************
 * Name: drv8212_pwm_init
 *
 * Description:
 *   Initialise TIM2 CH1 & CH2 for 20 kHz hardware PWM output.
 *   Must be called AFTER the GPIO pins have been set to alternate-function
 *   push-pull (GPIO_CNF_AFPP) via stm32_configgpio().
 *
 ****************************************************************************/

static void drv8212_pwm_init(void)
{
  uint32_t regval;
  uint32_t ccmr1;

  /* 1. Enable TIM2 peripheral clock in RCC APB1ENR */

  regval = DRV_GETREG32(DRV_RCC_APB1ENR);
  regval |= DRV_RCC_TIM2EN;
  DRV_PUTREG32(regval, DRV_RCC_APB1ENR);

  /* Small delay to let clock stabilize */

  up_udelay(2);

  /* 2. Reset TIM2 via RCC APB1RSTR to ensure clean state */

  regval = DRV_GETREG32(DRV_RCC_APB1RSTR);
  regval |= DRV_RCC_TIM2RST;
  DRV_PUTREG32(regval, DRV_RCC_APB1RSTR);

  /* Brief delay for reset */

  up_udelay(2);

  /* Release reset */

  regval &= ~DRV_RCC_TIM2RST;
  DRV_PUTREG32(regval, DRV_RCC_APB1RSTR);

  /* 3. Stop timer while configuring */

  DRV_PUTREG32(0, DRV_TIM2_CR1);

  /* 4. Explicitly clear all potentially-interfering registers */

  DRV_PUTREG32(0, DRV_TIM2_CR2);        /* No master mode */
  DRV_PUTREG32(0, DRV_TIM2_SMCR);       /* Internal clock source */
  DRV_PUTREG32(0, DRV_TIM2_DIER);       /* No interrupts, no DMA */
  DRV_PUTREG32(0, DRV_TIM2_SR);         /* Clear status flags */
  DRV_PUTREG32(0, DRV_TIM2_CNT);        /* Reset counter to 0 */

  /* 5. Prescaler = 0 → counter clock = TIM2 input clock = 72 MHz */

  DRV_PUTREG32(0, DRV_TIM2_PSC);

  /* 6. Auto-reload = 3599 → PWM period = 3600 / 72 MHz = 50 µs (20 kHz) */

  DRV_PUTREG32(DRV_PWM_ARR, DRV_TIM2_ARR);

  /* 7. CCMR1: CH1 & CH2 → PWM mode 1 + preload enable
   *    OC1M[2:0] = 110, OC1PE = 1
   *    OC2M[2:0] = 110, OC2PE = 1
   */

  ccmr1 = (DRV_CCMR_PWM1 << DRV_CCMR1_OC1M_SHIFT) | DRV_CCMR1_OC1PE |
           (DRV_CCMR_PWM1 << DRV_CCMR1_OC2M_SHIFT) | DRV_CCMR1_OC2PE;
  DRV_PUTREG32(ccmr1, DRV_TIM2_CCMR1);

  /* 8. CCER: enable CH1 and CH2 outputs (active-high polarity) */

  DRV_PUTREG32(DRV_CCER_CC1E | DRV_CCER_CC2E, DRV_TIM2_CCER);

  /* 9. Initial duty = 0 % on both channels (coast) */

  DRV_PUTREG32(0, DRV_TIM2_CCR1);
  DRV_PUTREG32(0, DRV_TIM2_CCR2);

  /* 10. Force update event so shadow registers (PSC, ARR, CCRx) load */

  DRV_PUTREG32(DRV_EGR_UG, DRV_TIM2_EGR);

  /* Clear the update flag that UG sets */

  DRV_PUTREG32(0, DRV_TIM2_SR);

  /* 11. Enable auto-reload preload + start counter */

  DRV_PUTREG32(DRV_CR1_ARPE | DRV_CR1_CEN, DRV_TIM2_CR1);

}

/****************************************************************************
 * Name: drv8212_pwm_deinit
 *
 * Description:
 *   Stop TIM2 PWM and set outputs to 0 %.
 *
 ****************************************************************************/

static void drv8212_pwm_deinit(void)
{
  DRV_PUTREG32(0, DRV_TIM2_CCR1);
  DRV_PUTREG32(0, DRV_TIM2_CCR2);
  DRV_PUTREG32(0, DRV_TIM2_CCER);
  DRV_PUTREG32(0, DRV_TIM2_CR1);
}

/****************************************************************************
 * Name: drv8212_dump_regs
 *
 * Description:
 *   Print TIM2 and GPIOA register values for debugging.
 *
 ****************************************************************************/

static void drv8212_dump_regs(void)
{
  uint32_t cnt1;
  uint32_t cnt2;

  printf("=== DRV8212 TIM2 Register Dump ===\n");
  printf("  RCC_APB1ENR  = 0x%08lx (TIM2EN bit0=%lu)\n",
         (unsigned long)DRV_GETREG32(DRV_RCC_APB1ENR),
         (unsigned long)(DRV_GETREG32(DRV_RCC_APB1ENR) & 1u));
  printf("  GPIOA_CRL    = 0x%08lx\n",
         (unsigned long)DRV_GETREG32(DRV_GPIOA_CRL));
  printf("  TIM2_CR1     = 0x%04lx (CEN=%lu ARPE=%lu)\n",
         (unsigned long)DRV_GETREG32(DRV_TIM2_CR1),
         (unsigned long)(DRV_GETREG32(DRV_TIM2_CR1) & 1u),
         (unsigned long)((DRV_GETREG32(DRV_TIM2_CR1) >> 7) & 1u));
  printf("  TIM2_SMCR    = 0x%04lx\n",
         (unsigned long)DRV_GETREG32(DRV_TIM2_SMCR));
  printf("  TIM2_CCMR1   = 0x%04lx\n",
         (unsigned long)DRV_GETREG32(DRV_TIM2_CCMR1));
  printf("  TIM2_CCER    = 0x%04lx (CC1E=%lu CC2E=%lu)\n",
         (unsigned long)DRV_GETREG32(DRV_TIM2_CCER),
         (unsigned long)(DRV_GETREG32(DRV_TIM2_CCER) & 1u),
         (unsigned long)((DRV_GETREG32(DRV_TIM2_CCER) >> 4) & 1u));
  printf("  TIM2_PSC     = %lu\n",
         (unsigned long)DRV_GETREG32(DRV_TIM2_PSC));
  printf("  TIM2_ARR     = %lu\n",
         (unsigned long)DRV_GETREG32(DRV_TIM2_ARR));
  printf("  TIM2_CCR1    = %lu\n",
         (unsigned long)DRV_GETREG32(DRV_TIM2_CCR1));
  printf("  TIM2_CCR2    = %lu\n",
         (unsigned long)DRV_GETREG32(DRV_TIM2_CCR2));

  /* Read CNT twice with a small delay to verify counter is running */

  cnt1 = DRV_GETREG32(DRV_TIM2_CNT);
  up_udelay(10);
  cnt2 = DRV_GETREG32(DRV_TIM2_CNT);
  printf("  TIM2_CNT     = %lu -> %lu (%s)\n",
         (unsigned long)cnt1,
         (unsigned long)cnt2,
         (cnt1 != cnt2) ? "RUNNING" : "STOPPED!");
  printf("==================================\n");
}

#endif /* CONFIG_ARCH_CHIP_STM32 */

/****************************************************************************
 * Name: drv8212_apply_mode
 *
 * Description:
 *   Apply the requested H-bridge mode by writing TIM2 CCR1/CCR2.
 *
 *   DRV8212P truth table (nSLEEP = 1):
 *     IN1  IN2 | OUT1  OUT2 | Description
 *      0    0  | Hi-Z  Hi-Z | Coast
 *      0    1  |  L     H   | Reverse (OUT2 → OUT1)
 *      1    0  |  H     L   | Forward (OUT1 → OUT2)
 *      1    1  |  L     L   | Brake (low-side slow decay)
 *
 *   Slow-decay PWM control (recommended for inductive motor loads):
 *
 *   Forward: IN1 = 100%, IN2 = inverted PWM
 *     PWM phase (IN2=LOW) → Forward (H,L) — motor driven
 *     Off phase (IN2=HIGH) → Brake  (L,L) — low-side FET recirculation
 *
 *   Reverse: IN2 = 100%, IN1 = inverted PWM
 *     PWM phase (IN1=LOW) → Reverse (L,H) — motor driven
 *     Off phase (IN1=HIGH) → Brake  (L,L) — low-side FET recirculation
 *
 *   This avoids Hi-Z states during PWM off-phase, preventing voltage
 *   spikes from motor back-EMF on the floating outputs.
 *
 ****************************************************************************/

static int drv8212_apply_mode(FAR struct drv8212_dev_s *dev,
                              enum drv8212_mode_e mode)
{
#ifdef CONFIG_ARCH_CHIP_STM32
  uint32_t ccr1 = 0;
  uint32_t ccr2 = 0;
  uint32_t full = DRV_PWM_ARR + 1u;    /* CCR value for 100 % duty */

  if (dev->sleeping)
    {
      return -EPERM;
    }

  switch (mode)
    {
      case DRV8212_MODE_FORWARD:
        ccr1 = full;                                       /* IN1 = 100% */
        ccr2 = full * (100u - dev->speed_percent) / 100u;  /* IN2 inverted */
        break;

      case DRV8212_MODE_REVERSE:
        ccr1 = full * (100u - dev->speed_percent) / 100u;  /* IN1 inverted */
        ccr2 = full;                                       /* IN2 = 100% */
        break;

      case DRV8212_MODE_BRAKE:
        ccr1 = full;        /* IN1 = 100 % */
        ccr2 = full;        /* IN2 = 100 % → Brake (L,L) */
        break;

      case DRV8212_MODE_COAST:
      default:
        ccr1 = 0;           /* IN1 = 0 % */
        ccr2 = 0;           /* IN2 = 0 % → Coast (Hi-Z,Hi-Z) */
        break;
    }

  DRV_PUTREG32(ccr1, DRV_TIM2_CCR1);
  DRV_PUTREG32(ccr2, DRV_TIM2_CCR2);

  dev->mode = mode;
  return OK;
#else
  return -ENOSYS;
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct drv8212_dev_s *drv8212_register(FAR const struct drv8212_pins_s *pins)
{
  FAR struct drv8212_dev_s *dev;

  if (pins == NULL)
    {
      return NULL;
    }

  dev = (FAR struct drv8212_dev_s *)kmm_zalloc(sizeof(struct drv8212_dev_s));
  if (dev == NULL)
    {
      printf("DRV8212: ERROR kmm_zalloc failed!\n");
      return NULL;
    }

  memcpy(&dev->pins, pins, sizeof(struct drv8212_pins_s));
  dev->speed_percent = 50;   /* Default 50% so PWM waveform is visible */
  dev->mode = DRV8212_MODE_COAST;

#ifdef CONFIG_ARCH_CHIP_STM32
  stm32_configgpio(dev->pins.in1pin);
  stm32_configgpio(dev->pins.in2pin);

  if (dev->pins.nsleeppin != 0)
    {
      dev->has_sleep_pin = true;
      stm32_configgpio(dev->pins.nsleeppin);
      stm32_gpiowrite(dev->pins.nsleeppin, true);
      dev->sleeping = false;
    }
  else
    {
      dev->has_sleep_pin = false;
      dev->sleeping = false;
    }

  if (dev->pins.dirpin != 0)
    {
      dev->has_dir_pin = true;
      stm32_configgpio(dev->pins.dirpin);
      dev->last_dir = stm32_gpioread(dev->pins.dirpin);
    }

  up_udelay(200);

  drv8212_pwm_init();
#else
  kmm_free(dev);
  return NULL;
#endif

  dev->initialized = true;
  printf("DRV8212: OK\n");
  return dev;
}

int drv8212_unregister(FAR struct drv8212_dev_s *dev)
{
  if (dev == NULL)
    {
      return -EINVAL;
    }

#ifdef CONFIG_ARCH_CHIP_STM32
  /* Stop PWM output */

  drv8212_pwm_deinit();

  /* Drive nSLEEP low → enter sleep */

  if (dev->pins.nsleeppin != 0)
    {
      stm32_gpiowrite(dev->pins.nsleeppin, false);
    }
#endif

  kmm_free(dev);
  return OK;
}

int drv8212_forward(FAR struct drv8212_dev_s *dev)
{
  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

  return drv8212_apply_mode(dev, DRV8212_MODE_FORWARD);
}

int drv8212_reverse(FAR struct drv8212_dev_s *dev)
{
  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

  return drv8212_apply_mode(dev, DRV8212_MODE_REVERSE);
}

int drv8212_brake(FAR struct drv8212_dev_s *dev)
{
  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

  return drv8212_apply_mode(dev, DRV8212_MODE_BRAKE);
}

int drv8212_coast(FAR struct drv8212_dev_s *dev)
{
  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

  return drv8212_apply_mode(dev, DRV8212_MODE_COAST);
}

int drv8212_set_speed(FAR struct drv8212_dev_s *dev, uint8_t speed_percent)
{
  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

  if (speed_percent > 100)
    {
      speed_percent = 100;
    }

  dev->speed_percent = speed_percent;

  /* Re-apply current mode with the new duty cycle */

  return drv8212_apply_mode(dev, dev->mode);
}

int drv8212_sleep(FAR struct drv8212_dev_s *dev)
{
  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

  if (!dev->has_sleep_pin)
    {
      return -ENODEV;
    }

  /* Save current mode, then coast before pulling nSLEEP low */

  dev->saved_mode = dev->mode;
  (void)drv8212_apply_mode(dev, DRV8212_MODE_COAST);

#ifdef CONFIG_ARCH_CHIP_STM32
  if (dev->pins.nsleeppin != 0)
    {
      stm32_gpiowrite(dev->pins.nsleeppin, false);
    }
#endif

  dev->sleeping = true;
  printf("DRV8212: sleep (saved mode=%d)\n", (int)dev->saved_mode);
  return OK;
}

int drv8212_wakeup(FAR struct drv8212_dev_s *dev)
{
  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

  if (!dev->has_sleep_pin)
    {
      return -ENODEV;
    }

#ifdef CONFIG_ARCH_CHIP_STM32
  if (dev->pins.nsleeppin != 0)
    {
      stm32_gpiowrite(dev->pins.nsleeppin, true);
    }
#endif

  /* Datasheet tWAKE = 100 µs max; busy-wait a bit */

  up_udelay(200);

  dev->sleeping = false;

  /* Restore the mode that was active before sleep */

  printf("DRV8212: wakeup (restoring mode=%d)\n", (int)dev->saved_mode);
  return drv8212_apply_mode(dev, dev->saved_mode);
}

int drv8212_get_state(FAR struct drv8212_dev_s *dev,
                      FAR enum drv8212_mode_e *mode,
                      FAR uint8_t *speed_percent,
                      FAR bool *sleeping)
{
  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

  if (mode != NULL)
    {
      *mode = dev->mode;
    }

  if (speed_percent != NULL)
    {
      *speed_percent = dev->speed_percent;
    }

  if (sleeping != NULL)
    {
      *sleeping = dev->sleeping;
    }

  return OK;
}

void drv8212_debug_dump(FAR struct drv8212_dev_s *dev)
{
#ifdef CONFIG_ARCH_CHIP_STM32
  if (dev == NULL || !dev->initialized)
    {
      printf("DRV8212: not initialized\n");
      return;
    }

  drv8212_dump_regs();
#else
  printf("DRV8212: debug dump not available (not STM32)\n");
#endif
}

/****************************************************************************
 * Name: drv8212_check_direction
 *
 * Description:
 *   Read the direction switch pin and update motor direction accordingly.
 *   HIGH = forward, LOW = reverse.
 *
 *   This function is lightweight and thread-safe for the GPIO read.
 *   It only calls drv8212_apply_mode() when the direction actually changes,
 *   making it efficient for periodic polling from a worker thread or timer.
 *
 * Returned Value:
 *    1 = direction changed and applied
 *    0 = no change
 *   <0 = error (not initialized, no dir pin, sleeping, etc.)
 *
 ****************************************************************************/

int drv8212_check_direction(FAR struct drv8212_dev_s *dev)
{
  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

  if (!dev->has_dir_pin)
    {
      return -ENODEV;
    }

  if (dev->sleeping)
    {
      return -EPERM;
    }

#ifdef CONFIG_ARCH_CHIP_STM32
  bool dir_now = stm32_gpioread(dev->pins.dirpin);
  bool need_apply = false;

  /* Apply direction if:
   *   1. The pin state changed from the last read, OR
   *   2. The motor is still in COAST/BRAKE (no explicit direction yet,
   *      e.g. first call after init).
   */

  if (dir_now != dev->last_dir)
    {
      need_apply = true;
    }
  else if (dev->mode != DRV8212_MODE_FORWARD &&
           dev->mode != DRV8212_MODE_REVERSE)
    {
      /* First call or motor was stopped — apply direction unconditionally */

      need_apply = true;
    }

  if (need_apply)
    {
      dev->last_dir = dir_now;

      if (dir_now)
        {
          /* HIGH = forward */

          drv8212_apply_mode(dev, DRV8212_MODE_FORWARD);
        }
      else
        {
          /* LOW = reverse */

          drv8212_apply_mode(dev, DRV8212_MODE_REVERSE);
        }

      return 1;  /* changed / applied */
    }

  return 0;  /* no change */
#else
  return -ENOSYS;
#endif
}

/****************************************************************************
 * Name: drv8212_force_fullspeed
 *
 * Description:
 *   Bypass PWM and force constant logic levels on IN1/IN2 using
 *   TIM2 "Force Active" / "Force Inactive" output compare modes.
 *   This gives a true DC output (no switching), safe to call from ISR.
 *
 *   Forward: IN1 = forced HIGH, IN2 = forced LOW  → OUT1=VM, OUT2=GND
 *   Reverse: IN1 = forced LOW,  IN2 = forced HIGH → OUT1=GND, OUT2=VM
 *
 ****************************************************************************/

int drv8212_force_fullspeed(FAR struct drv8212_dev_s *dev)
{
  if (dev == NULL || !dev->initialized)
    {
      return -EINVAL;
    }

#ifdef CONFIG_ARCH_CHIP_STM32
  {
    uint32_t ccmr1;
    bool forward = (dev->mode == DRV8212_MODE_FORWARD);

    if (forward)
      {
        ccmr1 = (DRV_CCMR_FORCE_HI << DRV_CCMR1_OC1M_SHIFT) |
                 DRV_CCMR1_OC1PE |
                (DRV_CCMR_FORCE_LO << DRV_CCMR1_OC2M_SHIFT) |
                 DRV_CCMR1_OC2PE;
      }
    else
      {
        ccmr1 = (DRV_CCMR_FORCE_LO << DRV_CCMR1_OC1M_SHIFT) |
                 DRV_CCMR1_OC1PE |
                (DRV_CCMR_FORCE_HI << DRV_CCMR1_OC2M_SHIFT) |
                 DRV_CCMR1_OC2PE;
      }

    DRV_PUTREG32(ccmr1, DRV_TIM2_CCMR1);
  }

  dev->speed_percent = 100;
  return OK;
#else
  return -ENOSYS;
#endif
}
