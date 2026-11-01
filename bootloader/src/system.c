/****************************************************************************
 * STM32F103 System Initialization for Bootloader
 *
 * Clock Configuration:
 *   - HSE (8MHz external crystal)
 *   - PLL x9 = 72MHz system clock
 *   - USB clock = 48MHz (72MHz / 1.5)
 *
 ****************************************************************************/

#include <stdint.h>
#include "stm32f103.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: system_init
 *
 * Description:
 *   Initialize system clock to 72MHz using HSE and PLL
 *
 ****************************************************************************/

void system_init(void)
{
    /* Enable HSE (High Speed External) oscillator */
    RCC->CR |= RCC_CR_HSEON;

    /* Wait for HSE to be ready */
    while (!(RCC->CR & RCC_CR_HSERDY));

    /* Configure Flash latency for 72MHz (2 wait states) */
    FLASH_IF->ACR = 0x00000012;  /* 2 wait states + prefetch enable */

    /* Configure PLL:
     * - PLL source = HSE
     * - PLL multiplier = 9 (8MHz * 9 = 72MHz)
     * - USB prescaler = 1.5 (72MHz / 1.5 = 48MHz)
     */
    RCC->CFGR = RCC_CFGR_PLLSRC |           /* PLL source = HSE */
                (0x07 << 18) |               /* PLLMUL = 9 */
                (0x00 << 14) |               /* APB2 prescaler = 1 */
                (0x04 << 8);                 /* APB1 prescaler = 2 */

    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;

    /* Wait for PLL to be ready */
    while (!(RCC->CR & RCC_CR_PLLRDY));

    /* Select PLL as system clock source */
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    /* Wait for PLL to be used as system clock */
    while ((RCC->CFGR & 0x0C) != RCC_CFGR_SWS_PLL);

    /* Enable peripheral clocks */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN |    /* GPIOA */
                    RCC_APB2ENR_IOPCEN |    /* GPIOC (for LED) */
                    RCC_APB2ENR_AFIOEN;     /* AFIO */

    RCC->APB1ENR |= RCC_APB1ENR_USBEN;      /* USB */

    /* Disable JTAG to free PA15 (keep SWD enabled)
     * NOTE: SWJ_CFG bits (26:24) are WRITE-ONLY, must not read-modify-write.
     * Since bootloader doesn't use any other remap, just write directly.
     */
    AFIO->MAPR = AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
}
