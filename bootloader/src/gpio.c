/****************************************************************************
 * STM32F103 GPIO Driver for Bootloader
 *
 * Provides:
 *   - PA15: DFU button input (with internal pull-up)
 *   - PC13: LED output (optional, for status indication)
 *
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "stm32f103.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpio_init
 *
 * Description:
 *   Initialize GPIO pins for bootloader
 *   - PA15: Input with pull-up (DFU button)
 *   - PC13: Output push-pull (LED)
 *
 ****************************************************************************/

void gpio_init(void)
{
    uint32_t temp;

    /* Configure PA15 as input with pull-up */
    temp = GPIOA->CRH;
    temp &= ~(0x0F << 28);                  /* Clear PA15 config (bit 28-31) */
    temp |= (GPIO_MODE_INPUT | GPIO_CNF_INPUT_PUPD) << 28;
    GPIOA->CRH = temp;

    /* Enable pull-up on PA15 by setting ODR bit */
    GPIOA->ODR |= (1 << 15);

    /* Configure PC13 as output push-pull 2MHz (LED) */
    temp = GPIOC->CRH;
    temp &= ~(0x0F << 20);                  /* Clear PC13 config (bit 20-23) */
    temp |= (GPIO_MODE_OUTPUT_2 | GPIO_CNF_OUTPUT_PP) << 20;
    GPIOC->CRH = temp;

    /* Turn off LED (PC13 is active low on Blue Pill) */
    GPIOC->BSRR = (1 << 13);
}

/****************************************************************************
 * Name: gpio_read_button
 *
 * Description:
 *   Read the state of DFU button (PA15)
 *
 * Returned Value:
 *   true if button is NOT pressed (PA15 is HIGH)
 *   false if button IS pressed (PA15 is LOW)
 *
 ****************************************************************************/

bool gpio_read_button(void)
{
    return (GPIOA->IDR & (1 << DFU_BUTTON_PIN)) != 0;
}

/****************************************************************************
 * Name: gpio_led_on
 *
 * Description:
 *   Turn on the LED (PC13 is active low)
 *
 ****************************************************************************/

void gpio_led_on(void)
{
    GPIOC->BRR = (1 << 13);
}

/****************************************************************************
 * Name: gpio_led_off
 *
 * Description:
 *   Turn off the LED (PC13 is active low)
 *
 ****************************************************************************/

void gpio_led_off(void)
{
    GPIOC->BSRR = (1 << 13);
}

/****************************************************************************
 * Name: gpio_led_toggle
 *
 * Description:
 *   Toggle the LED state
 *
 ****************************************************************************/

void gpio_led_toggle(void)
{
    GPIOC->ODR ^= (1 << 13);
}
