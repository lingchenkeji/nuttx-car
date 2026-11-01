/****************************************************************************
 * STM32F103 Bare-metal USB DFU Bootloader - Main
 *
 * Boot Flow:
 *   1. Check if DFU button (PA15) is pressed
 *   2. If pressed: Enter USB DFU mode, wait for firmware upgrade
 *   3. If not pressed: Jump to application at 0x08003000
 *
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "stm32f103.h"

/****************************************************************************
 * External Functions
 ****************************************************************************/

extern void system_init(void);
extern void gpio_init(void);
extern bool gpio_read_button(void);
extern void gpio_led_on(void);
extern void gpio_led_toggle(void);
extern void usb_dfu_init(void);
extern void usb_dfu_poll(void);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: jump_to_application
 *
 * Description:
 *   Jump to the application firmware at APP_ADDRESS
 *
 ****************************************************************************/

static void jump_to_application(void)
{
    uint32_t app_stack;
    uint32_t app_entry;
    void (*app_reset)(void);

    /* Get application stack pointer (first word) */
    app_stack = *(volatile uint32_t *)APP_ADDRESS;

    /* Get application reset vector (second word) */
    app_entry = *(volatile uint32_t *)(APP_ADDRESS + 4);

    /* Validate application - check if stack pointer is in SRAM range */
    if ((app_stack < SRAM_BASE) || (app_stack > (SRAM_BASE + 20 * 1024)))
    {
        /* Invalid application - stay in bootloader */
        return;
    }

    /* Disable all interrupts */
    __asm volatile ("cpsid i");

    /* Set vector table offset to application */
    /* SCB->VTOR = APP_ADDRESS */
    *(volatile uint32_t *)0xE000ED08 = APP_ADDRESS;

    /* Set main stack pointer */
    __asm volatile ("msr msp, %0" : : "r" (app_stack));

    /* Jump to application reset handler */
    app_reset = (void (*)(void))app_entry;
    app_reset();

    /* Should never reach here */
    while (1);
}

/****************************************************************************
 * Name: delay_ms
 *
 * Description:
 *   Simple delay in milliseconds (approximate)
 *
 ****************************************************************************/

static void delay_ms(uint32_t ms)
{
    /* Approximate delay at 72MHz */
    volatile uint32_t count = ms * 8000;
    while (count--)
    {
        __asm volatile ("nop");
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: main
 *
 * Description:
 *   Bootloader entry point
 *
 ****************************************************************************/

int main(void)
{
    bool enter_dfu = false;

    /* Initialize system clock */
    system_init();

    /* Initialize GPIO for button detection */
    gpio_init();

    /* Small delay for GPIO to stabilize */
    delay_ms(10);

    /* Check if DFU button (PA15) is pressed (LOW = pressed) */
    if (!gpio_read_button())
    {
        enter_dfu = true;
    }

    if (enter_dfu)
    {
        /* LED fast blink to indicate: DFU by button press */
        gpio_led_on();

        /* Enter USB DFU mode */
        usb_dfu_init();

        /* Process USB DFU requests in a loop */
        while (1)
        {
            usb_dfu_poll();
        }
    }
    else
    {
        /* Jump to application */
        jump_to_application();

        /* If jump fails (invalid application), enter DFU mode */
        /* LED slow blink to indicate: DFU by invalid app */
        int i;
        for (i = 0; i < 6; i++)
        {
            gpio_led_toggle();
            delay_ms(200);
        }

        usb_dfu_init();

        while (1)
        {
            usb_dfu_poll();
        }
    }

    return 0;
}
