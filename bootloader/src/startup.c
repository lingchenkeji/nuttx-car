/****************************************************************************
 * STM32F103 Startup Code for Bare-metal Bootloader
 ****************************************************************************/

#include <stdint.h>
#include "stm32f103.h"

/****************************************************************************
 * External Symbols from Linker Script
 ****************************************************************************/

extern uint32_t _sidata;    /* Start of init data in Flash */
extern uint32_t _sdata;     /* Start of .data in RAM */
extern uint32_t _edata;     /* End of .data in RAM */
extern uint32_t _sbss;      /* Start of .bss in RAM */
extern uint32_t _ebss;      /* End of .bss in RAM */
extern uint32_t _estack;    /* Top of stack */

/****************************************************************************
 * Function Prototypes
 ****************************************************************************/

extern int main(void);
void Reset_Handler(void);
void Default_Handler(void);

/* Cortex-M3 system exceptions */
void NMI_Handler(void)          __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void)    __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void)    __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void)          __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)       __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void)      __attribute__((weak, alias("Default_Handler")));

/* STM32F103 peripheral interrupts */
void USB_LP_CAN1_RX0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USB_HP_CAN1_TX_IRQHandler(void)  __attribute__((weak, alias("Default_Handler")));
void USBWakeUp_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));

/****************************************************************************
 * Vector Table
 ****************************************************************************/

__attribute__((section(".isr_vector")))
const uint32_t g_pfnVectors[] =
{
    (uint32_t)&_estack,             /* 0: Initial Stack Pointer */
    (uint32_t)Reset_Handler,        /* 1: Reset Handler */
    (uint32_t)NMI_Handler,          /* 2: NMI Handler */
    (uint32_t)HardFault_Handler,    /* 3: Hard Fault Handler */
    (uint32_t)MemManage_Handler,    /* 4: MPU Fault Handler */
    (uint32_t)BusFault_Handler,     /* 5: Bus Fault Handler */
    (uint32_t)UsageFault_Handler,   /* 6: Usage Fault Handler */
    0,                              /* 7: Reserved */
    0,                              /* 8: Reserved */
    0,                              /* 9: Reserved */
    0,                              /* 10: Reserved */
    (uint32_t)SVC_Handler,          /* 11: SVCall Handler */
    (uint32_t)DebugMon_Handler,     /* 12: Debug Monitor Handler */
    0,                              /* 13: Reserved */
    (uint32_t)PendSV_Handler,       /* 14: PendSV Handler */
    (uint32_t)SysTick_Handler,      /* 15: SysTick Handler */

    /* STM32F103 External Interrupts */
    0,                              /* 16: WWDG */
    0,                              /* 17: PVD */
    0,                              /* 18: TAMPER */
    0,                              /* 19: RTC */
    0,                              /* 20: FLASH */
    0,                              /* 21: RCC */
    0,                              /* 22: EXTI0 */
    0,                              /* 23: EXTI1 */
    0,                              /* 24: EXTI2 */
    0,                              /* 25: EXTI3 */
    0,                              /* 26: EXTI4 */
    0,                              /* 27: DMA1_Channel1 */
    0,                              /* 28: DMA1_Channel2 */
    0,                              /* 29: DMA1_Channel3 */
    0,                              /* 30: DMA1_Channel4 */
    0,                              /* 31: DMA1_Channel5 */
    0,                              /* 32: DMA1_Channel6 */
    0,                              /* 33: DMA1_Channel7 */
    0,                              /* 34: ADC1_2 */
    (uint32_t)USB_HP_CAN1_TX_IRQHandler,   /* 35: USB_HP_CAN1_TX */
    (uint32_t)USB_LP_CAN1_RX0_IRQHandler,  /* 36: USB_LP_CAN1_RX0 */
    0,                              /* 37: CAN1_RX1 */
    0,                              /* 38: CAN1_SCE */
    0,                              /* 39: EXTI9_5 */
    0,                              /* 40: TIM1_BRK */
    0,                              /* 41: TIM1_UP */
    0,                              /* 42: TIM1_TRG_COM */
    0,                              /* 43: TIM1_CC */
    0,                              /* 44: TIM2 */
    0,                              /* 45: TIM3 */
    0,                              /* 46: TIM4 */
    0,                              /* 47: I2C1_EV */
    0,                              /* 48: I2C1_ER */
    0,                              /* 49: I2C2_EV */
    0,                              /* 50: I2C2_ER */
    0,                              /* 51: SPI1 */
    0,                              /* 52: SPI2 */
    0,                              /* 53: USART1 */
    0,                              /* 54: USART2 */
    0,                              /* 55: USART3 */
    0,                              /* 56: EXTI15_10 */
    0,                              /* 57: RTC_Alarm */
    (uint32_t)USBWakeUp_IRQHandler, /* 58: USBWakeUp */
};

/****************************************************************************
 * Reset Handler
 ****************************************************************************/

void Reset_Handler(void)
{
    uint32_t *src, *dst;

    /* Copy .data section from Flash to RAM */
    src = &_sidata;
    dst = &_sdata;
    while (dst < &_edata)
    {
        *dst++ = *src++;
    }

    /* Zero out .bss section */
    dst = &_sbss;
    while (dst < &_ebss)
    {
        *dst++ = 0;
    }

    /* Call main() */
    main();

    /* Infinite loop if main returns */
    while (1);
}

/****************************************************************************
 * Default Handler for Unhandled Interrupts
 ****************************************************************************/

void Default_Handler(void)
{
    while (1);
}
