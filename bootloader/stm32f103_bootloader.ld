/****************************************************************************
 * STM32F103 Bare-metal USB DFU Bootloader Linker Script
 *
 * Flash Layout (128KB total):
 *   Bootloader: 0x08000000 - 0x08002FFF (12KB)
 *   Application: 0x08003000 - 0x0801FFFF (116KB)
 *
 ****************************************************************************/

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 12K
    SRAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 20K
}

/* Entry point */
ENTRY(Reset_Handler)

/* Stack size */
_Min_Stack_Size = 0x400;  /* 1KB */

SECTIONS
{
    /* Vector table - must be at flash start */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > FLASH

    /* Code */
    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.rodata)
        *(.rodata*)
        *(.glue_7)
        *(.glue_7t)
        KEEP(*(.init))
        KEEP(*(.fini))
        . = ALIGN(4);
        _etext = .;
    } > FLASH

    /* ARM exception handling */
    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH

    .ARM :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } > FLASH

    /* Used by startup to initialize data */
    _sidata = LOADADDR(.data);

    /* Initialized data - copied from Flash to RAM at startup */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > SRAM AT> FLASH

    /* Uninitialized data - zeroed at startup */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } > SRAM

    /* User heap - optional, not used in bootloader */
    ._user_heap_stack :
    {
        . = ALIGN(8);
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } > SRAM

    /* Top of stack */
    _estack = ORIGIN(SRAM) + LENGTH(SRAM);
}
