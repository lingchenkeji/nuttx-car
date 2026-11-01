/****************************************************************************
 * STM32F103 Flash Driver for Bootloader
 *
 * Provides:
 *   - Flash unlock/lock
 *   - Page erase (1KB pages on STM32F103)
 *   - Half-word programming
 *
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "stm32f103.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FLASH_PAGE_SIZE     1024        /* 1KB per page on STM32F103 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: flash_unlock
 *
 * Description:
 *   Unlock the flash for programming
 *
 * Returned Value:
 *   true on success, false on failure
 *
 ****************************************************************************/

bool flash_unlock(void)
{
    if (FLASH_IF->CR & FLASH_CR_LOCK)
    {
        /* Unlock sequence */
        FLASH_IF->KEYR = FLASH_KEY1;
        FLASH_IF->KEYR = FLASH_KEY2;
    }

    return !(FLASH_IF->CR & FLASH_CR_LOCK);
}

/****************************************************************************
 * Name: flash_lock
 *
 * Description:
 *   Lock the flash after programming
 *
 ****************************************************************************/

void flash_lock(void)
{
    FLASH_IF->CR |= FLASH_CR_LOCK;
}

/****************************************************************************
 * Name: flash_wait_busy
 *
 * Description:
 *   Wait for flash operation to complete
 *
 * Returned Value:
 *   true on success, false on error
 *
 ****************************************************************************/

static bool flash_wait_busy(void)
{
    /* Wait for BSY flag to be cleared */
    while (FLASH_IF->SR & FLASH_SR_BSY);

    /* Check for errors */
    if (FLASH_IF->SR & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR))
    {
        /* Clear error flags */
        FLASH_IF->SR = FLASH_SR_PGERR | FLASH_SR_WRPRTERR;
        return false;
    }

    return true;
}

/****************************************************************************
 * Name: flash_erase_page
 *
 * Description:
 *   Erase a flash page
 *
 * Input Parameters:
 *   page_addr - Address of the page to erase (must be page-aligned)
 *
 * Returned Value:
 *   true on success, false on failure
 *
 ****************************************************************************/

bool flash_erase_page(uint32_t page_addr)
{
    /* Verify address is in application area */
    if (page_addr < APP_ADDRESS)
    {
        return false;  /* Don't erase bootloader! */
    }

    /* Set page erase mode */
    FLASH_IF->CR |= FLASH_CR_PER;

    /* Set page address */
    FLASH_IF->AR = page_addr;

    /* Start erase */
    FLASH_IF->CR |= FLASH_CR_STRT;

    /* Wait for completion */
    bool result = flash_wait_busy();

    /* Clear page erase mode */
    FLASH_IF->CR &= ~FLASH_CR_PER;

    return result;
}

/****************************************************************************
 * Name: flash_program_halfword
 *
 * Description:
 *   Program a half-word (16 bits) to flash
 *
 * Input Parameters:
 *   addr - Flash address to program (must be half-word aligned)
 *   data - Data to program
 *
 * Returned Value:
 *   true on success, false on failure
 *
 ****************************************************************************/

bool flash_program_halfword(uint32_t addr, uint16_t data)
{
    /* Verify address is in application area */
    if (addr < APP_ADDRESS)
    {
        return false;  /* Don't program bootloader! */
    }

    /* Set programming mode */
    FLASH_IF->CR |= FLASH_CR_PG;

    /* Program half-word */
    *(volatile uint16_t *)addr = data;

    /* Wait for completion */
    bool result = flash_wait_busy();

    /* Clear programming mode */
    FLASH_IF->CR &= ~FLASH_CR_PG;

    /* Verify */
    if (result && (*(volatile uint16_t *)addr != data))
    {
        result = false;
    }

    return result;
}

/****************************************************************************
 * Name: flash_program_buffer
 *
 * Description:
 *   Program a buffer of data to flash
 *
 * Input Parameters:
 *   addr - Flash address to program (must be half-word aligned)
 *   data - Pointer to data buffer
 *   len  - Length in bytes (must be even)
 *
 * Returned Value:
 *   true on success, false on failure
 *
 ****************************************************************************/

bool flash_program_buffer(uint32_t addr, const uint8_t *data, uint32_t len)
{
    uint32_t i;
    uint16_t halfword;

    for (i = 0; i < len; i += 2)
    {
        /* Combine two bytes into a half-word (little endian) */
        halfword = data[i] | (data[i + 1] << 8);

        if (!flash_program_halfword(addr + i, halfword))
        {
            return false;
        }
    }

    return true;
}
