/****************************************************************************
 * STM32F103 Register Definitions (Minimal for Bootloader)
 ****************************************************************************/

#ifndef __STM32F103_H
#define __STM32F103_H

#include <stdint.h>

/****************************************************************************
 * Base Addresses
 ****************************************************************************/

#define FLASH_BASE          0x08000000UL
#define SRAM_BASE           0x20000000UL
#define PERIPH_BASE         0x40000000UL

#define APB1PERIPH_BASE     PERIPH_BASE
#define APB2PERIPH_BASE     (PERIPH_BASE + 0x10000UL)
#define AHBPERIPH_BASE      (PERIPH_BASE + 0x20000UL)

/****************************************************************************
 * Peripheral Base Addresses
 ****************************************************************************/

#define RCC_BASE            (AHBPERIPH_BASE + 0x1000UL)
#define FLASH_R_BASE        (AHBPERIPH_BASE + 0x2000UL)

#define AFIO_BASE           (APB2PERIPH_BASE + 0x0000UL)

#define GPIOA_BASE          (APB2PERIPH_BASE + 0x0800UL)
#define GPIOB_BASE          (APB2PERIPH_BASE + 0x0C00UL)
#define GPIOC_BASE          (APB2PERIPH_BASE + 0x1000UL)

#define USB_BASE            (APB1PERIPH_BASE + 0x5C00UL)
#define USB_PMAADDR         (APB1PERIPH_BASE + 0x6000UL)

/****************************************************************************
 * RCC Registers
 ****************************************************************************/

typedef struct
{
    volatile uint32_t CR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t APB1RSTR;
    volatile uint32_t AHBENR;
    volatile uint32_t APB2ENR;
    volatile uint32_t APB1ENR;
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
} RCC_TypeDef;

#define RCC                 ((RCC_TypeDef *)RCC_BASE)

/* RCC_CR bits */
#define RCC_CR_HSION        (1UL << 0)
#define RCC_CR_HSIRDY       (1UL << 1)
#define RCC_CR_HSEON        (1UL << 16)
#define RCC_CR_HSERDY       (1UL << 17)
#define RCC_CR_PLLON        (1UL << 24)
#define RCC_CR_PLLRDY       (1UL << 25)

/* RCC_CFGR bits */
#define RCC_CFGR_SW_HSI     0x00000000UL
#define RCC_CFGR_SW_HSE     0x00000001UL
#define RCC_CFGR_SW_PLL     0x00000002UL
#define RCC_CFGR_SWS_HSI    0x00000000UL
#define RCC_CFGR_SWS_HSE    0x00000004UL
#define RCC_CFGR_SWS_PLL    0x00000008UL
#define RCC_CFGR_PLLSRC     (1UL << 16)
#define RCC_CFGR_PLLXTPRE   (1UL << 17)
#define RCC_CFGR_USBPRE     (1UL << 22)

/* RCC_APB1ENR bits */
#define RCC_APB1ENR_USBEN   (1UL << 23)

/* RCC_APB2ENR bits */
#define RCC_APB2ENR_IOPAEN  (1UL << 2)
#define RCC_APB2ENR_IOPBEN  (1UL << 3)
#define RCC_APB2ENR_IOPCEN  (1UL << 4)
#define RCC_APB2ENR_AFIOEN  (1UL << 0)

/****************************************************************************
 * AFIO Registers
 ****************************************************************************/

typedef struct
{
    volatile uint32_t EVCR;
    volatile uint32_t MAPR;
    volatile uint32_t EXTICR[4];
    volatile uint32_t MAPR2;
} AFIO_TypeDef;

#define AFIO                ((AFIO_TypeDef *)AFIO_BASE)

/* AFIO_MAPR bits */
#define AFIO_MAPR_SWJ_CFG_MASK        (0x7UL << 24)
#define AFIO_MAPR_SWJ_CFG_JTAGDISABLE (0x2UL << 24) /* JTAG-DP disabled, SW-DP enabled */

/****************************************************************************
 * GPIO Registers
 ****************************************************************************/

typedef struct
{
    volatile uint32_t CRL;
    volatile uint32_t CRH;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t BRR;
    volatile uint32_t LCKR;
} GPIO_TypeDef;

#define GPIOA               ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *)GPIOC_BASE)

/* GPIO mode bits */
#define GPIO_MODE_INPUT     0x00
#define GPIO_MODE_OUTPUT_10 0x01
#define GPIO_MODE_OUTPUT_2  0x02
#define GPIO_MODE_OUTPUT_50 0x03

#define GPIO_CNF_INPUT_ANALOG  0x00
#define GPIO_CNF_INPUT_FLOAT   0x04
#define GPIO_CNF_INPUT_PUPD    0x08

#define GPIO_CNF_OUTPUT_PP     0x00
#define GPIO_CNF_OUTPUT_OD     0x04
#define GPIO_CNF_AF_PP         0x08
#define GPIO_CNF_AF_OD         0x0C

/****************************************************************************
 * Flash Registers
 ****************************************************************************/

typedef struct
{
    volatile uint32_t ACR;
    volatile uint32_t KEYR;
    volatile uint32_t OPTKEYR;
    volatile uint32_t SR;
    volatile uint32_t CR;
    volatile uint32_t AR;
    volatile uint32_t RESERVED;
    volatile uint32_t OBR;
    volatile uint32_t WRPR;
} FLASH_TypeDef;

#define FLASH_IF            ((FLASH_TypeDef *)FLASH_R_BASE)

/* Flash keys */
#define FLASH_KEY1          0x45670123UL
#define FLASH_KEY2          0xCDEF89ABUL

/* Flash SR bits */
#define FLASH_SR_BSY        (1UL << 0)
#define FLASH_SR_PGERR      (1UL << 2)
#define FLASH_SR_WRPRTERR   (1UL << 4)
#define FLASH_SR_EOP        (1UL << 5)

/* Flash CR bits */
#define FLASH_CR_PG         (1UL << 0)
#define FLASH_CR_PER        (1UL << 1)
#define FLASH_CR_MER        (1UL << 2)
#define FLASH_CR_STRT       (1UL << 6)
#define FLASH_CR_LOCK       (1UL << 7)

/****************************************************************************
 * Application Address
 ****************************************************************************/

#define APP_ADDRESS         0x08003000UL

/****************************************************************************
 * DFU Button (PA15)
 ****************************************************************************/

#define DFU_BUTTON_PORT     GPIOA
#define DFU_BUTTON_PIN      15

#endif /* __STM32F103_H */
