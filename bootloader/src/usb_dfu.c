/****************************************************************************
 * STM32F103 USB DFU Driver for Bootloader
 *
 * Implements USB Device Firmware Upgrade (DFU) Class
 * Based on DFU 1.1 / DFuSe specification
 *
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32f103.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* USB Registers - direct access */
#define USB_EP(n)           (*(volatile uint16_t *)(USB_BASE + (n) * 4))
#define USB_EP0R            USB_EP(0)
#define USB_CNTR            (*(volatile uint16_t *)(USB_BASE + 0x40))
#define USB_ISTR            (*(volatile uint16_t *)(USB_BASE + 0x44))
#define USB_DADDR           (*(volatile uint16_t *)(USB_BASE + 0x4C))
#define USB_BTABLE          (*(volatile uint16_t *)(USB_BASE + 0x50))

/* USB CNTR bits */
#define USB_CNTR_CTRM       (1 << 15)
#define USB_CNTR_RESETM     (1 << 10)
#define USB_CNTR_FRES       (1 << 0)
#define USB_CNTR_PDWN       (1 << 1)

/* USB ISTR bits */
#define USB_ISTR_CTR        (1 << 15)
#define USB_ISTR_RESET      (1 << 10)
#define USB_ISTR_EP_ID      0x0F

/* EP register bits */
#define EP_CTR_RX           (1 << 15)
#define EP_DTOG_RX          (1 << 14)
#define EP_STAT_RX          (3 << 12)
#define EP_SETUP            (1 << 11)
#define EP_TYPE             (3 << 9)
#define EP_KIND             (1 << 8)
#define EP_CTR_TX           (1 << 7)
#define EP_DTOG_TX          (1 << 6)
#define EP_STAT_TX          (3 << 4)
#define EP_EA               0x0F

/* EP status values */
#define EP_TX_DIS           (0 << 4)
#define EP_TX_STALL         (1 << 4)
#define EP_TX_NAK           (2 << 4)
#define EP_TX_VALID         (3 << 4)

#define EP_RX_DIS           (0 << 12)
#define EP_RX_STALL         (1 << 12)
#define EP_RX_NAK           (2 << 12)
#define EP_RX_VALID         (3 << 12)

/* EP type values */
#define EP_BULK             (0 << 9)
#define EP_CONTROL          (1 << 9)
#define EP_ISO              (2 << 9)
#define EP_INTERRUPT        (3 << 9)

/* DFU Request Codes */
#define DFU_DETACH          0
#define DFU_DNLOAD          1
#define DFU_UPLOAD          2
#define DFU_GETSTATUS       3
#define DFU_CLRSTATUS       4
#define DFU_GETSTATE        5
#define DFU_ABORT           6

/* DFU States */
#define STATE_IDLE          2
#define STATE_DNLOAD_SYNC   3
#define STATE_DNBUSY        4
#define STATE_DNLOAD_IDLE   5
#define STATE_MANIFEST_SYNC 6
#define STATE_ERROR         10

/* DFU Status */
#define STATUS_OK           0x00
#define STATUS_ERR_ERASE    0x04
#define STATUS_ERR_PROG     0x06
#define STATUS_ERR_ADDRESS  0x08

/* Buffer sizes */
#define EP0_SIZE            64
#define XFER_SIZE           256

/* PMA addresses */
#define BTABLE_ADDR         0
#define EP0_TX_ADDR         0x40
#define EP0_RX_ADDR         0x80

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct {
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} __attribute__((packed)) SetupPacket;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t g_state = STATE_IDLE;
static uint8_t g_status = STATUS_OK;
static uint16_t g_block = 0;
static uint8_t g_addr = 0;
static uint8_t g_rx_buf[XFER_SIZE];
static uint16_t g_rx_len = 0;
static uint16_t g_rx_expected = 0;
static uint16_t g_rx_total = 0;

/* DFuSe specific */
static uint32_t g_dfuse_addr = APP_ADDRESS;  /* Current address pointer */
static uint8_t g_dfuse_cmd = 0;              /* Pending command */
static volatile uint8_t g_flash_pending = 0; /* Flash op pending */

/* DFuSe Commands (sent in block 0) */
#define DFUSE_CMD_SET_ADDR   0x21
#define DFUSE_CMD_ERASE      0x41

/* Device Descriptor */
static const uint8_t DevDesc[] = {
    18, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, EP0_SIZE,
    0x83, 0x04,  /* VID: 0x0483 */
    0x11, 0xDF,  /* PID: 0xDF11 */
    0x00, 0x02,  /* bcdDevice */
    1, 2, 3, 1   /* iManufacturer, iProduct, iSerial, bNumConfigurations */
};

/* Configuration Descriptor */
static const uint8_t CfgDesc[] = {
    /* Configuration */
    9, 0x02, 27, 0, 1, 1, 0, 0xC0, 50,
    /* Interface */
    9, 0x04, 0, 0, 0, 0xFE, 0x01, 0x02, 4,
    /* DFU Functional */
    9, 0x21, 0x0B, 0xFF, 0x00, XFER_SIZE & 0xFF, XFER_SIZE >> 8, 0x1A, 0x01
};

/* String 0: Language */
static const uint8_t Str0[] = { 4, 0x03, 0x09, 0x04 };

/* String 1: Manufacturer */
static const uint8_t Str1[] = {
    12, 0x03, 'S',0,'T',0,'M',0,'3',0,'2',0
};

/* String 2: Product - "STM32 DFU" (9 chars = 18 bytes + 2 header = 20) */
static const uint8_t Str2[] = {
    20, 0x03, 'S',0,'T',0,'M',0,'3',0,'2',0,' ',0,'D',0,'F',0,'U',0
};

/* String 3: Serial */
static const uint8_t Str3[] = {
    10, 0x03, '1',0,'2',0,'3',0,'4',0
};

/* String 4: Interface - DFuSe memory layout
 * @Flash/0x08003000/116*1Kg
 */
static const uint8_t Str4[] = {
    52, 0x03,
    '@',0,'F',0,'l',0,'a',0,'s',0,'h',0,
    '/',0,'0',0,'x',0,'0',0,'8',0,'0',0,'0',0,'3',0,'0',0,'0',0,'0',0,
    '/',0,'1',0,'1',0,'6',0,'*',0,'1',0,'K',0,'g',0
};

/****************************************************************************
 * External Functions
 ****************************************************************************/

extern bool flash_unlock(void);
extern void flash_lock(void);
extern bool flash_erase_page(uint32_t addr);
extern bool flash_program_buffer(uint32_t addr, const uint8_t *data, uint32_t len);
extern void gpio_led_toggle(void);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* PMA access - STM32F103 PMA is 16-bit wide with 32-bit spacing */
static void pma_write(uint16_t offset, const uint8_t *buf, uint16_t len)
{
    volatile uint16_t *pma = (volatile uint16_t *)(USB_PMAADDR + offset * 2);
    for (uint16_t i = 0; i < (len + 1) / 2; i++)
    {
        uint16_t w = buf[i * 2];
        if (i * 2 + 1 < len) w |= buf[i * 2 + 1] << 8;
        pma[i * 2] = w;
    }
}

static void pma_read(uint16_t offset, uint8_t *buf, uint16_t len)
{
    volatile uint16_t *pma = (volatile uint16_t *)(USB_PMAADDR + offset * 2);
    for (uint16_t i = 0; i < (len + 1) / 2; i++)
    {
        uint16_t w = pma[i * 2];
        buf[i * 2] = w & 0xFF;
        if (i * 2 + 1 < len) buf[i * 2 + 1] = w >> 8;
    }
}

/* Buffer table access */
static volatile uint16_t *btable_entry(int ep, int field)
{
    return (volatile uint16_t *)(USB_PMAADDR + (BTABLE_ADDR + ep * 8 + field * 2) * 2);
}

#define BT_TX_ADDR  0
#define BT_TX_COUNT 1
#define BT_RX_ADDR  2
#define BT_RX_COUNT 3

/* EP register helpers - carefully handle toggle bits */
static void ep_set_tx_status(uint16_t stat)
{
    uint16_t v = USB_EP0R;
    /* Bits to toggle: stat XOR current */
    uint16_t t = (v & EP_STAT_TX) ^ stat;
    /* Write: invariant bits + CTR_RX/TX (write 1 to keep) + toggle bits */
    USB_EP0R = (v & (EP_TYPE | EP_KIND | EP_EA)) | EP_CTR_RX | EP_CTR_TX | t;
}

static void ep_set_rx_status(uint16_t stat)
{
    uint16_t v = USB_EP0R;
    uint16_t t = (v & EP_STAT_RX) ^ stat;
    USB_EP0R = (v & (EP_TYPE | EP_KIND | EP_EA)) | EP_CTR_RX | EP_CTR_TX | t;
}

static void ep_clear_ctr_rx(void)
{
    uint16_t v = USB_EP0R;
    /* Write 0 to CTR_RX to clear, 1 to CTR_TX to keep */
    USB_EP0R = (v & (EP_TYPE | EP_KIND | EP_EA)) | EP_CTR_TX;
}

static void ep_clear_ctr_tx(void)
{
    uint16_t v = USB_EP0R;
    USB_EP0R = (v & (EP_TYPE | EP_KIND | EP_EA)) | EP_CTR_RX;
}

/* Send data on EP0 */
static void ep0_tx(const uint8_t *data, uint16_t len)
{
    if (len > EP0_SIZE) len = EP0_SIZE;
    if (data && len) pma_write(EP0_TX_ADDR, data, len);
    *btable_entry(0, BT_TX_COUNT) = len;
    ep_set_tx_status(EP_TX_VALID);
}

static void ep0_stall(void)
{
    ep_set_tx_status(EP_TX_STALL);
    ep_set_rx_status(EP_RX_STALL);
}

static void ep0_rx_prepare(void)
{
    ep_set_rx_status(EP_RX_VALID);
}

/* Handle GET_DESCRIPTOR */
static void do_get_desc(SetupPacket *s)
{
    const uint8_t *d = NULL;
    uint16_t len = 0;

    switch (s->wValue >> 8)
    {
        case 1: d = DevDesc; len = sizeof(DevDesc); break;
        case 2: d = CfgDesc; len = sizeof(CfgDesc); break;
        case 3:
            switch (s->wValue & 0xFF)
            {
                case 0: d = Str0; len = sizeof(Str0); break;
                case 1: d = Str1; len = sizeof(Str1); break;
                case 2: d = Str2; len = sizeof(Str2); break;
                case 3: d = Str3; len = sizeof(Str3); break;
                case 4: d = Str4; len = sizeof(Str4); break;
            }
            break;
    }

    if (d)
    {
        if (len > s->wLength) len = s->wLength;
        ep0_tx(d, len);
    }
    else
    {
        ep0_stall();
    }
}

/* Handle DFU requests */
static void do_dfu(SetupPacket *s)
{
    uint8_t buf[6];

    switch (s->bRequest)
    {
        case DFU_GETSTATUS:
            /* DFuSe state machine:
             * DNLOAD_SYNC -> (1st GETSTATUS) -> set pending, return DNBUSY
             * DNBUSY -> flash op in main loop -> (2nd GETSTATUS) -> return DNLOAD_IDLE
             */
            if (g_state == STATE_DNLOAD_SYNC)
            {
                /* First GETSTATUS - mark flash op pending, return DNBUSY */
                g_flash_pending = 1;
                g_state = STATE_DNBUSY;
                buf[0] = g_status;
                buf[1] = 100; buf[2] = 0; buf[3] = 0;  /* pollTimeout = 100ms */
                buf[4] = STATE_DNBUSY;
                buf[5] = 0;
            }
            else if (g_state == STATE_DNBUSY)
            {
                /* Second+ GETSTATUS - check if flash op done */
                if (g_flash_pending)
                {
                    /* Still busy, return DNBUSY again */
                    buf[0] = g_status;
                    buf[1] = 100; buf[2] = 0; buf[3] = 0;
                    buf[4] = STATE_DNBUSY;
                    buf[5] = 0;
                }
                else
                {
                    /* Flash op done, return current state */
                    buf[0] = g_status;
                    buf[1] = 0; buf[2] = 0; buf[3] = 0;
                    buf[4] = g_state;
                    buf[5] = 0;
                }
            }
            else if (g_state == STATE_MANIFEST_SYNC)
            {
                g_state = STATE_IDLE;
                buf[0] = g_status;
                buf[1] = 0; buf[2] = 0; buf[3] = 0;
                buf[4] = g_state;
                buf[5] = 0;
            }
            else
            {
                /* IDLE or other states */
                buf[0] = g_status;
                buf[1] = 0; buf[2] = 0; buf[3] = 0;
                buf[4] = g_state;
                buf[5] = 0;
            }

            ep0_tx(buf, 6);
            break;

        case DFU_GETSTATE:
            buf[0] = g_state;
            ep0_tx(buf, 1);
            break;

        case DFU_CLRSTATUS:
            g_status = STATUS_OK;
            g_state = STATE_IDLE;
            g_dfuse_cmd = 0;
            g_rx_len = 0;
            ep0_tx(NULL, 0);
            break;

        case DFU_ABORT:
            g_state = STATE_IDLE;
            g_dfuse_cmd = 0;
            g_rx_len = 0;
            ep0_tx(NULL, 0);
            break;

        case DFU_DNLOAD:
            if (s->wLength == 0)
            {
                /* Download complete */
                g_state = STATE_MANIFEST_SYNC;
                ep0_tx(NULL, 0);
            }
            else
            {
                g_block = s->wValue;
                g_state = STATE_DNLOAD_SYNC;
                g_rx_expected = s->wLength;
                g_rx_total = 0;
                ep0_rx_prepare();  /* Prepare to receive data */
            }
            break;

        default:
            ep0_stall();
            break;
    }
}

/* Handle SETUP packet */
static void do_setup(void)
{
    SetupPacket s;
    pma_read(EP0_RX_ADDR, (uint8_t *)&s, 8);

    uint8_t type = s.bmRequestType & 0x60;

    if (type == 0x00)  /* Standard */
    {
        switch (s.bRequest)
        {
            case 0x00:  /* GET_STATUS */
                { uint8_t st[2] = {0, 0}; ep0_tx(st, 2); }
                break;
            case 0x05:  /* SET_ADDRESS */
                g_addr = s.wValue & 0x7F;
                ep0_tx(NULL, 0);
                break;
            case 0x06:  /* GET_DESCRIPTOR */
                do_get_desc(&s);
                break;
            case 0x09:  /* SET_CONFIGURATION */
            case 0x0B:  /* SET_INTERFACE */
                ep0_tx(NULL, 0);
                break;
            default:
                ep0_stall();
                break;
        }
    }
    else if (type == 0x20)  /* Class */
    {
        do_dfu(&s);
    }
    else
    {
        ep0_stall();
    }
}

/* Handle OUT data */
static void do_out(void)
{
    uint16_t cnt = *btable_entry(0, BT_RX_COUNT) & 0x3FF;

    if (g_state == STATE_DNLOAD_SYNC && cnt > 0)
    {
        /* Accumulate multi-packet control OUT data */
        if (g_rx_total + cnt > sizeof(g_rx_buf))
        {
            g_status = STATUS_ERR_ADDRESS;
            g_state = STATE_ERROR;
            ep0_stall();
            return;
        }

        pma_read(EP0_RX_ADDR, &g_rx_buf[g_rx_total], cnt);
        g_rx_total += cnt;

        if (g_rx_total < g_rx_expected)
        {
            /* Need more OUT packets */
            ep0_rx_prepare();
            return;
        }

        /* Full payload received */
        g_rx_len = g_rx_total;

        /* DFuSe: Block 0 contains special commands */
        if (g_block == 0)
        {
            uint8_t cmd = g_rx_buf[0];
            if (cmd == DFUSE_CMD_SET_ADDR && g_rx_len == 5)
            {
                /* Set Address Pointer: 0x21 + 4-byte address (little endian) */
                g_dfuse_addr = g_rx_buf[1] |
                               (g_rx_buf[2] << 8) |
                               (g_rx_buf[3] << 16) |
                               (g_rx_buf[4] << 24);
                g_dfuse_cmd = DFUSE_CMD_SET_ADDR;
                g_rx_len = 0;
            }
            else if (cmd == DFUSE_CMD_ERASE && g_rx_len == 5)
            {
                /* Erase Page: 0x41 + 4-byte address */
                g_dfuse_addr = g_rx_buf[1] |
                               (g_rx_buf[2] << 8) |
                               (g_rx_buf[3] << 16) |
                               (g_rx_buf[4] << 24);
                g_dfuse_cmd = DFUSE_CMD_ERASE;
                g_rx_len = 0;
            }
            else
            {
                /* Unknown command */
                g_status = STATUS_OK;
                g_dfuse_cmd = 0;
                g_rx_len = 0;
            }
        }
        else
        {
            /* Block >= 2: Regular data download */
            /* Store data, will be programmed on GETSTATUS */
            g_dfuse_cmd = 0;
        }

        /* Send ZLP to complete data stage */
        ep0_tx(NULL, 0);
    }
    else
    {
        ep0_rx_prepare();
    }
}

/* Handle IN complete */
static void do_in(void)
{
    if (g_addr)
    {
        USB_DADDR = 0x80 | g_addr;
        g_addr = 0;
    }
    ep0_rx_prepare();
}

/* USB Reset */
static void usb_reset(void)
{
    USB_BTABLE = BTABLE_ADDR;

    *btable_entry(0, BT_TX_ADDR) = EP0_TX_ADDR;
    *btable_entry(0, BT_TX_COUNT) = 0;
    *btable_entry(0, BT_RX_ADDR) = EP0_RX_ADDR;
    *btable_entry(0, BT_RX_COUNT) = (1 << 15) | (1 << 10);  /* 64 bytes */

    USB_EP0R = EP_CONTROL;  /* Control endpoint, EA=0 */
    ep_set_rx_status(EP_RX_VALID);
    ep_set_tx_status(EP_TX_NAK);

    USB_DADDR = 0x80;  /* Enable, address 0 */

    g_state = STATE_IDLE;
    g_status = STATUS_OK;
    g_addr = 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void usb_dfu_init(void)
{
    USB_CNTR = USB_CNTR_FRES;
    for (volatile int i = 0; i < 100000; i++);
    USB_CNTR = 0;
    USB_ISTR = 0;
    USB_CNTR = USB_CNTR_RESETM | USB_CNTR_CTRM;
}

void usb_dfu_process(void)
{
    uint16_t istr = USB_ISTR;

    if (istr & USB_ISTR_RESET)
    {
        USB_ISTR = ~USB_ISTR_RESET;
        usb_reset();
        return;
    }

    if (istr & USB_ISTR_CTR)
    {
        uint16_t ep = istr & USB_ISTR_EP_ID;
        if (ep == 0)
        {
            uint16_t epr = USB_EP0R;

            if (epr & EP_CTR_RX)
            {
                ep_clear_ctr_rx();
                if (epr & EP_SETUP)
                    do_setup();
                else
                    do_out();
            }

            if (epr & EP_CTR_TX)
            {
                ep_clear_ctr_tx();
                do_in();
            }
        }
    }
}

void usb_dfu_poll(void)
{
    /* Process USB events first */
    usb_dfu_process();

    /* Then handle pending flash operations (async) */
    if (g_flash_pending && g_state == STATE_DNBUSY)
    {
        if (g_dfuse_cmd == DFUSE_CMD_ERASE)
        {
            flash_unlock();
            if (!flash_erase_page(g_dfuse_addr))
            {
                g_status = STATUS_ERR_ERASE;
                g_state = STATE_ERROR;
            }
            else
            {
                g_state = STATE_DNLOAD_IDLE;
            }
            flash_lock();
            g_dfuse_cmd = 0;
            gpio_led_toggle();
        }
        else if (g_dfuse_cmd == DFUSE_CMD_SET_ADDR)
        {
            g_state = STATE_DNLOAD_IDLE;
            g_dfuse_cmd = 0;
        }
        else if (g_rx_len > 0)
        {
            /* Program data */
            flash_unlock();
            if (!flash_program_buffer(g_dfuse_addr, g_rx_buf, g_rx_len))
            {
                g_status = STATUS_ERR_PROG;
                g_state = STATE_ERROR;
            }
            else
            {
                g_dfuse_addr += g_rx_len;
                g_state = STATE_DNLOAD_IDLE;
            }
            flash_lock();
            g_rx_len = 0;
            gpio_led_toggle();
        }
        else
        {
            g_state = STATE_DNLOAD_IDLE;
        }

        g_flash_pending = 0;
    }
}
