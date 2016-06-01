/*
 * Texas Instruments TUSB422 Power Delivery
 *
 * Author: 
 * Copyright: (C) 2016 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include "tusb422_common.h"
#include "tusb422.h"
#include "tcpci.h"
#include "tcpm.h"

struct tusb422_timer_t *lfo_timer[NUM_TCPC_DEVICES]; 

void tusb422_lfo_timer_start(struct tusb422_timer_t *timer, int timeout_ms, void (*function)(unsigned int))
{
    unsigned int port = timer->data;

    // Set timer function pointer.
    timer->function = function;

    // Save timer pointer.
    lfo_timer[port] = timer;
    
    // Start timer.
    tcpc_write16(port, TUSB422_REG_LFO_TIMER, timeout_ms);

    return;
}


void tusb422_lfo_timer_cancel(struct tusb422_timer_t *timer)
{
    unsigned int port = timer->data;

    // Stop timer.
    tcpc_write16(port, TUSB422_REG_LFO_TIMER, 0);

    return;
}

void tusb422_send_fast_role_swap(unsigned int port)
{
    // Send Fast role swap.
    tcpc_write8(port, TUSB422_REG_BMC_TX_CTRL, TUSB422_BMC_TX_FASTROLE_SWAP);

    return;
}

#define OTSD1_EN_BIT  1

void tusb422_init(unsigned int port)
{
//    // BQ - for test.
//    tcpc_write8(port, TUSB422_REG_CC_CTRL, 0x20);
//    msleep(5);
//    tcpc_write8(port, TUSB422_REG_CC_CTRL, 0x40);
//    msleep(5);
    INFO("422 init\n");
    // BQ - trim for initial samples only.
    tcpc_write8(port, 0xFF, 1);
    tcpc_write8(port, 0xE0, 0xC0);  /* write 0xC4 to set I2C address to 0x24 */
    tcpc_write8(port, 0xE1, 0x8);
    tcpc_write8(port, 0xE2, 0x20);
    tcpc_write8(port, 0xE3, 0x80);
    tcpc_write8(port, 0xE4, 0x98);
    tcpc_write8(port, 0xE5, 0x1A);
    tcpc_write8(port, 0xE6, 0x80);
    tcpc_write8(port, 0xE7, 0x70);
    tcpc_write8(port, 0xFF, 0);

    // Enable OTSD1.
    tcpc_write8(port, TUSB422_REG_OTSD_CTRL, OTSD1_EN_BIT);

    // Clear interrupt status.
    tcpc_write8(port, TUSB422_REG_INT_STATUS, TUSB422_INT_MASK_ALL);

    // Unmask interrupts.
    tcpc_write8(port, TUSB422_REG_INT_MASK, TUSB422_INT_MASK_ALL);

    // Unmask alert.
    tcpc_modify8(port, TCPC_REG_ALERT_MASK, 0, 0x00);
    tcpc_modify8(port, TCPC_REG_ALERT_MASK + 1, 0, 0x80);

    return;
}

void tusb422_fast_role_swap_detect_ctrl(unsigned int port, bool enable)
{
    if (enable)
    {
        // Enable Fast role swap detect.
        tcpc_modify8(port, TUSB422_REG_BMC_RX_CTRL, 0, TUSB422_BMC_FASTROLE_RX_EN);
    }
    else
    {
        // Disable Fast role swap detect.
        tcpc_modify8(port, TUSB422_REG_BMC_RX_CTRL, TUSB422_BMC_FASTROLE_RX_EN, 0);
    }

    return;
}


void tusb422_isr(unsigned int port)
{
    uint8_t irq_status;

    tcpc_read8(port, TUSB422_REG_INT_STATUS, &irq_status);

    INFO("422IRQ = 0x%x\n", irq_status);

    if (irq_status & TUSB422_INT_LFO_TIMER)
    {
        DEBUG("422: LFO timer expired!\n");

        if (lfo_timer[port]->function)
        {
            lfo_timer[port]->function(lfo_timer[port]->data);
        }
    }

    if (irq_status & TUSB422_INT_CC_FAULT)
    {
        CRIT("422: CC fault (> 3.5V)!\n");
    }

    if (irq_status & TUSB422_INT_OTSD1_STAT)
    {
        CRIT("422: Over-temp detected!\n");
    }

    if (irq_status & TUSB422_INT_FAST_ROLE_SWAP)
    {
        DEBUG("422: FR Swap Rx'd!\n");
    }
      
    // Clear interrupt status.
    tcpc_write8(port, TUSB422_REG_INT_STATUS, irq_status);

    return;
}
