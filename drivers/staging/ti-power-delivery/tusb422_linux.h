/*
 * Constants for the Texas Instruments TUSB422 Power Delivery
 *
 * Author: Dan Murphy <dmurphy@ti.com>
 *
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

#ifndef _TUSB422_LINUX_H_
#define _TUSB422_LINUX_H_

#include <linux/types.h>
#include <linux/delay.h>
#include "tusb422_common.h"

#define TUSB422_I2C_NAME "tusb422"

/* ID registers */
#define TUSB422_VENDOR_ID_0 0x00
#define TUSB422_VENDOR_ID_1 0x01
#define TUSB422_PRODUCT_ID_0 0x02
#define TUSB422_PRODUCT_ID_1 0x03
#define TUSB422_DEV_ID_0 0x04
#define TUSB422_DEV_ID_1 0x05
#define TUSB422_USBTYPEC_REV_0 0x06
#define TUSB422_USBTYPEC_REV_1 0x07
#define TUSB422_USBPD_REV_VER_0 0x08
#define TUSB422_USBPD_REV_VER_1 0x09
#define TUSB422_PD_INTERFACE_REV_0 0x0a
#define TUSB422_PD_INTERFACE_REV_1 0x0b

/* Status Registers */
#define TUSB422_ALERT_0 0x10
#define TUSB422_ALERT_1 0x11
#define TUSB422_ALERT_MASK_0 0x12
#define TUSB422_ALERT_MASK_1 0x13
#define TUSB422_POWER_STATUS_MASK 0x14
#define TUSB422_FAULT_STATUS_MASK 0x15

/* Control Registers */
#define TUSB422_CFG_STAND_OUT 0x18
#define TUSB422_TCPC_CONTROL 0x19
#define TUSB422_ROLE_CONTROL 0x1a
#define TUSB422_FAULT_CONTROL 0x1b
#define TUSB422_PWR_CONTROL 0x1c
#define TUSB422_CC_STATUS 0x1d
#define TUSB422_PWR_STATUS 0x1e
#define TUSB422_FAULT_STATUS 0x1f

#define TUSB422_COMMAND 0x23
#define TUSB422_DEV_CAP_1_BYTE_0 0x24
#define TUSB422_DEV_CAP_1_BYTE_1 0x25
#define TUSB422_DEV_CAP_2_BYTE_0 0x26
#define TUSB422_DEV_CAP_2_BYTE_1 0x27
#define TUSB422_STANDARD_IN_CAP 0x28
#define TUSB422_STANDARD_OUT_CAP 0x29

#define TUSB422_INT_STATUS	0x91
#define TUSB422_INT_STATUS_MSK	0x92

int tusb422_read(int reg, int *value, int num_of_regs);
int tusb422_write(int reg, int value, int num_of_regs);
int tusb422_write_block(int reg, int *data, int num_of_regs);
int tusb422_modify_reg(int reg, int clr_mask, int set_mask);
int tusb422_set_vbus(int vbus_sel);
int tusb422_clr_vbus(int vbus_sel);
void tusb422_msleep(int msecs);

void tusb422_set_timer_func(void (*function)(unsigned int));
void tusb422_clr_timer_func(void);
int tusb422_start_timer(unsigned int timeout_ms);
int tusb422_stop_timer(void);

#endif
