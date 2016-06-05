/*
 * Texas Instruments TUSB422 Power Delivery
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

#include "tusb422_common.h"

#define NUM_TCPC_DEVICES   1

struct tcpc_device_config_t tcpc_dev_config[NUM_TCPC_DEVICES];

int8_t tcpc_read8(unsigned int port, uint8_t reg, uint8_t *data)
{
	return tusb422_read(reg, (int *) data, 1);
};

int8_t tcpc_read16(unsigned int port, uint8_t reg, uint16_t *data)
{
	tusb422_read(reg, (int *) data, 2);

	return 0;
};

int8_t tcpc_read_block(unsigned int port, uint8_t reg, uint8_t *data,
		       unsigned int len)
{
	return tusb422_read(reg, (int *) data, len);
};

int8_t tcpc_write8(unsigned int port, uint8_t reg, uint8_t data)
{
	return tusb422_write(reg, data, 1);
};

int8_t tcpc_write16(unsigned int port, uint8_t reg, uint16_t data)
{
	return tusb422_write(reg, data, 2);
};

int8_t tcpc_write_block(unsigned int port, uint8_t reg, uint8_t *data, uint8_t len)
{
	int i;

	printk("%s: enter %i\n", __func__, len);

	for (i = 0; i <= len; i++)
		tusb422_write(reg + i, *(data + i), 1);

	return 0;
};

// Modifies an 8-bit register.  
void tcpc_modify8(unsigned int port,
                  uint8_t reg,
                  uint8_t clr_mask,
                  uint8_t set_mask)
{

	printk("%s: enter \n", __func__);

	tusb422_modify_reg(reg, clr_mask, set_mask);
	return;
};

// Modifies an 16-bit register.
void tcpc_modify16(unsigned int port,
                  uint8_t reg,
                  uint16_t clr_mask,
                  uint16_t set_mask)
{
	tusb422_modify_reg(reg, clr_mask, set_mask);
	return;
};

int timer_start(struct tusb422_timer_t *timer,
	unsigned int timeout_ms,
	void (*function)(unsigned int))
{
	printk("%s: Enter\n", __func__);
	tusb422_set_timer_func(*function);
	tusb422_start_timer(timeout_ms);

	return 0;
};

void timer_cancel(struct tusb422_timer_t *timer)
{
	printk("%s: Enter\n", __func__);
	tusb422_stop_timer();
	return;
};

void tcpm_hal_vbus_enable(uint8_t port, enum vbus_select_t sel)
{
	printk("%s: Enter\n", __func__);
	tusb422_set_vbus((int) sel);
	return;
};

void tcpm_hal_vbus_disable(uint8_t port, enum vbus_select_t sel)
{
	printk("%s: Enter\n", __func__);
	tusb422_clr_vbus((int) sel);
	return;
};

void tcpc_config(unsigned int port, smbus_interface_t intf, uint8_t slave_addr)
{
    tcpc_dev_config[port].intf = intf;
    tcpc_dev_config[port].slave_addr = slave_addr;

    return;
};
