// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/

#ifndef _CAN_M_CAN_CORE_H_
#define _CAN_M_CAN_CORE_H_

#include <linux/can/core.h>
#include <linux/can/led.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/freezer.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/iopoll.h>
#include <linux/can/dev.h>
#include <linux/pinctrl/consumer.h>

/* m_can lec values */
enum m_can_lec_type {
	LEC_NO_ERROR = 0,
	LEC_STUFF_ERROR,
	LEC_FORM_ERROR,
	LEC_ACK_ERROR,
	LEC_BIT1_ERROR,
	LEC_BIT0_ERROR,
	LEC_CRC_ERROR,
	LEC_UNUSED,
};

enum m_can_mram_cfg {
	MRAM_SIDF = 0,
	MRAM_XIDF,
	MRAM_RXF0,
	MRAM_RXF1,
	MRAM_RXB,
	MRAM_TXE,
	MRAM_TXB,
	MRAM_CFG_NUM,
};

/* registers definition */
enum m_can_reg {
	M_CAN_CREL	= 0x0,
	M_CAN_ENDN	= 0x4,
	M_CAN_CUST	= 0x8,
	M_CAN_DBTP	= 0xc,
	M_CAN_TEST	= 0x10,
	M_CAN_RWD	= 0x14,
	M_CAN_CCCR	= 0x18,
	M_CAN_NBTP	= 0x1c,
	M_CAN_TSCC	= 0x20,
	M_CAN_TSCV	= 0x24,
	M_CAN_TOCC	= 0x28,
	M_CAN_TOCV	= 0x2c,
	M_CAN_ECR	= 0x40,
	M_CAN_PSR	= 0x44,
/* TDCR Register only available for version >=3.1.x */
	M_CAN_TDCR	= 0x48,
	M_CAN_IR	= 0x50,
	M_CAN_IE	= 0x54,
	M_CAN_ILS	= 0x58,
	M_CAN_ILE	= 0x5c,
	M_CAN_GFC	= 0x80,
	M_CAN_SIDFC	= 0x84,
	M_CAN_XIDFC	= 0x88,
	M_CAN_XIDAM	= 0x90,
	M_CAN_HPMS	= 0x94,
	M_CAN_NDAT1	= 0x98,
	M_CAN_NDAT2	= 0x9c,
	M_CAN_RXF0C	= 0xa0,
	M_CAN_RXF0S	= 0xa4,
	M_CAN_RXF0A	= 0xa8,
	M_CAN_RXBC	= 0xac,
	M_CAN_RXF1C	= 0xb0,
	M_CAN_RXF1S	= 0xb4,
	M_CAN_RXF1A	= 0xb8,
	M_CAN_RXESC	= 0xbc,
	M_CAN_TXBC	= 0xc0,
	M_CAN_TXFQS	= 0xc4,
	M_CAN_TXESC	= 0xc8,
	M_CAN_TXBRP	= 0xcc,
	M_CAN_TXBAR	= 0xd0,
	M_CAN_TXBCR	= 0xd4,
	M_CAN_TXBTO	= 0xd8,
	M_CAN_TXBCF	= 0xdc,
	M_CAN_TXBTIE	= 0xe0,
	M_CAN_TXBCIE	= 0xe4,
	M_CAN_TXEFC	= 0xf0,
	M_CAN_TXEFS	= 0xf4,
	M_CAN_TXEFA	= 0xf8,
};

/* address offset and element number for each FIFO/Buffer in the Message RAM */
struct mram_cfg {
	u16 off;
	u8  num;
};

struct m_can_classdev;

typedef	int (*can_dev_init) (struct m_can_classdev *m_can_class);
typedef	int (*can_clr_dev_interrupts) (struct m_can_classdev *m_can_class);
typedef	u32 (*can_reg_read) (struct m_can_classdev *m_can_class, int reg);
typedef	int (*can_reg_write) (struct m_can_classdev *m_can_class, int reg, int val);
typedef	u32 (*can_fifo_read) (struct m_can_classdev *m_can_class, int addr_offset);
typedef	int (*can_fifo_write) (struct m_can_classdev *m_can_class, int addr_offset, int val);

struct m_can_classdev {
	struct can_priv can;
	struct napi_struct napi;
	struct net_device *net;
	struct device *dev;
	struct clk *hclk;
	struct clk *cclk;

	struct workqueue_struct *wq;
	struct work_struct tx_work;
	struct sk_buff *skb;

	struct can_bittiming_const *bit_timing;
	struct can_bittiming_const *data_timing;

	void *device_data;

	/* Device specific call backs */
	can_dev_init device_init;
	can_clr_dev_interrupts clr_dev_interrupts;
	can_reg_read read_reg;
	can_reg_write write_reg;
	can_fifo_read read_fifo;
	can_fifo_write write_fifo;

	int version;
	int freq;
	u32 irqstatus;

	int pm_clock_support;
	bool is_peripherial;

	struct mram_cfg mcfg[MRAM_CFG_NUM];
};

struct m_can_classdev *m_can_core_allocate_dev(struct device *dev);
int m_can_core_register(struct m_can_classdev *m_can_dev);
void m_can_core_unregister(struct m_can_classdev *m_can_dev);
int m_can_core_get_clocks(struct m_can_classdev *m_can_dev);
void m_can_init_ram(struct m_can_classdev *priv);
void m_can_config_endisable(const struct m_can_classdev *priv, bool enable);

int m_can_core_suspend(struct device *dev);
int m_can_core_resume(struct device *dev);
#endif	/* _CAN_M_CAN_CORE_H_ */
