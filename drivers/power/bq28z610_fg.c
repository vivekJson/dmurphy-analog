/*
 * bqgauge battery driver
 *
 * Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Copyright (C) 2010-2011 Lars-Peter Clausen <lars@metafoo.de>
 * Copyright (C) 2011 Pali Rohár <pali.rohar@gmail.com>
 *
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can rebqstribute it and/or mobqfy
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/debugfs.h>
#include <linux/alarmtimer.h>

#define BQ28Z610_UPDATER
#undef  BQ28Z610_UPDATER

#ifdef BQ28Z610_UPDATER
#include "bqfs_cmd_type.h"
#include "bqfs_image.h"
#endif

#define I2C_RETRY_CNT    3
#define bqgauge_I2C_ROM_ADDR    (0x16 >> 1)
#define bqgauge_I2C_DEV_ADDR    (0xAA >> 1)

struct bqgauge_device_info;

struct bqgauge_Device {
	void (*updater)(struct bqgauge_device_info *bq);
	int (*read_fw_ver)(struct bqgauge_device_info *bq);
	int (*read_status)(struct bqgauge_device_info *bq);
	int (*read_fcc)(struct bqgauge_device_info *bq);
	int (*read_designcap)(struct bqgauge_device_info *bq);
	int (*read_rsoc)(struct bqgauge_device_info *bq);
	int (*read_temperature)(struct bqgauge_device_info *bq);
	int (*read_cyclecount)(struct bqgauge_device_info *bq);
	int (*read_timetoempty)(struct bqgauge_device_info *bq);
	int (*read_timetofull)(struct bqgauge_device_info *bq);
	int (*read_health)(struct bqgauge_device_info *bq);
	int (*read_voltage)(struct bqgauge_device_info *bq);
	int (*read_current)(struct bqgauge_device_info *bq, int *);
	int (*read_capacity_level)(struct bqgauge_device_info *bq);
};

enum bqgauge_chip { BQ28Z610 };

struct bqgauge_device_info {
	struct device *dev;
	struct i2c_client *client;

	struct bqgauge_Device *gauge;

	enum bqgauge_chip chip;

	int fw_ver; /* format : AABBCCDD: AABB version, CCDD build number */
	int df_ver;

	bool batt_pres;
	bool batt_fc;

	bool psy_changed;

	int batt_volt;
	int batt_curr;
	int batt_temp;
	int batt_soc;
	int batt_tte;
	int batt_ttf;
	int batt_fcc;
	int batt_cc;		/* cycle count */
	int batt_dc;		/* design capacity */
	int batt_rm;		/* remaining capacity */
	int batt_health;

	int seal_state;		/* 0 - Full Access, 1 - Unsealed, 2 - Sealed */

	int skip_reads;
	int skip_writes;

	int fake_soc;
	int fake_temp;

	struct alarm poll_timer;
	struct delayed_work fg_poll_work;

	struct dentry *debug_root;

	struct power_supply fg_psy;
	struct power_supply *batt_psy;

	struct mutex i2c_rw_lock;
	struct mutex data_lock;
	struct mutex update_lock;

};

/* common routines for bq I2C access */
#if 0
static int __bq_read_i2c_byte(struct i2c_client *client, u8 reg, u8 *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		pr_err("i2c read byte fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*val = (u8) ret;

	return 0;
}

static int __bq_write_i2c_byte(struct i2c_client *client, u8 reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		pr_err
		    ("i2c write byte fail: can't write 0x%02X to reg 0x%02X\n",
		     val, reg);
		return ret;
	}

	return 0;
}
#endif

static int __bq_read_i2c_word(struct i2c_client *client, u8 reg, u16 *val)
{
	s32 ret;

	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0) {
		pr_err("i2c read word fail: can't read from reg 0x%02X\n", reg);
		return ret;
	}

	*val = (u16) ret;

	return 0;
}

static int __bq_write_i2c_word(struct i2c_client *client, u8 reg, u16 val)
{
	s32 ret;

	ret = i2c_smbus_write_word_data(client, reg, val);
	if (ret < 0) {
		pr_err
		    ("i2c write word fail: can't write 0x%02X to reg 0x%02X\n",
		     val, reg);
		return ret;
	}

	return 0;
}

static int __bq_read_i2c_block(struct i2c_client *client, u8 reg, u8 *buf,
			       u8 len)
{
	int ret;
	struct i2c_msg msg[2];
	int i;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = &reg;
	msg[0].len = 1;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = len;

	for (i = 0; i < 3; i++) {
		ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
		if (ret >= 0)
			return ret;
		else
			msleep(5);
	}
	return ret;
}

static int __bq_write_i2c_block(struct i2c_client *client, u8 reg, u8 *buf,
				u8 len)
{
	int ret;
	struct i2c_msg msg;
	u8 data[64];
	int i = 0;

	data[0] = reg;
	memcpy(&data[1], buf, len);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.buf = data;
	msg.len = len + 1;

	for (i = 0; i < 3; i++) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret >= 0)
			return ret;
		else
			msleep(5);
	}
	return ret;
}

#if 0
static int bq_read_i2c_byte(struct bqgauge_device_info *bq, u8 reg, u8 *val)
{
	int ret;

	if (bq->skip_reads) {
		*val = 0;
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq_read_i2c_byte(bq->client, reg, val);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}
#endif

static int bq_read_i2c_word(struct bqgauge_device_info *bq, u8 reg, u16 *val)
{
	int ret;

	if (bq->skip_reads) {
		*val = 0;
		return 0;
	}

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq_read_i2c_word(bq->client, reg, val);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

#if 0
static int bq_write_i2c_byte(struct bqgauge_device_info *bq, u8 reg, u8 val)
{
	int ret;

	if (bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq_write_i2c_byte(bq->client, reg, val);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}
#endif

static int bq_write_i2c_word(struct bqgauge_device_info *bq, u8 reg, u16 val)
{
	int ret;

	if (bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq_write_i2c_word(bq->client, reg, val);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int bq_read_i2c_blk(struct bqgauge_device_info *bq, u8 reg, u8 *buf,
			   u8 len)
{
	int ret;

	if (bq->skip_reads)
		return 0;
	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq_read_i2c_block(bq->client, reg, buf, len);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

static int bq_write_i2c_blk(struct bqgauge_device_info *bq, u8 reg, u8 *data,
			    u8 len)
{
	int ret;

	if (bq->skip_writes)
		return 0;

	mutex_lock(&bq->i2c_rw_lock);
	ret = __bq_write_i2c_block(bq->client, reg, data, len);
	mutex_unlock(&bq->i2c_rw_lock);

	return ret;
}

/* bq28z610 device stuff */
#define BQ28Z610_REG_MAC		 0x00
/* TODO */
#define BQ28Z610_REG_BATTERYSTATUS   0x0A

#define BQ28Z610_REG_FCC             0x12
#define BQ28Z610_REG_RSOC            0x2C
#define BQ28Z610_REG_DESIGNCAP       0x3C
#define BQ28Z610_REG_TEMP            0x06
#define BQ28Z610_REG_CYCLECNT        0x2A
#define BQ28Z610_REG_TTE             0x16
#define BQ28Z610_REG_TTF             0x18
#define BQ28Z610_REG_VOLT            0x08
#define BQ28Z610_REG_CURRENT         0x14
/* TODO: */
#define BQ28Z610_REG_OPSTATUS        0x2C
#define BQ28Z610_REG_CHARGINGVOLTAGE	 0x30
#define BQ28Z610_REG_CHARGINGCURRENT	 0x32

#define BQ28Z610_REG_ALTMAC	         0x3E
#define BQ28Z610_REG_MACDATA		 0x40
#define BQ28Z610_REG_BLKCHKSUM       0x60

#define BQ28Z610_SECURITY_SEALED     0x03
#define BQ28Z610_SECURITY_UNSEALED   0x02
#define BQ28Z610_SECURITY_FA         0x01
#define BQ28Z610_SECURITY_MASK       0x03

#define BQ28Z610_UNSEAL_KEY          0x36720414
#define BQ28Z610_FA_KEY              0xFFFFFFFF

#define BQ28Z610_BATTERYSTATUS_FC             BIT(5)
#define BQ28Z610_BATTERYSTATUS_DSG            BIT(6)
#define BQ28Z610_BATTERYSTATUS_OTA            BIT(12)

#define	BQ28Z610_SUBCMD_OPERATIONSTATUS 0x0054

#define BQ28Z610_SUBCMD_FWVER		0x0002
#define BQ28Z610_SUBCMD_ENTER_ROM	0x0F00
#define BQ28Z610_SUBCMD_SEAL		0x0030

static int bq28z610_read_fw_version(struct bqgauge_device_info *bq)
{
	int ret;
	u16 version;

	ret = bq_write_i2c_word(bq, BQ28Z610_REG_MAC, BQ28Z610_SUBCMD_FWVER);
	if (ret < 0) {
		dev_err(bq->dev, "Failed to send read fw version command\n");
		return ret;
	}
	mdelay(2);
	ret = bq_read_i2c_word(bq, BQ28Z610_REG_MACDATA, &version);
	if (ret < 0) {
		dev_err(bq->dev, "Failed to read read fw version\n");
		return ret;
	}

	return ret;

}

static int bq28z610_read_current(struct bqgauge_device_info *, int *);

static int bq28z610_read_status(struct bqgauge_device_info *bq)
{
	u16 flags;
	int ret;

	ret = bq_read_i2c_word(bq, BQ28Z610_REG_BATTERYSTATUS, &flags);
	if (ret < 0) {
		dev_err(bq->dev, "Failed to read read BatteryStatus\n");
		return ret;
	}
	return flags;
}

static int bq28z610_read_prop_status(struct bqgauge_device_info *bq)
{
	int flags;
	int status;
	int curr;
	int ret;

	ret = bq28z610_read_current(bq, &curr);
	if (ret < 0)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	mdelay(2);

	flags = bq28z610_read_status(bq);
	if (ret < 0)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	if (flags & BQ28Z610_BATTERYSTATUS_FC)
		status = POWER_SUPPLY_STATUS_FULL;
	else if (flags & BQ28Z610_BATTERYSTATUS_DSG)
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	else if (curr > 0)
		status = POWER_SUPPLY_STATUS_CHARGING;
	else
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;

	return status;
}

static int bq28z610_read_fcc(struct bqgauge_device_info *bq)
{
	int ret;
	u16 fcc = 0;

	ret = bq_read_i2c_word(bq, BQ28Z610_REG_FCC, &fcc);
	if (ret < 0) {
		dev_err(bq->dev,
			"Failed to read FullChargeCapacity register:%d\n", ret);
		return ret;
	}
	return fcc;
}

static int bq28z610_read_designcapacity(struct bqgauge_device_info *bq)
{
	int ret;
	u16 dc = 0;

	ret = bq_read_i2c_word(bq, BQ28Z610_REG_DESIGNCAP, &dc);
	if (ret < 0) {
		dev_err(bq->dev,
			"Failed to read FullChargeCapacity register:%d\n", ret);
		return ret;
	}
	return dc;
}

static int bq28z610_read_rsoc(struct bqgauge_device_info *bq)
{
	int ret;
	u16 rsoc = 0;

	ret = bq_read_i2c_word(bq, BQ28Z610_REG_RSOC, &rsoc);
	if (ret < 0) {
		dev_err(bq->dev, "Failed to read RSOC register:%d\n", ret);
		return ret;
	}

	return rsoc;

}

static int bq28z610_read_temperature(struct bqgauge_device_info *bq)
{
	int ret;
	u16 temp = 0;

	ret = bq_read_i2c_word(bq, BQ28Z610_REG_TEMP, &temp);
	if (ret < 0) {
		dev_err(bq->dev, "Failed to read TEMP register:%d\n", ret);
		return ret;
	}

	return temp;

}

static int bq28z610_read_cyclecount(struct bqgauge_device_info *bq)
{
	int ret;
	u16 cc = 0;

	ret = bq_read_i2c_word(bq, BQ28Z610_REG_CYCLECNT, &cc);
	if (ret < 0) {
		dev_err(bq->dev, "Failed to read CycleCount register:%d\n",
			ret);
		return ret;
	}

	return cc;
}

static int bq28z610_read_timetoempty(struct bqgauge_device_info *bq)
{
	int ret;
	u16 tte = 0;

	ret = bq_read_i2c_word(bq, BQ28Z610_REG_TTE, &tte);
	if (ret < 0) {
		dev_err(bq->dev, "Failed to read TimeToEmpty register:%d\n",
			ret);
		return ret;
	}

	return tte;

}

static int bq28z610_read_timetofull(struct bqgauge_device_info *bq)
{
	int ret;
	u16 ttf = 0;

	ret = bq_read_i2c_word(bq, BQ28Z610_REG_TTF, &ttf);
	if (ret < 0) {
		dev_err(bq->dev, "Failed to read TimeToFull register:%d\n",
			ret);
		return ret;
	}

	return ttf;
}

static int bq28z610_read_health(struct bqgauge_device_info *bq)
{
	int status;

	if (bq->batt_temp == -EINVAL)
		return POWER_SUPPLY_HEALTH_UNKNOWN;

	if (bq->batt_temp > 550)
		status = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (bq->batt_temp > 400)
		status = POWER_SUPPLY_HEALTH_WARM;
	else if (bq->batt_temp < 0)
		status = POWER_SUPPLY_HEALTH_COLD;
	else if (bq->batt_temp < 100)
		status = POWER_SUPPLY_HEALTH_COOL;
	else
		status = POWER_SUPPLY_HEALTH_GOOD;

	return status;
}

static int bq28z610_read_voltage(struct bqgauge_device_info *bq)
{

	int ret;
	u16 volt;

	ret = bq_read_i2c_word(bq, BQ28Z610_REG_VOLT, &volt);
	if (ret < 0) {
		dev_err(bq->dev, "Failed to read Voltage register:%d\n", ret);
		return ret;
	}

	return volt;
}

static int bq28z610_read_current(struct bqgauge_device_info *bq, int *curr)
{
	int ret;
	u16 avg_curr = 0;

	ret = bq_read_i2c_word(bq, BQ28Z610_REG_CURRENT, &avg_curr);
	if (ret < 0) {
		dev_err(bq->dev, "Failed to read Curent register:%d\n", ret);
		return ret;
	}
	*curr = (int)((s16) avg_curr);

	return ret;
}

static int bq28z610_read_capacity_level(struct bqgauge_device_info *bq)
{
	int level;
	int rsoc;
	int flags;

	rsoc = bq28z610_read_rsoc(bq);
	if (rsoc < 0)
		return POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;

	flags = bq28z610_read_status(bq);

	if (rsoc > 95)
		level = POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
	else if (flags & 0x02)	/* SYSDOWN */
		level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	else if (flags & 0x04)
		level = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	else
		level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;

	return level;
}

#ifdef BQ28Z610_UPDATER
static void bq28z610_update_bqfs(struct bqgauge_device_info *bq);
#endif

struct bqgauge_Device bqgauge_28z610 = {
#ifdef BQ28z610_UPDATER
	.updater = bq28z610_update_bqfs,
#else
	.updater = NULL,
#endif
	.read_fw_ver = bq28z610_read_fw_version,
	.read_status = bq28z610_read_prop_status,
	.read_fcc = bq28z610_read_fcc,
	.read_designcap = bq28z610_read_designcapacity,
	.read_rsoc = bq28z610_read_rsoc,
	.read_health = bq28z610_read_health,
	.read_voltage = bq28z610_read_voltage,
	.read_current = bq28z610_read_current,
	.read_temperature = bq28z610_read_temperature,
	.read_cyclecount = bq28z610_read_cyclecount,
	.read_timetoempty = bq28z610_read_timetoempty,
	.read_timetofull = bq28z610_read_timetofull,
	.read_capacity_level = bq28z610_read_capacity_level,
};

#ifdef BQ28Z610_UPDATER

static u8 checksum(u8 *data, u8 len)
{
	u16 sum = 0;
	int i;

	for (i = 0; i < len; i++)
		sum += data[i];

	sum &= 0xff;

	return 0xff - sum;
}

/* function works with assumption that deivce is not sealed */
static int bq28z610_read_df(struct bqgauge_device_info *bq, u16 address,
			    u8 *buf, u8 len)
{
	int ret;
	u8 tmp_buf[40];
	int i;

	if (len > 32)
		return -1;	/* less than one block boundary one time */

	ret = bq_write_i2c_word(bq, BQ28Z610_REG_ALTMAC, address);
	if (ret < 0)
		return ret;

	udelay(2000);

	ret = bq_read_i2c_blk(bq, BQ28Z610_REG_ALTMAC, tmp_buf, len + 2);
	if (ret < 0)
		return ret;

	if (address != get_unaligned_le16(tmp_buf))
		return -2;

	for (i = 0; i < len; i++)
		buf[i] = tmp_buf[i + 2];

	return len;
}

static int bq28z610_write_df(struct bqgauge_device_info *bq, u16 address,
			     u8 *buf, u8 len)
{
	int ret;
	u8 tmp_buf[40];
	int i;
	u8 crc_calc = 0;

	if (len > 32)
		return -1; /* less than one block one time */

	put_unaligned_le16(address, &tmp_buf[0]);

	for (i = 0; i < len; i++)
		tmp_buf[i + 2] = buf[i];

	ret = bq_write_i2c_blk(bq, BQ28Z610_REG_ALTMAC, tmp_buf, len + 2);
	if (ret < 0)
		return ret;

	crc_calc = checksum(tmp_buf, len + 2);
	tmp_buf[0] = crc_calc;
	/* including length of address,checksum, len itself */
	tmp_buf[1] = len + 4;

	ret = bq_write_i2c_blk(bq, BQ28Z610_REG_BLKCHKSUM, tmp_buf, 2);

	return ret;
}

/* the following routines are for bqfs/dffs
 * update purpose, can be removed if not used
 */
static int bq28z610_check_seal_state(struct bqgauge_device_info *bq)
{
	int status = BQ28Z610_SECURITY_SEALED;
	int seal;
	bq_write_i2c_word(bq, BQ28Z610_REG_MAC,
			  BQ28Z610_SUBCMD_OPERATIONSTATUS);
	mdelay(2);
	status = bq_read_i2c_word(bq, BQ28Z610_REG_MACDATA);
	if (status < 0)
		return status;

	seal = (u16) status & 0x0300;

	if (seal == 0x0200)	/* FA and SS neither set */
		status = BQ28Z610_SECURITY_FA;
	else if (seal == 0x0100) /* SS not set */
		status = BQ28Z610_SECURITY_UNSEALED;
	else if (seal == 0x0300)
		status = BQ28Z610_SECURITY_SEALED;

	return status;
}

static int bq28z610_unseal(struct bqgauge_device_info *bq)
{
	int ret;

	bq_write_i2c_word(bq, BQ28Z610_REG_MAC, BQ28Z610_UNSEAL_KEY & 0xffff);
	mdelay(2);
	bq_write_i2c_word(bq, BQ28Z610_REG_MAC,
			  (BQ28Z610_UNSEAL_KEY >> 16) & 0xffff);
	mdelay(5);

	ret = bq28z610_check_seal_state(bq);
	if (ret == BQ28Z610_SECURITY_UNSEALED || ret == BQ28Z610_SECURITY_FA)
		return 1;
	else
		return 0;
}

static int bq28z610_unseal_full_access(struct bqgauge_device_info *bq)
{
	int ret;

	bq_write_i2c_word(bq, BQ28Z610_REG_MAC, BQ28Z610_FA_KEY & 0xffff);
	mdelay(2);
	bq_write_i2c_word(bq, BQ28Z610_REG_MAC,
			  (BQ28Z610_FA_KEY >> 16) & 0xffff);
	mdelay(5);

	ret = bq28z610_check_seal_state(bq);
	if (ret == BQ28Z610_SECURITY_FA)
		return 1;
	else
		return 0;

}

static bool bqgauge_check_rom_mode(struct bqgauge_device_info *bq)
{
	struct i2c_client *client = to_i2c_client(bq->dev);
	int ret;

	client->addr = bqgauge_I2C_ROM_ADDR;
	ret = bq_read_i2c_byte(bq, 0x66);
	mdelay(2);
	client->addr = bqgauge_I2C_DEV_ADDR;	/* restore address */
	if (ret < 0)
		return false;

	return true;
}

static bool bq28z610_enter_rom_mode(struct bqgauge_device_info *bq)
{
	int ret;

	ret = bq_write_i2c_word(bq, BQ28Z610_REG_MAC,
			BQ28Z610_SUBCMD_ENTER_ROM);
	mdelay(2);
	if (ret < 0)
		return false;

	return bqgauge_check_rom_mode(bq);
}

#define BQ28Z610_DEVICE_NAME_ADDRESS     0x4080
#define BQ28Z610_DEVICE_NAME_LENGTH      7

static bool bq28z610_check_update_necessary(struct bqgauge_device_info *bq)
{
	/* this is application specific, return true
	 * if need update firmware or data flash
	 */
	u8 buf[40];
	int ret;

	ret =
	    bq28z610_read_df(bq, BQ28Z610_DEVICE_NAME_ADDRESS, buf,
			     BQ28Z610_DEVICE_NAME_LENGTH);
	if (ret != BQ28Z610_DEVICE_NAME_LENGTH)
		return false;
	if (strncmp(buf, "VER0329", BQ28Z610_DEVICE_NAME_LENGTH) == 0)
		return false;
	else
		return true;
}

static bool bq28z610_mark_as_updated(struct bqgauge_device_info *bq)
{
	/* this is application specific */
	int ret;
	ret =
	    bq28z610_write_df(bq, BQ28Z610_DEVICE_NAME_ADDRESS, "VER0329",
			      BQ28Z610_DEVICE_NAME_LENGTH);
	if (ret < 0)
		return false;
	else
		return true;
}

static bool bq28z610_update_execute_cmd(struct bqgauge_device_info *bq,
					const bqfs_cmd_t *cmd)
{
	int ret;
	uint8_t tmp_buf[CMD_MAX_DATA_SIZE];

	switch (cmd->cmd_type) {
	case CMD_R:
		ret =
		    bq_read_i2c_blk(bq, cmd->reg, (u8 *) &cmd->data.bytes,
				    cmd->data_len);
		if (ret < 0)
			return false;

		return true;

	case CMD_W:
		ret =
		    bq_write_i2c_blk(bq, cmd->reg, (u8 *) &cmd->data.bytes,
				     cmd->data_len);
		if (ret < 0)
			return false;

		return true;

	case CMD_C:
		if (bq_read_i2c_blk(bq, cmd->reg, tmp_buf, cmd->data_len) < 0)
			return false;	/* read fail */

		if (memcmp(tmp_buf, cmd->data.bytes, cmd->data_len)) {
			dev_dbg(bq->dev, "\nCommand C failed at line %d:\n",
				cmd->line_num);
			return false;
		}

		return true;

	case CMD_X:
		mdelay(cmd->data.delay);
		return true;

	default:
		dev_err(bq->dev, "Unsupported command at line %d\n",
			cmd->line_num);
		return false;
	}
}

static void bq28z610_update_bqfs(struct bqgauge_device_info *bq)
{
	struct i2c_client *client = to_i2c_client(bq->dev);
	u16 i;

	if (bqgauge_check_rom_mode(bq))
		goto update;	/* already in rom mode */

	/* check if needed update */
	if (!bq28z610_check_update_necessary(bq))
		return;

	if (bq28z610_check_seal_state(bq) != BQ28Z610_SECURITY_FA) {
		if (!bq28z610_unseal(bq))
			return;
		mdelay(10);
		if (!bq28z610_unseal_full_access(bq))
			return;
	}

	if (!bq28z610_enter_rom_mode(bq))
		return;

update:
	client->addr = bqgauge_I2C_ROM_ADDR;
	dev_info(bq->dev, "Updating");
	for (i = 0; i < ARRAY_SIZE(bqfs_image); i++) {
		dev_info(bq->dev, ".");
		if (!bq28z610_update_execute_cmd(bq, &bqfs_image[i])) {
			dev_err(bq->dev, "%s:Failed at command:%d\n", __func__,
				i);
			return;
		}
	}
	dev_info(bq->dev, "Done!\n");

	client->addr = bqgauge_I2C_DEV_ADDR;
	/* mark as updated */
	bq28z610_mark_as_updated(bq);

	return;
}

#endif

static enum power_supply_property bqgauge_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
};

#define to_bqgauge_device_info(x) container_of((x), \
		struct bqgauge_device_info, fg_psy);

static int bqgauge_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct bqgauge_device_info *bq = to_bqgauge_device_info(psy);

	if (bq->gauge == NULL)
		return -ENODEV;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (bq->gauge->read_status)
			val->intval = bq->gauge->read_status(bq);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = bq->batt_volt * 1000;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = bq->batt_volt <= 0 ? 0 : 1;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = -bq->batt_curr * 1000;
		break;

	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = bq->batt_soc;
		break;

	case POWER_SUPPLY_PROP_TEMP:
		val->intval = bq->batt_temp;
		break;

	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		val->intval = bq->batt_tte;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = bq->batt_fcc * 1000;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = bq->batt_dc * 1000;
		break;

	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = bq->batt_cc;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = bq->batt_health;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bqgauge_set_property(struct power_supply *psy,
				enum power_supply_property prop,
				const union power_supply_propval *val)
{
	struct bqgauge_device_info *bq =
	    container_of(psy, struct bqgauge_device_info,
			 fg_psy);
	switch (prop) {
	case POWER_SUPPLY_PROP_TEMP:
		bq->fake_temp = val->intval;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		bq->fake_soc = val->intval;
		power_supply_changed(&bq->fg_psy);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bqgauge_prop_is_writeable(struct power_supply *psy,
				     enum power_supply_property prop)
{
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = 1;
		break;
	default:
		ret = 0;
		break;
	}
	return ret;
}

static void bqgauge_external_power_changed(struct power_supply *psy)
{
	/* struct bqgauge_device_info *bq = to_bqgauge_device_info(psy); */

}

static const u8 fg_dump_regs[] = {
	0x00, 0x02, 0x06, 0x08,
	0x0A, 0x0C, 0x10, 0x12,
	0x1C, 0x1E, 0x2A, 0x2C,
};

static int show_registers(struct seq_file *m, void *data)
{
	struct bqgauge_device_info *bq = m->private;
	int i;
	int ret;
	u16 val = 0;

	for (i = 0; i < ARRAY_SIZE(fg_dump_regs); i++) {
		msleep(5);
		ret = bq_read_i2c_word(bq, fg_dump_regs[i], &val);
		if (!ret)
			seq_printf(m, "Reg[%02X] = 0x%04X\n",
				   fg_dump_regs[i], val);
	}
	return 0;
}

static int reg_debugfs_open(struct inode *inode, struct file *file)
{
	struct bqgauge_device_info *bq = inode->i_private;

	return single_open(file, show_registers, bq);
}

static const struct file_operations reg_debugfs_ops = {
	.owner = THIS_MODULE,
	.open = reg_debugfs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void create_debugfs_entry(struct bqgauge_device_info *bq)
{
	bq->debug_root = debugfs_create_dir("bq_fg", NULL);
	if (!bq->debug_root)
		pr_err("Failed to create debug dir\n");

	if (bq->debug_root) {

		debugfs_create_file("registers", S_IFREG | S_IRUGO,
				    bq->debug_root, bq, &reg_debugfs_ops);

		debugfs_create_x32("fake_soc",
				   S_IFREG | S_IWUSR | S_IRUGO,
				   bq->debug_root, &(bq->fake_soc));

		debugfs_create_x32("fake_temp",
				   S_IFREG | S_IWUSR | S_IRUGO,
				   bq->debug_root, &(bq->fake_temp));

		debugfs_create_x32("skip_reads",
				   S_IFREG | S_IWUSR | S_IRUGO,
				   bq->debug_root, &(bq->skip_reads));
		debugfs_create_x32("skip_writes",
				   S_IFREG | S_IWUSR | S_IRUGO,
				   bq->debug_root, &(bq->skip_writes));
	}
}

static int bqgauge_powersupply_register(struct bqgauge_device_info *bq)
{
	int ret;

	bq->fg_psy.name = "bms";
	bq->fg_psy.type = POWER_SUPPLY_TYPE_BMS;
	bq->fg_psy.properties = bqgauge_battery_props;
	bq->fg_psy.num_properties = ARRAY_SIZE(bqgauge_battery_props);
	bq->fg_psy.get_property = bqgauge_get_property;
	bq->fg_psy.set_property = bqgauge_set_property;
	bq->fg_psy.external_power_changed = bqgauge_external_power_changed;
	bq->fg_psy.property_is_writeable = bqgauge_prop_is_writeable;

	ret = power_supply_register(bq->dev, &bq->fg_psy);
	if (ret < 0) {
		pr_err("Failed to register fg_psy:%d\n", ret);
		return ret;
	}

	return 0;
}

static void bqgauge_powersupply_unregister(struct bqgauge_device_info *bq)
{
	power_supply_unregister(&bq->fg_psy);
}

static ssize_t bqgauge_show_sbsreg(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct bqgauge_device_info *bq = dev_get_drvdata(dev);
	u8 addr;
	u16 val;
	u8 tmpbuf[200];
	int len;
	int idx = 0;
	int ret;

	idx = snprintf(buf, PAGE_SIZE, "%s:\n", "SBS Reg");
	for (addr = 0x0; addr <= 0x3C; addr += 2) {
		ret = bq_read_i2c_word(bq, addr, &val);
		if (ret == 0) {
			len =
			    snprintf(tmpbuf, PAGE_SIZE - idx,
				     "Reg[%.2X] = 0x%04X\n", addr, val);
			memcpy(&buf[idx], tmpbuf, len);
			idx += len;
		}
	}

	return idx;
}

static ssize_t bqgauge_show_macdata(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct bqgauge_device_info *bq = dev_get_drvdata(dev);
	u8 tmpbuf[200];
	int len;
	int idx = 0;
	int ret;
	int i;

	ret = bq_read_i2c_blk(bq, BQ28Z610_REG_ALTMAC, tmpbuf, 36);
	if (ret >= 0) {
		for (i = 0; i < ret; i++) {
			len =
			    snprintf(buf, PAGE_SIZE - idx, " %02X", tmpbuf[i]);
			idx += len;
		}
	}

	return idx;
}

static ssize_t bqgauge_store_macdata(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct bqgauge_device_info *bq = dev_get_drvdata(dev);
#define BUF_MAX_SIZE 40
	u8 data[BUF_MAX_SIZE];
	int data_buf[BUF_MAX_SIZE];
	int offset;
	int n = 0;
	int i;

	while (sscanf(buf, " %d%n", &data_buf[n], &offset) == 1) {
		buf += offset;
		if (n++ >= BUF_MAX_SIZE - 1)
			break;
	}

	for (i = 0; i < n; i++)
		data[i] = (u8) data_buf[i];

	bq_write_i2c_blk(bq, BQ28Z610_REG_ALTMAC, data, n);

	return count;
}

static DEVICE_ATTR(macdata, S_IRUGO | S_IWUSR, bqgauge_show_macdata,
		   bqgauge_store_macdata);
static DEVICE_ATTR(sbsreg, S_IRUGO, bqgauge_show_sbsreg, NULL);

static struct attribute *bqgauge_attributes[] = {
	&dev_attr_macdata.attr,
	&dev_attr_sbsreg.attr,
	NULL
};

static const struct attribute_group bqgauge_attr_group = {
	.attrs = bqgauge_attributes,
};

static void bqgauge_update_data(struct bqgauge_device_info *bq)
{
	int ret;
	int curr;
	int last_batt_soc = bq->batt_soc;
	int last_batt_health = bq->batt_health;

	if (bq->gauge && bq->gauge->read_voltage) {
		ret = bq->gauge->read_voltage(bq);
		if (ret >= 0)
			bq->batt_volt = ret;
	} else {
		bq->batt_volt = -EINVAL;
	}

	if (bq->gauge && bq->gauge->read_current) {
		ret = bq->gauge->read_current(bq, &curr);
		if (!ret)
			bq->batt_curr = curr;
	} else {
		bq->batt_curr = -EINVAL;
	}

	if (bq->gauge && bq->gauge->read_temperature) {
		ret = bq->gauge->read_temperature(bq);
		if (ret >= 0)
			bq->batt_temp = ret - 2730;
	} else {
		bq->batt_temp = -EINVAL;
	}

	if (bq->gauge && bq->gauge->read_rsoc) {
		ret = bq->gauge->read_rsoc(bq);
		if (ret >= 0)
			bq->batt_soc = ret;
	} else {
		bq->batt_soc = -EINVAL;
	}

	if (bq->gauge && bq->gauge->read_fcc) {
		ret = bq->gauge->read_fcc(bq);
		if (ret >= 0)
			bq->batt_fcc = ret;
	} else {
		bq->batt_fcc = 0;
	}

	if (bq->gauge && bq->gauge->read_cyclecount) {
		ret = bq->gauge->read_cyclecount(bq);
		if (ret >= 0)
			bq->batt_cc = ret;
	} else {
		bq->batt_cc = 0;
	}

	if (bq->gauge && bq->gauge->read_designcap) {
		ret = bq->gauge->read_designcap(bq);
		if (ret >= 0)
			bq->batt_dc = ret;
	} else {
		bq->batt_dc = -EINVAL;
	}

	if (bq->gauge && bq->gauge->read_timetoempty) {
		ret = bq->gauge->read_timetoempty(bq);
		if (ret >= 0)
			bq->batt_tte = ret;
	} else {
		bq->batt_tte = -EINVAL;
	}

	if (bq->gauge && bq->gauge->read_timetofull) {
		ret = bq->gauge->read_timetofull(bq);
		if (ret >= 0)
			bq->batt_ttf = ret;
	} else {
		bq->batt_ttf = -EINVAL;
	}

	if (bq->gauge && bq->gauge->read_health)
		bq->batt_health = bq->gauge->read_health(bq);

	if (last_batt_soc != bq->batt_soc ||
	    last_batt_health != bq->batt_health)
		bq->psy_changed = true;

}

static struct power_supply *get_batt_psy(struct bqgauge_device_info *bq)
{
	if (bq->batt_psy)
		return bq->batt_psy;

	bq->batt_psy = power_supply_get_by_name("battery");
	if (!bq->batt_psy)
		pr_debug("battery psy not found!");

	return bq->batt_psy;
}

static int bqgauge_set_batt_prop(struct bqgauge_device_info *bq,
				 enum power_supply_property prop,
				 const union power_supply_propval *val)
{
	struct power_supply *batt_psy = get_batt_psy(bq);
	int ret;

	if (!batt_psy)
		return -1;

	ret = batt_psy->set_property(batt_psy, prop, val);

	return ret;
}

static void bqgauge_set_charge_profile(struct bqgauge_device_info *bq)
{
	int charge_volt = -1;
	int charge_curr = -1;
	u16 val;
	union power_supply_propval prop = { 0, };
	int ret;

	ret = bq_read_i2c_word(bq, BQ28Z610_REG_CHARGINGVOLTAGE, &val);
	if (ret >= 0)
		charge_volt = val;

	ret = bq_read_i2c_word(bq, BQ28Z610_REG_CHARGINGCURRENT, &val);
	if (ret >= 0)
		charge_curr = val;

	if (charge_volt != -1) {
		prop.intval = charge_volt * 1000;
		ret = bqgauge_set_batt_prop(bq,
					  POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
					  &prop);
		pr_debug("Set ChargingVoltage:%d %s\n ", charge_volt,
			 !ret ? "Successfully" : "Failed");
	}

	if (charge_curr != -1) {
		prop.intval = charge_curr * 1000;
		ret = bqgauge_set_batt_prop(bq,
					  POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
					  &prop);
		pr_debug("Set ChargingCurrent:%d %s\n", charge_curr,
			 !ret ? "Successfully" : "Failed");
	}

}

static void bqgauge_poll_workfunc(struct work_struct *work)
{
	struct bqgauge_device_info *bq = container_of(work,
						      struct bqgauge_device_info,
						      fg_poll_work.work);

	bqgauge_update_data(bq);
	bqgauge_set_charge_profile(bq);

	if (bq->psy_changed) {
		power_supply_changed(&bq->fg_psy);
		bq->psy_changed = false;
	}

	pm_relax(bq->dev);
}

static enum alarmtimer_restart poll_timer_cb(struct alarm *alarm, ktime_t now)
{
	struct bqgauge_device_info *bq = container_of(alarm,
						      struct
						      bqgauge_device_info,
						      poll_timer);
	unsigned long ns;

	pm_stay_awake(bq->dev);
	schedule_delayed_work(&bq->fg_poll_work, HZ / 2);

	ns = 10 * 1000000000UL;	/* 10s */
	alarm_forward_now(alarm, ns_to_ktime(ns));

	return ALARMTIMER_RESTART;
}

static int bqgauge_battery_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	int ret;
	struct bqgauge_device_info *bq;

	bq = devm_kzalloc(&client->dev, sizeof(*bq), GFP_KERNEL);
	if (!bq) {
		pr_err("Could not allocate memory\n");
		return -ENOMEM;
	}

	bq->dev = &client->dev;
	bq->client = client;
	bq->chip = id->driver_data;

	bq->fake_soc = -EINVAL;
	bq->fake_temp = -EINVAL;

	if (bq->chip == BQ28Z610)
		bq->gauge = &bqgauge_28z610;
	else {
		dev_err(&client->dev, "Unexpected gas gague: %d\n", bq->chip);
		bq->gauge = NULL;
	}

	i2c_set_clientdata(client, bq);

	mutex_init(&bq->i2c_rw_lock);
	mutex_init(&bq->data_lock);

	if (bq->gauge && bq->gauge->read_fw_ver)
		bq->fw_ver = bq->gauge->read_fw_ver(bq);
	else
		bq->fw_ver = 0x00;

	dev_info(&client->dev, "Gas Guage fw version is 0x%04x\n", bq->fw_ver);

	if (bq->gauge && bq->gauge->updater)
		bq->gauge->updater(bq);

	INIT_DELAYED_WORK(&bq->fg_poll_work, bqgauge_poll_workfunc);

	alarm_init(&bq->poll_timer, ALARM_BOOTTIME, poll_timer_cb);

	ret =
	    alarm_start_relative(&bq->poll_timer,
				 ns_to_ktime(10 * 1000000000LL));
	if (ret)
		pr_err("start poll timer failed, ret=%d\n", ret);

	bqgauge_update_data(bq);

	bqgauge_powersupply_register(bq);

	create_debugfs_entry(bq);
	ret = sysfs_create_group(&client->dev.kobj, &bqgauge_attr_group);
	if (ret)
		dev_err(&client->dev, "could not create sysfs files\n");

	return 0;
}

static int bqgauge_battery_remove(struct i2c_client *client)
{
	struct bqgauge_device_info *bq = i2c_get_clientdata(client);

	bqgauge_powersupply_unregister(bq);

	mutex_destroy(&bq->data_lock);
	mutex_destroy(&bq->i2c_rw_lock);

	debugfs_remove_recursive(bq->debug_root);
	sysfs_remove_group(&bq->dev->kobj, &bqgauge_attr_group);

	return 0;
}

static int bqgauge_battery_suspend(struct i2c_client *client,
				   pm_message_t state)
{
	/* struct bqgauge_device_info *bq = i2c_get_clientdata(client); */

	return 0;
}

static int bqgauge_battery_resume(struct i2c_client *client)
{
	/* struct bqgauge_device_info *bq = i2c_get_clientdata(client); */

	return 0;
}

static struct of_device_id bq_fg_match_table[] = {
	{.compatible = "ti,bq28z610",},
	{},
};

MODULE_DEVICE_TABLE(of, bq_fg_match_table);

static const struct i2c_device_id bqgauge_id[] = {
	{"bq28z610", BQ28Z610},
	{},
};

MODULE_DEVICE_TABLE(i2c, bqgauge_id);

static struct i2c_driver bqgauge_battery_driver = {
	.driver = {
		   .name = "bqgauge-battery",
		   .of_match_table = bq_fg_match_table,
		   .owner = THIS_MODULE,
		   },
	.probe = bqgauge_battery_probe,
	.suspend = bqgauge_battery_suspend,
	.resume = bqgauge_battery_resume,
	.remove = bqgauge_battery_remove,
	.id_table = bqgauge_id,
};

module_i2c_driver(bqgauge_battery_driver);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("bqgauge battery monitor driver");
MODULE_LICENSE("GPL v2");
