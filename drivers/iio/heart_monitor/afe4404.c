/*
 * AFE4404 Heart Rate Monitors and Low-Cost Pulse Oximeters
 *
 * Author: Dan Murphy <dmurphy@ti.com>
 *
 * Copyright: (C) 2015 Texas Instruments, Inc.
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

#include <linux/device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/regulator/consumer.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>

#define AFE4404_CONTROL0		0x00
#define AFE4404_LED2STC			0x01
#define AFE4404_LED2ENDC		0x02
#define AFE4404_LED1LEDSTC		0x03
#define AFE4404_LED1LEDENDC		0x04
#define AFE4404_ALED2STC		0x05
#define AFE4404_ALED2ENDC		0x06
#define AFE4404_LED1STC			0x07
#define AFE4404_LED1ENDC		0x08
#define AFE4404_LED2LEDSTC		0x09
#define AFE4404_LED2LEDENDC		0x0a
#define AFE4404_ALED1STC		0x0b
#define AFE4404_ALED1ENDC		0x0c
#define AFE4404_LED2CONVST		0x0d
#define AFE4404_LED2CONVEND		0x0e
#define AFE4404_ALED2CONVST		0x0f
#define AFE4404_ALED2CONVEND	0x10
#define AFE4404_LED1CONVST		0x11
#define AFE4404_LED1CONVEND		0x12
#define AFE4404_ALED1CONVST		0x13
#define AFE4404_ALED1CONVEND	0x14
#define AFE4404_ADCRSTSTCT0		0x15
#define AFE4404_ADCRSTENDCT0	0x16
#define AFE4404_ADCRSTSTCT1		0x17
#define AFE4404_ADCRSTENDCT1	0x18
#define AFE4404_ADCRSTSTCT2		0x19
#define AFE4404_ADCRSTENDCT2	0x1a
#define AFE4404_ADCRSTSTCT3		0x1b
#define AFE4404_ADCRSTENDCT3	0x1c
#define AFE4404_PRPCOUNT		0x1d
#define AFE4404_CONTROL1		0x1e
#define AFE4404_SPARE1			0x1f
#define AFE4404_TIAGAIN			0x20
#define AFE4404_TIA_AMB_GAIN	0x21
#define AFE4404_LEDCNTRL		0x22
#define AFE4404_CONTROL2		0x23
#define AFE4404_SPARE2			0x24
#define AFE4404_SPARE3			0x25
#define AFE4404_SPARE4			0x26
#define AFE4404_ALARM			0x29
#define AFE4404_LED2VAL			0x2A
#define AFE4404_ALED2VAL		0x2B
#define AFE4404_LED1VAL			0x2C
#define AFE4404_ALED1VAL		0x2D
#define AFE4404_LED2_ALED2VAL	0x2E
#define AFE4404_LED1_ALED1VAL	0x2F
#define AFE4404_DIAG			0x30
#define AFE4404_CONTROL3		0x31
#define AFE4404_PDNCYCLESTC		0x32
#define AFE4404_PDNCYCLEENDC	0x33
#define AFE4404_LED3LEDSTC		0x36
#define AFE4404_LED3LEDENDC		0x37
#define AFE4404_CLKDIV_PRF		0x39
#define AFE4404_AMBDAC			0x3a

#define AFE4404_I2C_READ		BIT(0)
#define AFE4404_I2C_WRITE		0x0
#define AFE4404_SW_RESET		BIT(3)
#define AFE4404_PWR_DWN			BIT(0)

static DEFINE_MUTEX(afe4404_mutex);

/**
 * struct afe4404_data
 * @indio_dev - IIO device structure
 * @i2c - I2C device pointer the driver is attached to
 * @mutex - Read/Write mutex
 * @regmap - Register map of the device
 * @regulator - Pointer to the regulator for the IC
 * @data_gpio - Interrupt GPIO when AFE data is ready
 * @reset_gpio - Hard wire GPIO reset line
 * @timestamp - Timestamp of the IRQ event
 * @state - Current state of the IC.
 * @buff - Data buffer containing the 6 LED values and DIAG
 * @irq - AFE4404 interrupt number
**/
struct afe4404_data {
	struct iio_dev *indio_dev;
	struct i2c_client *client;
	struct mutex mutex;
	struct regmap *regmap;
	struct regulator *regulator;
	struct gpio_desc *data_gpio;
	struct gpio_desc *reset_gpio;
	int64_t timestamp;
	bool state;
	int buff[7];
	int irq;
};

enum afe4404_reg_id {
	LED1VAL,
	ALED1VAL,
	LED2VAL,
	ALED2VAL,
	LED2_ALED2VAL,
	LED1_ALED1VAL,
	DIAG,
	TIAGAIN,
	TIA_AMB_GAIN,
	LEDCNTRL,
	CONTROL3,
	AMBDAC,
};

static const struct iio_event_spec afe4404_event = {
	.type = IIO_EV_TYPE_MAG,
	.dir = IIO_EV_DIR_NONE,
	.mask_separate = BIT(IIO_EV_INFO_VALUE) |
			BIT(IIO_EV_INFO_ENABLE),
};

#define AFE4404_WRITE_FREQ_CHAN(index, name) { \
	.type = IIO_HEARTRATE,		\
	.indexed = 1,				\
	.channel = index,			\
	.datasheet_name = name, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_FREQUENCY), \
	.scan_index = index,		\
	.scan_type = {				\
		.sign = 'u',			\
		.realbits = 24,			\
		.storagebits = 32,		\
		.endianness = IIO_BE		\
	}, \
}

#define AFE4404_WRITE_BIAS_CHAN(index, name) { \
	.type = IIO_HEARTRATE,		\
	.indexed = 1,				\
	.channel = index,			\
	.datasheet_name = name, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),	\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_HARDWAREGAIN), \
	.scan_index = index,		\
	.scan_type = {				\
		.sign = 'u',			\
		.realbits = 24,			\
		.storagebits = 32,		\
		.endianness = IIO_BE		\
	}, \
}

#define AFE4404_READ_CHAN(index, name) { \
	.type = IIO_HEARTRATE,		\
	.indexed = 1,				\
	.channel = index,			\
	.datasheet_name = name, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED) | \
			      BIT(IIO_CHAN_INFO_RAW),	\
	.scan_index = index,		\
	.scan_type = {				\
		.sign = 'u',			\
		.realbits = 24,			\
		.storagebits = 32,		\
		.endianness = IIO_BE		\
	}, \
	.event_spec = &afe4404_event,		\
	.num_event_specs = 1,			\
}

static const struct iio_chan_spec afe4404_channels[] = {
	/* Read only values from the IC */
	AFE4404_READ_CHAN(LED1VAL, "LED1VAL"),
	AFE4404_READ_CHAN(ALED1VAL, "ALED1VAL"),
	AFE4404_READ_CHAN(LED2VAL, "LED2VAL"),
	AFE4404_READ_CHAN(ALED2VAL, "ALED2VAL"),
	AFE4404_READ_CHAN(LED2_ALED2VAL, "LED2_ALED2"),
	AFE4404_READ_CHAN(LED1_ALED1VAL, "LED1_ALED1"),
	AFE4404_READ_CHAN(DIAG, "DIAG"),

	/* Required writing for calibration */
	AFE4404_WRITE_BIAS_CHAN(TIAGAIN, "TIAGAIN"),
	AFE4404_WRITE_BIAS_CHAN(TIA_AMB_GAIN, "TIA_AMB_GAIN"),
	AFE4404_WRITE_BIAS_CHAN(LEDCNTRL, "LEDCNTRL"),
	AFE4404_WRITE_BIAS_CHAN(AMBDAC, "AMBDAC"),
	AFE4404_WRITE_FREQ_CHAN(CONTROL3, "CONTROL3"),
};

static int afe4404_read(struct afe4404_data *afe4404, u8 reg,
		int *data)
{
	int ret;

	mutex_lock(&afe4404_mutex);

	ret = regmap_write(afe4404->regmap, AFE4404_CONTROL0, AFE4404_I2C_READ);
	if (ret) {
		dev_err(&afe4404->client->dev,
			"failed to write afe4404 0x%X register\n",
			AFE4404_CONTROL0);
		goto out;
	}

	ret = regmap_read(afe4404->regmap, reg, data);
	if (ret) {
		dev_err(&afe4404->client->dev,
			"failed to read afe4404 0x%X register\n",
			reg);
			goto out;
	}

	ret = regmap_write(afe4404->regmap, AFE4404_CONTROL0,
			AFE4404_I2C_WRITE);
	if (ret)
		dev_err(&afe4404->client->dev,
			"failed to read afe4404 0x%X register\n",
			AFE4404_CONTROL0);
out:
	mutex_unlock(&afe4404_mutex);
	return ret;
};

static int afe4404_write(struct afe4404_data *afe4404, u8 reg,
		int data)
{
	int ret;

	mutex_lock(&afe4404_mutex);

	/* Enable write to the device */
	ret = regmap_write(afe4404->regmap, AFE4404_CONTROL0,
			AFE4404_I2C_WRITE);
	if (ret)
		dev_err(&afe4404->client->dev,
			"failed to read afe4404 0x%X register\n",
			AFE4404_CONTROL0);

	ret = regmap_write(afe4404->regmap, reg, data);
	if (ret)
		goto out;

	ret = regmap_write(afe4404->regmap, AFE4404_CONTROL0, AFE4404_I2C_READ);
	if (ret)
		goto out;

out:
	mutex_unlock(&afe4404_mutex);
	return ret;
};

/* Remove this line to disable debug */
#define AFE4404_DEBUG

#ifdef AFE4404_DEBUG
/* The registers can be accessed via
 * cat /sys/class/i2c
 * And written through echo for example
 * echo "CONTROL0 0x00" > /sys/class/
 */
struct afe4404_dbg_reg {
	const char *name;
	unsigned char reg;
	int writeable;
	int readable;
} afe4404_dbg_regs[] = {
	{ "CONTROL0", AFE4404_CONTROL0, 1, 0},
	{ "LED2STC", AFE4404_LED2STC, 1, 1 },
	{ "LED2ENDC", AFE4404_LED2ENDC, 1, 1 },
	{ "LED2LEDSTC", AFE4404_LED2LEDSTC, 1, 1 },
	{ "LED2LEDENDC", AFE4404_LED2LEDENDC, 1, 1 },
	{ "ALED2STC", AFE4404_ALED2STC, 1, 1 },
	{ "ALED2ENDC", AFE4404_ALED2ENDC, 1, 1 },
	{ "LED1STC", AFE4404_LED1STC, 1, 1 },
	{ "LED1ENDC", AFE4404_LED1ENDC, 1, 1 },
	{ "LED1LEDSTC", AFE4404_LED1LEDSTC, 1, 1 },
	{ "LED1LEDENDC", AFE4404_LED1LEDENDC, 1, 1 },
	{ "ALED1STC", AFE4404_ALED1STC, 1, 1 },
	{ "ALED1ENDC", AFE4404_ALED1ENDC, 1, 1 },
	{ "LED2CONVST", AFE4404_LED2CONVST, 1, 1 },
	{ "LED2CONVEND", AFE4404_LED2CONVEND, 1, 1 },
	{ "ALED2CONVST", AFE4404_ALED2CONVST, 1, 1 },
	{ "ALED2CONVEND", AFE4404_ALED2CONVEND, 1, 1 },
	{ "LED1CONVST", AFE4404_LED1CONVST, 1, 1 },
	{ "LED1CONVEND", AFE4404_LED1CONVEND, 1, 1 },
	{ "ALED1CONVST", AFE4404_ALED1CONVST, 1, 1 },
	{ "ALED1CONVEND", AFE4404_ALED1CONVEND, 1, 1 },
	{ "ADCRSTSTCT0", AFE4404_ADCRSTSTCT0, 1, 1 },
	{ "ADCRSTENDCT0", AFE4404_ADCRSTENDCT0, 1, 1 },
	{ "ADCRSTSTCT1", AFE4404_ADCRSTSTCT1, 1, 1 },
	{ "ADCRSTENDCT1", AFE4404_ADCRSTENDCT1, 1, 1 },
	{ "ADCRSTSTCT2", AFE4404_ADCRSTSTCT2, 1, 1 },
	{ "ADCRSTENDCT2", AFE4404_ADCRSTENDCT2, 1, 1 },
	{ "ADCRSTSTCT3", AFE4404_ADCRSTSTCT3, 1, 1 },
	{ "ADCRSTENDCT3", AFE4404_ADCRSTENDCT3, 1, 1 },
	{ "PRPCOUNT", AFE4404_PRPCOUNT, 1, 1 },
	{ "CONTROL1", AFE4404_CONTROL1, 1, 1 },
	{ "SPARE1", AFE4404_SPARE1, 1, 1 },
	{ "SPARE2", AFE4404_SPARE2, 1, 1 },
	{ "SPARE3", AFE4404_SPARE3, 1, 1},
	{ "SPARE4", AFE4404_SPARE4, 1, 1 },
	{ "TIAGAIN", AFE4404_TIAGAIN, 1, 1 },
	{ "TIA_AMB_GAIN", AFE4404_TIA_AMB_GAIN, 1, 1 },
	{ "LEDCNTRL", AFE4404_LEDCNTRL, 1, 1 },
	{ "CONTROL2", AFE4404_CONTROL2, 1, 1 },
	{ "CONTROL3", AFE4404_CONTROL3, 1, 1 },
	{ "PDNCYCLESTC", AFE4404_PDNCYCLESTC, 1, 1 },
	{ "PDNCYCLEENDC", AFE4404_PDNCYCLEENDC, 1, 1 },
	{ "ALARM", AFE4404_ALARM, 0, 1 },
	{ "LED2VAL", AFE4404_LED2VAL, 0, 1 },
	{ "ALED2VAL", AFE4404_ALED2VAL, 0, 1 },
	{ "LED1VAL", AFE4404_LED1VAL, 0, 1 },
	{ "ALEDV1AL", AFE4404_ALED1VAL, 0, 1 },
	{ "LED2_ALED2VAL", AFE4404_LED2_ALED2VAL, 0, 1 },
	{ "LED1_ALED1VAL", AFE4404_LED1_ALED1VAL, 0, 1 },
	{ "DIAG", AFE4404_DIAG, 0, 1 },
	{ "LED3LEDSTC", AFE4404_LED3LEDSTC, 1, 1 },
	{ "LED3LEDENDC", AFE4404_LED3LEDENDC, 1, 1 },
	{ "CLKDIV", AFE4404_CLKDIV_PRF, 1, 1 },
};

static ssize_t afe4404_registers_show(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct afe4404_data *data = iio_priv(indio_dev);
	unsigned i = 0, n, reg_count;
	unsigned int read_buf, ret;

	reg_count = sizeof(afe4404_dbg_regs) / sizeof(afe4404_dbg_regs[0]);
	for (i = 0, n = 0; i < reg_count; i++) {
		if (afe4404_dbg_regs[i].readable) {
			ret = afe4404_read(data, afe4404_dbg_regs[i].reg, &read_buf);
			if (ret)
				return ret;
			n += scnprintf(buf + n, PAGE_SIZE - n,
					   "%-20s = 0x%X\n",
					   afe4404_dbg_regs[i].name,
					   read_buf);
		} else {
				printk("%s:Register %s is write only\n",
						__func__, afe4404_dbg_regs[i].name);
		}
	}

	return n;
}

static ssize_t afe4404_registers_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct afe4404_data *data = iio_priv(indio_dev);
	unsigned i, reg_count, value;
	int error = 0;
	char name[30];

	if (count >= 30) {
		pr_err("%s:input too long\n", __func__);
		return -1;
	}

	if (sscanf(buf, "%s %x", name, &value) != 2) {
		pr_err("%s:unable to parse input\n", __func__);
		return -1;
	}

	reg_count = sizeof(afe4404_dbg_regs) / sizeof(afe4404_dbg_regs[0]);
	for (i = 0; i < reg_count; i++) {
		if (!strcmp(name, afe4404_dbg_regs[i].name)) {
			if (afe4404_dbg_regs[i].writeable) {
				error = afe4404_write(data, afe4404_dbg_regs[i].reg, value);
				if (error) {
					printk("%s:Failed to write %s\n",
						__func__, name);
					return -1;
				}
			} else {
				printk("%s:Register %s is read only\n",
						__func__, afe4404_dbg_regs[i].name);
					return -1;
			}

			return count;
		}
	}
	printk("%s:no such register %s\n", __func__, name);
	return -1;
}

static DEVICE_ATTR(registers, S_IWUSR | S_IRUGO,
		afe4404_registers_show, afe4404_registers_store);

static struct attribute *afe4404_attrs[] = {
	&dev_attr_registers.attr,
	NULL
};

static const struct attribute_group afe4404_attr_group = {
	.attrs = afe4404_attrs,
};
#endif

static int afe4404_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct afe4404_data *data = iio_priv(indio_dev);
	u8 reg;

	if (chan->channel >= ARRAY_SIZE(afe4404_channels))
		return -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (chan->channel == TIAGAIN)
			reg = AFE4404_TIAGAIN;
		else if (chan->channel == TIA_AMB_GAIN)
			reg = AFE4404_TIA_AMB_GAIN;
		else if (chan->channel == LEDCNTRL)
			reg = AFE4404_LEDCNTRL;
		else if (chan->channel == CONTROL3)
			reg = AFE4404_CONTROL3;
		else if (chan->channel == AMBDAC)
			reg = AFE4404_AMBDAC;
		else
			return -EINVAL;
		break;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (chan->channel == TIAGAIN)
			reg = AFE4404_TIAGAIN;
		else if (chan->channel == TIA_AMB_GAIN)
			reg = AFE4404_TIA_AMB_GAIN;
		else if (chan->channel == LEDCNTRL)
			reg = AFE4404_LEDCNTRL;
		else if (chan->channel == AMBDAC)
			reg = AFE4404_AMBDAC;
		else
			return -EINVAL;

		break;
	case IIO_CHAN_INFO_FREQUENCY:
		if (chan->channel == CONTROL3)
			reg = AFE4404_CONTROL3;
		else
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	return afe4404_write(data, reg, val);
}

static int afe4404_read_event_value(struct iio_dev *iio,
			const struct iio_chan_spec *chan,
			enum iio_event_type type,
			enum iio_event_direction dir,
			enum iio_event_info info,
			int *val, int *val2)
{
	struct afe4404_data *afe4404 = iio_priv(iio);
	int ret;
	int reg;

	if (chan->channel == LED1VAL)
		reg = AFE4404_LED1VAL;
	else if (chan->channel == ALED1VAL)
		reg = AFE4404_ALED1VAL;
	else if (chan->channel == LED2VAL)
		reg = AFE4404_LED2VAL;
	else if (chan->channel == ALED2VAL)
		reg = AFE4404_ALED2VAL;
	else if (chan->channel == LED2_ALED2VAL)
		reg = AFE4404_LED2_ALED2VAL;
	else if (chan->channel == LED1_ALED1VAL)
		reg = AFE4404_LED1_ALED1VAL;
	else if (chan->channel == DIAG)
		reg = AFE4404_DIAG;
	else
		return -EINVAL;

	ret = afe4404_read(afe4404, reg, &afe4404->buff[chan->channel]);
	if (ret)
		goto done;

	*val = afe4404->buff[chan->channel];
	*val2 = 0;
	ret = IIO_VAL_INT;
done:
	return ret;
}

static int afe4404_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct afe4404_data *data = iio_priv(indio_dev);

	if (chan->channel > ARRAY_SIZE(afe4404_channels))
		return -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
	case IIO_CHAN_INFO_PROCESSED:
		*val = data->buff[chan->channel];
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int afe4404_write_event(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir,
				     int state)
{
	struct afe4404_data *afe4404 = iio_priv(indio_dev);
	int ret;
	int control_val;

	ret = afe4404_read(afe4404, AFE4404_CONTROL2, &control_val);
	if (ret)
		return ret;

	if (state)
		control_val &= ~AFE4404_PWR_DWN;
	else
		control_val |= AFE4404_PWR_DWN;

	ret = afe4404_write(afe4404, AFE4404_CONTROL2, control_val);
	if (ret)
		return ret;

	ret = afe4404_read(afe4404, AFE4404_CONTROL2, &control_val);
	if (ret)
		return ret;

	afe4404->state = state;

	return 0;
}

static int afe4404_debugfs_reg_access(struct iio_dev *indio_dev,
				      unsigned reg, unsigned writeval,
				      unsigned *readval)
{
	struct afe4404_data *afe4404 = iio_priv(indio_dev);
	int ret;

	if (reg < AFE4404_CONTROL0 || reg > AFE4404_AMBDAC)
		return -EINVAL;

	if (readval != NULL) {
		ret = afe4404_read(afe4404, reg, readval);
		if (ret)
			return ret;
	} else if (writeval < 0) {
		ret = afe4404_write(afe4404, reg, writeval);
		if (ret)
			return ret;
	} else {
		ret = -EINVAL;
	}

	return ret;
}

static const struct iio_info afe4404_iio_info = {
	.read_raw = &afe4404_read_raw,
	.write_raw = &afe4404_write_raw,
	.read_event_value = &afe4404_read_event_value,
	.write_event_config = &afe4404_write_event,
	.debugfs_reg_access = &afe4404_debugfs_reg_access,
	.driver_module = THIS_MODULE,
};

static irqreturn_t afe4404_event_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct afe4404_data *afe4404 = iio_priv(indio_dev);

	afe4404->timestamp = iio_get_time_ns();

	iio_push_event(indio_dev, IIO_UNMOD_EVENT_CODE(IIO_HEARTRATE,
							0,
							IIO_EV_TYPE_CHANGE,
							IIO_EV_DIR_EITHER),
							afe4404->timestamp);
	return IRQ_HANDLED;
}

static struct reg_default afe4404_init_regs[] = {
	{ AFE4404_LED2STC, 0x50 },
	{ AFE4404_LED2ENDC, 0x3e7 },
	{ AFE4404_LED1LEDSTC, 0x7d0 },
	{ AFE4404_LED1LEDENDC, 0xbb7 },
	{ AFE4404_ALED2STC, 0x438 },
	{ AFE4404_ALED2ENDC, 0x7cf },
	{ AFE4404_LED1STC, 0x820 },
	{ AFE4404_LED1ENDC, 0xbb7 },
	{ AFE4404_LED2LEDSTC, 0x0 },
	{ AFE4404_LED2LEDENDC, 0x3e7 },
	{ AFE4404_ALED1STC, 0xc08 },
	{ AFE4404_ALED1ENDC, 0xf9f },
	{ AFE4404_LED2CONVST, 0x3ef },
	{ AFE4404_LED2CONVEND, 0x7cf },
	{ AFE4404_ALED2CONVST, 0x7d7 },
	{ AFE4404_ALED2CONVEND, 0xbb7 },
	{ AFE4404_LED1CONVST, 0xbbf },
	{ AFE4404_LED1CONVEND, 0x9c3f },
	{ AFE4404_ALED1CONVST, 0xfa7 },
	{ AFE4404_ALED1CONVEND, 0x1387 },
	{ AFE4404_ADCRSTSTCT0, 0x3e8 },
	{ AFE4404_ADCRSTENDCT0, 0x3eb },
	{ AFE4404_ADCRSTSTCT1, 0x7d0 },
	{ AFE4404_ADCRSTENDCT1, 0x7d3 },
	{ AFE4404_ADCRSTSTCT2, 0xbb8 },
	{ AFE4404_ADCRSTENDCT2, 0xbbb },
	{ AFE4404_ADCRSTSTCT3, 0xfa0 },
	{ AFE4404_ADCRSTENDCT3, 0xfa3 },
	{ AFE4404_LED3LEDSTC, 0x3e8 },
	{ AFE4404_LED3LEDENDC, 0x7cf },
	{ AFE4404_PRPCOUNT, 0x9c3f },
	{ AFE4404_CONTROL1, 0x000102 },
	{ AFE4404_TIAGAIN, 0x008003 },
	{ AFE4404_TIA_AMB_GAIN, 0x000003 },
	{ AFE4404_LEDCNTRL, 0x0030CF },
	{ AFE4404_CONTROL2, 0x124219 },
	{ AFE4404_CLKDIV_PRF, 0 },
	{ AFE4404_PDNCYCLESTC, 0x1518 },
	{ AFE4404_PDNCYCLEENDC, 0x991f },
};

static int afe4404_init(struct afe4404_data *afe4404)
{
	int ret;

	/* Hard reset the device needs to be held for 1ms per data sheet */
	if (afe4404->reset_gpio) {
		gpiod_set_value(afe4404->reset_gpio, 0);
		udelay(1000);
		gpiod_set_value(afe4404->reset_gpio, 1);
	} else {
		/* Soft reset the device */
		ret = afe4404_write(afe4404, AFE4404_CONTROL0,
				AFE4404_SW_RESET);
		if (ret)
			return ret;
	}

	ret = regmap_write(afe4404->regmap, AFE4404_CONTROL0,
			AFE4404_I2C_WRITE);
	if (ret)
		dev_err(&afe4404->client->dev,
			"failed to read afe4404 0x%X register\n",
			AFE4404_CONTROL0);

	ret = regmap_register_patch(afe4404->regmap, afe4404_init_regs,
				ARRAY_SIZE(afe4404_init_regs));
	if (ret) {
		dev_err(&afe4404->client->dev,
			"Failed to init device\n");
		return ret;
	}

	ret = regmap_write(afe4404->regmap, AFE4404_CONTROL0, AFE4404_I2C_READ);
	if (ret)
		dev_err(&afe4404->client->dev,
			"failed to read afe4404 0x%X register\n",
			AFE4404_CONTROL0);
	return ret;
};

static const struct regmap_config afe4404_regmap_config = {
	.reg_bits = 8,
	.val_bits = 24,

	.max_register = AFE4404_AMBDAC,
	.reg_defaults = afe4404_init_regs,
	.num_reg_defaults = ARRAY_SIZE(afe4404_init_regs),
	.cache_type = REGCACHE_NONE,
};

static int afe4404_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct afe4404_data *afe4404;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*afe4404));
	if (indio_dev == NULL) {
		dev_err(&client->dev, "Failed to allocate iio device\n");
		return -ENOMEM;
	}

	afe4404 = iio_priv(indio_dev);
	afe4404->client = client;

	indio_dev->dev.parent = &client->dev;
	indio_dev->name = "afe4404";
	indio_dev->info = &afe4404_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = afe4404_channels;
	indio_dev->num_channels = ARRAY_SIZE(afe4404_channels);

	i2c_set_clientdata(client, indio_dev);

	afe4404->data_gpio = devm_gpiod_get(&client->dev, "data-ready");
	if (IS_ERR(afe4404->data_gpio)) {
		ret = PTR_ERR(afe4404->data_gpio);
		if (ret != -ENOENT && ret != -ENOSYS) {
			dev_err(&client->dev,
				"Failed to allocate data_ready gpio\n");
			return ret;
		}
		afe4404->data_gpio = NULL;
	} else {
		gpiod_direction_input(afe4404->data_gpio);
		afe4404->irq = gpiod_to_irq(afe4404->data_gpio);
	}

	afe4404->reset_gpio = devm_gpiod_get(&client->dev, "reset");
	if (IS_ERR(afe4404->reset_gpio)) {
		ret = PTR_ERR(afe4404->reset_gpio);
		if (ret != -ENOENT && ret != -ENOSYS) {
			dev_err(&client->dev, "Failed to allocate reset gpio\n");
			return ret;
		}
		afe4404->reset_gpio = NULL;
	} else {
		gpiod_direction_output(afe4404->reset_gpio, 1);
	}

	afe4404->regulator = devm_regulator_get(&client->dev, "led");
	if (IS_ERR(afe4404->regulator)) {
		ret = PTR_ERR(afe4404->regulator);
		dev_err(&client->dev,
			"unable to get regulator, error: %d\n", ret);
		return ret;
	}

	afe4404->regmap = devm_regmap_init_i2c(client, &afe4404_regmap_config);
	if (IS_ERR(afe4404->regmap)) {
		ret = PTR_ERR(afe4404->regmap);
		dev_err(&client->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	if (afe4404->irq > 0) {
		ret = devm_request_threaded_irq(&client->dev, afe4404->irq,
			NULL,
			afe4404_event_handler,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"afe4404_event", indio_dev);
		if (ret) {
			dev_err(&client->dev, "unable to request IRQ\n");
			goto err_out;
		}
	}

	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(&client->dev, "unable to register iio device\n");
		goto err_out;
	}

	ret = afe4404_init(afe4404);
	if (ret < 0) {
		dev_err(&client->dev, "unable to communicate with i2c device\n");
		goto err_init_failed;
	}

#ifdef AFE4404_DEBUG
	ret = sysfs_create_group(&client->dev.kobj, &afe4404_attr_group);
	if (ret < 0)
		dev_err(&client->dev, "Failed to create sysfs: %d\n", ret);
#endif

	return 0;

err_init_failed:
	iio_device_unregister(indio_dev);
err_out:
	return ret;
}
static int afe4404_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct afe4404_data *data = iio_priv(indio_dev);

#ifdef AFE4404_DEBUG
	sysfs_remove_group(&client->dev.kobj, &afe4404_attr_group);
#endif

	iio_device_unregister(indio_dev);

	if (!IS_ERR(data->regulator))
		regulator_disable(data->regulator);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int afe4404_suspend(struct device *dev)
{
	struct afe4404_data *afe4404 = dev_get_drvdata(dev);
	int ret;

	ret = afe4404_write(afe4404, AFE4404_CONTROL2, AFE4404_PWR_DWN);
	if (ret)
		goto out;

	ret = regulator_disable(afe4404->regulator);
	if (ret)
		dev_err(dev, "Failed to disable regulator\n");

out:
	return ret;
}

static int afe4404_resume(struct device *dev)
{
	struct afe4404_data *afe4404 = dev_get_drvdata(dev);
	int ret;

	if (afe4404->state) {
		ret = afe4404_write(afe4404, AFE4404_CONTROL2,
				~AFE4404_PWR_DWN);
		if (ret)
			goto out;
	}

	ret = regulator_enable(afe4404->regulator);
	if (ret)
		dev_err(dev, "Failed to disable regulator\n");


out:
	return ret;
}

static SIMPLE_DEV_PM_OPS(afe4404_pm_ops, afe4404_suspend, afe4404_resume);
#define AFE4404_PM_OPS (&afe4404_pm_ops)
#else
#define AFE4404_PM_OPS NULL
#endif

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id afe4404_of_match[] = {
	{ .compatible = "ti,afe4404", },
	{}
};
MODULE_DEVICE_TABLE(of, afe4404_of_match);
#endif

static const struct i2c_device_id afe4404_id[] = {
	{"afe4404", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, afe4404_id);

static struct i2c_driver afe4404_i2c_driver = {
	.driver = {
		.name = "afe4404",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(afe4404_of_match),
		.pm	= AFE4404_PM_OPS,
	},
	.probe = afe4404_probe,
	.remove = afe4404_remove,
	.id_table = afe4404_id,
};
module_i2c_driver(afe4404_i2c_driver);

MODULE_AUTHOR("Dan Murphy <dmurphy@ti.com>");
MODULE_DESCRIPTION("TI AFE4404 Heart Rate and Pulse Oximeter");
MODULE_LICENSE("GPL");
