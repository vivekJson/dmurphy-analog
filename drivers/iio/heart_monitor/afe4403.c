/*
 * AFE4403 Heart Rate Monitors and Low-Cost Pulse Oximeters
 *
 * Author: Dan Murphy <dmurphy@ti.com>
 *
 * Copyright: (C) 2014 Texas Instruments, Inc.
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/regulator/consumer.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/driver.h>
#include <linux/iio/machine.h>
#include <linux/iio/events.h>
#include <linux/iio/kfifo_buf.h>

#define AFE4403_CONTROL0		0x00
#define AFE4403_LED2STC			0x01
#define AFE4403_LED2ENDC		0x02
#define AFE4403_LED2LEDSTC		0x03
#define AFE4403_LED2LEDENDC		0x04
#define AFE4403_ALED2STC		0x05
#define AFE4403_ALED2ENDC		0x06
#define AFE4403_LED1STC			0x07
#define AFE4403_LED1ENDC		0x08
#define AFE4403_LED1LEDSTC		0x09
#define AFE4403_LED1LEDENDC		0x0a
#define AFE4403_ALED1STC		0x0b
#define AFE4403_ALED1ENDC		0x0c
#define AFE4403_LED2CONVST		0x0d
#define AFE4403_LED2CONVEND		0x0e
#define AFE4403_ALED2CONVST		0x0f
#define AFE4403_ALED2CONVEND	0x10
#define AFE4403_LED1CONVST		0x11
#define AFE4403_LED1CONVEND		0x12
#define AFE4403_ALED1CONVST		0x13
#define AFE4403_ALED1CONVEND	0x14
#define AFE4403_ADCRSTSTCT0		0x15
#define AFE4403_ADCRSTENDCT0	0x16
#define AFE4403_ADCRSTSTCT1		0x17
#define AFE4403_ADCRSTENDCT1	0x18
#define AFE4403_ADCRSTSTCT2		0x19
#define AFE4403_ADCRSTENDCT2	0x1a
#define AFE4403_ADCRSTSTCT3		0x1b
#define AFE4403_ADCRSTENDCT3	0x1c
#define AFE4403_PRPCOUNT		0x1d
#define AFE4403_CONTROL1		0x1e
#define AFE4403_SPARE1			0x1f
#define AFE4403_TIAGAIN			0x20
#define AFE4403_TIA_AMB_GAIN	0x21
#define AFE4403_LEDCNTRL		0x22
#define AFE4403_CONTROL2		0x23
#define AFE4403_SPARE2			0x24
#define AFE4403_SPARE3			0x25
#define AFE4403_SPARE4			0x26
#define AFE4403_ALARM			0x29
#define AFE4403_LED2VAL			0x2A
#define AFE4403_ALED2VAL		0x2B
#define AFE4403_LED1VAL			0x2C
#define AFE4403_ALED1VAL		0x2D
#define AFE4403_LED2_ALED2VAL	0x2E
#define AFE4403_LED1_ALED1VAL	0x2F
#define AFE4403_DIAG			0x30
#define AFE4403_CONTROL3		0x31
#define AFE4403_PDNCYCLESTC		0x32
#define AFE4403_PDNCYCLEENDC	0x33

#define AFE4403_SPI_ON			0x0
#define AFE4403_SPI_OFF			0x1

#define AFE4403_SPI_READ		BIT(0)
#define AFE4403_SW_RESET		BIT(3)
#define AFE4403_PWR_DWN			BIT(0)

static DEFINE_MUTEX(afe4403_mutex);

/**
 * struct afe4403_data
 * @indio_dev - IIO device structure
 * @map - IIO map link between consumer and device channels
 * @spi - SPI device pointer the driver is attached to
 * @regulator - Pointer to the regulator for the IC
 * @ste_gpio - SPI serial interface enable line
 * @data_gpio - Interrupt GPIO when AFE data is ready
 * @reset_gpio - Hard wire GPIO reset line
 * @mutex - Read/Write mutex
 * @data_buffer - Data buffer containing the 4 LED values and DIAG
 * @irq - AFE4403 interrupt number
**/
struct afe4403_data {
	struct iio_dev *indio_dev;
	struct iio_map *map;
	struct spi_device *spi;
	struct regulator *regulator;
	struct gpio_desc *ste_gpio;
	struct gpio_desc *data_gpio;
	struct gpio_desc *reset_gpio;
	int data_buffer[5];
	int irq;
};

enum afe4403_reg_id {
	LED1VAL,
	ALED1VAL,
	LED2VAL,
	ALED2VAL,
	DIAG,
	TIAGAIN,
	TIA_AMB_GAIN,
	LEDCNTRL,
	CONTROL3,
};

static const struct iio_event_spec afe4403_events[] = {
	{
		.type = IIO_EV_TYPE_MAG,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
			BIT(IIO_EV_INFO_ENABLE),
	},
};

#define AFE4403_COMMON_CHAN(index, name) { \
		.type = IIO_HEARTRATE,		\
		.indexed = 1,				\
		.channel = index,			\
		.datasheet_name = name, \
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED) \
				| BIT(IIO_CHAN_INFO_SCALE), \
		.scan_index = index,		\
		.scan_type = {				\
			.sign = 'u',			\
			.realbits = 24,			\
			.storagebits = 24,		\
			.endianness = IIO_BE		\
		}, \
		.event_spec = afe4403_events,		\
		.num_event_specs = ARRAY_SIZE(afe4403_events),	\
	}

static const struct iio_chan_spec afe4403_channels[] = {
	/* Read only values from the IC */
	[LED1VAL] = AFE4403_COMMON_CHAN(0, "LED1VAL"),
	[ALED1VAL] = AFE4403_COMMON_CHAN(1, "ALED1VAL"),
	[LED2VAL] = AFE4403_COMMON_CHAN(2, "LED2VAL"),
	[ALED1VAL] = AFE4403_COMMON_CHAN(3, "ALED2VAL"),
	[DIAG] = AFE4403_COMMON_CHAN(4, "DIAG"),

	/* Required writing for calibration */
	[TIAGAIN] = AFE4403_COMMON_CHAN(5, "TIAGAIN"),
	[TIA_AMB_GAIN] = AFE4403_COMMON_CHAN(6, "TIA_AMB_GAIN"),
	[LEDCNTRL] = AFE4403_COMMON_CHAN(7, "LEDCNTRL"),
	[CONTROL3] = AFE4403_COMMON_CHAN(8, "CONTROL3"),
};

static struct iio_map afe4403_default_iio_maps[] = {
	{
		.consumer_dev_name = "afe4403_heartrate",
		.consumer_channel = "afe4403_led1value",
		.adc_channel_label = "LED1VAL",
	},
	{
		.consumer_dev_name = "afe4403_heartrate",
		.consumer_channel = "afe4403_aled1value",
		.adc_channel_label = "ALED1VAL",
	},
	{
		.consumer_dev_name = "afe4403_heartrate",
		.consumer_channel = "afe4403_led2value",
		.adc_channel_label = "LED2VAL",
	},
	{
		.consumer_dev_name = "afe4403_heartrate",
		.consumer_channel = "afe4403_aled2value",
		.adc_channel_label = "ALED2VAL",
	},
	{
		.consumer_dev_name = "afe4403_heartrate",
		.consumer_channel = "afe4403_diagnostics",
		.adc_channel_label = "DIAG",
	},
};

/**
 * Per section 8.5 of the data sheet the SPI interface enable
 * line needs to be pulled and held low throughout the
 * data reads and writes.
*/
static int afe4403_read(struct afe4403_data *afe4403, u8 reg,
		unsigned int *data)
{
	int ret;

	mutex_lock(&afe4403_mutex);

	gpiod_set_value(afe4403->ste_gpio, 0);
	ret = spi_write_then_read(afe4403->spi, (u8 *)&reg, 1, (u8 *)data, 3);
	gpiod_set_value(afe4403->ste_gpio, 1);

	mutex_unlock(&afe4403_mutex);
	return ret;
};

static int afe4403_write(struct afe4403_data *afe4403, u8 reg,
		unsigned int data)
{
	int ret;
	u8 tx[4] = {0x0, 0x0, 0x0, 0x0};

	mutex_lock(&afe4403_mutex);

	/* Enable write to the device */
	tx[0] = AFE4403_CONTROL0;
	tx[3] = 0x0;
	gpiod_set_value(afe4403->ste_gpio, 0);
	ret = spi_write(afe4403->spi, (u8 *)tx, 4);
	if (ret)
		goto out;
	gpiod_set_value(afe4403->ste_gpio, 1);

	tx[0] = reg;
	tx[1] = (data & 0x0f0000) >> 16;
	tx[2] = (data & 0x00ff00) >> 8;
	tx[3] = data & 0xff;
	gpiod_set_value(afe4403->ste_gpio, 0);
	ret = spi_write(afe4403->spi, (u8 *)tx, 4);
	if (ret)
		goto out;

	gpiod_set_value(afe4403->ste_gpio, 1);

	/* Re-Enable reading from the device */
	tx[0] = AFE4403_CONTROL0;
	tx[1] = 0x0;
	tx[2] = 0x0;
	tx[3] = AFE4403_SPI_READ;
	gpiod_set_value(afe4403->ste_gpio, 0);
	ret = spi_write(afe4403->spi, (u8 *)tx, 4);

out:
	gpiod_set_value(afe4403->ste_gpio, 1);
	mutex_unlock(&afe4403_mutex);
	return ret;
};

static int afe4403_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct afe4403_data *data = iio_priv(indio_dev);
	u8 reg;

	if (chan->channel >= ARRAY_SIZE(afe4403_channels))
		return -EINVAL;

	if (chan->channel < 5)
		return -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
	case IIO_CHAN_INFO_SCALE:
		if (chan->channel == 5)
			reg = AFE4403_TIAGAIN;
		else if (chan->channel == 6)
			reg = AFE4403_TIA_AMB_GAIN;
		else if (chan->channel == 7)
			reg = AFE4403_LEDCNTRL;
		else if (chan->channel == 8)
			reg = AFE4403_CONTROL2;
		else
			return -EINVAL;

		return afe4403_write(data, reg, val);
	default:
		break;
	}

	return -EINVAL;
}

static int afe4403_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct afe4403_data *data = iio_priv(indio_dev);

	if (iio_buffer_enabled(indio_dev))
			return -EBUSY;

	if (chan->channel > ARRAY_SIZE(afe4403_channels))
		return -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		*val = data->data_buffer[chan->channel];
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int afe4403_write_event(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir,
				     int state)
{
	struct afe4403_data *afe4403 = iio_priv(indio_dev);
	int ret;
	unsigned int control_val;

	ret = afe4403_read(afe4403, AFE4403_CONTROL2, &control_val);
	if (ret)
		return ret;

	if (state)
		control_val &= ~AFE4403_PWR_DWN;
	else
		control_val |= AFE4403_PWR_DWN;

	ret = afe4403_write(afe4403, AFE4403_CONTROL2, control_val);
	if (ret)
		return ret;

	ret = afe4403_read(afe4403, AFE4403_CONTROL2, &control_val);
	if (ret)
		return ret;

	return 0;
}

static const struct iio_info afe4403_iio_info = {
	.read_raw = &afe4403_read_raw,
	.write_raw = &afe4403_write_raw,
	.write_event_config = &afe4403_write_event,
	.driver_module = THIS_MODULE,
};

static irqreturn_t afe4403_event_handler(int irq, void *data)
{
	struct iio_dev *indio_dev = data;
	struct afe4403_data *afe4403 = iio_priv(indio_dev);
	int ret;

	ret = afe4403_read(afe4403, AFE4403_LED1VAL, &afe4403->data_buffer[0]);
	if (ret)
		goto done;

	ret = afe4403_read(afe4403, AFE4403_ALED1VAL, &afe4403->data_buffer[1]);
	if (ret)
		goto done;

	ret = afe4403_read(afe4403, AFE4403_LED2VAL, &afe4403->data_buffer[2]);
	if (ret)
		goto done;

	ret = afe4403_read(afe4403, AFE4403_ALED2VAL, &afe4403->data_buffer[3]);
	if (ret)
		goto done;

	ret = afe4403_read(afe4403, AFE4403_DIAG, &afe4403->data_buffer[4]);
	if (ret)
		goto done;

	iio_push_to_buffers_with_timestamp(indio_dev, &afe4403->data_buffer,
		iio_get_time_ns());
done:
	return IRQ_HANDLED;
}

struct afe4403_reg {
	uint8_t reg;
	u32 value;
} afe4403_init_regs[] = {
	{ AFE4403_LED2STC,  0x0820},
	{ AFE4403_LED2ENDC, 0x0f9e },
	{ AFE4403_LED2LEDSTC, 0x07d0 },
	{ AFE4403_LED2LEDENDC, 0x0f9f },
	{ AFE4403_ALED2STC, 0x0050 },
	{ AFE4403_ALED2ENDC, 0x07ce },
	{ AFE4403_LED1STC, 0xc350 },
	{ AFE4403_LED1ENDC, 0xc350 },
	{ AFE4403_LED1LEDSTC, 0xc350 },
	{ AFE4403_LED1LEDENDC, 0xc350 },
	{ AFE4403_ALED1STC, 0x0ff0 },
	{ AFE4403_ALED1ENDC, 0x176e },
	{ AFE4403_LED2CONVST, 0x1775 },
	{ AFE4403_LED2CONVEND, 0x1f3f },
	{ AFE4403_ALED2CONVST, 0x1f45 },
	{ AFE4403_ALED2CONVEND, 0x270f },
	{ AFE4403_LED1CONVST, 0x2715 },
	{ AFE4403_LED1CONVEND, 0x2edf },
	{ AFE4403_ALED1CONVST, 0x2ee5 },
	{ AFE4403_ALED1CONVEND, 0x36af },
	{ AFE4403_ADCRSTSTCT0, 0x1770 },
	{ AFE4403_ADCRSTENDCT0, 0x1774 },
	{ AFE4403_ADCRSTSTCT1, 0x1f40 },
	{ AFE4403_ADCRSTENDCT1, 0x1f44 },
	{ AFE4403_ADCRSTSTCT2, 0x2710 },
	{ AFE4403_ADCRSTENDCT2, 0x2714 },
	{ AFE4403_ADCRSTSTCT3, 0x2ee0 },
	{ AFE4403_ADCRSTENDCT3, 0x2ee4 },
	{ AFE4403_PRPCOUNT, 0x09c3f },
	{ AFE4403_CONTROL1, 0x0107 },
	{ AFE4403_TIAGAIN, 0x8006 },
	{ AFE4403_TIA_AMB_GAIN, 0x06 },
	{ AFE4403_LEDCNTRL, 0x11414 },
	{ AFE4403_CONTROL2, 0x20000 },
};

static int afe4403_init(struct afe4403_data *afe4403)
{
	int ret, i, reg_count;

	/* Hard reset the device needs to be held for 1ms per data sheet */
	if (afe4403->reset_gpio) {
		gpiod_set_value(afe4403->reset_gpio, 0);
		udelay(1000);
		gpiod_set_value(afe4403->reset_gpio, 1);
	} else {
		/* Soft reset the device */
		ret = afe4403_write(afe4403, AFE4403_CONTROL0, AFE4403_SW_RESET);
		if (ret)
			return ret;
	}

	reg_count = ARRAY_SIZE(afe4403_init_regs) / sizeof(afe4403_init_regs[0]);
	for (i = 0; i < reg_count; i++) {
		ret = afe4403_write(afe4403, afe4403_init_regs[i].reg,
					afe4403_init_regs[i].value);
		if (ret)
			return ret;
	}

	return ret;
};

static int afe4403_iio_buffered_hardware_setup(struct iio_dev *indio_dev)
{
	struct iio_buffer *buffer;
	int ret;

	buffer = iio_kfifo_allocate(indio_dev);
	if (!buffer)
		return -ENOMEM;

	iio_device_attach_buffer(indio_dev, buffer);

	ret = iio_buffer_register(indio_dev,
				  indio_dev->channels,
				  indio_dev->num_channels);
	if (ret)
		goto error_kfifo_free;

	return 0;

error_kfifo_free:
	iio_kfifo_free(indio_dev->buffer);
	return ret;
}

static int afe4403_spi_probe(struct spi_device *spi)
{
	struct afe4403_data *afe4403;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*afe4403));
	if (indio_dev == NULL) {
		dev_err(&spi->dev, "Failed to allocate iio device\n");
		return -ENOMEM;
	}

	spi_set_drvdata(spi, indio_dev);

	afe4403 = iio_priv(indio_dev);
	afe4403->spi = spi;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = "afe4403";
	indio_dev->info = &afe4403_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	indio_dev->channels = afe4403_channels;
	indio_dev->num_channels = ARRAY_SIZE(afe4403_channels);

	afe4403->ste_gpio = devm_gpiod_get(&spi->dev, "ste");
	if (IS_ERR(afe4403->ste_gpio)) {
		ret = PTR_ERR(afe4403->ste_gpio);
		dev_err(&spi->dev, "Failed to allocate ste gpio\n");
		afe4403->ste_gpio = NULL;
		return ret;
	} else {
		gpiod_direction_output(afe4403->ste_gpio, 1);
	}

	afe4403->data_gpio = devm_gpiod_get(&spi->dev, "data-ready");
	if (IS_ERR(afe4403->data_gpio)) {
		ret = PTR_ERR(afe4403->data_gpio);
		if (ret != -ENOENT && ret != -ENOSYS) {
			dev_err(&spi->dev, "Failed to allocate data_ready gpio\n");
			return ret;
		}
		afe4403->data_gpio = NULL;
	} else {
		gpiod_direction_input(afe4403->data_gpio);
		afe4403->irq = gpiod_to_irq(afe4403->data_gpio);
	}

	afe4403->reset_gpio = devm_gpiod_get(&spi->dev, "reset");
	if (IS_ERR(afe4403->reset_gpio)) {
		ret = PTR_ERR(afe4403->reset_gpio);
		if (ret != -ENOENT && ret != -ENOSYS) {
			dev_err(&spi->dev, "Failed to allocate reset gpio\n");
			return ret;
		}
		afe4403->reset_gpio = NULL;
	} else {
		gpiod_direction_output(afe4403->reset_gpio, 1);
	}

	afe4403->regulator = devm_regulator_get(&spi->dev, "led");
	if (IS_ERR(afe4403->regulator)) {
		ret = PTR_ERR(afe4403->regulator);
		dev_err(&spi->dev,
			"unable to get regulator, error: %d\n", ret);
		return ret;
	}

	ret = iio_map_array_register(indio_dev, afe4403_default_iio_maps);
	if (ret) {
		dev_err(&indio_dev->dev, "iio map err: %d\n", ret);
		return ret;
	}
	afe4403->map = afe4403_default_iio_maps;

	ret = afe4403_iio_buffered_hardware_setup(indio_dev);
	if (ret) {
		dev_err(&spi->dev, "Failed to allocate iio buffers %i\n", ret);
		goto buffer_setup_failed;
	}

	ret = devm_request_threaded_irq(&spi->dev, afe4403->irq,
				   NULL,
				   afe4403_event_handler,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				   "afe4403 int",
				   indio_dev);
	if (ret) {
		dev_err(&spi->dev, "Failed to allocate thread %i\n", ret);
		goto req_thread_failed;
	}

	ret = devm_iio_device_register(&spi->dev, indio_dev);
	if (ret < 0)
		goto req_thread_failed;

	ret = afe4403_init(afe4403);
	if (ret) {
		dev_err(&spi->dev, "Failed to init device\n");
		goto req_thread_failed;
	}

	return 0;

req_thread_failed:
	iio_kfifo_free(indio_dev->buffer);
	iio_buffer_unregister(indio_dev);
buffer_setup_failed:
	iio_map_array_unregister(indio_dev);
	return ret;
}

static int afe4403_spi_remove(struct spi_device *spi)
{
	struct afe4403_data *afe4403 = dev_get_drvdata(&spi->dev);

	iio_kfifo_free(afe4403->indio_dev->buffer);
	iio_buffer_unregister(afe4403->indio_dev);

	if (!IS_ERR(afe4403->regulator))
		regulator_disable(afe4403->regulator);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int afe4403_suspend(struct device *dev)
{
	struct afe4403_data *afe4403 = dev_get_drvdata(dev);
	int ret = 0;

	ret = afe4403_write(afe4403, AFE4403_CONTROL2, AFE4403_PWR_DWN);
	if (ret)
		goto out;

	ret = regulator_disable(afe4403->regulator);
	if (ret)
		dev_err(dev, "Failed to disable regulator\n");

out:
	return ret;
}

static int afe4403_resume(struct device *dev)
{
	struct afe4403_data *afe4403 = dev_get_drvdata(dev);
	int ret = 0;

	ret = afe4403_write(afe4403, AFE4403_CONTROL2, ~AFE4403_PWR_DWN);
	if (ret)
		goto out;

	ret = regulator_enable(afe4403->regulator);
	if (ret)
		dev_err(dev, "Failed to disable regulator\n");

out:
	return ret;
}

static SIMPLE_DEV_PM_OPS(afe4403_pm_ops, afe4403_suspend, afe4403_resume);
#define AFE4403_PM_OPS (&afe4403_pm_ops)
#else
#define AFE4403_PM_OPS NULL
#endif

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id afe4403_of_match[] = {
	{ .compatible = "ti,afe4403", },
	{}
};
MODULE_DEVICE_TABLE(of, afe4403_of_match);
#endif

static const struct spi_device_id afe4403_id[] = {
	{"afe4403", 0},
	{},
};
MODULE_DEVICE_TABLE(spi, afe4403_id);

static struct spi_driver afe4403_spi_driver = {
	.driver = {
		.name = "afe4403",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(afe4403_of_match),
		.pm	= AFE4403_PM_OPS,
	},
	.probe = afe4403_spi_probe,
	.remove = afe4403_spi_remove,
	.id_table = afe4403_id,
};
module_spi_driver(afe4403_spi_driver);

MODULE_AUTHOR("Dan Murphy <dmurphy@ti.com>");
MODULE_DESCRIPTION("TI AFE4403 Heart Rate and Pulse Oximeter");
MODULE_LICENSE("GPL");
