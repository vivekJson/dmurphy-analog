// SPDX-License-Identifier: GPL-2.0
// TI ADS124S0X chip family driver
// Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/

#include <linux/err.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/sysfs.h>

#define ADS124S08_ID		0x00
#define ADS124S06_ID		0x01

/* Commands */
#define ADS124S_CMD_NOP		0x00
#define ADS124S_CMD_WAKEUP	0x02
#define ADS124S_CMD_PWRDWN	0x04
#define ADS124S_CMD_RESET	0x06
#define ADS124S_CMD_START	0x08
#define ADS124S_CMD_STOP	0x0a
#define ADS124S_CMD_SYOCAL	0x16
#define ADS124S_CMD_SYGCAL	0x17
#define ADS124S_CMD_SFOCAL	0x19
#define ADS124S_CMD_RDATA	0x12
#define ADS124S_CMD_RREG	0x20
#define ADS124S_CMD_WREG	0x40

/* Registers */
#define ADS124S0X_ID		0x00
#define ADS124S0X_STATUS	0x01
#define ADS124S0X_INPUT_MUX	0x02
#define ADS124S0X_PGA		0x03
#define ADS124S0X_DATA_RATE	0x04
#define ADS124S0X_REF		0x05
#define ADS124S0X_IDACMAG	0x06
#define ADS124S0X_IDACMUX	0x07
#define ADS124S0X_VBIAS		0x08
#define ADS124S0X_SYS		0x09
#define ADS124S0X_OFCAL0	0x0a
#define ADS124S0X_OFCAL1	0x0b
#define ADS124S0X_OFCAL2	0x0c
#define ADS124S0X_FSCAL0	0x0d
#define ADS124S0X_FSCAL1	0x0e
#define ADS124S0X_FSCAL2	0x0f
#define ADS124S0X_GPIODAT	0x10
#define ADS124S0X_GPIOCON	0x11

/* ADS124S0x common channels */
#define ADS124S0X_AIN0		0x00
#define ADS124S0X_AIN1		0x01
#define ADS124S0X_AIN2		0x02
#define ADS124S0X_AIN3		0x03
#define ADS124S0X_AIN4		0x04
#define ADS124S0X_AIN5		0x05
#define ADS124S08_AINCOM	0x0c
/* ADS124S08 only channels */
#define ADS124S08_AIN6		0x06
#define ADS124S08_AIN7		0x07
#define ADS124S08_AIN8		0x08
#define ADS124S08_AIN9		0x09
#define ADS124S08_AIN10		0x0a
#define ADS124S08_AIN11		0x0b
#define ADS124S0X_MAX_CHANNELS	12

#define ADS124S0X_POS_MUX_SHIFT	0x04
#define ADS124S_INT_REF		0x09

#define ADS124S_START_REG_MASK	0x1f
#define ADS124S_NUM_BYTES_MASK	0x1f

#define ADS124S_START_CONV	0x01
#define ADS124S_STOP_CONV	0x00

struct ads124s_chip_info {
	const struct iio_chan_spec *channels;
	unsigned int num_channels;
};

struct ads124s_private {
	const struct ads124s_chip_info	*chip_info;
	struct gpio_desc *reset_gpio;
	struct spi_device *spi;
	struct regulator *reg;
	struct mutex lock;
	unsigned int vref_mv;
	u8 data[ADS124S0X_MAX_CHANNELS];
};

#define ADS124S_CHAN(index)					\
{								\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = index,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.scan_index = index,					\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = 24,					\
		.storagebits = 24,				\
		.endianness = IIO_BE,				\
	},							\
}

static const struct iio_chan_spec ads124s06_channels[] = {
	ADS124S_CHAN(0),
	ADS124S_CHAN(1),
	ADS124S_CHAN(2),
	ADS124S_CHAN(3),
	ADS124S_CHAN(4),
	ADS124S_CHAN(5),
};

static const struct iio_chan_spec ads124s08_channels[] = {
	ADS124S_CHAN(0),
	ADS124S_CHAN(1),
	ADS124S_CHAN(2),
	ADS124S_CHAN(3),
	ADS124S_CHAN(4),
	ADS124S_CHAN(5),
	ADS124S_CHAN(6),
	ADS124S_CHAN(7),
	ADS124S_CHAN(8),
	ADS124S_CHAN(9),
	ADS124S_CHAN(10),
	ADS124S_CHAN(11),
};

static const struct ads124s_chip_info ads124s_chip_info_tbl[] = {
	[ADS124S08_ID] = {
		.channels = ads124s08_channels,
		.num_channels = ARRAY_SIZE(ads124s08_channels),
	},
	[ADS124S06_ID] = {
		.channels = ads124s06_channels,
		.num_channels = ARRAY_SIZE(ads124s06_channels),
	},
};

static void ads124s_fill_nop(u8 *data, int start, int count)
{
	int i;

	for (i = start; i <= count; i++)
		data[i] = ADS124S_CMD_NOP;

};

static int ads124s_write_cmd(struct iio_dev *indio_dev, u8 command)
{
	struct ads124s_private *priv = iio_priv(indio_dev);
	struct spi_transfer t[] = {
		{
			.tx_buf = &priv->data[0],
			.len = 1,
		}
	};

	priv->data[0] = command;

	return spi_sync_transfer(priv->spi, t, ARRAY_SIZE(t));
}

static int ads124s_write_reg(struct iio_dev *indio_dev, u8 reg, u8 data)
{
	struct ads124s_private *priv = iio_priv(indio_dev);
	struct spi_transfer t[] = {
		{
			.tx_buf = &priv->data[0],
			.len = 3,
		}
	};

	priv->data[0] = ADS124S_CMD_WREG | reg;
	priv->data[1] = 0x0;
	priv->data[2] = data;

	return spi_sync_transfer(priv->spi, t, ARRAY_SIZE(t));
}

static int ads124s_reset(struct iio_dev *indio_dev)
{
	struct ads124s_private *priv = iio_priv(indio_dev);

	if (priv->reset_gpio) {
		gpiod_direction_output(priv->reset_gpio, 0);
		udelay(200);
		gpiod_direction_output(priv->reset_gpio, 1);
	} else {
		return ads124s_write_cmd(indio_dev, ADS124S_CMD_RESET);
	}

	return 0;
};

static int ads124s_read(struct iio_dev *indio_dev, unsigned int chan)
{
	struct ads124s_private *priv = iio_priv(indio_dev);
	int ret;
	u32 tmp;
	struct spi_transfer t[] = {
		{
			.tx_buf = &priv->data[0],
			.len = 4,
			.cs_change = 1,
		}, {
			.tx_buf = &priv->data[1],
			.rx_buf = &priv->data[1],
			.len = 4,
		},
	};

	priv->data[0] = ADS124S_CMD_RDATA;
	ads124s_fill_nop(priv->data, 1, 3);

	ret = spi_sync_transfer(priv->spi, t, ARRAY_SIZE(t));
	if (ret < 0)
		return ret;

	tmp = priv->data[2] << 16 | priv->data[3] << 8 | priv->data[4];

	return tmp;
}

static int ads124s_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long m)
{
	struct ads124s_private *priv = iio_priv(indio_dev);
	int ret;

	mutex_lock(&priv->lock);
	switch (m) {
	case IIO_CHAN_INFO_RAW:
		ret = ads124s_write_reg(indio_dev, ADS124S0X_INPUT_MUX,
					chan->channel);
		if (ret) {
			dev_err(&priv->spi->dev, "Set ADC CH failed\n");
			goto out;
		}

		ret = ads124s_write_cmd(indio_dev, ADS124S_START_CONV);
		if (ret) {
			dev_err(&priv->spi->dev, "Start ADC converions failed\n");
			goto out;
		}

		ret = ads124s_read(indio_dev, chan->channel);
		if (ret < 0) {
			dev_err(&priv->spi->dev, "Read ADC failed\n");
			goto out;
		} else
			*val = ret;

		ret = ads124s_write_cmd(indio_dev, ADS124S_STOP_CONV);
		if (ret) {
			dev_err(&priv->spi->dev, "Stop ADC converions failed\n");
			goto out;
		}

		mutex_unlock(&priv->lock);
		if (ret < 0)
			return ret;

		return IIO_VAL_INT;
	default:
		break;
	}
out:
	mutex_unlock(&priv->lock);
	return -EINVAL;
}

static const struct iio_info ads124s_info = {
	.read_raw = &ads124s_read_raw,
};

static irqreturn_t ads124s_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ads124s_private *priv = iio_priv(indio_dev);
	u16 buffer[ADS124S0X_MAX_CHANNELS];
	int i, j = 0;
	int ret;

	for (i = 0; i < indio_dev->masklength; i++) {
		if (!test_bit(i, indio_dev->active_scan_mask))
			continue;

		ret = ads124s_write_reg(indio_dev, ADS124S0X_INPUT_MUX, i);
		if (ret)
			dev_err(&priv->spi->dev, "Set ADC CH failed\n");

		ret = ads124s_write_cmd(indio_dev, ADS124S_START_CONV);
		if (ret)
			dev_err(&priv->spi->dev, "Start ADC converions failed\n");

		buffer[j] = ads124s_read(indio_dev, i);
		ret = ads124s_write_cmd(indio_dev, ADS124S_STOP_CONV);
		if (ret)
			dev_err(&priv->spi->dev, "Stop ADC converions failed\n");

		j++;
	}

	iio_push_to_buffers_with_timestamp(indio_dev, buffer,
			pf->timestamp);

	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static int ads124s_probe(struct spi_device *spi)
{
	struct ads124s_private *ads124s_priv;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*ads124s_priv));
	if (indio_dev == NULL)
		return -ENOMEM;

	ads124s_priv = iio_priv(indio_dev);

	ads124s_priv->reg = devm_regulator_get_optional(&spi->dev, "vref");
	if (!IS_ERR(ads124s_priv->reg)) {
		ret = regulator_enable(ads124s_priv->reg);
		if (ret)
			return ret;

		ads124s_priv->vref_mv = regulator_get_voltage(ads124s_priv->reg);
		if (ads124s_priv->vref_mv <= 0)
			goto err_regulator_disable;
	} else {
		/* Use internal reference */
		ads124s_priv->vref_mv = 0;
	}

	ads124s_priv->reset_gpio = devm_gpiod_get_optional(&spi->dev,
						   "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ads124s_priv->reset_gpio))
		dev_info(&spi->dev, "Reset GPIO not defined\n");

	ads124s_priv->chip_info = &ads124s_chip_info_tbl[spi_get_device_id(spi)->driver_data];

	spi_set_drvdata(spi, indio_dev);

	ads124s_priv->spi = spi;

	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->dev.parent = &spi->dev;
	indio_dev->dev.of_node = spi->dev.of_node;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ads124s_priv->chip_info->channels;
	indio_dev->num_channels = ads124s_priv->chip_info->num_channels;
	indio_dev->info = &ads124s_info;

	mutex_init(&ads124s_priv->lock);

	ret = iio_triggered_buffer_setup(indio_dev, NULL,
					 ads124s_trigger_handler, NULL);
	if (ret < 0) {
		dev_err(&spi->dev, "iio triggered buffer setup failed\n");
		goto err_regulator_disable;
	}

	ads124s_reset(indio_dev);

	ret = iio_device_register(indio_dev);
	if (ret)
		goto err_buffer_cleanup;

	return 0;

err_buffer_cleanup:
	iio_triggered_buffer_cleanup(indio_dev);

err_regulator_disable:
	if (!IS_ERR(ads124s_priv->reg))
		regulator_disable(ads124s_priv->reg);

	return ret;
}

static int ads124s_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);

	iio_device_unregister(indio_dev);
	iio_triggered_buffer_cleanup(indio_dev);

	return 0;
}

static const struct spi_device_id ads124s_id[] = {
	{ "ads124s08", ADS124S08_ID },
	{ "ads124s06", ADS124S06_ID },
	{ }
};
MODULE_DEVICE_TABLE(spi, ads124s_id);

static const struct of_device_id ads124s_of_table[] = {
	{ .compatible = "ti,ads124s08" },
	{ .compatible = "ti,ads124s06" },
	{ },
};
MODULE_DEVICE_TABLE(of, ads124s_of_table);

static struct spi_driver ads124s_driver = {
	.driver = {
		.name	= "ads124s",
		.of_match_table = ads124s_of_table,
	},
	.probe		= ads124s_probe,
	.remove		= ads124s_remove,
	.id_table	= ads124s_id,
};
module_spi_driver(ads124s_driver);

MODULE_AUTHOR("Dan Murphy <dmuprhy@ti.com>");
MODULE_DESCRIPTION("TI TI_ADS12S0X ADC");
MODULE_LICENSE("GPL v2");
