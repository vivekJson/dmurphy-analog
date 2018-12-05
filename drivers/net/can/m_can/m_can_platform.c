/*
 * CAN bus driver for Bosch M_CAN controller
 *
 * Copyright (C) 2014 Freescale Semiconductor, Inc.
 *	Dong Aisheng <b29396@freescale.com>
 *
 * Bosch M_CAN user manual can be obtained from:
 * http://www.bosch-semiconductors.de/media/pdf_1/ipmodules_1/m_can/
 * mcan_users_manual_v302.pdf
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/can/dev.h>
#include <linux/pinctrl/consumer.h>

#include "m_can_platform.h"

struct m_can_plat_priv {
	void __iomem *base;
	void __iomem *mram_base;
};

static u32 iomap_read_reg(struct m_can_classdev *m_can_class, int reg)
{
	struct m_can_plat_priv *priv = (struct m_can_plat_priv *)m_can_class->device_data;

	return readl(priv->base + reg);
}

static u32 iomap_read_fifo(struct m_can_classdev *m_can_class, int addr_offset)
{
	struct m_can_plat_priv *priv = (struct m_can_plat_priv *)m_can_class->device_data;

	return readl(priv->mram_base + addr_offset);
}

static int iomap_write_reg(struct m_can_classdev *m_can_class, int reg, int val)
{
	struct m_can_plat_priv *priv = (struct m_can_plat_priv *)m_can_class->device_data;

	writel(val, priv->base + reg);

	return 0;
}

static int iomap_write_fifo(struct m_can_classdev *m_can_class, int addr_offset, int val)
{
	struct m_can_plat_priv *priv = (struct m_can_plat_priv *)m_can_class->device_data;

	writel(val, priv->base + addr_offset);

	return 0;
}

static int m_can_plat_probe(struct platform_device *pdev)
{
	struct m_can_classdev *mcan_class;
	struct m_can_plat_priv *priv;
	struct resource *res;
	void __iomem *addr;
	void __iomem *mram_addr;
	int irq, ret = 0;

	mcan_class = m_can_core_allocate_dev(&pdev->dev);
	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	mcan_class->device_data = priv;

	m_can_core_get_clocks(mcan_class);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "m_can");
	addr = devm_ioremap_resource(&pdev->dev, res);
	irq = platform_get_irq_byname(pdev, "int0");
	if (IS_ERR(addr) || irq < 0) {
		ret = -EINVAL;
		goto failed_ret;
	}

	/* message ram could be shared */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "message_ram");
	if (!res) {
		ret = -ENODEV;
		goto failed_ret;
	}

	mram_addr = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!mram_addr) {
		ret = -ENOMEM;
		goto failed_ret;
	}

	priv->base = addr;
	priv->mram_base = mram_addr;

	mcan_class->net->irq = irq;
	mcan_class->pm_clock_support = 1;
	mcan_class->can.clock.freq = clk_get_rate(mcan_class->cclk);
	mcan_class->dev = &pdev->dev;

	mcan_class->read_reg = &iomap_read_reg;
	mcan_class->write_reg = &iomap_write_reg;
	mcan_class->write_fifo = &iomap_write_fifo;
	mcan_class->read_fifo = &iomap_read_fifo;
	mcan_class->is_peripherial = false;

	platform_set_drvdata(pdev, mcan_class->dev);

	m_can_init_ram(mcan_class);

	ret = m_can_core_register(mcan_class);

failed_ret:
	return ret;
}

static __maybe_unused int m_can_suspend(struct device *dev)
{
	return m_can_core_suspend(dev);
}

static __maybe_unused int m_can_resume(struct device *dev)
{
	return m_can_core_resume(dev);
}

static int m_can_plat_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct m_can_classdev *mcan_class = netdev_priv(dev);

	m_can_core_unregister(mcan_class);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static int __maybe_unused m_can_runtime_suspend(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct m_can_classdev *mcan_class = netdev_priv(ndev);

	m_can_core_suspend(dev);

	clk_disable_unprepare(mcan_class->cclk);
	clk_disable_unprepare(mcan_class->hclk);

	return 0;
}

static int __maybe_unused m_can_runtime_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct m_can_classdev *mcan_class = netdev_priv(ndev);
	int err;

	err = clk_prepare_enable(mcan_class->hclk);
	if (err)
		return err;

	err = clk_prepare_enable(mcan_class->cclk);
	if (err)
		clk_disable_unprepare(mcan_class->hclk);

	m_can_core_resume(dev);

	return err;
}

static const struct dev_pm_ops m_can_pmops = {
	SET_RUNTIME_PM_OPS(m_can_runtime_suspend,
			   m_can_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(m_can_suspend, m_can_resume)
};

static const struct of_device_id m_can_of_table[] = {
	{ .compatible = "bosch,m_can", .data = NULL },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, m_can_of_table);

static struct platform_driver m_can_plat_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = m_can_of_table,
		.pm     = &m_can_pmops,
	},
	.probe = m_can_plat_probe,
	.remove = m_can_plat_remove,
};

module_platform_driver(m_can_plat_driver);

MODULE_AUTHOR("Dong Aisheng <b29396@freescale.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CAN bus driver for Bosch M_CAN controller");
