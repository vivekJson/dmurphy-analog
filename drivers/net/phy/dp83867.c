/*
 * Driver for the Texas Instruments DP83867 PHY
 *
 * Copyright (C) 2015 Texas Instruments Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include <linux/ethtool.h>
#include <linux/kernel.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/phy.h>

#define DP83867_PHY_ID	0x2000a231
#define DP83867_DEVADDR		0x1f

#define MII_DP83867_PHYCTRL	0x10
#define MII_DP83867_MICR	0x12
#define MII_DP83867_ISR		0x13
#define DP83867_CTRL		0x1f

/* Extended Registers */
#define DP83867_RGMIICTL	0x0032
#define DP83867_RGMIIDCTL	0x0086

#define DP83867_SW_RESET	BIT(15)
#define DP83867_SW_RESTART	BIT(14)

/* MICR Interrupt bits */
#define MII_DP83867_MICR_AN_ERR_INT_EN		BIT(15)
#define MII_DP83867_MICR_SPEED_CHNG_INT_EN	BIT(14)
#define MII_DP83867_MICR_DUP_MODE_CHNG_INT_EN	BIT(13)
#define MII_DP83867_MICR_PAGE_RXD_INT_EN	BIT(12)
#define MII_DP83867_MICR_AUTONEG_COMP_INT_EN	BIT(11)
#define MII_DP83867_MICR_LINK_STS_CHNG_INT_EN	BIT(10)
#define MII_DP83867_MICR_FALSE_CARRIER_INT_EN	BIT(8)
#define MII_DP83867_MICR_SLEEP_MODE_CHNG_INT_EN	BIT(4)
#define MII_DP83867_MICR_WOL_INT_EN		BIT(3)
#define MII_DP83867_MICR_XGMII_ERR_INT_EN	BIT(2)
#define MII_DP83867_MICR_POL_CHNG_INT_EN	BIT(1)
#define MII_DP83867_MICR_JABBER_INT_EN		BIT(0)

/* RGMIICTL bits */
#define DP83867_RGMII_TX_CLK_DELAY_EN		BIT(1)
#define DP83867_RGMII_RX_CLK_DELAY_EN		BIT(0)

/* RGMIIDCTL bits */
#define DP83867_RGMII_RX_CLK_DELAY_SHIFT	4
enum {
	DP83867_RGMIIDCTL_250_PS,
	DP83867_RGMIIDCTL_500_PS,
	DP83867_RGMIIDCTL_750_PS,
	DP83867_RGMIIDCTL_1_NS,
	DP83867_RGMIIDCTL_1_25_NS,
	DP83867_RGMIIDCTL_1_50_NS,
	DP83867_RGMIIDCTL_1_75_NS,
	DP83867_RGMIIDCTL_2_00_NS,
	DP83867_RGMIIDCTL_2_25_NS,
	DP83867_RGMIIDCTL_2_50_NS,
	DP83867_RGMIIDCTL_2_75_NS,
	DP83867_RGMIIDCTL_3_00_NS,
	DP83867_RGMIIDCTL_3_25_NS,
	DP83867_RGMIIDCTL_3_50_NS,
	DP83867_RGMIIDCTL_3_75_NS,
	DP83867_RGMIIDCTL_4_00_NS,
};

int rx_tx_delay = (DP83867_RGMIIDCTL_2_75_NS << DP83867_RGMII_RX_CLK_DELAY_SHIFT) | DP83867_RGMIIDCTL_2_25_NS;
module_param(rx_tx_delay, int, 0664);


static int dp83867_ack_interrupt(struct phy_device *phydev)
{
	int err = phy_read(phydev, MII_DP83867_ISR);

	if (err < 0)
		return err;

	return 0;
}

static int dp83867_config_intr(struct phy_device *phydev)
{
	int micr_status;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED) {
		micr_status = phy_read(phydev, MII_DP83867_MICR);
		if (micr_status < 0)
			return micr_status;

		micr_status |=
			(MII_DP83867_MICR_AN_ERR_INT_EN |
			MII_DP83867_MICR_SPEED_CHNG_INT_EN |
			MII_DP83867_MICR_DUP_MODE_CHNG_INT_EN |
			MII_DP83867_MICR_SLEEP_MODE_CHNG_INT_EN);

		return phy_write(phydev, MII_DP83867_MICR, micr_status);
	}

	micr_status = 0x0;
	return phy_write(phydev, MII_DP83867_MICR, micr_status);
}

static int dp83867_config_init(struct phy_device *phydev)
{
	int val;

	if ((phydev->interface >= PHY_INTERFACE_MODE_RGMII) ||
	   (phydev->interface <= PHY_INTERFACE_MODE_RGMII_TXID))
		phy_write(phydev, MII_DP83867_PHYCTRL, 0x5048);

	if ((phydev->interface >= PHY_INTERFACE_MODE_RGMII_ID) ||
	    (phydev->interface <= PHY_INTERFACE_MODE_RGMII_RXID)) {
		val = phy_read_mmd_indirect(phydev, DP83867_RGMIICTL,
				DP83867_DEVADDR, phydev->addr);

		if (phydev->interface == PHY_INTERFACE_MODE_RGMII_ID)
			val |= (DP83867_RGMII_TX_CLK_DELAY_EN | DP83867_RGMII_RX_CLK_DELAY_EN);

		if (phydev->interface == PHY_INTERFACE_MODE_RGMII_TXID)
			val |= DP83867_RGMII_TX_CLK_DELAY_EN;

		if (phydev->interface == PHY_INTERFACE_MODE_RGMII_RXID)
			val |= DP83867_RGMII_RX_CLK_DELAY_EN;

		phy_write_mmd_indirect(phydev, DP83867_RGMIICTL,
					DP83867_DEVADDR, phydev->addr, val);

		phy_write_mmd_indirect(phydev, DP83867_RGMIIDCTL,
				DP83867_DEVADDR, phydev->addr, rx_tx_delay);
	}

	return 0;
}

static int dp83867_phy_reset(struct phy_device *phydev)
{
	int err;

	err = phy_write(phydev, DP83867_CTRL, DP83867_SW_RESET);
	if (err < 0)
		return err;

	err = dp83867_config_init(phydev);
	return err;
}

static struct phy_driver dp83867_driver = {
	.phy_id		= DP83867_PHY_ID,
	.phy_id_mask	= 0xfffffff0,
	.name		= "TI DP83867",
	.features	= PHY_GBIT_FEATURES,
	.flags		= PHY_HAS_INTERRUPT,

	.config_init	= dp83867_config_init,
	.soft_reset	= dp83867_phy_reset,

	/* IRQ related */
	.ack_interrupt	= dp83867_ack_interrupt,
	.config_intr	= dp83867_config_intr,

	.config_aneg	= genphy_config_aneg,
	.read_status	= genphy_read_status,
	.suspend	= genphy_suspend,
	.resume		= genphy_resume,

	.driver		= {.owner = THIS_MODULE,}
};

static int __init dp83867_init(void)
{
	return phy_driver_register(&dp83867_driver);
}

static void __exit dp83867_exit(void)
{
	phy_driver_unregister(&dp83867_driver);
}

module_init(dp83867_init);
module_exit(dp83867_exit);

static struct mdio_device_id __maybe_unused dp83867_tbl[] = {
	{ DP83867_PHY_ID, 0xfffffff0 },
	{ }
};

MODULE_DEVICE_TABLE(mdio, dp83867_tbl);

MODULE_DESCRIPTION("Texas Instruments DP83867 PHY driver");
MODULE_AUTHOR("Dan Murphy <dmurphy@ti.com");
MODULE_LICENSE("GPL");
