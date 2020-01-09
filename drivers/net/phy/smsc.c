/*
 * drivers/net/phy/smsc.c
 *
 * Driver for SMSC PHYs
 *
 * Author: Herbert Valerio Riedel
 *
 * Copyright (c) 2006 Herbert Valerio Riedel <hvr@gnu.org>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * Support added for SMSC LAN8187 and LAN8700 by steve.glendinning@shawell.net
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/phy.h>
#include <linux/netdevice.h>
#include <linux/smscphy.h>
#include <linux/debugfs.h>

struct smsc_phy_priv {
	bool energy_enable;
	/* Duty cycle for toggling power down bit.
	* The link is checked every `offtime` ms if the PHY is powered down
	* (taking into account that phy_read is executed every ~1s).
	* The PHY is powered for `ontime` ms to leave enough time for the link
	* to be detected.
	*/
	u32 ontime;
	u32 offtime;
	unsigned long last_ontime;
	/* We can't rely on phydev->link to know if the device's powered down */
	bool powered_down;
	bool dbg;
	struct dentry *debugfs;
};

static int smsc_phy_config_intr(struct phy_device *phydev)
{
	int rc = phy_write (phydev, MII_LAN83C185_IM,
			((PHY_INTERRUPT_ENABLED == phydev->interrupts)
			? MII_LAN83C185_ISF_INT_PHYLIB_EVENTS
			: 0));

	return rc < 0 ? rc : 0;
}

static int smsc_phy_ack_interrupt(struct phy_device *phydev)
{
	int rc = phy_read (phydev, MII_LAN83C185_ISF);

	return rc < 0 ? rc : 0;
}

static int smsc_phy_config_init(struct phy_device *phydev)
{
	struct smsc_phy_priv *priv = phydev->priv;

	int rc = phy_read(phydev, MII_LAN83C185_CTRL_STATUS);

	if (rc < 0)
		return rc;

	if (priv->energy_enable) {
		/* Enable energy detect mode for this SMSC Transceivers */
		rc = phy_write(phydev, MII_LAN83C185_CTRL_STATUS,
			       rc | MII_LAN83C185_EDPWRDOWN);
		if (rc < 0)
			return rc;
	}

	return smsc_phy_ack_interrupt(phydev);
}

int phy_modify(struct phy_device *phydev, u32 regnum, u16 mask, u16 set)
{
	int rc = phy_read(phydev, regnum);

	if (rc < 0)
		return rc;

	rc = (rc & ~mask) | set;

	return phy_write(phydev, regnum, rc);
}

static int smsc_phy_reset(struct phy_device *phydev)
{
	int rc = phy_read(phydev, MII_LAN83C185_SPECIAL_MODES);
	if (rc < 0)
		return rc;

	/* If the SMSC PHY is in power down mode, then set it
	 * in all capable mode before using it.
	 */
	if ((rc & MII_LAN83C185_MODE_MASK) == MII_LAN83C185_MODE_POWERDOWN) {
		/* set "all capable" mode */
		rc |= MII_LAN83C185_MODE_ALL;
		phy_write(phydev, MII_LAN83C185_SPECIAL_MODES, rc);
	}

	/* reset the phy */
	return genphy_soft_reset(phydev);
}

static int lan911x_config_init(struct phy_device *phydev)
{
	return smsc_phy_ack_interrupt(phydev);
}

static int lan87xx_read_status(struct phy_device *phydev)
{
	struct smsc_phy_priv *priv = phydev->priv;
	int err = genphy_read_status(phydev);

	/*
	 * If we are in the powered_down state, phydev->link will be true
	 * for one reason or another, so we manually need to keep track of
	 * that using the powered_down flag.
	 */
	if (phydev->link && !priv->powered_down)
		return err;

	if (!priv->powered_down)
		goto pdown;

	/* Check if we passed at least offtime before powering up again. */
	if (time_before(jiffies, msecs_to_jiffies(priv->offtime) +
			priv->last_ontime)) {
		/* Make sure we report that the link is not up when we are
		 * powered_down, for reasons, see above.
		 */
		phydev->link = 0;
		return err;
	}

	if (priv->dbg)
		netdev_info(phydev->attached_dev, "powering up\n");

	if (phydev->drv->soft_reset)
		phydev->drv->soft_reset(phydev);

	phy_modify(phydev, MII_BMCR, BMCR_ANENABLE | BMCR_PDOWN, BMCR_ANENABLE);
	priv->powered_down = false;

	/* After power up, wait at least ontime to make sure the PHY has time to
	 * fully detect a link up of even the worst links.
	 */
	msleep(priv->ontime);

	err = genphy_read_status(phydev);

	if (priv->dbg)
		netdev_info(phydev->attached_dev, phydev->link ? "got link\n" :
			    "powering down\n");

pdown:
	if (!phydev->link && priv->offtime) {
		phy_modify(phydev, MII_BMCR, BMCR_ANENABLE, 0);
		phy_modify(phydev, MII_BMCR, BMCR_PDOWN, BMCR_PDOWN);
		priv->last_ontime = jiffies;
		priv->powered_down = true;
	}

	return err;
}

static int smsc_phy_probe(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	struct device_node *of_node = dev->of_node;
	struct smsc_phy_priv *priv;
	struct dentry *smsc;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	/* Energy power down doesn't work on LAN8720A: link changes aren't
	 * detected */
	priv->energy_enable = false;

	/* Default value for ontime duty cycle. Offtime is 0 and in this
	 * configuration, the power consumption is the worst but the PHY is the
	 * most responsive. */
	priv->ontime = 2000;

	priv->debugfs = debugfs_create_dir("ethernet", NULL);
	smsc = debugfs_create_dir("smsc", priv->debugfs);
	debugfs_create_u32("ontime", S_IRUGO | S_IWUSR, smsc, &priv->ontime);
	debugfs_create_u32("offtime", S_IRUGO | S_IWUSR, smsc, &priv->offtime);
	debugfs_create_bool("dbg", S_IRUGO | S_IWUSR, smsc, &priv->dbg);

	if (of_property_read_bool(of_node, "smsc,disable-energy-detect"))
		priv->energy_enable = false;

	phydev->priv = priv;

	return 0;
}

static void smsc_phy_remove(struct phy_device *phydev)
{
	struct smsc_phy_priv *priv = phydev->priv;

	debugfs_remove_recursive(priv->debugfs);
}

static struct phy_driver smsc_phy_driver[] = {
{
	.phy_id		= 0x0007c0a0, /* OUI=0x00800f, Model#=0x0a */
	.phy_id_mask	= 0xfffffff0,
	.name		= "SMSC LAN83C185",

	.features	= (PHY_BASIC_FEATURES | SUPPORTED_Pause
				| SUPPORTED_Asym_Pause),
	.flags		= PHY_HAS_INTERRUPT | PHY_HAS_MAGICANEG,

	.probe		= smsc_phy_probe,

	/* basic functions */
	.config_aneg	= genphy_config_aneg,
	.read_status	= genphy_read_status,
	.config_init	= smsc_phy_config_init,
	.soft_reset	= smsc_phy_reset,

	/* IRQ related */
	.ack_interrupt	= smsc_phy_ack_interrupt,
	.config_intr	= smsc_phy_config_intr,

	.suspend	= genphy_suspend,
	.resume		= genphy_resume,
}, {
	.phy_id		= 0x0007c0b0, /* OUI=0x00800f, Model#=0x0b */
	.phy_id_mask	= 0xfffffff0,
	.name		= "SMSC LAN8187",

	.features	= (PHY_BASIC_FEATURES | SUPPORTED_Pause
				| SUPPORTED_Asym_Pause),
	.flags		= PHY_HAS_INTERRUPT | PHY_HAS_MAGICANEG,

	.probe		= smsc_phy_probe,

	/* basic functions */
	.config_aneg	= genphy_config_aneg,
	.read_status	= genphy_read_status,
	.config_init	= smsc_phy_config_init,
	.soft_reset	= smsc_phy_reset,

	/* IRQ related */
	.ack_interrupt	= smsc_phy_ack_interrupt,
	.config_intr	= smsc_phy_config_intr,

	.suspend	= genphy_suspend,
	.resume		= genphy_resume,
}, {
	.phy_id		= 0x0007c0c0, /* OUI=0x00800f, Model#=0x0c */
	.phy_id_mask	= 0xfffffff0,
	.name		= "SMSC LAN8700",

	.features	= (PHY_BASIC_FEATURES | SUPPORTED_Pause
				| SUPPORTED_Asym_Pause),
	.flags		= PHY_HAS_INTERRUPT | PHY_HAS_MAGICANEG,

	.probe		= smsc_phy_probe,

	/* basic functions */
	.config_aneg	= genphy_config_aneg,
	.read_status	= lan87xx_read_status,
	.config_init	= smsc_phy_config_init,
	.soft_reset	= smsc_phy_reset,

	/* IRQ related */
	.ack_interrupt	= smsc_phy_ack_interrupt,
	.config_intr	= smsc_phy_config_intr,

	.suspend	= genphy_suspend,
	.resume		= genphy_resume,
}, {
	.phy_id		= 0x0007c0d0, /* OUI=0x00800f, Model#=0x0d */
	.phy_id_mask	= 0xfffffff0,
	.name		= "SMSC LAN911x Internal PHY",

	.features	= (PHY_BASIC_FEATURES | SUPPORTED_Pause
				| SUPPORTED_Asym_Pause),
	.flags		= PHY_HAS_INTERRUPT | PHY_HAS_MAGICANEG,

	.probe		= smsc_phy_probe,

	/* basic functions */
	.config_aneg	= genphy_config_aneg,
	.read_status	= genphy_read_status,
	.config_init	= lan911x_config_init,

	/* IRQ related */
	.ack_interrupt	= smsc_phy_ack_interrupt,
	.config_intr	= smsc_phy_config_intr,

	.suspend	= genphy_suspend,
	.resume		= genphy_resume,
}, {
	.phy_id		= 0x0007c0f0, /* OUI=0x00800f, Model#=0x0f */
	.phy_id_mask	= 0xfffffff0,
	.name		= "SMSC LAN8710/LAN8720",

	.features	= (PHY_BASIC_FEATURES | SUPPORTED_Pause
				| SUPPORTED_Asym_Pause),
	.flags		= PHY_HAS_INTERRUPT | PHY_HAS_MAGICANEG,

	.probe		= smsc_phy_probe,

	/* basic functions */
	.config_aneg	= genphy_config_aneg,
	.read_status	= lan87xx_read_status,
	.config_init	= smsc_phy_config_init,
	.soft_reset	= smsc_phy_reset,

	/* IRQ related */
	.ack_interrupt	= smsc_phy_ack_interrupt,
	.config_intr	= smsc_phy_config_intr,

	.remove		= smsc_phy_remove,
	.suspend	= genphy_suspend,
	.resume		= genphy_resume,
}, {
	.phy_id		= 0x0007c110,
	.phy_id_mask	= 0xfffffff0,
	.name		= "SMSC LAN8740",

	.features	= (PHY_BASIC_FEATURES | SUPPORTED_Pause
				| SUPPORTED_Asym_Pause),
	.flags		= PHY_HAS_INTERRUPT | PHY_HAS_MAGICANEG,

	.probe		= smsc_phy_probe,

	/* basic functions */
	.config_aneg	= genphy_config_aneg,
	.read_status	= lan87xx_read_status,
	.config_init	= smsc_phy_config_init,
	.soft_reset	= smsc_phy_reset,

	/* IRQ related */
	.ack_interrupt	= smsc_phy_ack_interrupt,
	.config_intr	= smsc_phy_config_intr,

	.suspend	= genphy_suspend,
	.resume		= genphy_resume,
} };

module_phy_driver(smsc_phy_driver);

MODULE_DESCRIPTION("SMSC PHY driver");
MODULE_AUTHOR("Herbert Valerio Riedel");
MODULE_LICENSE("GPL");

static struct mdio_device_id __maybe_unused smsc_tbl[] = {
	{ 0x0007c0a0, 0xfffffff0 },
	{ 0x0007c0b0, 0xfffffff0 },
	{ 0x0007c0c0, 0xfffffff0 },
	{ 0x0007c0d0, 0xfffffff0 },
	{ 0x0007c0f0, 0xfffffff0 },
	{ 0x0007c110, 0xfffffff0 },
	{ }
};

MODULE_DEVICE_TABLE(mdio, smsc_tbl);
