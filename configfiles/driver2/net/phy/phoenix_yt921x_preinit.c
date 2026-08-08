// SPDX-License-Identifier: GPL-2.0
/*
 * Phoenix RK3588 YT9215S initializer — V0.55
 *
 * Full switch bring-up over SMI (indirect MDIO), modeled on the reference
 * yt9215.c swconfig driver:
 *
 *   1. RESET_CTRL_HW soft-reset (clean state)
 *   2. MIB enable (RMW)
 *   3. EXTIF: enable EXTIF0+EXTIF1, XMII select, RGMII 1G mode
 *      (gives the GMAC its RX_CLK so DMA SWR can pass after driver rebind)
 *   4. VLAN: port-per-VLAN isolation, ports 1-4 -> vlan_base+0..3,
 *      port 8 = tagged trunk uplink (CTAG_MODE_LOOKUP everywhere)
 *   5. PORT_CTRL: MAC RX/TX enable on all used ports
 *
 * After this module runs, userspace must unbind/rebind the GMAC platform
 * driver so stmmac re-probes with RX_CLK present.
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/mdio.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/platform_device.h>

#define PHOENIX_DRV_NAME "phoenix_yt921x_preinit"

#define YT921X_SMI_SWITCHID(id) (((id) & 0x3) << 2)
#define YT921X_SMI_ADDR 0
#define YT921X_SMI_DATA BIT(1)
#define YT921X_SMI_WRITE 0
#define YT921X_SMI_READ BIT(0)

#define YT921X_SMI_PHYADDR 0x1d

/* YT9215S register map (from reference yt9215.c) */
#define RESET_CTRL		0x80000
#define  RESET_CTRL_HW		BIT(31)
#define FUNC_CTRL		0x80004
#define  FUNC_CTRL_MIB		BIT(1)
#define CHIP_ID_REG		0x80008
#define PORT_IGR_LOOKUP_SVLAN	0x80014
#define EXTIF_CTRL		0x80028
#define  EXTIF_CTRL_EXTIF1_EN	BIT(1)
#define  EXTIF_CTRL_EXTIF0_EN	BIT(0)
#define PORT_CTRL(n)		(0x80100 + 4 * (n))
#define  PORT_CTRL_FC_AN	BIT(10)
#define  PORT_CTRL_LINK_AN	BIT(9)
#define  PORT_CTRL_DUPLEX_FULL	BIT(7)
#define  PORT_CTRL_RX_FC	BIT(6)
#define  PORT_CTRL_TX_FC	BIT(5)
#define  PORT_CTRL_RX_MAC_EN	BIT(4)
#define  PORT_CTRL_TX_MAC_EN	BIT(3)
#define  PORT_CTRL_SPEED_1000	2
#define PORT_STATUS(n)		(0x80200 + 4 * (n))
#define EXTIF_SEL		0x80394
#define EXTIF0_MODE		0x80400
#define EXTIF1_MODE		0x80408
#define MIB_CTRL		0xc0004
#define  MIB_CTRL_OP_CLEAN	BIT(30)
#define PORT_EGR_CTRL(n)	(0x100000 + 4 * (n))
#define PORT_EGR_VLAN(n)	(0x100080 + 4 * (n))
#define  EGR_VLAN_STAG_UNTAG	(0u << 27)
#define  EGR_VLAN_CTAG_LOOKUP	(5u << 12)
#define EGR_TPID(n)		(0x100300 + 4 * (n))
#define PORT_IGR_VLAN_FILTER	0x180280
#define PORT_EGR_VLAN_FILTER	0x180598
#define VLAN_CTRL1(v)		(0x188000 + 8 * (v))
#define  VLAN_CTRL1_MEMBER(x)	(((x) & GENMASK(10, 0)) << 7)
#define VLAN_CTRL2(v)		(0x188004 + 8 * (v))
#define  VLAN_CTRL2_UNTAG(x)	(((x) & GENMASK(10, 0)) << 8)
#define IGR_TPID(n)		(0x210000 + 4 * (n))
#define PORT_IGR_TPID(n)	(0x210010 + 4 * (n))
#define PORT_IGR_PVID(n)	(0x230010 + 4 * (n))
#define  IGR_PVID_CVID(v)	(((v) & 0xfff) << 6)

/* EXTIF*_MODE value proven on hardware:
 * RGMII mode(4)<<29 | LINK_UP(bit19) | PORT_EN(bit18) | TXC_DELAY_EN(bit8) */
#define EXTIF_MODE_RGMII_1G	0x800C0100

static bool apply = true;
module_param(apply, bool, 0644);
MODULE_PARM_DESC(apply, "write switch config when probing");

static bool hw_reset = true;
module_param(hw_reset, bool, 0644);
MODULE_PARM_DESC(hw_reset, "soft-reset the switch before configuring");

static int smi_addr = YT921X_SMI_PHYADDR;
module_param(smi_addr, int, 0644);
MODULE_PARM_DESC(smi_addr, "MDIO C22 PHY address used by YT921x SMI");

static bool auto_probe = true;
module_param(auto_probe, bool, 0644);
MODULE_PARM_DESC(auto_probe, "scan SMI switch-id 0..3");

static int vlan_base = 391;
module_param(vlan_base, int, 0644);
MODULE_PARM_DESC(vlan_base, "base VLAN ID for port isolation (port1=base, port2=base+1, ...)");

struct phoenix_yt921x_dev {
	struct device *dev;
	struct mii_bus *bus;
	struct device_node *mdio_np;
	u32 instance_id;
	u32 smi_id;
	u32 mdio_addr;
};

static bool phoenix_yt921x_chip_id_valid(u32 chip_id)
{
	u32 major = chip_id >> 16;
	return chip_id != 0xffffffff && chip_id != 0x00000000 &&
	       (major == 0x9002 || major == 0x9001);
}

static int phoenix_yt921x_read(struct phoenix_yt921x_dev *priv, u32 reg,
			       u32 *val)
{
	struct mii_bus *bus = priv->bus;
	int addr = priv->mdio_addr;
	u32 reg_addr, reg_data;
	int ret;

	mutex_lock_nested(&bus->mdio_lock, MDIO_MUTEX_NESTED);

	reg_addr = YT921X_SMI_SWITCHID(priv->smi_id) | YT921X_SMI_ADDR |
		   YT921X_SMI_READ;
	ret = __mdiobus_write(bus, addr, reg_addr, (u16)(reg >> 16));
	if (ret)
		goto out;
	ret = __mdiobus_write(bus, addr, reg_addr, (u16)reg);
	if (ret)
		goto out;

	reg_data = YT921X_SMI_SWITCHID(priv->smi_id) | YT921X_SMI_DATA |
		   YT921X_SMI_READ;
	ret = __mdiobus_read(bus, addr, reg_data);
	if (ret < 0)
		goto out;
	*val = (u16)ret;

	ret = __mdiobus_read(bus, addr, reg_data);
	if (ret < 0)
		goto out;
	*val = (*val << 16) | (u16)ret;
	ret = 0;

out:
	mutex_unlock(&bus->mdio_lock);
	return ret;
}

static int phoenix_yt921x_write(struct phoenix_yt921x_dev *priv, u32 reg,
				u32 val)
{
	struct mii_bus *bus = priv->bus;
	int addr = priv->mdio_addr;
	u32 reg_addr, reg_data;
	int ret;

	mutex_lock_nested(&bus->mdio_lock, MDIO_MUTEX_NESTED);

	reg_addr = YT921X_SMI_SWITCHID(priv->smi_id) | YT921X_SMI_ADDR |
		   YT921X_SMI_WRITE;
	ret = __mdiobus_write(bus, addr, reg_addr, (u16)(reg >> 16));
	if (ret)
		goto out;
	ret = __mdiobus_write(bus, addr, reg_addr, (u16)reg);
	if (ret)
		goto out;

	reg_data = YT921X_SMI_SWITCHID(priv->smi_id) | YT921X_SMI_DATA |
		   YT921X_SMI_WRITE;
	ret = __mdiobus_write(bus, addr, reg_data, (u16)(val >> 16));
	if (ret)
		goto out;
	ret = __mdiobus_write(bus, addr, reg_data, (u16)val);

out:
	mutex_unlock(&bus->mdio_lock);
	return ret;
}

static int phoenix_yt921x_rmw(struct phoenix_yt921x_dev *priv, u32 reg,
			      u32 mask, u32 set)
{
	u32 v;
	int ret;

	ret = phoenix_yt921x_read(priv, reg, &v);
	if (ret)
		return ret;
	v &= ~mask;
	v |= set;
	return phoenix_yt921x_write(priv, reg, v);
}

static int phoenix_yt921x_hw_reset(struct phoenix_yt921x_dev *priv)
{
	u32 val;
	int i, ret;

	ret = phoenix_yt921x_write(priv, RESET_CTRL, RESET_CTRL_HW);
	if (ret)
		return ret;

	/* poll until reset self-clears (reference: 100ms timeout) */
	for (i = 0; i < 20; i++) {
		usleep_range(10000, 15000);
		ret = phoenix_yt921x_read(priv, RESET_CTRL, &val);
		if (!ret && val == 0)
			break;
	}
	if (i == 20)
		dev_warn(priv->dev, "Phoenix YT921x inst=%u reset timeout\n",
			 priv->instance_id);

	/* reference driver: extra delay after reset (like GPIO hard reset) */
	msleep(10);
	return 0;
}

static int phoenix_yt921x_init_extif(struct phoenix_yt921x_dev *priv)
{
	int ret;
	u32 readback;

	/* EXTIF_SEL = 0x03: both EXTIF0/EXTIF1 in XMII (RGMII) mode */
	ret = phoenix_yt921x_write(priv, EXTIF_SEL, 0x03);
	if (ret)
		return ret;

	/* EXTIF_CTRL: enable EXTIF0 + EXTIF1 (RMW, keep other bits) */
	ret = phoenix_yt921x_rmw(priv, EXTIF_CTRL,
				 EXTIF_CTRL_EXTIF0_EN | EXTIF_CTRL_EXTIF1_EN,
				 EXTIF_CTRL_EXTIF0_EN | EXTIF_CTRL_EXTIF1_EN);
	if (ret)
		return ret;

	/* Both EXTIF mode registers: RGMII 1G, port enable, link up */
	ret = phoenix_yt921x_write(priv, EXTIF0_MODE, EXTIF_MODE_RGMII_1G);
	if (ret)
		return ret;
	ret = phoenix_yt921x_write(priv, EXTIF1_MODE, EXTIF_MODE_RGMII_1G);
	if (ret)
		return ret;

	ret = phoenix_yt921x_read(priv, EXTIF_SEL, &readback);
	if (ret)
		return ret;

	dev_info(priv->dev,
		 "Phoenix YT921x inst=%u EXTIF configured (EXTIF_SEL=0x%08x)\n",
		 priv->instance_id, readback);

	return 0;
}

static int phoenix_yt921x_init_vlan(struct phoenix_yt921x_dev *priv)
{
	int i, ret;
	int vid;
	u32 members, untag;

	/* TPIDs */
	ret = phoenix_yt921x_write(priv, EGR_TPID(0), ETH_P_8021Q);
	if (ret)
		return ret;
	ret = phoenix_yt921x_write(priv, IGR_TPID(0), ETH_P_8021Q);
	if (ret)
		return ret;
	ret = phoenix_yt921x_write(priv, IGR_TPID(1), ETH_P_8021AD);
	if (ret)
		return ret;
	ret = phoenix_yt921x_write(priv, PORT_IGR_LOOKUP_SVLAN, 0);
	if (ret)
		return ret;

	/* VLAN filters: all 11 ports */
	ret = phoenix_yt921x_write(priv, PORT_IGR_VLAN_FILTER, GENMASK(10, 0));
	if (ret)
		return ret;
	ret = phoenix_yt921x_write(priv, PORT_EGR_VLAN_FILTER, GENMASK(10, 0));
	if (ret)
		return ret;

	/*
	 * Port-per-VLAN isolation:
	 *   physical port N (1-4) + uplink ports 8/9 are members of vlan_base+N-1
	 *   untag member = physical port only -> uplink egresses tagged
	 *   (egress tagging via CTAG_MODE_LOOKUP + VLAN_CTRL2 untag bits)
	 *
	 * NOTE: the GMAC RGMII connects to EXTIF1 = switch port 9 (EXTIF1_MODE
	 * at 0x80408 is what provides RX_CLK). Port 8 = EXTIF0. Both are added
	 * as tagged trunk members so the wiring choice doesn't matter.
	 */
	for (i = 0; i < 4; i++) {
		int port = i + 1;
		vid = vlan_base + i;

		members = BIT(port) | BIT(8) | BIT(9);
		untag = BIT(port);

		ret = phoenix_yt921x_write(priv, VLAN_CTRL1(vid),
					   VLAN_CTRL1_MEMBER(members));
		if (ret)
			return ret;
		ret = phoenix_yt921x_write(priv, VLAN_CTRL2(vid),
					   VLAN_CTRL2_UNTAG(untag));
		if (ret)
			return ret;

		dev_info(priv->dev,
			 "Phoenix YT921x VLAN %d: port%d(untag) + port8/9(tag)\n",
			 vid, port);
	}

	/* Per-port egress/ingress config — all 11 ports, like the reference */
	for (i = 0; i < 11; i++) {
		ret = phoenix_yt921x_write(priv, PORT_EGR_CTRL(i), 0);
		if (ret)
			return ret;

		/* STAG untag, CTAG mode = lookup VLAN table untag bits */
		ret = phoenix_yt921x_write(priv, PORT_EGR_VLAN(i),
					   EGR_VLAN_STAG_UNTAG |
					   EGR_VLAN_CTAG_LOOKUP);
		if (ret)
			return ret;

		/* accept both IGR TPIDs as CTAG */
		ret = phoenix_yt921x_write(priv, PORT_IGR_TPID(i), 0x03);
		if (ret)
			return ret;
	}

	/* PVIDs: physical ports 1-4 get their VLAN; others vlan_base */
	for (i = 0; i < 11; i++) {
		if (i >= 1 && i <= 4)
			vid = vlan_base + (i - 1);
		else
			vid = vlan_base;
		ret = phoenix_yt921x_write(priv, PORT_IGR_PVID(i),
					   IGR_PVID_CVID(vid));
		if (ret)
			return ret;
	}

	return 0;
}

static int phoenix_yt921x_init_ports(struct phoenix_yt921x_dev *priv)
{
	int i, ret;
	u32 val;

	/* Ports 8/9 = EXTIF0/EXTIF1 uplinks: fixed 1G FD, MAC RX/TX enable, FC.
	 * GMAC RGMII is on EXTIF1 = port 9; configure both to be safe. */
	val = PORT_CTRL_RX_MAC_EN | PORT_CTRL_TX_MAC_EN |
	      PORT_CTRL_DUPLEX_FULL | PORT_CTRL_RX_FC | PORT_CTRL_TX_FC |
	      PORT_CTRL_SPEED_1000;
	ret = phoenix_yt921x_write(priv, PORT_CTRL(8), val);
	if (ret)
		return ret;
	ret = phoenix_yt921x_write(priv, PORT_CTRL(9), val);
	if (ret)
		return ret;

	/* Ports 0-7 = PHY ports: MAC enable + link/FC autoneg (0x618) */
	val = PORT_CTRL_RX_MAC_EN | PORT_CTRL_TX_MAC_EN |
	      PORT_CTRL_LINK_AN | PORT_CTRL_FC_AN;
	for (i = 0; i < 8; i++) {
		ret = phoenix_yt921x_write(priv, PORT_CTRL(i), val);
		if (ret)
			return ret;
	}

	return 0;
}

static int phoenix_yt921x_init_mib(struct phoenix_yt921x_dev *priv)
{
	int ret;

	/* RMW: only set the MIB enable bit, keep other FUNC_CTRL bits */
	ret = phoenix_yt921x_rmw(priv, FUNC_CTRL, FUNC_CTRL_MIB,
				 FUNC_CTRL_MIB);
	if (ret)
		return ret;

	return phoenix_yt921x_write(priv, MIB_CTRL, MIB_CTRL_OP_CLEAN);
}

static bool phoenix_yt921x_try_smi(struct phoenix_yt921x_dev *priv, u32 smi_id,
				   u32 *chip_id)
{
	u32 old_smi_id = priv->smi_id;
	int ret;

	priv->smi_id = smi_id;
	ret = phoenix_yt921x_read(priv, CHIP_ID_REG, chip_id);
	priv->smi_id = old_smi_id;

	if (ret)
		return false;

	return phoenix_yt921x_chip_id_valid(*chip_id);
}

static bool phoenix_yt921x_select_smi(struct phoenix_yt921x_dev *priv,
				      u32 *chip_id)
{
	u32 configured_smi = priv->smi_id;
	u32 candidate;

	if (phoenix_yt921x_try_smi(priv, configured_smi, chip_id))
		return true;

	if (!auto_probe)
		return false;

	for (candidate = 0; candidate < 4; candidate++) {
		if (candidate == configured_smi)
			continue;
		if (phoenix_yt921x_try_smi(priv, candidate, chip_id)) {
			dev_info(priv->dev,
				 "Phoenix YT921x inst=%u selected smi=%u\n",
				 priv->instance_id, candidate);
			priv->smi_id = candidate;
			return true;
		}
	}

	priv->smi_id = configured_smi;
	return false;
}

static int phoenix_yt921x_probe(struct platform_device *pdev)
{
	struct phoenix_yt921x_dev *priv;
	u32 chip_id, status;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &pdev->dev;
	priv->mdio_addr = smi_addr;
	priv->smi_id = 0;

	ret = of_property_read_u32(pdev->dev.of_node, "motorcomm,id",
				   &priv->instance_id);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "missing motorcomm,id\n");

	of_property_read_u32(pdev->dev.of_node, "motorcomm,mdio-addr",
			     &priv->mdio_addr);
	of_property_read_u32(pdev->dev.of_node, "motorcomm,smi-id",
			     &priv->smi_id);

	priv->mdio_np = of_parse_phandle(pdev->dev.of_node, "motorcomm,mdio",
					 0);
	if (!priv->mdio_np)
		return dev_err_probe(&pdev->dev, -EINVAL,
				     "missing motorcomm,mdio\n");

	priv->bus = of_mdio_find_bus(priv->mdio_np);
	if (!priv->bus) {
		of_node_put(priv->mdio_np);
		return dev_err_probe(&pdev->dev, -EPROBE_DEFER,
				     "MDIO bus not registered yet\n");
	}

	platform_set_drvdata(pdev, priv);

	if (!phoenix_yt921x_select_smi(priv, &chip_id)) {
		dev_warn(priv->dev,
			 "Phoenix YT921x inst=%u no valid chip on bus=%s\n",
			 priv->instance_id, priv->bus->id);
		put_device(&priv->bus->dev);
		of_node_put(priv->mdio_np);
		return 0;
	}

	dev_info(priv->dev, "Phoenix YT921x V0.55 inst=%u smi=%u chip=0x%08x\n",
		 priv->instance_id, priv->smi_id, chip_id);

	if (!apply) {
		dev_info(priv->dev, "Phoenix YT921x inst=%u apply=0, skipping config\n",
			 priv->instance_id);
		return 0;
	}

	/* 1. clean state */
	if (hw_reset) {
		ret = phoenix_yt921x_hw_reset(priv);
		if (ret)
			dev_warn(priv->dev, "reset failed: %d\n", ret);
	}

	/* 2. MIB */
	ret = phoenix_yt921x_init_mib(priv);
	if (ret)
		dev_warn(priv->dev, "MIB init failed: %d\n", ret);

	/* 3. EXTIF (RGMII RX_CLK for GMAC) */
	ret = phoenix_yt921x_init_extif(priv);
	if (ret)
		dev_warn(priv->dev, "EXTIF init failed: %d\n", ret);

	/* 4. VLAN */
	ret = phoenix_yt921x_init_vlan(priv);
	if (ret) {
		dev_err(priv->dev, "VLAN init failed: %d\n", ret);
		goto err;
	}

	/* 5. Port MACs */
	ret = phoenix_yt921x_init_ports(priv);
	if (ret) {
		dev_err(priv->dev, "port init failed: %d\n", ret);
		goto err;
	}

	ret = phoenix_yt921x_read(priv, PORT_STATUS(9), &status);
	if (!ret)
		dev_info(priv->dev,
			 "Phoenix YT921x inst=%u PORT_STATUS(9)=0x%08x (link=%d speed=%d)\n",
			 priv->instance_id, status,
			 !!(status & BIT(8)), status & 0x7);

	dev_info(priv->dev,
		 "Phoenix YT921x V0.55 inst=%u init complete\n",
		 priv->instance_id);
	return 0;

err:
	put_device(&priv->bus->dev);
	of_node_put(priv->mdio_np);
	return ret;
}

static int phoenix_yt921x_remove(struct platform_device *pdev)
{
	struct phoenix_yt921x_dev *priv = platform_get_drvdata(pdev);

	if (priv) {
		if (priv->bus)
			put_device(&priv->bus->dev);
		of_node_put(priv->mdio_np);
	}
	return 0;
}

static const struct of_device_id phoenix_yt921x_of_match[] = {
	{ .compatible = "phoenix,yt9215s-preinit" },
	{ }
};
MODULE_DEVICE_TABLE(of, phoenix_yt921x_of_match);

static struct platform_driver phoenix_yt921x_driver = {
	.probe = phoenix_yt921x_probe,
	.remove = phoenix_yt921x_remove,
	.driver = {
		.name = PHOENIX_DRV_NAME,
		.of_match_table = phoenix_yt921x_of_match,
	},
};
module_platform_driver(phoenix_yt921x_driver);

MODULE_DESCRIPTION("Phoenix RK3588 YT9215S EXTIF+VLAN/port initializer V0.55");
MODULE_AUTHOR("Phoenix");
MODULE_LICENSE("GPL");
