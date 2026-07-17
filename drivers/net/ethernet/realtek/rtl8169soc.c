// SPDX-License-Identifier: GPL-2.0 OR MIT
/* rtl8169soc.c: the embedded Realtek STB SoC 8169soc Ethernet driver.
 *
 * Copyright (c) 2002 Realtek Semiconductor Corp.
 * Copyright (c) 2002 ShuChen <shuchen@realtek.com.tw>
 * Copyright (c) 2003 - 2007 Francois Romieu <romieu@fr.zoreil.com>
 * Copyright (c) 2014 YuKuen Wu <yukuen@realtek.com>
 * Copyright (c) 2015 Eric Wang <ericwang@realtek.com>
 *
 * See MAINTAINERS file for support contact information.
 */

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/phy.h>
#include <linux/if_vlan.h>
#include <linux/crc32.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/bitfield.h>
#include <linux/prefetch.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_net.h>
#include <linux/of_address.h>
#include <linux/nvmem-consumer.h>
#include <linux/suspend.h>
#include <linux/reset.h>
#include <linux/pinctrl/consumer.h>
#include <linux/sys_soc.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <asm/unaligned.h>
#include <soc/realtek/rtk_pm.h>
#include "rtl8169soc_mdns_offload.h"
#include "rtl8169soc.h"

#define RTL8169SOC_VERSION "2.0.10"

#if defined(CONFIG_RTL_RX_NO_COPY)
static int rx_buf_sz = RX_BUF_SIZE;	/* 0x05F3 = 1522bye + 1 */
static int rx_buf_sz_new = RX_BUF_SIZE;
#else
static int rx_buf_sz = RX_BUF_SIZE;
#endif /* CONFIG_RTL_RX_NO_COPY */

#ifdef RTL_PROC
static struct proc_dir_entry *rtk_proc;

static int rtl_proc_file_register(struct rtl8169_private *tp);
static void rtl_proc_file_unregister(struct rtl8169_private *tp);
#endif

MODULE_AUTHOR("Eric Wang <ericwang@realtek.com>");
MODULE_DESCRIPTION("RealTek STB SoC r8169soc Ethernet driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(RTL8169SOC_VERSION);

static inline struct device *tp_to_dev(struct rtl8169_private *tp)
{
	return &tp->pdev->dev;
}

static inline void rtl_lock_config_regs(struct rtl8169_private *tp)
{
	RTL_W8(tp, CFG9346, CFG9346_LOCK);
}

static inline void rtl_unlock_config_regs(struct rtl8169_private *tp)
{
	RTL_W8(tp, CFG9346, CFG9346_UNLOCK);
}

static void rtl_lock_work(struct rtl8169_private *tp)
{
	mutex_lock(&tp->wk.mutex);
}

static void rtl_unlock_work(struct rtl8169_private *tp)
{
	mutex_unlock(&tp->wk.mutex);
}

struct rtl_cond {
	bool (*check)(struct rtl8169_private *tp);
	const char *msg;
};

static bool rtl_loop_wait(struct rtl8169_private *tp, const struct rtl_cond *c,
			  unsigned long usecs, int n, bool high)
{
	int i;

	for (i = 0; i < n; i++) {
		if (c->check(tp) == high)
			return true;
		fsleep(usecs);
	}

	if (net_ratelimit())
		netdev_err(tp->dev, "%s == %d (loop: %d, delay: %lu).\n",
			   c->msg, !high, n, usecs);
	return false;
}

static bool rtl_loop_wait_high(struct rtl8169_private *tp,
			       const struct rtl_cond *c,
			       unsigned long d, int n)
{
	return rtl_loop_wait(tp, c, d, n, true);
}

static bool rtl_loop_wait_low(struct rtl8169_private *tp,
			      const struct rtl_cond *c,
			      unsigned long d, int n)
{
	return rtl_loop_wait(tp, c, d, n, false);
}

#define DECLARE_RTL_COND(name)				\
static bool name ## _check(struct rtl8169_private *);	\
							\
static const struct rtl_cond name = {			\
	.check	= name ## _check,			\
	.msg	= #name					\
};							\
							\
static bool name ## _check(struct rtl8169_private *tp)

DECLARE_RTL_COND(rtl_eriar_cond)
{
	return RTL_R32(tp, ERIAR) & ERIAR_FLAG;
}

DECLARE_RTL_COND(rtl_link_list_ready_cond)
{
	return RTL_R8(tp, MCU) & LINK_LIST_RDY;
}

static bool rtl_ocp_reg_failure(struct rtl8169_private *tp, u32 reg)
{
	if (reg & 0xffff0001) {
		if (net_ratelimit())
			netdev_err(tp->dev, "Invalid ocp reg %x!\n", reg);
		return true;
	}
	return false;
}

static void rtl_ocp_write(struct rtl8169_private *tp, u32 reg, u32 value)
{
	unsigned int wdata;

	if (rtl_ocp_reg_failure(tp, reg))
		return;

	wdata = OCPDR_WRITE_CMD |
		(((reg >> 1) & OCPDR_REG_MASK) << OCPDR_REG_SHIFT) |
		(value & OCPDR_DATA_MASK);
	RTL_W32(tp, OCPDR, wdata);
}

static u32 rtl_ocp_read(struct rtl8169_private *tp, u32 reg)
{
	unsigned int wdata;
	unsigned int rdata;

	if (rtl_ocp_reg_failure(tp, reg))
		return 0;

	wdata = OCPDR_READ_CMD |
		(((reg >> 1) & OCPDR_REG_MASK) << OCPDR_REG_SHIFT);
	RTL_W32(tp, OCPDR, wdata);
	rdata = RTL_R32(tp, OCPDR);
	return (rdata & OCPDR_DATA_MASK);
}

DECLARE_RTL_COND(rtl_int_phyar_cond)
{
	return rtl_ocp_read(tp, 0xDE0A) & BIT(14);
}

DECLARE_RTL_COND(rtl_phyar_cond)
{
	return RTL_R32(tp, PHYAR) & 0x80000000;
}

static void __int_set_phy_addr(struct rtl8169_private *tp, int phy_addr)
{
	regmap_update_bits_base(tp->iso_base, ISO_PWRCUT_ETN,
				GENMASK(20, 16), (phy_addr << 16),
				NULL, false, true);
}

static inline int __int_mdio_read(struct rtl8169_private *tp, int reg)
{
	int value;

	/* read reg */
	RTL_W32(tp, PHYAR, 0x0 | (reg & 0x1f) << 16);

	value = rtl_loop_wait_high(tp, &rtl_phyar_cond, 25, 20) ?
		RTL_R32(tp, PHYAR) & 0xffff : ~0;

	/* According to hardware specs a 20us delay is required after read
	 * complete indication, but before sending next command.
	 */
	fsleep(20);

	return value;
}

static inline void __int_mdio_write(struct rtl8169_private *tp, int reg, int value)
{
	/* write reg */
	RTL_W32(tp, PHYAR, 0x80000000 | (reg & 0x1f) << 16 | (value & 0xffff));

	rtl_loop_wait_low(tp, &rtl_phyar_cond, 25, 20);
	/* According to hardware specs a 20us delay is required after write
	 * complete indication, but before sending next command.
	 */
	fsleep(20);
}

static int int_mdio_read(struct rtl8169_private *tp, int page, int reg)
{
	int value;

	tp->chip->mdio_lock(tp);

	/* write page */
	if (page != CURRENT_MDIO_PAGE)
		__int_mdio_write(tp, 0x1f, page);

	/* read reg */
	value = __int_mdio_read(tp, reg);

	tp->chip->mdio_unlock(tp);
	return value;
}

static void int_mdio_write(struct rtl8169_private *tp, int page, int reg, int value)
{
	tp->chip->mdio_lock(tp);

	/* write page */
	if (page != CURRENT_MDIO_PAGE)
		__int_mdio_write(tp, 0x1f, page);

	/* write reg */
	__int_mdio_write(tp, reg, value);

	tp->chip->mdio_unlock(tp);
}

static int int_ocp_mdio_read(struct rtl8169_private *tp, int page, int reg)
{
	int value;

	if (page != CURRENT_MDIO_PAGE)
		rtl_ocp_write(tp, 0xDE0C, page);
	rtl_ocp_write(tp, 0xDE0A, reg & 0x1f);

	value = rtl_loop_wait_low(tp, &rtl_int_phyar_cond, 25, 20) ?
		rtl_ocp_read(tp, 0xDE08) & 0xffff : ~0;

	return value;
}

static void int_ocp_mdio_write(struct rtl8169_private *tp, int page, int reg, int value)
{
	if (page != CURRENT_MDIO_PAGE)
		rtl_ocp_write(tp, 0xDE0C, page);
	rtl_ocp_write(tp, 0xDE08, value);
	rtl_ocp_write(tp, 0xDE0A, BIT(15) | (reg & 0x1f));

	rtl_loop_wait_low(tp, &rtl_int_phyar_cond, 25, 20);
}

DECLARE_RTL_COND(rtl_ext_phyar_cond)
{
	return rtl_ocp_read(tp, 0xDE2A) & BIT(14);
}

DECLARE_RTL_COND(rtl_ephyar_cond)
{
	return RTL_R32(tp, EPHYAR) & EPHYAR_FLAG;
}

static void __ext_set_phy_addr(struct rtl8169_private *tp, int phy_addr)
{
	regmap_update_bits_base(tp->iso_base, ISO_PWRCUT_ETN,
				GENMASK(25, 21), (phy_addr << 21),
				NULL, false, true);
}

static inline int __ext_mdio_read(struct rtl8169_private *tp, int reg)
{
	int value;

	/* read reg */
	RTL_W32(tp, EPHYAR, (reg & EPHYAR_REG_MASK) << EPHYAR_REG_SHIFT);

	value = rtl_loop_wait_high(tp, &rtl_ephyar_cond, 10, 100) ?
		RTL_R32(tp, EPHYAR) & EPHYAR_DATA_MASK : ~0;

	return value;
}

static inline void __ext_mdio_write(struct rtl8169_private *tp, int reg, int value)
{
	/* write reg */
	RTL_W32(tp, EPHYAR, EPHYAR_WRITE_CMD | (value & EPHYAR_DATA_MASK) |
		(reg & EPHYAR_REG_MASK) << EPHYAR_REG_SHIFT);

	rtl_loop_wait_low(tp, &rtl_ephyar_cond, 10, 100);

	fsleep(10);
}

static int ext_mdio_read(struct rtl8169_private *tp, int page, int reg)
{
	int value;

	tp->chip->mdio_lock(tp);

	/* write page */
	if (page != CURRENT_MDIO_PAGE)
		__ext_mdio_write(tp, 0x1f, page);

	/* read reg */
	value = __ext_mdio_read(tp, reg);

	tp->chip->mdio_unlock(tp);
	return value;
}

static void ext_mdio_write(struct rtl8169_private *tp, int page, int reg, int value)
{
	tp->chip->mdio_lock(tp);

	/* write page */
	if (page != CURRENT_MDIO_PAGE)
		__ext_mdio_write(tp, 0x1f, page);

	/* write reg */
	__ext_mdio_write(tp, reg, value);

	tp->chip->mdio_unlock(tp);
}

static int ext_ocp_mdio_read(struct rtl8169_private *tp, int page, int reg)
{
	int value;

	if (page != CURRENT_MDIO_PAGE)
		rtl_ocp_write(tp, 0xDE2C, page);
	rtl_ocp_write(tp, 0xDE2A, reg & 0x1f);

	value = rtl_loop_wait_low(tp, &rtl_ext_phyar_cond, 25, 20) ?
		rtl_ocp_read(tp, 0xDE28) & 0xffff : ~0;
	return value;
}

static void ext_ocp_mdio_write(struct rtl8169_private *tp, int page, int reg, int value)
{
	if (page != CURRENT_MDIO_PAGE)
		rtl_ocp_write(tp, 0xDE2C, page);
	rtl_ocp_write(tp, 0xDE28, value);
	rtl_ocp_write(tp, 0xDE2A, BIT(15) | (reg & 0x1f));

	rtl_loop_wait_low(tp, &rtl_ext_phyar_cond, 25, 20);
}

static void dummy_mdio_lock(struct rtl8169_private *tp)
{
	/* no lock */
}

static void dummy_mdio_unlock(struct rtl8169_private *tp)
{
	/* no unlock */
}

static void rtl_init_mdio_ops(struct rtl8169_private *tp)
{
	if (tp->output_mode == OUTPUT_EMBEDDED_PHY) {
		if (tp->chip->features & RTL_FEATURE_OCP_MDIO) {
			tp->chip->mdio_write = int_ocp_mdio_write;
			tp->chip->mdio_read = int_ocp_mdio_read;
			tp->chip->mdio_lock = dummy_mdio_lock;
			tp->chip->mdio_unlock = dummy_mdio_unlock;
		} else {
			tp->chip->mdio_write = int_mdio_write;
			tp->chip->mdio_read = int_mdio_read;
		}
	} else {
		if (tp->chip->features & RTL_FEATURE_OCP_MDIO) {
			tp->chip->mdio_write = ext_ocp_mdio_write;
			tp->chip->mdio_read = ext_ocp_mdio_read;
			tp->chip->mdio_lock = dummy_mdio_lock;
			tp->chip->mdio_unlock = dummy_mdio_unlock;
		} else {
			tp->chip->mdio_write = ext_mdio_write;
			tp->chip->mdio_read = ext_mdio_read;
		}
	}
}

static inline void rtl_phy_write(struct rtl8169_private *tp, int page, int reg, u32 val)
{
	tp->chip->mdio_write(tp, page, reg, val);
}

static inline int rtl_phy_read(struct rtl8169_private *tp, int page, int reg)
{
	return tp->chip->mdio_read(tp, page, reg);
}

static inline void rtl_patchphy(struct rtl8169_private *tp, int page,
				int reg_addr, int value)
{
	rtl_phy_write(tp, page, reg_addr,
		      rtl_phy_read(tp, page, reg_addr) | value);
}

static inline void rtl_w1w0_phy(struct rtl8169_private *tp, int page,
				int reg_addr, int p, int m)
{
	rtl_phy_write(tp, page, reg_addr,
		      (rtl_phy_read(tp, page, reg_addr) | p) & ~m);
}

static void int_mmd_write(struct rtl8169_private *tp, u32 dev_addr, u32 reg_addr, u32 value)
{
	tp->chip->mdio_lock(tp);

	/* set page 0 */
	__int_mdio_write(tp, 0x1f, 0);
	/* address mode */
	__int_mdio_write(tp, 13, (0x1f & dev_addr));
	/* write the desired address */
	__int_mdio_write(tp, 14, reg_addr);
	/* data mode, no post increment */
	__int_mdio_write(tp, 13, ((0x1f & dev_addr) | (0x1 << 14)));
	/* write the content of the selected register */
	__int_mdio_write(tp, 14, value);

	tp->chip->mdio_unlock(tp);
}

static u32 int_mmd_read(struct rtl8169_private *tp, u32 dev_addr, u32 reg_addr)
{
	u32 value;

	tp->chip->mdio_lock(tp);

	/* set page 0 */
	__int_mdio_write(tp, 0x1f, 0);
	/* address mode */
	__int_mdio_write(tp, 13, (0x1f & dev_addr));
	/* write the desired address */
	__int_mdio_write(tp, 14, reg_addr);
	/* data mode, no post increment */
	__int_mdio_write(tp, 13, ((0x1f & dev_addr) | (0x1 << 14)));
	/* read the content of the selected register */
	value = __int_mdio_read(tp, 14);

	tp->chip->mdio_unlock(tp);
	return value;
}

static void ext_mmd_write(struct rtl8169_private *tp, u32 dev_addr, u32 reg_addr, u32 value)
{
	tp->chip->mdio_lock(tp);

	/* set page 0 */
	__ext_mdio_write(tp, 0x1f, 0);
	/* address mode */
	__ext_mdio_write(tp, 13, (0x1f & dev_addr));
	/* write the desired address */
	__ext_mdio_write(tp, 14, reg_addr);
	/* data mode, no post increment */
	__ext_mdio_write(tp, 13, ((0x1f & dev_addr) | (0x1 << 14)));
	/* write the content of the selected register */
	__ext_mdio_write(tp, 14, value);

	tp->chip->mdio_unlock(tp);
}

static u32 ext_mmd_read(struct rtl8169_private *tp, u32 dev_addr, u32 reg_addr)
{
	u32 value;

	tp->chip->mdio_lock(tp);

	/* set page 0 */
	__ext_mdio_write(tp, 0x1f, 0);
	/* address mode */
	__ext_mdio_write(tp, 13, (0x1f & dev_addr));
	/* write the desired address */
	__ext_mdio_write(tp, 14, reg_addr);
	/* data mode, no post increment */
	__ext_mdio_write(tp, 13, ((0x1f & dev_addr) | (0x1 << 14)));
	/* read the content of the selected register */
	value = __ext_mdio_read(tp, 14);

	tp->chip->mdio_unlock(tp);
	return value;
}

static void int_ocp_mmd_write(struct rtl8169_private *tp, u32 dev_addr, u32 reg_addr, u32 value)
{
	/* address mode */
	int_ocp_mdio_write(tp, 0, 13, (0x1f & dev_addr));
	/* write the desired address */
	int_ocp_mdio_write(tp, 0, 14, reg_addr);
	/* data mode, no post increment */
	int_ocp_mdio_write(tp, 0, 13, ((0x1f & dev_addr) | (0x1 << 14)));
	/* write the content of the selected register */
	int_ocp_mdio_write(tp, 0, 14, value);
}

static u32 int_ocp_mmd_read(struct rtl8169_private *tp, u32 dev_addr, u32 reg_addr)
{
	/* address mode */
	int_ocp_mdio_write(tp, 0, 13, (0x1f & dev_addr));
	/* write the desired address */
	int_ocp_mdio_write(tp, 0, 14, reg_addr);
	/* data mode, no post increment */
	int_ocp_mdio_write(tp, 0, 13, ((0x1f & dev_addr) | (0x1 << 14)));
	/* read the content of the selected register */
	return int_ocp_mdio_read(tp, 0, 14);
}

static void ext_ocp_mmd_write(struct rtl8169_private *tp, u32 dev_addr, u32 reg_addr, u32 value)
{
	/* address mode */
	ext_ocp_mdio_write(tp, 0, 13, (0x1f & dev_addr));
	/* write the desired address */
	ext_ocp_mdio_write(tp, 0, 14, reg_addr);
	/* data mode, no post increment */
	ext_ocp_mdio_write(tp, 0, 13, ((0x1f & dev_addr) | (0x1 << 14)));
	/* write the content of the selected register */
	ext_ocp_mdio_write(tp, 0, 14, value);
}

static u32 ext_ocp_mmd_read(struct rtl8169_private *tp, u32 dev_addr, u32 reg_addr)
{
	/* address mode */
	ext_ocp_mdio_write(tp, 0, 13, (0x1f & dev_addr));
	/* write the desired address */
	ext_ocp_mdio_write(tp, 0, 14, reg_addr);
	/* data mode, no post increment */
	ext_ocp_mdio_write(tp, 0, 13, ((0x1f & dev_addr) | (0x1 << 14)));
	/* read the content of the selected register */
	return ext_ocp_mdio_read(tp, 0, 14);
}

static void rtl_init_mmd_ops(struct rtl8169_private *tp)
{
	if (tp->output_mode == OUTPUT_EMBEDDED_PHY) {
		if (tp->chip->features & RTL_FEATURE_OCP_MDIO) {
			tp->chip->mmd_write = int_ocp_mmd_write;
			tp->chip->mmd_read = int_ocp_mmd_read;
		} else {
			tp->chip->mmd_write = int_mmd_write;
			tp->chip->mmd_read = int_mmd_read;
		}
	} else {
		if (tp->chip->features & RTL_FEATURE_OCP_MDIO) {
			tp->chip->mmd_write = ext_ocp_mmd_write;
			tp->chip->mmd_read = ext_ocp_mmd_read;
		} else {
			tp->chip->mmd_write = ext_mmd_write;
			tp->chip->mmd_read = ext_mmd_read;
		}
	}
}

static void rtl_mmd_write(struct rtl8169_private *tp, u32 dev_addr, u32 reg_addr, u32 value)
{
	tp->chip->mmd_write(tp, dev_addr, reg_addr, value);
}

static u32 rtl_mmd_read(struct rtl8169_private *tp, u32 dev_addr, u32 reg_addr)
{
	return tp->chip->mmd_read(tp, dev_addr, reg_addr);
}

static void __rtl_eri_write(struct rtl8169_private *tp, int addr, u32 mask, u32 val, int type)
{
	WARN_ON((addr & 3) || (mask == 0));
	RTL_W32(tp, ERIDR, val);
	RTL_W32(tp, ERIAR, ERIAR_WRITE_CMD | type | mask | addr);

	rtl_loop_wait_low(tp, &rtl_eriar_cond, 100, 100);
}

static void rtl_eri_write(struct rtl8169_private *tp, int addr, u32 mask, u32 val)
{
	__rtl_eri_write(tp, addr, mask, val, ERIAR_EXGMAC);
}

static u32 __rtl_eri_read(struct rtl8169_private *tp, int addr, int type)
{
	RTL_W32(tp, ERIAR, ERIAR_READ_CMD | type | ERIAR_MASK_1111 | addr);

	return rtl_loop_wait_high(tp, &rtl_eriar_cond, 100, 100) ?
		RTL_R32(tp, ERIDR) : ~0;
}

static u32 rtl_eri_read(struct rtl8169_private *tp, int addr)
{
	return __rtl_eri_read(tp, addr, ERIAR_EXGMAC);
}

static void rtl_w0w1_eri(struct rtl8169_private *tp, int addr, u32 p, u32 m)
{
	u32 val = rtl_eri_read(tp, addr);

	rtl_eri_write(tp, addr, ERIAR_MASK_1111, (val & ~m) | p);
}

static void rtl_eri_set_bits(struct rtl8169_private *tp, int addr, u32 p)
{
	rtl_w0w1_eri(tp, addr, p, 0);
}

static void rtl_eri_clear_bits(struct rtl8169_private *tp, int addr, u32 m)
{
	rtl_w0w1_eri(tp, addr, 0, m);
}

static void rtl_reset_packet_filter(struct rtl8169_private *tp)
{
	rtl_eri_clear_bits(tp, 0xdc, BIT(0));
	rtl_eri_set_bits(tp, 0xdc, BIT(0));
}

static u16 rtl_get_events(struct rtl8169_private *tp)
{
	return RTL_R16(tp, INTR_STATUS);
}

static void rtl_ack_events(struct rtl8169_private *tp, u16 bits)
{
	RTL_W16(tp, INTR_STATUS, bits);
}

static void rtl_irq_disable(struct rtl8169_private *tp)
{
	RTL_W16(tp, INTR_MASK, 0);
}

static void rtl_irq_enable(struct rtl8169_private *tp)
{
	RTL_W16(tp, INTR_MASK, tp->irq_mask);
}

static void rtl8169_irq_mask_and_ack(struct rtl8169_private *tp)
{
	rtl_irq_disable(tp);
	rtl_ack_events(tp, 0xffff);
	RTL_R8(tp, CHIP_CMD);
}

static unsigned int rtl8169_xmii_link_ok(struct rtl8169_private *tp)
{
	return RTL_R8(tp, PHY_STATUS) & LINK_STATUS;
}

static unsigned int rtl8169_xmii_always_link_ok(struct rtl8169_private *tp)
{
	return true;
}

static __maybe_unused void
r8169_display_eee_info(struct net_device *dev, struct seq_file *m,
		       struct rtl8169_private *tp)
{
	/* display DUT and link partner EEE capability */
	unsigned int temp, tmp1, tmp2;
	int speed = 0;
	bool eee1000, eee100, eee10;
	int duplex;

	temp = rtl_phy_read(tp, 0x0a43, 26);
	if (0 == ((0x1 << 2) & temp)) {
		seq_printf(m, "%s: link is down\n", dev->name);
		return;
	}

	if ((0x1 << 3) & temp)
		duplex = 1;
	else
		duplex = 0;

	if ((0x0 << 4) == ((0x3 << 4) & temp)) {
		speed = 10;

		tmp1 = rtl_phy_read(tp, 0x0bcd, 19);

		tmp2 = rtl_phy_read(tp, 0xa60, 16);

		if (0 == ((0x1 << 4) & tmp1))
			eee10 = false;
		else
			eee10 = true;

		seq_printf(m, "%s: link speed = %dM %s, EEE = %s, PCS_Status = 0x%02x\n",
			   dev->name, speed,
			   duplex ? "full" : "half",
			   eee10 ? "Y" : "N",
			   tmp2 & 0xff);
		return;
	}
	if ((0x1 << 4) == ((0x3 << 4) & temp))
		speed = 100;
	if ((0x2 << 4) == ((0x3 << 4) & temp))
		speed = 1000;
	if ((0x1 << 8) == ((0x1 << 8) & temp)) {
		seq_printf(m, "%s: link speed = %dM %s, EEE = Y\n", dev->name,
			   speed, duplex ? "full" : "half");

		tmp1 = rtl_phy_read(tp, 0xa60, 16);

		tmp2 = rtl_phy_read(tp, 0xa5c, 19);
		seq_printf(m, "PCS_Status = 0x%02x, EEE_wake_error = 0x%04x\n",
			   tmp1 & 0xff, tmp2);
	} else {
		seq_printf(m, "%s: link speed = %dM %s, EEE = N\n", dev->name,
			   speed, duplex ? "full" : "half");

		temp = rtl_mmd_read(tp, 0x7, 0x3d);
		if (0 == (temp & (0x1 << 2)))
			eee1000 = false;
		else
			eee1000 = true;

		if (0 == (temp & (0x1 << 1)))
			eee100 = false;
		else
			eee100 = true;

		seq_printf(m, "%s: Link Partner EEE1000=%s, EEE100=%s\n",
			   dev->name, eee1000 ? "Y" : "N", eee100 ? "Y" : "N");
	}
}

#define WAKE_ANY (WAKE_PHY | WAKE_MAGIC | WAKE_UCAST | WAKE_BCAST | WAKE_MCAST)

static void rtl8169_get_wol(struct net_device *dev, struct ethtool_wolinfo *wol)
{
	struct rtl8169_private *tp = netdev_priv(dev);

	wol->supported = WAKE_ANY;
	wol->wolopts = tp->saved_wolopts;
}

static void __rtl8169_set_wol(struct rtl8169_private *tp, u32 wolopts)
{
	if (wolopts & WAKE_MAGIC)
		tp->wol_enable |= WOL_MAGIC;
	else
		tp->wol_enable &= ~WOL_MAGIC;
	tp->dev->wol_enabled = (tp->wol_enable & WOL_MAGIC) ? 1 : 0;
	device_set_wakeup_enable(tp_to_dev(tp), tp->dev->wol_enabled);
}

static int rtl8169_set_wol(struct net_device *dev, struct ethtool_wolinfo *wol)
{
	struct rtl8169_private *tp = netdev_priv(dev);

	if (wol->wolopts & ~WAKE_ANY)
		return -EINVAL;

	rtl_lock_work(tp);

	tp->saved_wolopts = wol->wolopts;
	__rtl8169_set_wol(tp, tp->saved_wolopts);

	rtl_unlock_work(tp);

	return 0;
}

static void rtl8169_get_drvinfo(struct net_device *dev,
				struct ethtool_drvinfo *info)
{
	strscpy(info->driver, KBUILD_MODNAME, sizeof(info->driver));
	strscpy(info->version, RTL8169SOC_VERSION, sizeof(info->version));
	strscpy(info->bus_info, "RTK-ETN", sizeof(info->bus_info));
}

static int rtl8169_get_regs_len(struct net_device *dev)
{
	return R8169_REGS_SIZE;
}

static netdev_features_t rtl8169_fix_features(struct net_device *dev,
					      netdev_features_t features)
{
	struct rtl8169_private *tp = netdev_priv(dev);

	if (dev->mtu > TD_MSS_MAX)
		features &= ~NETIF_F_ALL_TSO;

	if (dev->mtu > JUMBO_1K && !tp->chip->jumbo_tx_csum)
		features &= ~NETIF_F_IP_CSUM;

	return features;
}

static void rtl_set_rx_config_features(struct rtl8169_private *tp,
				       netdev_features_t features)
{
	u32 rx_config = RTL_R32(tp, RX_CONFIG);

	if (features & NETIF_F_RXALL)
		rx_config |= RX_CONFIG_ACCEPT_ERR_MASK;
	else
		rx_config &= ~RX_CONFIG_ACCEPT_ERR_MASK;

	RTL_W32(tp, RX_CONFIG, rx_config);
}

static void __rtl8169_set_features(struct rtl8169_private *tp, netdev_features_t features)
{
	rtl_set_rx_config_features(tp, features);

	if (features & NETIF_F_RXCSUM)
		tp->cp_cmd |= RX_CHK_SUM;
	else
		tp->cp_cmd &= ~RX_CHK_SUM;

	if (features & NETIF_F_HW_VLAN_CTAG_RX)
		tp->cp_cmd |= RX_VLAN;
	else
		tp->cp_cmd &= ~RX_VLAN;

	RTL_W16(tp, C_PLUS_CMD, tp->cp_cmd);
	RTL_R16(tp, C_PLUS_CMD);
}

static int rtl8169_set_features(struct net_device *dev,
				netdev_features_t features)
{
	struct rtl8169_private *tp = netdev_priv(dev);

	rtl_lock_work(tp);
	__rtl8169_set_features(tp, features);
	rtl_unlock_work(tp);

	return 0;
}

static inline u32 rtl8169_tx_vlan_tag(struct sk_buff *skb)
{
	return (skb_vlan_tag_present(skb)) ?
		TX_VLAN_TAG | swab16(skb_vlan_tag_get(skb)) : 0x00;
}

static void rtl8169_rx_vlan_tag(struct rx_desc *desc, struct sk_buff *skb)
{
	u32 opts2 = le32_to_cpu(desc->opts2);

	if (opts2 & RX_VLAN_TAG)
		__vlan_hwaccel_put_tag(skb, htons(ETH_P_8021Q),
				       swab16(opts2 & 0xffff));
}

static void rtl8169_get_regs(struct net_device *dev, struct ethtool_regs *regs,
			     void *p)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	u32 __iomem *data = tp->mmio_addr;
	u32 *dw = p;
	int i;

	for (i = 0; i < R8169_REGS_SIZE; i += 4)
		memcpy_fromio(dw++, data++, 4);
}

static const char rtl8169_gstrings[][ETH_GSTRING_LEN] = {
	"tx_packets",
	"rx_packets",
	"tx_errors",
	"rx_errors",
	"rx_missed",
	"align_errors",
	"tx_single_collisions",
	"tx_multi_collisions",
	"unicast",
	"broadcast",
	"multicast",
	"tx_aborted",
	"tx_underrun",
};

static int rtl8169_get_sset_count(struct net_device *dev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return ARRAY_SIZE(rtl8169_gstrings);
	default:
		return -EOPNOTSUPP;
	}
}

DECLARE_RTL_COND(rtl_counters_cond)
{
	return RTL_R32(tp, COUNTER_ADDR_LOW) & (COUNTER_RESET | COUNTER_DUMP);
}

static void rtl8169_do_counters(struct rtl8169_private *tp, u32 counter_cmd)
{
	u32 cmd = lower_32_bits(tp->counters_phys_addr);

	RTL_W32(tp, COUNTER_ADDR_HIGH, upper_32_bits(tp->counters_phys_addr));
	RTL_R8(tp, CHIP_CMD);
	RTL_W32(tp, COUNTER_ADDR_LOW, cmd);
	RTL_W32(tp, COUNTER_ADDR_LOW, cmd | counter_cmd);

	rtl_loop_wait_low(tp, &rtl_counters_cond, 10, 1000);
}

static void rtl8169_update_counters(struct rtl8169_private *tp)
{
	/* Some chips are unable to dump tally counters when the receiver
	 * is disabled.
	 */
	if (RTL_R8(tp, CHIP_CMD) & CMD_RX_ENB)
		rtl8169_do_counters(tp, COUNTER_DUMP);
}

static void rtl8169_init_counter_offsets(struct rtl8169_private *tp)
{
	if (tp->tc_inited)
		return;

	rtl8169_do_counters(tp, COUNTER_RESET);

	tp->tc_inited = true;
}

static void rtl8169_get_ethtool_stats(struct net_device *dev,
				      struct ethtool_stats *stats, u64 *data)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	struct rtl8169_counters *counters;

	counters = tp->counters;
	rtl8169_update_counters(tp);

	data[0] = le64_to_cpu(counters->tx_packets);
	data[1] = le64_to_cpu(counters->rx_packets);
	data[2] = le64_to_cpu(counters->tx_errors);
	data[3] = le32_to_cpu(counters->rx_errors);
	data[4] = le16_to_cpu(counters->rx_missed);
	data[5] = le16_to_cpu(counters->align_errors);
	data[6] = le32_to_cpu(counters->tx_one_collision);
	data[7] = le32_to_cpu(counters->tx_multi_collision);
	data[8] = le64_to_cpu(counters->rx_unicast);
	data[9] = le64_to_cpu(counters->rx_broadcast);
	data[10] = le32_to_cpu(counters->rx_multicast);
	data[11] = le16_to_cpu(counters->tx_aborted);
	data[12] = le16_to_cpu(counters->tx_underrun);
}

static void rtl8169_get_strings(struct net_device *dev, u32 stringset, u8 *data)
{
	switch (stringset) {
	case ETH_SS_STATS:
		memcpy(data, rtl8169_gstrings, sizeof(rtl8169_gstrings));
		break;
	}
}

/* Interrupt coalescing
 *
 * > 1 - the availability of the IntrMitigate (0xe2) register *
 * > 2 - the Tx timer unit at gigabit speed
 *
 * The unit of the timer depends on both the speed and the setting of CPlusCmd
 * (0xe0) bit 1 and bit 0.
 *
 * bit[1:0] \ speed        1000M           100M            10M
 * 0 0                     5us             2.56us          40.96us
 * 0 1                     40us            20.48us         327.7us
 * 1 0                     80us            40.96us         655.4us
 * 1 1                     160us           81.92us         1.31ms
 */

/* rx/tx scale factors for all CPlusCmd[0:1] cases */
struct rtl_coalesce_info {
	u32 speed;
	u32 nsecs;
};

/* produce array with base delay *1, *8, *8*2, *8*2*2 */
u8 scale_factor[INTT_MASK + 1] = { 1, 8, 16, 32 };

static const struct rtl_coalesce_info rtl_coalesce_delay_info[] = {
	{ SPEED_1000,	5000 },
	{ SPEED_100,	2560 },
	{ SPEED_10,	40960 },
	{ 0 },
};

#undef COALESCE_DELAY

/* get rx/tx scale vector corresponding to current speed */
static const struct rtl_coalesce_info *
rtl_coalesce_info(struct rtl8169_private *tp)
{
	const struct rtl_coalesce_info *ci = rtl_coalesce_delay_info;

	/* if speed is unknown assume highest one */
	if (tp->phydev->speed == SPEED_UNKNOWN)
		return ci;

	for (; ci->speed; ci++) {
		if (tp->phydev->speed == ci->speed)
			return ci;
	}

	return ERR_PTR(-ELNRNG);
}

static int rtl_get_coalesce(struct net_device *dev,
			    struct ethtool_coalesce *ec,
			    struct kernel_ethtool_coalesce *kernel_coal,
			    struct netlink_ext_ack *extack)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	const struct rtl_coalesce_info *ci;
	u32 scale, c_us, c_fr;
	u16 intrmit;

	memset(ec, 0, sizeof(*ec));

	/* get rx/tx scale corresponding to current speed and CPlusCmd[0:1] */
	ci = rtl_coalesce_info(tp);
	if (IS_ERR(ci))
		return PTR_ERR(ci);

	scale = ci->nsecs * scale_factor[tp->cp_cmd & INTT_MASK];

	intrmit = RTL_R16(tp, INTR_MITIGATE);

	c_us = FIELD_GET(RTL_COALESCE_TX_USECS, intrmit);
	ec->tx_coalesce_usecs = DIV_ROUND_UP(c_us * scale, 1000);

	c_fr = FIELD_GET(RTL_COALESCE_TX_FRAMES, intrmit);
	/* ethtool_coalesce states usecs and max_frames must not both be 0 */
	ec->tx_max_coalesced_frames = (c_us || c_fr) ? c_fr * 4 : 1;

	c_us = FIELD_GET(RTL_COALESCE_RX_USECS, intrmit);
	ec->rx_coalesce_usecs = DIV_ROUND_UP(c_us * scale, 1000);

	c_fr = FIELD_GET(RTL_COALESCE_RX_FRAMES, intrmit);
	ec->rx_max_coalesced_frames = (c_us || c_fr) ? c_fr * 4 : 1;

	return 0;
}

/* choose appropriate scale factor and CPlusCmd[0:1] for (speed, usec) */
static int rtl_coalesce_choose_scale(struct rtl8169_private *tp, u32 usec,
				     u16 *cp01)
{
	const struct rtl_coalesce_info *ci;
	u16 i;
	u32 tmp;

	ci = rtl_coalesce_info(tp);
	if (IS_ERR(ci))
		return PTR_ERR(ci);

	for (i = 0; i < 4; i++) {
		tmp = ci->nsecs * scale_factor[tp->cp_cmd & INTT_MASK];
		if (usec <= tmp * RTL_COALESCE_T_MAX / 1000U) {
			*cp01 = i;
			return tmp;
		}
	}

	return -ERANGE;
}

static int rtl_set_coalesce(struct net_device *dev,
			    struct ethtool_coalesce *ec,
			    struct kernel_ethtool_coalesce *kernel_coal,
			    struct netlink_ext_ack *extack)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	u32 tx_fr = ec->tx_max_coalesced_frames;
	u32 rx_fr = ec->rx_max_coalesced_frames;
	u32 coal_usec_max, units;
	u16 w = 0, cp01 = 0;
	int scale;

	if (rx_fr > RTL_COALESCE_FRAME_MAX || tx_fr > RTL_COALESCE_FRAME_MAX)
		return -ERANGE;

	coal_usec_max = max(ec->rx_coalesce_usecs, ec->tx_coalesce_usecs);
	scale = rtl_coalesce_choose_scale(tp, coal_usec_max, &cp01);
	if (scale < 0)
		return scale;

	/* Accept max_frames=1 we returned in rtl_get_coalesce. Accept it
	 * not only when usecs=0 because of e.g. the following scenario:
	 *
	 * - both rx_usecs=0 & rx_frames=0 in hardware (no delay on RX)
	 * - rtl_get_coalesce returns rx_usecs=0, rx_frames=1
	 * - then user does `ethtool -C eth0 rx-usecs 100`
	 *
	 * Since ethtool sends to kernel whole ethtool_coalesce settings,
	 * if we want to ignore rx_frames then it has to be set to 0.
	 */
	if (rx_fr == 1)
		rx_fr = 0;
	if (tx_fr == 1)
		tx_fr = 0;

	/* HW requires time limit to be set if frame limit is set */
	if ((tx_fr && !ec->tx_coalesce_usecs) ||
	    (rx_fr && !ec->rx_coalesce_usecs))
		return -EINVAL;

	w |= FIELD_PREP(RTL_COALESCE_TX_FRAMES, DIV_ROUND_UP(tx_fr, 4));
	w |= FIELD_PREP(RTL_COALESCE_RX_FRAMES, DIV_ROUND_UP(rx_fr, 4));

	units = DIV_ROUND_UP(ec->tx_coalesce_usecs * 1000U, scale);
	w |= FIELD_PREP(RTL_COALESCE_TX_USECS, units);
	units = DIV_ROUND_UP(ec->rx_coalesce_usecs * 1000U, scale);
	w |= FIELD_PREP(RTL_COALESCE_RX_USECS, units);

	RTL_W16(tp, INTR_MITIGATE, w);

	tp->cp_cmd = (tp->cp_cmd & ~INTT_MASK) | cp01;
	RTL_W16(tp, C_PLUS_CMD, tp->cp_cmd);
	RTL_R8(tp, CHIP_CMD);

	return 0;
}

static int rtl8169_get_eee(struct net_device *dev, struct ethtool_eee *data)
{
	struct rtl8169_private *tp = netdev_priv(dev);

	return phy_ethtool_get_eee(tp->phydev, data);
}

static int rtl8169_set_eee(struct net_device *dev, struct ethtool_eee *data)
{
	struct rtl8169_private *tp = netdev_priv(dev);

	rtl_lock_work(tp);
	tp->eee_enable = !!data->eee_enabled;
	tp->chip->eee_set(tp, tp->eee_enable);
	rtl_unlock_work(tp);
	fsleep(100000);	/* wait PHY ready */
	phy_restart_aneg(tp->phydev);
	return 0;
}

void	(*get_ringparam)(struct net_device *,
	struct ethtool_ringparam *,
	struct kernel_ethtool_ringparam *,
	struct netlink_ext_ack *);

static void rtl8169_get_ringparam(struct net_device *dev,
				  struct ethtool_ringparam *data,
				  struct kernel_ethtool_ringparam *ringparam,
				  struct netlink_ext_ack *ack)
{
	data->rx_max_pending = NUM_RX_DESC;
	data->rx_pending = NUM_RX_DESC;
	data->tx_max_pending = NUM_TX_DESC;
	data->tx_pending = NUM_TX_DESC;
}

static void rtl8169_get_pauseparam(struct net_device *dev,
				   struct ethtool_pauseparam *data)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	bool tx_pause, rx_pause;

	phy_get_pause(tp->phydev, &tx_pause, &rx_pause);

	data->autoneg = tp->phydev->autoneg;
	data->tx_pause = tx_pause ? 1 : 0;
	data->rx_pause = rx_pause ? 1 : 0;
}

static int rtl8169_set_pauseparam(struct net_device *dev,
				  struct ethtool_pauseparam *data)
{
	struct rtl8169_private *tp = netdev_priv(dev);

	if (dev->mtu > ETH_DATA_LEN)
		return -EOPNOTSUPP;

	phy_set_asym_pause(tp->phydev, data->rx_pause, data->tx_pause);

	return 0;
}

static const struct ethtool_ops rtl8169_ethtool_ops = {
	.supported_coalesce_params = ETHTOOL_COALESCE_USECS |
				     ETHTOOL_COALESCE_MAX_FRAMES,
	.get_drvinfo		= rtl8169_get_drvinfo,
	.get_regs_len		= rtl8169_get_regs_len,
	.get_link		= ethtool_op_get_link,
	.get_coalesce		= rtl_get_coalesce,
	.set_coalesce		= rtl_set_coalesce,
	.get_regs		= rtl8169_get_regs,
	.get_wol		= rtl8169_get_wol,
	.set_wol		= rtl8169_set_wol,
	.get_strings		= rtl8169_get_strings,
	.get_sset_count		= rtl8169_get_sset_count,
	.get_ethtool_stats	= rtl8169_get_ethtool_stats,
	.get_ts_info		= ethtool_op_get_ts_info,
	.nway_reset		= phy_ethtool_nway_reset,
	.get_eee		= rtl8169_get_eee,
	.set_eee		= rtl8169_set_eee,
	.get_link_ksettings	= phy_ethtool_get_link_ksettings,
	.set_link_ksettings	= phy_ethtool_set_link_ksettings,
	.get_ringparam		= rtl8169_get_ringparam,
	.get_pauseparam		= rtl8169_get_pauseparam,
	.set_pauseparam		= rtl8169_set_pauseparam,
};

static void rtl_hw_phy_config(struct rtl8169_private *tp)
{
	/* power down PHY */
	rtl_phy_write(tp, 0, MII_BMCR,
		      rtl_phy_read(tp, 0, MII_BMCR) | BMCR_PDOWN);

	/* set dis_mcu_clroob to avoid WOL fail when ALDPS mode is enabled */
	RTL_W8(tp, MCU, RTL_R8(tp, MCU) | DIS_MCU_CLROOB);

	tp->chip->hw_phy_config(tp);
}

static void rtl_schedule_task(struct rtl8169_private *tp, enum rtl_flag flag)
{
	if (!test_and_set_bit(flag, tp->wk.flags))
		schedule_work(&tp->wk.work);
}

static void __rtl_rar_set(struct rtl8169_private *tp, const u8 *addr)
{
	rtl_unlock_config_regs(tp);

	RTL_W32(tp, MAC4, get_unaligned_le16(addr + 4));
	RTL_R32(tp, MAC4);

	RTL_W32(tp, MAC0, get_unaligned_le32(addr));
	RTL_R32(tp, MAC0);

	rtl_lock_config_regs(tp);
}

static void rtl_rar_set(struct rtl8169_private *tp, const u8 *addr)
{
	rtl_lock_work(tp);
	__rtl_rar_set(tp, addr);
	rtl_unlock_work(tp);
}

static void rtl_phy_reinit(struct rtl8169_private *tp)
{
	u32 tmp;

	/* fill fuse_rdy & rg_ext_ini_done */
	rtl_phy_write(tp, 0x0a46, 20,
		      rtl_phy_read(tp, 0x0a46, 20) | (BIT(1) | BIT(0)));

	/* init_autoload_done = 1 */
	tmp = rtl_ocp_read(tp, 0xe004);
	tmp |= BIT(7);
	rtl_ocp_write(tp, 0xe004, tmp);

	/* wait LAN-ON */
	tmp = 0;
	do {
		tmp += 1;
		fsleep(1000);
		if (tmp >= 2000) {
			netdev_err(tp->dev, "PHY status is not 0x3, current = 0x%02x\n",
				   (rtl_phy_read(tp, 0x0a42, 16) & 0x07));
			set_bit(RTL_STATUS_HW_FAIL, tp->status);
			break;
		}
	} while (0x3 != (rtl_phy_read(tp, 0x0a42, 16) & 0x07));
	netdev_info(tp->dev, "wait %d ms for PHY ready, current = 0x%x\n",
		    tmp, rtl_phy_read(tp, 0x0a42, 16));

	fsleep(100000);	/* wait PHY ready */
	genphy_soft_reset(tp->phydev);
}

static void rtl8169_init_phy(struct rtl8169_private *tp)
{
	rtl_hw_phy_config(tp);

	/* disable now_is_oob */
	RTL_W8(tp, MCU, RTL_R8(tp, MCU) & ~NOW_IS_OOB);

	/* We may have called phy_speed_down before */
	phy_speed_up(tp->phydev);

	tp->chip->eee_set(tp, tp->eee_enable);

	genphy_soft_reset(tp->phydev);
}

static int rtl_set_mac_address(struct net_device *dev, void *p)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	struct sockaddr *addr = p;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	eth_hw_addr_set(dev, addr->sa_data);

	ether_addr_copy(tp->mac_addr, addr->sa_data);

	rtl_rar_set(tp, addr->sa_data);

	return 0;
}

static void rtl_storm_ctrl(struct rtl8169_private *tp,
			   u8 pkt_type,
			   u8 limit_type,
			   u16 limit)
{
	struct device *dev = &tp->pdev->dev;
	u32 data;
	u32 offset;
	u32 addr;

	if (limit_type > RTL_RATE_LIMIT) {
		dev_err(dev, "storm control: unknown limit type %d, limit = %d\n",
			limit_type, limit);
		return;
	}

	switch (pkt_type) {
	case RTL_BROADCAST_PKT:
		offset = 0;
		addr = 0xC084 + ((limit_type == RTL_RATE_LIMIT) ? 2 : 0);
		break;
	case RTL_MULTICAST_PKT:
		offset = 4;
		addr = 0xC088 + ((limit_type == RTL_RATE_LIMIT) ? 2 : 0);
		break;
	case RTL_UNKNOWN_PKT:
		offset = 8;
		addr = 0xC08C + ((limit_type == RTL_RATE_LIMIT) ? 2 : 0);
		break;
	default:
		dev_err(dev, "storm control: unknown pkt type %d\n", pkt_type);
		return;
	}

	/* set limit */
	rtl_ocp_write(tp, addr, limit);

	/* set limit type */
	data = rtl_ocp_read(tp, 0xC082);
	data &= ~(0x3 << offset);
	data |= limit_type << offset;
	rtl_ocp_write(tp, 0xC082, data);

	tp->sc[pkt_type].type = limit_type;
	tp->sc[pkt_type].limit = limit;
	dev_info(dev, "storm control: pkt type %d, limit type %d, limit %d\n",
		 pkt_type, limit_type, limit);
}

struct sk_buff *rtl_gen_loopback_pkt(struct rtl8169_private *tp, int len)
{
	struct sk_buff *skb;
	u16 type;
	u8 *iphdr;
	int ret = 0;

	type = htons(ETH_P_IP);
	skb = netdev_alloc_skb(tp->dev, rx_buf_sz + RTK_RX_ALIGN);
	if (!skb) {
		ret = -ENOMEM;
		goto err_out;
	}

	skb_reserve(skb, RTK_RX_ALIGN);

	memcpy(skb_put(skb, tp->dev->addr_len), tp->dev->dev_addr, tp->dev->addr_len);
	memcpy(skb_put(skb, tp->dev->addr_len), tp->dev->dev_addr, tp->dev->addr_len);
	memcpy(skb_put(skb, sizeof(type)), &type, sizeof(type));
	iphdr = skb_put(skb, len - ETH_HLEN);

	return skb;
err_out:
	return ERR_PTR(ret);
}

static int rtl_dump_hex_to_buf(u8 *s, u32 len, u8 *buf, u32 buf_len)
{
	u32 offset = 0;
	u32 k;

	if (!buf || len > buf_len)
		return -EINVAL;

	for (k = 0; k  < len; k++) {
		if ((k % 16) == 0)
			offset += snprintf(buf + offset, buf_len - offset,
				      "\n");
		else if ((k % 8) == 0)
			offset += snprintf(buf + offset, buf_len - offset,
				      "- ");
		offset += snprintf(buf + offset, buf_len - offset,
			      "%02X ", s[k]);
	}

	return offset;
}

static int rtl_loopback(struct rtl8169_private *tp, u32 mode, u32 len, u32 cnt);
static void rtl_mac_reinit(struct rtl8169_private *tp);
static int rtl_str2hex(struct rtl8169_private *tp, char *src, char *dst);

static int rtl_tool_ioctl(struct rtl8169_private *tp, struct ifreq *ifr)
{
	struct rtl_ioctl_struct *sub_cmd = (struct rtl_ioctl_struct *)&ifr->ifr_data;
	int ret = 0;
	void __user *user_datap;
	char buf[RTK_BUF_SIZE];
	int i;
	int start;
	int end;

	if (in_compat_syscall())
		user_datap = compat_ptr(sub_cmd->data);
	else
		user_datap = sub_cmd->buf;

	switch (sub_cmd->cmd) {
	case RTLTOOL_READ_WOL:
		sub_cmd->data = tp->wol_enable;
		break;
	case RTLTOOL_WRITE_WOL:
		tp->wol_enable = sub_cmd->data;
		tp->dev->wol_enabled = (tp->wol_enable & WOL_MAGIC) ? 1 : 0;
		tp->saved_wolopts |= (tp->wol_enable & WOL_MAGIC) ? WAKE_MAGIC : 0;
		device_set_wakeup_enable(tp_to_dev(tp), tp->dev->wol_enabled);
		netdev_dbg(tp->dev, "set wol_enable = 0x%x\n", tp->wol_enable);
		break;
	case RTLTOOL_READ_PWR_SAVING:
		sub_cmd->data = tp->pwr_saving;
		break;
	case RTLTOOL_WRITE_PWR_SAVING:
		tp->pwr_saving = sub_cmd->data;
		break;
	case RTLTOOL_READ_MAC:
		if (sub_cmd->offset & 0x3) {
			ret = -EINVAL;
			netdev_err(tp->dev, "offset 0x%08x is not 4-byte alignment\n",
				   sub_cmd->offset);
			break;
		}
		sub_cmd->data = RTL_R32(tp, sub_cmd->offset);
		break;
	case RTLTOOL_WRITE_MAC:
		if (sub_cmd->offset & 0x3) {
			ret = -EINVAL;
			netdev_err(tp->dev, "offset 0x%08x is not 4-byte alignment\n",
				   sub_cmd->offset);
			break;
		}
		RTL_W32(tp, sub_cmd->offset, sub_cmd->data);
		break;
	case RTLTOOL_READ_OCP:
		if (sub_cmd->offset & 0x1) {
			ret = -EINVAL;
			netdev_err(tp->dev, "offset 0x%08x is not 2-byte alignment\n",
				   sub_cmd->offset);
			break;
		}
		rtl_lock_work(tp);
		sub_cmd->data = rtl_ocp_read(tp, (u16)sub_cmd->offset);
		rtl_unlock_work(tp);
		break;
	case RTLTOOL_WRITE_OCP:
		if (sub_cmd->offset & 0x1) {
			ret = -EINVAL;
			netdev_err(tp->dev, "offset 0x%08x is not 2-byte alignment\n",
				   sub_cmd->offset);
			break;
		}
		rtl_lock_work(tp);
		rtl_ocp_write(tp, (u16)sub_cmd->offset, sub_cmd->data);
		rtl_unlock_work(tp);
		break;
	case RTLTOOL_READ_ERI:
		if (sub_cmd->offset & 0x1) {
			ret = -EINVAL;
			netdev_err(tp->dev, "offset 0x%08x is not 2-byte alignment\n",
				   sub_cmd->offset);
			break;
		}
		rtl_lock_work(tp);
		sub_cmd->data = rtl_eri_read(tp, (u16)sub_cmd->offset);
		rtl_unlock_work(tp);
		break;
	case RTLTOOL_WRITE_ERI:
		if (sub_cmd->offset & 0x1) {
			ret = -EINVAL;
			netdev_err(tp->dev, "offset 0x%08x is not 2-byte alignment\n",
				   sub_cmd->offset);
			break;
		}
		rtl_lock_work(tp);
		rtl_eri_write(tp, (u16)sub_cmd->offset, ERIAR_MASK_1111, sub_cmd->data);
		rtl_unlock_work(tp);
		break;
	case RTLTOOL_READ_PHY:
		rtl_lock_work(tp);
		sub_cmd->phy.val = rtl_phy_read(tp, sub_cmd->phy.page, sub_cmd->phy.addr);
		rtl_unlock_work(tp);
		break;
	case RTLTOOL_WRITE_PHY:
		rtl_lock_work(tp);
		rtl_phy_write(tp, sub_cmd->phy.page, sub_cmd->phy.addr, sub_cmd->phy.val);
		rtl_unlock_work(tp);
		break;
	case RTLTOOL_READ_EEE:
		sub_cmd->data = tp->eee_enable;
		break;
	case RTLTOOL_WRITE_EEE:
		if ((sub_cmd->data && !tp->eee_enable) ||
		    (!sub_cmd->data && tp->eee_enable)) {
			tp->eee_enable = !!sub_cmd->data;

			/* power down PHY */
			genphy_suspend(tp->phydev);
			rtl_lock_work(tp);
			tp->chip->eee_set(tp, tp->eee_enable);
			fsleep(100000); /* wait PHY ready */
			rtl_unlock_work(tp);
			genphy_soft_reset(tp->phydev);
		}
		break;
	case RTLTOOL_READ_WAKE_MASK:
		if (sub_cmd->len < tp->wol_rule_buf.mask_size) {
			ret = -EINVAL;
			netdev_err(tp->dev, "buffer size %d is too small, need %d bytes\n",
				   sub_cmd->len, tp->wol_rule_buf.mask_size);
			break;
		}
		if (copy_to_user(user_datap, tp->wol_rule_buf.mask, tp->wol_rule_buf.mask_size)) {
			ret = -EFAULT;
			netdev_dbg(tp->dev, "operation failed, src 0x%lx, dst 0x%lx, len %d\n",
				   (uintptr_t)tp->wol_rule_buf.mask, (uintptr_t)user_datap,
				   tp->wol_rule_buf.mask_size);
		}
		break;
	case RTLTOOL_WRITE_WAKE_MASK:
		if (sub_cmd->len > RTL_WAKE_MASK_SIZE) {
			ret = -EINVAL;
			netdev_dbg(tp->dev, "input size %d should not exceed %d bytes\n",
				   sub_cmd->len, RTL_WAKE_MASK_SIZE);
			sub_cmd->len = RTL_WAKE_MASK_SIZE;
		}
		if (copy_from_user(tp->wol_rule_buf.mask, user_datap, sub_cmd->len)) {
			ret = -EFAULT;
			netdev_dbg(tp->dev, "operation failed, src 0x%lx, dst 0x%lx, len %d\n",
				   (uintptr_t)user_datap, (uintptr_t)tp->wol_rule_buf.mask,
				   sub_cmd->len);
		} else {
			tp->wol_rule_buf.mask_size = sub_cmd->len;
		}
		break;
	case RTLTOOL_READ_WAKE_CRC:
		sub_cmd->data = tp->wol_rule_buf.crc;
		break;
	case RTLTOOL_WRITE_WAKE_CRC:
		tp->wol_rule_buf.crc = sub_cmd->data;
		break;
	case RTLTOOL_READ_WAKE_OFFSET:
		sub_cmd->data = tp->wol_rule_buf.offset;
		break;
	case RTLTOOL_WRITE_WAKE_OFFSET:
		tp->wol_rule_buf.offset = sub_cmd->data;
		break;
	case RTLTOOL_READ_WAKE_PATTERN:
		if (sub_cmd->len < tp->wol_rule_buf.pattern_size) {
			ret = -EINVAL;
			netdev_err(tp->dev, "buffer size %d is too small, need %d bytes\n",
				   sub_cmd->len, tp->wol_rule_buf.pattern_size);
			break;
		}
		if (copy_to_user(user_datap, tp->wol_rule_buf.pattern,
				 tp->wol_rule_buf.pattern_size)) {
			ret = -EFAULT;
			netdev_dbg(tp->dev, "operation failed, src 0x%lx, dst 0x%lx, len %d\n",
				   (uintptr_t)tp->wol_rule_buf.pattern,
				   (uintptr_t)user_datap,
				   tp->wol_rule_buf.pattern_size);
		}
		break;
	case RTLTOOL_WRITE_WAKE_PATTERN:
		if (sub_cmd->len > RTL_WAKE_PATTERN_SIZE) {
			ret = -EINVAL;
			netdev_dbg(tp->dev, "input size %d should not exceed %d bytes\n",
				   sub_cmd->len, RTL_WAKE_PATTERN_SIZE);
			sub_cmd->len = RTL_WAKE_PATTERN_SIZE;
		}
		if (copy_from_user(tp->wol_rule_buf.pattern, user_datap,
				   sub_cmd->len)) {
			ret = -EFAULT;
			netdev_dbg(tp->dev, "operation failed, src 0x%lx, dst 0x%lx, len %d\n",
				   (uintptr_t)user_datap,
				   (uintptr_t)tp->wol_rule_buf.pattern,
				   sub_cmd->len);
		} else {
			tp->wol_rule_buf.pattern_size = sub_cmd->len;
		}
		break;
	case RTLTOOL_READ_WAKE_IDX_EN:
		if (sub_cmd->offset < 0 || sub_cmd->offset > 31) {
			ret = -EINVAL;
			netdev_err(tp->dev, "index 0x%08x should be between 0 and 31\n",
				   sub_cmd->offset);
			break;
		}
		sub_cmd->data = !!(tp->wol_rule[sub_cmd->offset].flag &
				   WAKE_FLAG_ENABLE);
		break;
	case RTLTOOL_WRITE_WAKE_IDX_EN:
		if (sub_cmd->offset < 0 || sub_cmd->offset > 31) {
			ret = -EINVAL;
			netdev_err(tp->dev, "index 0x%08x should be between 0 and 31\n",
				   sub_cmd->offset);
			break;
		}
		if (sub_cmd->data > 1) {
			ret = -EINVAL;
			netdev_err(tp->dev, "data %u should be 0 or 1\n",
				   sub_cmd->data);
			break;
		}
		if (sub_cmd->data == 1) {
			if (!(tp->wol_rule[sub_cmd->offset].flag &
			      WAKE_FLAG_ENABLE))
				tp->wol_crc_cnt++;
			/* add/replace a rule */
			tp->wol_rule_buf.flag = WAKE_FLAG_ENABLE;
			memcpy(&tp->wol_rule[sub_cmd->offset], &tp->wol_rule_buf,
			       sizeof(struct rtl_wake_rule_s));
		} else {
			if (tp->wol_rule[sub_cmd->offset].flag & WAKE_FLAG_ENABLE)
				tp->wol_crc_cnt--;
			/* del a rule */
			tp->wol_rule_buf.flag &= ~WAKE_FLAG_ENABLE;
			tp->wol_rule[sub_cmd->offset].flag &= ~WAKE_FLAG_ENABLE;
		}
		break;
	case RTLTOOL_READ_STORM_CTRL:
		if (sub_cmd->sc.pkt_type > 2) {
			ret = -EINVAL;
			netdev_err(tp->dev, "invalid pkt type %d\n",
				   sub_cmd->sc.pkt_type);
			break;
		}
		sub_cmd->sc.limit_type = tp->sc[sub_cmd->sc.pkt_type].type;
		sub_cmd->sc.limit = tp->sc[sub_cmd->sc.pkt_type].limit;
		break;
	case RTLTOOL_WRITE_STORM_CTRL:
		if (sub_cmd->sc.pkt_type > 2) {
			ret = -EINVAL;
			netdev_err(tp->dev, "invalid pkt type %d\n",
				   sub_cmd->sc.pkt_type);
			break;
		}
		if (sub_cmd->sc.limit_type > 2) {
			ret = -EINVAL;
			netdev_err(tp->dev, "invalid limit type %d\n",
				   sub_cmd->sc.limit_type);
			break;
		}
		rtl_lock_work(tp);
		rtl_storm_ctrl(tp, sub_cmd->sc.pkt_type,
			       sub_cmd->sc.limit_type, sub_cmd->sc.limit);
		rtl_unlock_work(tp);
		break;
	case RTLTOOL_REINIT_MAC:
		if (sub_cmd->data)
			rtl_mac_reinit(tp);
		break;
	case RTLTOOL_REINIT_PHY:
		if (sub_cmd->data)
			rtl_phy_reinit(tp);
		break;
	case RTLTOOL_WRITE_ETH_LED:
		rtl_lock_work(tp);
		tp->chip->led_set(tp, !!sub_cmd->data);
		rtl_unlock_work(tp);
		break;
	case RTLTOOL_DUMP_WAKE_RULE:
		pr_err("WOL rules dump:\n");
		if (sub_cmd->offset >= 0 && sub_cmd->offset <= 31) {
			start = sub_cmd->offset;
			end = sub_cmd->offset + 1;
		} else {
			start = 0;
			end = RTL_WAKE_SIZE;
		}
		for (i = start; i < end; i++) {
			if (!(tp->wol_rule[i].flag & WAKE_FLAG_ENABLE))
				continue;

			pr_err("##### index %d #####\n", i);

			rtl_dump_hex_to_buf(tp->wol_rule[i].mask,
					    tp->wol_rule[i].mask_size,
					    buf, RTK_BUF_SIZE);
			pr_err("MASK = [%s]\n", buf);

			pr_err("CRC16  = 0x%04X\n", tp->wol_rule[i].crc);

			if (tp->chip->features & RTL_FEATURE_PAT_WAKE) {
				pr_err("OFFSET = 0x%04X\n", tp->wol_rule[i].offset);

				rtl_dump_hex_to_buf(tp->wol_rule[i].pattern,
						    tp->wol_rule[i].pattern_size,
						    buf, RTK_BUF_SIZE);
				pr_err("PATTERN = [%s]\n", buf);
			}
		}
		break;
	case RTLTOOL_TEST_LOOPBACK:
		if (sub_cmd->offset < 0 || sub_cmd->offset >= RTL_LOOPBACK_MAX) {
			ret = -EINVAL;
			netdev_err(tp->dev, "offset (mode) %u should be smaller than %d\n",
				   sub_cmd->offset, RTL_LOOPBACK_MAX);
			break;
		}
		if (sub_cmd->len < 60 || sub_cmd->len > 1514) {
			ret = -EINVAL;
			netdev_err(tp->dev, "packet len %u should be between 60 and 1514\n",
				   sub_cmd->data);
			break;
		}
		ret = rtl_loopback(tp, sub_cmd->offset, sub_cmd->len, sub_cmd->data);
		break;
	default:
		ret = -EOPNOTSUPP;
		netdev_dbg(tp->dev, "unknown cmd = 0x%08x\n", sub_cmd->cmd);
	}

	return ret;
}

/* FW passthrough behavior: 0: passthrough_list, 1: drop_all, 2: forward_all */
/* SW passthrough behavior: 0: forward_all,      1: drop_all, 2: passthrough_list */
static const int passthrough_behavior_map[] = { 2, 1, 0 };

static int rtl_hex2bin(char *src, int src_len, char *dst)
{
	int i;
	int dst_len = 0;
	char *p;
	char *t;
	char c = 0;

	t = dst;
	for (i = 0; i < src_len; i++) {
		p = src + i;
		switch (*p) {
		case '0' ... '9':
			c |= *p - '0';
			break;
		case 'a' ... 'f':
			c |= *p - 'a' + 10;
			break;
		case 'A' ... 'F':
			c |= *p - 'A' + 10;
			break;
		default:
			pr_err("Invalid ASCII hex value 0x%x, offset %d\n", *p, i);
			return -EINVAL;
		}

		if (i & 0x1) {
			*t++ = c;
			dst_len++;
			c = 0;
		} else {
			c <<= 4;
		}
	}

	return dst_len;
}

static int _mdns_qname_copy(u8 *buf, char *raw_pkt, u16 offset)
{
	int len = 0;
	u8 *p;
	u8 *t;
	u8 c = 0;

	p = raw_pkt + offset;
	t = buf;

	do {
		c = *p;
		if ((c & 0xC0) == 0xC0) {
			/* RR pointer */
			offset = p[1] | ((p[0] & 0x3F) << 8);
			p = raw_pkt + offset;
			continue;
		} else if (c > 0) {
			/* label size */
			memcpy(t, p, c + 1);
			p += c + 1;
			t += c + 1;
			len += c + 1;
			if (len > MDNS_NAME_SIZE_MAX)
				return -EINVAL;
		} else {
			/* c == 0, end */
			*t = c;
			len++;
		}
	} while (c);

	return len;
}

static int _mdns_set_qname(u8 *buf, char *name)
{
	u8 *curr = buf;
	char *n = kstrdup(name, GFP_KERNEL);  /* dynamically allocated memory */
	char *p;
	char *t;
	char *delim = ".";
	int len;

	/* fill qname */
	p = n;
	while ((t = strsep(&p, delim)) != NULL) {
		len = strlen(t);
		*curr = len;
		curr++;
		strscpy(curr, t, len + 1);
		curr += len;
	}
	kfree(n);
	*curr = '\0';
	curr++;

	len = curr - buf;
	return len;
}

static __maybe_unused void
mdns_free_split_string(unsigned char **tokens, int max_tokens)
{
	int i = 0;

	for (i = 0; i < max_tokens; i++)
		kfree(tokens[i]);
}

/* Function to split a string */
static __maybe_unused int
mdns_split_string(char *str, unsigned char *delim, unsigned char **tokens,
		  int max_tokens, unsigned char **str_point)
{
	unsigned char *token;
	int token_cnt = 0;

	/* The first call to strsep requires the string pointer */
	/* subsequent calls should use NULL */
	token = strsep(&str, delim);
	while (token && token_cnt < max_tokens) {
		tokens[token_cnt] = kstrdup(token, GFP_KERNEL);  /* Dynamically allocate memory */
		token_cnt++;
		if (token_cnt >= max_tokens)
			break;
		/* Subsequent calls to strsep with NULL */
		token = strsep(&str, delim);
	}

	*str_point = str;
	return token_cnt;  /* Return the number of tokens stored */
}

static int mdns_verify_passthrough(struct rtl8169_private *tp, unsigned char *qname, int qname_size)
{
	int i = 0;

	if (tp->passthrough_list.list_size >= MDNS_PASSTHROUGH_MAX) {
		netdev_err(tp->dev, "passthrough_list.list_size >= MDNS_PASSTHROUGH_MAX\n");
		return -1;
	}

	if (qname_size >= MDNS_QNAME_LEN_MAX) {
		netdev_err(tp->dev, "qname_size >= MDNS_QNAME_LEN_MAX\n");
		return -1;
	}

	for (i = 0; i < tp->passthrough_list.list_size; i++) {
		if (strncmp(tp->passthrough_list.list[i].qname, qname, qname_size) == 0) {
			netdev_info(tp->dev, "Already set [%s]\n",
				    tp->passthrough_list.list[i].qname);
			return -1;
		}
	}

	return 0;
}

static int mdns_add_passthrough(struct rtl8169_private *tp, u8 *qname, int qname_size)
{
	int idx = MDNS_PASSTHROUGH_MAX;
	int i;

	if (!qname || qname_size != strlen(qname) ||
	    qname_size >= MDNS_QNAME_LEN_MAX)
		goto OUT;

	for (i = 0; i < MDNS_PASSTHROUGH_MAX; i++)
		if (tp->passthrough_list.list[i].qname_len == 0) {
			idx = i;
			break;
		}
	if (idx == MDNS_PASSTHROUGH_MAX) {
		netdev_err(tp->dev, "passthrough_list is full\n");
		goto OUT;
	}

	if (mdns_verify_passthrough(tp, qname, qname_size) == 0) {
		tp->passthrough_list.list[idx].qname_len = qname_size;
		memcpy(tp->passthrough_list.list[idx].qname, qname, qname_size);
		tp->passthrough_list.list[idx].qname[qname_size] = '\0';
		tp->passthrough_list.list_size++;
	} else {
		netdev_err(tp->dev, "mdns_verify_passthrough fail\n");
		idx = MDNS_PASSTHROUGH_MAX;
	}
OUT:
	return idx;
}

static int mdns_del_passthrough(struct rtl8169_private *tp, u8 *qname, int qname_size)
{
	int idx = MDNS_PASSTHROUGH_MAX;
	int i;

	if (!qname || qname_size != strlen(qname) ||
	    qname_size >= MDNS_QNAME_LEN_MAX)
		goto OUT;

	for (i = 0; i < MDNS_PASSTHROUGH_MAX; i++) {
		if (tp->passthrough_list.list[i].qname_len == 0)
			continue;

		if (tp->passthrough_list.list[i].qname_len == qname_size &&
		    (strncmp(tp->passthrough_list.list[i].qname, qname, qname_size) == 0)) {
			idx = i;
			tp->passthrough_list.list[i].qname_len = 0;
			tp->passthrough_list.list[i].qname[0] = '\0';
			tp->passthrough_list.list_size--;
			break;
		}
	}
OUT:
	return idx;
}

static void mdns_reset_passthrough(struct rtl8169_private *tp)
{
	memset(&tp->passthrough_list, 0, sizeof(struct mdns_offload_passthrough_list));
}

static int mdns_verify_protocol_data_type(struct rtl8169_private *tp, int packet_index, int type)
{
	int i = 0;

	for (i = 0; i < tp->mdns_data_list.list[packet_index].type_size; i++) {
		if (type == tp->mdns_data_list.list[packet_index].type[i].qtype) {
			netdev_err(tp->dev, "Already set qtype: %d\n", type);
			return 1;
		}
	}

	if (tp->mdns_data_list.list[packet_index].type_size >= MDNS_TYPE_MAX) {
		netdev_err(tp->dev,
			   "mdns_data_list.list[%d].type_size >= MDNS_TYPE_MAX\n", packet_index);
		return -1;
	}

	return 0;
}

static __maybe_unused void
mdns_set_protocol_data_type(struct rtl8169_private *tp, int packet_index,
			    int type, int nameoffset)
{
	int type_size = tp->mdns_data_list.list[packet_index].type_size;

	if (mdns_verify_protocol_data_type(tp, packet_index, type) == 0) {
		tp->mdns_data_list.list[packet_index].type[type_size].nameoffset = nameoffset;
		tp->mdns_data_list.list[packet_index].type[type_size].qtype = type;
		tp->mdns_data_list.list[packet_index].type_size++;
	} else {
		netdev_err(tp->dev, "%s fail\n", __func__);
	}
}

static int mdns_verify_protocol_data_packet(struct rtl8169_private *tp,
					    unsigned char *packet, int packet_size)
{
	int i;
	int j;

	if (packet_size >= MDNS_MAX_PACKET_SIZE) {
		netdev_err(tp->dev, "packet_size >= MDNS_MAX_PACKET_SIZE\n");
		return -1;
	}

	if (!packet) {
		netdev_err(tp->dev, "the empty packet is invalid\n");
		return -1;
	}

	if (packet_size != strlen(packet)) {
		netdev_err(tp->dev, "packet_size %d != strlen(packet) %ld\n",
			   packet_size, strlen(packet));
		return -1;
	}

	for (i = 0, j = 0; i < MDNS_PACKET_MAX; i++) {
		if (tp->mdns_data_list.list[i].packet_size == 0)
			continue;
		if (strncmp(tp->mdns_data_list.list[i].packet, packet, packet_size) == 0) {
			netdev_info(tp->dev, "Already set mdns_data_list.list[%d].packet: %s\n",
				    i, tp->mdns_data_list.list[i].packet);
			return i;
		}
		if (++j == tp->mdns_data_list.list_size)
			break;
	}

	if (tp->mdns_data_list.list_size >= MDNS_PACKET_MAX) {
		netdev_err(tp->dev, "mdns_data_list.list_size >= MDNS_PACKET_MAX\n");
		return -1;
	}

	return -2; /* not found */
}

static __maybe_unused int
mdns_set_protocol_data_packet(struct rtl8169_private *tp,
			      unsigned char *packet_data,
			      int type, int nameoffset, int packet_size)
{
	int list_size = tp->mdns_data_list.list_size;
	int ret;

	ret = mdns_verify_protocol_data_packet(tp, packet_data, packet_size);

	if (ret == -2) {
		memcpy(tp->mdns_data_list.list[list_size].packet, packet_data, packet_size);
		tp->mdns_data_list.list[list_size].packet[packet_size] = '\0';
		tp->mdns_data_list.list[list_size].type[0].nameoffset = nameoffset;
		tp->mdns_data_list.list[list_size].type[0].qtype = type;
		tp->mdns_data_list.list[list_size].packet_size = packet_size;
		tp->mdns_data_list.list[list_size].type_size = 1;
		tp->mdns_data_list.list_size++;
	} else if (ret == -1) {
		netdev_err(tp->dev, "%s fail\n", __func__);
	}

	return ret;
}

static int mdns_add_protocol_data_packet(struct rtl8169_private *tp,
					 struct mdns_proto_data *data)
{
	int ret;
	int idx = MDNS_PACKET_MAX;
	int i;
	int len;

	data->packet[MDNS_MAX_PACKET_SIZE - 1] = '\0';
	ret = mdns_verify_protocol_data_packet(tp, data->packet,
					       data->packet_size);
	len = sizeof(struct mdns_proto_data);

	if (ret == -2) { /* not found */
		for (i = 0; i < MDNS_PACKET_MAX; i++)
			if (tp->mdns_data_list.list[i].packet_size == 0) {
				idx = i;
				break;
			}
		if (idx == MDNS_PACKET_MAX) {
			netdev_err(tp->dev, "mdns_data_list is full\n");
			ret = -EPERM;
			goto OUT;
		}
		memcpy(&tp->mdns_data_list.list[idx], data, len);
		ret = idx;
		tp->mdns_data_list.list_size++;
	} else if (ret == -1) {
		netdev_err(tp->dev, "%s fail\n", __func__);
	} else {
		/* update the original record */
		memset(&tp->mdns_data_list.list[ret], 0, len);
		memcpy(&tp->mdns_data_list.list[ret], data, len);
	}

OUT:
	return ret;
}

static int mdns_del_protocol_data_packet(struct rtl8169_private *tp, int idx)
{
	int ret = idx;

	if (idx >= MDNS_PACKET_MAX || idx < 0) {
		netdev_err(tp->dev, "invalid index %d\n", idx);
		ret = -EINVAL;
		goto OUT;
	}
	if (tp->mdns_data_list.list[idx].packet_size > 0)
		tp->mdns_data_list.list_size--;
	memset(&tp->mdns_data_list.list[idx], 0, sizeof(struct mdns_proto_data));

OUT:
	return ret;
}

static void mdns_reset_protocol_data_packet(struct rtl8169_private *tp)
{
	memset(&tp->mdns_data_list, 0, sizeof(struct mdns_proto_data_list));
}

static int rtl_mdns_ioctl(struct rtl8169_private *tp, struct ifreq *ifr)
{
	struct rtl_ioctl_struct *sub_cmd = (struct rtl_ioctl_struct *)&ifr->ifr_data;
	int ret = 0;
	void __user *user_datap;
	char buf[MDNS_MAX_PACKET_SIZE];
	struct mdns_proto_data offload_data;
	int i;

	memset(&offload_data, 0, sizeof(offload_data));

	if (in_compat_syscall())
		user_datap = compat_ptr(sub_cmd->data);
	else
		user_datap = sub_cmd->buf;

	switch (sub_cmd->cmd) {
	case RTLMDNS_READ_PASSTHROUGH:
		if (sub_cmd->offset >= MDNS_PASSTHROUGH_MAX) {
			ret = -EINVAL;
			netdev_err(tp->dev, "offset %d should be smaller than %d\n",
				   sub_cmd->offset, MDNS_PASSTHROUGH_MAX);
			break;
		}
		if (sub_cmd->len < MDNS_QNAME_LEN_MAX) {
			ret = -EINVAL;
			netdev_err(tp->dev, "buffer size %d is too small, need %d bytes\n",
				   sub_cmd->len, MDNS_QNAME_LEN_MAX);
			break;
		}
		if (tp->passthrough_list.list[sub_cmd->offset].qname_len == 0) {
			ret = copy_to_user(user_datap, "\0", 1);
			sub_cmd->len = 0;
			break;
		}
		if (copy_to_user(user_datap, tp->passthrough_list.list[sub_cmd->offset].qname,
				 tp->passthrough_list.list[sub_cmd->offset].qname_len)) {
			ret = -EFAULT;
			netdev_dbg(tp->dev, "operation failed, src 0x%lx, dst 0x%lx, len %d\n",
				   (uintptr_t)tp->passthrough_list.list[sub_cmd->offset].qname,
				   (uintptr_t)user_datap,
				   tp->passthrough_list.list[sub_cmd->offset].qname_len);
		} else {
			sub_cmd->len = tp->passthrough_list.list[sub_cmd->offset].qname_len;
		}
		break;
	case RTLMDNS_ADD_PASSTHROUGH:
		if (sub_cmd->len >= MDNS_QNAME_LEN_MAX) {
			ret = -EINVAL;
			netdev_dbg(tp->dev, "input size %d should not exceed %d bytes\n",
				   sub_cmd->len, MDNS_QNAME_LEN_MAX - 1);
			sub_cmd->len = RTL_WAKE_MASK_SIZE - 1;
		}
		if (copy_from_user(buf, user_datap, sub_cmd->len)) {
			ret = -EFAULT;
			netdev_dbg(tp->dev, "operation failed, src 0x%lx, dst 0x%lx, len %d\n",
				   (uintptr_t)user_datap, (uintptr_t)buf,
				   sub_cmd->len);
		} else {
			buf[sub_cmd->len] = '\0';
			sub_cmd->offset = mdns_add_passthrough(tp, buf, sub_cmd->len);
		}
		break;
	case RTLMDNS_DEL_PASSTHROUGH:
		if (sub_cmd->len >= MDNS_QNAME_LEN_MAX) {
			ret = -EINVAL;
			netdev_err(tp->dev, "input size %d should not exceed %d bytes\n",
				   sub_cmd->len, MDNS_QNAME_LEN_MAX - 1);
			sub_cmd->len = RTL_WAKE_MASK_SIZE - 1;
		}
		if (copy_from_user(buf, user_datap, sub_cmd->len)) {
			ret = -EFAULT;
			netdev_dbg(tp->dev, "operation failed, src 0x%lx, dst 0x%lx, len %d\n",
				   (uintptr_t)user_datap, (uintptr_t)buf,
				   sub_cmd->len);
		} else {
			buf[sub_cmd->len] = '\0';
			sub_cmd->offset = mdns_del_passthrough(tp, buf, sub_cmd->len);
		}
		break;
	case RTLMDNS_RESET_PASSTHROUGH:
		mdns_reset_passthrough(tp);
		break;
	case RTLMDNS_READ_PASSTHROUGH_BEHAVIOR:
		sub_cmd->data = passthrough_behavior_map[tp->passthrough_behavior];
		break;
	case RTLMDNS_WRITE_PASSTHROUGH_BEHAVIOR:
		if (sub_cmd->data >= 3) {
			ret = -EINVAL;
			netdev_err(tp->dev, "invalid passthrough behavior %d\n",
				   sub_cmd->data);
			break;
		}
		tp->passthrough_behavior = passthrough_behavior_map[sub_cmd->data];
		break;
	case RTLMDNS_READ_PROTO_DATA:
		if (sub_cmd->offset >= MDNS_PACKET_MAX) {
			ret = -EINVAL;
			netdev_err(tp->dev, "offset %d should be smaller than %d\n",
				   sub_cmd->offset, MDNS_PACKET_MAX);
			break;
		}
		if (sub_cmd->len < sizeof(offload_data)) {
			ret = -EINVAL;
			netdev_err(tp->dev, "buffer size %d is too small, need %ld bytes\n",
				   sub_cmd->len, sizeof(offload_data));
			break;
		}
		if (tp->mdns_data_list.list[sub_cmd->offset].packet_size == 0) {
			ret = copy_to_user(user_datap, &offload_data, sizeof(offload_data));
			sub_cmd->len = 0;
			break;
		}
		if (copy_to_user(user_datap, &tp->mdns_data_list.list[sub_cmd->offset],
				 sizeof(offload_data))) {
			ret = -EFAULT;
			netdev_dbg(tp->dev, "operation failed, src 0x%lx, dst 0x%lx, len %ld\n",
				   (uintptr_t)&tp->mdns_data_list.list[sub_cmd->offset],
				   (uintptr_t)user_datap,
				   sizeof(offload_data));
		}
		break;
	case RTLMDNS_ADD_PROTO_DATA:
		if (sub_cmd->len > sizeof(offload_data)) {
			ret = -EINVAL;
			netdev_dbg(tp->dev, "input size %d should not exceed %ld bytes\n",
				   sub_cmd->len, sizeof(offload_data));
			sub_cmd->offset = ret;
			break;
		}
		if (copy_from_user(&offload_data, user_datap, sizeof(offload_data))) {
			ret = -EFAULT;
			netdev_dbg(tp->dev, "operation failed, src 0x%lx, dst 0x%lx, len %ld\n",
				   (uintptr_t)user_datap, (uintptr_t)&offload_data,
				   sizeof(offload_data));
		} else {
			sub_cmd->offset = mdns_add_protocol_data_packet(tp, &offload_data);
		}
		break;
	case RTLMDNS_DEL_PROTO_DATA:
		if (sub_cmd->offset >= MDNS_PACKET_MAX) {
			ret = -EINVAL;
			netdev_err(tp->dev, "offset %d should be smaller than %d\n",
				   sub_cmd->offset, MDNS_PACKET_MAX);
			break;
		}
		sub_cmd->offset = mdns_del_protocol_data_packet(tp, sub_cmd->offset);
		break;
	case RTLMDNS_RESET_PROTO_DATA:
		mdns_reset_protocol_data_packet(tp);
		break;
	case RTLMDNS_READ_WOL_IPV4:
		for (i = 0; i < 4; i++)
			sub_cmd->ipv4[i] = tp->g_wol_ipv4_addr[i];
		break;
	case RTLMDNS_WRITE_WOL_IPV4:
		for (i = 0; i < 4; i++)
			tp->g_wol_ipv4_addr[i] = sub_cmd->ipv4[i];
		break;
	case RTLMDNS_READ_WOL_IPV6:
		if (sub_cmd->len < 16) {
			ret = -EINVAL;
			netdev_err(tp->dev, "buffer size %d is too small, need %d bytes\n",
				   sub_cmd->len, 16);
			break;
		}
		if (copy_to_user(user_datap, tp->g_wol_ipv6_addr, 16)) {
			ret = -EFAULT;
			netdev_dbg(tp->dev, "operation failed, src 0x%lx, dst 0x%lx, len %d\n",
				   (uintptr_t)tp->g_wol_ipv6_addr, (uintptr_t)user_datap, 16);
		} else {
			sub_cmd->len = 16;
		}
		break;
	case RTLMDNS_WRITE_WOL_IPV6:
		if (sub_cmd->len > 16) {
			ret = -EINVAL;
			netdev_dbg(tp->dev, "input size %d should not exceed %d bytes\n",
				   sub_cmd->len, 16);
			sub_cmd->offset = ret;
		}
		if (copy_from_user(&buf, user_datap, 16)) {
			ret = -EFAULT;
			netdev_dbg(tp->dev, "operation failed, src 0x%lx, dst 0x%lx, len %d\n",
				   (uintptr_t)buf, (uintptr_t)&offload_data, 16);
		} else {
			for (i = 0; i < 16; i++)
				tp->g_wol_ipv6_addr[i] = buf[i];
			sub_cmd->len = 16;
		}
		break;
	case RTLMDNS_READ_STATE:
		sub_cmd->data = tp->mdns_offload_state;
		break;
	case RTLMDNS_WRITE_STATE:
		tp->mdns_offload_state = !!sub_cmd->data;
		break;
	default:
		ret = -EOPNOTSUPP;
		netdev_dbg(tp->dev, "unknown cmd = 0x%08x\n", sub_cmd->cmd);
	}

	return ret;
}

static int rtl8169soc_ioctl(struct net_device *dev,
			    struct ifreq *ifr,
			    void __user *data,
			    int cmd)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	int ret = 0;

	switch (cmd) {
	case SIOCDEVPRIVATE_RTLTOOL:
		if (!capable(CAP_NET_ADMIN)) {
			ret = -EPERM;
			break;
		}
		ret = rtl_tool_ioctl(tp, ifr);
		break;
	case SIOCDEVPRIVATE_RTLMDNS:
		if (!capable(CAP_NET_ADMIN)) {
			ret = -EPERM;
			break;
		}
		ret = rtl_mdns_ioctl(tp, ifr);
		break;
	default:
		ret = -EOPNOTSUPP;
		netdev_dbg(tp->dev, "unknown cmd = 0x%08x\n", cmd);
	}

	return ret;
}

static void rtl_wpd_set(struct rtl8169_private *tp)
{
	u32 tmp;
	u32 i;

	/* clear WPD registers */
	rtl_ocp_write(tp, 0xD23A, 0);
	rtl_ocp_write(tp, 0xD23C, 0);
	rtl_ocp_write(tp, 0xD23E, 0);
	for (i = 0; i < 128; i += 2)
		rtl_ocp_write(tp, 0xD240 + i, 0);

	tmp = rtl_ocp_read(tp, 0xC0C2);
	tmp &= ~BIT(4);
	rtl_ocp_write(tp, 0xC0C2, tmp);

	/* enable WPD */
	tmp = rtl_ocp_read(tp, 0xC0C2);
	tmp |= BIT(0);
	rtl_ocp_write(tp, 0xC0C2, tmp);
}

/* EXACTLY PATTERN WAKE UP */
static int rtl_cp_reduced_pattern(struct rtl8169_private *tp, u32 idx,
				  u8 *src, u32 len)
{
	struct device *d = tp_to_dev(tp);
	u32 i;
	u32 j;
	u32 src_offset = 0;
	u32 dst_offset = 0;

	memset(&tp->wol_rule[idx].pattern[0], 0, RTL_WAKE_PATTERN_SIZE);

	if (tp->wol_rule[idx].mask_size >= RTL_WAKE_MASK_SIZE) {
		dev_err(d, "WOL rule[%d]: incorrect mask size %d\n", idx,
			tp->wol_rule[idx].mask_size);
		return 0;
	}

	for (i = 0; i < tp->wol_rule[idx].mask_size; i++) {
		for (j = 0; j < 8; j++) {
			if (tp->wol_rule[idx].mask[i] & (1 << j)) {
				dst_offset = (i * 8) + j;
				tp->wol_rule[idx].pattern[dst_offset] =
					src[src_offset++];
			}
		}
	}

	if (src_offset != len)
		dev_err(d, "WOL rule[%d]: expected pattern len %d, actual len %d\n",
			idx, len, src_offset);

	return (dst_offset + 1);
}

static void rtl_write_pat_wakeup_pattern(struct rtl8169_private *tp, u32 idx)
{
	u8 i;
	u32 reg_shift;
	u32 reg_offset;
	u16 mask_sel;
	u16 cross_flag;
	u8 zero_prefix;
	u16 tmp;
	u16 reg;
	u8 temp_mask[RTL_WAKE_MASK_REG_SIZE];
	u8 zero_mask[] = {0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F};
	u16 data_byte;
	u16 half_byte;
	u8 temp_pattern[RTL_WAKE_PATTERN_SIZE + 2];
	u16 *ptr;

	/* check size of wake-mask */
	if (tp->wol_rule[idx].mask_size > RTL_WAKE_MASK_SIZE) {
		netdev_err(tp->dev, "size of WoL wake-mask%d is too long\n", idx);
		netdev_err(tp->dev, "mask size %d (max %d)\n",
			   tp->wol_rule[idx].mask_size, RTL_WAKE_MASK_SIZE);
		return;
	}

	/* check size of wake-pattern */
	if (tp->wol_rule[idx].pattern_size >= RTL_WAKE_PATTERN_SIZE) {
		netdev_err(tp->dev, "size of WoL wake-pattern%d is too long\n", idx);
		netdev_err(tp->dev, "pattern size %d (max %d)\n",
			   tp->wol_rule[idx].pattern_size, RTL_WAKE_PATTERN_SIZE);
		return;
	}

	/* make sure mask contains correct zero prefix */
	zero_prefix = zero_mask[tp->wol_rule[idx].offset % 8];
	if (tp->wol_rule[idx].mask[0] & zero_prefix) {
		netdev_err(tp->dev, "incorrect zero prefix of WoL wake-mask%d\n", idx);
		netdev_err(tp->dev, "offset %d, zero prefix 0x%x, mask[0] 0x%x\n",
			   tp->wol_rule[idx].offset, zero_prefix,
			   tp->wol_rule[idx].mask[0]);
		return;
	}

	/* make sure pattern contains correct zero prefix */
	tmp = tp->wol_rule[idx].offset % 8;
	for (i = 0; i < tmp; i++) {
		if (tp->wol_rule[idx].pattern[i] == 0x00)
			continue;
		netdev_err(tp->dev, "incorrect zero prefix of WoL wake-pattern%d\n", idx);
		netdev_err(tp->dev, "offset %d, zero prefix cnt %d, pattern[i] 0x%x\n",
			   tp->wol_rule[idx].offset, tmp,
			   tp->wol_rule[idx].pattern[i]);
		return;
	}

	/* fill CRC_MASK_SEL_SET reg */
	reg_offset = (idx / 4) * 2;
	reg_shift  = idx % 4;

	cross_flag = ((tp->wol_rule[idx].offset % 256) >= 128) ? 1 : 0;
	mask_sel = ((tp->wol_rule[idx].offset / 256) << 1) | cross_flag;

	reg = 0xC130 + reg_offset;
	tmp = rtl_ocp_read(tp, reg);
	tmp &= ~(0xF << reg_shift);
	tmp |= mask_sel << reg_shift;
	rtl_ocp_write(tp, reg, tmp);

	/* create 32-byte WAKE_MASK according to 17-byte tp->wol_rule[idx].mask */
	memset(temp_mask, 0, RTL_WAKE_MASK_REG_SIZE);
	tmp = (tp->wol_rule[idx].offset % 256) / 8;
	for (i = 0; i < tp->wol_rule[idx].mask_size; i++)
		temp_mask[(tmp + i) % RTL_WAKE_MASK_REG_SIZE] =
			tp->wol_rule[idx].mask[i];

	/* fill WAKE_MASK reg */
	reg_offset = (idx / 4) * 2;
	reg_shift = (idx % 4) * 4;

	for (i = 0; i < 64; i++) {
		reg = 0x7C00 + reg_offset + (i * 16);
		tmp = rtl_ocp_read(tp, reg);
		tmp &= ~(0xF << reg_shift);
		data_byte = temp_mask[(i * 4) / 8];
		half_byte = (data_byte >> ((i * 4) % 8)) & 0xF;
		tmp |= half_byte << reg_shift;
		rtl_ocp_write(tp, reg, tmp);
	}

	/* fill CRC reg */
	if (idx < 8)
		reg = 0xC020 + (idx * 2);
	else
		reg = 0xC100 + ((idx - 8) * 2);
	rtl_ocp_write(tp, reg, tp->wol_rule[idx].crc);

	/* fill start byte and exactly match pattern */
	memset(temp_pattern, 0, sizeof(temp_pattern));
	ptr = (u16 *)&temp_pattern[0];
	*ptr = tp->wol_rule[idx].offset & 0x7F8; /* 8 alignment */

	for (i = 0; i < tp->wol_rule[idx].pattern_size; i++)
		temp_pattern[i + 2] = tp->wol_rule[idx].pattern[i];

	for (i = 0; i < (RTL_WAKE_PATTERN_SIZE + 2) / 2; i++) {
		reg = 0x6A00 + (idx * 138) + (i * 2);
		ptr = (u16 *)&temp_pattern[i * 2];
		rtl_ocp_write(tp, reg, *ptr);
	}
}

static void rtl_pat_wakeup_rules_set(struct rtl8169_private *tp)
{
	int i;

	for (i = 0; i < RTL_WAKE_SIZE; i++)
		if (tp->wol_rule[i].flag & WAKE_FLAG_ENABLE)
			rtl_write_pat_wakeup_pattern(tp, i);
}

static void rtl_clear_pat_wakeup_pattern(struct rtl8169_private *tp)
{
	rtl_ocp_write(tp, 0xC140, 0xFFFF);
	rtl_ocp_write(tp, 0xC142, 0xFFFF);
}

static void rtl_pat_wakeup_set(struct rtl8169_private *tp, bool enable)
{
	u16 tmp;

	rtl_clear_pat_wakeup_pattern(tp);

	if (enable) {
		if (tp->wol_crc_cnt > 0 && (tp->wol_enable & WOL_CRC_MATCH)) {
			rtl_pat_wakeup_rules_set(tp);

			tmp = rtl_ocp_read(tp, 0xE8DE);
			tmp |= BIT(14); /* borrow 8K SRAM */
			rtl_ocp_write(tp, 0xE8DE, tmp);

			tmp = rtl_ocp_read(tp, 0xC0C2);
			tmp |= BIT(3); /* enable wol exactly pattern match */
			rtl_ocp_write(tp, 0xC0C2, tmp);
		}
	} else {
		if (tp->wol_crc_cnt > 0 && (tp->wol_enable & WOL_CRC_MATCH)) {
			tmp = rtl_ocp_read(tp, 0xC0C2);
			tmp &= ~BIT(3); /* disable wol exactly pattern match */
			rtl_ocp_write(tp, 0xC0C2, tmp);

			tmp = rtl_ocp_read(tp, 0xE8DE);
			tmp &= ~BIT(14); /* stop borrow 8K SRAM */
			rtl_ocp_write(tp, 0xE8DE, tmp);
		}

		tmp = rtl_ocp_read(tp, 0xE006);
		tmp &= ~BIT(15); /* unset LAN wake */
		rtl_ocp_write(tp, 0xE006, tmp);

		tmp = rtl_ocp_read(tp, 0xDC80);
		tmp |= BIT(0); /* clear OOB wakeup status */
		rtl_ocp_write(tp, 0xDC80, tmp);

		tmp = rtl_ocp_read(tp, 0xE8DE);
		tmp |= BIT(15); /* re-init shared FIFO */
		rtl_ocp_write(tp, 0xE8DE, tmp);

		/* wait link_list ready =1 (IO 0xD2 bit9=1) */
		rtl_loop_wait_high(tp, &rtl_link_list_ready_cond, 100, 42);
	}
}

/* mDNS offload + passthrough wake up + exactly pattern match */
static void rtl_mdns_wakeup_set(struct rtl8169_private *tp, bool enable)
{
	u16 tmp;

	if (enable) {
		tmp = rtl_ocp_read(tp, 0xE8DE);
		tmp |= BIT(14); /* borrow 8K SRAM */
		rtl_ocp_write(tp, 0xE8DE, tmp);

		if (tp->wol_enable & WOL_CRC_MATCH) {
			rtl_clear_pat_wakeup_pattern(tp);
			if (tp->wol_crc_cnt > 0)
				rtl_pat_wakeup_rules_set(tp);

			tmp = rtl_ocp_read(tp, 0xC0C2);
			tmp |= BIT(3); /* enable wol exactly pattern match */
			rtl_ocp_write(tp, 0xC0C2, tmp);
		}

		if ((tp->wol_enable & WOL_MDNS_OFFLOAD) && tp->mdns_offload_state > 0)
			rtl8169soc_mdns_mac_offload_set(tp);
	} else {
		if (tp->wol_enable & WOL_CRC_MATCH) {
			tmp = rtl_ocp_read(tp, 0xC0C2);
			tmp &= ~BIT(3); /* disable wol exactly pattern match */
			rtl_ocp_write(tp, 0xC0C2, tmp);
		}

		if ((tp->wol_enable & WOL_MDNS_OFFLOAD) && tp->mdns_offload_state > 0)
			rtl8169soc_mdns_mac_offload_unset(tp);

		tmp = rtl_ocp_read(tp, 0xE006);
		tmp &= ~BIT(15); /* unset LAN wake */
		rtl_ocp_write(tp, 0xE006, tmp);

		tmp = rtl_ocp_read(tp, 0xDC80);
		tmp |= BIT(0); /* clear OOB wakeup status */
		rtl_ocp_write(tp, 0xDC80, tmp);

		tmp = rtl_ocp_read(tp, 0xE8DE);
		tmp &= ~BIT(14); /* stop borrow 8K SRAM */
		rtl_ocp_write(tp, 0xE8DE, tmp);

		tmp = rtl_ocp_read(tp, 0xE8DE);
		tmp |= BIT(15); /* re-init shared FIFO */
		rtl_ocp_write(tp, 0xE8DE, tmp);

		/* wait link_list ready =1 (IO 0xD2 bit9=1) */
		rtl_loop_wait_high(tp, &rtl_link_list_ready_cond, 100, 42);
	}
}

/* CRC WAKE UP */
static void rtl_write_crc_wakeup_pattern(struct rtl8169_private *tp, u32 idx)
{
	u8 i;
	u8 j;
	u32 reg_mask;
	u32 reg_shift;
	u32 reg_offset;

	reg_offset = idx & ~(BIT(0) | BIT(1));
	reg_shift = (idx % 4) * 8;
	switch (reg_shift) {
	case 0:
		reg_mask = ERIAR_MASK_0001;
		break;
	case 8:
		reg_mask = ERIAR_MASK_0010;
		break;
	case 16:
		reg_mask = ERIAR_MASK_0100;
		break;
	case 24:
		reg_mask = ERIAR_MASK_1000;
		break;
	default:
		netdev_err(tp->dev, "Invalid shift bit 0x%x, idx = %d\n", reg_shift, idx);
		return;
	}

	for (i = 0, j = 0; i < 0x80; i += 8, j++) {
		rtl_eri_write(tp, i + reg_offset, reg_mask,
			      tp->wol_rule[idx].mask[j] << reg_shift);
	}

	reg_offset = idx * 2;
	if (idx % 2) {
		reg_mask = ERIAR_MASK_1100;
		reg_offset -= 2;
		reg_shift = 16;
	} else {
		reg_mask = ERIAR_MASK_0011;
		reg_shift = 0;
	}
	rtl_eri_write(tp, (int)(0x80 + reg_offset), reg_mask,
		      tp->wol_rule[idx].crc << reg_shift);
}

static void rtl_mdns_crc_wakeup(struct rtl8169_private *tp)
{
	int i;

	for (i = 0; i < RTL_WAKE_SIZE_CRC; i++)
		if (tp->wol_rule[i].flag & WAKE_FLAG_ENABLE)
			rtl_write_crc_wakeup_pattern(tp, i);
}

static void rtl_clear_crc_wakeup_pattern(struct rtl8169_private *tp)
{
	u8 i;

	for (i = 0; i < 0x80; i += 4)
		rtl_eri_write(tp, i, ERIAR_MASK_1111, 0x0);

	for (i = 0x80; i < 0x90; i += 4)
		rtl_eri_write(tp, i, ERIAR_MASK_1111, 0x0);
}

static void rtl_crc_wakeup_set(struct rtl8169_private *tp, bool enable)
{
	u16 tmp;

	rtl_clear_crc_wakeup_pattern(tp);

	if (enable) {
		if (tp->wol_crc_cnt > 0 && (tp->wol_enable & WOL_CRC_MATCH))
			rtl_mdns_crc_wakeup(tp);
	} else {
		tmp = rtl_ocp_read(tp, 0xE006);
		tmp &= ~BIT(15); /* unset LAN wake */
		rtl_ocp_write(tp, 0xE006, tmp);

		tmp = rtl_ocp_read(tp, 0xDC80);
		tmp |= BIT(0); /* clear OOB wakeup status */
		rtl_ocp_write(tp, 0xDC80, tmp);

		tmp = rtl_ocp_read(tp, 0xE8DE);
		tmp |= BIT(15); /* re-init shared FIFO */
		rtl_ocp_write(tp, 0xE8DE, tmp);

		/* wait link_list ready =1 (IO 0xD2 bit9=1) */
		rtl_loop_wait_high(tp, &rtl_link_list_ready_cond, 100, 42);
	}
}

static void rtl_init_rxcfg(struct rtl8169_private *tp)
{
	/* disable RX128_INT_EN to reduce CPU loading */
	RTL_W32(tp, RX_CONFIG, RX_DMA_BURST | RX_EARLY_OFF);
}

static void rtl8169_init_ring_indexes(struct rtl8169_private *tp)
{
	tp->dirty_tx = 0;
	tp->cur_tx = 0;
	tp->cur_rx = 0;

#if defined(CONFIG_RTL_RX_NO_COPY)
	tp->dirty_rx = 0;
#endif /* CONFIG_RTL_RX_NO_COPY */
}

DECLARE_RTL_COND(rtl_chipcmd_cond)
{
	return RTL_R8(tp, CHIP_CMD) & CMD_RESET;
}

static void rtl_hw_reset(struct rtl8169_private *tp)
{
	RTL_W8(tp, CHIP_CMD, CMD_RESET);

	rtl_loop_wait_low(tp, &rtl_chipcmd_cond, 100, 100);
}

static void rtl_rx_close(struct rtl8169_private *tp)
{
	RTL_W32(tp, RX_CONFIG, RTL_R32(tp, RX_CONFIG) & ~RX_CONFIG_ACCEPT_MASK);
}

DECLARE_RTL_COND(rtl_txcfg_empty_cond)
{
	return RTL_R32(tp, TX_CONFIG) & TXCFG_EMPTY;
}

DECLARE_RTL_COND(rtl_rxtx_empty_cond)
{
	return (RTL_R8(tp, MCU) & RXTX_EMPTY) == RXTX_EMPTY;
}

static void rtl_wait_txrx_fifo_empty(struct rtl8169_private *tp)
{
	rtl_loop_wait_high(tp, &rtl_txcfg_empty_cond, 100, 42);
	rtl_loop_wait_high(tp, &rtl_rxtx_empty_cond, 100, 42);
}

static void rtl_disable_rxdvgate(struct rtl8169_private *tp)
{
	RTL_W32(tp, MISC, RTL_R32(tp, MISC) & ~RXDV_GATED_EN);
}

static void rtl_enable_rxdvgate(struct rtl8169_private *tp)
{
	RTL_W32(tp, MISC, RTL_R32(tp, MISC) | RXDV_GATED_EN);
	fsleep(2000);
	rtl_wait_txrx_fifo_empty(tp);
}

static void rtl_wol_enable_rx(struct rtl8169_private *tp)
{
	RTL_W32(tp, RX_CONFIG, RTL_R32(tp, RX_CONFIG) |
		ACCEPT_BROADCAST | ACCEPT_MULTICAST | ACCEPT_MY_PHYS);
	rtl_disable_rxdvgate(tp);
}

static void rtl_prepare_power_down(struct rtl8169_private *tp)
{
	rtl_unlock_config_regs(tp);
	if (tp->wol_enable & WOL_MAGIC) {
		/* enable now_is_oob */
		RTL_W8(tp, MCU, RTL_R8(tp, MCU) | NOW_IS_OOB);
		RTL_W8(tp, CONFIG5, RTL_R8(tp, CONFIG5) | LAN_WAKE);
		RTL_W8(tp, CONFIG3, RTL_R8(tp, CONFIG3) | MAGIC_PKT);

		tp->chip->wakeup_set(tp, true);
		if (tp->wol_enable & WOL_WPD)
			rtl_wpd_set(tp);
	} else {
		RTL_W8(tp, CONFIG5, RTL_R8(tp, CONFIG5) & ~LAN_WAKE);
		RTL_W8(tp, CONFIG3, RTL_R8(tp, CONFIG3) & ~MAGIC_PKT);
	}
	rtl_lock_config_regs(tp);

	if ((tp->wol_enable & WOL_MAGIC) || device_may_wakeup(tp_to_dev(tp))) {
		phy_speed_down(tp->phydev, false);
		rtl_wol_enable_rx(tp);
	}
}

static void rtl_set_tx_config_registers(struct rtl8169_private *tp)
{
	/* Set DMA burst size and Interframe Gap Time */
	RTL_W32(tp, TX_CONFIG, (TX_DMA_BURST << TX_DMA_SHIFT) |
		(INTER_FRAME_GAP << TX_INTER_FRAME_GAP_SHIFT));
}

static void rtl_set_rx_max_size(struct rtl8169_private *tp, unsigned int rx_buf_sz)
{
	/* Low hurts. Let's disable the filtering. */
	RTL_W16(tp, RX_MAX_SIZE, rx_buf_sz);
}

static void rtl_set_rx_tx_desc_registers(struct rtl8169_private *tp)
{
	/* Magic spell: some iop3xx ARM board needs the TxDescAddrHigh
	 * register to be written before TxDescAddrLow to work.
	 * Switching from MMIO to I/O access fixes the issue as well.
	 */
	RTL_W32(tp, TX_DESC_START_ADDR_HIGH, ((u64)tp->tx_phy_addr) >> 32);
	RTL_W32(tp, TX_DESC_START_ADDR_LOW,
		((u64)tp->tx_phy_addr) & DMA_BIT_MASK(32));
	RTL_W32(tp, RX_DESC_ADDR_HIGH, ((u64)tp->rx_phy_addr) >> 32);
	RTL_W32(tp, RX_DESC_ADDR_LOW, ((u64)tp->rx_phy_addr) & DMA_BIT_MASK(32));
}

static void rtl_set_rx_mode(struct net_device *dev)
{
	u32 rx_mode = ACCEPT_BROADCAST | ACCEPT_MY_PHYS | ACCEPT_MULTICAST;
	/* Multicast hash filter */
	u32 mc_filter[2] = { 0xffffffff, 0xffffffff };
	struct rtl8169_private *tp = netdev_priv(dev);
	u32 tmp;

	if (dev->flags & IFF_PROMISC) {
		/* Unconditionally log net taps. */
		netdev_notice(dev, "Promiscuous mode enabled\n");
		rx_mode |= ACCEPT_ALL_PHYS;
	} else if (!(dev->flags & IFF_MULTICAST)) {
		rx_mode &= ~ACCEPT_MULTICAST;
	} else if ((netdev_mc_count(dev) > MC_FILTER_LIMIT) ||
		(dev->flags & IFF_ALLMULTI)) {
		/* Too many to filter perfectly -- accept all multicasts. */
	} else {
		struct netdev_hw_addr *ha;

		mc_filter[1] = 0;
		mc_filter[0] = 0;
		netdev_for_each_mc_addr(ha, dev) {
			u32 bit_nr = eth_hw_addr_crc(ha) >> 26;

			mc_filter[bit_nr >> 5] |= BIT(bit_nr & 31);
		}
	}

	tmp = mc_filter[0];

	mc_filter[0] = swab32(mc_filter[1]);
	mc_filter[1] = swab32(tmp);

	RTL_W32(tp, MAR0 + 4, mc_filter[1]);
	RTL_W32(tp, MAR0 + 0, mc_filter[0]);

	tmp = RTL_R32(tp, RX_CONFIG);
	RTL_W32(tp, RX_CONFIG, (tmp & ~RX_CONFIG_ACCEPT_OK_MASK) | rx_mode);
}

static void rtl_led_set(struct rtl8169_private *tp)
{
	/* LED setting */
	if (tp->led_cfg)
		RTL_W32(tp, LEDSEL, tp->led_cfg);
	else
		RTL_W32(tp, LEDSEL, tp->chip->led_cfg);
}

static void rtl_hw_start(struct rtl8169_private *tp)
{
	u32 tmp;
	u32 phy_status;
	const struct soc_device_attribute rtk_soc_rtd161x_a01[] = {
		{
			.family = "Realtek Thor",
			.revision = "A01",
		},
		{
		/* empty */
		}
	};
	const struct soc_device_attribute rtk_soc_rtd131x[] = {
		{
			.family = "Realtek Hank",
		},
		{
		/* empty */
		}
	};
	const struct soc_device_attribute rtk_soc_kent_b[] = {
		{
			.soc_id = "RTD1501B",
		},
		{
			.soc_id = "RTD1861B",
		},
		{
		/* empty */
		}
	};

	rtl_unlock_config_regs(tp);

	RTL_W8(tp, MAX_TX_PACKET_SIZE, TX_PACKET_MAX);

	rtl_set_rx_max_size(tp, rx_buf_sz);

	tp->cp_cmd |= RTL_R16(tp, C_PLUS_CMD) | PKT_CNTR_DISABLE | INTT_3;

	/* Disable VLAN De-tagging */
	tp->cp_cmd &= ~RX_VLAN;

	RTL_W16(tp, C_PLUS_CMD, tp->cp_cmd);

	RTL_W16(tp, INTR_MITIGATE, 0x5151);

	rtl_set_rx_tx_desc_registers(tp);

	rtl_set_tx_config_registers(tp);

	if (tp->chip->features & RTL_FEATURE_TX_NO_CLOSE) {
		/* enable tx no close mode */
		rtl_ocp_write(tp, 0xE610,
			      rtl_ocp_read(tp, 0xE610) | (BIT(4) | BIT(6)));
	}

	/* disable pause frame resend caused by nearfull for rtd161x A01 */
	if (soc_device_match(rtk_soc_rtd161x_a01))
		rtl_ocp_write(tp, 0xE862, rtl_ocp_read(tp, 0xE862) | BIT(0));

	if (tp->chip->features & RTL_FEATURE_ADJUST_FIFO) {
		/* TX FIFO threshold */
		rtl_ocp_write(tp, 0xE618, 0x0006);
		rtl_ocp_write(tp, 0xE61A, 0x0010);

		/* RX FIFO threshold */
		rtl_ocp_write(tp, 0xC0A0, 0x0002);
		rtl_ocp_write(tp, 0xC0A2, 0x0008);

		phy_status = rtl_ocp_read(tp, 0xde40);
		if ((phy_status & 0x0030) == 0x0020) {
			/* 1000 Mbps */
			rtl_ocp_write(tp, 0xC0A4, 0x0088);
			rtl_ocp_write(tp, 0xC0A8, 0x00A8);
		} else {
			/* 10/100 Mbps */
			rtl_ocp_write(tp, 0xC0A4, 0x0038);
			rtl_ocp_write(tp, 0xC0A8, 0x0048);
		}
	} else {
		/* TX FIFO threshold */
		rtl_ocp_write(tp, 0xE618, 0x0006);
		rtl_ocp_write(tp, 0xE61A, 0x0010);

		/* RX FIFO threshold */
		rtl_ocp_write(tp, 0xC0A0, 0x0002);
		rtl_ocp_write(tp, 0xC0A2, 0x0008);
		rtl_ocp_write(tp, 0xC0A4, 0x0038);
		rtl_ocp_write(tp, 0xC0A8, 0x0048);
	}

	RTL_W8(tp, CONFIG5, RTL_R8(tp, CONFIG5) & ~LAN_WAKE);
	RTL_W8(tp, CONFIG3, RTL_R8(tp, CONFIG3) & ~MAGIC_PKT);

	RTL_R8(tp, INTR_MASK);

	RTL_W32(tp, TX_CONFIG, RTL_R32(tp, TX_CONFIG) | TXCFG_AUTO_FIFO);

	rtl_reset_packet_filter(tp);
	rtl_eri_write(tp, 0x2f8, ERIAR_MASK_0011, 0x1d8f);

	rtl_disable_rxdvgate(tp);

	RTL_W8(tp, MAX_TX_PACKET_SIZE, EARLY_SIZE);

	rtl_eri_write(tp, 0xc0, ERIAR_MASK_0011, 0x0000);
	rtl_eri_write(tp, 0xb8, ERIAR_MASK_0011, 0x0000);

	/* Adjust EEE LED frequency */
	RTL_W8(tp, EEE_LED, RTL_R8(tp, EEE_LED) & ~0x07);

	rtl_w0w1_eri(tp, 0x2fc, 0x01, 0x06);
	rtl_eri_clear_bits(tp, 0x1b0, BIT(12));

	/* disable aspm and clock request before access ephy */
	RTL_W8(tp, CONFIG2, RTL_R8(tp, CONFIG2) & ~CLK_REQ_EN);
	RTL_W8(tp, CONFIG5, RTL_R8(tp, CONFIG5) & ~ASPM_EN);

	rtl_lock_config_regs(tp);

	RTL_W8(tp, CHIP_CMD, CMD_TX_ENB | CMD_RX_ENB);

	if (soc_device_match(rtk_soc_rtd131x) ||
	    soc_device_match(rtk_soc_kent_b)) {
		phy_status = rtl_ocp_read(tp, 0xde40);
		switch (tp->output_mode) {
		case OUTPUT_EMBEDDED_PHY:
			/* do nothing */
			break;
		case OUTPUT_RMII:
			/* adjust RMII interface setting, speed */
			tmp = rtl_ocp_read(tp, 0xea30) & ~(BIT(6) | BIT(5));
			switch (phy_status & 0x0030) { /* link speed */
			case 0x0000:
				/* 10M, RGMII clock speed = 2.5MHz */
				break;
			case 0x0010:
				/* 100M, RGMII clock speed = 25MHz */
				tmp |= BIT(5);
			}
			/* adjust RMII interface setting, duplex */
			if ((phy_status & BIT(3)) == 0)
				/* ETN spec, half duplex */
				tmp &= ~BIT(4);
			else	/* ETN spec, full duplex */
				tmp |= BIT(4);
			rtl_ocp_write(tp, 0xea30, tmp);
			break;
		case OUTPUT_RGMII_TO_MAC:
		case OUTPUT_RGMII_TO_PHY:
			/* adjust RGMII interface setting, duplex */
			tmp = rtl_ocp_read(tp, 0xea34) & ~(BIT(4) | BIT(3));
			switch (phy_status & 0x0030) { /* link speed */
			case 0x0000:
				/* 10M, RGMII clock speed = 2.5MHz */
				break;
			case 0x0010:
				/* 100M, RGMII clock speed = 25MHz */
				tmp |= BIT(3);
				break;
			case 0x0020:
				/* 1000M, RGMII clock speed = 125MHz */
				tmp |= BIT(4);
				break;
			}
			/* adjust RGMII interface setting, duplex */
			if ((phy_status & BIT(3)) == 0)
				/* ETN spec, half duplex */
				tmp &= ~BIT(2);
			else	/* ETN spec, full duplex */
				tmp |= BIT(2);
			rtl_ocp_write(tp, 0xea34, tmp);
			break;
		default:
			netdev_err(tp->dev, "invalid output mode %d\n", tp->output_mode);
			return;
		}
	}

	rtl_set_rx_mode(tp->dev);

	RTL_W16(tp, MULTI_INTR, RTL_R16(tp, MULTI_INTR) & 0xF000);

	rtl_led_set(tp);

	rtl_irq_enable(tp);
}

static int rtl8169_change_mtu(struct net_device *dev, int new_mtu)
{
#if defined(CONFIG_RTL_RX_NO_COPY)
	struct rtl8169_private *tp = netdev_priv(dev);

	if (new_mtu > tp->chip->jumbo_max)
		goto out;

	if (!netif_running(dev)) {
		rx_buf_sz_new = (new_mtu > ETH_DATA_LEN) ?
			new_mtu + ETH_HLEN + 8 + 1 : RX_BUF_SIZE;
		rx_buf_sz = rx_buf_sz_new;
		goto out;
	}

	rx_buf_sz_new = (new_mtu > ETH_DATA_LEN) ?
		new_mtu + ETH_HLEN + 8 + 1 : RX_BUF_SIZE;

	rtl_schedule_task(tp, RTL_FLAG_TASK_RESET_PENDING);

out:
#endif /* CONFIG_RTL_RX_NO_COPY */

	dev->mtu = new_mtu;
	netdev_update_features(dev);

	return 0;
}

static inline void rtl8169_mark_to_asic(struct rx_desc *desc, u32 rx_buf_sz)
{
	u32 eor = le32_to_cpu(desc->opts1) & RING_END;

	desc->opts2 = 0;
	/* Force memory writes to complete before releasing descriptor */
	dma_wmb();
	WRITE_ONCE(desc->opts1, cpu_to_le32(DESC_OWN | eor | rx_buf_sz));
}

#if defined(CONFIG_RTL_RX_NO_COPY)
static int
rtl8169_alloc_rx_skb(struct rtl8169_private *tp, struct sk_buff **sk_buff,
		     struct rx_desc *desc, int rx_buf_sz)
{
	struct device *d = &tp->pdev->dev;
	struct sk_buff *skb;
	dma_addr_t mapping;
	int ret = 0;

	skb = netdev_alloc_skb(tp->dev, rx_buf_sz + RTK_RX_ALIGN);
	if (!skb)
		goto err_out;

	skb_reserve(skb, RTK_RX_ALIGN);
	if (tp->acp_enable) {
		mapping = virt_to_phys(skb->data);
	} else {
		mapping = dma_map_single(d, skb->data, rx_buf_sz,
					 DMA_FROM_DEVICE);

		if (unlikely(dma_mapping_error(d, mapping))) {
			if (net_ratelimit())
				netdev_err(tp->dev, "Failed to map RX DMA!\n");
			goto free_skb;
		}
	}
	*sk_buff = skb;
	desc->addr = cpu_to_le64(mapping);
	rtl8169_mark_to_asic(desc, rx_buf_sz);

out:
	return ret;

free_skb:
	dev_kfree_skb(skb);
err_out:
	ret = -ENOMEM;
	desc->addr = 0;
	desc->opts1 = 0;
	goto out;
}
#else
static struct page *rtl8169_alloc_rx_data(struct rtl8169_private *tp,
					  struct rx_desc *desc)
{
	struct device *d = tp_to_dev(tp);
	int node = dev_to_node(d);
	dma_addr_t mapping;
	struct page *data;

	data = alloc_pages_node(node, GFP_KERNEL, get_order(rx_buf_sz));
	if (!data)
		return NULL;

	if (tp->acp_enable) {
		mapping = virt_to_phys(data);
	} else {
		mapping = dma_map_page(d, data, 0, rx_buf_sz, DMA_FROM_DEVICE);
		if (unlikely(dma_mapping_error(d, mapping))) {
			if (net_ratelimit())
				netdev_err(tp->dev, "Failed to map RX DMA!\n");
			__free_pages(data, get_order(rx_buf_sz));
			return NULL;
		}
	}

	desc->addr = cpu_to_le64(mapping);
	rtl8169_mark_to_asic(desc, rx_buf_sz);

	return data;
}
#endif /* CONFIG_RTL_RX_NO_COPY */

#if defined(CONFIG_RTL_RX_NO_COPY)
static void rtl8169_rx_clear(struct rtl8169_private *tp)
{
	int i;

	for (i = 0; i < NUM_RX_DESC && tp->rx_databuff[i]; i++) {
		if (!tp->acp_enable)
			dma_unmap_single(tp_to_dev(tp),
					 le64_to_cpu(tp->rx_desc_array[i].addr),
					 rx_buf_sz, DMA_FROM_DEVICE);
		dev_kfree_skb(tp->rx_databuff[i]);
		tp->rx_databuff[i] = NULL;
		tp->rx_desc_array[i].addr = 0;
		tp->rx_desc_array[i].opts1 = 0;
	}
}
#else
static void rtl8169_rx_clear(struct rtl8169_private *tp)
{
	int i;

	for (i = 0; i < NUM_RX_DESC && tp->rx_databuff[i]; i++) {
		if (!tp->acp_enable)
			dma_unmap_page(tp_to_dev(tp),
				       le64_to_cpu(tp->rx_desc_array[i].addr),
				       rx_buf_sz, DMA_FROM_DEVICE);
		__free_pages(tp->rx_databuff[i], get_order(rx_buf_sz));
		tp->rx_databuff[i] = NULL;
		tp->rx_desc_array[i].addr = 0;
		tp->rx_desc_array[i].opts1 = 0;
	}
}
#endif /* CONFIG_RTL_RX_NO_COPY */

#if defined(CONFIG_RTL_RX_NO_COPY)
static int rtl8169_rx_skb_fill(struct rtl8169_private *tp, u32 start, u32 end)
{
	u32 cur;

	for (cur = start; end - cur > 0; cur++) {
		int ret, i = cur % NUM_RX_DESC;

		if (tp->rx_databuff[i])
			continue;
		ret = rtl8169_alloc_rx_skb(tp, tp->rx_databuff + i,
					   tp->rx_desc_array + i, rx_buf_sz);
		if (ret < 0)
			break;
		if (i == (NUM_RX_DESC - 1))
			tp->rx_desc_array[i].opts1 |= cpu_to_le32(RING_END);
	}
	return cur - start;
}
#else
static int rtl8169_rx_fill(struct rtl8169_private *tp)
{
	int i;

	for (i = 0; i < NUM_RX_DESC; i++) {
		struct page *data;

		data = rtl8169_alloc_rx_data(tp, tp->rx_desc_array + i);
		if (!data) {
			rtl8169_rx_clear(tp);
			return -ENOMEM;
		}
		tp->rx_databuff[i] = data;
	}

	/* mark as last descriptor in the ring */
	tp->rx_desc_array[NUM_RX_DESC - 1].opts1 |= cpu_to_le32(RING_END);

	return 0;
}
#endif /* CONFIG_RTL_RX_NO_COPY */

static int rtl8169_init_ring(struct rtl8169_private *tp)
{
	int ret;

	rtl8169_init_ring_indexes(tp);

	memset(tp->tx_skb, 0x0, sizeof(tp->tx_skb));
	memset(tp->rx_databuff, 0x0, sizeof(tp->rx_databuff));

#if defined(CONFIG_RTL_RX_NO_COPY)
	ret = rtl8169_rx_skb_fill(tp, 0, NUM_RX_DESC);
	if (ret < NUM_RX_DESC)
		ret = -ENOMEM;
	else
		ret = 0;
#else
	ret = rtl8169_rx_fill(tp);
#endif /* CONFIG_RTL_RX_NO_COPY */

	return ret;
}

static void rtl8169_unmap_tx_skb(struct rtl8169_private *tp, unsigned int entry)
{
	struct ring_info *tx_skb = tp->tx_skb + entry;
	struct tx_desc *desc = tp->tx_desc_array + entry;

	if (!tp->acp_enable)
		dma_unmap_single(tp_to_dev(tp), le64_to_cpu(desc->addr), tx_skb->len,
				 DMA_TO_DEVICE);

	memset(desc, 0, sizeof(*desc));
	memset(tx_skb, 0, sizeof(*tx_skb));
}

static void rtl8169_tx_clear_range(struct rtl8169_private *tp, u32 start,
				   unsigned int n)
{
	unsigned int i;

	for (i = 0; i < n; i++) {
		unsigned int entry = (start + i) % NUM_TX_DESC;
		struct ring_info *tx_skb = tp->tx_skb + entry;
		unsigned int len = tx_skb->len;

		if (len) {
			struct sk_buff *skb = tx_skb->skb;

			rtl8169_unmap_tx_skb(tp, entry);
			if (skb)
				dev_consume_skb_any(skb);
		}
	}
}

static void rtl8169_tx_clear(struct rtl8169_private *tp)
{
	rtl8169_tx_clear_range(tp, tp->dirty_tx, NUM_TX_DESC);
	netdev_reset_queue(tp->dev);
	tp->cur_tx = 0;
	tp->dirty_tx = 0;
}

static void rtl8169_cleanup(struct rtl8169_private *tp, bool going_down)
{
	napi_disable(&tp->napi);

	/* Give a racing hard_start_xmit a few cycles to complete. */
	synchronize_net();

	/* Disable interrupts */
	rtl8169_irq_mask_and_ack(tp);

	rtl_rx_close(tp);

	if (going_down && (tp->wol_enable & WOL_MAGIC))
		goto no_reset;

	rtl_enable_rxdvgate(tp);
	fsleep(2000);

	rtl_hw_reset(tp);

no_reset:
	rtl8169_tx_clear(tp);
	rtl8169_init_ring_indexes(tp);

	if (tp->chip->features & RTL_FEATURE_TX_NO_CLOSE) {
		/* disable tx no close mode */
		rtl_ocp_write(tp, 0xE610,
			      rtl_ocp_read(tp, 0xE610) & ~(BIT(4) | BIT(6)));
	};
}

static void rtl_reset_work(struct rtl8169_private *tp)
{
	struct net_device *dev = tp->dev;
	int i;

	netif_stop_queue(dev);

	rtl8169_cleanup(tp, false);

#if defined(CONFIG_RTL_RX_NO_COPY)
	rtl8169_rx_clear(tp);

	if (rx_buf_sz_new != rx_buf_sz)
		rx_buf_sz = rx_buf_sz_new;

	memset(tp->tx_desc_array, 0x0, NUM_TX_DESC * sizeof(struct tx_desc));
	for (i = 0; i < NUM_TX_DESC; i++) {
		if (i == (NUM_TX_DESC - 1))
			tp->tx_desc_array[i].opts1 = cpu_to_le32(RING_END);
	}
	memset(tp->rx_desc_array, 0x0, NUM_RX_DESC * sizeof(struct rx_desc));

	if (rtl8169_init_ring(tp) < 0) {
		napi_enable(&tp->napi);
		netif_wake_queue(dev);
		netdev_warn(tp->dev, "No memory. Try to restart......\n");
		msleep(1000);
		rtl_schedule_task(tp, RTL_FLAG_TASK_RESET_PENDING);
		return;
	}
#else
	for (i = 0; i < NUM_RX_DESC; i++)
		rtl8169_mark_to_asic(tp->rx_desc_array + i, rx_buf_sz);

#endif /* CONFIG_RTL_RX_NO_COPY */

	napi_enable(&tp->napi);
	rtl_hw_start(tp);

	netif_wake_queue(dev);
}

static void rtl8169_tx_timeout(struct net_device *dev, unsigned int txqueue)
{
	struct rtl8169_private *tp = netdev_priv(dev);

	rtl_schedule_task(tp, RTL_FLAG_TASK_TX_TIMEOUT);
}

static int rtl8169_tx_map(struct rtl8169_private *tp, const u32 *opts, u32 len,
			  void *addr, unsigned int entry, bool desc_own)
{
	struct tx_desc *txd = tp->tx_desc_array + entry;
	struct device *d = tp_to_dev(tp);
	dma_addr_t mapping;
	u32 opts1;
	int ret;

	if (tp->acp_enable) {
		mapping = virt_to_phys(addr);
	} else {
		mapping = dma_map_single(d, addr, len, DMA_TO_DEVICE);
		ret = dma_mapping_error(d, mapping);
		if (unlikely(ret)) {
			if (net_ratelimit())
				netdev_err(tp->dev, "Failed to map TX data!\n");
			return ret;
		}
	}

	txd->addr = cpu_to_le64(mapping);
	txd->opts2 = cpu_to_le32(opts[1]);

	opts1 = opts[0] | len;
	if (entry == NUM_TX_DESC - 1)
		opts1 |= RING_END;
	if (desc_own)
		opts1 |= DESC_OWN;
	txd->opts1 = cpu_to_le32(opts1);

	tp->tx_skb[entry].len = len;

	return 0;
}

static int rtl8169_xmit_frags(struct rtl8169_private *tp, struct sk_buff *skb,
			      const u32 *opts, unsigned int entry)
{
	struct skb_shared_info *info = skb_shinfo(skb);
	unsigned int cur_frag;

	for (cur_frag = 0; cur_frag < info->nr_frags; cur_frag++) {
		const skb_frag_t *frag = info->frags + cur_frag;
		void *addr = skb_frag_address(frag);
		u32 len = skb_frag_size(frag);

		entry = (entry + 1) % NUM_TX_DESC;

		if (unlikely(rtl8169_tx_map(tp, opts, len, addr, entry, true)))
			goto err_out;
	}

	return 0;

err_out:
	rtl8169_tx_clear_range(tp, tp->cur_tx + 1, cur_frag);
	return -EIO;
}

static inline bool rtl8169_tso_csum(struct rtl8169_private *tp,
				    struct sk_buff *skb, u32 *opts)
{
	u32 mss = skb_shinfo(skb)->gso_size;

	if (mss) {
		opts[0] |= TD_LSO;
		opts[1] |= min(mss, TD_MSS_MAX) << TD1_MSS_SHIFT;
	} else if (skb->ip_summed == CHECKSUM_PARTIAL) {
		const struct iphdr *ip = ip_hdr(skb);

		if (ip->protocol == IPPROTO_TCP)
			opts[1] |= TD1_IP_CS | TD1_TCP_CS;
		else if (ip->protocol == IPPROTO_UDP)
			opts[1] |= TD1_IP_CS | TD1_UDP_CS;
		else
			WARN_ON_ONCE(1);
	}
	return true;
}

static inline bool rtl_tx_slots_avail(struct rtl8169_private *tp)
{
	unsigned int slots_avail = READ_ONCE(tp->dirty_tx) + NUM_TX_DESC
					- READ_ONCE(tp->cur_tx);

	/* A skbuff with nr_frags needs nr_frags+1 entries in the tx queue */
	return slots_avail > MAX_SKB_FRAGS;
}

static inline void rtl8169_doorbell(struct rtl8169_private *tp)
{
	RTL_W8(tp, TX_POLL, NPQ);
}

static netdev_tx_t rtl8169_start_xmit(struct sk_buff *skb,
				      struct net_device *dev)
{
	unsigned int frags = skb_shinfo(skb)->nr_frags;
	struct rtl8169_private *tp = netdev_priv(dev);
	unsigned int entry = tp->cur_tx % NUM_TX_DESC;
	struct tx_desc *txd_first, *txd_last;
	bool stop_queue, door_bell;
	u32 opts[2];
	u16 close_idx;
	u16 tail_idx;

	if (unlikely(!rtl_tx_slots_avail(tp))) {
		if (net_ratelimit())
			netdev_err(dev, "BUG! Tx Ring full when queue awake!\n");
		goto err_stop_0;
	}

	txd_first = tp->tx_desc_array + entry;

	if (tp->chip->features & RTL_FEATURE_TX_NO_CLOSE) {
		close_idx = RTL_R16(tp, TX_DESC_CLOSE_IDX) & TX_DESC_CNT_MASK;
		tail_idx = READ_ONCE(tp->cur_tx) & TX_DESC_CNT_MASK;
		if ((tail_idx > close_idx && (tail_idx - close_idx == NUM_TX_DESC)) ||
		    (tail_idx < close_idx &&
		    (TX_DESC_CNT_SIZE - close_idx + tail_idx == NUM_TX_DESC)))
			goto err_stop_0;
	} else {
		if (unlikely(le32_to_cpu(txd_first->opts1) & DESC_OWN))
			goto err_stop_0;
	}

	opts[1] = cpu_to_le32(rtl8169_tx_vlan_tag(skb));
	opts[0] = 0;

	if (!rtl8169_tso_csum(tp, skb, opts))
		goto err_dma_0;

	if (unlikely(rtl8169_tx_map(tp, opts, skb_headlen(skb), skb->data,
				    entry, false)))
		goto err_dma_0;

	if (frags) {
		if (rtl8169_xmit_frags(tp, skb, opts, entry))
			goto err_dma_1;
		entry = (entry + frags) % NUM_TX_DESC;
	}

	txd_last = tp->tx_desc_array + entry;
	txd_last->opts1 |= cpu_to_le32(LAST_FRAG);
	tp->tx_skb[entry].skb = skb;

	skb_tx_timestamp(skb);

	/* Force memory writes to complete before releasing descriptor */
	dma_wmb();

	door_bell = __netdev_sent_queue(dev, skb->len, netdev_xmit_more());

	txd_first->opts1 |= cpu_to_le32(DESC_OWN | FIRST_FRAG);

	/* rtl_tx needs to see descriptor changes before updated tp->cur_tx */
	smp_wmb();

	WRITE_ONCE(tp->cur_tx, tp->cur_tx + frags + 1);

	if (tp->chip->features & RTL_FEATURE_TX_NO_CLOSE)
		RTL_W16(tp, TX_DESC_TAIL_IDX, READ_ONCE(tp->cur_tx) & TX_DESC_CNT_MASK);

	stop_queue = !rtl_tx_slots_avail(tp);
	if (unlikely(stop_queue)) {
		/* Avoid wrongly optimistic queue wake-up: rtl_tx thread must
		 * not miss a ring update when it notices a stopped queue.
		 */
		smp_wmb();
		netif_stop_queue(dev);

		/* Sync with rtl_tx:
		 * - publish queue status and cur_tx ring index (write barrier)
		 * - refresh dirty_tx ring index (read barrier).
		 * May the current thread have a pessimistic view of the ring
		 * status and forget to wake up queue, a racing rtl_tx thread
		 * can't.
		 */
		smp_mb__after_atomic();
		if (rtl_tx_slots_avail(tp))
			netif_start_queue(dev);
		door_bell = true;
	}

	if (door_bell)
		rtl8169_doorbell(tp);

	return NETDEV_TX_OK;

err_dma_1:
	rtl8169_unmap_tx_skb(tp, entry);
err_dma_0:
	dev_kfree_skb_any(skb);
	dev->stats.tx_dropped++;
	return NETDEV_TX_OK;

err_stop_0:
	netif_stop_queue(dev);
	dev->stats.tx_dropped++;
	return NETDEV_TX_BUSY;
}

static void rtl_tx(struct net_device *dev, struct rtl8169_private *tp, int budget)
{
	unsigned int dirty_tx, tx_left, bytes_compl = 0, pkts_compl = 0;
	u16 close_idx;
	u16 dirty_tx_idx;
	struct sk_buff *skb = NULL;

	dirty_tx = tp->dirty_tx;
	if (tp->chip->features & RTL_FEATURE_TX_NO_CLOSE) {
		close_idx = RTL_R16(tp, TX_DESC_CLOSE_IDX) & TX_DESC_CNT_MASK;
		dirty_tx_idx = dirty_tx & TX_DESC_CNT_MASK;
		smp_rmb(); /* make sure dirty_tx and close_idx are updated */

		if (dirty_tx_idx <= close_idx)
			tx_left = close_idx - dirty_tx_idx;
		else
			tx_left = close_idx + TX_DESC_CNT_SIZE - dirty_tx_idx;
	} else {
		smp_rmb(); /* make sure dirty_tx is updated */
		tx_left = READ_ONCE(tp->cur_tx) - dirty_tx;
	}

	while (tx_left > 0) {
		unsigned int entry = dirty_tx % NUM_TX_DESC;
		u32 status;

		status = le32_to_cpu(READ_ONCE(tp->tx_desc_array[entry].opts1));
		if (!(tp->chip->features & RTL_FEATURE_TX_NO_CLOSE) && (status & DESC_OWN))
			break;

		skb = tp->tx_skb[entry].skb;
		rtl8169_unmap_tx_skb(tp, entry);

		if (skb) {
			pkts_compl++;
			bytes_compl += skb->len;
			napi_consume_skb(skb, budget);
		}
		dirty_tx++;
		tx_left--;
	}

	if (tp->dirty_tx != dirty_tx) {
		netdev_completed_queue(dev, pkts_compl, bytes_compl);

		dev_sw_netstats_tx_add(dev, pkts_compl, bytes_compl);

		/* Sync with rtl8169_start_xmit:
		 * - publish dirty_tx ring index (write barrier)
		 * - refresh cur_tx ring index and queue status (read barrier)
		 * May the current thread miss the stopped queue condition,
		 * a racing xmit thread can only have a right view of the
		 * ring status.
		 */
		smp_store_mb(tp->dirty_tx, dirty_tx);
		if (netif_queue_stopped(dev) && rtl_tx_slots_avail(tp))
			netif_wake_queue(dev);

		/* 8168 hack: TX_POLL requests are lost when the Tx packets are
		 * too close. Let's kick an extra TX_POLL request when a burst
		 * of start_xmit activity is detected (if it is not detected,
		 * it is slow enough). -- FR
		 */
		if (READ_ONCE(tp->cur_tx) != dirty_tx && skb)
			rtl8169_doorbell(tp);
	}
}

static inline int rtl8169_fragmented_frame(u32 status)
{
	return (status & (FIRST_FRAG | LAST_FRAG)) != (FIRST_FRAG | LAST_FRAG);
}

static inline void rtl8169_rx_csum(struct sk_buff *skb, u32 opts1)
{
	u32 status = opts1 & (RX_PROTO_MASK | RX_CS_FAIL_MASK);

	if (status == RX_PROTO_TCP || status == RX_PROTO_UDP)
		skb->ip_summed = CHECKSUM_UNNECESSARY;
	else
		skb_checksum_none_assert(skb);
}

#if defined(CONFIG_RTL_RX_NO_COPY)
static int rtl_rx(struct net_device *dev, struct rtl8169_private *tp, int budget)
{
	struct device *d = tp_to_dev(tp);
	int count;
	int delta;

	for (count = 0; count < budget; count++, tp->cur_rx++) {
		unsigned int pkt_size, entry = tp->cur_rx % NUM_RX_DESC;
		struct rx_desc *desc = tp->rx_desc_array + entry;
		struct sk_buff *skb;
		dma_addr_t addr;
		u32 status;

		status = le32_to_cpu(READ_ONCE(desc->opts1));
		if (status & DESC_OWN)
			break;

		/* This barrier is needed to keep us from reading
		 * any other fields out of the Rx descriptor until
		 * we know the status of DescOwn
		 */
		dma_rmb();

		if (unlikely(status & RX_RES)) {
			if (net_ratelimit())
				netdev_warn(dev, "Rx ERROR. status = %08x\n",
					    status);
			dev->stats.rx_errors++;
			if (status & (RX_RWT | RX_RUNT))
				dev->stats.rx_length_errors++;
			if (status & RX_CRC)
				dev->stats.rx_crc_errors++;
			if (!(dev->features & NETIF_F_RXALL))
				goto release_descriptor;
			else if (status & RX_RWT || !(status & (RX_RUNT | RX_CRC)))
				goto release_descriptor;
		}

		pkt_size = status & GENMASK(13, 0);
		if (likely(!(dev->features & NETIF_F_RXFCS)))
			pkt_size -= ETH_FCS_LEN;

		/* The driver does not support incoming fragmented frames.
		 * They are seen as a symptom of over-mtu sized frames.
		 */
		if (unlikely(rtl8169_fragmented_frame(status))) {
			dev->stats.rx_dropped++;
			dev->stats.rx_length_errors++;
			goto release_descriptor;
		}

		skb = tp->rx_databuff[entry];
		addr = le64_to_cpu(desc->addr);

		if (!tp->acp_enable)
			dma_sync_single_for_cpu(d, addr, pkt_size, DMA_FROM_DEVICE);
		tp->rx_databuff[entry] = NULL;
		skb->tail += pkt_size;
		skb->len = pkt_size;
		if (!tp->acp_enable)
			dma_unmap_single(d, addr, pkt_size, DMA_FROM_DEVICE);

		rtl8169_rx_csum(skb, status);
		skb->protocol = eth_type_trans(skb, dev);

		rtl8169_rx_vlan_tag(desc, skb);

		if (skb->pkt_type == PACKET_MULTICAST)
			dev->stats.multicast++;

		napi_gro_receive(&tp->napi, skb);

		dev_sw_netstats_rx_add(dev, pkt_size);
release_descriptor:
		rtl8169_mark_to_asic(desc, rx_buf_sz);
	}

	delta = rtl8169_rx_skb_fill(tp, tp->dirty_rx, tp->cur_rx);
	tp->dirty_rx += delta;

	if (tp->dirty_rx + NUM_RX_DESC == tp->cur_rx) {
		rtl_schedule_task(tp, RTL_FLAG_TASK_RESET_PENDING);
		if (net_ratelimit())
			netdev_err(tp->dev, "%s: Rx buffers exhausted\n", dev->name);
	}

	return count;
}
#else
static int rtl_rx(struct net_device *dev, struct rtl8169_private *tp, int budget)
{
	struct device *d = tp_to_dev(tp);
	int count;

	for (count = 0; count < budget; count++, tp->cur_rx++) {
		unsigned int pkt_size, entry = tp->cur_rx % NUM_RX_DESC;
		struct rx_desc *desc = tp->rx_desc_array + entry;
		struct sk_buff *skb;
		const void *rx_buf;
		dma_addr_t addr;
		u32 status;

		status = le32_to_cpu(READ_ONCE(desc->opts1));
		if (status & DESC_OWN)
			break;

		/* This barrier is needed to keep us from reading
		 * any other fields out of the Rx descriptor until
		 * we know the status of DescOwn
		 */
		dma_rmb();

		if (unlikely(status & RX_RES)) {
			if (net_ratelimit())
				netdev_warn(dev, "Rx ERROR. status = %08x\n",
					    status);
			dev->stats.rx_errors++;
			if (status & (RX_RWT | RX_RUNT))
				dev->stats.rx_length_errors++;
			if (status & RX_CRC)
				dev->stats.rx_crc_errors++;
			if (!(dev->features & NETIF_F_RXALL))
				goto release_descriptor;
			else if (status & RX_RWT || !(status & (RX_RUNT | RX_CRC)))
				goto release_descriptor;
		}

		pkt_size = status & GENMASK(13, 0);
		if (likely(!(dev->features & NETIF_F_RXFCS)))
			pkt_size -= ETH_FCS_LEN;

		/* The driver does not support incoming fragmented frames.
		 * They are seen as a symptom of over-mtu sized frames.
		 */
		if (unlikely(rtl8169_fragmented_frame(status))) {
			dev->stats.rx_dropped++;
			dev->stats.rx_length_errors++;
			goto release_descriptor;
		}

		skb = napi_alloc_skb(&tp->napi, pkt_size);
		if (unlikely(!skb)) {
			dev->stats.rx_dropped++;
			goto release_descriptor;
		}

		addr = le64_to_cpu(desc->addr);
		rx_buf = page_address(tp->rx_databuff[entry]);

		if (!tp->acp_enable)
			dma_sync_single_for_cpu(d, addr, pkt_size, DMA_FROM_DEVICE);
		prefetch(rx_buf);
		skb_copy_to_linear_data(skb, rx_buf, pkt_size);
		skb->tail += pkt_size;
		skb->len = pkt_size;
		if (!tp->acp_enable)
			dma_sync_single_for_device(d, addr, pkt_size, DMA_FROM_DEVICE);

		rtl8169_rx_csum(skb, status);
		skb->protocol = eth_type_trans(skb, dev);

		rtl8169_rx_vlan_tag(desc, skb);

		if (skb->pkt_type == PACKET_MULTICAST)
			dev->stats.multicast++;

		napi_gro_receive(&tp->napi, skb);

		dev_sw_netstats_rx_add(dev, pkt_size);
release_descriptor:
		rtl8169_mark_to_asic(desc, rx_buf_sz);
	}

	return count;
}
#endif /* CONFIG_RTL_RX_NO_COPY */

static irqreturn_t rtl8169_interrupt(int irq, void *dev_instance)
{
	struct rtl8169_private *tp = dev_instance;
	u16 status = rtl_get_events(tp);

	if (status == 0xffff || !(status & tp->irq_mask))
		return IRQ_NONE;

	if (status & LINK_CHG)
		phy_mac_interrupt(tp->phydev);

	rtl_irq_disable(tp);
	napi_schedule(&tp->napi);

	return IRQ_HANDLED;
}

/* Workqueue context.
 */
static void rtl_task(struct work_struct *work)
{
	static const struct {
		int bitnr;
		void (*action)(struct rtl8169_private *tp);
	} rtl_work[] = {
		{ RTL_FLAG_TASK_RESET_PENDING,	rtl_reset_work },
		{ RTL_FLAG_TASK_TX_TIMEOUT,	rtl_mac_reinit }
	};
	struct rtl8169_private *tp =
		container_of(work, struct rtl8169_private, wk.work);
	struct net_device *dev = tp->dev;
	int i;

	rtl_lock_work(tp);

	if (!netif_running(dev) ||
	    !test_bit(RTL_FLAG_TASK_ENABLED, tp->wk.flags))
		goto out_unlock;

	for (i = 0; i < ARRAY_SIZE(rtl_work); i++) {
		bool pending;

		pending = test_and_clear_bit(rtl_work[i].bitnr, tp->wk.flags);
		if (pending)
			rtl_work[i].action(tp);
	}

out_unlock:
	rtl_unlock_work(tp);
}

static int rtl8169_poll(struct napi_struct *napi, int budget)
{
	struct rtl8169_private *tp = container_of(napi, struct rtl8169_private, napi);
	struct net_device *dev = tp->dev;
	int work_done = 0;
	u16 status = rtl_get_events(tp);

#if defined(CONFIG_RTL_RX_NO_COPY)
	u32 old_dirty_rx;
#endif /* CONFIG_RTL_RX_NO_COPY */

	rtl_ack_events(tp, status & ~RX_OVERFLOW);

	if (unlikely(status & SYS_ERR))
		goto out;

	rtl_tx(dev, tp, budget);

#if defined(CONFIG_RTL_RX_NO_COPY)
	old_dirty_rx = tp->dirty_rx;

	/* always check rx queue */
	work_done = rtl_rx(dev, tp, budget);

	if ((status & RX_OVERFLOW) && tp->dirty_rx != old_dirty_rx)
		rtl_ack_events(tp, RX_OVERFLOW);
#else
	/* always check rx queue */
	work_done = rtl_rx(dev, tp, budget);

	if ((status & RX_OVERFLOW) && work_done > 0)
		rtl_ack_events(tp, RX_OVERFLOW);

#endif /* CONFIG_RTL_RX_NO_COPY */

	if (work_done < budget && napi_complete_done(napi, work_done))
		rtl_irq_enable(tp);

out:
	return work_done;
}

static void r8169_phylink_handler(struct net_device *ndev)
{
	struct rtl8169_private *tp = netdev_priv(ndev);
	struct device *d = tp_to_dev(tp);

	if (netif_carrier_ok(ndev)) {
		pm_request_resume(d);

		/* prevent ALDPS enter MAC Powercut Tx/Rx disable */
		/* use MAC reset to set counter to offset 0 */
		rtl_schedule_task(tp, RTL_FLAG_TASK_RESET_PENDING);
	} else {
		pm_runtime_idle(d);
	}

	phy_print_status(tp->phydev);
}

static int r8169_phy_connect(struct rtl8169_private *tp)
{
	struct phy_device *phydev = tp->phydev;
	phy_interface_t phy_mode;
	int ret;

	switch (tp->output_mode) {
	case OUTPUT_RGMII_TO_MAC:
	case OUTPUT_RGMII_TO_PHY:
		phy_mode = PHY_INTERFACE_MODE_RGMII;
		break;
	case OUTPUT_SGMII_TO_MAC:
	case OUTPUT_SGMII_TO_PHY:
		phy_mode = PHY_INTERFACE_MODE_SGMII;
		break;
	case OUTPUT_RMII:
		phy_mode = PHY_INTERFACE_MODE_RMII;
		break;
	default:
		phy_mode = tp->chip->features & RTL_FEATURE_GMII ?
			   PHY_INTERFACE_MODE_GMII : PHY_INTERFACE_MODE_MII;
	}

	ret = phy_connect_direct(tp->dev, phydev, r8169_phylink_handler,
				 phy_mode);
	if (ret)
		return ret;

	if (tp->output_mode == OUTPUT_EMBEDDED_PHY &&
	    !(tp->chip->features & RTL_FEATURE_GMII))
		phy_set_max_speed(phydev, SPEED_100);

	phy_attached_info(phydev);

	return 0;
}

static void rtl8169_down(struct rtl8169_private *tp)
{
	/* Clear all task flags */
	bitmap_zero(tp->wk.flags, RTL_FLAG_MAX);

	/* disable EEE if it is enabled */
	if (tp->eee_enable)
		tp->chip->eee_set(tp, false);

	phy_stop(tp->phydev);

	rtl8169_update_counters(tp);

	rtl8169_cleanup(tp, true);

	rtl_prepare_power_down(tp);

	set_bit(RTL_STATUS_DOWN, tp->status);
}

static void rtl8169_up(struct rtl8169_private *tp)
{
	phy_init_hw(tp->phydev);
	phy_resume(tp->phydev);
	if (test_and_clear_bit(RTL_STATUS_DOWN, tp->status))
		rtl8169_init_phy(tp);
	napi_enable(&tp->napi);
	set_bit(RTL_FLAG_TASK_ENABLED, tp->wk.flags);
	rtl_schedule_task(tp, RTL_FLAG_TASK_RESET_PENDING);

	phy_start(tp->phydev);
}

static int __rtl8169_close(struct net_device *dev)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	struct platform_device *pdev = tp->pdev;

	pm_runtime_get_sync(&pdev->dev);

	netif_stop_queue(dev);

	rtl8169_down(tp);

	rtl8169_rx_clear(tp);

	// Todo: due to Android GKI symbol issues, this function is temporarily removed
	//cancel_work(&tp->wk.work);

	free_irq(dev->irq, tp);

	phy_disconnect(tp->phydev);

	if (tp->acp_enable) {
		kfree(tp->rx_desc_array);
		kfree(tp->tx_desc_array);
	} else {
		dma_free_coherent(&pdev->dev, R8169_RX_RING_BYTES,
				  tp->rx_desc_array, tp->rx_phy_addr);
		dma_free_coherent(&pdev->dev, R8169_TX_RING_BYTES,
				  tp->tx_desc_array, tp->tx_phy_addr);
	}
	tp->tx_desc_array = NULL;
	tp->rx_desc_array = NULL;

	pm_runtime_put_sync(&pdev->dev);

	return 0;
}

static int rtl8169_close(struct net_device *dev)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	int ret;

	rtl_lock_work(tp);
	ret = __rtl8169_close(dev);
	rtl_unlock_work(tp);
	return ret;
}

#ifdef CONFIG_NET_POLL_CONTROLLER
static void rtl8169_netpoll(struct net_device *dev)
{
	struct rtl8169_private *tp = netdev_priv(dev);

	rtl8169_interrupt(dev->irq, tp);
}
#endif

static int rtl_open(struct net_device *dev)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	struct platform_device *pdev = tp->pdev;
	int retval = -ENOMEM;
	int node = dev->dev.parent ? dev_to_node(dev->dev.parent) : -1;

	pm_runtime_get_sync(&pdev->dev);

	/* Rx and Tx descriptors needs 256 bytes alignment.
	 * dma_alloc_coherent provides more.
	 */
	if (tp->acp_enable) {
		tp->tx_desc_array = kzalloc_node(R8169_TX_RING_BYTES,
						 GFP_KERNEL, node);
		tp->tx_phy_addr = virt_to_phys(tp->tx_desc_array);
	} else {
		tp->tx_desc_array = dma_alloc_coherent(&pdev->dev,
						       R8169_TX_RING_BYTES,
						       &tp->tx_phy_addr,
						       GFP_KERNEL);
	}
	if (!tp->tx_desc_array)
		goto out;

	if (tp->acp_enable) {
		tp->rx_desc_array = kzalloc_node(R8169_RX_RING_BYTES,
						 GFP_KERNEL, node);
		tp->rx_phy_addr = virt_to_phys(tp->rx_desc_array);
	} else {
		tp->rx_desc_array = dma_alloc_coherent(&pdev->dev,
						       R8169_RX_RING_BYTES,
						       &tp->rx_phy_addr,
						       GFP_KERNEL);
	}
	if (!tp->rx_desc_array)
		goto err_free_tx_0;

	retval = rtl8169_init_ring(tp);
	if (retval < 0)
		goto err_free_rx_1;

	smp_mb(); /* make sure TX/RX rings are ready */

	retval = request_irq(dev->irq, rtl8169_interrupt, 0, dev->name, tp);
	if (retval < 0)
		goto err_free_rx_2;

	retval = r8169_phy_connect(tp);
	if (retval)
		goto err_free_irq;

	rtl8169_up(tp);

	rtl8169_init_counter_offsets(tp);

	netif_start_queue(dev);

out:
	pm_runtime_put_sync(&pdev->dev);

	return retval;

err_free_irq:
	free_irq(dev->irq, tp);
err_free_rx_2:
	rtl8169_rx_clear(tp);
err_free_rx_1:
	if (tp->acp_enable)
		kfree(tp->rx_desc_array);
	else
		dma_free_coherent(&pdev->dev, R8169_RX_RING_BYTES,
				  tp->rx_desc_array, tp->rx_phy_addr);
	tp->rx_desc_array = NULL;
err_free_tx_0:
	if (tp->acp_enable)
		kfree(tp->tx_desc_array);
	else
		dma_free_coherent(&pdev->dev, R8169_TX_RING_BYTES,
				  tp->tx_desc_array, tp->tx_phy_addr);
	tp->tx_desc_array = NULL;
	goto out;
}

static void
rtl8169_get_stats64(struct net_device *dev, struct rtnl_link_stats64 *stats)
{
	struct rtl8169_private *tp = netdev_priv(dev);
	struct device *d = tp_to_dev(tp);
	struct rtl8169_counters *counters = tp->counters;

	pm_runtime_get_noresume(d);

	netdev_stats_to_stats64(stats, &dev->stats);
	dev_fetch_sw_netstats(stats, dev->tstats);

	/* Fetch additional counter values missing in stats collected by driver
	 * from tally counters.
	 */
	if (pm_runtime_active(d))
		rtl8169_update_counters(tp);

	stats->tx_errors = le64_to_cpu(counters->tx_errors);
	stats->collisions = le32_to_cpu(counters->tx_multi_collision);
	stats->tx_aborted_errors = le16_to_cpu(counters->tx_aborted);
	stats->rx_missed_errors = le16_to_cpu(counters->rx_missed);

	pm_runtime_put_noidle(d);
}

/* protocol offload driver flow */
static void rtl8169_protocol_offload(struct rtl8169_private *tp)
{
	u32 tmp;

	/* Set Now_is_OOB = 0 */
	RTL_W8(tp, MCU, RTL_R8(tp, MCU) & ~NOW_IS_OOB);

	/* Flow control related (only for OOB) */
	/* RX FIFO threshold */
	rtl_ocp_write(tp, 0xC0A0, 0x0003);
	rtl_ocp_write(tp, 0xC0A2, 0x0180);
	rtl_ocp_write(tp, 0xC0A4, 0x004A);
	rtl_ocp_write(tp, 0xC0A8, 0x005A);

	/* Turn off RCR (IO offset 0x44 set to 0) */
	rtl_rx_close(tp);

	/* Set rxdv_gated_en (IO 0xF2 bit3 = 1) */
	/* check FIFO empty (IO 0xD2 bit[12:13]) */
	rtl_enable_rxdvgate(tp);

	/* disable Tx/Rx enable = 0 (IO 0x36 bit[10:11]=0b) */
	RTL_W8(tp, CHIP_CMD, RTL_R8(tp, CHIP_CMD) & ~(CMD_TX_ENB | CMD_RX_ENB));
	fsleep(1000);

	/* check link_list ready =1 (IO 0xD2 bit9=1) */
	rtl_loop_wait_high(tp, &rtl_link_list_ready_cond, 100, 42);

	/* set re_ini_ll =1 (MACOCP : 0xE8DE bit15=1) */
	tmp = rtl_ocp_read(tp, 0xE8DE);
	tmp |= BIT(15);
	rtl_ocp_write(tp, 0xE8DE, tmp);

	/* check link_list ready =1 (IO 0xD2 bit9=1) */
	rtl_loop_wait_high(tp, &rtl_link_list_ready_cond, 100, 42);

	/* Setting RMS (IO offset 0xdb-0xda set to 0x05F3) */
	rtl_set_rx_max_size(tp, 0x05F3);

	/* Enable VLAN De-tagging (IO offset 0xE0 bit6 set to 1) */
	RTL_W16(tp, C_PLUS_CMD, RTL_R16(tp, C_PLUS_CMD) | RX_VLAN);

	/* Enable now_is_oob (IO offset 0xd3 bit 7 set to 1b) */
	RTL_W8(tp, MCU, RTL_R8(tp, MCU) | NOW_IS_OOB);

	/*  set  MACOCP 0xE8DE bit14 mcu_borw_en to 1b for modifying ShareFIFO's points */
	tmp = rtl_ocp_read(tp, 0xE8DE);
	tmp |= BIT(14); /* borrow 8K SRAM */
	rtl_ocp_write(tp, 0xE8DE, tmp);

	/* Patch code to share FIFO if need */

	/* Set rxdv_gated_en = 0 (IO 0xF2 bit3=0) */
	/* Turn on RCR (IO offset 0x44 set to 0x0e) */
	rtl_wol_enable_rx(tp);

	/* Set Multicast Registers to accept all addresses */
	RTL_W32(tp, MAR0, 0xFFFFFFFF);
	RTL_W32(tp, MAR0 + 4, 0xFFFFFFFF);
}

static void rtl8169_net_suspend(struct rtl8169_private *tp)
{
	netif_device_detach(tp->dev);

	if (netif_running(tp->dev))
		rtl8169_down(tp);
}

static void rtl_hw_initialize(struct rtl8169_private *tp)
{
	u32 data;

	rtl_enable_rxdvgate(tp);

	RTL_W8(tp, CHIP_CMD, RTL_R8(tp, CHIP_CMD) & ~(CMD_TX_ENB | CMD_RX_ENB));
	fsleep(1000);
	RTL_W8(tp, MCU, RTL_R8(tp, MCU) & ~NOW_IS_OOB);

	data = rtl_ocp_read(tp, 0xe8de);
	data &= ~BIT(14);
	rtl_ocp_write(tp, 0xe8de, data);

	rtl_loop_wait_high(tp, &rtl_link_list_ready_cond, 100, 42);

	data = rtl_ocp_read(tp, 0xe8de);
	data |= BIT(15);
	rtl_ocp_write(tp, 0xe8de, data);

	rtl_loop_wait_high(tp, &rtl_link_list_ready_cond, 100, 42);
}

static void rtl_reinit_mac_phy(struct rtl8169_private *tp)
{
	set_bit(RTL_STATUS_REINIT, tp->status);
	tp->chip->reset_phy_gmac(tp);
	tp->chip->acp_init(tp);
	tp->chip->pll_clock_init(tp);
	tp->chip->mac_mcu_patch(tp);	/* patch SRAM if necessary */
	tp->chip->mdio_init(tp);
	/* after r8169soc_mdio_init(),
	 * SGMII : tp->ext_phy == true  ==> external MDIO,
	 * RGMII : tp->ext_phy == true  ==> external MDIO,
	 * RMII  : tp->ext_phy == false ==> internal MDIO,
	 * FE PHY: tp->ext_phy == false ==> internal MDIO
	 */

	/* Enable ALDPS */
	rtl_phy_write(tp, 0x0a43, 24,
		      rtl_phy_read(tp, 0x0a43, 24) | BIT(2));

	rtl_init_rxcfg(tp);

	rtl8169_irq_mask_and_ack(tp);

	rtl_hw_initialize(tp);

	rtl_hw_reset(tp);

	rtl_unlock_config_regs(tp);
	RTL_W8(tp, CONFIG1, RTL_R8(tp, CONFIG1) | PM_ENABLE);
	RTL_W8(tp, CONFIG5, RTL_R8(tp, CONFIG5) & PME_STATUS);

	/* disable magic packet WOL */
	RTL_W8(tp, CONFIG3, RTL_R8(tp, CONFIG3) & ~MAGIC_PKT);
	rtl_lock_config_regs(tp);

	clear_bit(RTL_STATUS_REINIT, tp->status);

	/* Set MAC address */
	__rtl_rar_set(tp, tp->mac_addr);

	rtl_led_set(tp);
}

static void rtl_mac_reinit(struct rtl8169_private *tp)
{
	if (netif_running(tp->dev)) {
		tp->netif_is_running = true;
		netdev_info(tp->dev, "take %s down\n", tp->dev->name);
		__rtl8169_close(tp->dev);
	} else {
		tp->netif_is_running = false;
	}

	rtl_reinit_mac_phy(tp);

	if (tp->netif_is_running) {
		netdev_info(tp->dev, "bring %s up\n", tp->dev->name);
		rtl_open(tp->dev);
	}
}

static int rtl_loopback(struct rtl8169_private *tp, u32 mode, u32 len, u32 cnt)
{
	struct device *d = tp_to_dev(tp);
	struct sk_buff *skb;
	dma_addr_t mapping;
	struct tx_desc *txd;
	struct rx_desc *rxd;
	void *rx_buf;
	u32 rx_len;
	u32 rx_cmd;
	u32 i;
	u32 j;
	u32 phy_status;
	u32 tmp;
	int ret = 0;
	u8 buf[RX_BUF_SIZE];
	u8 pattern = 0x5A;
	u8 ori_output_mode = tp->output_mode;

	if (cnt == 0xFFFFFFFF) {
		dev_err(d, "loopback: cnt 0x%X (infinite loop) is not supported\n", cnt);
		cnt = 1;
	}

	if (netif_running(tp->dev)) {
		tp->netif_is_running = true;
		dev_dbg(d, "loopback: take %s down\n", tp->dev->name);
		__rtl8169_close(tp->dev);
	} else {
		tp->netif_is_running = false;
	}

	switch (tp->output_mode) {
	case OUTPUT_RGMII_TO_MAC:
	case OUTPUT_RGMII_TO_PHY:
		if (mode >= RTL_EXT_PHY_PCS_LOOPBACK)
			tp->output_mode = OUTPUT_RGMII_TO_PHY;
		else
			tp->output_mode = OUTPUT_EMBEDDED_PHY;
		break;
	case OUTPUT_SGMII_TO_MAC:
	case OUTPUT_SGMII_TO_PHY:
		if (mode >= RTL_EXT_PHY_PCS_LOOPBACK)
			tp->output_mode = OUTPUT_SGMII_TO_PHY;
		else
			tp->output_mode = OUTPUT_EMBEDDED_PHY;
		break;
	case OUTPUT_RMII:
		if (mode >= RTL_EXT_PHY_PCS_LOOPBACK)
			tp->output_mode = OUTPUT_RMII;
		else
			tp->output_mode = OUTPUT_EMBEDDED_PHY;
		break;
	case OUTPUT_FORCE_LINK:
	case OUTPUT_EMBEDDED_PHY:
	default:
		if (mode >= RTL_EXT_PHY_PCS_LOOPBACK) {
			tp->output_mode = OUTPUT_RGMII_TO_PHY;
			/* set default values of tx/rx delay */
			tp->rgmii.tx_delay = 1;
			tp->rgmii.rx_delay = 4;
		} else {
			tp->output_mode = OUTPUT_EMBEDDED_PHY;
		}
	}

	rtl_init_mdio_ops(tp);
	rtl_init_mmd_ops(tp);
	rtl_phy_write(tp, 0, 0, 0x8000); /* reset PHY */
	rtl_reinit_mac_phy(tp);
	rtl_lock_work(tp);

	set_bit(RTL_STATUS_LOOPBACK, tp->status);
	dev_info(d, "loopback: Loopback test (mode %d, len %d, cnt %u) on %s\n",
		 mode, len, cnt, tp->dev->name);

	tp->tx_desc_array = dma_alloc_coherent(d, R8169_TX_RING_BYTES,
					       &tp->tx_phy_addr, GFP_KERNEL);
	if (!tp->tx_desc_array) {
		ret = -ENOMEM;
		goto err_out;
	}

	tp->rx_desc_array = dma_alloc_coherent(d, R8169_RX_RING_BYTES,
					       &tp->rx_phy_addr, GFP_KERNEL);
	if (!tp->rx_desc_array) {
		ret = -ENOMEM;
		goto err_free_tx_0;
	}

	ret = rtl8169_init_ring(tp);
	if (ret < 0)
		goto err_free_rx_0;

	smp_mb(); /* make sure TX/RX rings are ready */

	rtl_unlock_config_regs(tp);

	RTL_W8(tp, MAX_TX_PACKET_SIZE, TX_PACKET_MAX);

	rtl_set_rx_max_size(tp, rx_buf_sz);

	tp->cp_cmd |= RTL_R16(tp, C_PLUS_CMD) | PKT_CNTR_DISABLE | INTT_1;

	/* Disable VLAN De-tagging */
	tp->cp_cmd &= ~RX_VLAN;

	RTL_W16(tp, C_PLUS_CMD, tp->cp_cmd);

	RTL_W16(tp, INTR_MITIGATE, 0x5151);

	rtl_set_rx_tx_desc_registers(tp);

	rtl_set_tx_config_registers(tp);

	/* TX FIFO threshold */
	rtl_ocp_write(tp, 0xE618, 0x0006);
	rtl_ocp_write(tp, 0xE61A, 0x0010);

	/* RX FIFO threshold */
	rtl_ocp_write(tp, 0xC0A0, 0x0002);
	rtl_ocp_write(tp, 0xC0A2, 0x0008);
	rtl_ocp_write(tp, 0xC0A4, 0x0088);
	rtl_ocp_write(tp, 0xC0A8, 0x00A8);

	RTL_W8(tp, CONFIG5, RTL_R8(tp, CONFIG5) & ~LAN_WAKE);
	RTL_W8(tp, CONFIG3, RTL_R8(tp, CONFIG3) & ~MAGIC_PKT);

	RTL_R8(tp, INTR_MASK);

	RTL_W32(tp, TX_CONFIG, RTL_R32(tp, TX_CONFIG) | TXCFG_AUTO_FIFO);

	rtl_reset_packet_filter(tp);
	rtl_eri_write(tp, 0x2f8, ERIAR_MASK_0011, 0x1d8f);

	rtl_disable_rxdvgate(tp);

	RTL_W8(tp, MAX_TX_PACKET_SIZE, EARLY_SIZE);

	rtl_eri_write(tp, 0xc0, ERIAR_MASK_0011, 0x0000);
	rtl_eri_write(tp, 0xb8, ERIAR_MASK_0011, 0x0000);

	/* Adjust EEE LED frequency */
	RTL_W8(tp, EEE_LED, RTL_R8(tp, EEE_LED) & ~0x07);

	rtl_w0w1_eri(tp, 0x2fc, 0x01, 0x06);
	rtl_eri_clear_bits(tp, 0x1b0, BIT(12));

	/* disable aspm and clock request before access ephy */
	RTL_W8(tp, CONFIG2, RTL_R8(tp, CONFIG2) & ~CLK_REQ_EN);
	RTL_W8(tp, CONFIG5, RTL_R8(tp, CONFIG5) & ~ASPM_EN);

	rtl_lock_config_regs(tp);

	RTL_W8(tp, CHIP_CMD, CMD_TX_ENB | CMD_RX_ENB);

	switch (mode) {
	case RTL_INT_PHY_PCS_LOOPBACK:
	case RTL_EXT_PHY_PCS_LOOPBACK:
		RTL_W32(tp, TX_CONFIG, (RTL_R32(tp, TX_CONFIG) & ~GENMASK(18, 13)));

		if ((rtl_phy_read(tp, 0, 4) & (BIT(7) | BIT(8))) &&
		    !(rtl_phy_read(tp, 0, 9) & BIT(9))) {
			rtl_phy_write(tp, 0, 0, 0x6100); /* 100 Mbps */
			rtl_phy_write(tp, 0, 4, 0x0181);
			dev_info(d, "loopback: PHY links up at 100 Mbps\n");
		} else {
			rtl_phy_write(tp, 0, 0, 0x4140); /* 1000 Mbps */
			rtl_phy_write(tp, 0, 4, 0x0001);
			dev_info(d, "loopback: PHY links up at 1000 Mbps\n");
		}
		for (i = 0; i < 50; i++) { /* wait up to 5 secs */
			fsleep(100000);
			phy_status = rtl_ocp_read(tp, 0xde40);
			if (phy_status & BIT(2)) {
				dev_dbg(d, "loopback: wait %dms to be link-up\n", (i + 1) * 100);
				break;
			}
		}
		if (i == 50) {
			dev_err(d, "loopback: wait PHY link-up timeout\n");
			ret = -ENODEV;
			goto err_free_rx_0;
		}
		break;
	case RTL_INT_PHY_REMOTE_LOOPBACK:
	case RTL_EXT_PHY_REMOTE_LOOPBACK:
		RTL_W32(tp, TX_CONFIG, (RTL_R32(tp, TX_CONFIG) & ~GENMASK(18, 13)));

		for (i = 0; i < 50; i++) { /* wait up to 5 secs */
			fsleep(100000);
			phy_status = rtl_ocp_read(tp, 0xde40);
			if (phy_status & BIT(2)) {
				dev_dbg(d, "loopback: wait %dms to be link-up\n", (i + 1) * 100);
				break;
			}
		}
		if (i == 50) {
			dev_err(d, "loopback: wait PHY link-up timeout\n");
			ret = -ENODEV;
			goto err_free_rx_0;
		}
		break;
	case RTL_MAC_LOOPBACK:
	default:
		RTL_W32(tp, TX_CONFIG, (RTL_R32(tp, TX_CONFIG) & ~GENMASK(18, 13)) |
			BIT(13) | BIT(14) | BIT(15) | BIT(17));
	}
	dev_dbg(d, "loopback: TX_CONFIG = 0x%08X\n", RTL_R32(tp, TX_CONFIG));
	dev_dbg(d, "loopback: PHY reg 2 = 0x%04X\n", rtl_phy_read(tp, 0, 2));
	dev_dbg(d, "loopback: PHY reg 3 = 0x%04X\n", rtl_phy_read(tp, 0, 3));

	if (mode != RTL_MAC_LOOPBACK) {
		phy_status = rtl_ocp_read(tp, 0xde40);
		dev_dbg(d, "OCP 0xDE40 phy_status = 0x%04x\n", phy_status);
		switch (tp->output_mode) {
		case OUTPUT_EMBEDDED_PHY:
		case OUTPUT_SGMII_TO_PHY:
		default:
			/* do nothing */
			break;
		case OUTPUT_RMII:
			/* adjust RMII interface setting, speed */
			tmp = rtl_ocp_read(tp, 0xea30) & ~(BIT(6) | BIT(5));
			switch (phy_status & 0x0030) { /* link speed */
			case 0x0000:
				/* 10M, RGMII clock speed = 2.5MHz */
				break;
			case 0x0010:
				/* 100M, RGMII clock speed = 25MHz */
				tmp |= BIT(5);
			}
			/* adjust RMII interface setting, duplex */
			if ((phy_status & BIT(3)) == 0)
				/* ETN spec, half duplex */
				tmp &= ~BIT(4);
			else	/* ETN spec, full duplex */
				tmp |= BIT(4);
			rtl_ocp_write(tp, 0xea30, tmp);
			dev_dbg(d, "OCP 0xEA30 = 0x%04x\n", tmp);
			break;
		case OUTPUT_RGMII_TO_PHY:
			/* adjust RGMII interface setting, duplex */
			tmp = rtl_ocp_read(tp, 0xea34) & ~(BIT(4) | BIT(3));
			switch (phy_status & 0x0030) { /* link speed */
			case 0x0000:
				/* 10M, RGMII clock speed = 2.5MHz */
				break;
			case 0x0010:
				/* 100M, RGMII clock speed = 25MHz */
				tmp |= BIT(3);
				break;
			case 0x0020:
				/* 1000M, RGMII clock speed = 125MHz */
				tmp |= BIT(4);
				break;
			}
			/* adjust RGMII interface setting, duplex */
			if ((phy_status & BIT(3)) == 0)
				/* ETN spec, half duplex */
				tmp &= ~BIT(2);
			else	/* ETN spec, full duplex */
				tmp |= BIT(2);
			rtl_ocp_write(tp, 0xea34, tmp);
			dev_dbg(d, "OCP 0xEA34 = 0x%04x\n", tmp);
		}
	}

	RTL_W16(tp, MULTI_INTR, RTL_R16(tp, MULTI_INTR) & 0xF000);

	RTL_W32(tp, RX_CONFIG, RTL_R32(tp, RX_CONFIG) | ACCEPT_MY_PHYS);

	skb = rtl_gen_loopback_pkt(tp, len);
	if (IS_ERR_OR_NULL(skb)) {
		ret = PTR_ERR(skb);
		dev_err(d, "loopback: can't allocate skb, ret 0x%x\n", ret);
		goto err_free_rx_0;
	}

	mapping = dma_map_single(d, skb->data, rx_buf_sz, DMA_TO_DEVICE);

	for (j = 0; j < cnt; j++) {
		txd = tp->tx_desc_array + (j % NUM_TX_DESC);
		rxd = tp->rx_desc_array + (j % NUM_RX_DESC);

		txd->addr = cpu_to_le64(mapping);
		txd->opts2 = 0;

		dma_sync_single_for_cpu(d, le64_to_cpu(mapping), len, DMA_TO_DEVICE);
		memset(skb->data + ETH_HLEN, pattern++, len - ETH_HLEN);
		dma_sync_single_for_device(d, le64_to_cpu(mapping), len, DMA_TO_DEVICE);

		/* Force memory writes to complete before releasing descriptor */
		dma_wmb();

		if ((j % NUM_TX_DESC) == (NUM_TX_DESC - 1))
			txd->opts1 = cpu_to_le32(DESC_OWN | RING_END |
						 FIRST_FRAG | LAST_FRAG | len);
		else
			txd->opts1 = cpu_to_le32(DESC_OWN | FIRST_FRAG | LAST_FRAG | len);

		/* rtl_tx needs to see descriptor changes before updated tp->cur_tx */
		smp_wmb();
		rtl8169_doorbell(tp);

		for (i = 0; i < 1000; i++) { /* wait up to 10 ms */
			fsleep(10);
			rx_cmd = le32_to_cpu(rxd->opts1);
			if ((rx_cmd & DESC_OWN) == 0) {
				dev_dbg(d, "loopback: receive a pkt after %dns\n", (i + 1) * 10);
				break;
			}
		}
		if (i == 1000) {
			dev_err(d, "loopback: wait pkt[%d] timeout\n", j);
			ret = -EIO;
			goto timeout;
		}

		rx_len = (rx_cmd & 0x3FFF) - 4;

		rx_buf = page_address(tp->rx_databuff[j % NUM_RX_DESC]);
		dma_sync_single_for_cpu(d, le64_to_cpu(rxd->addr), rx_len, DMA_FROM_DEVICE);
		prefetch(rx_buf);

		if (len == rx_len) {
			ret = memcmp(skb->data, rx_buf, rx_len);
			if (ret) {
				dev_err(d, "loopback: packet[%d] content mismatch, len = %d\n",
					j, len);
				i = min(len, 42);
				rtl_dump_hex_to_buf(skb->data, i, buf, RX_BUF_SIZE);
				dev_err(d, "tx pkt header = [%s]\n", buf);

				i = min(rx_len, 42);
				rtl_dump_hex_to_buf(rx_buf, i, buf, RX_BUF_SIZE);
				dev_err(d, "rx pkt header = [%s]\n", buf);

				ret = -EIO;
				goto timeout;
			} else if (j > 0 && ((j % 1000000) == 0)) {
				dev_err(d, "looback mode %d len %d loop %d pattern 0x%02x test OK\n",
					mode, len, j, pattern - 1);
				i = min(rx_len, 42);
				rtl_dump_hex_to_buf(rx_buf, i, buf, RX_BUF_SIZE);
				dev_dbg(d, "rx pkt header = [%s]\n", buf);
			}
		} else {
			dev_err(d, "packet[%d] size mismatch, tx len = %d, rx len = %d\n",
				j, len, rx_len);
			i = min(len, 42);
			rtl_dump_hex_to_buf(skb->data, i, buf, RX_BUF_SIZE);
			dev_err(d, "tx pkt header = [%s]\n", buf);

			i = min(rx_len, 42);
			rtl_dump_hex_to_buf(rx_buf, i, buf, RX_BUF_SIZE);
			dev_err(d, "rx pkt header = [%s]\n", buf);

			ret = -EIO;
			goto timeout;
		}

		dma_sync_single_for_device(d, le64_to_cpu(rxd->addr), rx_buf_sz, DMA_FROM_DEVICE);
		if ((j % NUM_RX_DESC) == (NUM_RX_DESC - 1))
			rxd->opts1 = cpu_to_le32(DESC_OWN | RING_END | rx_buf_sz);
		else
			rxd->opts1 = cpu_to_le32(DESC_OWN | rx_buf_sz);
	}

timeout:
	dev_err(d, "looback mode %d len %d loop %d test %s\n",
		mode, len, j, (j == cnt) ? "OK" : "FAIL");

	dma_unmap_single(d, le64_to_cpu(mapping), len, DMA_TO_DEVICE);
	dev_kfree_skb_any(skb);

err_free_rx_0:
	dma_free_coherent(d, R8169_RX_RING_BYTES,
			  tp->rx_desc_array, tp->rx_phy_addr);
	tp->rx_desc_array = NULL;

err_free_tx_0:
	dma_free_coherent(d, R8169_TX_RING_BYTES,
			  tp->tx_desc_array, tp->tx_phy_addr);
	tp->tx_desc_array = NULL;

err_out:
	/* recover network */
	tp->output_mode = ori_output_mode;
	rtl_init_mdio_ops(tp);
	rtl_init_mmd_ops(tp);
	rtl_unlock_work(tp);
	clear_bit(RTL_STATUS_LOOPBACK, tp->status);
	dev_dbg(d, "reinit ETN MAC\n");
	rtl_reinit_mac_phy(tp);

	if (tp->netif_is_running) {
		dev_dbg(d, "bring %s up\n", tp->dev->name);
		rtl_open(tp->dev);
	}

	return ret;
}

static int rtl8169_runtime_resume(struct device *dev)
{
	struct rtl8169_private *tp = dev_get_drvdata(dev);

	rtl_lock_work(tp);
	__rtl_rar_set(tp, tp->mac_addr);
	__rtl8169_set_wol(tp, tp->saved_wolopts);

	if (tp->tx_desc_array)
		rtl8169_up(tp);

	tp->chip->wakeup_set(tp, false);

	netif_device_attach(tp->dev);
	rtl_unlock_work(tp);

	return 0;
}

static int rtl8169_suspend(struct device *dev)
{
	struct rtl8169_private *tp = dev_get_drvdata(dev);
	struct pm_dev_param *dev_param;

	dev_param = rtk_pm_get_param(LAN);
	if (dev_param && (*(int *)dev_param->data == DCO_ENABLE))
		tp->pwr_saving = true;

	rtl_lock_work(tp);
	if (tp->pwr_saving) {
		if (netif_running(tp->dev)) {
			tp->netif_is_running = true;
			netdev_info(tp->dev, "take %s down\n", tp->dev->name);
			__rtl8169_close(tp->dev);
		} else {
			/* Clear all task flags */
			bitmap_zero(tp->wk.flags, RTL_FLAG_MAX);

			tp->netif_is_running = false;
		}
	} else {
		if (tp->wol_enable)
			rtl8169_protocol_offload(tp);

		rtl8169_net_suspend(tp);
	}

	/* turn off LED, and current solution is switch pad to GPIO input */
	if (tp->output_mode == OUTPUT_EMBEDDED_PHY)
		tp->chip->led_set(tp, false);
	rtl_unlock_work(tp);

	return 0;
}

static int rtl8169_resume(struct device *dev)
{
	struct rtl8169_private *tp = dev_get_drvdata(dev);

	if (tp->pwr_saving) {
		netdev_dbg(tp->dev, "enable ETN clk and reset\n");

		rtl_lock_work(tp);
		rtl_reinit_mac_phy(tp);

		if (tp->netif_is_running) {
			netdev_dbg(tp->dev, "bring %s up\n", tp->dev->name);
			rtl_open(tp->dev);
		}
		rtl_unlock_work(tp);
	} else {
		rtl8169_runtime_resume(dev);
	}

	rtl_lock_work(tp);
	/* turn on LED */
	if (tp->output_mode == OUTPUT_EMBEDDED_PHY &&
	    rtk_pm_get_wakeup_reason() != ALARM_EVENT)
		tp->chip->led_set(tp, true);
	rtl_unlock_work(tp);

	return 0;
}

static int rtl8169_runtime_suspend(struct device *dev)
{
	struct rtl8169_private *tp = dev_get_drvdata(dev);

	if (!tp->tx_desc_array) {
		netif_device_detach(tp->dev);
		return 0;
	}

	rtl_lock_work(tp);
	__rtl8169_set_wol(tp, WAKE_ANY);
	rtl8169_net_suspend(tp);
	rtl_unlock_work(tp);

	return 0;
}

static int rtl8169_runtime_idle(struct device *dev)
{
	struct rtl8169_private *tp = dev_get_drvdata(dev);

	return tp->tx_desc_array ? -EBUSY : 0;
}

static const struct dev_pm_ops rtl8169soc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(rtl8169_suspend, rtl8169_resume)
	SET_RUNTIME_PM_OPS(rtl8169_runtime_suspend, rtl8169_runtime_resume,
			   rtl8169_runtime_idle)
};

static void rtl_shutdown(struct platform_device *pdev)
{
	struct rtl8169_private *tp = platform_get_drvdata(pdev);

	rtl_lock_work(tp);
	rtl8169_net_suspend(tp);

	/* Restore original MAC address */
	__rtl_rar_set(tp, tp->dev->perm_addr);
	rtl_unlock_work(tp);
}

static struct phy_driver r1869soc_phy_drv;
static int rtl_remove_one(struct platform_device *pdev)
{
	struct rtl8169_private *tp = platform_get_drvdata(pdev);
	struct pm_dev_param *dev_param;

	pm_runtime_get_noresume(&pdev->dev);

#ifdef RTL_PROC
	do {
		rtl_proc_file_unregister(tp);

		if (!rtk_proc)
			break;

		remove_proc_entry(KBUILD_MODNAME, rtk_proc);
		remove_proc_entry(tp->dev->name, init_net.proc_net);

		rtk_proc = NULL;

	} while (0);
#endif

	dev_param = rtk_pm_get_param(LAN);
	if (dev_param)
		rtk_pm_del_list(dev_param);

	netif_napi_del(&tp->napi);

	cancel_work_sync(&tp->wk.work);

	// Todo: due to Android GKI symbol issues, this function is temporarily removed
	//phy_driver_unregister(&r1869soc_phy_drv);

	unregister_netdev(tp->dev);

	/* restore original MAC address */
	__rtl_rar_set(tp, tp->dev->perm_addr);

	iounmap(tp->mmio_addr);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct net_device_ops rtl_netdev_ops = {
	.ndo_open		= rtl_open,
	.ndo_stop		= rtl8169_close,
	.ndo_get_stats64	= rtl8169_get_stats64,
	.ndo_start_xmit		= rtl8169_start_xmit,
	.ndo_tx_timeout		= rtl8169_tx_timeout,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_change_mtu		= rtl8169_change_mtu,
	.ndo_fix_features	= rtl8169_fix_features,
	.ndo_set_features	= rtl8169_set_features,
	.ndo_set_mac_address	= rtl_set_mac_address,
	.ndo_siocdevprivate	= rtl8169soc_ioctl,
	.ndo_eth_ioctl		= phy_do_ioctl_running,
	.ndo_set_rx_mode	= rtl_set_rx_mode,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller	= rtl8169_netpoll,
#endif

};

/* Start of mDNS offload */
static int _mdns_mac_apply_offload_rr_raw_pkt(struct rtl8169_private *tp)
{
	union {
		char octet[SFF_RAW_PKT_SIZE];
		u16 word[SFF_RAW_PKT_SIZE / 2];
	} *raw_pkt;
	union {
		char octet[SFF_RR_SIZE];
		u16 word[SFF_RR_SIZE / 2];
	} *rr;
	union {
		char octet[2];
		u16 word;
	} qtype;
	int list_size = tp->mdns_data_list.list_size;
	int hex_packet_size;
	int i;
	int j;
	int data_len;
	int raw_pkt_len = 0;
	int rr_len = 0;
	int tmp_len;
	int qnlen;
	int ret = 0;
	u16 base_addr;
	u16 nameoffset;

	netdev_dbg(tp->dev, "try to allocate raw_pkt buffer %ld bytes\n", sizeof(*raw_pkt));
	raw_pkt = kzalloc(sizeof(*raw_pkt), GFP_KERNEL);
	if (!raw_pkt) {
		ret = -ENOMEM;
		goto err_out_no_raw_pkt;
	}

	netdev_dbg(tp->dev, "try to allocate rr buffer %ld bytes\n", sizeof(*rr));
	rr = kzalloc(sizeof(*rr), GFP_KERNEL);
	if (!rr) {
		ret = -ENOMEM;
		goto err_out_no_rr;
	}

	for (i = 0; i < list_size; i++) {
		/* raw packet */
		hex_packet_size = tp->mdns_data_list.list[i].packet_size;
		tmp_len = hex_packet_size / 2;
		if (raw_pkt_len + tmp_len > SFF_RAW_PKT_SIZE) {
			netdev_err(tp->dev, "%d: Exceed raw pkt list size, used %d, new pkt %d\n",
				   i, raw_pkt_len, tmp_len);
			ret = -ENOMEM;
			goto err_out;
		}
		data_len = rtl_hex2bin(tp->mdns_data_list.list[i].packet,
				       hex_packet_size, raw_pkt->octet + raw_pkt_len);
		if (data_len <= 0 || ((2 * data_len) != hex_packet_size)) {
			netdev_err(tp->dev, "Invalid packet[%d] = [%s], hex size %d, len = %d\n",
				   i, tp->mdns_data_list.list[i].packet,
				   hex_packet_size, data_len);
			ret = -EINVAL;
			goto err_out;
		}
		netdev_dbg(tp->dev, "Set raw packet[%d], len %d\n", i, data_len);

		/* RRs (qname and qtype) */
		for (j = 0; j < tp->mdns_data_list.list[i].type_size; j++) {
			nameoffset = tp->mdns_data_list.list[i].type[j].nameoffset;
			qnlen = _mdns_qname_copy(rr->octet + rr_len,
						 raw_pkt->octet + raw_pkt_len,
						 nameoffset);
			if (qnlen <= 0) {
				netdev_err(tp->dev, "Invalid qname[%d][%d] offset = %d, qnlen = %d\n",
					   i, j, nameoffset, qnlen);
				ret = -EINVAL;
				goto err_out;
			}
			netdev_dbg(tp->dev, "Set RR[%d][%d], qnlen %d, offset %d\n",
				   i, j, qnlen, nameoffset);
			rr_len += qnlen;

			/* qtype */
			qtype.word = cpu_to_be16(tp->mdns_data_list.list[i].type[j].qtype);
			rr->octet[rr_len] = qtype.octet[0];
			rr->octet[rr_len + 1] = qtype.octet[1];
			rr_len += 2;
		}

		raw_pkt_len += data_len;
	}

	/* write RRs to SFF */
	base_addr = SFF_RR_START_ADDR;
	tmp_len = (rr_len + 1) / 2;
	for (i = 0; i < tmp_len; i++)
		rtl_ocp_write(tp, base_addr + (2 * i), rr->word[i]);

	/* write raw packet to SFF */
	base_addr = SFF_RAW_PKT_START_ADDR;
	tmp_len = (raw_pkt_len + 1) / 2;
	for (i = 0; i < tmp_len; i++)
		rtl_ocp_write(tp, base_addr + (2 * i), raw_pkt->word[i]);

err_out:
	kfree(rr);
err_out_no_rr:
	kfree(raw_pkt);
err_out_no_raw_pkt:
	return ret;
}

static void _mdns_mac_apply_offload_option(struct rtl8169_private *tp)
{
	int list_size = tp->mdns_data_list.list_size;
	u16 base_addr;
	u16 offset;
	int i;
	int tmp;

	base_addr = SFF_RAW_PKT_OPT_START_ADDR;

	/* raw packet count */
	rtl_ocp_write(tp, base_addr, list_size);

	/* raw start address */
	rtl_ocp_write(tp, base_addr + 2, SFF_RAW_PKT_START_ADDR);

	/* raw RR count */
	tmp = 0;
	for (i = 0; i < list_size; i++)
		tmp += tp->mdns_data_list.list[i].type_size;
	rtl_ocp_write(tp, base_addr + 4, tmp);

	/* raw RR start address */
	rtl_ocp_write(tp, base_addr + 6, SFF_RR_START_ADDR);

	for (i = 0; i < list_size; i++) {
		/* RR count of the raw packet */
		offset = 8 + (4 * i);
		rtl_ocp_write(tp, base_addr + offset, tp->mdns_data_list.list[i].type_size);

		/* raw packet length */
		offset = 10 + (4 * i);
		rtl_ocp_write(tp, base_addr + offset, tp->mdns_data_list.list[i].packet_size / 2);
	}
}

static int _mdns_mac_apply_passthrough_rules(struct rtl8169_private *tp)
{
	union {
		char octet[SFF_PASSTHROUGH_SIZE];
		u16 word[SFF_PASSTHROUGH_SIZE / 2];
	} *passthrough;
	int passthrough_len = 0;
	int list_size = tp->passthrough_list.list_size;
	int i;
	int j;
	int *qnlen;
	int ret = 0;
	int tmp_len;
	u16 base_addr;

	if (list_size <= 0) {
		/* passthrough count */
		rtl_ocp_write(tp, SFF_PASSTHROUGH_OPT_START_ADDR, 0);
		goto err_out_no_passthrough;
	}

	netdev_dbg(tp->dev, "try to allocate passthrough buffer %ld bytes\n", sizeof(*passthrough));
	passthrough = kzalloc(sizeof(*passthrough), GFP_KERNEL);
	if (!passthrough) {
		ret = -ENOMEM;
		goto err_out_no_passthrough;
	}

	netdev_dbg(tp->dev, "try to allocate qnlen buffer %ld bytes\n", sizeof(qnlen));
	qnlen = kcalloc(list_size, sizeof(int), GFP_KERNEL);
	if (!qnlen) {
		ret = -ENOMEM;
		goto err_out_no_qnlen;
	}

	for (i = 0, j = 0; i < MDNS_PASSTHROUGH_MAX; i++) {
		if (tp->passthrough_list.list[i].qname_len == 0)
			continue;
		qnlen[j] = _mdns_set_qname(passthrough->octet + passthrough_len,
					   tp->passthrough_list.list[i].qname);

		if (qnlen[j] <= 1) {
			netdev_err(tp->dev, "%d: Invalid qname [%s], qnlen = %d\n",
				   i, tp->passthrough_list.list[i].qname, qnlen[j]);
			ret = -EINVAL;
			goto err_out;
		}

		passthrough_len += qnlen[j];
		if (++j == list_size)
			break;
	}

	/* passthrough qname */
	base_addr = SFF_PASSTHROUGH_START_ADDR;
	tmp_len = (passthrough_len + 1) / 2;
	for (i = 0; i < tmp_len; i++)
		rtl_ocp_write(tp, base_addr + (2 * i), passthrough->word[i]);

	/* passthrough count */
	rtl_ocp_write(tp, SFF_PASSTHROUGH_OPT_START_ADDR, list_size);

	base_addr = SFF_PASSTHROUGH_OPT_START_ADDR + 2;
	for (i = 0; i < list_size; i++)
		rtl_ocp_write(tp, base_addr + (2 * i), qnlen[i]);

err_out:
	kfree(qnlen);
err_out_no_qnlen:
	kfree(passthrough);
err_out_no_passthrough:
	/* passthrough function */
	rtl_ocp_write(tp, SFF_PASSTHROUGH_BEHAVIOR_ADDR, tp->passthrough_behavior);

	return ret;
}

static void _mdns_mac_apply_oob_ip(struct rtl8169_private *tp)
{
	u16 base_addr;
	int i;
	union {
		u32 addr;
		u16 word[2];
		u8 octet[4];
	} ipv4;
	union {
		u32 addr32[4];
		u16 word[8];
		u8 octet[16];
	} ipv6;

	/* OOB IPv4 */
	base_addr = OCP_OOB_IPV4_0;

	for (i = 0; i < 4; i++)
		ipv4.octet[i] = tp->g_wol_ipv4_addr[i];

	netdev_dbg(tp->dev, "ipv4 = %pI4\n", ipv4.octet);

	/* OOB_IPv4_0 */
	rtl_ocp_write(tp, base_addr, ipv4.word[0]);

	/* OOB_IPv4_1 */
	rtl_ocp_write(tp, base_addr + 2, ipv4.word[1]);

	/* OOB IPv6 */
	base_addr = OCP_NS0_OOB_LC0_IPV6_0;
	for (i = 0; i < 16; i++)
		ipv6.octet[i] = tp->g_wol_ipv6_addr[i];

	netdev_dbg(tp->dev, "ipv6 = %pI6\n", ipv6.octet);

	for (i = 0; i < 8; i++)
		rtl_ocp_write(tp, base_addr + (2 * i), ipv6.word[i]);
}

static void _mdns_mac_apply_mac_addr(struct rtl8169_private *tp)
{
	u16 base_addr;
	int i;
	union {
		u16 word[ETH_ALEN / 2];
		u8 octet[ETH_ALEN];
	} mac_addr;

	/* src mac addr of the response */
	base_addr = OCP_NS0_MACID_0;

	for (i = 0; i < ETH_ALEN; i++)
		mac_addr.octet[i] = tp->mac_addr[i];

	netdev_dbg(tp->dev, "src MAC addr = %pM\n", mac_addr.octet);

	for (i = 0; i < (ETH_ALEN / 2); i++)
		rtl_ocp_write(tp, base_addr + (2 * i), mac_addr.word[i]);

	/* acpt_mar_r */
	/* equal to IO RCR_0 */
	/* backup the original OCP_RCR_0 */
	tp->ocp_rcr_0_bak = rtl_ocp_read(tp, OCP_RCR_0);
	rtl_ocp_write(tp, OCP_RCR_0, 6);

	/* cfg_en_rxtp_r */
	rtl_ocp_write(tp, OCP_CFG_EN_MAGIC, 0x04ff);

	/* rsvd5_port = swap(5353) = swap(0x14e9) = 0xe914 */
	rtl_ocp_write(tp, OCP_L4_PORT_5, 0xe914);

	/* proxy udp port sel */
	rtl_ocp_write(tp, OCP_PROXY_PORT_MATCH_SEL, 0x20);

	/* proxy udp match en */
	rtl_ocp_write(tp, OCP_OOB_PORT_MATCH_EN, 0x100);

	/* MAR0 ~ MAR7 */
	/* equal to IO MAR0 */
	base_addr = OCP_MAR0;
	/* backup the original OCP_MAR0 ~ 7 */
	for (i = 0; i < 4; i++)
		tp->ocp_mar_bak[i] = rtl_ocp_read(tp, base_addr + (2 * i));

	for (i = 0; i < 4; i++)
		rtl_ocp_write(tp, base_addr + (2 * i), 0xffff);

	i = rtl_ocp_read(tp, OCP_FTR_MCU_CTRL) & ~BIT(0);
	rtl_ocp_write(tp, OCP_FTR_MCU_CTRL, i);
	rtl_ocp_write(tp, OCP_FTR_MCU_CTRL, i | BIT(0));
}

void rtd1625_mdns_mac_mcu_patch(struct rtl8169_private *tp)
{
	u16 base_addr;
	int i;
	int len;
	int page;

	/* 1.a enable now_is_oob */
	RTL_W8(tp, MCU, RTL_R8(tp, MCU) | NOW_IS_OOB);

	/* disable break point */
	rtl_ocp_write(tp, 0xfc28, 0);
	rtl_ocp_write(tp, 0xfc2a, 0);
	rtl_ocp_write(tp, 0xfc2c, 0);
	rtl_ocp_write(tp, 0xfc2e, 0);
	rtl_ocp_write(tp, 0xfc30, 0);
	rtl_ocp_write(tp, 0xfc32, 0);
	rtl_ocp_write(tp, 0xfc34, 0);
	rtl_ocp_write(tp, 0xfc36, 0);
	fsleep(3000);

	/* disable base address */
	rtl_ocp_write(tp, 0xfc26, 0);

	/* patch ISRAM */
	len = rtd1625_isram_len / SARM_PATCH_WORD_SIZE;
	if (len > SRAM_PATCH_ARRAY_MAX)
		netdev_err(tp->dev, "SRAM patch size overflow\n");
	page = 0;
	base_addr = 0xf800;
	for (i = 0; i < len; i++) {
		/* check 1KB boundary, 1024 bytes = 512 words */
		if ((i & SRAM_PATCH_PAGE_MASK) == 0) {
			page = i / SRAM_PATCH_PAGE_SIZE;
			rtl_ocp_write(tp, 0xe440, page);
		}
		rtl_ocp_write(tp, base_addr + (2 * i) - (1024 * page), rtd1625_isram[i]);
	}

	/* patch share FIFO OCP 0x6000 ~ 0x67FF if necessary */

	/* enable base address */
	rtl_ocp_write(tp, 0xfc26, 0x8000);
}

void rtl8169soc_mdns_mac_offload_set(struct rtl8169_private *tp)
{
	/* Set mNDS wake reason = 0 */
	rtl_ocp_write(tp, SFF_WAKEUP_REASON_ADDR, 0);

	/* clear wol_task toggle bit */
	rtl_ocp_write(tp, 0xd2ea, 0);

	/* write offloaded RR & raw packets */
	_mdns_mac_apply_offload_rr_raw_pkt(tp);
	_mdns_mac_apply_offload_option(tp);
	_mdns_mac_apply_passthrough_rules(tp);

	_mdns_mac_apply_oob_ip(tp);
	_mdns_mac_apply_mac_addr(tp);

	/* enable breakpoint */
	rtl_ocp_write(tp, 0xfc28, 0x0e91);
}

static int _mdns_mac_upload_wakeup_pkt(struct rtl8169_private *tp)
{
	u32 tmp;
	unsigned int pkt_size = 0;
	struct sk_buff *skb = NULL;
	u16 base_addr;
	char *p;
	int i;

	/* check wakeup packet length */
	pkt_size = rtl_ocp_read(tp, SFF_WAKEUP_PKT_LEN_ADDR) & 0xffff;
	netdev_dbg(tp->dev, "mDNS wakeup pkt size = 0x%04x\n", pkt_size);
	if (pkt_size <= 64 || pkt_size > 1536)
		netdev_err(tp->dev, "abnormal mDNS wakeup pkt size = %d\n", pkt_size);
	else
		skb = netdev_alloc_skb_ip_align(tp->dev, pkt_size + 2);
	if (!skb)
		return -ENOMEM;

	p = skb->data;
	base_addr = 0x7000;
	for (i = 0; i < (pkt_size + 2); i += 2) {
		tmp = rtl_ocp_read(tp, base_addr + i) & 0xffff;
		*(u16 *)(p + i) = tmp;
	}

	skb->ip_summed = CHECKSUM_UNNECESSARY;
	skb_put(skb, pkt_size);
	skb->protocol = eth_type_trans(skb, tp->dev);
	napi_gro_receive(&tp->napi, skb);
	return 0;
}

void rtl8169soc_mdns_mac_offload_unset(struct rtl8169_private *tp)
{
	u16 base_addr;
	u32 tmp;
	u32 i;

	base_addr = SFF_RAW_PKT_OPT_START_ADDR;

	/* disable mDNS settings */

	/* disable breakpoint */
	rtl_ocp_write(tp, 0xfc28, 0);

	/* raw packet count */
	rtl_ocp_write(tp, base_addr, 0);

	/* raw start address */
	rtl_ocp_write(tp, base_addr + 2, 0);

	/* raw RR start address */
	rtl_ocp_write(tp, base_addr + 6, 0);

	/* passthrough count */
	rtl_ocp_write(tp, SFF_PASSTHROUGH_OPT_START_ADDR, 0);

	/* passthrough function */
	rtl_ocp_write(tp, SFF_PASSTHROUGH_BEHAVIOR_ADDR, DROP_ALL);

	/* check wakeup reason */
	tmp = rtl_ocp_read(tp, SFF_WAKEUP_REASON_ADDR) & 0xffff;
	netdev_dbg(tp->dev, "mDNS wakeup reason = 0x%04x\n", tmp);
	if (tmp & 0x3)
		_mdns_mac_upload_wakeup_pkt(tp);

	/* restore the original OCP_RCR_0 */
	rtl_ocp_write(tp, OCP_RCR_0, tp->ocp_rcr_0_bak);
	/* restore the original OCP_MAR0 ~ 7 */
	base_addr = OCP_MAR0;
	for (i = 0; i < 4; i++)
		rtl_ocp_write(tp, base_addr + (2 * i), tp->ocp_mar_bak[i]);
}

/* End of mDNS offload */

static inline void rtl_set_irq_mask(struct rtl8169_private *tp)
{
	tp->irq_mask = RX_OK | RX_ERR | RX_OVERFLOW | TX_OK | TX_ERR | SYS_ERR | LINK_CHG;
}

static int r8169_mdio_read_reg(struct mii_bus *mii_bus, int phyaddr, int phyreg)
{
	struct rtl8169_private *tp = mii_bus->priv;

	if (phyaddr > 0)
		return -ENODEV;

	return rtl_phy_read(tp, CURRENT_MDIO_PAGE, phyreg);
}

static int r8169_mdio_write_reg(struct mii_bus *mii_bus, int phyaddr,
				int phyreg, u16 val)
{
	struct rtl8169_private *tp = mii_bus->priv;

	if (phyaddr > 0)
		return -ENODEV;

	rtl_phy_write(tp, CURRENT_MDIO_PAGE, phyreg, val);

	return 0;
}

static int rtl_phy_read_status(struct phy_device *phydev)
{
	struct rtl8169_private *tp = phydev->mdio.bus->priv;
	int ret;
	int val;

	ret = genphy_read_status(phydev);
	if (ret < 0)
		return ret;

	val = rtl_phy_read(tp, 0x0a43, 26);
	phydev->link = !!(val & BIT(2));
	if (!phydev->link)
		return 0;

	if (val & BIT(3))
		phydev->duplex = DUPLEX_FULL;
	else
		phydev->duplex = DUPLEX_HALF;
	switch (val & GENMASK(5, 4)) {
	case 0x0000:
		phydev->speed = SPEED_10;
		break;
	case 0x0010:
		phydev->speed = SPEED_100;
		break;
	case 0x0020:
		phydev->speed = SPEED_1000;
		break;
	default:
		break;
	}

	return 0;
}

static int rtl_phydev_resume(struct phy_device *phydev)
{
	int ret = genphy_resume(phydev);

	msleep(20);

	return ret;
}

static int rtl_phydev_read_page(struct phy_device *phydev)
{
	struct rtl8169_private *tp = phydev->mdio.bus->priv;

	return rtl_phy_read(tp, 0, 31);
}

static int rtl_phydev_write_page(struct phy_device *phydev, int page)
{
	struct rtl8169_private *tp = phydev->mdio.bus->priv;

	rtl_phy_write(tp, 0, 31, page);

	return 0;
}

static int rtl_phydev_read_mmd(struct phy_device *phydev, int devnum, u16 regnum)
{
	struct rtl8169_private *tp = phydev->mdio.bus->priv;

	return rtl_mmd_read(tp, devnum, regnum);
}

static int rtl_phydev_write_mmd(struct phy_device *phydev, int devnum, u16 regnum,
				u16 val)
{
	struct rtl8169_private *tp = phydev->mdio.bus->priv;

	rtl_mmd_write(tp, devnum, regnum, val);

	return 0;
}

static struct phy_driver r1869soc_phy_drv = {
	PHY_ID_MATCH_VENDOR(0x001cc800),
	.name		= "Realtek r8169soc embedded PHY",
	.read_status	= rtl_phy_read_status,
	.suspend	= genphy_suspend,
	.resume		= rtl_phydev_resume,
	.read_page	= rtl_phydev_read_page,
	.write_page	= rtl_phydev_write_page,
	.read_mmd	= rtl_phydev_read_mmd,
	.write_mmd	= rtl_phydev_write_mmd,
};

static int r8169_mdio_register(struct rtl8169_private *tp)
{
	struct platform_device *pdev = tp->pdev;
	struct mii_bus *new_bus;
	int ret;

	ret = phy_driver_register(&r1869soc_phy_drv, THIS_MODULE);
	if (ret)
		return ret;

	new_bus = devm_mdiobus_alloc(&pdev->dev);
	if (!new_bus)
		return -ENOMEM;

	new_bus->name = KBUILD_MODNAME;
	new_bus->priv = tp;
	new_bus->parent = &pdev->dev;
	new_bus->irq[0] = PHY_MAC_INTERRUPT;
	snprintf(new_bus->id, MII_BUS_ID_SIZE, "%s-%d", KBUILD_MODNAME, tp->output_mode);

	new_bus->read = r8169_mdio_read_reg;
	new_bus->write = r8169_mdio_write_reg;

	ret = devm_mdiobus_register(&pdev->dev, new_bus);
	if (ret)
		return ret;

	tp->phydev = mdiobus_get_phy(new_bus, 0);
	if (tp->phydev) {
		tp->phydev->priv = tp;
	} else {
		dev_err(&pdev->dev, "No phydev found!\n");
		return -ENODEV;
	}

	tp->phydev->mac_managed_pm = true;

	phy_support_asym_pause(tp->phydev);

	switch (tp->force_speed) {
	case 100:
		tp->phydev->autoneg = AUTONEG_DISABLE;
		tp->phydev->speed = SPEED_100;
		tp->phydev->duplex = DUPLEX_FULL;
		linkmode_set_bit(ETHTOOL_LINK_MODE_100baseT_Full_BIT,
				 tp->phydev->supported);
		// Todo: due to Android GKI symbol issues, this function is temporarily removed
		//phy_start_aneg(tp->phydev);
		break;
	case 1000:
		tp->phydev->autoneg = AUTONEG_DISABLE;
		tp->phydev->speed = SPEED_1000;
		tp->phydev->duplex = DUPLEX_FULL;
		linkmode_set_bit(ETHTOOL_LINK_MODE_1000baseT_Full_BIT,
				 tp->phydev->supported);
		// Todo: due to Android GKI symbol issues, this function is temporarily removed
		//phy_start_aneg(tp->phydev);
		break;
	default:
		tp->phydev->autoneg = AUTONEG_ENABLE;
		dev_dbg(&pdev->dev, "auto-negotiation mode (%d)\n",
			tp->force_speed);
	}

	/* PHY will be woken up in rtl_open() */
	phy_suspend(tp->phydev);
	set_bit(RTL_STATUS_DOWN, tp->status);

	return 0;
}

#ifdef RTL_PROC
static int wol_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);

	netdev_dbg(dev, "WoL setting:\n");
	netdev_dbg(dev, "\tBIT 0:\t WoL enable\n");
	netdev_dbg(dev, "\tBIT 1:\t CRC match\n");
	netdev_dbg(dev, "\tBIT 2:\t WPD\n");
	netdev_dbg(dev, "\tBIT 3:\t mDNS offload\n");
	netdev_dbg(dev, "wol_enable = 0x%x\n", tp->wol_enable);
	seq_printf(m, "%d\n", tp->wol_enable);
	return 0;
}

static ssize_t wol_write_proc(struct file *file, const char __user *buffer,
			      size_t count, loff_t *pos)
{
	struct net_device *dev = (struct net_device *)
		((struct seq_file *)file->private_data)->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	char tmp[32];
	u32 val = 0;
	u32 len = 0;
	int ret;

	len = min(count, sizeof(tmp) - 1);
	if (buffer && !copy_from_user(tmp, buffer, len)) {
		tmp[len] = '\0';
		ret = kstrtou32(tmp, 0, &val);
		if (ret) {
			netdev_err(dev, "invalid WoL setting [%s], ret = %d\n", tmp, ret);
			return count;
		}
		tp->wol_enable = val;
		tp->dev->wol_enabled = (tp->wol_enable & WOL_MAGIC) ? 1 : 0;
		tp->saved_wolopts |= (tp->wol_enable & WOL_MAGIC) ? WAKE_MAGIC : 0;
		device_set_wakeup_enable(tp_to_dev(tp), tp->dev->wol_enabled);
		netdev_dbg(dev, "set wol_enable = %x\n", tp->wol_enable);
	}
	return count;
}

static int wol_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, wol_read_proc, dev);
}

static const struct proc_ops wol_proc_fops = {
	.proc_open		= wol_proc_open,
	.proc_read		= seq_read,
	.proc_write		= wol_write_proc,
	.proc_lseek		= seq_lseek,
	.proc_release		= single_release,
};

static int pwr_saving_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);

	netdev_dbg(dev, "Power saving of suspend mode:\n");
	netdev_dbg(dev, "pwr_saving = 0x%x\n", tp->pwr_saving);
	seq_printf(m, "%d\n", tp->pwr_saving);
	return 0;
}

static ssize_t pwr_saving_write_proc(struct file *file, const char __user *buffer,
				     size_t count, loff_t *pos)
{
	struct net_device *dev = (struct net_device *)
		((struct seq_file *)file->private_data)->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	char tmp[32];
	u32 val = 0;
	u32 len = 0;
	int ret;

	len = min(count, sizeof(tmp) - 1);
	if (buffer && !copy_from_user(tmp, buffer, len)) {
		tmp[len] = '\0';
		ret = kstrtou32(tmp, 0, &val);
		if (ret) {
			netdev_err(dev, "invalid power saving setting [%s], ret = %d\n", tmp, ret);
			return count;
		}
		tp->pwr_saving = val;
		netdev_dbg(dev, "set pwr_saving = %x\n", tp->pwr_saving);
	}
	return count;
}

static int pwr_saving_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, pwr_saving_read_proc, dev);
}

static const struct proc_ops pwr_saving_proc_fops = {
	.proc_open		= pwr_saving_proc_open,
	.proc_read		= seq_read,
	.proc_write		= pwr_saving_write_proc,
	.proc_lseek		= seq_lseek,
	.proc_release		= single_release,
};

static int mac_reinit_read_proc(struct seq_file *m, void *v)
{
	seq_puts(m, "\n\nUsage: echo 1 > mac_reinit\n");

	return 0;
}

static ssize_t mac_reinit_write_proc(struct file *file,
				     const char __user *buffer,
				     size_t count, loff_t *pos)
{
	struct net_device *dev = (struct net_device *)
		((struct seq_file *)file->private_data)->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	char tmp[32];
	u32 val = 0;
	u32 len = 0;
	int ret;

	len = min(count, sizeof(tmp) - 1);
	if (buffer && !copy_from_user(tmp, buffer, len)) {
		tmp[len] = '\0';
		ret = kstrtou32(tmp, 0, &val);
		if (ret || val == 0) {
			netdev_err(dev, "invalid mac_reinit [%s], ret = %d\n", tmp, ret);
			return count;
		}
		netdev_dbg(dev, "mac_reinit = %x\n", val);

		rtl_mac_reinit(tp);
	}
	return count;
}

static int mac_reinit_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, mac_reinit_read_proc, dev);
}

static const struct proc_ops mac_reinit_proc_fops = {
	.proc_open		= mac_reinit_proc_open,
	.proc_read		= seq_read,
	.proc_write		= mac_reinit_write_proc,
	.proc_lseek		= seq_lseek,
	.proc_release		= single_release,
};

static int phy_reinit_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	u32 isr;

	regmap_read(tp->iso_base, ISO_UMSK_ISR, &isr);
	seq_printf(m, "ISO_UMSK_ISR\t[0x98007004] = %08x\n", isr);

	seq_puts(m, "\n\nUsage: echo 1 > phy_reinit\n");

	return 0;
}

static ssize_t phy_reinit_write_proc(struct file *file,
				     const char __user *buffer,
				     size_t count, loff_t *pos)
{
	struct net_device *dev = (struct net_device *)
		((struct seq_file *)file->private_data)->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	char tmp[32];
	u32 val = 0;
	u32 len = 0;
	int ret;

	len = min(count, sizeof(tmp) - 1);
	if (buffer && !copy_from_user(tmp, buffer, len)) {
		tmp[len] = '\0';
		ret = kstrtou32(tmp, 0, &val);
		if (ret || val == 0) {
			netdev_err(dev, "invalid phy_reinit [%s], ret = %d\n", tmp, ret);
			return count;
		}
		netdev_dbg(dev, "phy_reinit = %x\n", val);

		rtl_lock_work(tp);
		rtl_phy_reinit(tp);
		rtl_unlock_work(tp);
	}
	return count;
}

static int phy_reinit_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, phy_reinit_read_proc, dev);
}

static const struct proc_ops phy_reinit_proc_fops = {
	.proc_open		= phy_reinit_proc_open,
	.proc_read		= seq_read,
	.proc_write		= phy_reinit_write_proc,
	.proc_lseek		= seq_lseek,
	.proc_release		= single_release,
};

static int eee_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	unsigned short e040, e080;

	rtl_lock_work(tp);
	e040 = rtl_ocp_read(tp, 0xe040);	/* EEE */
	e080 = rtl_ocp_read(tp, 0xe080);	/* EEE+ */
	seq_printf(m, "%s: eee = %d, OCP 0xe040 = 0x%x, OCP 0xe080 = 0x%x\n",
		   dev->name, tp->eee_enable, e040, e080);
	r8169_display_eee_info(dev, m, tp);
	rtl_unlock_work(tp);

	return 0;
}

static ssize_t eee_write_proc(struct file *file, const char __user *buffer,
			      size_t count, loff_t *pos)
{
	char tmp[32];
	u32 val = 0;
	u32 len = 0;
	int ret;
	bool chg = false;
	struct net_device *dev = (struct net_device *)
		((struct seq_file *)file->private_data)->private;
	struct rtl8169_private *tp = netdev_priv(dev);

	len = min(count, sizeof(tmp) - 1);
	if (buffer && !copy_from_user(tmp, buffer, len)) {
		tmp[len] = '\0';
		ret = kstrtou32(tmp, 0, &val);
		if (ret) {
			netdev_err(dev, "invalid EEE setting [%s], ret = %d\n", tmp, ret);
			return count;
		}
		netdev_dbg(dev, "write %s eee = %x\n", dev->name, val);

		if (val > 0 && !tp->eee_enable) {
			tp->eee_enable = true;
			chg = true;
		} else if (val == 0 && tp->eee_enable) {
			tp->eee_enable = false;
			chg = true;
		}
	}

	if (!chg)
		goto done;

	/* power down PHY */
	genphy_suspend(tp->phydev);
	rtl_lock_work(tp);
	tp->chip->eee_set(tp, tp->eee_enable);
	fsleep(100000);	/* wait PHY ready */
	rtl_unlock_work(tp);
	genphy_soft_reset(tp->phydev);

done:
	return count;
}

/* seq_file wrappers for procfile show routines. */
static int eee_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, eee_read_proc, dev);
}

static const struct proc_ops eee_proc_fops = {
	.proc_open		= eee_proc_open,
	.proc_read		= seq_read,
	.proc_write		= eee_write_proc,
	.proc_lseek		= seq_lseek,
	.proc_release		= single_release,
};

static void r8169soc_dump_regs(struct seq_file *m, struct rtl8169_private *tp)
{
	u32 addr;
	u32 val;
	u32 i;

	tp->chip->dump_regs(m, tp);

	seq_puts(m, "ETN MAC regs:\n");
	for (i = 0; i < 256; i += 4) {
		addr = 0x98016000 + i;
		val = readl(tp->mmio_addr + i);
		seq_printf(m, "[%08x] = %08x\n", addr, val);
	}
}

static int registers_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);

	r8169soc_dump_regs(m, tp);

	return 0;
}

/* seq_file wrappers for procfile show routines. */
static int registers_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, registers_read_proc, dev);
}

static const struct proc_ops registers_proc_fops = {
	.proc_open		= registers_proc_open,
	.proc_read		= seq_read,
	.proc_write		= NULL,
	.proc_lseek		= seq_lseek,
	.proc_release		= single_release,
};

static void r8169soc_dump_tx_desc(struct seq_file *m,
				  struct rtl8169_private *tp)
{
	u32 i;

	if (!tp->tx_desc_array) {
		seq_puts(m, "no tx_desc_array\n");
		return;
	}

	seq_printf(m, "SW TX INDEX: %d\n", tp->cur_tx % NUM_TX_DESC);
	seq_printf(m, "RECYCLED TX INDEX: %d\n", tp->dirty_tx % NUM_TX_DESC);
	if (tp->chip->features & RTL_FEATURE_TX_NO_CLOSE) {
		i = RTL_R16(tp, TX_DESC_CLOSE_IDX) & TX_DESC_CNT_MASK;
		seq_printf(m, "HW TX INDEX: %d\n", i % NUM_TX_DESC);
	}
	seq_puts(m, "TX DESC:\n");
	for (i = 0; i < NUM_TX_DESC; i++)
		seq_printf(m, "Desc[%04d] opts1 0x%08x, opts2 0x%08x, addr 0x%llx\n",
			   i, tp->tx_desc_array[i].opts1,
			   tp->tx_desc_array[i].opts2,
			   tp->tx_desc_array[i].addr);
}

static int tx_desc_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);

	r8169soc_dump_tx_desc(m, tp);

	return 0;
}

/* seq_file wrappers for procfile show routines. */
static int tx_desc_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, tx_desc_read_proc, dev);
}

static const struct proc_ops tx_desc_proc_fops = {
	.proc_open		= tx_desc_proc_open,
	.proc_read		= seq_read,
	.proc_write		= NULL,
	.proc_lseek		= seq_lseek,
	.proc_release		= single_release,
};

static void r8169soc_dump_rx_desc(struct seq_file *m,
				  struct rtl8169_private *tp)
{
	u32 i;

	if (!tp->rx_desc_array) {
		seq_puts(m, "no rx_desc_array\n");
		return;
	}

	seq_printf(m, "SW RX INDEX: %d\n", tp->cur_rx % NUM_RX_DESC);
	#if defined(CONFIG_RTL_RX_NO_COPY)
	seq_printf(m, "REFILLED RX INDEX: %d\n", tp->dirty_rx % NUM_RX_DESC);
	#endif /* CONFIG_RTL_RX_NO_COPY */
	seq_puts(m, "RX DESC:\n");
	for (i = 0; i < NUM_RX_DESC; i++)
		seq_printf(m, "Desc[%04d] opts1 0x%08x, opts2 0x%08x, addr 0x%llx\n",
			   i, tp->rx_desc_array[i].opts1,
			   tp->rx_desc_array[i].opts2,
			   tp->rx_desc_array[i].addr);
}

static int rx_desc_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);

	r8169soc_dump_rx_desc(m, tp);

	return 0;
}

/* seq_file wrappers for procfile show routines. */
static int rx_desc_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, rx_desc_read_proc, dev);
}

static const struct proc_ops rx_desc_proc_fops = {
	.proc_open		= rx_desc_proc_open,
	.proc_read		= seq_read,
	.proc_write		= NULL,
	.proc_lseek		= seq_lseek,
	.proc_release		= single_release,
};

static int driver_var_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);

	seq_puts(m, "\nDump Driver Variable\n");

	seq_puts(m, "Variable\tValue\n----------\t-----\n");
	seq_printf(m, "KBUILD_MODNAME\t%s\n", KBUILD_MODNAME);
	seq_printf(m, "chipset_name\t%s\n", tp->chip->name);
	seq_printf(m, "driver version\t%s\n", RTL8169SOC_VERSION);
	seq_printf(m, "chip features\t0x%x\n", tp->chip->features);
	seq_printf(m, "mtu\t\t%d\n", dev->mtu);
	seq_printf(m, "NUM_RX_DESC\t%d\n", NUM_RX_DESC);
	seq_printf(m, "cur_rx\t\t%d\n", tp->cur_rx);
#if defined(CONFIG_RTL_RX_NO_COPY)
	seq_printf(m, "dirty_rx\t%d\n", tp->dirty_rx);
#endif /* CONFIG_RTL_RX_NO_COPY */
	seq_printf(m, "NUM_TX_DESC\t%d\n", NUM_TX_DESC);
	seq_printf(m, "cur_tx\t\t%d\n", tp->cur_tx);
	seq_printf(m, "dirty_tx\t%d\n", tp->dirty_tx);
	seq_printf(m, "rx_buf_sz\t%d\n", rx_buf_sz);
	seq_printf(m, "cp_cmd\t\t0x%x\n", tp->cp_cmd);
	seq_printf(m, "irq_mask\t0x%x\n", tp->irq_mask);
	seq_printf(m, "wol_enable\t0x%x\n", tp->wol_enable);
	seq_printf(m, "pwr_saving\t%d\n", tp->pwr_saving);
	seq_printf(m, "saved_wolopts\t0x%x\n", tp->saved_wolopts);
	seq_printf(m, "wol_crc_cnt\t%d\n", tp->wol_crc_cnt);
	seq_printf(m, "led_cfg\t\t0x%x\n", tp->led_cfg);
	seq_printf(m, "eee_enable\t%d\n", tp->eee_enable);
	seq_printf(m, "acp_enable\t%d\n", tp->acp_enable);
	seq_printf(m, "amp_k_offset\t0x%x\n", tp->amp_k_offset);
	seq_printf(m, "ext_phy\t\t%d\n", tp->ext_phy);
	seq_printf(m, "ext_phy_id\t%d\n", tp->ext_phy_id);
	seq_printf(m, "output_mode\t%d\n", tp->output_mode);
	seq_printf(m, "force_speed\t%d\n", tp->force_speed);
	seq_printf(m, "status\t\t0x%lx\n", tp->status[0]);
	tp->chip->dump_var(m, tp);
	seq_printf(m, "ETN IRQ\t\t%d\n", dev->irq);
	seq_printf(m, "perm_addr\t%pM\n", dev->perm_addr);
	seq_printf(m, "dev_addr\t%pM\n", dev->dev_addr);

	seq_putc(m, '\n');
	return 0;
}

static int driver_var_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, driver_var_read_proc, dev);
}

static const struct proc_ops driver_var_proc_fops = {
	.proc_open		= driver_var_proc_open,
	.proc_read		= seq_read,
	.proc_write		= NULL,
	.proc_lseek		= seq_lseek,
	.proc_release		= single_release,
};

static int tally_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	struct rtl8169_counters *counters = tp->counters;

	rtl8169_update_counters(tp);

	seq_puts(m, "\nDump Tally Counter\n");
	seq_puts(m, "Statistics\t\tValue\n----------\t\t-----\n");
	seq_printf(m, "tx_packets\t\t%lld\n",
		   le64_to_cpu(counters->tx_packets));
	seq_printf(m, "rx_packets\t\t%lld\n",
		   le64_to_cpu(counters->rx_packets));
	seq_printf(m, "tx_errors\t\t%lld\n", le64_to_cpu(counters->tx_errors));
	seq_printf(m, "rx_errors\t\t%d\n", le32_to_cpu(counters->rx_errors));
	seq_printf(m, "rx_missed\t\t%d\n", le16_to_cpu(counters->rx_missed));
	seq_printf(m, "align_errors\t\t%d\n",
		   le16_to_cpu(counters->align_errors));
	seq_printf(m, "tx_one_collision\t%d\n",
		   le32_to_cpu(counters->tx_one_collision));
	seq_printf(m, "tx_multi_collision\t%d\n",
		   le32_to_cpu(counters->tx_multi_collision));
	seq_printf(m, "rx_unicast\t\t%lld\n",
		   le64_to_cpu(counters->rx_unicast));
	seq_printf(m, "rx_broadcast\t\t%lld\n",
		   le64_to_cpu(counters->rx_broadcast));
	seq_printf(m, "rx_multicast\t\t%d\n",
		   le32_to_cpu(counters->rx_multicast));
	seq_printf(m, "tx_aborted\t\t%d\n", le16_to_cpu(counters->tx_aborted));
	seq_printf(m, "tx_underrun\t\t%d\n",
		   le16_to_cpu(counters->tx_underrun));

	seq_putc(m, '\n');
	return 0;
}

static int tally_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, tally_read_proc, dev);
}

static const struct proc_ops tally_proc_fops = {
	.proc_open		= tally_proc_open,
	.proc_read		= seq_read,
	.proc_write		= NULL,
	.proc_lseek		= seq_lseek,
	.proc_release		= single_release,
};

static int ocp_read_proc(struct seq_file *m, void *v)
{
	seq_puts(m, "USAGE: echo \"<reg> [<value>]\" > ocp\n");
	return 0;
}

static ssize_t ocp_write_proc(struct file *file, const char __user *buffer,
			      size_t count, loff_t *pos)
{
	char tmp[32];
	u32 reg;
	u32 val;
	u32 len;
	int ret;
	char *p;
	char *t;
	char *delim = " ";
	struct net_device *dev = (struct net_device *)
			((struct seq_file *)file->private_data)->private;
	struct rtl8169_private *tp = netdev_priv(dev);

	len = min(count, sizeof(tmp) - 1);
	if (buffer && !copy_from_user(tmp, buffer, len)) {
		tmp[len] = '\0';

		p = tmp;
		t = strsep(&p, delim);
		if (!t)
			goto usage;
		ret = kstrtou32(t, 0, &reg);
		if (ret) {
			netdev_err(dev, "invalid reg [%s], ret = %d\n", t, ret);
			goto usage;
		} else if (reg & 0x1 || reg > 0xFFFF) {
			netdev_err(dev, "invalid reg 0x%04x\n", reg);
			goto usage;
		}

		t = strsep(&p, delim);
		if (!t) {
			rtl_lock_work(tp);
			/* read cmd */
			val = rtl_ocp_read(tp, reg);
			rtl_unlock_work(tp);
			netdev_err(dev, "Read OCP reg 0x%04x, value = 0x%04x\n",
				   reg, val);
			goto out;
		}
		ret = kstrtou32(t, 0, &val);
		if (ret) {
			netdev_err(dev, "invalid value [%s], ret = %d\n", t, ret);
			goto usage;
		} else {
			rtl_lock_work(tp);
			/* write cmd */
			rtl_ocp_write(tp, reg, val);
			rtl_unlock_work(tp);
			netdev_dbg(dev, "Write OCP reg 0x%04x value 0x%04x\n",
				   reg, val);
			goto out;
		}
	}

usage:
	pr_info("USAGE: echo \"<reg> [<value>]\" > ocp\n");
out:
	return count;
}

static int ocp_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, ocp_read_proc, dev);
}

static const struct proc_ops ocp_proc_fops = {
	.proc_open		= ocp_proc_open,
	.proc_read		= seq_read,
	.proc_write		= ocp_write_proc,
	.proc_lseek		= seq_lseek,
	.proc_release		= single_release,
};

static int eth_phy_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	int i, n, max = 16;
	u16 word_rd;
	struct rtl8169_private *tp = netdev_priv(dev);

	seq_puts(m, "\nDump Ethernet PHY\n");
	seq_puts(m, "\nOffset\tValue\n------\t-----\n ");

	rtl_lock_work(tp);
	seq_puts(m, "\n####################page 0##################\n ");
	for (n = 0; n < max;) {
		seq_printf(m, "\n0x%02x:\t", n);

		for (i = 0; i < 8 && n < max; i++, n++) {
			word_rd = rtl_phy_read(tp, 0, n);
			seq_printf(m, "%04x ", word_rd);
		}
	}
	rtl_unlock_work(tp);

	seq_putc(m, '\n');
	return 0;
}

static ssize_t eth_phy_write_proc(struct file *file, const char __user *buffer,
				  size_t count, loff_t *pos)
{
	char tmp[32];
	u32 page;
	u32 reg;
	u32 val;
	u32 len;
	int ret;
	char *p;
	char *t;
	char *delim = " ";
	struct net_device *dev = (struct net_device *)
			((struct seq_file *)file->private_data)->private;
	struct rtl8169_private *tp = netdev_priv(dev);

	len = min(count, sizeof(tmp) - 1);
	if (buffer && !copy_from_user(tmp, buffer, len)) {
		tmp[len] = '\0';

		p = tmp;
		t = strsep(&p, delim);
		if (!t)
			goto usage;
		ret = kstrtou32(t, 0, &page);
		if (ret) {
			netdev_err(dev, "invalid page [%s], ret = %d\n", t, ret);
			goto usage;
		}

		t = strsep(&p, delim);
		if (!t)
			goto usage;
		ret = kstrtou32(t, 0, &reg);
		if (ret) {
			netdev_err(dev, "invalid reg [%s], ret = %d\n", t, ret);
			goto usage;
		}

		t = strsep(&p, delim);
		if (!t) {
			rtl_lock_work(tp);
			/* read cmd */
			val = rtl_phy_read(tp, page, reg & 0x1f);
			rtl_unlock_work(tp);
			netdev_err(dev, "Read page 0x%x reg 0x%x, value = 0x%04x\n",
				   page, reg & 0x1f, val);
			goto out;
		}
		ret = kstrtou32(t, 0, &val);
		if (ret) {
			netdev_err(dev, "invalid value [%s], ret = %d\n", t, ret);
			goto usage;
		} else {
			rtl_lock_work(tp);
			/* write cmd */
			rtl_phy_write(tp, page, reg & 0x1f, val);
			rtl_unlock_work(tp);
			netdev_dbg(dev, "Write page 0x%x reg 0x%x value 0x%04x\n",
				   page, reg & 0x1f, val);
			goto out;
		}
	}

usage:
	pr_info("USAGE: echo \"<page> <reg> [<value>]\" > eth_phy\n");
out:
	return count;
}

static int eth_phy_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, eth_phy_read_proc, dev);
}

static const struct proc_ops eth_phy_proc_fops = {
	.proc_open		= eth_phy_proc_open,
	.proc_read		= seq_read,
	.proc_write		= eth_phy_write_proc,
	.proc_lseek		= seq_lseek,
	.proc_release		= single_release,
};

static int eth_led_read_proc(struct seq_file *m, void *v)
{
	seq_puts(m, "USAGE:\necho \"<value>\" > eth_led\n");
	seq_puts(m, "\tvalue: 0:disable, 1:enable\n");

	return 0;
}

static ssize_t eth_led_write_proc(struct file *file, const char __user *buffer,
				  size_t count, loff_t *pos)
{
	char tmp[32];
	u32 val;
	u32 len;
	int ret;
	bool enable = false;
	struct net_device *dev = (struct net_device *)
			((struct seq_file *)file->private_data)->private;
	struct rtl8169_private *tp = netdev_priv(dev);

	len = min(count, sizeof(tmp) - 1);

	if (tp->output_mode != OUTPUT_EMBEDDED_PHY) {
		netdev_dbg(dev, "only take effect when using embedded PHY\n");
		goto out;
	}

	if (buffer && !copy_from_user(tmp, buffer, len)) {
		tmp[len] = '\0';
		ret = kstrtou32(tmp, 0, &val);
		if (ret) {
			pr_err("invalid LED setting [%s], ret = %d\n",
			       tmp, ret);
			goto out;
		}
		netdev_dbg(dev, "write %s eth_led = %x\n", dev->name, val);

		enable = (val > 0) ? true : false;
	}

	rtl_lock_work(tp);
	tp->chip->led_set(tp, enable);
	rtl_unlock_work(tp);
out:
	return count;
}

static int eth_led_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, eth_led_read_proc, dev);
}

static const struct proc_ops eth_led_proc_fops = {
	.proc_open		= eth_led_proc_open,
	.proc_read		= seq_read,
	.proc_write		= eth_led_write_proc,
	.proc_lseek		= seq_lseek,
	.proc_release		= single_release,
};

static int ext_regs_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	int i, n, max = 256;
	u32 dword_rd;
	struct rtl8169_private *tp = netdev_priv(dev);

	seq_puts(m, "\nDump Extended Registers\n");
	seq_puts(m, "\nOffset\tValue\n------\t-----\n ");

	rtl_lock_work(tp);
	for (n = 0; n < max;) {
		seq_printf(m, "\n0x%02x:\t", n);

		for (i = 0; i < 4 && n < max; i++, n += 4) {
			dword_rd = rtl_eri_read(tp, n);
			seq_printf(m, "%08x ", dword_rd);
		}
	}
	rtl_unlock_work(tp);

	seq_putc(m, '\n');
	return 0;
}

static int ext_regs_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, ext_regs_read_proc, dev);
}

static const struct proc_ops ext_regs_proc_fops = {
	.proc_open		= ext_regs_proc_open,
	.proc_read		= seq_read,
	.proc_write		= NULL,
	.proc_lseek		= seq_lseek,
	.proc_release		= single_release,
};

static int wpd_evt_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	u32 tmp;

	rtl_lock_work(tp);
	/* check if wol_own and wpd_en are both set */
	tmp = rtl_ocp_read(tp, 0xC0C2);
	if ((tmp & BIT(4)) == 0 || (tmp & BIT(0)) == 0) {
		seq_puts(m, "\nNo WPD event\n");
		rtl_unlock_work(tp);
		return 0;
	}

	seq_puts(m, "\nWPD event:\n");
	tmp = rtl_ocp_read(tp, 0xD23A) & 0x1F01;
	seq_printf(m, "Type (0: CRC match,  1: magic pkt) = %d\n",
		   tmp & 0x1);
	if ((tmp & 0x1) == 0)
		seq_printf(m, "CRC match ID = %d\n", tmp >> 8);
	seq_printf(m, "Original packet length = %d\n",
		   rtl_ocp_read(tp, 0xD23C));
	seq_printf(m, "Stored packet length = %d\n",
		   rtl_ocp_read(tp, 0xD23E));
	rtl_unlock_work(tp);

	seq_putc(m, '\n');
	return 0;
}

static int wpd_evt_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, wpd_evt_read_proc, dev);
}

static const struct proc_ops wpd_evt_proc_fops = {
	.proc_open		= wpd_evt_proc_open,
	.proc_read		= seq_read,
	.proc_write		= NULL,
	.proc_lseek		= seq_lseek,
	.proc_release		= single_release,
};

static int wol_pkt_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	int i;
	char wol_pkt[WOL_BUF_LEN];
	u32 len;
	u32 tmp;
	u16 *ptr;

	memset(wol_pkt, 0, WOL_BUF_LEN);

	rtl_lock_work(tp);
	/* check if wol_own and wpd_en are both set */
	tmp = rtl_ocp_read(tp, 0xC0C2);
	if ((tmp & BIT(4)) == 0 || (tmp & BIT(0)) == 0) {
		rtl_unlock_work(tp);
		return 0;
	}

	/* read 128-byte packet buffer */
	for (i = 0; i < WOL_BUF_LEN; i += 2) {
		ptr = (u16 *)&wol_pkt[i];
		*ptr = rtl_ocp_read(tp, 0xD240 + i);
	}

	/* get stored packet length */
	len = min((int)rtl_ocp_read(tp, 0xD23E), WOL_BUF_LEN);
	for (i = 0; i < len; i++) {
		if ((i % 16) == 0)
			seq_puts(m, "\n");
		else if ((i % 8) == 0)
			seq_puts(m, "  ");
		seq_printf(m, "%02x ", wol_pkt[i]);
	}
	rtl_unlock_work(tp);

	seq_putc(m, '\n');
	return 0;
}

static int wol_pkt_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, wol_pkt_read_proc, dev);
}

static const struct proc_ops wol_pkt_proc_fops = {
	.proc_open		= wol_pkt_proc_open,
	.proc_read		= seq_read,
	.proc_write		= NULL,
	.proc_lseek		= seq_lseek,
	.proc_release		= single_release,
};

static void rtl_proc_hex_dump(struct seq_file *m, char *ptr, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		if ((i % 16) == 0)
			seq_putc(m, '\n');
		else if ((i % 8) == 0)
			seq_puts(m, "- ");

		seq_printf(m, "%02X ", ptr[i]);
	}
}

static int rtl_str2hex(struct rtl8169_private *tp, char *src, char *dst)
{
	int i = 0;
	char *p;
	char *t;
	char *delim = " ";
	int ret;

	p = src;
	while ((t = strsep(&p, delim)) != NULL) {
		ret = kstrtou8(t, 16, &dst[i]);
		if (ret) {
			netdev_err(tp->dev, "%s:%d: invalid token[%s] to hex, ret 0x%x\n",
				   __func__, __LINE__, t, ret);
			return i;
		}
		i++;
	}

	return i;
}

static int wake_mask_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);

	seq_puts(m, "\nMASK = [");
	rtl_proc_hex_dump(m, tp->wol_rule_buf.mask, tp->wol_rule_buf.mask_size);
	seq_puts(m, "\n]\n");

	return 0;
}

static ssize_t wake_mask_write_proc(struct file *file,
				    const char __user *buffer,
				    size_t count, loff_t *pos)
{
	char tmp[512];
	u32 len;
	struct net_device *dev = (struct net_device *)
			((struct seq_file *)file->private_data)->private;
	struct rtl8169_private *tp = netdev_priv(dev);

	len = min(count, sizeof(tmp) - 1);
	if (buffer && !copy_from_user(tmp, buffer, len)) {
		tmp[len] = '\0';

		if (strlen(tmp) >= (RTL_WAKE_MASK_SIZE * 3)) {
			netdev_err(dev, "input length should be smaller than %d\n",
				   RTL_WAKE_MASK_SIZE * 3);
			goto out;
		}
		memset(tp->wol_rule_buf.mask, 0, RTL_WAKE_MASK_SIZE);

		tp->wol_rule_buf.mask_size =
			rtl_str2hex(tp, tmp, tp->wol_rule_buf.mask);
	}

out:
	return count;
}

static int wake_mask_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, wake_mask_read_proc, dev);
}

static const struct proc_ops wake_mask_proc_fops = {
	.proc_open           = wake_mask_proc_open,
	.proc_read           = seq_read,
	.proc_write          = wake_mask_write_proc,
	.proc_lseek          = seq_lseek,
	.proc_release        = single_release,
};

static int wake_crc_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);

	seq_printf(m, "CRC16 = [0x%04X]\n", tp->wol_rule_buf.crc);

	return 0;
}

static ssize_t wake_crc_write_proc(struct file *file, const char __user *buffer,
				   size_t count, loff_t *pos)
{
	char tmp[80];
	u32 len;
	struct net_device *dev = (struct net_device *)
			((struct seq_file *)file->private_data)->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	int ret;

	len = min(count, sizeof(tmp) - 1);
	if (buffer && !copy_from_user(tmp, buffer, len)) {
		tmp[len] = '\0';
		ret = kstrtou16(tmp, 16, &tp->wol_rule_buf.crc);
		if (ret) {
			netdev_err(dev, "%s:%d: invalid token[%s] to hex, ret 0x%x\n",
				   __func__, __LINE__, tmp, ret);
			return count;
		}
	}

	return count;
}

static int wake_crc_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, wake_crc_read_proc, dev);
}

static const struct proc_ops wake_crc_proc_fops = {
	.proc_open           = wake_crc_proc_open,
	.proc_read           = seq_read,
	.proc_write          = wake_crc_write_proc,
	.proc_lseek          = seq_lseek,
	.proc_release        = single_release,
};

static int wake_offset_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);

	seq_printf(m, "OFFSET = [0x%04X]\n", tp->wol_rule_buf.offset);

	return 0;
}

static ssize_t wake_offset_write_proc(struct file *file,
				      const char __user *buffer,
				      size_t count, loff_t *pos)
{
	char tmp[80];
	u32 len;
	struct net_device *dev = (struct net_device *)
			((struct seq_file *)file->private_data)->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	int ret;

	len = min(count, sizeof(tmp) - 1);
	if (buffer && !copy_from_user(tmp, buffer, len)) {
		tmp[len] = '\0';
		ret = kstrtou16(tmp, 0, &tp->wol_rule_buf.offset);
		if (ret) {
			netdev_err(dev, "%s:%d: invalid token[%s] to hex, ret 0x%x\n",
				   __func__, __LINE__, tmp, ret);
			return count;
		}
	}

	return count;
}

static int wake_offset_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, wake_offset_read_proc, dev);
}

static const struct proc_ops wake_offset_proc_fops = {
	.proc_open           = wake_offset_proc_open,
	.proc_read           = seq_read,
	.proc_write          = wake_offset_write_proc,
	.proc_lseek          = seq_lseek,
	.proc_release        = single_release,
};

static int wake_pattern_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);

	seq_puts(m, "\nPATTERN = [");
	rtl_proc_hex_dump(m, tp->wol_rule_buf.pattern,
			  tp->wol_rule_buf.pattern_size);
	seq_puts(m, "\n]\n");

	return 0;
}

static ssize_t wake_pattern_write_proc(struct file *file,
				       const char __user *buffer,
				       size_t count, loff_t *pos)
{
	char tmp[512];
	u32 len;
	struct net_device *dev = (struct net_device *)
			((struct seq_file *)file->private_data)->private;
	struct rtl8169_private *tp = netdev_priv(dev);

	len = min(count, sizeof(tmp) - 1);
	if (buffer && !copy_from_user(tmp, buffer, len)) {
		tmp[len] = '\0';
		if (strlen(tmp) >= (RTL_WAKE_PATTERN_SIZE * 3)) {
			netdev_err(dev, "input length should be smaller than %d\n",
				   RTL_WAKE_PATTERN_SIZE * 3);
			goto out;
		}
		memset(tp->wol_rule_buf.pattern, 0, RTL_WAKE_PATTERN_SIZE);

		tp->wol_rule_buf.pattern_size =
			rtl_str2hex(tp, tmp, tp->wol_rule_buf.pattern);
	}

out:
	return count;
}

static int wake_pattern_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, wake_pattern_read_proc, dev);
}

static const struct proc_ops wake_pattern_proc_fops = {
	.proc_open           = wake_pattern_proc_open,
	.proc_read           = seq_read,
	.proc_write          = wake_pattern_write_proc,
	.proc_lseek          = seq_lseek,
	.proc_release        = single_release,
};

static int wake_idx_en_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	int wake_size;
	int i;

	seq_puts(m, "USAGE:\necho \"[index] [enable]\" > wake_idx_en\n");
	seq_puts(m, "\tindex\t0 ~ 31\n");
	seq_puts(m, "\tenable\t0 ~ 1\n");
	seq_puts(m, "\t\tenable = 0 ==> remove rule[index]\n");
	if (tp->chip->features & RTL_FEATURE_PAT_WAKE) {
		seq_puts(m, "\t\tenable = 1 ==> add {wake_mask, wake_crc, ");
		seq_puts(m, "wake_offset, wake_pattern} to rule[index]\n");
	} else {
		seq_puts(m, "\t\tenable = 1 ==> add {wake_mask, wake_crc} to ");
		seq_puts(m, "rule[index]\n");
	}

	if (tp->chip->features & RTL_FEATURE_PAT_WAKE)
		wake_size = RTL_WAKE_SIZE;
	else
		wake_size = RTL_WAKE_SIZE_CRC;

	seq_printf(m, "## Total rules = %d\n", tp->wol_crc_cnt);
	for (i = 0; i < wake_size; i++)
		seq_printf(m, "rule[%d] = %s\n", i,
			   tp->wol_rule[i].flag & WAKE_FLAG_ENABLE ? "enable" : "disable");
	return 0;
}

static ssize_t wake_idx_en_write_proc(struct file *file,
				      const char __user *buffer,
				      size_t count, loff_t *pos)
{
	char tmp[80];
	u32 len;
	struct net_device *dev = (struct net_device *)
			((struct seq_file *)file->private_data)->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	char *p;
	char *t;
	char *delim = " ";
	int ret;
	u8 idx;
	u8 en;

	len = min(count, sizeof(tmp) - 1);
	if (buffer && !copy_from_user(tmp, buffer, len)) {
		tmp[len] = '\0';

		p = tmp;
		t = strsep(&p, delim);
		if (!t)
			goto usage;
		ret = kstrtou8(t, 0, &idx);
		if (ret) {
			netdev_err(dev, "invalid idx token[%s], ret 0x%x\n", t, ret);
			goto usage;
		}

		if (idx > 31) {
			netdev_err(dev, "invalid idx [%d] (idx <= 31)\n", idx);
			goto usage;
		}

		t = strsep(&p, delim);
		if (!t)
			goto usage;
		ret = kstrtou8(t, 0, &en);
		if (ret) {
			netdev_err(dev, "invalid en token[%s], ret 0x%x\n", t, ret);
			goto usage;
		}

		if (en > 1) {
			netdev_err(dev, "invalid en [%d] (en = 0 or 1)\n", idx);
			goto usage;
		}

		if (en == 1) {
			if (!(tp->wol_rule[idx].flag & WAKE_FLAG_ENABLE))
				tp->wol_crc_cnt++;
			/* add/replace a rule */
			tp->wol_rule_buf.flag = WAKE_FLAG_ENABLE;
			memcpy(&tp->wol_rule[idx], &tp->wol_rule_buf,
			       sizeof(struct rtl_wake_rule_s));
		} else {
			if (tp->wol_rule[idx].flag & WAKE_FLAG_ENABLE)
				tp->wol_crc_cnt--;
			/* del a rule */
			tp->wol_rule_buf.flag &= ~WAKE_FLAG_ENABLE;
			tp->wol_rule[idx].flag &= ~WAKE_FLAG_ENABLE;
		}
		goto out;
	}

usage:
	pr_info("USAGE:\necho \"[index] [enable]\" > wake_idx_en\n");
	pr_info("\tindex\t0 ~ 31\n");
	pr_info("\tenable\t0 ~ 1\n");
	pr_info("\t\tenable = 0 ==> remove rule[index]\n");
	if (tp->chip->features & RTL_FEATURE_PAT_WAKE) {
		pr_info("\t\tenable = 1 ==> add {wake_mask, wake_crc, ");
		pr_info("wake_offset, wake_pattern} to rule[index]\n");
	} else {
		pr_info("\t\tenable = 1 ==> add {wake_mask, wake_crc} to ");
		pr_info("rule[index]\n");
	}

out:
	return count;
}

static int wake_idx_en_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, wake_idx_en_read_proc, dev);
}

static const struct proc_ops wake_idx_en_proc_fops = {
	.proc_open           = wake_idx_en_proc_open,
	.proc_read           = seq_read,
	.proc_write          = wake_idx_en_write_proc,
	.proc_lseek          = seq_lseek,
	.proc_release        = single_release,
};

static int wake_dump_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	int i;

	seq_puts(m, "\nWOL rules dump:\n");
	for (i = 0; i < RTL_WAKE_SIZE; i++) {
		if (!(tp->wol_rule[i].flag & WAKE_FLAG_ENABLE))
			continue;

		seq_printf(m, "##### index %d #####\n", i);

		seq_puts(m, "\nMASK = [");
		rtl_proc_hex_dump(m, tp->wol_rule[i].mask,
				  tp->wol_rule[i].mask_size);
		seq_puts(m, "\n]\n");

		seq_printf(m, "CRC16  = 0x%04X\n", tp->wol_rule[i].crc);
		if (tp->chip->features & RTL_FEATURE_PAT_WAKE) {
			seq_printf(m, "OFFSET = 0x%04X\n", tp->wol_rule[i].offset);
			seq_puts(m, "\nPATTERN = [");
			rtl_proc_hex_dump(m, tp->wol_rule[i].pattern,
					  tp->wol_rule[i].pattern_size);
			seq_puts(m, "\n]\n");
		}
	}

	seq_putc(m, '\n');
	return 0;
}

static int wake_dump_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, wake_dump_read_proc, dev);
}

static const struct proc_ops wake_dump_proc_fops = {
	.proc_open           = wake_dump_proc_open,
	.proc_read           = seq_read,
	.proc_write          = NULL,
	.proc_lseek          = seq_lseek,
	.proc_release        = single_release,
};

static int storm_ctrl_read_proc(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	int i;
	char *name;
	char *type;

	for (i = 0; i < 3; i++) {
		switch (i) {
		case RTL_BROADCAST_PKT:
			name = "Broadcast";
			break;
		case RTL_MULTICAST_PKT:
			name = "Multicast";
			break;
		case RTL_UNKNOWN_PKT:
		default:
			name = "Unknown";
		}

		switch (tp->sc[i].type) {
		case RTL_NO_LIMIT:
			type = "disabled";
			break;
		case RTL_PKT_LIMIT:
			type = "pkt-based";
			break;
		case RTL_RATE_LIMIT:
			type = "rate-based";
			break;
		default:
			type = "invalid";
		}
		seq_printf(m, "%s: %s limit = %d\n", name, type, tp->sc[i].limit);
	}

	seq_puts(m, "USAGE:\necho \"[pkt_type] [limit_type] [limit]\" > storm_ctrl\n");
	seq_puts(m, "\tpkt_type\t0:broadcast, 1:multicast, 2:unknown\n");
	seq_puts(m, "\tlimit_type\t0:disable, 1:pkt_limit, 2:rate_limit\n");
	seq_puts(m, "\tlimit\t0 ~ 0xFFFFFFFF\n");

	return 0;
}

static ssize_t storm_ctrl_write_proc(struct file *file,
				     const char __user *buffer,
				     size_t count, loff_t *pos)
{
	char tmp[80];
	u32 len;
	struct net_device *dev = (struct net_device *)
			((struct seq_file *)file->private_data)->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	char *p;
	char *t;
	char *delim = " ";
	int ret;
	u8 pkt_type;
	u8 limit_type;
	u16 limit;

	len = min(count, sizeof(tmp) - 1);
	if (buffer && !copy_from_user(tmp, buffer, len)) {
		tmp[len] = '\0';

		p = tmp;
		t = strsep(&p, delim);
		if (!t)
			goto usage;
		ret = kstrtou8(t, 0, &pkt_type);
		if (ret) {
			netdev_err(dev, "invalid pkt_type token[%s], ret 0x%x\n", t, ret);
			goto usage;
		}

		if (pkt_type > 2) {
			netdev_err(dev, "invalid pkt_type [%d] (pkt_type <= 2)\n", pkt_type);
			goto usage;
		}

		t = strsep(&p, delim);
		if (!t)
			goto usage;
		ret = kstrtou8(t, 0, &limit_type);
		if (ret) {
			netdev_err(dev, "invalid limit_type token[%s], ret 0x%x\n", t, ret);
			goto usage;
		}

		if (limit_type > 2) {
			netdev_err(dev, "invalid limit_type [%d] (limit_type <= 2)\n", limit_type);
			goto usage;
		}

		t = strsep(&p, delim);
		if (!t)
			goto usage;
		ret = kstrtou16(t, 0, &limit);
		if (ret) {
			netdev_err(dev, "invalid limit token[%s], ret 0x%x\n", t, ret);
			goto usage;
		}

		rtl_lock_work(tp);
		rtl_storm_ctrl(tp, pkt_type, limit_type, limit);
		rtl_unlock_work(tp);

		goto out;
	}

usage:
	pr_info("USAGE:\necho \"[pkt_type] [limit_type] [limit]\" > storm_ctrl\n");
	pr_info("\tpkt_type\t0:broadcast, 1:multicast, 2:unknown\n");
	pr_info("\tlimit_type\t0:disable, 1:pkt_limit, 2:rate_limit\n");
	pr_info("\tlimit\t0 ~ 0xFFFFFFFF\n");

out:
	return count;
}

static int storm_ctrl_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, storm_ctrl_read_proc, dev);
}

static const struct proc_ops storm_ctrl_proc_fops = {
	.proc_open           = storm_ctrl_proc_open,
	.proc_read           = seq_read,
	.proc_write          = storm_ctrl_write_proc,
	.proc_lseek          = seq_lseek,
	.proc_release        = single_release,
};

static int loopback_read_proc(struct seq_file *m, void *v)
{
	seq_puts(m, "USAGE:\necho \"[mode] [len] [cnt]\" > loopback\n");
	seq_puts(m, "\tmode\t0:MAC loopback, 1: internal PHY PCS loopback, 2:internal PHY remote loopback\n");
	seq_puts(m, "\t\t3:external PHY PCS loopback, 4:external PHY remote loopback\n");
	seq_puts(m, "\tlen\tpacket size (60 ~ 1514)\n");
	seq_puts(m, "\tcnt\tloop count\n");

	return 0;
}

static ssize_t loopback_write_proc(struct file *file,
				   const char __user *buffer,
				   size_t count, loff_t *pos)
{
	char tmp[80];
	u32 len;
	struct net_device *dev = (struct net_device *)
			((struct seq_file *)file->private_data)->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	char *p;
	char *t;
	char *delim = " ";
	int ret;
	u8 mode;
	u32 pkt_len = 0;
	u32 cnt = 0;

	len = min(count, sizeof(tmp) - 1);
	if (buffer && !copy_from_user(tmp, buffer, len)) {
		tmp[len] = '\0';

		p = tmp;
		t = strsep(&p, delim);
		if (!t)
			goto usage;
		ret = kstrtou8(t, 0, &mode);
		if (ret) {
			netdev_err(dev, "invalid mode token[%s], ret 0x%x\n", t, ret);
			goto usage;
		}

		if (mode >= RTL_LOOPBACK_MAX) {
			netdev_err(dev, "invalid mode [%d] (mode < %d)\n", mode, RTL_LOOPBACK_MAX);
			goto usage;
		}

		t = strsep(&p, delim);
		if (!t)
			goto usage;
		ret = kstrtou32(t, 0, &pkt_len);
		if (ret) {
			netdev_err(dev, "invalid len token[%s], ret 0x%x\n", t, ret);
			goto usage;
		}
		if (pkt_len < 60 || pkt_len > 1514) {
			netdev_err(dev, "invalid len [%d] (60 <= len <= 1514)\n", pkt_len);
			goto usage;
		}

		t = strsep(&p, delim);
		if (!t)
			goto usage;
		ret = kstrtou32(t, 0, &cnt);
		if (ret) {
			netdev_err(dev, "invalid cnt token[%s], ret 0x%x\n", t, ret);
			goto usage;
		}

		rtl_loopback(tp, mode, pkt_len, cnt);

		goto out;
	}

usage:
	pr_info("USAGE:\necho \"[mode] [len] [cnt]\" > loopback\n");
	pr_info("\tmode\t0:MAC loopback, 1: internal PHY PCS loopback, 2:internal PHY remote loopback\n");
	pr_info("\t\t3:external PHY PCS loopback, 4:external PHY remote loopback\n");
	pr_info("\tlen\tpacket size (60 ~ 1514)\n");
	pr_info("\tcnt\tloop count\n");

out:
	return count;
}

static int loopback_proc_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, loopback_read_proc, dev);
}

static const struct proc_ops loopback_proc_fops = {
	.proc_open           = loopback_proc_open,
	.proc_read           = seq_read,
	.proc_write          = loopback_write_proc,
	.proc_lseek          = seq_lseek,
	.proc_release        = single_release,
};

static int mdns_offload_passthrough_show(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	int list_size = tp->passthrough_list.list_size;
	int i;
	int j;

	for (i = 0, j = 0; i < MDNS_PASSTHROUGH_MAX; i++) {
		if (tp->passthrough_list.list[i].qname_len == 0)
			continue;
		seq_printf(m, "%2d: qname = [%s], size = %d\n", i,
			   tp->passthrough_list.list[i].qname,
			   tp->passthrough_list.list[i].qname_len);
		if (++j == list_size)
			break;
	}
	seq_printf(m, "Total %d entries\n", list_size);

	return 0;
}

static ssize_t mdns_offload_passthrough_store(struct file *file, const char __user *buffer,
					      size_t count, loff_t *pos)
{
	char tmp[TMP_BUF_SIZE];
	u32 len;
	struct net_device *dev = (struct net_device *)
			((struct seq_file *)file->private_data)->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	int qname_len = 0, in_val = 0;
	unsigned char *buforg = NULL;
	unsigned char *tokens[5] = {0};  /* Assuming you have a maximum of 5 tokens */
	unsigned char *delim = ",";  /* The delimiter is a comma "," */
	unsigned char *str_pointer = NULL;
	int token_cnt = 0;

	len = min(count, sizeof(tmp) - 1);

	if (!buffer || copy_from_user(tmp, buffer, len))
		goto exit;

	tmp[len] = '\0';
	buforg = tmp;
	token_cnt = mdns_split_string(buforg, delim, tokens, 3, &str_pointer);
	if (strncmp(tokens[0], "add", 3) == 0) {
		/* echo "add,qname,qname size" > /sys/etn/wol_mdns_offload_passthrough */
		if (token_cnt != 3)
			goto error;
		if (kstrtou32(tokens[2], 0, &in_val)) {
			netdev_err(tp->dev, "qname size = [%s]\n", tokens[2]);
			goto error;
		}
		qname_len = in_val;
		mdns_add_passthrough(tp, tokens[1], qname_len);
	} else if (strncmp(tokens[0], "del", 3) == 0) {
		/* echo "del,qname,qname size" > /sys/etn/wol_mdns_offload_passthrough */
		if (token_cnt != 3)
			goto error;
		if (kstrtou32(tokens[2], 0, &in_val)) {
			netdev_err(tp->dev, "qname size = [%s]\n", tokens[2]);
			goto error;
		}
		qname_len = in_val;
		mdns_del_passthrough(tp, tokens[1], qname_len);
	} else if (strncmp(tokens[0], "clean", 5) == 0) {
		/* echo clean > /sys/etn/wol_mdns_offload_passthrough */
		if (token_cnt != 1) {
			netdev_err(tp->dev, "token_cnt != 1\n");
			goto error;
		}
		mdns_reset_passthrough(tp);
	}

error:
	mdns_free_split_string(tokens, token_cnt);

exit:
	return count;
}

static int mdns_offload_passthrough_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, mdns_offload_passthrough_show, dev);
}

static const struct proc_ops mdns_offload_passthrough_proc_fops = {
	.proc_open	= mdns_offload_passthrough_open,
	.proc_read	= seq_read,
	.proc_write	= mdns_offload_passthrough_store,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
};

static int mdns_offload_protocol_data_show(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	int i;
	int j;
	int k;

	for (i = 0, k = 0; i < MDNS_PACKET_MAX; i++) {
		if (tp->mdns_data_list.list[i].packet_size == 0)
			continue;
		seq_printf(m, "mdns_data_list.list[%d].packet = [%s], packet_size = %d\n", i,
			   tp->mdns_data_list.list[i].packet,
			   tp->mdns_data_list.list[i].packet_size);
		for (j = 0; j < tp->mdns_data_list.list[i].type_size; j++)
			seq_printf(m, "\t list[%d].type[%d].nameoffset = %d, qtype = %d\n",
				   i, j, tp->mdns_data_list.list[i].type[j].nameoffset,
				   tp->mdns_data_list.list[i].type[j].qtype);
		if (++k == tp->mdns_data_list.list_size)
			break;
	}
	seq_printf(m, "Total %d packets\n", tp->mdns_data_list.list_size);

	return 0;
}

static ssize_t mdns_offload_protocol_data_store(struct file *file, const char __user *buffer,
						size_t count, loff_t *pos)
{
	char tmp[TMP_BUF_SIZE];
	u32 len;
	struct net_device *dev = (struct net_device *)
			((struct seq_file *)file->private_data)->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	int type = 0, nameoffset = 0, packet_size = 0, in_val = 0, packet_index = 0;
	unsigned char *buforg = NULL;
	unsigned char *tokens[5] = {0};  /* Assuming you have a maximum of 5 tokens */
	unsigned char *delim = ",";  /* The delimiter is a comma "," */
	unsigned char *str_pointer = NULL;
	int token_cnt = 0;

	len = min(count, sizeof(tmp) - 1);

	if (!buffer || copy_from_user(tmp, buffer, len))
		goto exit;

	tmp[len] = '\0';
	buforg = tmp;

	token_cnt = mdns_split_string(buforg, delim, tokens, 4, &str_pointer);

	/* echo add,qtype,offset,packet size,packet > mdns_offload_protocol_data */
	if (strncmp(tokens[0], "add", strlen("add")) == 0) {
		if (token_cnt != 4)
			goto error;

		if (kstrtou32(tokens[1], 0, &in_val)) {
			netdev_err(tp->dev, "type = %s\n", tokens[1]);
			goto error;
		}
		type = in_val;

		if (kstrtou32(tokens[2], 0, &in_val)) {
			netdev_err(tp->dev, "nameoffset = %s\n", tokens[2]);
			goto error;
		}
		nameoffset = in_val;

		if (kstrtou32(tokens[3], 0, &in_val)) {
			netdev_err(tp->dev, "packet_size = %s\n", tokens[3]);
			goto error;
		}
		packet_size = in_val;
		packet_index = mdns_set_protocol_data_packet(tp, str_pointer, type,
							     nameoffset, packet_size);

		if (packet_index >= 0)
			mdns_set_protocol_data_type(tp, packet_index, type, nameoffset);
	} else if (strncmp(tokens[0], "clean", 5) == 0) {
		/* echo clean > mdns_offload_protocol_data */
		if (token_cnt != 1) {
			netdev_err(tp->dev, "token_cnt != 1\n");
			goto error;
		}
		mdns_reset_protocol_data_packet(tp);
	}

error:
	mdns_free_split_string(tokens, token_cnt);

exit:
	return count;
}

static int mdns_offload_protocol_data_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, mdns_offload_protocol_data_show, dev);
}

static const struct proc_ops mdns_offload_protocol_data_proc_fops = {
	.proc_open	= mdns_offload_protocol_data_open,
	.proc_read	= seq_read,
	.proc_write	= mdns_offload_protocol_data_store,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
};

static int mdns_offload_passthrough_behavior_show(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);

	if (tp->passthrough_behavior == FORWARD_ALL)
		seq_puts(m, "passthrough_behavior: FORWARD_ALL\n");
	else if (tp->passthrough_behavior == DROP_ALL)
		seq_puts(m, "passthrough_behavior: DROP_ALL\n");
	else if (tp->passthrough_behavior == PASSTHROUGH_LIST)
		seq_puts(m, "passthrough_behavior: PASSTHROUGH_LIST\n");

	return 0;
}

static ssize_t
mdns_offload_passthrough_behavior_store(struct file *file, const char __user *buffer,
					size_t count, loff_t *pos)
{
	char tmp[TMP_BUF_SIZE];
	u32 len;
	struct net_device *dev = (struct net_device *)
			((struct seq_file *)file->private_data)->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	int in_val;

	len = min(count, sizeof(tmp) - 1);

	if (!buffer || copy_from_user(tmp, buffer, len))
		goto error;

	tmp[len] = '\0';
	if (kstrtou32(tmp, 0, &in_val)) {
		netdev_err(tp->dev, "invalid value [%s], len = %d\n", tmp, len);
		goto error;
	}
	if (in_val == FORWARD_ALL || in_val == DROP_ALL || in_val == PASSTHROUGH_LIST) {
		tp->passthrough_behavior = in_val;
	} else {
		netdev_err(tp->dev, "invalid value [%s], len = %d\n", tmp, len);
		goto error;
	}

error:
	return count;
}

static int mdns_offload_passthrough_behavior_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, mdns_offload_passthrough_behavior_show, dev);
}

static const struct proc_ops mdns_offload_passthrough_behavior_proc_fops = {
	.proc_open	= mdns_offload_passthrough_behavior_open,
	.proc_read	= seq_read,
	.proc_write	= mdns_offload_passthrough_behavior_store,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
};

static int mdns_wol_ipv4_show(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);

	seq_printf(m, "ipv4: %pI4\n", tp->g_wol_ipv4_addr);
	return 0;
}

static ssize_t mdns_wol_ipv4_store(struct file *file, const char __user *buffer,
				   size_t count, loff_t *pos)
{
	char tmp[TMP_BUF_SIZE];
	u32 len;
	struct net_device *dev = (struct net_device *)
			((struct seq_file *)file->private_data)->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	int i = 0;
	unsigned int ipv4[4] = {0};

	len = min(count, sizeof(tmp) - 1);

	if (!buffer || copy_from_user(tmp, buffer, len))
		goto error;

	tmp[len] = '\0';
	if (sscanf(tmp, "%u.%u.%u.%u", &ipv4[0], &ipv4[1], &ipv4[2], &ipv4[3]) != 4) {
		netdev_err(tp->dev, "invalid ip [%s], len = %d\n", tmp, len);
		goto error;
	}

	for (i = 0; i < 4; i++) {
		if (ipv4[i] > 255) {
			netdev_err(tp->dev, "invalid ipv4[%d] = %d\n", i, ipv4[i]);
			goto error;
		}
		tp->g_wol_ipv4_addr[i] = ipv4[i];
	}

error:
	return count;
}

static int mdns_wol_ipv4_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, mdns_wol_ipv4_show, dev);
}

static const struct proc_ops mdns_wol_ipv4_proc_fops = {
	.proc_open	= mdns_wol_ipv4_open,
	.proc_read	= seq_read,
	.proc_write	= mdns_wol_ipv4_store,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
};

static int mdns_wol_ipv6_show(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);

	seq_printf(m, "ipv6: %pI6\n", tp->g_wol_ipv6_addr);

	return 0;
}

static ssize_t mdns_wol_ipv6_store(struct file *file, const char __user *buffer,
				   size_t count, loff_t *pos)
{
	char tmp[TMP_BUF_SIZE];
	u32 len;
	struct net_device *dev = (struct net_device *)
			((struct seq_file *)file->private_data)->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	int i = 0;
	unsigned int ipv6[16] = {0};

	len = min(count, sizeof(tmp) - 1);

	if (!buffer || copy_from_user(tmp, buffer, len))
		goto error;

	tmp[len] = '\0';
	if (sscanf(tmp, "%x:%x:%x:%x:%x:%x:%x:%x",
		   &ipv6[0], &ipv6[1], &ipv6[2], &ipv6[3],
		   &ipv6[4], &ipv6[5], &ipv6[6], &ipv6[7]) != 8) {
		netdev_err(tp->dev, "invalid ip [%s], len = %d\n", tmp, len);
		goto error;
	}

	for (i = 0; i < 8; i++) {
		if (ipv6[i] > 0xFFFF) {
			netdev_err(tp->dev, "invalid ipv6[%d] = 0x%x\n", i, ipv6[i]);
			goto error;
		}
		*(short *)(tp->g_wol_ipv6_addr + (2 * i)) = cpu_to_be16(ipv6[i]);
	}

error:
	return count;
}

static int mdns_wol_ipv6_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, mdns_wol_ipv6_show, dev);
}

static const struct proc_ops mdns_wol_ipv6_proc_fops = {
	.proc_open	= mdns_wol_ipv6_open,
	.proc_read	= seq_read,
	.proc_write	= mdns_wol_ipv6_store,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
};

static int mdns_offload_state_show(struct seq_file *m, void *v)
{
	struct net_device *dev = m->private;
	struct rtl8169_private *tp = netdev_priv(dev);

	seq_printf(m, "mdns_offload_state = %d\n", tp->mdns_offload_state);

	return 0;
}

static ssize_t mdns_offload_state_store(struct file *file, const char __user *buffer,
					size_t count, loff_t *pos)
{
	char tmp[TMP_BUF_SIZE];
	u32 len;
	struct net_device *dev = (struct net_device *)
			((struct seq_file *)file->private_data)->private;
	struct rtl8169_private *tp = netdev_priv(dev);
	int in_val;

	len = min(count, sizeof(tmp) - 1);

	if (!buffer || copy_from_user(tmp, buffer, len))
		goto error;

	tmp[len] = '\0';
	if (kstrtou32(tmp, 0, &in_val)) {
		netdev_err(tp->dev, "invalid value [%s], len = %d\n", tmp, len);
		goto error;
	}

	if (in_val == 1 || in_val == 0) {
		tp->mdns_offload_state = in_val;
	} else {
		netdev_err(tp->dev, "invalid value [%s](%d), len = %d\n", tmp, in_val, len);
		goto error;
	}

error:
	return count;
}

static int mdns_offload_state_open(struct inode *inode, struct file *file)
{
	struct net_device *dev = proc_get_parent_data(inode);

	return single_open(file, mdns_offload_state_show, dev);
}

static const struct proc_ops mdns_offload_state_proc_fops = {
	.proc_open	= mdns_offload_state_open,
	.proc_read	= seq_read,
	.proc_write	= mdns_offload_state_store,
	.proc_lseek	= seq_lseek,
	.proc_release	= single_release,
};

static int rtl_proc_file_register(struct rtl8169_private *tp)
{
	struct proc_dir_entry *dir_dev = NULL;
	struct proc_dir_entry *entry = NULL;
	int ret = 0;

	/* create /proc/net/$tp->dev->name */
	rtk_proc = proc_mkdir_data(tp->dev->name, 0555, init_net.proc_net, NULL);

	if (!rtk_proc) {
		dev_err(tp_to_dev(tp), "procfs:create /proc/net/%s failed\n", tp->dev->name);
		return -EPERM;
	}

	/* create /proc/net/$tp->dev->name/$KBUILD_MODNAME */
	if (!tp->dir_dev) {
		tp->dir_dev = proc_mkdir_data(KBUILD_MODNAME, S_IFDIR | 0555,
					      rtk_proc, tp->dev);
		dir_dev = tp->dir_dev;

		if (!dir_dev) {
			dev_err(tp_to_dev(tp), "procfs:create %s failed\n", KBUILD_MODNAME);

			if (rtk_proc) {
				remove_proc_entry(tp->dev->name, init_net.proc_net);
				rtk_proc = NULL;
			}
			return -EPERM;
		}
	}

	entry = proc_create_data("wol_enable", S_IFREG | 0644,
				 dir_dev, &wol_proc_fops, NULL);
	if (!entry) {
		dev_err(tp_to_dev(tp), "procfs:create wol_enable failed\n");
		ret |= EPERM;
	}

	entry = proc_create_data("pwr_saving", S_IFREG | 0644,
				 dir_dev, &pwr_saving_proc_fops, NULL);
	if (!entry) {
		dev_err(tp_to_dev(tp), "procfs:create pwr_saving failed\n");
		ret |= EPERM;
	}

	entry = proc_create_data("mac_reinit", S_IFREG | 0644,
				 dir_dev, &mac_reinit_proc_fops, NULL);
	if (!entry) {
		dev_err(tp_to_dev(tp), "procfs:create mac_reinit failed\n");
		ret |= EPERM;
	}

	entry = proc_create_data("phy_reinit", S_IFREG | 0644,
				 dir_dev, &phy_reinit_proc_fops, NULL);
	if (!entry) {
		dev_err(tp_to_dev(tp), "procfs:create phy_reinit failed\n");
		ret |= EPERM;
	}

	entry = proc_create_data("loopback", S_IFREG | 0644,
				 dir_dev, &loopback_proc_fops, NULL);
	if (!entry) {
		dev_err(tp_to_dev(tp), "procfs:create loopback failed\n");
		ret |= EPERM;
	}

	entry = proc_create_data("eee", S_IFREG | 0644,
				 dir_dev, &eee_proc_fops, NULL);
	if (!entry) {
		dev_err(tp_to_dev(tp), "procfs:create eee failed\n");
		ret |= EPERM;
	}

	entry = proc_create_data("driver_var", S_IFREG | 0444,
				 dir_dev, &driver_var_proc_fops, NULL);
	if (!entry) {
		dev_err(tp_to_dev(tp), "procfs:create driver_var failed\n");
		ret |= EPERM;
	}

	entry = proc_create_data("ocp", S_IFREG | 0666,
				 dir_dev, &ocp_proc_fops, NULL);
	if (!entry) {
		dev_err(tp_to_dev(tp), "procfs:create ocp failed\n");
		ret |= EPERM;
	}

	entry = proc_create_data("eth_phy", S_IFREG | 0666,
				 dir_dev, &eth_phy_proc_fops, NULL);
	if (!entry) {
		dev_err(tp_to_dev(tp), "procfs:create eth_phy failed\n");
		ret |= EPERM;
	}

	entry = proc_create_data("eth_led", S_IFREG | 0666,
				 dir_dev, &eth_led_proc_fops, NULL);
	if (!entry) {
		dev_err(tp_to_dev(tp), "procfs:create eth_led failed\n");
		ret |= EPERM;
	}

	entry = proc_create_data("ext_regs", S_IFREG | 0444,
				 dir_dev, &ext_regs_proc_fops, NULL);
	if (!entry) {
		dev_err(tp_to_dev(tp), "procfs:create ext_regs failed\n");
		ret |= EPERM;
	}

	entry = proc_create_data("registers", S_IFREG | 0444,
				 dir_dev, &registers_proc_fops, NULL);
	if (!entry) {
		dev_err(tp_to_dev(tp), "procfs:create registers failed\n");
		ret |= EPERM;
	}

	entry = proc_create_data("tx_desc", S_IFREG | 0444,
				 dir_dev, &tx_desc_proc_fops, NULL);
	if (!entry) {
		dev_err(tp_to_dev(tp), "procfs:create tx_desc failed\n");
		ret |= EPERM;
	}

	entry = proc_create_data("rx_desc", S_IFREG | 0444,
				 dir_dev, &rx_desc_proc_fops, NULL);
	if (!entry) {
		dev_err(tp_to_dev(tp), "procfs:create rx_desc failed\n");
		ret |= EPERM;
	}

	entry = proc_create_data("tally", S_IFREG | 0444,
				 dir_dev, &tally_proc_fops, NULL);
	if (!entry) {
		dev_err(tp_to_dev(tp), "procfs:create tally failed\n");
		ret |= EPERM;
	}

	entry = proc_create_data("wpd_event", S_IFREG | 0444,
				 dir_dev, &wpd_evt_proc_fops, NULL);
	if (!entry) {
		dev_err(tp_to_dev(tp), "procfs:create wpd_event failed\n");
		ret |= EPERM;
	}

	entry = proc_create_data("wol_packet", S_IFREG | 0444,
				 dir_dev, &wol_pkt_proc_fops, NULL);
	if (!entry) {
		dev_err(tp_to_dev(tp), "procfs:create wol_packet failed\n");
		ret |= EPERM;
	}

	entry = proc_create_data("wake_mask", S_IFREG | 0644,
				 dir_dev, &wake_mask_proc_fops, NULL);
	if (!entry) {
		dev_err(tp_to_dev(tp), "procfs:create wake_mask failed\n");
		ret |= EPERM;
	}

	entry = proc_create_data("wake_crc", S_IFREG | 0644,
				 dir_dev, &wake_crc_proc_fops, NULL);
	if (!entry) {
		dev_err(tp_to_dev(tp), "procfs:create wake_crc failed\n");
		ret |= EPERM;
	}

	entry = proc_create_data("wake_idx_en", S_IFREG | 0644,
				 dir_dev, &wake_idx_en_proc_fops, NULL);
	if (!entry) {
		dev_err(tp_to_dev(tp), "procfs:create wake_idx_en failed\n");
		ret |= EPERM;
	}

	entry = proc_create_data("wake_dump", S_IFREG | 0444,
				 dir_dev, &wake_dump_proc_fops, NULL);
	if (!entry) {
		dev_err(tp_to_dev(tp), "procfs:create wake_dump failed\n");
		ret |= EPERM;
	}

	if (tp->chip->features & RTL_FEATURE_PAT_WAKE) {
		entry = proc_create_data("wake_offset", S_IFREG | 0644,
					 dir_dev, &wake_offset_proc_fops, NULL);
		if (!entry) {
			dev_err(tp_to_dev(tp), "procfs:create wake_offset failed\n");
			ret |= EPERM;
		}

		entry = proc_create_data("wake_pattern", S_IFREG | 0644,
					 dir_dev, &wake_pattern_proc_fops, NULL);
		if (!entry) {
			dev_err(tp_to_dev(tp), "procfs:create wake_pattern failed\n");
			ret |= EPERM;
		}
	}

	if (tp->chip->features & RTL_FEATURE_STORM_CTRL) {
		entry = proc_create_data("storm_ctrl", S_IFREG | 0644,
					 dir_dev, &storm_ctrl_proc_fops, NULL);
		if (!entry) {
			dev_err(tp_to_dev(tp), "procfs:create storm_ctrl failed\n");
			ret |= EPERM;
		}
	}

	if (tp->chip->features & RTL_FEATURE_MDNS_OFFLOAD) {
		entry = proc_create_data("mdns_offload_passthrough", S_IFREG | 0644,
					 tp->dir_dev, &mdns_offload_passthrough_proc_fops, NULL);
		if (!entry) {
			dev_err(tp_to_dev(tp), "procfs: create mdns_offload_passthrough failed\n");
			ret |= EPERM;
		}

		entry = proc_create_data("mdns_offload_protocol_data", S_IFREG | 0644,
					 tp->dir_dev, &mdns_offload_protocol_data_proc_fops, NULL);
		if (!entry) {
			dev_err(tp_to_dev(tp), "procfs: create mdns_offload_protocol_data failed\n");
			ret |= EPERM;
		}

		entry = proc_create_data("mdns_offload_passthrough_behavior", S_IFREG | 0644,
					 tp->dir_dev,
					 &mdns_offload_passthrough_behavior_proc_fops, NULL);
		if (!entry) {
			dev_err(tp_to_dev(tp), "procfs: create mdns_offload_passthrough_behavior failed\n");
			ret |= EPERM;
		}

		entry = proc_create_data("mdns_wol_ipv4", S_IFREG | 0644,
					 tp->dir_dev, &mdns_wol_ipv4_proc_fops, NULL);
		if (!entry) {
			dev_err(tp_to_dev(tp), "procfs: create mdns_wol_ipv4 failed\n");
			ret |= EPERM;
		}

		entry = proc_create_data("mdns_wol_ipv6", S_IFREG | 0644,
					 tp->dir_dev, &mdns_wol_ipv6_proc_fops, NULL);
		if (!entry) {
			dev_err(tp_to_dev(tp), "procfs: create mdns_wol_ipv6 failed\n");
			ret |= EPERM;
		}

		entry = proc_create_data("mdns_offload_state", S_IFREG | 0644,
					 tp->dir_dev, &mdns_offload_state_proc_fops, NULL);
		if (!entry) {
			dev_err(tp_to_dev(tp), "procfs: create mdns_offload_state failed\n");
			ret |= EPERM;
		}
	}

	return -ret;
}

static void rtl_proc_file_unregister(struct rtl8169_private *tp)
{
	if (!tp->dir_dev)
		return;

	remove_proc_entry("wol_enable", tp->dir_dev);
	remove_proc_entry("pwr_saving", tp->dir_dev);
	remove_proc_entry("mac_reinit", tp->dir_dev);
	remove_proc_entry("phy_reinit", tp->dir_dev);
	remove_proc_entry("loopback", tp->dir_dev);
	remove_proc_entry("eee", tp->dir_dev);
	remove_proc_entry("driver_var", tp->dir_dev);
	remove_proc_entry("ocp", tp->dir_dev);
	remove_proc_entry("eth_phy", tp->dir_dev);
	remove_proc_entry("eth_led", tp->dir_dev);
	remove_proc_entry("ext_regs", tp->dir_dev);
	remove_proc_entry("registers", tp->dir_dev);
	remove_proc_entry("tx_desc", tp->dir_dev);
	remove_proc_entry("rx_desc", tp->dir_dev);
	remove_proc_entry("tally", tp->dir_dev);
	remove_proc_entry("wpd_event", tp->dir_dev);
	remove_proc_entry("wol_packet", tp->dir_dev);
	remove_proc_entry("wake_mask", tp->dir_dev);
	remove_proc_entry("wake_crc", tp->dir_dev);
	remove_proc_entry("wake_idx_en", tp->dir_dev);
	remove_proc_entry("wake_dump", tp->dir_dev);
	if (tp->chip->features & RTL_FEATURE_PAT_WAKE) {
		remove_proc_entry("wake_offset", tp->dir_dev);
		remove_proc_entry("wake_pattern", tp->dir_dev);
	}
	if (tp->chip->features & RTL_FEATURE_STORM_CTRL)
		remove_proc_entry("storm_ctrl", tp->dir_dev);

	if (tp->chip->features & RTL_FEATURE_MDNS_OFFLOAD) {
		remove_proc_entry("mdns_offload_passthrough", tp->dir_dev);
		remove_proc_entry("mdns_offload_protocol_data", tp->dir_dev);
		remove_proc_entry("mdns_offload_passthrough_behavior", tp->dir_dev);
		remove_proc_entry("mdns_wol_ipv4", tp->dir_dev);
		remove_proc_entry("mdns_wol_ipv6", tp->dir_dev);
		remove_proc_entry("mdns_offload_state", tp->dir_dev);
	}
}
#endif

static __must_check
void *r8169soc_read_otp(struct rtl8169_private *tp, const char *name)
{
	struct device *d = tp_to_dev(tp);
	struct device_node *np = d->of_node;
	struct nvmem_cell *cell;
	unsigned char *buf;
	size_t buf_size;

	cell = of_nvmem_cell_get(np, name);
	if (IS_ERR(cell)) {
		dev_err(d, "failed to get nvmem cell %s: %ld\n",
			name, PTR_ERR(cell));
		return ERR_CAST(cell);
	}

	buf = nvmem_cell_read(cell, &buf_size);
	if (IS_ERR(buf))
		dev_err(d, "failed to read nvmem cell %s: %ld\n",
			name, PTR_ERR(buf));
	nvmem_cell_put(cell);
	return buf;
}

/* dummy functions */
static void dummy_wakeup_set(struct rtl8169_private *tp, bool enable)
{
	/* no wake up */
}

static void dummy_reset_phy_gmac(struct rtl8169_private *tp)
{
	dev_dbg(tp_to_dev(tp), "%s is called\n", __func__);
}

static void dummy_pll_clock_init(struct rtl8169_private *tp)
{
	dev_dbg(tp_to_dev(tp), "%s is called\n", __func__);
}

static void dummy_acp_init(struct rtl8169_private *tp)
{
	/* no acp */
}

static void dummy_mdio_init(struct rtl8169_private *tp)
{
	/* no MDIO init */
}

static void dummy_mac_mcu_patch(struct rtl8169_private *tp)
{
	/* no patch */
}

static void dummy_hw_phy_config(struct rtl8169_private *tp)
{
	/* no config */
}

static void dummy_eee_set(struct rtl8169_private *tp, bool enable)
{
	/* no EEE */
}

static void dummy_led_set(struct rtl8169_private *tp, bool enable)
{
	/* no LED */
}

static void dummy_dump_regs(struct seq_file *m, struct rtl8169_private *tp)
{
	/* no special registers */
}

static void dummy_dump_var(struct seq_file *m, struct rtl8169_private *tp)
{
	/* no special variables */
}

/* RTD119X */
static void rtd119x_reset_phy_gmac(struct rtl8169_private *tp)
{
	struct clk *clk_etn  = clk_get(&tp->pdev->dev, "etn");
	struct clk *clk_etn_sys  = clk_get(&tp->pdev->dev, "etn_sys");
	struct clk *clk_etn_250m = clk_get(&tp->pdev->dev, "etn_250m");
	struct reset_control *rstc_etn =
		reset_control_get_exclusive(&tp->pdev->dev, "rst_etn");
	struct reset_control *rstc_gphy =
		reset_control_get_exclusive(&tp->pdev->dev, "gphy");
	struct reset_control *rstc_gmac =
		reset_control_get_exclusive(&tp->pdev->dev, "gmac");

	/* pre-init clk */
	if (!test_bit(RTL_STATUS_REINIT, tp->status)) {
		clk_prepare_enable(clk_etn);
		clk_prepare_enable(clk_etn_sys);
		clk_prepare_enable(clk_etn_250m);
	}

	/* disable clk and hold reset bits */
	clk_disable_unprepare(clk_etn_sys);
	clk_disable_unprepare(clk_etn_250m);
	clk_disable_unprepare(clk_etn);
	reset_control_assert(rstc_gphy);
	reset_control_assert(rstc_gmac);
	reset_control_assert(rstc_etn);

	/* release resource */
	reset_control_put(rstc_etn);
	reset_control_put(rstc_gphy);
	reset_control_put(rstc_gmac);
	clk_put(clk_etn);
	clk_put(clk_etn_sys);
	clk_put(clk_etn_250m);
}

static void rtd119x_pll_clock_init(struct rtl8169_private *tp)
{
	struct clk *clk_etn  = clk_get(&tp->pdev->dev, "etn");
	struct clk *clk_etn_sys  = clk_get(&tp->pdev->dev, "etn_sys");
	struct clk *clk_etn_250m = clk_get(&tp->pdev->dev, "etn_250m");
	struct reset_control *rstc_etn =
		reset_control_get_exclusive(&tp->pdev->dev, "rst_etn");
	struct reset_control *rstc_gphy =
		reset_control_get_exclusive(&tp->pdev->dev, "gphy");
	struct reset_control *rstc_gmac =
		reset_control_get_exclusive(&tp->pdev->dev, "gmac");

	/* enable clk bits and release reset bits */
	clk_prepare_enable(clk_etn);
	clk_prepare_enable(clk_etn_sys);
	clk_prepare_enable(clk_etn_250m);
	reset_control_deassert(rstc_etn);
	reset_control_deassert(rstc_gphy);
	reset_control_deassert(rstc_gmac);

	fsleep(10000);		/* wait 10ms for GMAC uC to be stable */

	/* release resource */
	reset_control_put(rstc_etn);
	reset_control_put(rstc_gphy);
	reset_control_put(rstc_gmac);
	clk_put(clk_etn);
	clk_put(clk_etn_sys);
	clk_put(clk_etn_250m);
}

static void rtd119x_mac_mcu_patch(struct rtl8169_private *tp)
{
	const struct soc_device_attribute rtk_soc_rtd119x_a00[] = {
		{
			.family = "Realtek Phoenix",
			.revision = "A00",
		},
		{
		/* empty */
		}
	};

	if (soc_device_match(rtk_soc_rtd119x_a00))
		return;

	/* Fix ALDPS */
	RTL_W32(tp, OCPDR, 0xFE140000);
	RTL_W32(tp, OCPDR, 0xFE150000);
	RTL_W32(tp, OCPDR, 0xFE160000);
	RTL_W32(tp, OCPDR, 0xFE170000);
	RTL_W32(tp, OCPDR, 0xFE180000);
	RTL_W32(tp, OCPDR, 0xFE190000);
	RTL_W32(tp, OCPDR, 0xFE1A0000);
	RTL_W32(tp, OCPDR, 0xFE1B0000);
	fsleep(3000);
	RTL_W32(tp, OCPDR, 0xFE130000);

	RTL_W32(tp, OCPDR, 0xFC00E008);
	RTL_W32(tp, OCPDR, 0xFC01E051);
	RTL_W32(tp, OCPDR, 0xFC02E059);
	RTL_W32(tp, OCPDR, 0xFC03E05B);
	RTL_W32(tp, OCPDR, 0xFC04E05D);
	RTL_W32(tp, OCPDR, 0xFC05E05F);
	RTL_W32(tp, OCPDR, 0xFC06E061);
	RTL_W32(tp, OCPDR, 0xFC07E063);
	RTL_W32(tp, OCPDR, 0xFC08C422);
	RTL_W32(tp, OCPDR, 0xFC097380);
	RTL_W32(tp, OCPDR, 0xFC0A49B7);
	RTL_W32(tp, OCPDR, 0xFC0BF003);
	RTL_W32(tp, OCPDR, 0xFC0C1D02);
	RTL_W32(tp, OCPDR, 0xFC0D8D80);
	RTL_W32(tp, OCPDR, 0xFC0EC51D);
	RTL_W32(tp, OCPDR, 0xFC0F73A0);
	RTL_W32(tp, OCPDR, 0xFC101300);
	RTL_W32(tp, OCPDR, 0xFC11F104);
	RTL_W32(tp, OCPDR, 0xFC1273A2);
	RTL_W32(tp, OCPDR, 0xFC131300);
	RTL_W32(tp, OCPDR, 0xFC14F010);
	RTL_W32(tp, OCPDR, 0xFC15C517);
	RTL_W32(tp, OCPDR, 0xFC1676A0);
	RTL_W32(tp, OCPDR, 0xFC1774A2);
	RTL_W32(tp, OCPDR, 0xFC180601);
	RTL_W32(tp, OCPDR, 0xFC193720);
	RTL_W32(tp, OCPDR, 0xFC1A9EA0);
	RTL_W32(tp, OCPDR, 0xFC1B9CA2);
	RTL_W32(tp, OCPDR, 0xFC1CC50F);
	RTL_W32(tp, OCPDR, 0xFC1D73A2);
	RTL_W32(tp, OCPDR, 0xFC1E4023);
	RTL_W32(tp, OCPDR, 0xFC1FF813);
	RTL_W32(tp, OCPDR, 0xFC20F304);
	RTL_W32(tp, OCPDR, 0xFC2173A0);
	RTL_W32(tp, OCPDR, 0xFC224033);
	RTL_W32(tp, OCPDR, 0xFC23F80F);
	RTL_W32(tp, OCPDR, 0xFC24C206);
	RTL_W32(tp, OCPDR, 0xFC257340);
	RTL_W32(tp, OCPDR, 0xFC2649B7);
	RTL_W32(tp, OCPDR, 0xFC27F013);
	RTL_W32(tp, OCPDR, 0xFC28C207);
	RTL_W32(tp, OCPDR, 0xFC29BA00);
	RTL_W32(tp, OCPDR, 0xFC2AC0BC);
	RTL_W32(tp, OCPDR, 0xFC2BD2C8);
	RTL_W32(tp, OCPDR, 0xFC2CD2CC);
	RTL_W32(tp, OCPDR, 0xFC2DC0C4);
	RTL_W32(tp, OCPDR, 0xFC2ED2E4);
	RTL_W32(tp, OCPDR, 0xFC2F100A);
	RTL_W32(tp, OCPDR, 0xFC30104C);
	RTL_W32(tp, OCPDR, 0xFC310C7E);
	RTL_W32(tp, OCPDR, 0xFC321D02);
	RTL_W32(tp, OCPDR, 0xFC33C6F7);
	RTL_W32(tp, OCPDR, 0xFC348DC0);
	RTL_W32(tp, OCPDR, 0xFC351C01);
	RTL_W32(tp, OCPDR, 0xFC36C5F7);
	RTL_W32(tp, OCPDR, 0xFC378CA1);
	RTL_W32(tp, OCPDR, 0xFC38C6F8);
	RTL_W32(tp, OCPDR, 0xFC39BE00);
	RTL_W32(tp, OCPDR, 0xFC3AC5F4);
	RTL_W32(tp, OCPDR, 0xFC3B74A0);
	RTL_W32(tp, OCPDR, 0xFC3C49C0);
	RTL_W32(tp, OCPDR, 0xFC3DF010);
	RTL_W32(tp, OCPDR, 0xFC3E74A2);
	RTL_W32(tp, OCPDR, 0xFC3F76A4);
	RTL_W32(tp, OCPDR, 0xFC404034);
	RTL_W32(tp, OCPDR, 0xFC41F804);
	RTL_W32(tp, OCPDR, 0xFC420601);
	RTL_W32(tp, OCPDR, 0xFC439EA4);
	RTL_W32(tp, OCPDR, 0xFC44E009);
	RTL_W32(tp, OCPDR, 0xFC451D02);
	RTL_W32(tp, OCPDR, 0xFC46C4E4);
	RTL_W32(tp, OCPDR, 0xFC478D80);
	RTL_W32(tp, OCPDR, 0xFC48C5E5);
	RTL_W32(tp, OCPDR, 0xFC4964A1);
	RTL_W32(tp, OCPDR, 0xFC4A4845);
	RTL_W32(tp, OCPDR, 0xFC4B8CA1);
	RTL_W32(tp, OCPDR, 0xFC4CE7EC);
	RTL_W32(tp, OCPDR, 0xFC4D1C20);
	RTL_W32(tp, OCPDR, 0xFC4EC5DC);
	RTL_W32(tp, OCPDR, 0xFC4F8CA1);
	RTL_W32(tp, OCPDR, 0xFC50C2E1);
	RTL_W32(tp, OCPDR, 0xFC51BA00);
	RTL_W32(tp, OCPDR, 0xFC521D02);
	RTL_W32(tp, OCPDR, 0xFC53C606);
	RTL_W32(tp, OCPDR, 0xFC548DC0);
	RTL_W32(tp, OCPDR, 0xFC551D20);
	RTL_W32(tp, OCPDR, 0xFC568DC0);
	RTL_W32(tp, OCPDR, 0xFC57C603);
	RTL_W32(tp, OCPDR, 0xFC58BE00);
	RTL_W32(tp, OCPDR, 0xFC59C0BC);
	RTL_W32(tp, OCPDR, 0xFC5A0E22);
	RTL_W32(tp, OCPDR, 0xFC5BC102);
	RTL_W32(tp, OCPDR, 0xFC5CB900);
	RTL_W32(tp, OCPDR, 0xFC5D02A2);
	RTL_W32(tp, OCPDR, 0xFC5EC602);
	RTL_W32(tp, OCPDR, 0xFC5FBE00);
	RTL_W32(tp, OCPDR, 0xFC600000);
	RTL_W32(tp, OCPDR, 0xFC61C602);
	RTL_W32(tp, OCPDR, 0xFC62BE00);
	RTL_W32(tp, OCPDR, 0xFC630000);
	RTL_W32(tp, OCPDR, 0xFC64C602);
	RTL_W32(tp, OCPDR, 0xFC65BE00);
	RTL_W32(tp, OCPDR, 0xFC660000);
	RTL_W32(tp, OCPDR, 0xFC67C602);
	RTL_W32(tp, OCPDR, 0xFC68BE00);
	RTL_W32(tp, OCPDR, 0xFC690000);
	RTL_W32(tp, OCPDR, 0xFC6AC602);
	RTL_W32(tp, OCPDR, 0xFC6BBE00);
	RTL_W32(tp, OCPDR, 0xFC6C0000);

	RTL_W32(tp, OCPDR, 0xFE138000);
	RTL_W32(tp, OCPDR, 0xFE140FD1);
	RTL_W32(tp, OCPDR, 0xFE150D21);
	RTL_W32(tp, OCPDR, 0xFE16029D);

	/* MDC/MDIO clock speedup */
	RTL_W32(tp, OCPDR, 0xEf080040);
}

static void rtd119x_hw_phy_config(struct rtl8169_private *tp)
{
	int revision = REVISION_NONE;
	const struct soc_device_attribute *soc;
	const struct soc_device_attribute rtk_soc_rtd119x[] = {
		{
			.family = "Realtek Phoenix",
			.revision = "A00",
			.data = (void *)REVISION_A00,
		},
		{
			.family = "Realtek Phoenix",
			.revision = "A01",
			.data = (void *)REVISION_A01,
		},
		{
		/* empty */
		}
	};

	soc = soc_device_match(rtk_soc_rtd119x);
	if (soc)
		revision = (uintptr_t)soc->data;

	switch (revision) {
	case REVISION_A00:
		/* disable green */
		rtl_phy_write(tp, 0x0a43, 0x1b, 0x8011);
		rtl_phy_write(tp, 0x0a43, 0x1c, 0x1737);

		/* write abiq /ldvbias */
		rtl_phy_write(tp, 0x0bcc, 0x11, 0x4444);

		/* R/RC auto k */
		/* default ff08  disable auto k RC/R */
		rtl_phy_write(tp, 0x0a43, 0x1b, 0x8013);
		rtl_phy_write(tp, 0x0a43, 0x1c, 0x0F08);

		/* force tapbin = 4 */
		rtl_phy_write(tp, 0x0bce, 0x10, 0x4444);

		rtl_phy_write(tp, 0x0bcd, 0x17, 0x8888);		/* tx rc */
		rtl_phy_write(tp, 0x0bcd, 0x16, 0x9999);		/* rx rc */

		/* increase sd thd */
		/* master sd  : 0x1e00 */
		rtl_phy_write(tp, 0x0a43, 0x1b, 0x8101);
		rtl_phy_write(tp, 0x0a43, 0x1c, 0x3E00);

		/* default sd  : 0x0e00 */
		rtl_phy_write(tp, 0x0a43, 0x1b, 0x80E2);
		rtl_phy_write(tp, 0x0a43, 0x1c, 0x3E00);

		/* slave sd  : 0x0e00 */
		rtl_phy_write(tp, 0x0a43, 0x1b, 0x8120);
		rtl_phy_write(tp, 0x0a43, 0x1c, 0x3E00);
		break;
	case REVISION_A01:
		/* GDAC updown */
		rtl_phy_write(tp, 0x0bcc, 0x12, 0x00BE);

		/* R_RC */
		/* fbfe [15:8] tapbin tx -3, [7:0] tapbin rx -2 */
		rtl_phy_write(tp, 0x0a43, 0x1b, 0x81DE);
		rtl_phy_write(tp, 0x0a43, 0x1c, 0xFDFE);

		rtl_phy_write(tp, 0x0a43, 0x1b, 0x81E0);
		rtl_phy_write(tp, 0x0a43, 0x1c, 0xFDFF);		/* [15:8] rlen  -3 */

		rtl_phy_write(tp, 0x0a43, 0x1b, 0x81E2);
		/* [15:8] rlen_100 +4, -3+4=1 */
		rtl_phy_write(tp, 0x0a43, 0x1c, 0x0400);
		rtl_phy_write(tp, 0x0a43, 0x1b, 0x80d3);
		/* fnet cable length constant 0aa4 */
		rtl_phy_write(tp, 0x0a43, 0x1c, 0x04a4);

		rtl_phy_write(tp, 0x0a43, 0x1b, 0x8111);
		/* slave cable length constant fa7f */
		rtl_phy_write(tp, 0x0a43, 0x1c, 0x0a7f);

		rtl_phy_write(tp, 0x0a43, 0x1b, 0x810d);
		/* slave const dagc 0606 */
		rtl_phy_write(tp, 0x0a43, 0x1c, 0x5604);

		rtl_phy_write(tp, 0x0a43, 0x1b, 0x80f4);
		/* master cable length delta 3df7 */
		rtl_phy_write(tp, 0x0a43, 0x1c, 0x5df7);

		rtl_phy_write(tp, 0x0a43, 0x1b, 0x80f2);
		/* master cable length constant fa8f */
		rtl_phy_write(tp, 0x0a43, 0x1c, 0x0a8f);

		rtl_phy_write(tp, 0x0a43, 0x1b, 0x80f6);
		/* master delta dagc 63ca */
		rtl_phy_write(tp, 0x0a43, 0x1c, 0x54ca);

		rtl_phy_write(tp, 0x0a43, 0x1b, 0x80ec);
		/* master aagc 146c */
		rtl_phy_write(tp, 0x0a43, 0x1c, 0x007c);

		/* disable ALDPS interrupt */
		rtl_phy_write(tp, 0x0a42, 0x12,
			      rtl_phy_read(tp, 0x0a42, 0x12) & ~BIT(9));

		/* enable ALDPS */
		rtl_phy_write(tp, 0x0a43, 0x10,
			      rtl_phy_read(tp, 0x0a43, 0x10) | BIT(2));
	}
}

static void rtd119x_led_set(struct rtl8169_private *tp, bool enable)
{
	struct reset_control *rstc_etn =
		reset_control_get_exclusive(&tp->pdev->dev, "rst_etn");
	struct reset_control *rstc_gphy =
		reset_control_get_exclusive(&tp->pdev->dev, "gphy");
	struct reset_control *rstc_gmac =
		reset_control_get_exclusive(&tp->pdev->dev, "gmac");

	if (enable) {
		/* hold reset bits */
		reset_control_assert(rstc_gphy);
		reset_control_assert(rstc_gmac);
		reset_control_assert(rstc_etn);

		/* release reset bits */
		reset_control_deassert(rstc_etn);
		reset_control_deassert(rstc_gphy);
		reset_control_deassert(rstc_gmac);

		msleep(300);

		regmap_update_bits_base(tp->iso_base, RTD119X_ISO_MUXPAD0,
					GENMASK(31, 28), (5 << 28), NULL, false, true);
	} else {
		regmap_update_bits_base(tp->iso_base, RTD119X_ISO_MUXPAD0,
					GENMASK(31, 28), 0, NULL, false, true);
	}

	/* release resource */
	reset_control_put(rstc_etn);
	reset_control_put(rstc_gphy);
	reset_control_put(rstc_gmac);
}

/******************* END of RTD119X ****************************/

/* RTD129X */
static void rtd129x_mdio_lock(struct rtl8169_private *tp)
{
	/* disable interrupt from PHY to MCU */
	rtl_ocp_write(tp, 0xFC1E,
		      rtl_ocp_read(tp, 0xFC1E) & ~(BIT(1) | BIT(11) | BIT(12)));
}

static void rtd129x_mdio_unlock(struct rtl8169_private *tp)
{
	u32 tmp;

	/* enable interrupt from PHY to MCU */
	tmp = rtl_ocp_read(tp, 0xFC1E);
	if (tp->output_mode == OUTPUT_RGMII_TO_MAC)
		tmp |= (BIT(11) | BIT(12)); /* ignore BIT(1):mac_intr*/
	else
		tmp |= (BIT(1) | BIT(11) | BIT(12));
	rtl_ocp_write(tp, 0xFC1E, tmp);
}

static void rtd129x_reset_phy_gmac(struct rtl8169_private *tp)
{
	struct clk *clk_etn_sys  = clk_get(&tp->pdev->dev, "etn_sys");
	struct clk *clk_etn_250m = clk_get(&tp->pdev->dev, "etn_250m");
	struct reset_control *rstc_gphy =
		reset_control_get_exclusive(&tp->pdev->dev, "gphy");
	struct reset_control *rstc_gmac =
		reset_control_get_exclusive(&tp->pdev->dev, "gmac");

	if (unlikely(!__clk_is_enabled(clk_etn_sys) || !__clk_is_enabled(clk_etn_250m))) {
		/* pre-init clk bits */
		if (!test_bit(RTL_STATUS_REINIT, tp->status)) {
			clk_prepare_enable(clk_etn_sys);
			clk_prepare_enable(clk_etn_250m);
		}

		/* disable clk bits and hold reset bits */
		clk_disable_unprepare(clk_etn_sys);
		clk_disable_unprepare(clk_etn_250m);
		reset_control_assert(rstc_gphy);
		reset_control_assert(rstc_gmac);
	}

	/* release resource */
	reset_control_put(rstc_gphy);
	reset_control_put(rstc_gmac);
	clk_put(clk_etn_sys);
	clk_put(clk_etn_250m);
}

static void rtd129x_pll_clock_init(struct rtl8169_private *tp)
{
	struct clk *clk_etn_sys  = clk_get(&tp->pdev->dev, "etn_sys");
	struct clk *clk_etn_250m = clk_get(&tp->pdev->dev, "etn_250m");
	struct reset_control *rstc_gphy =
		reset_control_get_exclusive(&tp->pdev->dev, "gphy");
	struct reset_control *rstc_gmac =
		reset_control_get_exclusive(&tp->pdev->dev, "gmac");

	if (likely(__clk_is_enabled(clk_etn_sys) && __clk_is_enabled(clk_etn_250m))) {
		/* enable again to prevent clk framework from disabling them */
		clk_prepare_enable(clk_etn_sys);
		clk_prepare_enable(clk_etn_250m);
	} else {
		/* 1. reg_0x98007088[10] = 1 */
		/* ISO spec, reset bit of gphy */
		reset_control_deassert(rstc_gphy);

		/* 2. CPU software waiting 200uS */
		fsleep(200);

		/* 3. reg_0x98007060[1] = 0 */
		/* ISO spec, Ethernet Boot up bypass gphy ready mode */
		regmap_update_bits_base(tp->iso_base, ISO_ETN_TESTIO,
					BIT(1), 0, NULL, false, true);

		/* 4. reg_0x98007fc0[0] = 0 */
		/* ISO spec, Ethernet Boot up disable dbus clock gating */
		regmap_update_bits_base(tp->iso_base, RTD129X_ISO_DBUS_CTRL,
					BIT(0), 0, NULL, false, true);

		/* 5. CPU software waiting 200uS */
		fsleep(200);

		/* 6. reg_0x9800708c[12:11] = 11 */
		/* ISO spec, clock enable bit for etn clock & etn 250MHz */
		clk_prepare_enable(clk_etn_sys);
		clk_prepare_enable(clk_etn_250m);

		/* 7. reg_0x9800708c[12:11] = 00 */
		/* ISO spec, clock enable bit for etn clock & etn 250MHz */
		clk_disable_unprepare(clk_etn_sys);
		clk_disable_unprepare(clk_etn_250m);

		/* 8. reg_0x98007088[9] = 1 */
		/* ISO spec, reset bit of gmac */
		reset_control_deassert(rstc_gmac);

		/* 9. reg_0x9800708c[12:11] = 11 */
		/* ISO spec, clock enable bit for etn clock & etn 250MHz */
		clk_prepare_enable(clk_etn_sys);
		clk_prepare_enable(clk_etn_250m);

		msleep(100);
	}

	/* release resource */
	reset_control_put(rstc_gphy);
	reset_control_put(rstc_gmac);
	clk_put(clk_etn_sys);
	clk_put(clk_etn_250m);
}

static void rtd129x_mac_setup(struct rtl8169_private *tp)
{
	unsigned int tmp;

	if (tp->output_mode == OUTPUT_EMBEDDED_PHY) {
		tmp = rtl_ocp_read(tp, 0xea34) &
			~(BIT(0) | BIT(1));
		tmp |= BIT(1); /* MII */
		rtl_ocp_write(tp, 0xea34, tmp);
	} else { /* RGMII */
		if (tp->output_mode == OUTPUT_RGMII_TO_PHY) {
			/* # ETN spec, MDC freq=2.5MHz */
			tmp = rtl_ocp_read(tp, 0xde30);
			rtl_ocp_write(tp, 0xde30, tmp & ~(BIT(6) | BIT(7)));
			/* # ETN spec, set external PHY addr */
			tmp = rtl_ocp_read(tp, 0xde24) & ~(0x1F);
			rtl_ocp_write(tp, 0xde24,
				      tmp | (tp->ext_phy_id & 0x1F));
		}

		tmp = rtl_ocp_read(tp, 0xea34) & ~(BIT(0) | BIT(1));
		tmp |= BIT(0); /* RGMII */

		if (tp->output_mode == OUTPUT_RGMII_TO_MAC) {
			tmp &= ~(BIT(3) | BIT(4));
			tmp |= BIT(4); /* speed: 1G */
			tmp |= BIT(2); /* full duplex */
		}
		if (tp->rgmii.rx_delay == RTD129X_RGMII_DELAY_0NS)
			tmp &= ~BIT(6);
		else
			tmp |= BIT(6);

		if (tp->rgmii.tx_delay == RTD129X_RGMII_DELAY_0NS)
			tmp &= ~BIT(7);
		else
			tmp |= BIT(7);

		rtl_ocp_write(tp, 0xea34, tmp);

		/* adjust RGMII voltage */
		switch (tp->rgmii.voltage) {
		case RTD129X_VOLTAGE_1_DOT_8V:
			regmap_write(tp->sb2_base, RTD129X_SB2_PFUNC_RG0, 0);
			regmap_write(tp->sb2_base, RTD129X_SB2_PFUNC_RG1, 0x44444444);
			regmap_write(tp->sb2_base, RTD129X_SB2_PFUNC_RG2, 0x24444444);
			break;
		case RTD129X_VOLTAGE_2_DOT_5V:
			regmap_write(tp->sb2_base, RTD129X_SB2_PFUNC_RG0, 0);
			regmap_write(tp->sb2_base, RTD129X_SB2_PFUNC_RG1, 0x44444444);
			regmap_write(tp->sb2_base, RTD129X_SB2_PFUNC_RG2, 0x64444444);
			break;
		case RTD129X_VOLTAGE_3_DOT_3V:
			regmap_write(tp->sb2_base, RTD129X_SB2_PFUNC_RG0, 0x3F);
			regmap_write(tp->sb2_base, RTD129X_SB2_PFUNC_RG1, 0);
			regmap_write(tp->sb2_base, RTD129X_SB2_PFUNC_RG2, 0xA4000000);
		}

		/* switch RGMII/MDIO to GMAC */
		regmap_update_bits_base(tp->iso_base, RTD129X_ISO_RGMII_MDIO_TO_GMAC,
					BIT(1), BIT(1), NULL, false, true);

		if (tp->output_mode == OUTPUT_RGMII_TO_MAC) {
			/* force GMAC link up */
			rtl_ocp_write(tp, 0xde40, 0x30EC);
			/* ignore mac_intr from PHY */
			tmp = rtl_ocp_read(tp, 0xfc1e) & ~BIT(1);
			rtl_ocp_write(tp, 0xfc1e, tmp);
		}
	}
}

static void rtd129x_mac_mcu_patch(struct rtl8169_private *tp)
{
	const struct soc_device_attribute rtk_soc_rtd129x_bxx[] = {
		{
			.family = "Realtek Kylin",
			.revision = "B0*",
		},
		{
		/* empty */
		}
	};

	/* disable break point */
	rtl_ocp_write(tp, 0xfc28, 0);
	rtl_ocp_write(tp, 0xfc2a, 0);
	rtl_ocp_write(tp, 0xfc2c, 0);
	rtl_ocp_write(tp, 0xfc2e, 0);
	rtl_ocp_write(tp, 0xfc30, 0);
	rtl_ocp_write(tp, 0xfc32, 0);
	rtl_ocp_write(tp, 0xfc34, 0);
	rtl_ocp_write(tp, 0xfc36, 0);
	fsleep(3000);

	/* disable base address */
	rtl_ocp_write(tp, 0xfc26, 0);

	/* patch code */
	rtl_ocp_write(tp, 0xf800, 0xE008);
	rtl_ocp_write(tp, 0xf802, 0xE012);
	rtl_ocp_write(tp, 0xf804, 0xE044);
	rtl_ocp_write(tp, 0xf806, 0xE046);
	rtl_ocp_write(tp, 0xf808, 0xE048);
	rtl_ocp_write(tp, 0xf80a, 0xE04A);
	rtl_ocp_write(tp, 0xf80c, 0xE04C);
	rtl_ocp_write(tp, 0xf80e, 0xE04E);
	rtl_ocp_write(tp, 0xf810, 0x44E3);
	rtl_ocp_write(tp, 0xf812, 0xC708);
	rtl_ocp_write(tp, 0xf814, 0x75E0);
	rtl_ocp_write(tp, 0xf816, 0x485D);
	rtl_ocp_write(tp, 0xf818, 0x9DE0);
	rtl_ocp_write(tp, 0xf81a, 0xC705);
	rtl_ocp_write(tp, 0xf81c, 0xC502);
	rtl_ocp_write(tp, 0xf81e, 0xBD00);
	rtl_ocp_write(tp, 0xf820, 0x01EE);
	rtl_ocp_write(tp, 0xf822, 0xE85A);
	rtl_ocp_write(tp, 0xf824, 0xE000);
	rtl_ocp_write(tp, 0xf826, 0xC72D);
	rtl_ocp_write(tp, 0xf828, 0x76E0);
	rtl_ocp_write(tp, 0xf82a, 0x49ED);
	rtl_ocp_write(tp, 0xf82c, 0xF026);
	rtl_ocp_write(tp, 0xf82e, 0xC02A);
	rtl_ocp_write(tp, 0xf830, 0x7400);
	rtl_ocp_write(tp, 0xf832, 0xC526);
	rtl_ocp_write(tp, 0xf834, 0xC228);
	rtl_ocp_write(tp, 0xf836, 0x9AA0);
	rtl_ocp_write(tp, 0xf838, 0x73A2);
	rtl_ocp_write(tp, 0xf83a, 0x49BE);
	rtl_ocp_write(tp, 0xf83c, 0xF11E);
	rtl_ocp_write(tp, 0xf83e, 0xC324);
	rtl_ocp_write(tp, 0xf840, 0x9BA2);
	rtl_ocp_write(tp, 0xf842, 0x73A2);
	rtl_ocp_write(tp, 0xf844, 0x49BE);
	rtl_ocp_write(tp, 0xf846, 0xF0FE);
	rtl_ocp_write(tp, 0xf848, 0x73A2);
	rtl_ocp_write(tp, 0xf84a, 0x49BE);
	rtl_ocp_write(tp, 0xf84c, 0xF1FE);
	rtl_ocp_write(tp, 0xf84e, 0x1A02);
	rtl_ocp_write(tp, 0xf850, 0x49C9);
	rtl_ocp_write(tp, 0xf852, 0xF003);
	rtl_ocp_write(tp, 0xf854, 0x4821);
	rtl_ocp_write(tp, 0xf856, 0xE002);
	rtl_ocp_write(tp, 0xf858, 0x48A1);
	rtl_ocp_write(tp, 0xf85a, 0x73A2);
	rtl_ocp_write(tp, 0xf85c, 0x49BE);
	rtl_ocp_write(tp, 0xf85e, 0xF10D);
	rtl_ocp_write(tp, 0xf860, 0xC313);
	rtl_ocp_write(tp, 0xf862, 0x9AA0);
	rtl_ocp_write(tp, 0xf864, 0xC312);
	rtl_ocp_write(tp, 0xf866, 0x9BA2);
	rtl_ocp_write(tp, 0xf868, 0x73A2);
	rtl_ocp_write(tp, 0xf86a, 0x49BE);
	rtl_ocp_write(tp, 0xf86c, 0xF0FE);
	rtl_ocp_write(tp, 0xf86e, 0x73A2);
	rtl_ocp_write(tp, 0xf870, 0x49BE);
	rtl_ocp_write(tp, 0xf872, 0xF1FE);
	rtl_ocp_write(tp, 0xf874, 0x48ED);
	rtl_ocp_write(tp, 0xf876, 0x9EE0);
	rtl_ocp_write(tp, 0xf878, 0xC602);
	rtl_ocp_write(tp, 0xf87a, 0xBE00);
	rtl_ocp_write(tp, 0xf87c, 0x0532);
	rtl_ocp_write(tp, 0xf87e, 0xDE00);
	rtl_ocp_write(tp, 0xf880, 0xE85A);
	rtl_ocp_write(tp, 0xf882, 0xE086);
	rtl_ocp_write(tp, 0xf884, 0x0A44);
	rtl_ocp_write(tp, 0xf886, 0x801F);
	rtl_ocp_write(tp, 0xf888, 0x8015);
	rtl_ocp_write(tp, 0xf88a, 0x0015);
	rtl_ocp_write(tp, 0xf88c, 0xC602);
	rtl_ocp_write(tp, 0xf88e, 0xBE00);
	rtl_ocp_write(tp, 0xf890, 0x0000);
	rtl_ocp_write(tp, 0xf892, 0xC602);
	rtl_ocp_write(tp, 0xf894, 0xBE00);
	rtl_ocp_write(tp, 0xf896, 0x0000);
	rtl_ocp_write(tp, 0xf898, 0xC602);
	rtl_ocp_write(tp, 0xf89a, 0xBE00);
	rtl_ocp_write(tp, 0xf89c, 0x0000);
	rtl_ocp_write(tp, 0xf89e, 0xC602);
	rtl_ocp_write(tp, 0xf8a0, 0xBE00);
	rtl_ocp_write(tp, 0xf8a2, 0x0000);
	rtl_ocp_write(tp, 0xf8a4, 0xC602);
	rtl_ocp_write(tp, 0xf8a6, 0xBE00);
	rtl_ocp_write(tp, 0xf8a8, 0x0000);
	rtl_ocp_write(tp, 0xf8aa, 0xC602);
	rtl_ocp_write(tp, 0xf8ac, 0xBE00);
	rtl_ocp_write(tp, 0xf8ae, 0x0000);

	/* enable base address */
	rtl_ocp_write(tp, 0xfc26, 0x8000);

	/* enable breakpoint */
	rtl_ocp_write(tp, 0xfc28, 0x01ED);
	rtl_ocp_write(tp, 0xfc2a, 0x0531);

	if (tp->eee_enable) {
		/* EEE MAC mode */
		rtl_ocp_write(tp, 0xe040,
			      rtl_ocp_read(tp, 0xe040) | (BIT(1) | BIT(0)));
		/* EEE+ MAC mode */
		rtl_ocp_write(tp, 0xe080, rtl_ocp_read(tp, 0xe080) | BIT(1));
		/* enable EEE of 10Mbps */
		rtl_patchphy(tp, 0x0a43, 25, BIT(4));
	}

	if (soc_device_match(rtk_soc_rtd129x_bxx))
		rtd129x_mac_setup(tp);
}

static void rtd129x_hw_phy_config(struct rtl8169_private *tp)
{
	/* enable ALDPS mode */
	rtl_w1w0_phy(tp, 0x0a43, 24, BIT(2), BIT(12) | BIT(1) | BIT(0));
}

static void rtd129x_eee_set(struct rtl8169_private *tp, bool enable)
{
	if (enable) {
		rtl_mmd_write(tp, 0x7, 0x3c, 0x6);

		/* turn on EEE */
		rtl_ocp_write(tp, 0xe040,
			      rtl_ocp_read(tp, 0xe040) | BIT(1) | BIT(0));
		/* turn on EEE+ */
		rtl_ocp_write(tp, 0xe080, rtl_ocp_read(tp, 0xe080) | BIT(1));
	} else {
		rtl_mmd_write(tp, 0x7, 0x3c, 0);

		/* turn off EEE */
		rtl_ocp_write(tp, 0xe040,
			      rtl_ocp_read(tp, 0xe040) & ~(BIT(1) | BIT(0)));
		/* turn off EEE+ */
		rtl_ocp_write(tp, 0xe080, rtl_ocp_read(tp, 0xe080) & ~BIT(1));
	}
}

static void rtd129x_led_set(struct rtl8169_private *tp, bool enable)
{
	if (enable) {
		regmap_update_bits_base(tp->iso_base, RTD129X_ISO_MUXPAD0,
					GENMASK(29, 26), (5 << 26), NULL, false, true);
	} else {
		regmap_update_bits_base(tp->iso_base, RTD129X_ISO_MUXPAD0,
					GENMASK(29, 26), 0, NULL, false, true);
	}
}

static void rtd129x_dump_regs(struct seq_file *m, struct rtl8169_private *tp)
{
	u32 val;

	regmap_read(tp->iso_base, ISO_UMSK_ISR, &val);
	seq_printf(m, "ISO_UMSK_ISR\t[0x98007004] = %08x\n", val);
	regmap_read(tp->iso_base, ISO_PWRCUT_ETN, &val);
	seq_printf(m, "ISO_PWRCUT_ETN\t[0x9800705c] = %08x\n", val);
	regmap_read(tp->iso_base, ISO_ETN_TESTIO, &val);
	seq_printf(m, "ISO_ETN_TESTIO\t[0x98007060] = %08x\n", val);
	regmap_read(tp->iso_base, ISO_SOFT_RESET, &val);
	seq_printf(m, "ETN_RESET_CTRL\t[0x98007088] = %08x\n", val);
	regmap_read(tp->iso_base, ISO_CLOCK_ENABLE, &val);
	seq_printf(m, "ETN_CLK_CTRL\t[0x9800708c] = %08x\n", val);
}

static void rtd129x_dump_var(struct seq_file *m, struct rtl8169_private *tp)
{
	seq_printf(m, "rgmii_voltage\t%d\n", tp->rgmii.voltage);
	seq_printf(m, "rgmii_tx_delay\t%d\n", tp->rgmii.tx_delay);
	seq_printf(m, "rgmii_rx_delay\t%d\n", tp->rgmii.rx_delay);
}

/******************* END of RTD129X ****************************/

/* RTD139X */
#define MDIO_WAIT_TIMEOUT	100
static void rtd139x_mdio_lock(struct rtl8169_private *tp)
{
	u32 wait_cnt = 0;
	u32 log_de4e = 0;

	/* disable EEE IMR */
	rtl_ocp_write(tp, 0xE044,
		      rtl_ocp_read(tp, 0xE044) &
		      ~(BIT(3) | BIT(2) | BIT(1) | BIT(0)));
	/* disable timer 2 */
	rtl_ocp_write(tp, 0xE404,
		      rtl_ocp_read(tp, 0xE404) | BIT(9));
	/* wait MDIO channel is free */
	log_de4e = BIT(0) & rtl_ocp_read(tp, 0xDE4E);
	log_de4e = (log_de4e << 1) |
		(BIT(0) & rtl_ocp_read(tp, 0xDE4E));
	/* check if 0 for continuous 2 times */
	while (0 != (((0x1 << 2) - 1) & log_de4e)) {
		wait_cnt++;
		udelay(1);
		log_de4e = (log_de4e << 1) | (BIT(0) &
			rtl_ocp_read(tp, 0xDE4E));
		if (wait_cnt > MDIO_WAIT_TIMEOUT)
			break;
	}
	/* enter driver mode */
	rtl_ocp_write(tp, 0xDE42, rtl_ocp_read(tp, 0xDE42) | BIT(0));
	if (wait_cnt > MDIO_WAIT_TIMEOUT)
		dev_err(tp_to_dev(tp), "%s:%d: MDIO lock failed\n", __func__, __LINE__);
}

static void rtd139x_mdio_unlock(struct rtl8169_private *tp)
{
	/* exit driver mode */
	rtl_ocp_write(tp, 0xDE42, rtl_ocp_read(tp, 0xDE42) & ~BIT(0));
	/* enable timer 2 */
	rtl_ocp_write(tp, 0xE404,
		      rtl_ocp_read(tp, 0xE404) & ~BIT(9));
	/* enable EEE IMR */
	rtl_ocp_write(tp, 0xE044,
		      rtl_ocp_read(tp, 0xE044) | BIT(3) | BIT(2) | BIT(1) |
				   BIT(0));
}

static void rtd139x_reset_phy_gmac(struct rtl8169_private *tp)
{
	struct clk *clk_etn_sys  = clk_get(&tp->pdev->dev, "etn_sys");
	struct clk *clk_etn_250m = clk_get(&tp->pdev->dev, "etn_250m");
	struct reset_control *rstc_gphy =
		reset_control_get_exclusive(&tp->pdev->dev, "gphy");
	struct reset_control *rstc_gmac =
		reset_control_get_exclusive(&tp->pdev->dev, "gmac");
	struct clk *clk_en_sds = NULL;
	struct reset_control *rstc_sds_reg = NULL;
	struct reset_control *rstc_sds = NULL;
	struct reset_control *rstc_pcie0_power = NULL;
	struct reset_control *rstc_pcie0_phy = NULL;
	struct reset_control *rstc_pcie0_sgmii_mdio = NULL;
	struct reset_control *rstc_pcie0_phy_mdio = NULL;

	/* pre-init clk bits */
	if (!test_bit(RTL_STATUS_REINIT, tp->status)) {
		clk_prepare_enable(clk_etn_sys);
		clk_prepare_enable(clk_etn_250m);
	}

	/* reg_0x9800708c[12:11] = 00 */
	/* ISO spec, clock enable bit for etn clock & etn 250MHz */
	clk_disable_unprepare(clk_etn_sys);
	clk_disable_unprepare(clk_etn_250m);

	/* reg_0x98007088[10:9] = 00 */
	/* ISO spec, rstn_gphy & rstn_gmac */
	reset_control_assert(rstc_gphy);
	reset_control_assert(rstc_gmac);

	/* reg_0x98007060[1] = 1 */
	/* ISO spec, bypass mode enable */
	regmap_update_bits_base(tp->iso_base, ISO_ETN_TESTIO,
				BIT(1), BIT(1), NULL, false, true);

	/* reg_0x9800705c = 0x00616703 */
	/* ISO spec, default value */
	regmap_write(tp->iso_base, ISO_PWRCUT_ETN, 0x00616703);

	/* RESET for SGMII if needed */
	if (tp->output_mode != OUTPUT_EMBEDDED_PHY) {
		clk_en_sds = clk_get(&tp->pdev->dev, "sds");
		rstc_sds_reg = reset_control_get_exclusive(&tp->pdev->dev,
							   "sds_reg");
		rstc_sds = reset_control_get_exclusive(&tp->pdev->dev, "sds");
		rstc_pcie0_power =
			reset_control_get_exclusive(&tp->pdev->dev,
						    "pcie0_power");
		rstc_pcie0_phy =
			reset_control_get_exclusive(&tp->pdev->dev,
						    "pcie0_phy");
		rstc_pcie0_sgmii_mdio =
			reset_control_get_exclusive(&tp->pdev->dev,
						    "pcie0_sgmii_mdio");
		rstc_pcie0_phy_mdio =
			reset_control_get_exclusive(&tp->pdev->dev,
						    "pcie0_phy_mdio");

		if (!test_bit(RTL_STATUS_REINIT, tp->status))
			clk_prepare_enable(clk_en_sds);

		/* reg_0x9800000c[7] = 0 */
		/* CRT spec, clk_en_sds */
		clk_disable_unprepare(clk_en_sds);

		/* reg_0x98000000[4:3] = 00 */
		/* CRT spec, rstn_sds_reg & rstn_sds */
		reset_control_assert(rstc_sds);
		reset_control_assert(rstc_sds_reg);

		/* reg_0x98000004[7] = 0 */
		/* reg_0x98000004[14] = 0 */
		/* CRT spec, rstn_pcie0_power & rstn_pcie0_phy */
		reset_control_assert(rstc_pcie0_power);
		reset_control_assert(rstc_pcie0_phy);

		/* reg_0x98000050[13] = 0 */
		/* reg_0x98000050[16] = 0 */
		/* CRT spec, rstn_pcie0_sgmii_mdio & rstn_pcie0_phy_mdio */
		reset_control_assert(rstc_pcie0_sgmii_mdio);
		reset_control_assert(rstc_pcie0_phy_mdio);

		reset_control_put(rstc_sds_reg);
		reset_control_put(rstc_sds);
		reset_control_put(rstc_pcie0_power);
		reset_control_put(rstc_pcie0_phy);
		reset_control_put(rstc_pcie0_sgmii_mdio);
		reset_control_put(rstc_pcie0_phy_mdio);
		clk_put(clk_en_sds);
	}

	fsleep(1000);

	/* release resource */
	reset_control_put(rstc_gphy);
	reset_control_put(rstc_gmac);
	clk_put(clk_etn_sys);
	clk_put(clk_etn_250m);
}

static void rtd139x_acp_init(struct rtl8169_private *tp)
{
	struct device *d = tp_to_dev(tp);
	u32 tmp;
	u32 val;

	/* SBX spec, Select ETN access DDR path. */
	if (tp->acp_enable) {
		/* reg_0x9801c20c[6] = 1 */
		/* SBX spec, Mask ETN_ALL to SB3 DBUS REQ */
		regmap_update_bits_base(tp->sbx_base, RTD139X_SBX_SB3_CHANNEL_REQ_MASK,
					BIT(6), BIT(6), NULL, false, true);

		dev_dbg(d, "wait all SB3 access finished...");
		tmp = 0;
		regmap_read(tp->sbx_base, RTD139X_SBX_SB3_CHANNEL_REQ_BUSY, &val);
		while ((val & BIT(6)) != 0) {
			fsleep(1000);
			tmp += 1;
			if (tmp >= 100) {
				dev_err(d, "\n wait SB3 access failed (wait %d ms)\n", tmp);
				set_bit(RTL_STATUS_HW_FAIL, tp->status);
				break;
			}
			regmap_read(tp->sbx_base, RTD139X_SBX_SB3_CHANNEL_REQ_BUSY, &val);
		}
		if (tmp < 100)
			dev_dbg(d, "done.\n");

		/* reg_0x9801d100[29] = 0 */
		/* SCPU wrapper spec, CLKACP division, 0 = div 2, 1 = div 3 */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD139X_SC_WRAP_CRT_CTRL,
					BIT(29), 0, NULL, false, true);

		/* reg_0x9801d124[1:0] = 00 */
		/* SCPU wrapper spec, ACP master active, 0 = active */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD139X_SC_WRAP_INTERFACE_EN,
					GENMASK(1, 0), 0, NULL, false, true);

		/* reg_0x9801d100[30] = 1 */
		/* SCPU wrapper spec, dy_icg_en_acp */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD139X_SC_WRAP_CRT_CTRL,
					BIT(30), BIT(30), NULL, false, true);

		/* reg_0x9801d100[21] = 1 */
		/* SCPU wrapper spec, ACP CLK enable */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD139X_SC_WRAP_CRT_CTRL,
					BIT(21), BIT(21), NULL, false, true);

		/* reg_0x9801d100[14] = 1 */
		/* Do not apply reset to ACP port axi3 master */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD139X_SC_WRAP_CRT_CTRL,
					BIT(14), BIT(14), NULL, false, true);

		/* reg_0x9801d800[3:0] = 0111 */
		/* reg_0x9801d800[7:4] = 0111 */
		/* reg_0x9801d800[9] = 1 */
		/* reg_0x9801d800[20:16] = 01100 */
		/* reg_0x9801d800[28:24] = 01110 */
		/* Configure control of ACP port */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD139X_SC_WRAP_ACP_CTRL,
					GENMASK(28, 24) | GENMASK(20, 16) |
					BIT(9) | GENMASK(7, 4) | GENMASK(3, 0),
					(0x0e << 24) | (0x0c << 16) | BIT(9) |
					(0x7 << 4) | (0x7 << 0),
					NULL, false, true);

		/* reg_0x9801d030[28] = 1 */
		/* SCPU wrapper spec, dy_icg_en_acp */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD139X_SC_WRAP_ACP_CRT_CTRL,
					BIT(28), BIT(28), NULL, false, true);

		/* reg_0x9801d030[16] = 1 */
		/* SCPU wrapper spec, ACP CLK Enable for acp of scpu_chip_top */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD139X_SC_WRAP_ACP_CRT_CTRL,
					BIT(16), BIT(16), NULL, false, true);

		/* reg_0x9801d030[0] = 1 */
		/* Do not apply reset to ACP port axi3 master */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD139X_SC_WRAP_ACP_CRT_CTRL,
					BIT(0), BIT(0), NULL, false, true);

		/* reg_0x9801c814[17] = 1 */
		/* through ACP to SCPU_ACP */
		regmap_update_bits_base(tp->sbx_base, RTD139X_SBX_ACP_MISC_CTRL,
					BIT(17), BIT(17), NULL, false, true);

		dev_dbg(d, "ARM ACP on\n.");
	}
}

static void rtd139x_pll_clock_init(struct rtl8169_private *tp)
{
	struct clk *clk_etn_sys  = clk_get(&tp->pdev->dev, "etn_sys");
	struct clk *clk_etn_250m = clk_get(&tp->pdev->dev, "etn_250m");
	struct reset_control *rstc_gphy =
		reset_control_get_exclusive(&tp->pdev->dev, "gphy");
	struct reset_control *rstc_gmac =
		reset_control_get_exclusive(&tp->pdev->dev, "gmac");

	/* reg_0x98007004[27] = 1 */
	/* ISO spec, ETN_PHY_INTR, clear ETN interrupt for ByPassMode */
	regmap_write(tp->iso_base, ISO_UMSK_ISR, BIT(27));

	/* reg_0x98007088[10] = 1 */
	/* ISO spec, reset bit of gphy */
	reset_control_deassert(rstc_gphy);

	fsleep(1000);	/* wait 1ms for PHY PLL stable */

	/* In Hercules, EPHY need choose the bypass mode or Non-bypass mode */
	/* Bypass mode : ETN MAC bypass efuse update flow.
	 * SW need to take this sequence.
	 */
	/* Non-Bypass mode : ETN MAC set efuse update and efuse_rdy setting */
	/* Default : Bypass mode (0x9800_7060[1] = 1'b1) */
	if (!tp->bypass_enable) {
		/* reg_0x98007060[1] = 0 */
		/* ISO spec, bypass mode disable */
		regmap_update_bits_base(tp->iso_base, ISO_ETN_TESTIO,
					BIT(1), 0, NULL, false, true);
	} else {
		/* reg_0x98007060[1] = 1 */
		/* ISO spec, bypass mode enable */
		/* bypass mode, SW need to handle the EPHY Status check ,
		 * EFUSE data update and EPHY fuse_rdy setting.
		 */
		regmap_update_bits_base(tp->iso_base, ISO_ETN_TESTIO,
					BIT(1), BIT(1), NULL, false, true);
	}

	/* reg_0x98007088[9] = 1 */
	/* ISO spec, reset bit of gmac */
	reset_control_deassert(rstc_gmac);

	/* reg_0x9800708c[12:11] = 11 */
	/* ISO spec, clock enable bit for etn clock & etn 250MHz */
	clk_prepare_enable(clk_etn_sys);
	clk_prepare_enable(clk_etn_250m);

	fsleep(10000);	/* wait 10ms for GMAC uC to be stable */

	/* release resource */
	reset_control_put(rstc_gphy);
	reset_control_put(rstc_gmac);
	clk_put(clk_etn_sys);
	clk_put(clk_etn_250m);
}

/* Hercules only uses 13 bits in OTP 0x9801_72C8[12:8] and 0x9801_72D8[7:0]
 * if 0x9801_72C8[12] is 1, then
 * 0x9801_72C8[11:8] (R-calibration) is used to set PHY
 * 0x9801_72D8[7:0] is used to set idac_fine for PHY
 */
static void rtd139x_load_otp_content(struct rtl8169_private *tp)
{
	int otp;
	u16 tmp;
	u8 *buf;

	buf = r8169soc_read_otp(tp, "para");
	if (IS_ERR(buf))
		goto set_idac;

	otp = *buf;
	/* OTP[4] = valid flag, OTP[3:0] = content */
	if (0 != ((0x1 << 4) & otp)) {
		/* frc_r_value_default = 0x8 */
		tmp = otp ^ RTD139X_R_K_DEFAULT;
		rtl_phy_write(tp, 0x0bc0, 20,
			      tmp | (rtl_phy_read(tp, 0x0bc0, 20) & ~(0x1f << 0)));
	}

	kfree(buf);

set_idac:

	buf = r8169soc_read_otp(tp, "idac");
	if (IS_ERR(buf))
		return;

	otp = *buf;
	tmp = otp ^ RTD139X_IDAC_FINE_DEFAULT;	/* IDAC_FINE_DEFAULT = 0x33 */
	tmp += tp->amp_k_offset;
	rtl_phy_write(tp, 0x0bc0, 23,
		      tmp | (rtl_phy_read(tp, 0x0bc0, 23) & ~(0xff << 0)));

	kfree(buf);
}

static u32 rtd139x_serdes_init(struct rtl8169_private *tp)
{
	struct device *d = tp_to_dev(tp);
	u32 stable_ticks;
	u32 tmp;
	u32 val;
	struct clk *clk_en_sds = clk_get(&tp->pdev->dev, "sds");
	struct reset_control *rstc_sds_reg =
		reset_control_get_exclusive(&tp->pdev->dev, "sds_reg");
	struct reset_control *rstc_sds =
		reset_control_get_exclusive(&tp->pdev->dev, "sds");
	struct reset_control *rstc_pcie0_power =
		reset_control_get_exclusive(&tp->pdev->dev, "pcie0_power");
	struct reset_control *rstc_pcie0_phy =
		reset_control_get_exclusive(&tp->pdev->dev, "pcie0_phy");
	struct reset_control *rstc_pcie0_sgmii_mdio =
		reset_control_get_exclusive(&tp->pdev->dev, "pcie0_sgmii_mdio");
	struct reset_control *rstc_pcie0_phy_mdio =
		reset_control_get_exclusive(&tp->pdev->dev, "pcie0_phy_mdio");

	/* reg_0x98000000[4:3] = 11 */
	/* CRT spec, rstn_sds_reg & rstn_sds */
	reset_control_deassert(rstc_sds);
	reset_control_deassert(rstc_sds_reg);

	/* reg_0x98000004[7] = 1 */
	/* reg_0x98000004[14] = 1 */
	/* CRT spec, rstn_pcie0_power & rstn_pcie0_phy */
	reset_control_deassert(rstc_pcie0_power);
	reset_control_deassert(rstc_pcie0_phy);

	/* reg_0x98000050[13] = 1 */
	/* reg_0x98000050[16] = 1 */
	/* CRT spec, rstn_pcie0_sgmii_mdio & rstn_pcie0_phy_mdio */
	reset_control_deassert(rstc_pcie0_sgmii_mdio);
	reset_control_deassert(rstc_pcie0_phy_mdio);

	/* reg_0x9800000c[7] = 1 */
	/* CRT spec, clk_en_sds */
	clk_prepare_enable(clk_en_sds);

	/* reg_0x9800705c[6] = 1 */
	/* ISO spec, set PCIE channel to SGMII */
	regmap_update_bits_base(tp->iso_base, ISO_PWRCUT_ETN,
				BIT(6), BIT(6), NULL, false, true);

	/* reg_0x9800705c[7] = 1 */
	/* ISO spec, set internal MDIO to PCIE */
	regmap_update_bits_base(tp->iso_base, ISO_PWRCUT_ETN,
				BIT(7), BIT(7), NULL, false, true);

	/* ### Beginning of SGMII DPHY register tuning ### */
	/* reg_0x9800705c[20:16] = 00000 */
	/* ISO spec, set internal PHY addr to SERDES_DPHY_0 */
	__int_set_phy_addr(tp, RTD139X_SERDES_DPHY_0);

	/* # DPHY spec, DPHY reg13[8:7]=00, choose 1.25GHz */
	int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x0d,
		       int_mdio_read(tp, CURRENT_MDIO_PAGE, 0x0d) & ~(BIT(8) | BIT(7)));

	/* # DPHY spec, 5GHz tuning */
	int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x4, 0x52f5);
	int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x5, 0xead7);
	int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x6, 0x0010);
	int_mdio_write(tp, CURRENT_MDIO_PAGE, 0xa, 0xc653);
	int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x1, 0xa830);

	/* reg_0x9800705c[20:16] = 00001 */
	/* ISO spec, set internal PHY addr to SERDES_DPHY_1 */
	__int_set_phy_addr(tp, RTD139X_SERDES_DPHY_1);

	/* RTD139X_TX_SWING_1040MV */
	int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x0, 0xd4aa);
	int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x1, 0x88aa);

	/* reg_0x9800705c[20:16] = 00001 */
	/* ISO spec, set internal PHY addr to INT_PHY_ADDR */
	__int_set_phy_addr(tp, RTD139X_INT_PHY_ADDR);

	tp->ext_phy = true;
	fsleep(10000);     /* wait for clock stable */
	/* ext_phy == true now */

	/* reg_0x981c8070[9:0] = 0000000110 */
	/* SDS spec, set SP_CFG_SDS_DBG_SEL to get PHY_Ready */
	regmap_update_bits_base(tp->sds_base, RTD139X_SDS_REG28,
				GENMASK(9, 0), (0x006 << 0), NULL, false, true);

	tmp = 0;
	stable_ticks = 0;
	while (stable_ticks < 10) {
		/* # SDS spec, wait for phy ready */
		regmap_read(tp->sds_base, RTD139X_SDS_REG29, &val);
		if ((val & BIT(14)) == 0) {
			stable_ticks = 0;
			if (tmp >= 100) {
				dev_err(d, "SGMII PHY not ready in 100ms\n");
				set_bit(RTL_STATUS_HW_FAIL, tp->status);
				break;
			}
		}
		stable_ticks++;
		tmp++;
		fsleep(1000);
	}
	if (stable_ticks > 0)
		dev_info(d, "SGMII PHY is ready in %d ms", tmp);

	/* reg_0x9800705c[4] = 1 */
	/* ISO spec, set data path to SGMII */
	regmap_update_bits_base(tp->iso_base, ISO_PWRCUT_ETN,
				BIT(4), BIT(4), NULL, false, true);

	/* reg_0x981c8008[9:8] = 00 */
	/* # SDS spec, SP_SDS_FRC_AN, SERDES auto mode */
	regmap_update_bits_base(tp->sds_base, RTD139X_SDS_REG02,
				GENMASK(9, 8), 0, NULL, false, true);

	/* # SDS spec, wait for SERDES link up */
	tmp = 0;
	stable_ticks = 0;
	while (stable_ticks < 10) {
		regmap_read(tp->sds_base, RTD139X_SDS_MISC, &val);
		if ((val & GENMASK(13, 12)) != GENMASK(13, 12)) {
			stable_ticks = 0;
			if (tmp >= 100) {
				dev_err(d, "SGMII link down in 100ms\n");
				set_bit(RTL_STATUS_HW_FAIL, tp->status);
				break;
			}
		}
		stable_ticks++;
		tmp++;
		fsleep(1000);
	}
	if (stable_ticks > 0)
		dev_info(d, "SGMII link up in %d ms", tmp);

	reset_control_put(rstc_sds_reg);
	reset_control_put(rstc_sds);
	reset_control_put(rstc_pcie0_power);
	reset_control_put(rstc_pcie0_phy);
	reset_control_put(rstc_pcie0_sgmii_mdio);
	reset_control_put(rstc_pcie0_phy_mdio);
	clk_put(clk_en_sds);
	return 0;
}

static void rtd139x_phy_iol_tuning(struct rtl8169_private *tp)
{
	int revision = REVISION_NONE;
	const struct soc_device_attribute *soc;
	const struct soc_device_attribute rtk_soc_rtd139x[] = {
		{
			.family = "Realtek Hercules",
			.revision = "A00",
			.data = (void *)REVISION_A00,
		},
		{
			.family = "Realtek Hercules",
			.revision = "A01",
			.data = (void *)REVISION_A01,
		},
		{
			.family = "Realtek Hercules",
			.revision = "A02",
			.data = (void *)REVISION_A02,
		},
		{
		/* empty */
		}
	};

	soc = soc_device_match(rtk_soc_rtd139x);
	if (soc)
		revision = (uintptr_t)soc->data;

	switch (revision) {
	case REVISION_A00: /* TSMC, cut A */
	case REVISION_A01: /* TSMC, cut B */
		/* idacfine */
		int_mdio_write(tp, 0x0bc0, 23, 0x0088);

		/* abiq */
		int_mdio_write(tp, 0x0bc0, 21, 0x0004);

		/* ldvbias */
		int_mdio_write(tp, 0x0bc0, 22, 0x0777);

		/* iatt */
		int_mdio_write(tp, 0x0bd0, 16, 0x0300);

		/* vcm_ref, cf_l */
		int_mdio_write(tp, 0x0bd0, 17, 0xe8ca);
		break;
	case REVISION_A02: /* UMC, cut C */
		/* 100M Swing */
		/* idac_fine_mdix, idac_fine_mdi */
		int_mdio_write(tp, 0x0bc0, 23, 0x0044);

		/* 100M Tr/Tf */
		/* abiq_10m=0x0, abiq_100m_short=0x4, abiq_normal=0x6 */
		int_mdio_write(tp, 0x0bc0, 21, 0x0046);

		/* 10M */
		/* ldvbias_10m=0x7, ldvbias_10m_short=0x4, ldvbias_normal=0x4 */
		int_mdio_write(tp, 0x0bc0, 22, 0x0744);

		/* vcmref=0x0, cf_l=0x3 */
		int_mdio_write(tp, 0x0bd0, 17, 0x18ca);

		/* iatt=0x2 */
		int_mdio_write(tp, 0x0bd0, 16, 0x0200);
		break;

	/* default: */
	}
}

static void rtd139x_mdio_init(struct rtl8169_private *tp)
{
	struct device *d = tp_to_dev(tp);
	u32 tmp;
	u32 val;
	struct pinctrl_state *ps_sgmii_mdio;

	/* ETN_PHY_INTR, wait interrupt from PHY and it means MDIO is ready */
	tmp = 0;
	regmap_read(tp->iso_base, ISO_UMSK_ISR, &val);
	while ((val & BIT(27)) == 0) {
		tmp += 1;
		fsleep(1000);
		if (tmp >= 100) {
			dev_err(d, "PHY_Status timeout.\n");
			set_bit(RTL_STATUS_HW_FAIL, tp->status);
			break;
		}
		regmap_read(tp->iso_base, ISO_UMSK_ISR, &val);
	}
	dev_info(d, "wait %d ms for PHY interrupt. UMSK_ISR = 0x%x\n", tmp, val);

	/* In Hercules ByPass mode,
	 * SW need to handle the EPHY Status check ,
	 * OTP data update and EPHY fuse_rdy setting.
	 */
	if (tp->bypass_enable) {
		/* PHY will stay in state 1 mode */
		tmp = 0;
		while (0x1 != (int_mdio_read(tp, 0x0a42, 16) & 0x07)) {
			tmp += 1;
			fsleep(1000);
			if (tmp >= 2000) {
				dev_err(d, "PHY status is not 0x1 in bypass mode, current = 0x%02x\n",
					(int_mdio_read(tp, 0x0a42, 16) & 0x07));
				set_bit(RTL_STATUS_HW_FAIL, tp->status);
				break;
			}
		}

		/* adjust FE PHY electrical characteristics */
		rtd139x_phy_iol_tuning(tp);

		/* 1. read OTP 0x9801_72C8[12:8]
		 * 2. xor 0x08
		 * 3. set value to PHY registers to correct R-calibration
		 * 4. read OTP 0x9801_72D8[7:0]
		 * 5. xor 0x33
		 * 6. set value to PHY registers to correct AMP
		 */
		rtd139x_load_otp_content(tp);

		/* fill fuse_rdy & rg_ext_ini_done */
		int_mdio_write(tp, 0x0a46, 20,
			       (int_mdio_read(tp, 0x0a46, 20) | (BIT(1) | BIT(0))));
	} else {
		/* adjust FE PHY electrical characteristics */
		rtd139x_phy_iol_tuning(tp);
	}

	/* init_autoload_done = 1 */
	tmp = rtl_ocp_read(tp, 0xe004);
	tmp |= BIT(7);
	rtl_ocp_write(tp, 0xe004, tmp);
	/* ee_mode = 3 */
	rtl_unlock_config_regs(tp);

	/* wait LAN-ON */
	tmp = 0;
	do {
		tmp += 1;
		fsleep(1000);
		if (tmp >= 2000) {
			dev_err(d, "PHY status is not 0x3, current = 0x%02x\n",
				(int_mdio_read(tp, 0x0a42, 16) & 0x07));
			set_bit(RTL_STATUS_HW_FAIL, tp->status);
			break;
		}
	} while (0x3 != (int_mdio_read(tp, 0x0a42, 16) & 0x07));
	dev_info(d, "wait %d ms for PHY ready, current = 0x%x\n",
		 tmp, int_mdio_read(tp, 0x0a42, 16));

	if (tp->output_mode == OUTPUT_EMBEDDED_PHY) {
		/* Init PHY path */
		/* reg_0x9800705c[5] = 0 */
		/* reg_0x9800705c[7] = 0 */
		/* ISO spec, set internal MDIO to access PHY */
		regmap_update_bits_base(tp->iso_base, ISO_PWRCUT_ETN,
					BIT(7) | BIT(5), 0, NULL, false, true);

		/* reg_0x9800705c[4] = 0 */
		/* ISO spec, set data path to access PHY */
		regmap_update_bits_base(tp->iso_base, ISO_PWRCUT_ETN,
					BIT(4), 0, NULL, false, true);

		/* # ETN spec, GMAC data path select MII-like(embedded GPHY),
		 * not SGMII(external PHY)
		 */
		tmp = rtl_ocp_read(tp, 0xea34) & ~(BIT(0) | BIT(1));
		tmp |= BIT(1); /* MII */
		rtl_ocp_write(tp, 0xea34, tmp);

		/* disable GMII for embedded FEPHY */
		tp->chip->features &= ~RTL_FEATURE_GMII;
	} else {
		/* SGMII */
		/* # ETN spec, adjust MDC freq=2.5MHz */
		rtl_ocp_write(tp, 0xDE30,
			      rtl_ocp_read(tp, 0xDE30) & ~(BIT(7) | BIT(6)));
		/* # ETN spec, set external PHY addr */
		rtl_ocp_write(tp, 0xDE24,
			      ((rtl_ocp_read(tp, 0xDE24) & ~(0x1f << 0)) |
					     (tp->ext_phy_id & 0x1f)));
		/* ISO mux spec, GPIO29 is set to MDC pin */
		/* ISO mux spec, GPIO46 is set to MDIO pin */
		ps_sgmii_mdio = pinctrl_lookup_state(tp->pc, "sgmii");
		pinctrl_select_state(tp->pc, ps_sgmii_mdio);

		/* check if external PHY is available */
		dev_info(d, "Searching external PHY...");
		tp->ext_phy = true;
		tmp = 0;
		while (ext_mdio_read(tp, 0x0a43, 31) != 0x0a43) {
			tmp += 1;
			fsleep(1000);
			if (tmp >= 2000) {
				dev_err(d, "\n External SGMII PHY not found, current = 0x%02x\n",
					ext_mdio_read(tp, 0x0a43, 31));
				set_bit(RTL_STATUS_HW_FAIL, tp->status);
				break;
			}
		}
		if (tmp < 2000)
			dev_info(d, "found.\n");

		/* lower SGMII TX swing of RTL8211FS to reduce EMI */
		/* TX swing = 470mV, default value */
		ext_mdio_write(tp, 0x0dcd, 16, 0x104e);

		tp->ext_phy = false;

		/* # ETN spec, GMAC data path select SGMII(external PHY),
		 * not MII-like(embedded GPHY)
		 */
		tmp = rtl_ocp_read(tp, 0xea34) & ~(BIT(0) | BIT(1));
		tmp |= BIT(1) | BIT(0); /* SGMII */
		rtl_ocp_write(tp, 0xea34, tmp);

		if (rtd139x_serdes_init(tp) != 0)
			dev_err(d, "SERDES init failed\n");
		/* ext_phy == true now */

		/* SDS spec, auto update SGMII link capability */
		regmap_update_bits_base(tp->sds_base, RTD139X_SDS_LINK,
					BIT(2), BIT(2), NULL, false, true);
	}
}

static void rtd139x_eee_set(struct rtl8169_private *tp, bool enable)
{
	if (enable) {
		tp->chip->mdio_lock(tp);
		if (tp->output_mode == OUTPUT_EMBEDDED_PHY) {
			/* 100M EEE capability */
			rtl_mmd_write(tp, 0x7, 60, BIT(1));
			/* 10M EEE */
			rtl_phy_write(tp, 0x0a43, 25,
				      rtl_phy_read(tp, 0x0a43, 25) | BIT(4) | BIT(2));
			/* disable dynamic RX power in PHY */
			rtl_phy_write(tp, 0x0bd0, 21,
				      (rtl_phy_read(tp, 0x0bd0, 21) & ~BIT(8)) | BIT(9));
		} else { /* SGMII */
			/* 1000M & 100M EEE capability */
			rtl_mmd_write(tp, 0x7, 60, (BIT(2) | BIT(1)));
			/* 10M EEE */
			rtl_phy_write(tp, 0x0a43, 25,
				      rtl_phy_read(tp, 0x0a43, 25) | BIT(4) | BIT(2));
		}
		/* EEE MAC mode */
		rtl_ocp_write(tp, 0xe040,
			      rtl_ocp_read(tp, 0xe040) | BIT(1) | BIT(0));
		/* EEE+ MAC mode */
		/* timer to wait FEPHY ready */
		rtl_ocp_write(tp, 0xe08a, 0x00a7);
		rtl_ocp_write(tp, 0xe080, rtl_ocp_read(tp, 0xe080) | BIT(1));
	} else {
		if (tp->output_mode == OUTPUT_EMBEDDED_PHY) {
			/* no EEE capability */
			rtl_mmd_write(tp, 0x7, 60, 0);
			/* 10M EEE */
			rtl_phy_write(tp, 0x0a43, 25,
				      rtl_phy_read(tp, 0x0a43, 25) & ~(BIT(4) | BIT(2)));
		} else { /* SGMII */
			/* no EEE capability */
			rtl_mmd_write(tp, 0x7, 60, 0);
			/* 10M EEE */
			rtl_phy_write(tp, 0x0a43, 25,
				      rtl_phy_read(tp, 0x0a43, 25) & ~(BIT(4) | BIT(2)));
		}
		/* reset to default value */
		rtl_ocp_write(tp, 0xe040, rtl_ocp_read(tp, 0xe040) | BIT(13));
		/* EEE MAC mode */
		rtl_ocp_write(tp, 0xe040,
			      rtl_ocp_read(tp, 0xe040) & ~(BIT(1) | BIT(0)));
		/* EEE+ MAC mode */
		rtl_ocp_write(tp, 0xe080, rtl_ocp_read(tp, 0xe080) & ~BIT(1));
		rtl_ocp_write(tp, 0xe08a, 0x003f); /* default value */
	}
}

static void rtd139x_led_set(struct rtl8169_private *tp, bool enable)
{
	if (enable) {
		regmap_update_bits_base(tp->pinctrl_base, RTD139X_ISO_TESTMUX_MUXPAD1,
					GENMASK(7, 4), (5 << 4), NULL, false, true);
	} else {
		regmap_update_bits_base(tp->pinctrl_base, RTD139X_ISO_TESTMUX_MUXPAD1,
					GENMASK(7, 4), 0, NULL, false, true);
	}
}

static void rtd139x_dump_regs(struct seq_file *m, struct rtl8169_private *tp)
{
	u32 val;

	regmap_read(tp->iso_base, ISO_UMSK_ISR, &val);
	seq_printf(m, "ISO_UMSK_ISR\t[0x98007004] = %08x\n", val);
	regmap_read(tp->iso_base, ISO_PWRCUT_ETN, &val);
	seq_printf(m, "ISO_PWRCUT_ETN\t[0x9800705c] = %08x\n", val);
	regmap_read(tp->iso_base, ISO_ETN_TESTIO, &val);
	seq_printf(m, "ISO_ETN_TESTIO\t[0x98007060] = %08x\n", val);
	regmap_read(tp->iso_base,  ISO_SOFT_RESET, &val);
	seq_printf(m, "ETN_RESET_CTRL\t[0x98007088] = %08x\n", val);
	regmap_read(tp->iso_base,  ISO_CLOCK_ENABLE, &val);
	seq_printf(m, "ETN_CLK_CTRL\t[0x9800708c] = %08x\n", val);
	if (tp->output_mode != OUTPUT_EMBEDDED_PHY) {
		regmap_read(tp->sds_base, RTD139X_SDS_REG02, &val);
		seq_printf(m, "SDS_REG02\t[0x981c8008] = %08x\n", val);
		regmap_read(tp->sds_base, RTD139X_SDS_REG28, &val);
		seq_printf(m, "SDS_REG28\t[0x981c8070] = %08x\n", val);
		regmap_read(tp->sds_base, RTD139X_SDS_REG29, &val);
		seq_printf(m, "SDS_REG29\t[0x981c8074] = %08x\n", val);
		regmap_read(tp->sds_base, RTD139X_SDS_MISC, &val);
		seq_printf(m, "SDS_MISC\t\t[0x981c9804] = %08x\n", val);
		regmap_read(tp->sds_base, RTD139X_SDS_LINK, &val);
		seq_printf(m, "SDS_LINK\t\t[0x981c9810] = %08x\n", val);
	}
}

static void rtd139x_dump_var(struct seq_file *m, struct rtl8169_private *tp)
{
	seq_printf(m, "bypass_enable\t%d\n", tp->bypass_enable);
}

/******************* END of RTD139X ****************************/

/* RTD16XX */
static void rtd16xx_reset_phy_gmac(struct rtl8169_private *tp)
{
	struct clk *clk_etn_sys = clk_get(&tp->pdev->dev, "etn_sys");
	struct clk *clk_etn_250m = clk_get(&tp->pdev->dev, "etn_250m");
	struct reset_control *rstc_gphy =
		reset_control_get_exclusive(&tp->pdev->dev, "gphy");
	struct reset_control *rstc_gmac =
		reset_control_get_exclusive(&tp->pdev->dev, "gmac");
	struct clk *clk_en_sds = NULL;
	struct reset_control *rstc_sds_reg = NULL;
	struct reset_control *rstc_sds = NULL;
	struct reset_control *rstc_pcie0_power = NULL;
	struct reset_control *rstc_pcie0_phy = NULL;
	struct reset_control *rstc_pcie0_sgmii_mdio = NULL;
	struct reset_control *rstc_pcie0_phy_mdio = NULL;

	/* pre-init clk bits */
	if (!test_bit(RTL_STATUS_REINIT, tp->status)) {
		clk_prepare_enable(clk_etn_sys);
		clk_prepare_enable(clk_etn_250m);
	}

	if (tp->output_mode != OUTPUT_EMBEDDED_PHY) {
		clk_en_sds = clk_get(&tp->pdev->dev, "sds");
		rstc_sds_reg =
			reset_control_get_exclusive(&tp->pdev->dev, "sds_reg");
		rstc_sds = reset_control_get_exclusive(&tp->pdev->dev, "sds");
		rstc_pcie0_power =
			reset_control_get_exclusive(&tp->pdev->dev, "pcie0_power");
		rstc_pcie0_phy =
			reset_control_get_exclusive(&tp->pdev->dev, "pcie0_phy");
		rstc_pcie0_sgmii_mdio =
			reset_control_get_exclusive(&tp->pdev->dev, "pcie0_sgmii_mdio");
		rstc_pcie0_phy_mdio =
			reset_control_get_exclusive(&tp->pdev->dev, "pcie0_phy_mdio");
	}

	/* reg_0x9800708c[12:11] = 00 */
	/* ISO spec, clock enable bit for etn clock & etn 250MHz */
	clk_disable_unprepare(clk_etn_sys);
	clk_disable_unprepare(clk_etn_250m);

	/* reg_0x98007088[10:9] = 00 */
	/* ISO spec, rstn_gphy & rstn_gmac */
	reset_control_assert(rstc_gphy);
	reset_control_assert(rstc_gmac);

	/* reg_0x98007060[1] = 1 */
	/* ISO spec, bypass mode enable */
	regmap_update_bits_base(tp->iso_base, ISO_ETN_TESTIO,
				BIT(1), BIT(1), NULL, false, true);

	/* reg_0x9800705c = 0x00616703 */
	/* ISO spec, default value */
	regmap_write(tp->iso_base, ISO_PWRCUT_ETN, 0x00616703);

	if (tp->output_mode != OUTPUT_EMBEDDED_PHY) {
		/* RESET for SGMII if needed */
		if (!test_bit(RTL_STATUS_REINIT, tp->status))
			clk_prepare_enable(clk_en_sds);

		/* reg_0x98000050[13:12] = 10 */
		/* CRT spec, clk_en_sds */
		clk_disable_unprepare(clk_en_sds);

		/* reg_0x98000000[7:6] = 10 */
		/* reg_0x98000000[9:8] = 10 */
		/* CRT spec, rstn_sds_reg & rstn_sds */
		reset_control_assert(rstc_sds);
		reset_control_assert(rstc_sds_reg);

		/* reg_0x98000004[25:24] = 10 CRT spec, rstn_pcie0_sgmii_mdio */
		/* reg_0x98000004[23:22] = 10 CRT spec, rstn_pcie0_phy_mdio */
		/* reg_0x98000004[19:18] = 10 CRT spec, rstn_pcie0_power */
		/* reg_0x98000004[13:12] = 10 CRT spec, rstn_pcie0_phy */
		reset_control_assert(rstc_pcie0_sgmii_mdio);
		reset_control_assert(rstc_pcie0_phy_mdio);
		reset_control_assert(rstc_pcie0_power);
		reset_control_assert(rstc_pcie0_phy);
	}

	fsleep(1000);

	/* release resource */
	if (tp->output_mode != OUTPUT_EMBEDDED_PHY) {
		reset_control_put(rstc_sds_reg);
		reset_control_put(rstc_sds);
		reset_control_put(rstc_pcie0_power);
		reset_control_put(rstc_pcie0_phy);
		reset_control_put(rstc_pcie0_sgmii_mdio);
		reset_control_put(rstc_pcie0_phy_mdio);
		clk_put(clk_en_sds);
	}
	reset_control_put(rstc_gphy);
	reset_control_put(rstc_gmac);
	clk_put(clk_etn_sys);
	clk_put(clk_etn_250m);
}

static void rtd16xx_acp_init(struct rtl8169_private *tp)
{
	struct device *d = tp_to_dev(tp);
	u32 tmp;
	u32 val;

	/* SBX spec, Select ETN access DDR path. */
	if (tp->acp_enable) {
		/* reg_0x9801c20c[6] = 1 */
		/* SBX spec, Mask ETN_ALL to SB3 DBUS REQ */
		regmap_update_bits_base(tp->sbx_base, RTD16XX_SBX_SB3_CHANNEL_REQ_MASK,
					BIT(6), BIT(6), NULL, false, true);

		dev_info(d, "wait all SB3 access finished...");
		tmp = 0;
		regmap_read(tp->sbx_base, RTD16XX_SBX_SB3_CHANNEL_REQ_BUSY, &val);
		while (val & BIT(6)) {
			fsleep(1000);
			tmp += 1;
			if (tmp >= 100) {
				dev_err(d, "\n wait SB3 access failed (wait %d ms)\n", tmp);
				set_bit(RTL_STATUS_HW_FAIL, tp->status);
				break;
			}
			regmap_read(tp->sbx_base, RTD16XX_SBX_SB3_CHANNEL_REQ_BUSY, &val);
		}
		if (tmp < 100)
			dev_info(d, "done.\n");

		/* reg_0x9801d100[29] = 0 */
		/* SCPU wrapper spec, CLKACP division, 0 = div 2, 1 = div 3 */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD16XX_SC_WRAP_CRT_CTRL,
					BIT(29), 0, NULL, false, true);

		/* reg_0x9801d124[1:0] = 00 */
		/* SCPU wrapper spec, ACP master active, 0 = active */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD16XX_SC_WRAP_INTERFACE_EN,
					GENMASK(1, 0), 0, NULL, false, true);

		/* reg_0x9801d100[30] = 1 */
		/* SCPU wrapper spec, dy_icg_en_acp */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD16XX_SC_WRAP_CRT_CTRL,
					BIT(30), BIT(30), NULL, false, true);

		/* reg_0x9801d100[21] = 1 */
		/* SCPU wrapper spec, ACP CLK enable */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD16XX_SC_WRAP_CRT_CTRL,
					BIT(21), BIT(21), NULL, false, true);

		/* reg_0x9801d100[14] = 1 */
		/* SCPU wrapper spec,
		 * Do not apply reset to ACP port axi3 master
		 */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD16XX_SC_WRAP_CRT_CTRL,
					BIT(14), BIT(14), NULL, false, true);

		/* reg_0x9801d800[3:0] = 0111 */
		/* reg_0x9801d800[7:4] = 0111 */
		/* reg_0x9801d800[9] = 1 */
		/* reg_0x9801d800[20:16] = 01100 */
		/* reg_0x9801d800[28:24] = 01110 */
		/* Configure control of ACP port */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD16XX_SC_WRAP_ACP_CTRL,
					GENMASK(28, 24) | GENMASK(20, 16) |
					BIT(9) | GENMASK(7, 4) | GENMASK(3, 0),
					(0x0e << 24) | (0x0c << 16) | BIT(9) |
					(0x7 << 4) | (0x7 << 0),
					NULL, false, true);

		/* reg_0x9801d030[28] = 1 */
		/* SCPU wrapper spec, dy_icg_en_acp */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD16XX_SC_WRAP_ACP_CRT_CTRL,
					BIT(28), BIT(28), NULL, false, true);

		/* reg_0x9801d030[16] = 1 */
		/* SCPU wrapper spec, ACP CLK Enable for acp of scpu_chip_top */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD16XX_SC_WRAP_ACP_CRT_CTRL,
					BIT(16), BIT(16), NULL, false, true);

		/* reg_0x9801d030[0] = 1 */
		/* SCPU wrapper spec,
		 * Do not apply reset to ACP port axi3 master
		 */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD16XX_SC_WRAP_ACP_CRT_CTRL,
					BIT(0), BIT(0), NULL, false, true);

		/* reg_0x9801c814[17] = 1 */
		/* through ACP to SCPU_ACP */
		regmap_update_bits_base(tp->sbx_base, RTD16XX_SBX_ACP_MISC_CTRL,
					BIT(17), BIT(17), NULL, false, true);

		/* SBX spec, Remove mask ETN_ALL to ACP DBUS REQ */
		regmap_update_bits_base(tp->sbx_base, RTD16XX_SBX_ACP_CHANNEL_REQ_MASK,
					BIT(1), 0, NULL, false, true);

		dev_dbg(d, "ARM ACP on\n.");
	} else {
		/* SBX spec, Mask ETN_ALL to ACP DBUS REQ */
		regmap_update_bits_base(tp->sbx_base, RTD16XX_SBX_ACP_CHANNEL_REQ_MASK,
					BIT(1), BIT(1), NULL, false, true);

		dev_info(d, "wait all ACP access finished...");
		tmp = 0;
		regmap_read(tp->sbx_base, RTD16XX_SBX_ACP_CHANNEL_REQ_BUSY, &val);
		while (val & BIT(1)) {
			fsleep(1000);
			tmp += 1;
			if (tmp >= 100) {
				dev_err(d, "\n ACP channel is still busy (wait %d ms)\n", tmp);
				set_bit(RTL_STATUS_HW_FAIL, tp->status);
				break;
			}
			regmap_read(tp->sbx_base, RTD16XX_SBX_ACP_CHANNEL_REQ_BUSY, &val);
		}
		if (tmp < 100)
			dev_info(d, "done.\n");

		/* SCPU wrapper spec, Inactive MP4 AINACTS signal */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD16XX_SC_WRAP_INTERFACE_EN,
					GENMASK(1, 0), GENMASK(1, 0), NULL, false, true);

		/* SCPU wrapper spec, nACPRESET_DVFS & CLKENACP_DVFS */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD16XX_SC_WRAP_CRT_CTRL,
					(BIT(21) | BIT(14)), 0, NULL, false, true);

		/* SCPU wrapper spec, nACPRESET & CLKENACP */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD16XX_SC_WRAP_ACP_CRT_CTRL,
					(BIT(16) | BIT(0)), 0, NULL, false, true);

		/* reg_0x9801c814[17] = 0 */
		/* SBX spec, Switch ETN_ALL to DC_SYS path */
		regmap_update_bits_base(tp->sbx_base, RTD16XX_SBX_ACP_MISC_CTRL,
					BIT(17), 0, NULL, false, true);

		/* SBX spec, Remove mask ETN_ALL to SB3 DBUS REQ */
		regmap_update_bits_base(tp->sbx_base, RTD16XX_SBX_SB3_CHANNEL_REQ_MASK,
					BIT(6), 0, NULL, false, true);

		dev_dbg(d, "ARM ACP off\n.");
	}
}

static void rtd16xx_pll_clock_init(struct rtl8169_private *tp)
{
	struct clk *clk_etn_sys = clk_get(&tp->pdev->dev, "etn_sys");
	struct clk *clk_etn_250m = clk_get(&tp->pdev->dev, "etn_250m");
	struct reset_control *rstc_gphy =
		reset_control_get_exclusive(&tp->pdev->dev, "gphy");
	struct reset_control *rstc_gmac =
		reset_control_get_exclusive(&tp->pdev->dev, "gmac");

	/* reg_0x98007004[27] = 1 */
	/* ISO spec, ETN_PHY_INTR, clear ETN interrupt for ByPassMode */
	regmap_write(tp->iso_base, ISO_UMSK_ISR, BIT(27));

	/* reg_0x98007088[10] = 1 */
	/* ISO spec, reset bit of gphy */
	reset_control_deassert(rstc_gphy);

	fsleep(1000);		/* wait 1ms for PHY PLL stable */

	/* Thor only supports the bypass mode */
	/* Bypass mode : ETN MAC bypass efuse update flow.
	 * SW need to take this sequence.
	 */
	/* reg_0x98007060[1] = 1 */
	/* ISO spec, bypass mode enable */
	/* bypass mode, SW need to handle the EPHY Status check ,
	 * EFUSE data update and EPHY fuse_rdy setting.
	 */
	regmap_update_bits_base(tp->iso_base, ISO_ETN_TESTIO,
				BIT(1), BIT(1), NULL, false, true);

	/* reg_0x98007088[9] = 1 */
	/* ISO spec, reset bit of gmac */
	reset_control_deassert(rstc_gmac);

	/* reg_0x9800708c[12:11] = 11 */
	/* ISO spec, clock enable bit for etn clock & etn 250MHz */
	clk_prepare_enable(clk_etn_sys);
	clk_prepare_enable(clk_etn_250m);

	fsleep(10000);		/* wait 10ms for GMAC uC to be stable */

	/* release resource */
	reset_control_put(rstc_gphy);
	reset_control_put(rstc_gmac);
	clk_put(clk_etn_sys);
	clk_put(clk_etn_250m);
}

static void rtd16xx_load_otp_content(struct rtl8169_private *tp)
{
	int otp;
	u16 tmp;
	u8 *buf;

	/* RC-K 0x980174F8[27:24] */
	buf = r8169soc_read_otp(tp, "rc_k");
	if (IS_ERR(buf))
		goto set_r_amp_cal;

	otp = *buf;
	tmp = (otp << 12) | (otp << 8) | (otp << 4) | otp;
	tmp ^= RTD16XX_RC_K_DEFAULT;
	int_mdio_write(tp, 0x0bcd, 22, tmp);
	int_mdio_write(tp, 0x0bcd, 23, tmp);

	kfree(buf);

set_r_amp_cal:

	buf = r8169soc_read_otp(tp, "r_amp_k");
	if (IS_ERR(buf))
		return;

	/* R-K 0x98017500[18:15] */
	otp = ((buf[5] & 0x80) >> 7) | ((buf[6] & 0x07) << 1);
	tmp = (otp << 12) | (otp << 8) | (otp << 4) | otp;
	tmp ^= RTD16XX_R_K_DEFAULT;
	int_mdio_write(tp, 0x0bce, 16, tmp);
	int_mdio_write(tp, 0x0bce, 17, tmp);

	/* Amp-K 0x980174FC[15:0] */
	otp = buf[0] | (buf[1] << 8);
	tmp = otp ^ RTD16XX_AMP_K_DEFAULT;
	tmp += tp->amp_k_offset;
	int_mdio_write(tp, 0x0bca, 22, tmp);

	/* Bias-K 0x980174FC[31:16] */
	otp = buf[2] | (buf[3] << 8);
	tmp = otp ^ RTD16XX_ADC_BIAS_K_DEFAULT;
	int_mdio_write(tp, 0x0bcf, 22, tmp);

	kfree(buf);
}

static u32 rtd16xx_serdes_init(struct rtl8169_private *tp)
{
	struct device *d = tp_to_dev(tp);
	u32 stable_ticks;
	u32 tmp;
	u32 val;
	struct clk *clk_en_sds = clk_get(&tp->pdev->dev, "sds");
	struct reset_control *rstc_sds_reg =
		reset_control_get_exclusive(&tp->pdev->dev, "sds_reg");
	struct reset_control *rstc_sds =
		reset_control_get_exclusive(&tp->pdev->dev, "sds");
	struct reset_control *rstc_pcie0_power =
		reset_control_get_exclusive(&tp->pdev->dev, "pcie0_power");
	struct reset_control *rstc_pcie0_phy =
		reset_control_get_exclusive(&tp->pdev->dev, "pcie0_phy");
	struct reset_control *rstc_pcie0_sgmii_mdio =
		reset_control_get_exclusive(&tp->pdev->dev, "pcie0_sgmii_mdio");
	struct reset_control *rstc_pcie0_phy_mdio =
		reset_control_get_exclusive(&tp->pdev->dev, "pcie0_phy_mdio");

	/* reg_0x98000050[13:12] = 11 */
	/* CRT spec, clk_en_sds */
	clk_prepare_enable(clk_en_sds);

	/* reg_0x98000000[9:8] = 11 */
	/* reg_0x98000000[7:6] = 11 */
	/* CRT spec, rstn_sds_reg & rstn_sds */
	reset_control_deassert(rstc_sds);
	reset_control_deassert(rstc_sds_reg);

	/* reg_0x98000004[25:24] = 11   CRT spec, rstn_pcie0_sgmii_mdio */
	/* reg_0x98000004[23:22] = 11   CRT spec, rstn_pcie0_phy_mdio */
	/* reg_0x98000004[19:18] = 11   CRT spec, rstn_pcie0_power */
	/* reg_0x98000004[13:12] = 11   CRT spec, rstn_pcie0_phy */
	reset_control_deassert(rstc_pcie0_power);
	reset_control_deassert(rstc_pcie0_phy);
	reset_control_deassert(rstc_pcie0_sgmii_mdio);
	reset_control_deassert(rstc_pcie0_phy_mdio);

	/* reg_0x9800705c[6] = 1 */
	/* ISO spec, set PCIe channel to SGMII */
	regmap_update_bits_base(tp->iso_base, ISO_PWRCUT_ETN,
				BIT(6), BIT(6), NULL, false, true);

	/* ### Beginning of SGMII DPHY register tuning ### */
	/* reg_0x9800705c[7] = 1 */
	/* ISO spec, set internal MDIO to PCIe */
	regmap_update_bits_base(tp->iso_base, ISO_PWRCUT_ETN,
				BIT(7), BIT(7), NULL, false, true);

	/* reg_0x9800705c[20:16] = 00000 */
	/* ISO spec, set internal PHY addr to SERDES_DPHY_0 */
	__int_set_phy_addr(tp, RTD16XX_SERDES_DPHY_0);

	/* # DPHY spec, 5GHz tuning */
	int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x4, 0x52f5);
	int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x5, 0xead7);
	int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x6, 0x0010);
	int_mdio_write(tp, CURRENT_MDIO_PAGE, 0xa, 0xc653);
	int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x1, 0xe030);
	int_mdio_write(tp, CURRENT_MDIO_PAGE, 0xd, 0xee1c);

	/* reg_0x9800705c[20:16] = 00001 */
	/* ISO spec, set internal PHY addr to SERDES_DPHY_1 */
	__int_set_phy_addr(tp, RTD16XX_SERDES_DPHY_1);

	/* tx_swing_550mv by default */
	switch (tp->sgmii.swing) {
	case RTD16XX_TX_SWING_190MV:
		int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x0, 0xd411);
		int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x1, 0x2277);
		break;
	case RTD16XX_TX_SWING_250MV:
		int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x0, 0xd433);
		int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x1, 0x2244);
		break;
	case RTD16XX_TX_SWING_380MV:
		int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x0, 0xd433);
		int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x1, 0x22aa);
		break;
	case RTD16XX_TX_SWING_550MV:	/* recommended by RDC */
	default:
		int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x0, 0xd455);
		int_mdio_write(tp, CURRENT_MDIO_PAGE, 0x1, 0x2828);
	}

	/* reg_0x9800705c[20:16] = 00001 */
	/* ISO spec, set internal PHY addr to INT_PHY_ADDR */
	__int_set_phy_addr(tp, RTD16XX_INT_PHY_ADDR);

	fsleep(10000);		/* wait for clock stable */

	/* reg_0x9800705c[5] = 0 */
	/* reg_0x9800705c[7] = 0 */
	/* ISO spec, set internal MDIO to GPHY */
	regmap_update_bits_base(tp->iso_base, ISO_PWRCUT_ETN,
				(BIT(7) | BIT(5)), 0, NULL, false, true);

	tp->ext_phy = true;
	/* ext_phy == true now */

	tmp = 0;
	stable_ticks = 0;
	while (stable_ticks < 10) {
		/* # SDS spec, wait for phy ready */
		regmap_read(tp->sds_base, RTD16XX_SDS_LINK, &val);
		if ((val & BIT(24)) == 0) {
			stable_ticks = 0;
			if (tmp >= 100) {
				dev_err(d, "SGMII PHY not ready in 100ms\n");
				set_bit(RTL_STATUS_HW_FAIL, tp->status);
				break;
			}
		}
		stable_ticks++;
		tmp++;
		fsleep(1000);
	}
	if (stable_ticks > 0)
		dev_info(d, "SGMII PHY is ready in %d ms", tmp);

	/* reg_0x9800705c[4] = 1 */
	/* ISO spec, set data path to SGMII */
	regmap_update_bits_base(tp->iso_base, ISO_PWRCUT_ETN,
				BIT(4), BIT(4), NULL, false, true);

	/* reg_0x981c8008[9:8] = 00 */
	/* # SDS spec, SP_SDS_FRC_AN, SERDES auto mode */
	regmap_update_bits_base(tp->sds_base, RTD16XX_SDS_REG02,
				GENMASK(9, 8), 0, NULL, false, true);

	/* # SDS spec, wait for SERDES link up */
	tmp = 0;
	stable_ticks = 0;
	while (stable_ticks < 10) {
		regmap_read(tp->sds_base, RTD16XX_SDS_MISC, &val);
		if ((val & BIT(12)) == 0) {
			stable_ticks = 0;
			if (tmp >= 100) {
				dev_err(d, "SGMII link down in 100ms\n");
				set_bit(RTL_STATUS_HW_FAIL, tp->status);
				break;
			}
		}
		stable_ticks++;
		tmp++;
		fsleep(1000);
	}
	if (stable_ticks > 0)
		dev_info(d, "SGMII link up in %d ms", tmp);

	reset_control_put(rstc_sds_reg);
	reset_control_put(rstc_sds);
	reset_control_put(rstc_pcie0_power);
	reset_control_put(rstc_pcie0_phy);
	reset_control_put(rstc_pcie0_sgmii_mdio);
	reset_control_put(rstc_pcie0_phy_mdio);
	clk_put(clk_en_sds);
	return 0;
}

static void rtd16xx_phy_iol_tuning(struct rtl8169_private *tp)
{
	/* for common mode voltage */
	int_mdio_write(tp, 0x0bc0, 17,
		       (int_mdio_read(tp, 0x0bc0, 17) & ~(0xff << 4)) | (0xb4 << 4));

	/* for 1000 Base-T, Transmitter Distortion */
	int_mdio_write(tp, 0x0a43, 27, 0x8082);
	int_mdio_write(tp, 0x0a43, 28,
		       (int_mdio_read(tp, 0x0a43, 28) & ~(0xff << 8)) | (0xae << 8));

	/* for 1000 Base-T, Master Filtered Jitter */
	int_mdio_write(tp, 0x0a43, 27, 0x807c);
	/* adjust ldvbias_busy, abiq_busy, gdac_ib_up_tm
	 * to accelerate slew rate
	 */
	int_mdio_write(tp, 0x0a43, 28, 0xf001);
	/* set CP_27to25=7 and REF_27to25_L=4 to decrease jitter */
	int_mdio_write(tp, 0x0bc5, 16, 0xc67f);
}

static void rtd16xx_phy_sram_table(struct rtl8169_private *tp)
{
	/* enable echo power*2 */
	int_mdio_write(tp, 0x0a42, 22, 0x0f10);

	/* Channel estimation, 100Mbps adjustment */
	int_mdio_write(tp, 0x0a43, 27, 0x8087);	/* gain_i slope */
	int_mdio_write(tp, 0x0a43, 28, 0x42f0);	/* 0x43 => 0x42 */
	int_mdio_write(tp, 0x0a43, 27, 0x808e);	/* clen_i_c initial value */
	int_mdio_write(tp, 0x0a43, 28, 0x14a4);	/* 0x13 => 0x14 */
	/* adc peak adjustment */
	int_mdio_write(tp, 0x0a43, 27, 0x8088);	/* aagc_lvl_c initial value 0x3f0 => ok */
	int_mdio_write(tp, 0x0a43, 28, 0xf0eb);	/* delta_a slope 0x1e => 0x1d */
	/* cb0 adjustment */
	int_mdio_write(tp, 0x0a43, 27, 0x808c);	/* cb0_i_c initial value */
	int_mdio_write(tp, 0x0a43, 28, 0xef09);	/* 0x9ef => ok */
	int_mdio_write(tp, 0x0a43, 27, 0x808f);	/* delta_b slope */
	int_mdio_write(tp, 0x0a43, 28, 0xa4c6);	/* 0xa4 => ok */
	/* DAGC adjustment */
	int_mdio_write(tp, 0x0a43, 27, 0x808a);	/* cg_i_c initial value */
	int_mdio_write(tp, 0x0a43, 28, 0x400a);	/* 0xb40 => 0xa40 */
	int_mdio_write(tp, 0x0a43, 27, 0x8092);	/* delta_g slope */
	int_mdio_write(tp, 0x0a43, 28, 0xc21e);	/* 0x0c0 => 0x0c2 */

	/* 1000Mbps master adjustment */
	/* line adjustment */
	int_mdio_write(tp, 0x0a43, 27, 0x8099);	/* gain_i slope */
	int_mdio_write(tp, 0x0a43, 28, 0x2ae0);	/* 0x2e => 0x2a */
	int_mdio_write(tp, 0x0a43, 27, 0x80a0);	/* clen_i_c initial value */
	int_mdio_write(tp, 0x0a43, 28, 0xf28f);	/* 0xfe => 0xf2 */
	/* adc peak adjustment */
	/* aagc_lvl_c initial value 0x3e0 => 0x470 */
	int_mdio_write(tp, 0x0a43, 27, 0x809a);
	int_mdio_write(tp, 0x0a43, 28, 0x7084);	/* delta_a slope 0x0d => 0x10 */
	/* cb0 adjustment */
	int_mdio_write(tp, 0x0a43, 27, 0x809e);	/* cb0_i_c initial value */
	int_mdio_write(tp, 0x0a43, 28, 0xa008);	/* 0x7a1 => 0x8a0 */
	int_mdio_write(tp, 0x0a43, 27, 0x80a1);	/* delta_b slope */
	int_mdio_write(tp, 0x0a43, 28, 0x783d);	/* 0x8f => 0x78 */
	/* DAGC adjustment */
	int_mdio_write(tp, 0x0a43, 27, 0x809c);	/* cg_i_c initial value */
	int_mdio_write(tp, 0x0a43, 28, 0x8008);	/* 0xb00 => 0x880 */
	int_mdio_write(tp, 0x0a43, 27, 0x80a4);	/* delta_g slope */
	int_mdio_write(tp, 0x0a43, 28, 0x580c);	/* 0x063 => 0x058 */

	/* 1000Mbps slave adjustment */
	/* line adjustment */
	int_mdio_write(tp, 0x0a43, 27, 0x80ab);	/* gain_i slope */
	int_mdio_write(tp, 0x0a43, 28, 0x2b4a);	/* 0x2e => 0x2b */
	int_mdio_write(tp, 0x0a43, 27, 0x80b2);	/* clen_i_c initial value */
	int_mdio_write(tp, 0x0a43, 28, 0xf47f);	/* 0xfa => 0xf4 */
	/* adc peak adjustment */
	/* aagc_lvl_c initial value 0x44a => 0x488 */
	int_mdio_write(tp, 0x0a43, 27, 0x80ac);
	int_mdio_write(tp, 0x0a43, 28, 0x8884);	/* delta_a slope 0x0e => 0x10 */
	/* cb0 adjustment */
	int_mdio_write(tp, 0x0a43, 27, 0x80b0);	/* cb0_i_c initial value */
	int_mdio_write(tp, 0x0a43, 28, 0xa107);	/* 0x6a1 => 0x7a1 */
	int_mdio_write(tp, 0x0a43, 27, 0x80b3);	/* delta_b slope */
	int_mdio_write(tp, 0x0a43, 28, 0x683d);	/* 0x7f => 0x68 */
	/* DAGC adjustment */
	int_mdio_write(tp, 0x0a43, 27, 0x80ae);	/* cg_i_c initial value */
	int_mdio_write(tp, 0x0a43, 28, 0x2006);	/* 0x760 => 0x620 */
	int_mdio_write(tp, 0x0a43, 27, 0x80b6);	/* delta_g slope */
	int_mdio_write(tp, 0x0a43, 28, 0x850c);	/* 0x090 => 0x085 */
}

static void rtd16xx_patch_gphy_uc_code(struct rtl8169_private *tp)
{
	struct device *d = tp_to_dev(tp);
	u32 tmp;

	/* patch for GPHY uC firmware,
	 * adjust 1000M EEE lpi_waketx_timer = 1.3uS
	 */
#define PATCH_KEY_ADDR  0x8028	/* for RL6525 */
#define PATCH_KEY       0x5600	/* for RL6525 */

	/* Patch request & wait for the asserting of patch_rdy */
	int_mdio_write(tp, 0x0b82, 16,
		       int_mdio_read(tp, 0x0b82, 16) | BIT(4));

	tmp = 0;
	while ((int_mdio_read(tp, 0x0b80, 16) & BIT(6)) == 0) {
		tmp += 1;
		fsleep(1000);
		if (tmp >= 100) {
			dev_err(d, "GPHY patch_rdy timeout.\n");
			set_bit(RTL_STATUS_HW_FAIL, tp->status);
			break;
		}
	}
	dev_info(d, "wait %d ms for GPHY patch_rdy. reg = 0x%x\n",
		 tmp, int_mdio_read(tp, 0x0b80, 16));
	dev_info(d, "patch_rdy is asserted!!\n");

	/* Set patch_key & patch_lock */
	int_mdio_write(tp, 0, 27, PATCH_KEY_ADDR);
	int_mdio_write(tp, 0, 28, PATCH_KEY);
	int_mdio_write(tp, 0, 27, PATCH_KEY_ADDR);
	dev_info(d, "check patch key = %04x\n", int_mdio_read(tp, 0, 28));
	int_mdio_write(tp, 0, 27, 0xb82e);
	int_mdio_write(tp, 0, 28, 0x0001);

	/* uC patch code, released by Digital Designer */
	int_mdio_write(tp, 0, 27, 0xb820);
	int_mdio_write(tp, 0, 28, 0x0290);

	int_mdio_write(tp, 0, 27, 0xa012);
	int_mdio_write(tp, 0, 28, 0x0000);

	int_mdio_write(tp, 0, 27, 0xa014);
	int_mdio_write(tp, 0, 28, 0x2c04);
	int_mdio_write(tp, 0, 28, 0x2c06);
	int_mdio_write(tp, 0, 28, 0x2c09);
	int_mdio_write(tp, 0, 28, 0x2c0c);
	int_mdio_write(tp, 0, 28, 0xd093);
	int_mdio_write(tp, 0, 28, 0x2265);
	int_mdio_write(tp, 0, 28, 0x9e20);
	int_mdio_write(tp, 0, 28, 0xd703);
	int_mdio_write(tp, 0, 28, 0x2502);
	int_mdio_write(tp, 0, 28, 0x9e40);
	int_mdio_write(tp, 0, 28, 0xd700);
	int_mdio_write(tp, 0, 28, 0x0800);
	int_mdio_write(tp, 0, 28, 0x9e80);
	int_mdio_write(tp, 0, 28, 0xd70d);
	int_mdio_write(tp, 0, 28, 0x202e);

	int_mdio_write(tp, 0, 27, 0xa01a);
	int_mdio_write(tp, 0, 28, 0x0000);

	int_mdio_write(tp, 0, 27, 0xa006);
	int_mdio_write(tp, 0, 28, 0x002d);

	int_mdio_write(tp, 0, 27, 0xa004);
	int_mdio_write(tp, 0, 28, 0x0507);

	int_mdio_write(tp, 0, 27, 0xa002);
	int_mdio_write(tp, 0, 28, 0x0501);

	int_mdio_write(tp, 0, 27, 0xa000);
	int_mdio_write(tp, 0, 28, 0x1264);

	int_mdio_write(tp, 0, 27, 0xb820);
	int_mdio_write(tp, 0, 28, 0x0210);

	fsleep(10000);

	/* Clear patch_key & patch_lock */
	int_mdio_write(tp, 0, 27, PATCH_KEY_ADDR);
	int_mdio_write(tp, 0, 28, 0);
	int_mdio_write(tp, 0x0b82, 23, 0);

	/* Release patch request & wait for the deasserting of patch_rdy. */
	int_mdio_write(tp, 0x0b82, 16,
		       int_mdio_read(tp, 0x0b82, 16) & ~BIT(4));

	tmp = 0;
	while ((int_mdio_read(tp, 0x0b80, 16) & BIT(6)) != 0) {
		tmp += 1;
		fsleep(1000);
		if (tmp >= 100) {
			dev_err(d, "GPHY patch_rdy timeout.\n");
			set_bit(RTL_STATUS_HW_FAIL, tp->status);
			break;
		}
	}
	dev_info(d, "wait %d ms for GPHY patch_rdy. reg = 0x%x\n",
		 tmp, int_mdio_read(tp, 0x0b80, 16));

	dev_info(d, "patch_rdy is de-asserted!!\n");
	dev_dbg(d, "GPHY uC code patched.\n");
}

static void rtd16xx_mdio_init(struct rtl8169_private *tp)
{
	struct device *d = tp_to_dev(tp);
	u32 tmp;
	u32 val;
	struct pinctrl_state *ps_sgmii_mdio;

	/* ISO spec, ETN_PHY_INTR,
	 * wait interrupt from PHY and it means MDIO is ready
	 */
	tmp = 0;
	regmap_read(tp->iso_base, ISO_UMSK_ISR, &val);
	while ((val & BIT(27)) == 0) {
		tmp += 1;
		fsleep(1000);
		if (tmp >= 100) {
			dev_err(d, "PHY_Status timeout.\n");
			set_bit(RTL_STATUS_HW_FAIL, tp->status);
			break;
		}
		regmap_read(tp->iso_base, ISO_UMSK_ISR, &val);
	}
	dev_info(d, "wait %d ms for PHY interrupt. UMSK_ISR = 0x%x\n", tmp, val);

	/* In ByPass mode,
	 * SW need to handle the EPHY Status check ,
	 * OTP data update and EPHY fuse_rdy setting.
	 */
	/* PHY will stay in state 1 mode */
	tmp = 0;
	while ((int_mdio_read(tp, 0x0a42, 16) & 0x07) != 0x1) {
		tmp += 1;
		fsleep(1000);
		if (tmp >= 2000) {
			dev_err(d, "PHY status is not 0x1 in bypass mode, current = 0x%02x\n",
				int_mdio_read(tp, 0x0a42, 16) & 0x07);
			set_bit(RTL_STATUS_HW_FAIL, tp->status);
			break;
		}
	}

	/* fill fuse_rdy & rg_ext_ini_done */
	int_mdio_write(tp, 0x0a46, 20,
		       int_mdio_read(tp, 0x0a46, 20) | (BIT(1) | BIT(0)));

	/* init_autoload_done = 1 */
	tmp = rtl_ocp_read(tp, 0xe004);
	tmp |= BIT(7);
	rtl_ocp_write(tp, 0xe004, tmp);

	/* ee_mode = 3 */
	rtl_unlock_config_regs(tp);

	/* wait LAN-ON */
	tmp = 0;
	do {
		tmp += 1;
		fsleep(1000);
		if (tmp >= 2000) {
			dev_err(d, "PHY status is not 0x3, current = 0x%02x\n",
				int_mdio_read(tp, 0x0a42, 16) & 0x07);
			set_bit(RTL_STATUS_HW_FAIL, tp->status);
			break;
		}
	} while ((int_mdio_read(tp, 0x0a42, 16) & 0x07) != 0x3);
	dev_info(d, "wait %d ms for PHY ready, current = 0x%x\n",
		 tmp, int_mdio_read(tp, 0x0a42, 16));

	/* adjust PHY SRAM table */
	rtd16xx_phy_sram_table(tp);

	/* GPHY patch code */
	rtd16xx_patch_gphy_uc_code(tp);

	/* adjust PHY electrical characteristics */
	rtd16xx_phy_iol_tuning(tp);

	/* load OTP contents (RC-K, R-K, Amp-K, and Bias-K)
	 * RC-K:        0x980174F8[27:24]
	 * R-K:         0x98017500[18:15]
	 * Amp-K:       0x980174FC[15:0]
	 * Bias-K:      0x980174FC[31:16]
	 */
	rtd16xx_load_otp_content(tp);

	if (tp->output_mode == OUTPUT_EMBEDDED_PHY) {
		/* Init PHY path */
		/* reg_0x9800705c[5] = 0 */
		/* reg_0x9800705c[7] = 0 */
		/* ISO spec, set internal MDIO to access PHY */
		regmap_update_bits_base(tp->iso_base, ISO_PWRCUT_ETN,
					(BIT(7) | BIT(5)), 0, NULL, false, true);

		/* reg_0x9800705c[4] = 0 */
		/* ISO spec, set data path to access PHY */
		regmap_update_bits_base(tp->iso_base, ISO_PWRCUT_ETN,
					BIT(4), 0, NULL, false, true);

		/* ETN spec, GMAC data path select MII-like(embedded GPHY),
		 * not SGMII(external PHY)
		 */
		tmp = rtl_ocp_read(tp, 0xea34) & ~(BIT(0) | BIT(1));
		tmp |= BIT(1);	/* MII */
		rtl_ocp_write(tp, 0xea34, tmp);
	} else {
		/* SGMII */
		/* ETN spec, adjust MDC freq=2.5MHz */
		tmp = rtl_ocp_read(tp, 0xDE30) & ~(BIT(7) | BIT(6));
		rtl_ocp_write(tp, 0xDE30, tmp);
		/* ETN spec, set external PHY addr */
		tmp = rtl_ocp_read(tp, 0xDE24) & ~(0x1f << 0);
		rtl_ocp_write(tp, 0xDE24, tmp | (tp->ext_phy_id & 0x1f));
		/* ISO mux spec, GPIO29 is set to MDC pin */
		/* ISO mux spec, GPIO46 is set to MDIO pin */
		ps_sgmii_mdio = pinctrl_lookup_state(tp->pc, "sgmii");
		pinctrl_select_state(tp->pc, ps_sgmii_mdio);

		/* check if external PHY is available */
		dev_info(d, "Searching external PHY...");
		tp->ext_phy = true;
		tmp = 0;
		while (ext_mdio_read(tp, 0x0a43, 31) != 0x0a43) {
			tmp += 1;
			fsleep(1000);
			if (tmp >= 2000) {
				dev_err(d, "\n External SGMII PHY not found, current = 0x%02x\n",
					ext_mdio_read(tp, 0x0a43, 31));
				set_bit(RTL_STATUS_HW_FAIL, tp->status);
				return;
			}
		}
		if (tmp < 2000)
			dev_info(d, "found.\n");

		/* lower SGMII TX swing of RTL8211FS to reduce EMI */
		/* TX swing = 470mV, default value */
		ext_mdio_write(tp, 0x0dcd, 16, 0x104e);

		tp->ext_phy = false;

		/* ETN spec, GMAC data path select SGMII(external PHY),
		 * not MII-like(embedded GPHY)
		 */
		tmp = rtl_ocp_read(tp, 0xea34) & ~(BIT(0) | BIT(1));
		tmp |= BIT(1) | BIT(0);	/* SGMII */
		rtl_ocp_write(tp, 0xea34, tmp);

		if (rtd16xx_serdes_init(tp) != 0)
			dev_err(d, "SERDES init failed\n");
		/* ext_phy == true now */

		/* SDS spec, auto update SGMII link capability */
		regmap_update_bits_base(tp->sds_base, RTD16XX_SDS_DEBUG,
					BIT(2), BIT(2), NULL, false, true);

		/* enable 8b/10b symbol error
		 * even it is only valid in 1000Mbps
		 */
		ext_mdio_write(tp, 0x0dcf, 16,
			       (ext_mdio_read(tp, 0x0dcf, 16) &
				 ~(BIT(2) | BIT(1) | BIT(0))));
		ext_mdio_read(tp, 0x0dcf, 17);	/* dummy read */
	}
}

static void rtd16xx_eee_set(struct rtl8169_private *tp, bool enable)
{
	if (enable) {
		tp->chip->mdio_lock(tp);
		if (tp->output_mode == OUTPUT_EMBEDDED_PHY) {
			/* 1000M/100M EEE capability */
			rtl_mmd_write(tp, 0x7, 60, (BIT(2) | BIT(1)));
			/* 10M EEE */
			rtl_phy_write(tp, 0x0a43, 25,
				      rtl_phy_read(tp, 0x0a43, 25) | (BIT(4) | BIT(2)));
			/* disable dynamic RX power in PHY */
			rtl_phy_write(tp, 0x0bd0, 21,
				      (rtl_phy_read(tp, 0x0bd0, 21) & ~BIT(8)) | BIT(9));
		} else {	/* SGMII */
			/* 1000M & 100M EEE capability */
			rtl_mmd_write(tp, 0x7, 60, (BIT(2) | BIT(1)));
			/* 10M EEE */
			rtl_phy_write(tp, 0x0a43, 25,
				      rtl_phy_read(tp, 0x0a43, 25) | (BIT(4) | BIT(2)));
		}
		/* EEE MAC mode */
		rtl_ocp_write(tp, 0xe040,
			      rtl_ocp_read(tp, 0xe040) | (BIT(1) | BIT(0)));
		/* EEE+ MAC mode */
		rtl_ocp_write(tp, 0xe080, rtl_ocp_read(tp, 0xe080) | BIT(1));
	} else {
		if (tp->output_mode == OUTPUT_EMBEDDED_PHY) {
			/* no EEE capability */
			rtl_mmd_write(tp, 0x7, 60, 0);
			/* 10M EEE */
			rtl_phy_write(tp, 0x0a43, 25,
				      rtl_phy_read(tp, 0x0a43, 25) & ~(BIT(4) | BIT(2)));
		} else {	/* SGMII */
			/* no EEE capability */
			rtl_mmd_write(tp, 0x7, 60, 0);
			/* 10M EEE */
			rtl_phy_write(tp, 0x0a43, 25,
				      rtl_phy_read(tp, 0x0a43, 25) & ~(BIT(4) | BIT(2)));
		}
		/* reset to default value */
		rtl_ocp_write(tp, 0xe040, rtl_ocp_read(tp, 0xe040) | BIT(13));
		/* EEE MAC mode */
		rtl_ocp_write(tp, 0xe040,
			      rtl_ocp_read(tp, 0xe040) & ~(BIT(1) | BIT(0)));
		/* EEE+ MAC mode */
		rtl_ocp_write(tp, 0xe080, rtl_ocp_read(tp, 0xe080) & ~BIT(1));
	}
}

static void rtd16xx_led_set(struct rtl8169_private *tp, bool enable)
{
	if (enable) {
		regmap_update_bits_base(tp->pinctrl_base, RTD16XX_ISO_TESTMUX_MUXPAD0,
					GENMASK(29, 26), (5 << 26), NULL, false, true);
		regmap_update_bits_base(tp->pinctrl_base, RTD16XX_ISO_TESTMUX_MUXPAD2,
					GENMASK(29, 27), (3 << 27), NULL, false, true);
	} else {
		regmap_update_bits_base(tp->pinctrl_base, RTD16XX_ISO_TESTMUX_MUXPAD0,
					GENMASK(29, 26), 0, NULL, false, true);
		regmap_update_bits_base(tp->pinctrl_base, RTD16XX_ISO_TESTMUX_MUXPAD2,
					GENMASK(29, 27), 0, NULL, false, true);
	}
}

static void rtd16xx_dump_regs(struct seq_file *m, struct rtl8169_private *tp)
{
	u32 val;

	regmap_read(tp->iso_base, ISO_UMSK_ISR, &val);
	seq_printf(m, "ISO_UMSK_ISR\t[0x98007004] = %08x\n", val);
	regmap_read(tp->iso_base, ISO_PWRCUT_ETN, &val);
	seq_printf(m, "ISO_PWRCUT_ETN\t[0x9800705c] = %08x\n", val);
	regmap_read(tp->iso_base, ISO_ETN_TESTIO, &val);
	seq_printf(m, "ISO_ETN_TESTIO\t[0x98007060] = %08x\n", val);
	regmap_read(tp->iso_base, ISO_SOFT_RESET, &val);
	seq_printf(m, "ETN_RESET_CTRL\t[0x98007088] = %08x\n", val);
	regmap_read(tp->iso_base, ISO_CLOCK_ENABLE, &val);
	seq_printf(m, "ETN_CLK_CTRL\t[0x9800708c] = %08x\n", val);
	if (tp->output_mode != OUTPUT_EMBEDDED_PHY) {
		regmap_read(tp->sds_base, RTD16XX_SDS_REG02, &val);
		seq_printf(m, "SDS_REG02\t[0x981c8008] = %08x\n", val);
		regmap_read(tp->sds_base, RTD16XX_SDS_MISC, &val);
		seq_printf(m, "SDS_MISC\t\t[0x981c9804] = %08x\n", val);
		regmap_read(tp->sds_base, RTD16XX_SDS_LINK, &val);
		seq_printf(m, "SDS_LINK\t\t[0x981c980c] = %08x\n", val);
		regmap_read(tp->sds_base, RTD16XX_SDS_DEBUG, &val);
		seq_printf(m, "SDS_DEBUG\t\t[0x981c9810] = %08x\n", val);
	}
}

static void rtd16xx_dump_var(struct seq_file *m, struct rtl8169_private *tp)
{
	seq_printf(m, "sgmii_swing\t%d\n", tp->sgmii.swing);
}

/******************* END of RTD16XX ****************************/

/* RTD13XX */
static void rtd13xx_reset_phy_gmac(struct rtl8169_private *tp)
{
	struct clk *clk_etn_sys  = clk_get(&tp->pdev->dev, "etn_sys");
	struct clk *clk_etn_250m = clk_get(&tp->pdev->dev, "etn_250m");
	struct reset_control *rstc_gphy =
		reset_control_get_exclusive(&tp->pdev->dev, "gphy");
	struct reset_control *rstc_gmac =
		reset_control_get_exclusive(&tp->pdev->dev, "gmac");

	/* pre-init clk bits */
	if (!test_bit(RTL_STATUS_REINIT, tp->status)) {
		clk_prepare_enable(clk_etn_sys);
		clk_prepare_enable(clk_etn_250m);
	}

	/* reg_0x9800708c[12:11] = 00 */
	/* ISO spec, clock enable bit for etn clock & etn 250MHz */
	clk_disable_unprepare(clk_etn_sys);
	clk_disable_unprepare(clk_etn_250m);

	/* reg_0x98007088[10:9] = 00 */
	/* ISO spec, rstn_gphy & rstn_gmac */
	reset_control_assert(rstc_gphy);
	reset_control_assert(rstc_gmac);

	/* reg_0x98007060[1] = 1 */
	/* ISO spec, bypass mode enable */
	regmap_update_bits_base(tp->iso_base, ISO_ETN_TESTIO,
				BIT(1), BIT(1), NULL, false, true);

	/* reg_0x9800705c[5] = 0 */
	/* ISO spec, default value and specify internal/external PHY ID as 1 */
	regmap_update_bits_base(tp->iso_base, ISO_PWRCUT_ETN,
				BIT(5), 0, NULL, false, true);
	/* set int PHY addr */
	__int_set_phy_addr(tp, RTD13XX_INT_PHY_ADDR);
	/* set ext PHY addr */
	__ext_set_phy_addr(tp, RTD13XX_EXT_PHY_ADDR);

	fsleep(1000);

	/* release resource */
	reset_control_put(rstc_gphy);
	reset_control_put(rstc_gmac);
	clk_put(clk_etn_sys);
	clk_put(clk_etn_250m);
}

static void rtd13xx_acp_init(struct rtl8169_private *tp)
{
	struct device *d = tp_to_dev(tp);
	u32 tmp;
	u32 val;

	/* SBX spec, Select ETN access DDR path. */
	if (tp->acp_enable) {
		/* reg_0x9801c20c[6] = 1 */
		/* SBX spec, Mask ETN_ALL to SB3 DBUS REQ */
		regmap_update_bits_base(tp->sbx_base, RTD13XX_SBX_SB3_CHANNEL_REQ_MASK,
					BIT(6), BIT(6), NULL, false, true);

		dev_info(d, "wait all SB3 access finished...");
		tmp = 0;
		regmap_read(tp->sbx_base, RTD13XX_SBX_SB3_CHANNEL_REQ_BUSY, &val);
		while (val & BIT(6)) {
			fsleep(10000);
			tmp += 10;
			if (tmp >= 100) {
				dev_err(d, "\n wait SB3 access failed (wait %d ms)\n", tmp);
				set_bit(RTL_STATUS_HW_FAIL, tp->status);
				break;
			}
			regmap_read(tp->sbx_base, RTD13XX_SBX_SB3_CHANNEL_REQ_BUSY, &val);
		}
		if (tmp < 100)
			dev_info(d, "done.\n");

		/* reg_0x9801d100[29] = 0 */
		/* SCPU wrapper spec, CLKACP division, 0 = div 2, 1 = div 3 */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD13XX_SC_WRAP_CRT_CTRL,
					BIT(29), 0, NULL, false, true);

		/* reg_0x9801d124[1:0] = 00 */
		/* SCPU wrapper spec, ACP master active, 0 = active */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD13XX_SC_WRAP_INTERFACE_EN,
					GENMASK(1, 0), 0, NULL, false, true);

		/* reg_0x9801d100[30] = 1 */
		/* SCPU wrapper spec, dy_icg_en_acp */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD13XX_SC_WRAP_CRT_CTRL,
					BIT(30), BIT(30), NULL, false, true);

		/* reg_0x9801d100[21] = 1 */
		/* SCPU wrapper spec, ACP CLK enable */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD13XX_SC_WRAP_CRT_CTRL,
					BIT(21), BIT(21), NULL, false, true);

		/* reg_0x9801d100[14] = 1 */
		/* Do not apply reset to ACP port axi3 master */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD13XX_SC_WRAP_CRT_CTRL,
					BIT(14), BIT(14), NULL, false, true);

		/* reg_0x9801d800[3:0] = 0111 */
		/* reg_0x9801d800[7:4] = 0111 */
		/* reg_0x9801d800[9] = 1 */
		/* reg_0x9801d800[20:16] = 01100 */
		/* reg_0x9801d800[28:24] = 01110 */
		/* Configure control of ACP port */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD13XX_SC_WRAP_ACP_CTRL,
					GENMASK(28, 24) | GENMASK(20, 16) |
					BIT(9) | GENMASK(7, 4) | GENMASK(3, 0),
					(0x0e << 24) | (0x0c << 16) | BIT(9) |
					(0x7 << 4) | (0x7 << 0),
					NULL, false, true);

		/* reg_0x9801d030[28] = 1 */
		/* SCPU wrapper spec, dy_icg_en_acp */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD13XX_SC_WRAP_ACP_CRT_CTRL,
					BIT(28), BIT(28), NULL, false, true);

		/* reg_0x9801d030[16] = 1 */
		/* ACP CLK Enable for acp of scpu_chip_top */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD13XX_SC_WRAP_ACP_CRT_CTRL,
					BIT(16), BIT(16), NULL, false, true);

		/* reg_0x9801d030[0] = 1 */
		/* Do not apply reset to ACP port axi3 master */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD13XX_SC_WRAP_ACP_CRT_CTRL,
					BIT(0), BIT(0), NULL, false, true);

		/* reg_0x9801c814[17] = 1 */
		/* through ACP to SCPU_ACP */
		regmap_update_bits_base(tp->sbx_base, RTD13XX_SBX_ACP_MISC_CTRL,
					BIT(17), BIT(17), NULL, false, true);

		/* SBX spec, Remove mask ETN_ALL to ACP DBUS REQ */
		regmap_update_bits_base(tp->sbx_base, RTD13XX_SBX_ACP_CHANNEL_REQ_MASK,
					BIT(1), 0, NULL, false, true);

		dev_dbg(d, "ARM ACP on\n.");
	} else {
		/* SBX spec, Mask ETN_ALL to ACP DBUS REQ */
		regmap_update_bits_base(tp->sbx_base, RTD13XX_SBX_ACP_CHANNEL_REQ_MASK,
					BIT(1), BIT(1), NULL, false, true);

		dev_info(d, "wait all ACP access finished...");
		tmp = 0;
		regmap_read(tp->sbx_base, RTD13XX_SBX_ACP_CHANNEL_REQ_BUSY, &val);
		while (val & BIT(1)) {
			fsleep(1000);
			tmp += 1;
			if (tmp >= 100) {
				dev_err(d, "\n ACP channel is still busy (wait %d ms)\n", tmp);
				set_bit(RTL_STATUS_HW_FAIL, tp->status);
				break;
			}
			regmap_read(tp->sbx_base, RTD13XX_SBX_ACP_CHANNEL_REQ_BUSY, &val);
		}
		if (tmp < 100)
			dev_info(d, "done.\n");

		/* SCPU wrapper spec, Inactive MP4 AINACTS signal */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD13XX_SC_WRAP_INTERFACE_EN,
					GENMASK(1, 0), GENMASK(1, 0), NULL, false, true);

		/* SCPU wrapper spec, nACPRESET_DVFS & CLKENACP_DVFS */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD13XX_SC_WRAP_CRT_CTRL,
					(BIT(21) | BIT(14)), 0, NULL, false, true);

		/* SCPU wrapper spec, nACPRESET & CLKENACP */
		regmap_update_bits_base(tp->scpu_wrap_base, RTD13XX_SC_WRAP_ACP_CRT_CTRL,
					(BIT(16) | BIT(0)), 0, NULL, false, true);

		/* reg_0x9801c814[17] = 0 */
		/* SBX spec, Switch ETN_ALL to DC_SYS path */
		regmap_update_bits_base(tp->sbx_base, RTD13XX_SBX_ACP_MISC_CTRL,
					BIT(17), 0, NULL, false, true);

		/* SBX spec, Remove mask ETN_ALL to SB3 DBUS REQ */
		regmap_update_bits_base(tp->sbx_base, RTD13XX_SBX_SB3_CHANNEL_REQ_MASK,
					BIT(6), 0, NULL, false, true);

		dev_dbg(d, "ARM ACP off\n.");
	}
}

static void rtd13xx_pll_clock_init(struct rtl8169_private *tp)
{
	struct clk *clk_etn_sys  = clk_get(&tp->pdev->dev, "etn_sys");
	struct clk *clk_etn_250m = clk_get(&tp->pdev->dev, "etn_250m");
	struct reset_control *rstc_gphy =
		reset_control_get_exclusive(&tp->pdev->dev, "gphy");
	struct reset_control *rstc_gmac =
		reset_control_get_exclusive(&tp->pdev->dev, "gmac");

	/* reg_0x98007004[27] = 1 */
	/* ISO spec, ETN_PHY_INTR, clear ETN interrupt for ByPassMode */
	regmap_write(tp->iso_base, ISO_UMSK_ISR, BIT(27));

	/* reg_0x98007088[10] = 1 */
	/* ISO spec, reset bit of fephy */
	reset_control_deassert(rstc_gphy);

	fsleep(1000);	/* wait 1ms for PHY PLL stable */

	/* reg_0x98007060[1] = 1 */
	/* ISO spec, bypass mode enable */
	regmap_update_bits_base(tp->iso_base, ISO_ETN_TESTIO,
				BIT(1), BIT(1), NULL, false, true);

	/* reg_0x98007088[9] = 1 */
	/* ISO spec, reset bit of gmac */
	reset_control_deassert(rstc_gmac);

	/* reg_0x9800708c[12:11] = 11 */
	/* ISO spec, clock enable bit for etn clock & etn 250MHz */
	clk_prepare_enable(clk_etn_sys);
	clk_prepare_enable(clk_etn_250m);

	fsleep(10000);	/* wait 10ms for GMAC uC to be stable */

	/* release resource */
	reset_control_put(rstc_gphy);
	reset_control_put(rstc_gmac);
	clk_put(clk_etn_sys);
	clk_put(clk_etn_250m);
}

static void rtd13xx_load_otp_content(struct rtl8169_private *tp)
{
	int otp;
	u16 tmp;
	u8 *buf;

	buf = r8169soc_read_otp(tp, "para");
	if (IS_ERR(buf))
		goto set_idac;

	/* enable 0x980174FC[4] */
	/* R-K 0x980174FC[3:0] */
	otp = *buf;
	if (otp & BIT(4)) {
		tmp = otp ^ RTD13XX_R_K_DEFAULT;
		int_mdio_write(tp, 0x0bc0, 20,
			       (int_mdio_read(tp, 0x0bc0, 20) & ~(0x1f << 0)) | tmp);
	}

	kfree(buf);

set_idac:

	buf = r8169soc_read_otp(tp, "idac");
	if (IS_ERR(buf))
		return;

	/* Amp-K 0x98017500[7:0] */
	otp = *buf;
	tmp = otp ^ RTD13XX_IDAC_FINE_DEFAULT;
	tmp += tp->amp_k_offset;
	int_mdio_write(tp, 0x0bc0, 23,
		       (int_mdio_read(tp, 0x0bc0, 23) & ~(0xff << 0)) | tmp);

	kfree(buf);
}

static void rtd13xx_phy_iol_tuning(struct rtl8169_private *tp)
{
	u16 tmp;

	/* rs_offset=rsw_offset=0xc */
	tmp = 0xcc << 8;
	int_mdio_write(tp, 0x0bc0, 20,
		       (int_mdio_read(tp, 0x0bc0, 20) & ~(0xff << 8)) | tmp);

	/* 100M Tr/Tf */
	/* reg_cf_l=0x2 */
	tmp = 0x2 << 11;
	int_mdio_write(tp, 0x0bd0, 17,
		       (int_mdio_read(tp, 0x0bd0, 17) & ~(0x7 << 11)) | tmp);

	/* 100M Swing */
	/* idac_fine_mdix, idac_fine_mdi */
	tmp = 0x55 << 0;
	int_mdio_write(tp, 0x0bc0, 23,
		       (int_mdio_read(tp, 0x0bc0, 23) & ~(0xff << 0)) | tmp);
}

static void rtd13xx_mdio_init(struct rtl8169_private *tp)
{
	struct device *d = tp_to_dev(tp);
	u32 tmp;
	u32 val;
	struct pinctrl_state *ps_rgmii_mdio;
	const struct soc_device_attribute rtk_soc_rtd13xx_a00[] = {
		{
			.family = "Realtek Hank",
			.revision = "A00",
		},
		{
		/* empty */
		}
	};

	/* ETN_PHY_INTR, wait interrupt from PHY and it means MDIO is ready */
	tmp = 0;
	regmap_read(tp->iso_base, ISO_UMSK_ISR, &val);
	while ((val & BIT(27)) == 0) {
		tmp += 1;
		fsleep(1000);
		if (tmp >= 100) {
			dev_err(d, "PHY_Status timeout.\n");
			set_bit(RTL_STATUS_HW_FAIL, tp->status);
			break;
		}
		regmap_read(tp->iso_base, ISO_UMSK_ISR, &val);
	}
	dev_info(d, "wait %d ms for PHY interrupt. UMSK_ISR = 0x%x\n", tmp, val);

	/* In ByPass mode,
	 * SW need to handle the EPHY Status check ,
	 * OTP data update and EPHY fuse_rdy setting.
	 */
	/* PHY will stay in state 1 mode */
	tmp = 0;
	while (0x1 != (int_mdio_read(tp, 0x0a42, 16) & 0x07)) {
		tmp += 1;
		fsleep(1000);
		if (tmp >= 2000) {
			dev_err(d, "PHY status is not 0x1 in bypass mode, current = 0x%02x\n",
				(int_mdio_read(tp, 0x0a42, 16) & 0x07));
			set_bit(RTL_STATUS_HW_FAIL, tp->status);
			break;
		}
	}

	/* fix A00 known issue */
	if (soc_device_match(rtk_soc_rtd13xx_a00)) {
		/* fix AFE RX power as 1 */
		int_mdio_write(tp, 0x0bd0, 21, 0x7201);
	}

	/* ETN spec. set MDC freq = 8.9MHZ */
	tmp = rtl_ocp_read(tp, 0xde10) & ~(BIT(7) | BIT(6));
	tmp |= BIT(6); /* MDC freq = 8.9MHz */
	rtl_ocp_write(tp, 0xde10, tmp);
	tmp = rtl_ocp_read(tp, 0xde30) & ~(BIT(7) | BIT(6));
	tmp |= BIT(6); /* MDC freq = 8.9MHz */
	rtl_ocp_write(tp, 0xde30, tmp);

	/* fill fuse_rdy & rg_ext_ini_done */
	int_mdio_write(tp, 0x0a46, 20,
		       int_mdio_read(tp, 0x0a46, 20) | (BIT(1) | BIT(0)));

	/* init_autoload_done = 1 */
	tmp = rtl_ocp_read(tp, 0xe004);
	tmp |= BIT(7);
	rtl_ocp_write(tp, 0xe004, tmp);

	/* ee_mode = 3 */
	rtl_unlock_config_regs(tp);

	/* wait LAN-ON */
	tmp = 0;
	do {
		tmp += 1;
		fsleep(1000);
		if (tmp >= 2000) {
			dev_err(d, "PHY status is not 0x3, current = 0x%02x\n",
				(int_mdio_read(tp, 0x0a42, 16) & 0x07));
			set_bit(RTL_STATUS_HW_FAIL, tp->status);
			break;
		}
	} while (0x3 != (int_mdio_read(tp, 0x0a42, 16) & 0x07));
	dev_info(d, "wait %d ms for PHY ready, current = 0x%x\n",
		 tmp, int_mdio_read(tp, 0x0a42, 16));

	/* adjust PHY electrical characteristics */
	rtd13xx_phy_iol_tuning(tp);

	/* load OTP contents (R-K, Amp)
	 * R-K:		0x980174FC[4:0]
	 * Amp:		0x98017500[7:0]
	 */
	rtd13xx_load_otp_content(tp);

	switch (tp->output_mode) {
	case OUTPUT_EMBEDDED_PHY:
		/* ISO spec, set data path to FEPHY */
		tmp = rtl_ocp_read(tp, 0xea30);
		tmp &= ~(BIT(0));
		rtl_ocp_write(tp, 0xea30, tmp);

		tmp = rtl_ocp_read(tp, 0xea34);
		tmp &= ~(BIT(1) | BIT(0));
		tmp |= BIT(1);  /* FEPHY */
		rtl_ocp_write(tp, 0xea34, tmp);

		/* LED high active circuit */
		/* output current: 4mA */
		regmap_update_bits_base(tp->pinctrl_base, RTD13XX_ISO_TESTMUX_PFUNC9,
					GENMASK(23, 16), 0, NULL, false, true);

		/* disable GMII for embedded FEPHY */
		tp->chip->features &= ~RTL_FEATURE_GMII;
		break;
	case OUTPUT_RGMII_TO_MAC:
		dev_err(d, "%s:%d: PHY_TYPE_RGMII_MAC is not supported yet\n", __func__, __LINE__);
		set_bit(RTL_STATUS_HW_FAIL, tp->status);
		break;
	case OUTPUT_RMII:
		dev_err(d, "%s:%d: PHY_TYPE_RMII is not supported yet\n", __func__, __LINE__);
		set_bit(RTL_STATUS_HW_FAIL, tp->status);
		break;
	case OUTPUT_RGMII_TO_PHY:
		/* ISO mux spec, GPIO15 is set to MDC pin */
		/* ISO mux spec, GPIO14 is set to MDIO pin */
		/* ISO mux spec, GPIO65~76 is set to TX/RX pin */
		ps_rgmii_mdio = pinctrl_lookup_state(tp->pc, "rgmii");
		pinctrl_select_state(tp->pc, ps_rgmii_mdio);

		/* no Schmitt_Trigger */
		switch (tp->rgmii.voltage) {
		case 3: /* 3.3v */
			/* MODE2=1, MODE=0, SR=1, smt=0, pudsel=0, puden=0,
			 * E2=0
			 */
			/* GPIO[70:65] */
			regmap_update_bits_base(tp->pinctrl_base, RTD13XX_ISO_TESTMUX_PFUNC20,
						GENMASK(27, 4), 0, NULL, false, true);

			/* GPIO[76:71] */
			regmap_update_bits_base(tp->pinctrl_base, RTD13XX_ISO_TESTMUX_PFUNC21,
						GENMASK(31, 0),
						(0x2 << 30) | (0x10 << 25) | (0x10 << 20) |
						 (0x10 << 15) | (0x10 << 10) | (0x10 << 5) |
						 (0x10 << 0),
						NULL, false, true);

			/* DP=000, DN=111 */
			regmap_update_bits_base(tp->pinctrl_base, RTD13XX_ISO_TESTMUX_PFUNC25,
						GENMASK(5, 0),
						(0x0 << 3) | (0x7 << 0),
						NULL, false, true);
			break;
		case 2: /* 2.5v */
			/* MODE2=0, MODE=1, SR=0, smt=0, pudsel=0, puden=0,
			 * E2=1
			 */
			/* GPIO[70:65] */
			regmap_update_bits_base(tp->pinctrl_base, RTD13XX_ISO_TESTMUX_PFUNC20,
						GENMASK(27, 4),
						(0x1 << 24) | (0x1 << 20) | (0x1 << 16) |
						 (0x1 << 12) | (0x1 << 8) | (0x1 << 4),
						NULL, false, true);

			/* GPIO[76:71] */
			regmap_update_bits_base(tp->pinctrl_base, RTD13XX_ISO_TESTMUX_PFUNC21,
						GENMASK(31, 0),
						(0x1 << 30) | (0x1 << 25) | (0x1 << 20) |
						 (0x1 << 15) | (0x1 << 10) | (0x1 << 5) |
						 (0x1 << 0),
						NULL, false, true);

			/* DP=000, DN=111 */
			regmap_update_bits_base(tp->pinctrl_base, RTD13XX_ISO_TESTMUX_PFUNC25,
						GENMASK(5, 0),
						(0x0 << 3) | (0x7 << 0),
						NULL, false, true);
			break;
		case 1: /* 1.8v */
		default:
			/* MODE2=0, MODE=0, SR=0, smt=0, pudsel=0, puden=0,
			 * E2=1
			 */
			/* GPIO[70:65] */
			regmap_update_bits_base(tp->pinctrl_base, RTD13XX_ISO_TESTMUX_PFUNC20,
						GENMASK(27, 4),
						(0x1 << 24) | (0x1 << 20) | (0x1 << 16) |
						 (0x1 << 12) | (0x1 << 8) | (0x1 << 4),
						NULL, false, true);

			/* GPIO[76:71] */
			regmap_update_bits_base(tp->pinctrl_base, RTD13XX_ISO_TESTMUX_PFUNC21,
						GENMASK(31, 0),
						(0x0 << 30) | (0x1 << 25) | (0x1 << 20) |
						 (0x1 << 15) | (0x1 << 10) | (0x1 << 5) |
						 (0x1 << 0),
						NULL, false, true);

			/* DP=001, DN=110 */
			regmap_update_bits_base(tp->pinctrl_base, RTD13XX_ISO_TESTMUX_PFUNC25,
						GENMASK(5, 0),
						(0x1 << 3) | (0x6 << 0),
						NULL, false, true);
		}

		/* check if external PHY is available */
		dev_info(d, "Searching external PHY...");
		tp->ext_phy = true;
		tmp = 0;
		while (ext_mdio_read(tp, 0x0a43, 31) != 0x0a43) {
			tmp += 1;
			fsleep(1000);
			if (tmp >= 2000) {
				dev_err(d, "\n External RGMII PHY not found, current = 0x%02x\n",
					ext_mdio_read(tp, 0x0a43, 31));
				set_bit(RTL_STATUS_HW_FAIL, tp->status);
				return;
			}
		}
		if (tmp < 2000)
			dev_info(d, "found.\n");

		/* set RGMII RX delay */
		tmp = rtl_ocp_read(tp, 0xea34) &
			~(BIT(14) | BIT(9) | BIT(8) | BIT(6));
		switch (tp->rgmii.rx_delay) {
		case 1:
			tmp |= (0x0 << 6) | (0x1 << 8);
			break;
		case 2:
			tmp |= (0x0 << 6) | (0x2 << 8);
			break;
		case 3:
			tmp |= (0x0 << 6) | (0x3 << 8);
			break;
		case 4:
			tmp |= (0x1 << 6) | (0x0 << 8);
			break;
		case 5:
			tmp |= (0x1 << 6) | (0x1 << 8);
			break;
		case 6:
			tmp |= (0x1 << 6) | (0x2 << 8);
			break;
		case 7:
			tmp |= (0x1 << 6) | (0x3 << 8);
		}
		rtl_ocp_write(tp, 0xea34, tmp);

		tmp = rtl_ocp_read(tp, 0xea36) & ~(BIT(0)); /* rg_clk_en */
		rtl_ocp_write(tp, 0xea36, tmp);

		/* external PHY RGMII timing tuning,
		 * rg_rgmii_id_mode = 1 (default)
		 */
		tmp = ext_mdio_read(tp, 0x0d08, 21);
		switch (tp->rgmii.rx_delay) {
		case 0:
		case 1:
		case 2:
		case 3:
			tmp |= BIT(3);
			break;
		case 4:
		case 5:
		case 6:
		case 7:
			tmp &= ~BIT(3);
		}
		ext_mdio_write(tp, 0x0d08, 21, tmp);

		/* set RGMII TX delay */
		if (tp->rgmii.tx_delay == 0) {
			tmp = rtl_ocp_read(tp, 0xea34) & ~(BIT(7));
			rtl_ocp_write(tp, 0xea34, tmp);

			/* external PHY RGMII timing tuning, tx_dly_mode = 1 */
			ext_mdio_write(tp, 0x0d08, 17,
				       ext_mdio_read(tp, 0x0d08, 17) | BIT(8));
		} else {	/* tp->tx_delay == 1 (2ns) */
			tmp = rtl_ocp_read(tp, 0xea34) | (BIT(7));
			rtl_ocp_write(tp, 0xea34, tmp);

			/* external PHY RGMII timing tuning, tx_dly_mode = 0 */
			ext_mdio_write(tp, 0x0d08, 17,
				       ext_mdio_read(tp, 0x0d08, 17) & ~BIT(8));
		}

		/* ISO spec, data path is set to RGMII */
		tmp = rtl_ocp_read(tp, 0xea30) & ~(BIT(0));
		rtl_ocp_write(tp, 0xea30, tmp);

		tmp = rtl_ocp_read(tp, 0xea34) & ~(BIT(0) | BIT(1));
		tmp |= BIT(0); /* RGMII */
		rtl_ocp_write(tp, 0xea34, tmp);

		/* ext_phy == true now */

		break;
	default:
		dev_err(d, "%s:%d: unsupported output mode (%d)\n",
			__func__, __LINE__, tp->output_mode);
		set_bit(RTL_STATUS_HW_FAIL, tp->status);
	}
}

static void rtd13xx_eee_set(struct rtl8169_private *tp, bool enable)
{
	if (enable) {
		tp->chip->mdio_lock(tp);
		if (tp->output_mode == OUTPUT_EMBEDDED_PHY) {
			/* enable eee auto fallback function */
			rtl_phy_write(tp, 0x0a4b, 17,
				      rtl_phy_read(tp, 0x0a4b, 17) | BIT(2));
			/* 1000M/100M EEE capability */
			rtl_mmd_write(tp, 0x7, 60, (BIT(2) | BIT(1)));
			/* 10M EEE */
			rtl_phy_write(tp, 0x0a43, 25,
				      rtl_phy_read(tp, 0x0a43, 25) | BIT(4) | BIT(2));
			/* keep RXC in LPI mode */
			rtl_mmd_write(tp, 0x3, 0,
				      rtl_mmd_read(tp, 0x3, 0) & ~BIT(10));
		} else { /* RGMII */
			/* enable eee auto fallback function */
			rtl_phy_write(tp, 0x0a4b, 17,
				      rtl_phy_read(tp, 0x0a4b, 17) | BIT(2));
			/* 1000M & 100M EEE capability */
			rtl_mmd_write(tp, 0x7, 60, (BIT(2) | BIT(1)));
			/* 10M EEE */
			rtl_phy_write(tp, 0x0a43, 25,
				      rtl_phy_read(tp, 0x0a43, 25) | BIT(4) | BIT(2));
			/* stop RXC in LPI mode */
			rtl_mmd_write(tp, 0x3, 0,
				      rtl_mmd_read(tp, 0x3, 0) | BIT(10));
		}
		/* EEE Tw_sys_tx timing adjustment,
		 * make sure MAC would send data after FEPHY wake up
		 */
		/* default 0x001F, 100M EEE */
		rtl_ocp_write(tp, 0xe04a, 0x002f);
		/* default 0x0011, 1000M EEE */
		rtl_ocp_write(tp, 0xe04c, 0x001a);
		/* EEEP Tw_sys_tx timing adjustment,
		 * make sure MAC would send data after FEPHY wake up
		 */
		 /* default 0x3f, 10M EEEP */
		if (tp->output_mode == OUTPUT_EMBEDDED_PHY) {
			/* FEPHY needs 160us */
			rtl_ocp_write(tp, 0xe08a, 0x00a8);
		} else {	/* RGMII, GPHY needs 25us */
			rtl_ocp_write(tp, 0xe08a, 0x0020);
		}
		/* default 0x0005, 100M/1000M EEEP */
		rtl_ocp_write(tp, 0xe08c, 0x0008);
		/* EEE MAC mode */
		rtl_ocp_write(tp, 0xe040,
			      rtl_ocp_read(tp, 0xe040) | BIT(1) | BIT(0));
		/* EEE+ MAC mode */
		rtl_ocp_write(tp, 0xe080, rtl_ocp_read(tp, 0xe080) | BIT(1));
	} else {
		tp->chip->mdio_lock(tp);
		if (tp->output_mode == OUTPUT_EMBEDDED_PHY) {
			/* disable eee auto fallback function */
			rtl_phy_write(tp, 0x0a4b, 17,
				      rtl_phy_read(tp, 0x0a4b, 17) & ~BIT(2));
			/* 100M & 1000M EEE capability */
			rtl_mmd_write(tp, 0x7, 60, 0);
			/* 10M EEE */
			rtl_phy_write(tp, 0x0a43, 25,
				      rtl_phy_read(tp, 0x0a43, 25) & ~(BIT(4) | BIT(2)));
			/* keep RXC in LPI mode */
			rtl_mmd_write(tp, 0x3, 0,
				      rtl_mmd_read(tp, 0x3, 0) & ~BIT(10));
		} else { /* RGMII */
			/* disable eee auto fallback function */
			rtl_phy_write(tp, 0x0a4b, 17,
				      rtl_phy_read(tp, 0x0a4b, 17) & ~BIT(2));
			/* 100M & 1000M EEE capability */
			rtl_mmd_write(tp, 0x7, 60, 0);
			/* 10M EEE */
			rtl_phy_write(tp, 0x0a43, 25,
				      rtl_phy_read(tp, 0x0a43, 25) & ~(BIT(4) | BIT(2)));
			/* keep RXC in LPI mode */
			rtl_mmd_write(tp, 0x3, 0,
				      rtl_mmd_read(tp, 0x3, 0) & ~BIT(10));
		}
		/* reset to default value */
		rtl_ocp_write(tp, 0xe040, rtl_ocp_read(tp, 0xe040) | BIT(13));
		/* EEE MAC mode */
		rtl_ocp_write(tp, 0xe040,
			      rtl_ocp_read(tp, 0xe040) & ~(BIT(1) | BIT(0)));
		/* EEE+ MAC mode */
		rtl_ocp_write(tp, 0xe080, rtl_ocp_read(tp, 0xe080) & ~BIT(1));
		/* EEE Tw_sys_tx timing restore */
		/* default 0x001F, 100M EEE */
		rtl_ocp_write(tp, 0xe04a, 0x001f);
		/* default 0x0011, 1000M EEE */
		rtl_ocp_write(tp, 0xe04c, 0x0011);
		/* EEEP Tw_sys_tx timing restore */
		/* default 0x3f, 10M EEEP */
		rtl_ocp_write(tp, 0xe08a, 0x003f);
		/* default 0x0005, 100M/1000M EEEP */
		rtl_ocp_write(tp, 0xe08c, 0x0005);
	}
}

static void rtd13xx_led_set(struct rtl8169_private *tp, bool enable)
{
	if (enable) {
		regmap_update_bits_base(tp->pinctrl_base, RTD13XX_ISO_TESTMUX_MUXPAD2,
					GENMASK(9, 4), (9 << 4), NULL, false, true);
	} else {
		regmap_update_bits_base(tp->pinctrl_base, RTD13XX_ISO_TESTMUX_MUXPAD2,
					GENMASK(9, 4), 0, NULL, false, true);
	}
}

static void rtd13xx_dump_regs(struct seq_file *m, struct rtl8169_private *tp)
{
	u32 val;

	regmap_read(tp->iso_base, ISO_UMSK_ISR, &val);
	seq_printf(m, "ISO_UMSK_ISR\t[0x98007004] = %08x\n", val);
	regmap_read(tp->iso_base, ISO_PWRCUT_ETN, &val);
	seq_printf(m, "ISO_PWRCUT_ETN\t[0x9800705c] = %08x\n", val);
	regmap_read(tp->iso_base, ISO_ETN_TESTIO, &val);
	seq_printf(m, "ISO_ETN_TESTIO\t[0x98007060] = %08x\n", val);
	regmap_read(tp->iso_base, ISO_SOFT_RESET, &val);
	seq_printf(m, "ETN_RESET_CTRL\t[0x98007088] = %08x\n", val);
	regmap_read(tp->iso_base, ISO_CLOCK_ENABLE, &val);
	seq_printf(m, "ETN_CLK_CTRL\t[0x9800708c] = %08x\n", val);
}

static void rtd13xx_dump_var(struct seq_file *m, struct rtl8169_private *tp)
{
	seq_printf(m, "voltage \t%d\n", tp->rgmii.voltage);
	seq_printf(m, "tx_delay\t%d\n", tp->rgmii.tx_delay);
	seq_printf(m, "rx_delay\t%d\n", tp->rgmii.rx_delay);
}

/******************* END of RTD13XX ****************************/

/* RTD16XXB */
static void rtd16xxb_reset_phy_gmac(struct rtl8169_private *tp)
{
	struct clk *clk_etn_sys  = clk_get(&tp->pdev->dev, "etn_sys");
	struct clk *clk_etn_250m = clk_get(&tp->pdev->dev, "etn_250m");
	struct reset_control *rstc_gphy =
		reset_control_get_exclusive(&tp->pdev->dev, "gphy");
	struct reset_control *rstc_gmac =
		reset_control_get_exclusive(&tp->pdev->dev, "gmac");

	/* pre-init clk bits */
	if (!test_bit(RTL_STATUS_REINIT, tp->status)) {
		clk_prepare_enable(clk_etn_sys);
		clk_prepare_enable(clk_etn_250m);
	}

	/* reg_0x9800708c[12:11] = 00 */
	/* ISO spec, clock enable bit for etn clock & etn 250MHz */
	clk_disable_unprepare(clk_etn_sys);
	clk_disable_unprepare(clk_etn_250m);

	/* reg_0x98007088[10:9] = 00 */
	/* ISO spec, rstn_gphy & rstn_gmac */
	reset_control_assert(rstc_gphy);
	reset_control_assert(rstc_gmac);

	/* reg_0x98007060[1] = 1 */
	/* ISO spec, bypass mode enable */
	regmap_update_bits_base(tp->iso_base, ISO_ETN_TESTIO,
				BIT(1), BIT(1), NULL, false, true);

	/* reg_0x9800705c[5] = 0 */
	/* ISO spec, default value and specify internal/external PHY ID as 1 */
	regmap_update_bits_base(tp->iso_base, ISO_PWRCUT_ETN,
				BIT(5), 0, NULL, false, true);
	/* set int PHY addr */
	__int_set_phy_addr(tp, RTD16XXB_INT_PHY_ADDR);
	/* set ext PHY addr */
	__ext_set_phy_addr(tp, RTD16XXB_EXT_PHY_ADDR);

	fsleep(1000);

	/* release resource */
	reset_control_put(rstc_gphy);
	reset_control_put(rstc_gmac);
	clk_put(clk_etn_sys);
	clk_put(clk_etn_250m);
}

static void rtd16xxb_pll_clock_init(struct rtl8169_private *tp)
{
	struct device *d = tp_to_dev(tp);
	struct clk *clk_etn_sys  = clk_get(&tp->pdev->dev, "etn_sys");
	struct clk *clk_etn_250m = clk_get(&tp->pdev->dev, "etn_250m");
	struct reset_control *rstc_gphy =
		reset_control_get_exclusive(&tp->pdev->dev, "gphy");
	struct reset_control *rstc_gmac =
		reset_control_get_exclusive(&tp->pdev->dev, "gmac");
	u32 tmp;
	u32 val;

	/* reg_0x98007004[27] = 1 */
	/* ISO spec, ETN_PHY_INTR, clear ETN interrupt for ByPassMode */
	regmap_write(tp->iso_base, ISO_UMSK_ISR, BIT(27));

	/* reg_0x98007088[10] = 1 */
	/* ISO spec, reset bit of fephy */
	reset_control_deassert(rstc_gphy);

	fsleep(1000);	/* wait 1ms for PHY PLL stable */

	/* reg_0x98007060[1] = 1 */
	/* ISO spec, bypass mode enable */
	regmap_update_bits_base(tp->iso_base, ISO_ETN_TESTIO,
				BIT(1), BIT(1), NULL, false, true);

	/* reg_0x98007088[9] = 1 */
	/* ISO spec, reset bit of gmac */
	reset_control_deassert(rstc_gmac);

	/* reg_0x9800708c[12:11] = 11 */
	/* ISO spec, clock enable bit for etn clock & etn 250MHz */
	clk_prepare_enable(clk_etn_sys);
	clk_prepare_enable(clk_etn_250m);

	/* Check ETN MAC init_autoload_done reg_0x98007070[0] = 1 */
	/* Bit 0 changes to 1 means ETN Rbus is valid */
	regmap_read(tp->iso_base, ISO_PLL_WDOUT, &val);
	tmp = 0;
	while (0x1 != (val & BIT(0))) {
		tmp += 1;
		fsleep(1000);
		if (tmp >= MAC_INIT_TIMEOUT) {
			dev_err(d, "GMAC init timeout\n");
			set_bit(RTL_STATUS_HW_FAIL, tp->status);
			break;
		}
		regmap_read(tp->iso_base, ISO_PLL_WDOUT, &val);
	}
	if (tmp < MAC_INIT_TIMEOUT)
		dev_info(d, "GMAC wait %d ms for init_autoload_done\n", tmp);

	/* release resource */
	reset_control_put(rstc_gphy);
	reset_control_put(rstc_gmac);
	clk_put(clk_etn_sys);
	clk_put(clk_etn_250m);
}

static void rtd16xxb_load_otp_content(struct rtl8169_private *tp)
{
	int otp;
	u16 tmp;
	u8 *buf;
	const struct soc_device_attribute *soc;
	const struct soc_device_attribute rtk_soc_rtd1315c[] = {
		{
			.soc_id = "RTD1315C",
		},
		{
		/* empty */
		}
	};

	soc = soc_device_match(rtk_soc_rtd1315c);

	/* R-K		0x9803_2504[3:0]  : 4 bits, common setting
	 * Amp-K	0x9803_2504[31:16]: total 16 bits, 4 bits/channel
	 * RC-K		0x9803_2508[15:0] : total 16 bits, 4 bits/channel
	 */

	buf = r8169soc_read_otp(tp, "rc_r_amp_cal");
	if (IS_ERR(buf))
		return;

	/* R-K 0x9803_2504[3:0] */
	otp = *buf & 0xF;
	tmp = otp ^ RTD16XXB_R_K_DEFAULT;
	/* Set tapbin_p3 & tapbin_p2 */
	int_mdio_write(tp, 0x0bcf, 18, ((tmp << 8) | tmp));
	/* Set tapbin_p1 & tapbin_p0 */
	int_mdio_write(tp, 0x0bcf, 19, ((tmp << 8) | tmp));
	/* Set tapbin_pm_p3 & tapbin_pm_p2 */
	int_mdio_write(tp, 0x0bcf, 20, ((tmp << 8) | tmp));
	/* Set tapbin_pm_p1 & tapbin_pm_p0 */
	int_mdio_write(tp, 0x0bcf, 21, ((tmp << 8) | tmp));

	/* Amp-K 0x9803_2504[31:16] */
	otp = buf[2] | (buf[3] << 8);
	tmp = otp ^ RTD16XXB_AMP_K_DEFAULT;
	/* hotfix for rtd1315c w/ amp-k pattern version 0 */
	if (soc && ((buf[6] & 0x3) == 0))
		tmp = (tmp & ~(0xFF << 0)) | (0x22 << 0);
	tmp += tp->amp_k_offset;
	int_mdio_write(tp, 0x0bca, 22, tmp);

	/* RC-K 0x9803_2508[15:0] */
	otp = buf[4] | (buf[5] << 8);
	tmp = otp ^ RTD16XXB_RC_K_DEFAULT;
	int_mdio_write(tp, 0x0bcd, 22, tmp); /* len_p[3:0] */
	int_mdio_write(tp, 0x0bcd, 23, tmp); /* rlen_p[3:0] */

	kfree(buf);
}

static void rtd16xxb_phy_iol_tuning(struct rtl8169_private *tp)
{
	struct device *d = tp_to_dev(tp);
	u32 otp;
	u8 *buf;
	u16 tmp;
	const struct soc_device_attribute *soc;
	const struct soc_device_attribute rtk_soc_rtd1315c[] = {
		{
			.soc_id = "RTD1315C",
		},
		{
		/* empty */
		}
	};

	soc = soc_device_match(rtk_soc_rtd1315c);

	/* for common mode voltage */
	tmp = 0xb4 << 4;
	int_mdio_write(tp, 0x0bc0, 17,
		       (int_mdio_read(tp, 0x0bc0, 17) & ~(0xff << 4)) | tmp);

       /* Set LD_COMP to 0x0 */
	tmp = 0 << 9;
	int_mdio_write(tp, 0x0bc0, 23,
		       (int_mdio_read(tp, 0x0bc0, 23) & ~(0x3 << 9)) | tmp);

	/* set lock main */
	tmp = 0x1 << 1;
	int_mdio_write(tp, 0x0a46, 21,
		       int_mdio_read(tp, 0x0a46, 21) | tmp);
	/* wait until locked */
	tmp = 0;
	while (0x1 != (int_mdio_read(tp, 0x0a60, 16) & (0xff << 0))) {
		tmp += 10;
		fsleep(10);
		if (tmp >= PHY_LOCK_TIMEOUT) {
			dev_err(d, "Ethernet PHY is not locked (0x1), current = 0x%02x\n",
				(int_mdio_read(tp, 0x0a60, 16) & (0xff << 0)));
			set_bit(RTL_STATUS_HW_FAIL, tp->status);
			break;
		}
	}
	if (tmp < PHY_LOCK_TIMEOUT)
		dev_info(d, "Ethernet PHY: wait %d ns to start IOL tuning\n", tmp);

	/* Change green table default LDVBIAS to 0x66 */
	tmp = 0x66 << 8;
	int_mdio_write(tp, 0x0a43, 27, 0x8049);
	int_mdio_write(tp, 0x0a43, 28,
		       (int_mdio_read(tp, 0x0a43, 28) & ~(0xff << 8)) | tmp);

	/* Change green table 10M LDVBIAS to 0x66 */
	int_mdio_write(tp, 0x0a43, 27, 0x8050);
	int_mdio_write(tp, 0x0a43, 28,
		       (int_mdio_read(tp, 0x0a43, 28) & ~(0xff << 8)) | tmp);

	/* Change green table 100M short LDVBIAS to 0x66 */
	int_mdio_write(tp, 0x0a43, 27, 0x8057);
	int_mdio_write(tp, 0x0a43, 28,
		       (int_mdio_read(tp, 0x0a43, 28) & ~(0xff << 8)) | tmp);

	/* Change green table 100M long LDVBIAS to 0x66 */
	int_mdio_write(tp, 0x0a43, 27, 0x805e);
	int_mdio_write(tp, 0x0a43, 28,
		       (int_mdio_read(tp, 0x0a43, 28) & ~(0xff << 8)) | tmp);

	/* Change green table GIGA short LDVBIAS to 0x66 */
	int_mdio_write(tp, 0x0a43, 27, 0x8065);
	int_mdio_write(tp, 0x0a43, 28,
		       (int_mdio_read(tp, 0x0a43, 28) & ~(0xff << 8)) | tmp);

	/* Change green table GIGA middle LDVBIAS to 0x66 */
	int_mdio_write(tp, 0x0a43, 27, 0x806c);
	int_mdio_write(tp, 0x0a43, 28,
		       (int_mdio_read(tp, 0x0a43, 28) & ~(0xff << 8)) | tmp);

	/* Change green table GIGA long LDVBIAS to 0x66 */
	int_mdio_write(tp, 0x0a43, 27, 0x8073);
	int_mdio_write(tp, 0x0a43, 28,
		       (int_mdio_read(tp, 0x0a43, 28) & ~(0xff << 8)) | tmp);

	/* Use OTP 0x98032508 bit[17:16] to identify amp-k pattern version */
	/* b'00: original amp-k pattern */
	/* b'01: new amp-k pattern (rg_cal_itune = 0, LDVDC = 1, LD_CMFB = 2).
	 * Change LD_CMFB to 0x2 and LDVDC to 0x5 in normal mode.
	 */
	buf = r8169soc_read_otp(tp, "rc_r_amp_cal");
	if (IS_ERR(buf))
		goto out;

	otp = buf[6];
	kfree(buf);

	/* rtd1315c or other chip w/ amp-k pattern version 1 */
	if (soc || ((otp & 0x3) == 0x1)) {
		/* Change LD_CMFB to 0x2 */
		tmp = 0x2 << 12;
		int_mdio_write(tp, 0x0bc0, 23,
			       (int_mdio_read(tp, 0x0bc0, 23) & ~(0x3 << 12)) | tmp);

		/* Change green table default LDVDC to 0x5 */
		tmp = 0x5 << 8;
		int_mdio_write(tp, 0x0a43, 27, 0x804d);
		int_mdio_write(tp, 0x0a43, 28,
			       (int_mdio_read(tp, 0x0a43, 28) & ~(0x7 << 8)) | tmp);

		/* Change green table 10M LDVDC to 0x5 */
		int_mdio_write(tp, 0x0a43, 27, 0x8054);
		int_mdio_write(tp, 0x0a43, 28,
			       (int_mdio_read(tp, 0x0a43, 28) & ~(0x7 << 8)) | tmp);

		/* Change green table 100M short LDVDC to 0x5 */
		int_mdio_write(tp, 0x0a43, 27, 0x805b);
		int_mdio_write(tp, 0x0a43, 28,
			       (int_mdio_read(tp, 0x0a43, 28) & ~(0x7 << 8)) | tmp);

		/* Change green table 100M long LDVDC to 0x5 */
		int_mdio_write(tp, 0x0a43, 27, 0x8062);
		int_mdio_write(tp, 0x0a43, 28,
			       (int_mdio_read(tp, 0x0a43, 28) & ~(0x7 << 8)) | tmp);

		/* Change green table Giga short LDVDC to 0x5 */
		int_mdio_write(tp, 0x0a43, 27, 0x8069);
		int_mdio_write(tp, 0x0a43, 28,
			       (int_mdio_read(tp, 0x0a43, 28) & ~(0x7 << 8)) | tmp);

		/* Change green table Giga middle LDVDC to 0x5 */
		int_mdio_write(tp, 0x0a43, 27, 0x8070);
		int_mdio_write(tp, 0x0a43, 28,
			       (int_mdio_read(tp, 0x0a43, 28) & ~(0x7 << 8)) | tmp);

		/* Change green table Giga long LDVDC to 0x5 */
		int_mdio_write(tp, 0x0a43, 27, 0x8077);
		int_mdio_write(tp, 0x0a43, 28,
			       (int_mdio_read(tp, 0x0a43, 28) & ~(0x7 << 8)) | tmp);
	}

out:
	/* release lock main */
	tmp = 0x0 << 1;
	int_mdio_write(tp, 0x0a46, 21,
		       (int_mdio_read(tp, 0x0a46, 21) & ~(0x1 << 1)) | tmp);
}

static void rtd16xxb_mdio_init(struct rtl8169_private *tp)
{
	struct device *d = tp_to_dev(tp);
	u32 tmp;

	fsleep(10000);	/* wait for MDIO ready */

	/* PHY will stay in state 1 mode */
	tmp = 0;
	while (0x1 != (int_mdio_read(tp, 0x0a42, 16) & 0x07)) {
		tmp += 1;
		fsleep(1000);
		if (tmp >= 2000) {
			dev_err(d, "PHY status is not 0x1 in bypass mode, current = 0x%02x\n",
				(int_mdio_read(tp, 0x0a42, 16) & 0x07));
			set_bit(RTL_STATUS_HW_FAIL, tp->status);
			break;
		}
	}

	/* ETN spec. set MDC freq = 8.9MHZ */
	tmp = rtl_ocp_read(tp, 0xde10) & ~(BIT(7) | BIT(6));
	tmp |= BIT(6); /* MDC freq = 8.9MHz */
	rtl_ocp_write(tp, 0xde10, tmp);

	/* fill fuse_rdy & rg_ext_ini_done */
	int_mdio_write(tp, 0x0a46, 20,
		       int_mdio_read(tp, 0x0a46, 20) | (BIT(1) | BIT(0)));

	/* init_autoload_done = 1 */
	tmp = rtl_ocp_read(tp, 0xe004);
	tmp |= BIT(7);
	rtl_ocp_write(tp, 0xe004, tmp);

	/* ee_mode = 3 */
	rtl_unlock_config_regs(tp);

	/* wait LAN-ON */
	tmp = 0;
	do {
		tmp += 1;
		fsleep(1000);
		if (tmp >= 2000) {
			dev_err(d, "PHY status is not 0x3, current = 0x%02x\n",
				(int_mdio_read(tp, 0x0a42, 16) & 0x07));
			set_bit(RTL_STATUS_HW_FAIL, tp->status);
			break;
		}
	} while (0x3 != (int_mdio_read(tp, 0x0a42, 16) & 0x07));
	dev_info(d, "wait %d ms for PHY ready, current = 0x%x\n",
		 tmp, int_mdio_read(tp, 0x0a42, 16));

	/* adjust PHY electrical characteristics */
	rtd16xxb_phy_iol_tuning(tp);

	/* load OTP contents (R-K, Amp)
	 */
	rtd16xxb_load_otp_content(tp);

	switch (tp->output_mode) {
	case OUTPUT_EMBEDDED_PHY:
		/* ISO spec, set data path to FEPHY */
		tmp = rtl_ocp_read(tp, 0xea30);
		tmp &= ~(BIT(0));
		rtl_ocp_write(tp, 0xea30, tmp);

		tmp = rtl_ocp_read(tp, 0xea34);
		tmp &= ~(BIT(1) | BIT(0));
		tmp |= BIT(1);  /* FEPHY */
		rtl_ocp_write(tp, 0xea34, tmp);

		/* LED low active circuit */
		/* output current: 4mA */
		regmap_update_bits_base(tp->pinctrl_base, RTD16XXB_ISO_TESTMUX_PFUNC12,
					GENMASK(14, 0),
					(0x16 << 10) | (0x16 << 5) | (0x16 << 0),
					NULL, false, true);
		break;
	default:
		dev_err(d, "%s:%d: unsupported output mode (%d)\n",
			__func__, __LINE__, tp->output_mode);
		set_bit(RTL_STATUS_HW_FAIL, tp->status);
	}
}

static void rtd16xxb_eee_set(struct rtl8169_private *tp, bool enable)
{
	if (enable) {
		/* enable eee auto fallback function */
		rtl_phy_write(tp, 0x0a4b, 17,
			      rtl_phy_read(tp, 0x0a4b, 17) | BIT(2));
		/* 1000M/100M EEE capability */
		rtl_mmd_write(tp, 0x7, 60, (BIT(2) | BIT(1)));
		/* 10M EEE */
		rtl_phy_write(tp, 0x0a43, 25,
			      rtl_phy_read(tp, 0x0a43, 25) | (BIT(4) | BIT(2)));
		/* keep RXC in LPI mode */
		rtl_mmd_write(tp, 0x3, 0,
			      rtl_mmd_read(tp, 0x3, 0) & ~BIT(10));
		/* EEE Tw_sys_tx timing adjustment,
		 * make sure MAC would send data after FEPHY wake up
		 */
		/* default 0x001F, 100M EEE */
		rtl_ocp_write(tp, 0xe04a, 0x002f);
		/* default 0x0011, 1000M EEE */
		rtl_ocp_write(tp, 0xe04c, 0x001a);
		/* EEEP Tw_sys_tx timing adjustment,
		 * make sure MAC would send data after FEPHY wake up
		 */
		 /* default 0x3f, 10M EEEP */
		/* FEPHY needs 160us */
		rtl_ocp_write(tp, 0xe08a, 0x00a8);
		/* default 0x0005, 100M/1000M EEEP */
		rtl_ocp_write(tp, 0xe08c, 0x0008);
		/* EEE MAC mode */
		rtl_ocp_write(tp, 0xe040,
			      rtl_ocp_read(tp, 0xe040) | BIT(1) | BIT(0));
		/* EEE+ MAC mode */
		rtl_ocp_write(tp, 0xe080, rtl_ocp_read(tp, 0xe080) | BIT(1));
	} else {
		/* disable eee auto fallback function */
		rtl_phy_write(tp, 0x0a4b, 17,
			      rtl_phy_read(tp, 0x0a4b, 17) & ~BIT(2));
		/* 100M & 1000M EEE capability */
		rtl_mmd_write(tp, 0x7, 60, 0);
		/* 10M EEE */
		rtl_phy_write(tp, 0x0a43, 25,
			      rtl_phy_read(tp, 0x0a43, 25) & ~(BIT(4) | BIT(2)));
		/* keep RXC in LPI mode */
		rtl_mmd_write(tp, 0x3, 0,
			      rtl_mmd_read(tp, 0x3, 0) & ~BIT(10));
		/* reset to default value */
		rtl_ocp_write(tp, 0xe040, rtl_ocp_read(tp, 0xe040) | BIT(13));
		/* EEE MAC mode */
		rtl_ocp_write(tp, 0xe040,
			      rtl_ocp_read(tp, 0xe040) & ~(BIT(1) | BIT(0)));
		/* EEE+ MAC mode */
		rtl_ocp_write(tp, 0xe080, rtl_ocp_read(tp, 0xe080) & ~BIT(1));
		/* EEE Tw_sys_tx timing restore */
		/* default 0x001F, 100M EEE */
		rtl_ocp_write(tp, 0xe04a, 0x001f);
		/* default 0x0011, 1000M EEE */
		rtl_ocp_write(tp, 0xe04c, 0x0011);
		/* EEEP Tw_sys_tx timing restore */
		/* default 0x3f, 10M EEEP */
		rtl_ocp_write(tp, 0xe08a, 0x003f);
		/* default 0x0005, 100M/1000M EEEP */
		rtl_ocp_write(tp, 0xe08c, 0x0005);
	}
}

static void rtd16xxb_led_set(struct rtl8169_private *tp, bool enable)
{
	if (enable) {
		regmap_update_bits_base(tp->pinctrl_base, RTD16XXB_ISO_TESTMUX_MUXPAD2,
					GENMASK(30, 29) | GENMASK(12, 8),
					(0x1 << 29) | (0x1 << 11) | (0x1 << 8),
					NULL, false, true);
	} else {
		regmap_update_bits_base(tp->pinctrl_base, RTD16XXB_ISO_TESTMUX_MUXPAD2,
					GENMASK(30, 29) | GENMASK(12, 8),
					0, NULL, false, true);
	}
}

static void rtd16xxb_dump_regs(struct seq_file *m, struct rtl8169_private *tp)
{
	u32 val;

	regmap_read(tp->iso_base, ISO_UMSK_ISR, &val);
	seq_printf(m, "ISO_UMSK_ISR\t[0x98007004] = %08x\n", val);
	regmap_read(tp->iso_base, ISO_PWRCUT_ETN, &val);
	seq_printf(m, "ISO_PWRCUT_ETN\t[0x9800705c] = %08x\n", val);
	regmap_read(tp->iso_base, ISO_ETN_TESTIO, &val);
	seq_printf(m, "ISO_ETN_TESTIO\t[0x98007060] = %08x\n", val);
	regmap_read(tp->iso_base, ISO_SOFT_RESET, &val);
	seq_printf(m, "ETN_RESET_CTRL\t[0x98007088] = %08x\n", val);
	regmap_read(tp->iso_base, ISO_CLOCK_ENABLE, &val);
	seq_printf(m, "ETN_CLK_CTRL\t[0x9800708c] = %08x\n", val);
}

/******************* END of RTD16XXB ****************************/

/* RTD13XXD */
static void rtd13xxd_reset_phy_gmac(struct rtl8169_private *tp)
{
	struct clk *clk_etn_sys  = clk_get(&tp->pdev->dev, "etn_sys");
	struct clk *clk_etn_250m = clk_get(&tp->pdev->dev, "etn_250m");
	struct reset_control *rstc_gphy =
		reset_control_get_exclusive(&tp->pdev->dev, "gphy");
	struct reset_control *rstc_gmac =
		reset_control_get_exclusive(&tp->pdev->dev, "gmac");

	/* pre-init clk bits */
	if (!test_bit(RTL_STATUS_REINIT, tp->status)) {
		clk_prepare_enable(clk_etn_sys);
		clk_prepare_enable(clk_etn_250m);
	}

	/* reg_0x9800708c[12:11] = 00 */
	/* ISO spec, clock enable bit for etn clock & etn 250MHz */
	clk_disable_unprepare(clk_etn_sys);
	clk_disable_unprepare(clk_etn_250m);

	/* reg_0x98007088[10:9] = 00 */
	/* ISO spec, rstn_gphy & rstn_gmac */
	reset_control_assert(rstc_gphy);
	reset_control_assert(rstc_gmac);

	/* reg_0x98007060[1] = 1 */
	/* ISO spec, bypass mode enable */
	regmap_update_bits_base(tp->iso_base, ISO_ETN_TESTIO,
				BIT(1), BIT(1), NULL, false, true);

	/* reg_0x9800705c[5] = 0 */
	/* ISO spec, default value and specify internal/external PHY ID as 1 */
	regmap_update_bits_base(tp->iso_base, ISO_PWRCUT_ETN,
				BIT(5), 0, NULL, false, true);

	/* set int PHY addr */
	__int_set_phy_addr(tp, RTD13XXD_INT_PHY_ADDR);
	/* set ext PHY addr */
	__ext_set_phy_addr(tp, RTD13XXD_EXT_PHY_ADDR);

	fsleep(1000);

	/* release resource */
	reset_control_put(rstc_gphy);
	reset_control_put(rstc_gmac);
	clk_put(clk_etn_sys);
	clk_put(clk_etn_250m);
}

static void rtd13xxd_pll_clock_init(struct rtl8169_private *tp)
{
	struct device *d = tp_to_dev(tp);
	struct clk *clk_etn_sys  = clk_get(&tp->pdev->dev, "etn_sys");
	struct clk *clk_etn_250m = clk_get(&tp->pdev->dev, "etn_250m");
	struct reset_control *rstc_gphy =
		reset_control_get_exclusive(&tp->pdev->dev, "gphy");
	struct reset_control *rstc_gmac =
		reset_control_get_exclusive(&tp->pdev->dev, "gmac");
	u32 tmp;
	u32 val;

	/* reg_0x98007004[27] = 1 */
	/* ISO spec, ETN_PHY_INTR, clear ETN interrupt for ByPassMode */
	regmap_write(tp->iso_base, ISO_UMSK_ISR, BIT(27));

	/* reg_0x98007088[10] = 1 */
	/* ISO spec, reset bit of fephy */
	reset_control_deassert(rstc_gphy);

	fsleep(1000);	/* wait 1ms for PHY PLL stable */

	/* reg_0x98007060[1] = 1 */
	/* ISO spec, bypass mode enable */
	regmap_update_bits_base(tp->iso_base, ISO_ETN_TESTIO,
				BIT(1), BIT(1), NULL, false, true);

	/* reg_0x98007088[9] = 1 */
	/* ISO spec, reset bit of gmac */
	reset_control_deassert(rstc_gmac);

	/* reg_0x9800708c[12:11] = 11 */
	/* ISO spec, clock enable bit for etn clock & etn 250MHz */
	clk_prepare_enable(clk_etn_sys);
	clk_prepare_enable(clk_etn_250m);

	/* Check ETN MAC init_autoload_done reg_0x98007070[0] = 1 */
	/* Bit 0 changes to 1 means ETN Rbus is valid */
	regmap_read(tp->iso_base, ISO_PLL_WDOUT, &val);
	tmp = 0;
	while (0x1 != (val & BIT(0))) {
		tmp += 1;
		fsleep(1000);
		if (tmp >= MAC_INIT_TIMEOUT) {
			dev_err(d, "GMAC init timeout\n");
			set_bit(RTL_STATUS_HW_FAIL, tp->status);
			break;
		}
		regmap_read(tp->iso_base, ISO_PLL_WDOUT, &val);
	}
	if (tmp < MAC_INIT_TIMEOUT)
		dev_info(d, "GMAC wait %d ms for init_autoload_done\n", tmp);

	/* release resource */
	reset_control_put(rstc_gphy);
	reset_control_put(rstc_gmac);
	clk_put(clk_etn_sys);
	clk_put(clk_etn_250m);
}

static void rtd13xxd_load_otp_content(struct rtl8169_private *tp)
{
	int otp;
	u16 tmp;
	u8 *buf;

	/* Fix R-K and Amp
	 * idac_fine_mdi:  0x98032500[4:0]
	 * idac_fine_mdix: 0x98032500[9:5]
	 * rs:             0x98032500[18:16]
	 * rsw:            0x98032500[19:21]
	 */

	buf = r8169soc_read_otp(tp, "rc_r_amp_cal");
	if (IS_ERR(buf))
		return;

	/* Set RS (for Tx) */
	otp = buf[2] & 0x7;
	tmp = otp ^ RTD13XXD_RS_DEFAULT;
	tmp = (int_mdio_read(tp, 0x0bd0, 21) & ~(0x7 << 13)) | (tmp << 13);
	int_mdio_write(tp, 0x0bd0, 21, tmp);

	/* Set RSW (for Rx) */
	otp = (buf[2] & (0x7 << 3)) >> 3;
	tmp = otp ^ RTD13XXD_RSW_DEFAULT;
	tmp = (int_mdio_read(tp, 0x0bd0, 20) & ~(0x7 << 13)) | (tmp << 13);
	int_mdio_write(tp, 0x0bd0, 20, tmp);

	/* Set idac_fine_mdi */
	otp = buf[0] & 0x1f;
	tmp = otp ^ RTD13XXD_IDAC_FINE_MDI_DEFAULT;
	tmp += tp->amp_k_offset;
	tmp = (int_mdio_read(tp, 0x0bc0, 23) & ~(0x1f << 0)) | (tmp << 0);
	int_mdio_write(tp, 0x0bc0, 23, tmp);

	/* Set idac_fine_mdix */
	otp = ((buf[0] | (buf[1] << 8)) & (0x1f << 5)) >> 5;
	tmp = otp ^ RTD13XXD_IDAC_FINE_MDIX_DEFAULT;
	tmp += tp->amp_k_offset;
	tmp = (int_mdio_read(tp, 0x0bc0, 23) & ~(0x1f << 8)) | (tmp << 8);
	int_mdio_write(tp, 0x0bc0, 23, tmp);

	kfree(buf);
}

static void rtd13xxd_phy_iol_tuning(struct rtl8169_private *tp)
{
	u16 tmp;

	tmp = 0x30 << 4;
	tmp = (int_mdio_read(tp, 0x0a81, 23) & ~(0x3F << 4)) | tmp;
	int_mdio_write(tp, 0x0a81, 23, tmp);
}

static void rtd13xxd_mdio_init(struct rtl8169_private *tp)
{
	struct device *d = tp_to_dev(tp);
	u32 tmp;

	fsleep(10000);	/* wait for MDIO ready */

	/* PHY will stay in state 1 mode */
	tmp = 0;
	while (0x1 != (int_mdio_read(tp, 0x0a42, 16) & 0x07)) {
		tmp += 1;
		fsleep(1000);
		if (tmp >= 2000) {
			dev_err(d, "PHY status is not 0x1 in bypass mode, current = 0x%02x\n",
				(int_mdio_read(tp, 0x0a42, 16) & 0x07));
			set_bit(RTL_STATUS_HW_FAIL, tp->status);
			break;
		}
	}

	/* ETN spec. set MDC freq = 8.9MHZ */
	tmp = rtl_ocp_read(tp, 0xde10) & ~(BIT(7) | BIT(6));
	tmp |= BIT(6); /* MDC freq = 8.9MHz */
	rtl_ocp_write(tp, 0xde10, tmp);

	/* fill fuse_rdy & rg_ext_ini_done */
	int_mdio_write(tp, 0x0a46, 20,
		       int_mdio_read(tp, 0x0a46, 20) | (BIT(1) | BIT(0)));

	/* init_autoload_done = 1 */
	tmp = rtl_ocp_read(tp, 0xe004);
	tmp |= BIT(7);
	rtl_ocp_write(tp, 0xe004, tmp);

	/* ee_mode = 3 */
	rtl_unlock_config_regs(tp);

	/* wait LAN-ON */
	tmp = 0;
	do {
		tmp += 1;
		fsleep(1000);
		if (tmp >= 2000) {
			dev_err(d, "PHY status is not 0x3, current = 0x%02x\n",
				(int_mdio_read(tp, 0x0a42, 16) & 0x07));
			set_bit(RTL_STATUS_HW_FAIL, tp->status);
			break;
		}
	} while (0x3 != (int_mdio_read(tp, 0x0a42, 16) & 0x07));
	dev_info(d, "wait %d ms for PHY ready, current = 0x%x\n",
		 tmp, int_mdio_read(tp, 0x0a42, 16));

	/* adjust PHY electrical characteristics */
	rtd13xxd_phy_iol_tuning(tp);

	/* load OTP contents (R-K, Amp)
	 */
	rtd13xxd_load_otp_content(tp);

	switch (tp->output_mode) {
	case OUTPUT_EMBEDDED_PHY:
		/* ISO spec, set data path to FEPHY */
		tmp = rtl_ocp_read(tp, 0xea30);
		tmp &= ~(BIT(0));
		rtl_ocp_write(tp, 0xea30, tmp);

		tmp = rtl_ocp_read(tp, 0xea34);
		tmp &= ~(BIT(1) | BIT(0));
		tmp |= BIT(1);  /* FEPHY */
		rtl_ocp_write(tp, 0xea34, tmp);

		/* LED low active circuit */
		/* output current: 4mA */
		regmap_update_bits_base(tp->pinctrl_base, RTD13XXD_ISO_TESTMUX_PFUNC20,
					GENMASK(30, 26),
					(0x16 << 26),
					NULL, false, true);
		regmap_update_bits_base(tp->pinctrl_base, RTD13XXD_ISO_TESTMUX_PFUNC21,
					GENMASK(4, 0),
					(0x16 << 0),
					NULL, false, true);

		/* disable GMII for embedded FEPHY */
		tp->chip->features &= ~RTL_FEATURE_GMII;
		break;
	default:
		dev_err(d, "%s:%d: unsupported output mode (%d)\n",
			__func__, __LINE__, tp->output_mode);
		set_bit(RTL_STATUS_HW_FAIL, tp->status);
	}
}

static void rtd13xxd_eee_set(struct rtl8169_private *tp, bool enable)
{
	if (enable) {
		/* enable eee auto fallback function */
		rtl_phy_write(tp, 0x0a4b, 17,
			      rtl_phy_read(tp, 0x0a4b, 17) | BIT(2));
		/* 100M EEE capability */
		rtl_mmd_write(tp, 0x7, 60, (BIT(2) | BIT(1)));
		/* 10M EEE */
		rtl_phy_write(tp, 0x0a43, 25,
			      rtl_phy_read(tp, 0x0a43, 25) | (BIT(4) | BIT(2)));
		/* keep RXC in LPI mode */
		rtl_mmd_write(tp, 0x3, 0,
			      rtl_mmd_read(tp, 0x3, 0) & ~BIT(10));
		/* EEE Tw_sys_tx timing adjustment,
		 * make sure MAC would send data after FEPHY wake up
		 */
		/* default 0x001F, 100M EEE */
		rtl_ocp_write(tp, 0xe04a, 0x002f);
		/* EEEP Tw_sys_tx timing adjustment,
		 * make sure MAC would send data after FEPHY wake up
		 */
		 /* default 0x3f, 10M EEEP */
		/* FEPHY needs 160us */
		rtl_ocp_write(tp, 0xe08a, 0x00a8);
		/* default 0x0005, 100M EEEP */
		rtl_ocp_write(tp, 0xe08c, 0x0008);
		/* EEE MAC mode */
		rtl_ocp_write(tp, 0xe040,
			      rtl_ocp_read(tp, 0xe040) | BIT(1) | BIT(0));
		/* EEE+ MAC mode */
		rtl_ocp_write(tp, 0xe080, rtl_ocp_read(tp, 0xe080) | BIT(1));
	} else {
		/* disable eee auto fallback function */
		rtl_phy_write(tp, 0x0a4b, 17,
			      rtl_phy_read(tp, 0x0a4b, 17) & ~BIT(2));
		/* 100M EEE capability */
		rtl_mmd_write(tp, 0x7, 60, 0);
		/* 10M EEE */
		rtl_phy_write(tp, 0x0a43, 25,
			      rtl_phy_read(tp, 0x0a43, 25) & ~(BIT(4) | BIT(2)));
		/* keep RXC in LPI mode */
		rtl_mmd_write(tp, 0x3, 0,
			      rtl_mmd_read(tp, 0x3, 0) & ~BIT(10));
		/* reset to default value */
		rtl_ocp_write(tp, 0xe040, rtl_ocp_read(tp, 0xe040) | BIT(13));
		/* EEE MAC mode */
		rtl_ocp_write(tp, 0xe040,
			      rtl_ocp_read(tp, 0xe040) & ~(BIT(1) | BIT(0)));
		/* EEE+ MAC mode */
		rtl_ocp_write(tp, 0xe080, rtl_ocp_read(tp, 0xe080) & ~BIT(1));
		/* EEE Tw_sys_tx timing restore */
		/* default 0x001F, 100M EEE */
		rtl_ocp_write(tp, 0xe04a, 0x001f);
		/* EEEP Tw_sys_tx timing restore */
		/* default 0x3f, 10M EEEP */
		rtl_ocp_write(tp, 0xe08a, 0x003f);
		/* default 0x0005, 100M EEEP */
		rtl_ocp_write(tp, 0xe08c, 0x0005);
	}
}

static void rtd13xxd_led_set(struct rtl8169_private *tp, bool enable)
{
	if (enable) {
		regmap_update_bits_base(tp->pinctrl_base, RTD13XXD_ISO_TESTMUX_MUXPAD3,
					GENMASK(31, 24),
					(0x1 << 28) | (0x1 << 24),
					NULL, false, true);
	} else {
		regmap_update_bits_base(tp->pinctrl_base, RTD13XXD_ISO_TESTMUX_MUXPAD3,
					GENMASK(31, 24),
					0, NULL, false, true);
	}
}

static void rtd13xxd_dump_regs(struct seq_file *m, struct rtl8169_private *tp)
{
	u32 val;

	regmap_read(tp->iso_base, ISO_UMSK_ISR, &val);
	seq_printf(m, "ISO_UMSK_ISR\t[0x98007004] = %08x\n", val);
	regmap_read(tp->iso_base, ISO_PWRCUT_ETN, &val);
	seq_printf(m, "ISO_PWRCUT_ETN\t[0x9800705c] = %08x\n", val);
	regmap_read(tp->iso_base, ISO_ETN_TESTIO, &val);
	seq_printf(m, "ISO_ETN_TESTIO\t[0x98007060] = %08x\n", val);
	regmap_read(tp->iso_base, ISO_SOFT_RESET, &val);
	seq_printf(m, "ETN_RESET_CTRL\t[0x98007088] = %08x\n", val);
	regmap_read(tp->iso_base, ISO_CLOCK_ENABLE, &val);
	seq_printf(m, "ETN_CLK_CTRL\t[0x9800708c] = %08x\n", val);
}

/******************* END of RTD13XXD ****************************/

/* RTD13XXE */
static void rtd13xxe_load_otp_content(struct rtl8169_private *tp)
{
	int otp;
	u16 tmp;
	u8 *buf;

	buf = r8169soc_read_otp(tp, "rc_r_amp_cal");
	if (IS_ERR(buf))
		return;

	/* Set RS (for TX) from OTP 0x98032504[2:0] */
	otp = (buf[4] & (0x7 << 0)) >> 0;
	tmp = otp ^ RTD13XXE_RS_DEFAULT;
	tmp = (int_mdio_read(tp, 0x0bd0, 21) & ~(0x7 << 13)) | (tmp << 13);
	int_mdio_write(tp, 0x0bd0, 21, tmp);

	/* Set RSW (for RX) from OTP 0x98032504[5:3] */
	otp = (buf[4] & (0x7 << 3)) >> 3;
	tmp = otp ^ RTD13XXE_RSW_DEFAULT;
	tmp = (int_mdio_read(tp, 0x0bd0, 20) & ~(0x7 << 13)) | (tmp << 13);
	int_mdio_write(tp, 0x0bd0, 20, tmp);

	/* Set idac_fine_mdi from OTP 0x98032500[4:0] */
	otp = (buf[0] & (0x1F << 0)) >> 0;
	tmp = otp ^ RTD13XXE_IDAC_FINE_MDI_DEFAULT;
	tmp += tp->amp_k_offset;
	tmp = (int_mdio_read(tp, 0x0bc0, 23) & ~(0x1f << 0)) | (tmp << 0);
	int_mdio_write(tp, 0x0bc0, 23, tmp);

	/* Set idac_fine_mdix from OTP 0x98032500[9:5] */
	otp = ((buf[0] | (buf[1] << 8)) & (0x1F << 5)) >> 5;
	tmp = otp ^ RTD13XXE_IDAC_FINE_MDIX_DEFAULT;
	tmp += tp->amp_k_offset;
	tmp = (int_mdio_read(tp, 0x0bc0, 23) & ~(0x1f << 8)) | (tmp << 8);
	int_mdio_write(tp, 0x0bc0, 23, tmp);

	kfree(buf);
}

static void rtd13xxe_phy_iol_tuning(struct rtl8169_private *tp)
{
	u16 tmp;

	tmp = 0x30 << 4;
	tmp = (int_mdio_read(tp, 0x0a81, 23) & ~(0x3F << 4)) | tmp;
	int_mdio_write(tp, 0x0a81, 23, tmp);
}

static void rtd13xxe_mdio_init(struct rtl8169_private *tp)
{
	struct device *d = tp_to_dev(tp);
	u32 tmp;

	fsleep(10000);	/* wait for MDIO ready */

	/* PHY will stay in state 1 mode */
	tmp = 0;
	while (0x1 != (int_mdio_read(tp, 0x0a42, 16) & 0x07)) {
		tmp += 1;
		fsleep(1000);
		if (tmp >= 2000) {
			dev_err(d, "PHY status is not 0x1 in bypass mode, current = 0x%02x\n",
				(int_mdio_read(tp, 0x0a42, 16) & 0x07));
			set_bit(RTL_STATUS_HW_FAIL, tp->status);
			break;
		}
	}

	/* ETN spec. set MDC freq = 8.9MHZ */
	tmp = rtl_ocp_read(tp, 0xde10) & ~(BIT(7) | BIT(6));
	tmp |= BIT(6); /* MDC freq = 8.9MHz */
	rtl_ocp_write(tp, 0xde10, tmp);

	/* fill fuse_rdy & rg_ext_ini_done */
	int_mdio_write(tp, 0x0a46, 20,
		       int_mdio_read(tp, 0x0a46, 20) | (BIT(1) | BIT(0)));

	/* init_autoload_done = 1 */
	tmp = rtl_ocp_read(tp, 0xe004);
	tmp |= BIT(7);
	rtl_ocp_write(tp, 0xe004, tmp);

	/* ee_mode = 3 */
	rtl_unlock_config_regs(tp);

	/* wait LAN-ON */
	tmp = 0;
	do {
		tmp += 1;
		fsleep(1000);
		if (tmp >= 2000) {
			dev_err(d, "PHY status is not 0x3, current = 0x%02x\n",
				(int_mdio_read(tp, 0x0a42, 16) & 0x07));
			set_bit(RTL_STATUS_HW_FAIL, tp->status);
			break;
		}
	} while (0x3 != (int_mdio_read(tp, 0x0a42, 16) & 0x07));
	dev_info(d, "wait %d ms for PHY ready, current = 0x%x\n",
		 tmp, int_mdio_read(tp, 0x0a42, 16));

	/* adjust PHY electrical characteristics */
	rtd13xxe_phy_iol_tuning(tp);

	/* load OTP contents (R-K, Amp)
	 */
	rtd13xxe_load_otp_content(tp);

	switch (tp->output_mode) {
	case OUTPUT_EMBEDDED_PHY:
		/* ISO spec, set data path to FEPHY */
		tmp = rtl_ocp_read(tp, 0xea30);
		tmp &= ~(BIT(0));
		rtl_ocp_write(tp, 0xea30, tmp);

		tmp = rtl_ocp_read(tp, 0xea34);
		tmp &= ~(BIT(1) | BIT(0));
		tmp |= BIT(1);  /* FEPHY */
		rtl_ocp_write(tp, 0xea34, tmp);

		/* LED low active circuit */
		/* output current: 4mA */
		regmap_update_bits_base(tp->pinctrl_base, RTD13XXE_ISO_TESTMUX_PFUNC7,
					GENMASK(24, 15),
					(0x16 << 20) | (0x16 << 15),
					NULL, false, true);

		/* disable GMII for embedded FEPHY */
		tp->chip->features &= ~RTL_FEATURE_GMII;
		break;
	default:
		dev_err(d, "%s:%d: unsupported output mode (%d)\n",
			__func__, __LINE__, tp->output_mode);
		set_bit(RTL_STATUS_HW_FAIL, tp->status);
	}
}

static void rtd13xxe_led_set(struct rtl8169_private *tp, bool enable)
{
	if (enable) {
		regmap_update_bits_base(tp->pinctrl_base, RTD13XXE_ISO_TESTMUX_MUXPAD2,
					GENMASK(31, 24),
					(0x1 << 28) | (0x1 << 24),
					NULL, false, true);
	} else {
		regmap_update_bits_base(tp->pinctrl_base, RTD13XXE_ISO_TESTMUX_MUXPAD2,
					GENMASK(31, 24),
					0, NULL, false, true);
	}
}

/******************* END of RTD13XXE ****************************/

/* RTD1625 */
static void rtd1625_reset_phy_gmac(struct rtl8169_private *tp)
{
	struct clk *clk_etn_sys  = clk_get(&tp->pdev->dev, "etn_sys");
	struct clk *clk_etn_250m = clk_get(&tp->pdev->dev, "etn_250m");
	struct clk *clk_misc = clk_get(&tp->pdev->dev, "misc");
	struct reset_control *rstc_gphy =
		reset_control_get_exclusive(&tp->pdev->dev, "gphy");
	struct reset_control *rstc_gmac =
		reset_control_get_exclusive(&tp->pdev->dev, "gmac");
	struct reset_control *rstc_misc =
		reset_control_get_exclusive(&tp->pdev->dev, "misc");

	/* pre-init clk bits */
	if (!test_bit(RTL_STATUS_REINIT, tp->status)) {
		clk_prepare_enable(clk_etn_sys);
		clk_prepare_enable(clk_etn_250m);
	}

	/* 0x98129300[17:16]=0: Release main2 power */
	regmap_update_bits_base(tp->iso_sys_base, RTD1625_ISO_SYS_PWR_CTRL,
				BIT(16) | BIT(17), 0, NULL, false, true);

	/* 0x98000000[1:0]=0x3: Deassert rstn_misc */
	reset_control_deassert(rstc_misc);

	/* 0x98000050[11:10]=0x3: Clock enable for MISC */
	clk_prepare_enable(clk_misc);

	/* 0x98007088[10]=0x1: Deassert FEPHY reset */
	reset_control_deassert(rstc_gphy);

	/* 0x9804f188[0]=0x0: Disable DBUS clock gating */
	regmap_update_bits_base(tp->m2tmx_base, RTD1625_M2TMX_ETN_DBUS_CTRL,
				BIT(0), 0, NULL, false, true);

	/* reg_0x9800708c[12:11] = 00 */
	/* ISO spec, clock enable bit for etn clock & etn 250MHz */
	clk_disable_unprepare(clk_etn_sys);
	clk_disable_unprepare(clk_etn_250m);

	/* reg_0x98007088[10:9] = 00 */
	/* ISO spec, rstn_gphy & rstn_gmac */
	reset_control_assert(rstc_gphy);
	reset_control_assert(rstc_gmac);

	/* reg_0x98007060[1] = 1 */
	/* ISO spec, bypass mode enable */
	regmap_update_bits_base(tp->iso_base, ISO_ETN_TESTIO,
				BIT(1), BIT(1), NULL, false, true);

	/* reg_0x9804f180[5] = 0 */
	/* set int PHY addr = 1 */
	/* set ext PHY addr as dts setting */
	regmap_write(tp->m2tmx_base, RTD1625_M2TMX_PWRCUT_ETN,
		     0x04012713 | ((tp->ext_phy_id & 0x1f) << 21));

	fsleep(1000);

	/* release resource */
	reset_control_put(rstc_gphy);
	reset_control_put(rstc_gmac);
	reset_control_put(rstc_misc);
	clk_put(clk_etn_sys);
	clk_put(clk_etn_250m);
	clk_put(clk_misc);
}

static void rtd1625_pll_clock_init(struct rtl8169_private *tp)
{
	struct device *d = tp_to_dev(tp);
	struct clk *clk_etn_sys  = clk_get(&tp->pdev->dev, "etn_sys");
	struct clk *clk_etn_250m = clk_get(&tp->pdev->dev, "etn_250m");
	struct reset_control *rstc_gphy =
		reset_control_get_exclusive(&tp->pdev->dev, "gphy");
	struct reset_control *rstc_gmac =
		reset_control_get_exclusive(&tp->pdev->dev, "gmac");
	u32 tmp;
	u32 val;

	/* reg_0x98007004[27] = 1 */
	/* ISO spec, ETN_PHY_INTR, clear ETN interrupt for ByPassMode */
	regmap_write(tp->iso_base, ISO_UMSK_ISR, BIT(27));

	/* reg_0x98007088[10] = 1 */
	/* ISO spec, reset bit of fephy */
	reset_control_deassert(rstc_gphy);

	fsleep(1000);	/* wait 1ms for PHY PLL stable */

	/* reg_0x98007060[1] = 1 */
	/* ISO spec, bypass mode enable */
	regmap_update_bits_base(tp->iso_base, ISO_ETN_TESTIO,
				BIT(1), BIT(1), NULL, false, true);

	/* reg_0x98007088[9] = 1 */
	/* ISO spec, reset bit of gmac */
	reset_control_deassert(rstc_gmac);

	/* reg_0x9800708c[12:11] = 11 */
	/* ISO spec, clock enable bit for etn clock & etn 250MHz */
	clk_prepare_enable(clk_etn_sys);
	clk_prepare_enable(clk_etn_250m);

	if (tp->phy_ctrl_loc) {
		/* 0x9804e128[9]=0x1: ETN PHY CTRL mux to location 1 */
		regmap_update_bits_base(tp->pinctrl_base, RTD1625_ISO_MUXPAD32,
					BIT(8) | BIT(9), BIT(9), NULL, false,
					true);
	}

	/* Check ETN MAC init_autoload_done reg_0x98007070[0] = 1 */
	/* Bit 0 changes to 1 means ETN Rbus is valid */
	regmap_read(tp->iso_base, ISO_PLL_WDOUT, &val);
	tmp = 0;
	while (0x1 != (val & BIT(0))) {
		tmp += 1;
		fsleep(1000);
		if (tmp >= MAC_INIT_TIMEOUT) {
			dev_err(d, "GMAC init timeout\n");
			set_bit(RTL_STATUS_HW_FAIL, tp->status);
			break;
		}
		regmap_read(tp->iso_base, ISO_PLL_WDOUT, &val);
	}
	if (tmp < MAC_INIT_TIMEOUT)
		dev_info(d, "GMAC wait %d ms for init_autoload_done\n", tmp);

	/* release resource */
	reset_control_put(rstc_gphy);
	reset_control_put(rstc_gmac);
	clk_put(clk_etn_sys);
	clk_put(clk_etn_250m);
}

static void rtd1625_load_otp_content(struct rtl8169_private *tp)
{
}

static void rtd1625_phy_iol_tuning(struct rtl8169_private *tp)
{
}

static void rtd1625_mdio_init(struct rtl8169_private *tp)
{
	struct device *d = tp_to_dev(tp);
	u32 tmp;
	struct pinctrl_state *ps_rgmii_mdio;
	const struct soc_device_attribute rtk_soc_kent_b[] = {
		{
			.soc_id = "RTD1501B",
		},
		{
			.soc_id = "RTD1861B",
		},
		{
		/* empty */
		}
	};

	fsleep(10000);	/* wait for MDIO ready */

	/* PHY will stay in state 1 mode */
	tmp = 0;
	while (0x1 != (int_mdio_read(tp, 0x0a42, 16) & 0x07)) {
		tmp += 1;
		fsleep(1000);
		if (tmp >= 2000) {
			dev_err(d, "PHY status is not 0x1 in bypass mode, current = 0x%02x\n",
				(int_mdio_read(tp, 0x0a42, 16) & 0x07));
			set_bit(RTL_STATUS_HW_FAIL, tp->status);
			break;
		}
	}

	/* ETN spec. set MDC freq = 8.9MHZ */
	tmp = rtl_ocp_read(tp, 0xde10) & ~(BIT(7) | BIT(6));
	tmp |= BIT(6); /* MDC freq = 8.9MHz */
	rtl_ocp_write(tp, 0xde10, tmp);
	tmp = rtl_ocp_read(tp, 0xde30) & ~(BIT(7) | BIT(6));
	tmp |= BIT(6); /* MDC freq = 8.9MHz */
	rtl_ocp_write(tp, 0xde30, tmp);

	/* fill fuse_rdy & rg_ext_ini_done */
	int_mdio_write(tp, 0x0a46, 20,
		       int_mdio_read(tp, 0x0a46, 20) | (BIT(1) | BIT(0)));

	/* init_autoload_done = 1 */
	tmp = rtl_ocp_read(tp, 0xe004);
	tmp |= BIT(7);
	rtl_ocp_write(tp, 0xe004, tmp);

	/* ee_mode = 3 */
	rtl_unlock_config_regs(tp);

	/* wait LAN-ON */
	tmp = 0;
	do {
		tmp += 1;
		fsleep(1000);
		if (tmp >= 2000) {
			dev_err(d, "PHY status is not 0x3, current = 0x%02x\n",
				(int_mdio_read(tp, 0x0a42, 16) & 0x07));
			set_bit(RTL_STATUS_HW_FAIL, tp->status);
			break;
		}
	} while (0x3 != (int_mdio_read(tp, 0x0a42, 16) & 0x07));
	dev_info(d, "wait %d ms for PHY ready, current = 0x%x\n",
		 tmp, int_mdio_read(tp, 0x0a42, 16));

	/* adjust PHY electrical characteristics */
	rtd1625_phy_iol_tuning(tp);

	/* load OTP contents (R-K, Amp)
	 */
	rtd1625_load_otp_content(tp);

	/* only some variants can support RMII/RGMII */
	if (!soc_device_match(rtk_soc_kent_b))
		tp->output_mode = OUTPUT_EMBEDDED_PHY;

	switch (tp->output_mode) {
	case OUTPUT_EMBEDDED_PHY:
		/* ISO spec, set data path to FEPHY */
		tmp = rtl_ocp_read(tp, 0xea30);
		tmp &= ~(BIT(0));
		rtl_ocp_write(tp, 0xea30, tmp);

		tmp = rtl_ocp_read(tp, 0xea34);
		tmp &= ~(BIT(1) | BIT(0));
		tmp |= BIT(1);  /* FEPHY */
		rtl_ocp_write(tp, 0xea34, tmp);

		/* disable GMII for embedded FEPHY */
		tp->chip->features &= ~RTL_FEATURE_GMII;
		break;
	case OUTPUT_RGMII_TO_MAC:
		dev_err(d, "%s:%d: PHY_TYPE_RGMII_MAC is not supported yet\n", __func__, __LINE__);
		set_bit(RTL_STATUS_HW_FAIL, tp->status);
		break;
	case OUTPUT_RMII:
		dev_err(d, "%s:%d: PHY_TYPE_RMII is not supported yet\n", __func__, __LINE__);
		set_bit(RTL_STATUS_HW_FAIL, tp->status);
		break;
	case OUTPUT_RGMII_TO_PHY:
		/* ISO mux spec, GPIO15 is set to MDC pin */
		/* ISO mux spec, GPIO14 is set to MDIO pin */
		/* ISO mux spec, GPIO65~76 is set to TX/RX pin */
		ps_rgmii_mdio = pinctrl_lookup_state(tp->pc, "rgmii");
		pinctrl_select_state(tp->pc, ps_rgmii_mdio);

		/* ISO Spec: Configure data path to access ETN */
		/* 0x9804f180[4] = 0 */
		regmap_update_bits_base(tp->m2tmx_base, RTD1625_M2TMX_PWRCUT_ETN,
					BIT(4), 0, NULL, false, true);

		/* check if RGMII voltage detection is enabled */
		regmap_read(tp->pinctrl_base, RTD1625_ISO_DBG_STATUS, &tmp);
		if (tmp & BIT(8)) {
			tp->rgmii.voltage = (tmp & BIT(1)) ?  3 : 1;
			dev_info(d, "detected RGMII voltage = %sV\n",
				 (tp->rgmii.voltage == 3) ? "3.3" : "1.8");
		}

		switch (tp->rgmii.voltage) {
		case 3: /* 3.3v */
			/* RGMII_VDSEL = 3.3V */
			regmap_update_bits_base(tp->pinctrl_base, RTD1625_ISO_PFUNC42,
						GENMASK(17, 16), 0, NULL, false, true);

			/* pad_driving_n=0, pad_driving_p=0,
			 * pudsel=0, puden=0, E2=0 (4mA)
			 */
			/* GPIO[81:80] */
			regmap_update_bits_base(tp->m2tmx_base, RTD1625_M2TMX_PFUNC9,
						GENMASK(31, 26), 0, NULL, false, true);

			/* GPIO[87:82] */
			regmap_update_bits_base(tp->m2tmx_base, RTD1625_M2TMX_PFUNC10,
						GENMASK(31, 0), 0, NULL, false, true);

			/* GPIO[90:88] */
			regmap_update_bits_base(tp->m2tmx_base, RTD1625_M2TMX_PFUNC11,
						GENMASK(29, 0), 0, NULL, false, true);

			/* GPIO[91] */
			regmap_update_bits_base(tp->m2tmx_base, RTD1625_M2TMX_PFUNC12,
						GENMASK(9, 0), 0, NULL, false, true);

			break;
		case 2: /* 2.5v */
			dev_err(d, "%s:%d: 2.5V is not supported yet\n", __func__, __LINE__);
			set_bit(RTL_STATUS_HW_FAIL, tp->status);
			break;
		case 1: /* 1.8v */
		default:
			/* RGMII_VDSEL = 1.8V */
			regmap_update_bits_base(tp->pinctrl_base, RTD1625_ISO_PFUNC42,
						GENMASK(17, 16), BIT(17), NULL, false, true);

			/* pad_driving_n=0, pad_driving_p=0,
			 * pudsel=0, puden=0, E2=1 (8mA)
			 */
			/* GPIO[81:80] */
			regmap_update_bits_base(tp->m2tmx_base, RTD1625_M2TMX_PFUNC9,
						GENMASK(31, 26), (1 << 29) | (1 << 26),
						NULL, false, true);

			/* GPIO[87:82] */
			regmap_update_bits_base(tp->m2tmx_base, RTD1625_M2TMX_PFUNC10,
						GENMASK(31, 0),
						(1 << 9) | (1 << 6) | (1 << 3) | (1 << 0),
						NULL, false, true);

			/* GPIO[90:88] */
			regmap_update_bits_base(tp->m2tmx_base, RTD1625_M2TMX_PFUNC11,
						GENMASK(29, 0), 0, NULL, false, true);

			/* GPIO[91] */
			regmap_update_bits_base(tp->m2tmx_base, RTD1625_M2TMX_PFUNC12,
						GENMASK(9, 0), 0, NULL, false, true);
		}

		/* check if external PHY is available */
		dev_info(d, "Searching external PHY...");
		tp->ext_phy = true;
		tmp = 0;
		while (rtl_phy_read(tp, 0x0a43, 31) != 0x0a43) {
			tmp += 1;
			fsleep(1000);
			if (tmp >= 2000) {
				dev_err(d, "\n External RGMII PHY not found, current = 0x%02x\n",
					rtl_phy_read(tp, 0x0a43, 31));
				set_bit(RTL_STATUS_HW_FAIL, tp->status);
				return;
			}
		}
		if (tmp < 2000)
			dev_info(d, "found.\n");

		/* set RGMII RX delay */
		tmp = rtl_ocp_read(tp, 0xea34) &
			~(BIT(14) | BIT(9) | BIT(8) | BIT(6));
		switch (tp->rgmii.rx_delay) {
		case 1:
			tmp |= (0x0 << 6) | (0x1 << 8);
			break;
		case 2:
			tmp |= (0x0 << 6) | (0x2 << 8);
			break;
		case 3:
			tmp |= (0x0 << 6) | (0x3 << 8);
			break;
		case 4:
			tmp |= (0x1 << 6) | (0x0 << 8);
			break;
		case 5:
			tmp |= (0x1 << 6) | (0x1 << 8);
			break;
		case 6:
			tmp |= (0x1 << 6) | (0x2 << 8);
			break;
		case 7:
			tmp |= (0x1 << 6) | (0x3 << 8);
		}
		rtl_ocp_write(tp, 0xea34, tmp);

		tmp = rtl_ocp_read(tp, 0xea36) | (BIT(0)); /* rg_clk_en */
		rtl_ocp_write(tp, 0xea36, tmp);

		/* set RGMII TX delay */
		if (tp->rgmii.tx_delay == 0) {
			tmp = rtl_ocp_read(tp, 0xea34) & ~(BIT(7));
			rtl_ocp_write(tp, 0xea34, tmp);
		} else {	/* tp->tx_delay == 1 (2ns) */
			tmp = rtl_ocp_read(tp, 0xea34) | (BIT(7));
			rtl_ocp_write(tp, 0xea34, tmp);
		}

		/* ISO spec, data path is set to RGMII */
		tmp = rtl_ocp_read(tp, 0xea30) & ~(BIT(0));
		rtl_ocp_write(tp, 0xea30, tmp);

		tmp = rtl_ocp_read(tp, 0xea34) & ~(BIT(0) | BIT(1));
		tmp |= BIT(0); /* RGMII */
		rtl_ocp_write(tp, 0xea34, tmp);

		/* ext_phy == true now */

		break;
	default:
		dev_err(d, "%s:%d: unsupported output mode (%d)\n",
			__func__, __LINE__, tp->output_mode);
		set_bit(RTL_STATUS_HW_FAIL, tp->status);
	}
}

static void rtd1625_eee_set(struct rtl8169_private *tp, bool enable)
{
	if (enable) {
		/* enable eee auto fallback function */
		//rtl_phy_write(tp, 0x0a4b, 17,
		//	      rtl_phy_read(tp, 0x0a4b, 17) | BIT(2));
		/* 100M EEE capability */
		rtl_mmd_write(tp, 0x7, 60, rtl_mmd_read(tp, 0x7, 60) | BIT(1));
		/* 10M EEE */
		rtl_phy_write(tp, 0x0a43, 25,
			      rtl_phy_read(tp, 0x0a43, 25) | (BIT(5) | BIT(4)));
		/* keep RXC in LPI mode */
		rtl_mmd_write(tp, 0x3, 0,
			      rtl_mmd_read(tp, 0x3, 0) & ~BIT(10));
		/* EEE Tw_sys_tx timing adjustment,
		 * make sure MAC would send data after FEPHY wake up
		 */
		/* default 0x001F, 100M EEE */
		rtl_ocp_write(tp, 0xe04a, 0x002f);
		/* EEEP Tw_sys_tx timing adjustment,
		 * make sure MAC would send data after FEPHY wake up
		 */
		 /* default 0x3f, 10M EEEP */
		/* FEPHY needs 160us */
		rtl_ocp_write(tp, 0xe08a, 0x00a8);
		/* default 0x0005, 100M EEEP */
		rtl_ocp_write(tp, 0xe08c, 0x0008);
		/* EEE MAC mode */
		rtl_ocp_write(tp, 0xe040,
			      rtl_ocp_read(tp, 0xe040) | BIT(1) | BIT(0));
		/* EEE+ MAC mode */
		rtl_ocp_write(tp, 0xe080, rtl_ocp_read(tp, 0xe080) | BIT(1));
	} else {
		/* disable eee auto fallback function */
		//rtl_phy_write(tp, 0x0a4b, 17,
		//	      rtl_phy_read(tp, 0x0a4b, 17) & ~BIT(2));
		/* 100M EEE capability */
		rtl_mmd_write(tp, 0x7, 60, rtl_mmd_read(tp, 0x7, 60) & ~BIT(1));
		/* 10M EEE */
		rtl_phy_write(tp, 0x0a43, 25,
			      rtl_phy_read(tp, 0x0a43, 25) & ~(BIT(5) | BIT(4)));
		/* keep RXC in LPI mode */
		rtl_mmd_write(tp, 0x3, 0,
			      rtl_mmd_read(tp, 0x3, 0) & ~BIT(10));
		/* reset to default value */
		rtl_ocp_write(tp, 0xe040, rtl_ocp_read(tp, 0xe040) | BIT(13));
		/* EEE MAC mode */
		rtl_ocp_write(tp, 0xe040,
			      rtl_ocp_read(tp, 0xe040) & ~(BIT(1) | BIT(0)));
		/* EEE+ MAC mode */
		rtl_ocp_write(tp, 0xe080, rtl_ocp_read(tp, 0xe080) & ~BIT(1));
		/* EEE Tw_sys_tx timing restore */
		/* default 0x001F, 100M EEE */
		rtl_ocp_write(tp, 0xe04a, 0x001f);
		/* EEEP Tw_sys_tx timing restore */
		/* default 0x3f, 10M EEEP */
		rtl_ocp_write(tp, 0xe08a, 0x003f);
		/* default 0x0005, 100M EEEP */
		rtl_ocp_write(tp, 0xe08c, 0x0005);
	}
}

static void rtd1625_dump_regs(struct seq_file *m, struct rtl8169_private *tp)
{
	u32 val;

	regmap_read(tp->iso_base, ISO_UMSK_ISR, &val);
	seq_printf(m, "ISO_UMSK_ISR\t[0x98007004] = %08x\n", val);
	regmap_read(tp->iso_base, ISO_SOFT_RESET, &val);
	seq_printf(m, "ETN_RESET_CTRL\t[0x98007088] = %08x\n", val);
	regmap_read(tp->iso_base, ISO_CLOCK_ENABLE, &val);
	seq_printf(m, "ETN_CLK_CTRL\t[0x9800708c] = %08x\n", val);
	regmap_read(tp->pinctrl_base, RTD1625_ISO_MUXPAD32, &val);
	seq_printf(m, "ISO_MUXPAD32\t[0x9804e128] = %08x\n", val);
	regmap_read(tp->pinctrl_base, RTD1625_ISO_PFUNC42, &val);
	seq_printf(m, "ISO_PFUNC42\t[0x9804e188] = %08x\n", val);
	regmap_read(tp->pinctrl_base, RTD1625_ISO_DBG_STATUS, &val);
	seq_printf(m, "ISO_DBG_STATUS\t[0x9804e1a0] = %08x\n", val);
	regmap_read(tp->m2tmx_base, RTD1625_M2TMX_PWRCUT_ETN, &val);
	seq_printf(m, "M2TMX_PWRCUT\t[0x9804f180] = %08x\n", val);
	regmap_read(tp->m2tmx_base, RTD1625_M2TMX_ETN_DBUS_CTRL, &val);
	seq_printf(m, "M2TMX_DBUS_CTL\t[0x9804f188] = %08x\n", val);
	regmap_read(tp->m2tmx_base, RTD1625_M2TMX_ETN_MISC, &val);
	seq_printf(m, "M2TMX_ETN_MISC\t[0x9804f194] = %08x\n", val);
	regmap_read(tp->m2tmx_base, RTD1625_M2TMX_PFUNC9, &val);
	seq_printf(m, "M2TMX_PFUNC9\t[0x9804f238] = %08x\n", val);
	regmap_read(tp->m2tmx_base, RTD1625_M2TMX_PFUNC10, &val);
	seq_printf(m, "M2TMX_PFUNC10\t[0x9804f23c] = %08x\n", val);
	regmap_read(tp->m2tmx_base, RTD1625_M2TMX_PFUNC11, &val);
	seq_printf(m, "M2TMX_PFUNC11\t[0x9804f240] = %08x\n", val);
	regmap_read(tp->m2tmx_base, RTD1625_M2TMX_PFUNC12, &val);
	seq_printf(m, "M2TMX_PFUNC12\t[0x9804f244] = %08x\n", val);
}

static void rtd1625_led_set(struct rtl8169_private *tp, bool enable)
{
	struct pinctrl_state *ps_led;

	if (enable)
		ps_led = pinctrl_lookup_state(tp->pc, "led-on");
	else
		ps_led = pinctrl_lookup_state(tp->pc, "led-off");

	pinctrl_select_state(tp->pc, ps_led);
}

/******************* END of RTD1625 ****************************/

static struct r8169soc_chip_info rtd119x_info = {
	.name			= "RTD119X",
	.reset_phy_gmac		= rtd119x_reset_phy_gmac,
	.acp_init		= NULL,
	.pll_clock_init		= rtd119x_pll_clock_init,
	.mac_mcu_patch		= rtd119x_mac_mcu_patch,
	.hw_phy_config		= rtd119x_hw_phy_config,
	.led_set		= rtd119x_led_set,
	.jumbo_max		= JUMBO_1K,
	.jumbo_tx_csum		= false,
	.led_cfg		= 0x000670CA,
	.features		= RTL_FEATURE_GMII,
};

static struct r8169soc_chip_info rtd129x_info = {
	.name			= "RTD129X",
	.mdio_lock		= rtd129x_mdio_lock,
	.mdio_unlock		= rtd129x_mdio_unlock,
	.reset_phy_gmac		= rtd129x_reset_phy_gmac,
	.acp_init		= NULL,
	.pll_clock_init		= rtd129x_pll_clock_init,
	.mac_mcu_patch		= rtd129x_mac_mcu_patch,
	.hw_phy_config		= rtd129x_hw_phy_config,
	.eee_set		= rtd129x_eee_set,
	.led_set		= rtd129x_led_set,
	.dump_regs		= rtd129x_dump_regs,
	.dump_var		= rtd129x_dump_var,
	.jumbo_max		= JUMBO_1K,
	.jumbo_tx_csum		= false,
	.led_cfg		= 0x000670CA,
	.features		= RTL_FEATURE_GMII |
				  RTL_FEATURE_EEE,
};

static struct r8169soc_chip_info rtd139x_info = {
	.name			= "RTD139X",
	.mdio_lock		= rtd139x_mdio_lock,
	.mdio_unlock		= rtd139x_mdio_unlock,
	.reset_phy_gmac		= rtd139x_reset_phy_gmac,
	.acp_init		= rtd139x_acp_init,
	.pll_clock_init		= rtd139x_pll_clock_init,
	.mdio_init		= rtd139x_mdio_init,
	.wakeup_set		= rtl_crc_wakeup_set,
	.eee_set		= rtd139x_eee_set,
	.led_set		= rtd139x_led_set,
	.dump_regs		= rtd139x_dump_regs,
	.dump_var		= rtd139x_dump_var,
	.jumbo_max		= JUMBO_1K,
	.jumbo_tx_csum		= false,
	.led_cfg		= 0x000679A9,
	.features		= RTL_FEATURE_GMII |
				  RTL_FEATURE_ACP | RTL_FEATURE_EEE,
};

static struct r8169soc_chip_info rtd16xx_info = {
	.name			= "RTD16XX",
	.mdio_lock		= rtd139x_mdio_lock,
	.mdio_unlock		= rtd139x_mdio_unlock,
	.reset_phy_gmac		= rtd16xx_reset_phy_gmac,
	.acp_init		= rtd16xx_acp_init,
	.pll_clock_init		= rtd16xx_pll_clock_init,
	.mdio_init		= rtd16xx_mdio_init,
	.wakeup_set		= rtl_crc_wakeup_set,
	.eee_set		= rtd16xx_eee_set,
	.led_set		= rtd16xx_led_set,
	.dump_regs		= rtd16xx_dump_regs,
	.dump_var		= rtd16xx_dump_var,
	.jumbo_max		= JUMBO_1K,
	.jumbo_tx_csum		= false,
	.led_cfg		= 0x00067CA9,
	.features		= RTL_FEATURE_GMII |
				  RTL_FEATURE_TX_NO_CLOSE |
				  RTL_FEATURE_ADJUST_FIFO |
				  RTL_FEATURE_ACP | RTL_FEATURE_EEE,
};

static struct r8169soc_chip_info rtd13xx_info = {
	.name			= "RTD13XX",
	.mdio_lock		= rtd139x_mdio_lock,
	.mdio_unlock		= rtd139x_mdio_unlock,
	.reset_phy_gmac		= rtd13xx_reset_phy_gmac,
	.acp_init		= rtd13xx_acp_init,
	.pll_clock_init		= rtd13xx_pll_clock_init,
	.mdio_init		= rtd13xx_mdio_init,
	.wakeup_set		= rtl_crc_wakeup_set,
	.eee_set		= rtd13xx_eee_set,
	.led_set		= rtd13xx_led_set,
	.dump_regs		= rtd13xx_dump_regs,
	.dump_var		= rtd13xx_dump_var,
	.jumbo_max		= JUMBO_1K,
	.jumbo_tx_csum		= false,
	.led_cfg		= 0x00067CA9,
	.features		= RTL_FEATURE_GMII |
				  RTL_FEATURE_TX_NO_CLOSE |
				  RTL_FEATURE_ADJUST_FIFO |
				  RTL_FEATURE_ACP | RTL_FEATURE_EEE,
};

static struct r8169soc_chip_info rtd16xxb_info = {
	.name			= "RTD16XXB",
	.mdio_lock		= rtd139x_mdio_lock,
	.mdio_unlock		= rtd139x_mdio_unlock,
	.reset_phy_gmac		= rtd16xxb_reset_phy_gmac,
	.pll_clock_init		= rtd16xxb_pll_clock_init,
	.mdio_init		= rtd16xxb_mdio_init,
	.wakeup_set		= rtl_pat_wakeup_set,
	.eee_set		= rtd16xxb_eee_set,
	.led_set		= rtd16xxb_led_set,
	.dump_regs		= rtd16xxb_dump_regs,
	.jumbo_max		= JUMBO_1K,
	.jumbo_tx_csum		= false,
	.led_cfg		= 0x17000CA9,
	.features		= RTL_FEATURE_GMII |
				  RTL_FEATURE_TX_NO_CLOSE |
				  RTL_FEATURE_ADJUST_FIFO |
				  RTL_FEATURE_EEE |
				  RTL_FEATURE_OCP_MDIO |
				  RTL_FEATURE_PAT_WAKE,
};

static struct r8169soc_chip_info rtd13xxd_info = {
	.name			= "RTD13XXD",
	.mdio_lock		= rtd139x_mdio_lock,
	.mdio_unlock		= rtd139x_mdio_unlock,
	.reset_phy_gmac		= rtd13xxd_reset_phy_gmac,
	.pll_clock_init		= rtd13xxd_pll_clock_init,
	.mdio_init		= rtd13xxd_mdio_init,
	.wakeup_set		= rtl_pat_wakeup_set,
	.eee_set		= rtd13xxd_eee_set,
	.led_set		= rtd13xxd_led_set,
	.dump_regs		= rtd13xxd_dump_regs,
	.jumbo_max		= JUMBO_1K,
	.jumbo_tx_csum		= false,
	.led_cfg		= 0x170000A9,
	.features		= RTL_FEATURE_TX_NO_CLOSE |
				  RTL_FEATURE_ADJUST_FIFO |
				  RTL_FEATURE_EEE |
				  RTL_FEATURE_OCP_MDIO |
				  RTL_FEATURE_PAT_WAKE,
};

static struct r8169soc_chip_info rtd13xxe_info = {
	.name			= "RTD13XXE",
	.mdio_lock		= rtd139x_mdio_lock,
	.mdio_unlock		= rtd139x_mdio_unlock,
	.reset_phy_gmac		= rtd13xxd_reset_phy_gmac,
	.pll_clock_init		= rtd13xxd_pll_clock_init,
	.mdio_init		= rtd13xxe_mdio_init,
	.wakeup_set		= rtl_pat_wakeup_set,
	.eee_set		= rtd13xxd_eee_set,
	.led_set		= rtd13xxe_led_set,
	.dump_regs		= rtd13xxd_dump_regs,
	.jumbo_max		= JUMBO_1K,
	.jumbo_tx_csum		= false,
	.led_cfg		= 0x170000A9,
	.features		= RTL_FEATURE_TX_NO_CLOSE |
				  RTL_FEATURE_ADJUST_FIFO |
				  RTL_FEATURE_EEE |
				  RTL_FEATURE_OCP_MDIO |
				  RTL_FEATURE_PAT_WAKE |
				  RTL_FEATURE_STORM_CTRL,
};

static struct r8169soc_chip_info rtd1625_info = {
	.name			= "RTD1625",
	.mdio_lock		= rtd139x_mdio_lock,
	.mdio_unlock		= rtd139x_mdio_unlock,
	.reset_phy_gmac		= rtd1625_reset_phy_gmac,
	.pll_clock_init		= rtd1625_pll_clock_init,
	.mdio_init		= rtd1625_mdio_init,
	.mac_mcu_patch		= rtd1625_mdns_mac_mcu_patch,
	.wakeup_set		= rtl_mdns_wakeup_set,
	.eee_set		= rtd1625_eee_set,
	.led_set		= rtd1625_led_set,
	.dump_regs		= rtd1625_dump_regs,
	.jumbo_max		= JUMBO_1K,
	.jumbo_tx_csum		= false,
	.led_cfg		= 0x170000A9,
	.features		= RTL_FEATURE_TX_NO_CLOSE |
				  RTL_FEATURE_ADJUST_FIFO |
				  RTL_FEATURE_EEE |
				  RTL_FEATURE_OCP_MDIO |
				  RTL_FEATURE_PAT_WAKE |
				  RTL_FEATURE_STORM_CTRL |
				  RTL_FEATURE_MDNS_OFFLOAD,
};

static const struct of_device_id r8169soc_dt_ids[] = {
	{
		.compatible = "realtek,rtd119x-r8169soc",
		.data = &rtd119x_info,
	},
	{
		.compatible = "realtek,rtd129x-r8169soc",
		.data = &rtd129x_info,
	},
	{
		.compatible = "realtek,rtd139x-r8169soc",
		.data = &rtd139x_info,
	},
	{
		.compatible = "realtek,rtd16xx-r8169soc",
		.data = &rtd16xx_info,
	},
	{
		.compatible = "realtek,rtd13xx-r8169soc",
		.data = &rtd13xx_info,
	},
	{
		.compatible = "realtek,rtd16xxb-r8169soc",
		.data = &rtd16xxb_info,
	},
	{
		.compatible = "realtek,rtd13xxd-r8169soc",
		.data = &rtd13xxd_info,
	},
	{
		.compatible = "realtek,rtd13xxe-r8169soc",
		.data = &rtd13xxe_info,
	},
	{
		.compatible = "realtek,rtd1325-r8169soc",
		.data = &rtd13xxe_info,
	},
	{
		.compatible = "realtek,rtd1625-r8169soc",
		.data = &rtd1625_info,
	},
	{
	}
};

MODULE_DEVICE_TABLE(of, r8169soc_dt_ids);

static void rtk_chip_info_check(struct r8169soc_chip_info *chip)
{
	if (!chip)
		return;

	if (!chip->mdio_lock)
		chip->mdio_lock = dummy_mdio_lock;
	if (!chip->mdio_unlock)
		chip->mdio_unlock = dummy_mdio_unlock;
	if (!chip->link_ok)
		chip->link_ok = rtl8169_xmii_link_ok;
	if (!chip->reset_phy_gmac)
		chip->reset_phy_gmac = dummy_reset_phy_gmac;
	if (!chip->acp_init)
		chip->acp_init = dummy_acp_init;
	if (!chip->pll_clock_init)
		chip->pll_clock_init = dummy_pll_clock_init;
	if (!chip->mdio_init)
		chip->mdio_init = dummy_mdio_init;
	if (!chip->mac_mcu_patch)
		chip->mac_mcu_patch = dummy_mac_mcu_patch;
	if (!chip->hw_phy_config)
		chip->hw_phy_config = dummy_hw_phy_config;
	if (!chip->wakeup_set)
		chip->wakeup_set = dummy_wakeup_set;
	if (!chip->eee_set)
		chip->eee_set = dummy_eee_set;
	if (!chip->led_set)
		chip->led_set = dummy_led_set;
	if (!chip->dump_regs)
		chip->dump_regs = dummy_dump_regs;
	if (!chip->dump_var)
		chip->dump_var = dummy_dump_var;

	#if defined(CONFIG_RTL_RX_NO_COPY)
	chip->features |= RTL_FEATURE_RX_NO_COPY;
	#endif /* CONFIG_RTL_RX_NO_COPY */
}

static void rtl_init_mac_address(struct rtl8169_private *tp)
{
	u8 mac_addr[ETH_ALEN] __aligned(4) = {};
	struct net_device *dev = tp->dev;
	int rc;
	int retry;
	int i;

	rc = eth_platform_get_mac_address(tp_to_dev(tp), mac_addr);
	if (!rc)
		goto done;

	/* assume the MAC address has been pre-initialized in the bootloader */
	/* workaround: avoid getting deadbeef */
#define RETRY_MAX	10
	for (retry = 0; retry < RETRY_MAX; retry++) {
		for (i = 0; i < ETH_ALEN; i++)
			mac_addr[i] = RTL_R8(tp, MAC0 + i);

		if (*(u32 *)mac_addr == 0xdeadbeef) {
			dev_err(tp_to_dev(tp), "get invalid MAC address %pM, retry %d\n",
				mac_addr, retry);
			rc = RTL_R32(tp, PHYAR);	/* read something else */
			fsleep(10000);
		} else {
			break;
		}
	}

	if (is_valid_ether_addr(mac_addr))
		goto done;

	eth_random_addr(mac_addr);
	dev->addr_assign_type = NET_ADDR_RANDOM;
	dev_warn(tp_to_dev(tp), "can't read MAC address, setting random one\n");
done:
	eth_hw_addr_set(dev, mac_addr);
	ether_addr_copy(tp->mac_addr, mac_addr);
	__rtl_rar_set(tp, mac_addr);
}

#define TMP_STR_LEN	80
static int rtl_init_one(struct platform_device *pdev)
{
	struct r8169soc_chip_info *chip;
	struct rtl8169_private *tp;
	struct net_device *ndev;
	struct pm_dev_param *dev_param;
	void __iomem *ioaddr;
	int i;
	int rc;
	int led_config;
	u32 tmp;
	int irq;
	struct property *wake_mask;
	struct property *wake_pattern;
	char tmp_str[TMP_STR_LEN];
	int wake_size;
	int wake_mask_size;
	u32 bypass_enable = 0;
	u32 acp_enable = 0;
	u32 sgmii_swing = 0;
	u32 voltage = 1;
	u32 tx_delay = 0;
	u32 rx_delay = 0;
	u32 force_speed = 0;
	u32 output_mode = 0;
	u32 ext_phy_id = 1;
	u32 eee_enable = 0;
	u32 wol_enable = 0;
	int amp_k_offset = 0;
	u32 phy_ctrl_loc = 0;

	dev_info(&pdev->dev, "%s Gigabit Ethernet driver %s loaded\n",
		 KBUILD_MODNAME, RTL8169SOC_VERSION);

	chip = (struct r8169soc_chip_info *)of_device_get_match_data(&pdev->dev);
	if (chip) {
		rtk_chip_info_check(chip);
	} else {
		dev_err(&pdev->dev, "%s no proper chip matched\n", __func__);
		rc = -EINVAL;
		goto out;
	}
	if (of_property_read_u32(pdev->dev.of_node, "output-mode", &output_mode))
		dev_dbg(&pdev->dev, "%s can't get output mode", __func__);
	if (of_property_read_u32(pdev->dev.of_node, "wol-enable", &wol_enable))
		dev_dbg(&pdev->dev, "%s can't get wol_enable", __func__);
	if (of_property_read_u32(pdev->dev.of_node, "eee", &eee_enable))
		dev_dbg(&pdev->dev, "%s can't get eee_enable", __func__);
	if (of_property_read_u32(pdev->dev.of_node, "acp", &acp_enable))
		dev_dbg(&pdev->dev, "%s can't get acp", __func__);

	/* optional properties */
	if (of_property_read_u32(pdev->dev.of_node, "bypass",
				 &bypass_enable))
		bypass_enable = 1;
	if (of_property_read_u32(pdev->dev.of_node, "led-cfg", &led_config))
		led_config = 0;
	if (of_property_read_u32(pdev->dev.of_node, "ext-phy-id", &ext_phy_id))
		ext_phy_id = 0;
	if (of_property_read_u32(pdev->dev.of_node, "sgmii-swing",
				 &sgmii_swing))
		sgmii_swing = 0;
	if (of_property_read_u32(pdev->dev.of_node, "voltage", &voltage))
		voltage = 1;
	if (of_property_read_u32(pdev->dev.of_node, "tx-delay", &tx_delay))
		tx_delay = 0;
	if (of_property_read_u32(pdev->dev.of_node, "rx-delay", &rx_delay))
		rx_delay = 0;
	if (of_property_read_u32(pdev->dev.of_node, "force-speed", &force_speed))
		force_speed = 0;
	if (of_get_property(pdev->dev.of_node, "force-Gb-off", NULL)) {
		dev_info(&pdev->dev, "~~~ disable Gb features~~~\n");
		chip->features &= ~RTL_FEATURE_GMII;
	}
	if (of_property_read_u32(pdev->dev.of_node, "amp-k-offset", &tmp)) {
		amp_k_offset = 0;
	} else {
		amp_k_offset = tmp & 0xFFFF;
		if (tmp & BIT(16))	/* bit-16 means negtive value */
			amp_k_offset = 0 - amp_k_offset;
	}
	if (of_property_read_u32(pdev->dev.of_node, "phy-ctrl-loc", &phy_ctrl_loc))
		phy_ctrl_loc = 0;
	/* end of optional properties */

	irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
	if (!pdev->dev.coherent_dma_mask)
		pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	ioaddr = of_iomap(pdev->dev.of_node, 0);

	ndev = devm_alloc_etherdev(&pdev->dev, sizeof(*tp));
	if (!ndev) {
		rc = -ENOMEM;
		goto err_out_iomap;
	}

	SET_NETDEV_DEV(ndev, &pdev->dev);
	ndev->irq = irq;
	ndev->netdev_ops = &rtl_netdev_ops;
	tp = netdev_priv(ndev);
	tp->dev = ndev;
	tp->pdev = pdev;
	tp->chip = chip;
	tp->mmio_addr = ioaddr;
	tp->led_cfg = led_config;
	tp->output_mode = output_mode;
	tp->wol_enable = wol_enable;
	tp->ext_phy_id = ext_phy_id;
	tp->amp_k_offset = amp_k_offset;
	tp->phy_ctrl_loc = !!phy_ctrl_loc;
	tp->force_speed = force_speed;
	if ((tp->chip->features & RTL_FEATURE_EEE) && eee_enable > 0)
		tp->eee_enable = true;
	else
		tp->eee_enable = false;

	ndev->tstats = devm_netdev_alloc_pcpu_stats(&pdev->dev, struct pcpu_sw_netstats);
	if (!ndev->tstats) {
		rc = -ENOMEM;
		goto err_out_iomap;
	}

	/* ISO regs 0x98007000 */
	tp->iso_base = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "realtek,iso");
	if (IS_ERR_OR_NULL(tp->iso_base)) {
		dev_err(&pdev->dev,  "Fail to get iso_base address\n");
		rc = -ENODEV;
		goto err_out_netdev;
	} else {
		dev_dbg(&pdev->dev,  "Get iso_base address\n");
	}

	/* SB2 regs 0x9801a000 (optional) */
	/* it should be valid for rtd129x */
	tp->sb2_base = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "realtek,sb2");
	if (IS_ERR_OR_NULL(tp->sb2_base)) {
		if (tp->chip == &rtd129x_info) {
			dev_err(&pdev->dev,  "Fail to get sb2_base address\n");
			rc = -ENODEV;
			goto err_out_netdev;
		}
	} else {
		dev_dbg(&pdev->dev,  "Get sb2_base address\n");
	}

	/* SBX regs 0x9801c000 for ACP (optional) */
	/* it is not used by rtd119x, rtd129x, and rtd16xxb, rtd13xxd, rtd13xxe */
	/* it should be valid for rtd139x, rtd16xx, and rtd13xx */
	tp->sbx_base = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "realtek,sbx");
	if (IS_ERR_OR_NULL(tp->sbx_base)) {
		if (tp->chip == &rtd139x_info ||
		    tp->chip == &rtd16xx_info ||
		    tp->chip == &rtd13xx_info) {
			dev_err(&pdev->dev,  "Fail to get sbx_base address\n");
			rc = -ENODEV;
			goto err_out_netdev;
		}
	} else {
		dev_dbg(&pdev->dev,  "Get sbx_base address\n");
	}

	/* SCPU wrapper regs 0x9801d000 for ACP (optional) */
	/* it is not used by rtd119x, rtd129x, and rtd16xxb, rtd13xxd, rtd13xxe */
	/* it should be valid for rtd139x, rtd16xx, and rtd13xx */
	tp->scpu_wrap_base = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
							     "realtek,scpu_wrapper");
	if (IS_ERR_OR_NULL(tp->scpu_wrap_base)) {
		if (tp->chip == &rtd139x_info ||
		    tp->chip == &rtd16xx_info ||
		    tp->chip == &rtd13xx_info) {
			dev_err(&pdev->dev,  "Fail to get scpu_wrap_base address\n");
			rc = -ENODEV;
			goto err_out_netdev;
		}
	} else {
		dev_dbg(&pdev->dev,  "Get scpu_wrap_base address\n");
	}

	/* pinctrl regs 0x9804e000 (optional) */
	/* it is not used by rtd119x and rtd129x */
	/* it should be valid for rtd139x, rtd16xx, and rtd13xx */
	tp->pinctrl_base = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "realtek,pinctrl");
	if (IS_ERR_OR_NULL(tp->pinctrl_base)) {
		if (tp->chip != &rtd119x_info && tp->chip != &rtd129x_info) {
			dev_err(&pdev->dev,  "Fail to get pinctrl_base address\n");
			rc = -ENODEV;
			goto err_out_netdev;
		}
	} else {
		dev_dbg(&pdev->dev,  "Get pinctrl_base address\n");
	}

	/* SDS regs 0x981c8000 (optional) */
	/* it should be valid for rtd139x and rtd16xx */
	tp->sds_base = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "realtek,sds");
	if (IS_ERR_OR_NULL(tp->sds_base)) {
		if (tp->chip == &rtd139x_info || tp->chip == &rtd16xx_info) {
			dev_err(&pdev->dev,  "Fail to get sds_base address\n");
			rc = -ENODEV;
			goto err_out_netdev;
		}
	} else {
		dev_dbg(&pdev->dev,  "Get sds_base address\n");
	}

	/* main2 regs 0x9804f000 (optional) */
	/* it should be valid for rtd1625 */
	tp->m2tmx_base = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "realtek,m2tmx");
	if (IS_ERR_OR_NULL(tp->m2tmx_base)) {
		if (tp->chip == &rtd1625_info) {
			dev_err(&pdev->dev,  "Fail to get m2tmx_base address\n");
			rc = -ENODEV;
			goto err_out_netdev;
		}
	} else {
		dev_dbg(&pdev->dev,  "Get m2tmx_base address\n");
	}

	/* iso sys regs 0x98129000 (optional) */
	/* it should be valid for rtd1625 */
	tp->iso_sys_base = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "realtek,iso_sys");
	if (IS_ERR_OR_NULL(tp->iso_sys_base)) {
		if (tp->chip == &rtd1625_info) {
			dev_err(&pdev->dev,  "Fail to get iso_sys_base address\n");
			rc = -ENODEV;
			goto err_out_netdev;
		}
	} else {
		dev_dbg(&pdev->dev,  "Get iso_sys_base address\n");
	}

	/* pinctrl is used to set LED on/off mode */
	tp->pc = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR_OR_NULL(tp->pc)) {
		dev_err(&pdev->dev,  "Fail to get pinctrl\n");
		rc = -ENODEV;
		goto err_out_netdev;
	} else {
		dev_dbg(&pdev->dev,  "Get pinctrl\n");
	}

	tp->bypass_enable = !!(bypass_enable > 0);
	if ((tp->chip->features & RTL_FEATURE_ACP) && acp_enable > 0)
		tp->acp_enable = true;
	else
		tp->acp_enable = false;
	tp->ext_phy = false;

	switch (tp->output_mode) {
	case OUTPUT_RGMII_TO_MAC:
	case OUTPUT_RGMII_TO_PHY:
		tp->rgmii.voltage = voltage;
		tp->rgmii.tx_delay = tx_delay;
		tp->rgmii.rx_delay = rx_delay;
		break;
	case OUTPUT_SGMII_TO_MAC:
	case OUTPUT_SGMII_TO_PHY:
		tp->sgmii.swing = sgmii_swing;
		break;
	case OUTPUT_RMII:
		tp->rmii.voltage = voltage;
		tp->rmii.tx_delay = tx_delay;
		tp->rmii.rx_delay = rx_delay;
		break;
	case OUTPUT_EMBEDDED_PHY:
	case OUTPUT_FORCE_LINK:
	default:
		/* do nothing */
		break;
	}

	/* support DCO mode */
	dev_param = devm_kzalloc(&pdev->dev, sizeof(*dev_param), GFP_KERNEL);
	if (!dev_param) {
		rc = -ENOMEM;
		goto err_out_netdev;
	}

	dev_param->dev = &pdev->dev;
	dev_param->dev_type = LAN;
	dev_param->data = &tp->dco_flag;
	rtk_pm_add_list(dev_param);

	rtl_init_mdio_ops(tp);
	rtl_init_mmd_ops(tp);
	tp->chip->reset_phy_gmac(tp);
	if (test_bit(RTL_STATUS_HW_FAIL, tp->status)) {
		rc = -ENODEV;
		goto err_out_pm_list;
	}

	tp->chip->acp_init(tp);
	if (test_bit(RTL_STATUS_HW_FAIL, tp->status)) {
		rc = -ENODEV;
		goto err_out_pm_list;
	}

	tp->chip->pll_clock_init(tp);
	if (test_bit(RTL_STATUS_HW_FAIL, tp->status)) {
		rc = -ENODEV;
		goto err_out_pm_list;
	}

	tp->chip->mac_mcu_patch(tp);	/* patch SRAM if necessary */
	if (test_bit(RTL_STATUS_HW_FAIL, tp->status)) {
		rc = -ENODEV;
		goto err_out_pm_list;
	}

	tp->chip->mdio_init(tp);
	if (test_bit(RTL_STATUS_HW_FAIL, tp->status)) {
		rc = -ENODEV;
		goto err_out_pm_list;
	}

	/* after r8169soc_mdio_init(),
	 * SGMII : tp->ext_phy == true  ==> external MDIO,
	 * RGMII : tp->ext_phy == true  ==> external MDIO,
	 * RMII  : tp->ext_phy == false ==> internal MDIO,
	 * FE PHY: tp->ext_phy == false ==> internal MDIO
	 */

	/* Enable ALDPS */
	rtl_phy_write(tp, 0x0a43, 24,
		      rtl_phy_read(tp, 0x0a43, 24) | BIT(2));

	tp->saved_wolopts = (tp->wol_enable & WOL_MAGIC) ? WAKE_MAGIC : 0;
	tp->dev->wol_enabled = (tp->wol_enable & WOL_MAGIC) ? 1 : 0;
	tp->wol_crc_cnt = 0;
	if (tp->chip->features & RTL_FEATURE_PAT_WAKE) {
		wake_size = RTL_WAKE_SIZE;
		wake_mask_size = RTL_WAKE_MASK_SIZE;
	} else {
		wake_size = RTL_WAKE_SIZE_CRC;
		wake_mask_size = RTL_WAKE_MASK_SIZE_CRC;
	}
	for (i = 0; i < wake_size; i++) {
		memset(tmp_str, 0, TMP_STR_LEN);
		snprintf(tmp_str, TMP_STR_LEN, "wake-mask%d", i);
		wake_mask = of_find_property(pdev->dev.of_node, tmp_str, NULL);
		if (!wake_mask)
			break;
		tp->wol_rule[i].mask_size = wake_mask->length;
		memcpy(&tp->wol_rule[i].mask[0], wake_mask->value,
		       (wake_mask->length > wake_mask_size) ?
		       wake_mask_size : wake_mask->length);

		snprintf(tmp_str, TMP_STR_LEN, "wake-crc%d", i);
		if (of_property_read_u32(pdev->dev.of_node, tmp_str, &tmp))
			break;
		tp->wol_rule[i].crc = tmp & 0xFFFF;
		if (tp->chip->features & RTL_FEATURE_PAT_WAKE) {
			snprintf(tmp_str, TMP_STR_LEN, "wake-pattern%d", i);
			wake_pattern = of_find_property(pdev->dev.of_node, tmp_str, NULL);
			if (!wake_pattern)
				break;
			tmp = (wake_pattern->length > RTL_WAKE_PATTERN_SIZE) ?
				RTL_WAKE_PATTERN_SIZE : wake_pattern->length;
			tp->wol_rule[i].pattern_size =
				rtl_cp_reduced_pattern(tp, i, wake_pattern->value, tmp);

			snprintf(tmp_str, TMP_STR_LEN, "wake-offset%d", i);
			if (of_property_read_u32(pdev->dev.of_node, tmp_str, &tmp))
				break;
			tp->wol_rule[i].offset = tmp & 0xFFFF;
		}
		tp->wol_crc_cnt += 1;
		tp->wol_rule[i].flag |= WAKE_FLAG_ENABLE;
	}

	rtl_init_rxcfg(tp);

	rtl8169_irq_mask_and_ack(tp);

	rtl_hw_initialize(tp);

	rtl_hw_reset(tp);

	rtl_unlock_config_regs(tp);
	RTL_W8(tp, CONFIG1, RTL_R8(tp, CONFIG1) | PM_ENABLE);
	RTL_W8(tp, CONFIG5, RTL_R8(tp, CONFIG5) & PME_STATUS);

	/* disable magic packet WOL */
	RTL_W8(tp, CONFIG3, RTL_R8(tp, CONFIG3) & ~MAGIC_PKT);
	rtl_lock_config_regs(tp);

	if (tp->output_mode == OUTPUT_SGMII_TO_MAC ||
	    tp->output_mode == OUTPUT_RGMII_TO_MAC)
		tp->chip->link_ok = rtl8169_xmii_always_link_ok;

	mutex_init(&tp->wk.mutex);

	INIT_WORK(&tp->wk.work, rtl_task);

	rtl_init_mac_address(tp);

	rtl_led_set(tp);

	rtl_set_irq_mask(tp);

	if (tp->acp_enable) {
		tp->counters = devm_kzalloc(&pdev->dev, sizeof(*tp->counters), GFP_KERNEL);
		tp->counters_phys_addr = virt_to_phys(tp->counters);
	} else {
		tp->counters = dmam_alloc_coherent(&pdev->dev, sizeof(*tp->counters),
						   &tp->counters_phys_addr, GFP_KERNEL);
	}
	if (!tp->counters) {
		rc = -ENOMEM;
		goto err_out_mdio;
	}

	rc = r8169_mdio_register(tp);
	if (rc)
		goto err_out_mdio;

	ndev->ethtool_ops = &rtl8169_ethtool_ops;
	ndev->watchdog_timeo = RTL8169_TX_TIMEOUT;

	netif_napi_add(ndev, &tp->napi, rtl8169_poll);

	ndev->hw_features = NETIF_F_SG | NETIF_F_IP_CSUM | NETIF_F_TSO |
		NETIF_F_RXCSUM | NETIF_F_HW_VLAN_CTAG_TX |
		NETIF_F_HW_VLAN_CTAG_RX;

	ndev->features |= ndev->hw_features;

	ndev->vlan_features = NETIF_F_SG | NETIF_F_IP_CSUM | NETIF_F_TSO |
		NETIF_F_HIGHDMA;

	ndev->hw_features |= NETIF_F_RXALL;
	ndev->hw_features |= NETIF_F_RXFCS;

	ndev->gro_flush_timeout = 400000;

	/* configure chip for default features */
	__rtl8169_set_features(tp, ndev->features);

	ndev->min_mtu = ETH_ZLEN;
	ndev->max_mtu = tp->chip->jumbo_max;

	rc = register_netdev(ndev);
	if (rc < 0)
		goto err_out_netdev2;

	platform_set_drvdata(pdev, tp);

	netdev_info(ndev, "%s, XID %08x IRQ %d\n",
		    tp->chip->name,
		    (u32)(RTL_R32(tp, TX_CONFIG) & 0x9cf0f8ff), ndev->irq);
	if (tp->chip->jumbo_max != JUMBO_1K) {
		netdev_info(ndev, "jumbo features [frames: %d bytes, tx checksumming: %s]\n",
			    tp->chip->jumbo_max,
			    tp->chip->jumbo_tx_csum ? "ok" : "ko");
	}

	device_set_wakeup_enable(&pdev->dev, tp->dev->wol_enabled);

#ifdef RTL_PROC
	rc = rtl_proc_file_register(tp);
	if (rc) {
		dev_err(&pdev->dev, "failed to register proc files\n");
		goto err_out_netdev2;
	}
#endif

	netif_carrier_off(ndev);

	pm_runtime_put_sync(&pdev->dev);

out:
	return rc;

err_out_netdev2:
	netif_napi_del(&tp->napi);

err_out_mdio:
#ifdef RTL_PROC
	do {
		rtl_proc_file_unregister(tp);

		if (!rtk_proc)
			break;

		remove_proc_entry(KBUILD_MODNAME, rtk_proc);
		remove_proc_entry(tp->dev->name, init_net.proc_net);

		rtk_proc = NULL;

	} while (0);
#endif

err_out_pm_list:
	rtk_pm_del_list(dev_param);

err_out_netdev:
err_out_iomap:
	iounmap(ioaddr);
	return rc;
}

static struct platform_driver rtl8169_soc_driver = {
	.probe		= rtl_init_one,
	.remove		= rtl_remove_one,
	.shutdown	= rtl_shutdown,
	.driver = {
		.name		= KBUILD_MODNAME,
		.owner		= THIS_MODULE,
		.pm		= pm_ptr(&rtl8169soc_pm_ops),
		.of_match_table = of_match_ptr(r8169soc_dt_ids),
	},
};

module_platform_driver(rtl8169_soc_driver);

