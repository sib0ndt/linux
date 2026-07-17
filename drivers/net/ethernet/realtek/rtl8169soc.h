/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/* rtl8169soc.h: the embedded Realtek STB SoC 8169soc Ethernet driver.
 *
 * Copyright (c) 2002 Realtek Semiconductor Corp.
 * Copyright (c) 2002 ShuChen <shuchen@realtek.com.tw>
 * Copyright (c) 2003 - 2007 Francois Romieu <romieu@fr.zoreil.com>
 * Copyright (c) 2014 YuKuen Wu <yukuen@realtek.com>
 * Copyright (c) 2015 Eric Wang <ericwang@realtek.com>
 *
 * See MAINTAINERS file for support contact information.
 */

#ifndef __RTL8169SOC_H__
#define __RTL8169SOC_H__

#define CURRENT_MDIO_PAGE 0xFFFFFFFF

/* Maximum number of multicast addresses to filter (vs. Rx-all-multicast).
 * The RTL chips use a 64 element hash table based on the Ethernet CRC.
 */
#define	MC_FILTER_LIMIT	32

#define MAX_READ_REQUEST_SHIFT	12
#define TX_DMA_BURST	4	/* Maximum DMA burst, '7' is unlimited */
#define INTER_FRAME_GAP	0x03	/* 3 means INTER_FRAME_GAP = the shortest one */

#define R8169_REGS_SIZE		256
#define R8169_NAPI_WEIGHT	64
#define NUM_TX_DESC	1024U	/* Number of Tx descriptor registers */
#if defined(CONFIG_RTL_RX_NO_COPY)
#define NUM_RX_DESC	4096U	/* Number of Rx descriptor registers */
#else
#define NUM_RX_DESC	1024U	/* Number of Rx descriptor registers */
#endif /* CONFIG_RTL_RX_NO_COPY */
#define R8169_TX_RING_BYTES	(NUM_TX_DESC * sizeof(struct tx_desc))
#define R8169_RX_RING_BYTES	(NUM_RX_DESC * sizeof(struct rx_desc))

#define RTL8169_TX_TIMEOUT	(6 * HZ)
#define MAC_INIT_TIMEOUT	20
#define PHY_LOCK_TIMEOUT	1000
#define RTK_BUF_SIZE		512

#define RX_BUF_SIZE	0x05F3	/* 0x05F3 = 1522bye + 1 */
#define RTK_RX_ALIGN	8

#if IS_ENABLED(CONFIG_PROC_FS)
#define RTL_PROC 1
#endif

/* write/read MMIO register */
#define RTL_W8(tp, reg, val8)	writeb((val8), (tp)->mmio_addr + (reg))
#define RTL_W16(tp, reg, val16)	writew((val16), (tp)->mmio_addr + (reg))
#define RTL_W32(tp, reg, val32)	writel((val32), (tp)->mmio_addr + (reg))
#define RTL_R8(tp, reg)		readb((tp)->mmio_addr + (reg))
#define RTL_R16(tp, reg)	readw((tp)->mmio_addr + (reg))
#define RTL_R32(tp, reg)	readl((tp)->mmio_addr + (reg))

#define JUMBO_1K	ETH_DATA_LEN
#define JUMBO_9K	(9 * SZ_1K - VLAN_ETH_HLEN - ETH_FCS_LEN)

enum rtl_registers {
	MAC0				= 0,	/* Ethernet hardware address. */
	MAC4				= 4,
	MAR0				= 8,	/* Multicast filter. */
	COUNTER_ADDR_LOW		= 0x10,
	COUNTER_ADDR_HIGH		= 0x14,
	LEDSEL				= 0x18,
	TX_DESC_START_ADDR_LOW		= 0x20,
	TX_DESC_START_ADDR_HIGH		= 0x24,
	TXH_DESC_START_ADDR_LOW		= 0x28,
	TXH_DESC_START_ADDR_HIGH	= 0x2c,
	FLASH				= 0x30,
	ERSR				= 0x36,
	CHIP_CMD			= 0x37,
	TX_POLL				= 0x38,
	INTR_MASK			= 0x3c,
	INTR_STATUS			= 0x3e,

	TX_CONFIG			= 0x40,
#define	TXCFG_AUTO_FIFO			BIT(7)	/* 8111e-vl */
#define	TXCFG_EMPTY			BIT(11)	/* 8111e-vl */

	RX_CONFIG			= 0x44,
#define	RX128_INT_EN			BIT(15)	/* 8111c and later */
#define	RX_MULTI_EN			BIT(14)	/* 8111c only */
#define	RXCFG_FIFO_SHIFT		13
					/* No threshold before first PCI xfer */
#define	RX_FIFO_THRESH			(7 << RXCFG_FIFO_SHIFT)
#define	RX_EARLY_OFF			BIT(11)
#define	RXCFG_DMA_SHIFT			8
					/* Unlimited maximum PCI burst. */
#define	RX_DMA_BURST			(3 << RXCFG_DMA_SHIFT)	/* 128 bytes */

	RX_MISSED			= 0x4c,
	CFG9346				= 0x50,
	CONFIG0				= 0x51,
	CONFIG1				= 0x52,
	CONFIG2				= 0x53,
#define PME_SIGNAL			BIT(5)	/* 8168c and later */

	CONFIG3				= 0x54,
	CONFIG4				= 0x55,
	CONFIG5				= 0x56,
	MULTI_INTR			= 0x5c,
	PHYAR				= 0x60,
	PHY_STATUS			= 0x6c,
	RX_MAX_SIZE			= 0xda,
	C_PLUS_CMD			= 0xe0,
	INTR_MITIGATE			= 0xe2,
#define RTL_COALESCE_TX_USECS	GENMASK(15, 12)
#define RTL_COALESCE_TX_FRAMES	GENMASK(11, 8)
#define RTL_COALESCE_RX_USECS	GENMASK(7, 4)
#define RTL_COALESCE_RX_FRAMES	GENMASK(3, 0)

#define RTL_COALESCE_T_MAX	0x0fU
#define RTL_COALESCE_FRAME_MAX	(RTL_COALESCE_T_MAX * 4)
	RX_DESC_ADDR_LOW		= 0xe4,
	RX_DESC_ADDR_HIGH		= 0xe8,
	EARLY_TX_THRES			= 0xec,	/* 8169. Unit of 32 bytes. */

#define NO_EARLY_TX	0x3f	/* Max value : no early transmit. */

	MAX_TX_PACKET_SIZE		= 0xec,	/* Unit of 128 bytes. */

#define TX_PACKET_MAX	(8064 >> 7)
#define EARLY_SIZE	0x27

	FUNC_EVENT			= 0xf0,
	FUNC_EVENT_MASK			= 0xf4,
	FUNC_PRESET_STATE		= 0xf8,
	FUNC_FORCE_EVENT		= 0xfc,
};

enum rtl8168_8101_registers {
	CSIDR			= 0x64,
	CSIAR			= 0x68,
#define	CSIAR_FLAG			0x80000000
#define	CSIAR_WRITE_CMD			0x80000000
#define	CSIAR_BYTE_ENABLE		0x0f
#define	CSIAR_BYTE_ENABLE_SHIFT		12
#define	CSIAR_ADDR_MASK			0x0fff
#define CSIAR_FUNC_CARD			0x00000000
#define CSIAR_FUNC_SDIO			0x00010000
#define CSIAR_FUNC_NIC			0x00020000
	PMCH			= 0x6f,
	EPHYAR			= 0x80,
#define	EPHYAR_FLAG			0x80000000
#define	EPHYAR_WRITE_CMD		0x80000000
#define	EPHYAR_REG_MASK			0x1f
#define	EPHYAR_REG_SHIFT		16
#define	EPHYAR_DATA_MASK		0xffff
	DLLPR			= 0xd0,
#define	PFM_EN				BIT(6)
	DBG_REG			= 0xd1,
#define	FIX_NAK_1			BIT(4)
#define	FIX_NAK_2			BIT(3)
	TWSI			= 0xd2,
	MCU			= 0xd3,
#define	NOW_IS_OOB			BIT(7)
#define	TX_EMPTY			BIT(5)
#define	RX_EMPTY			BIT(4)
#define	RXTX_EMPTY			(TX_EMPTY | RX_EMPTY)
#define	EN_NDP				BIT(3)
#define	EN_OOB_RESET			BIT(2)
#define	LINK_LIST_RDY			BIT(1)
#define	DIS_MCU_CLROOB			BIT(0)
	EFUSEAR			= 0xdc,
#define	EFUSEAR_FLAG			0x80000000
#define	EFUSEAR_WRITE_CMD		0x80000000
#define	EFUSEAR_READ_CMD		0x00000000
#define	EFUSEAR_REG_MASK		0x03ff
#define	EFUSEAR_REG_SHIFT		8
#define	EFUSEAR_DATA_MASK		0xff
};

enum rtl8168_registers {
	LED_FREQ		= 0x1a,
	EEE_LED			= 0x1b,
	ERIDR			= 0x70,
	ERIAR			= 0x74,
#define ERIAR_FLAG			0x80000000
#define ERIAR_WRITE_CMD			0x80000000
#define ERIAR_READ_CMD			0x00000000
#define ERIAR_ADDR_BYTE_ALIGN		4
#define ERIAR_TYPE_SHIFT		16
#define ERIAR_EXGMAC			(0x00 << ERIAR_TYPE_SHIFT)
#define ERIAR_MSIX			(0x01 << ERIAR_TYPE_SHIFT)
#define ERIAR_ASF			(0x02 << ERIAR_TYPE_SHIFT)
#define ERIAR_MASK_SHIFT		12
#define ERIAR_MASK_0001			(0x1 << ERIAR_MASK_SHIFT)
#define ERIAR_MASK_0010			(0x2 << ERIAR_MASK_SHIFT)
#define ERIAR_MASK_0011			(0x3 << ERIAR_MASK_SHIFT)
#define ERIAR_MASK_0100			(0x4 << ERIAR_MASK_SHIFT)
#define ERIAR_MASK_0101			(0x5 << ERIAR_MASK_SHIFT)
#define ERIAR_MASK_1000			(0x8 << ERIAR_MASK_SHIFT)
#define ERIAR_MASK_1100			(0xc << ERIAR_MASK_SHIFT)
#define ERIAR_MASK_1111			(0xf << ERIAR_MASK_SHIFT)
	EPHY_RXER_NUM		= 0x7c,
	OCPDR			= 0xb0,	/* OCP GPHY access */
#define OCPDR_WRITE_CMD			0x80000000
#define OCPDR_READ_CMD			0x00000000
#define OCPDR_REG_MASK			0x7fff
#define OCPDR_REG_SHIFT			16
#define OCPDR_DATA_MASK			0xffff
	RDSAR1			= 0xd0,	/* 8168c only. Undocumented on 8168dp */
	MISC			= 0xf0,	/* 8168e only. */
#define TXPLA_RST			BIT(29)
#define DISABLE_LAN_EN			BIT(23) /* Enable GPIO pin */
#define PWM_EN				BIT(22)
#define RXDV_GATED_EN			BIT(19)
#define EARLY_TALLY_EN			BIT(16)
};

enum r8169soc_registers {
	TX_DESC_TAIL_IDX	= 0x20,	/* the last descriptor index */
	TX_DESC_CLOSE_IDX	= 0x22,	/* the closed descriptor index */
#define TX_DESC_CNT_MASK		0x3FFF
#define TX_DESC_CNT_SIZE		0x4000
};

enum rtl_register_content {
	/* InterruptStatusBits */
	SYS_ERR		= 0x8000,
	PCS_TIMEOUT	= 0x4000,
	SW_INT		= 0x0100,
	TX_DESC_UNAVAIL	= 0x0080,
	RX_FIFO_OVER	= 0x0040,
	LINK_CHG	= 0x0020,
	RX_OVERFLOW	= 0x0010,
	TX_ERR		= 0x0008,
	TX_OK		= 0x0004,
	RX_ERR		= 0x0002,
	RX_OK		= 0x0001,

	/* RxStatusDesc */
	RX_BOVF		= BIT(24),
	RX_FOVF		= BIT(23),
	RX_RWT		= BIT(22),
	RX_RES		= BIT(21),
	RX_RUNT		= BIT(20),
	RX_CRC		= BIT(19),

	/* ChipCmdBits */
	STOP_REQ	= 0x80,
	CMD_RESET	= 0x10,
	CMD_RX_ENB	= 0x08,
	CMD_TX_ENB	= 0x04,
	RX_BUF_EMPTY	= 0x01,

	/* TXPoll register p.5 */
	HPQ		= 0x80,		/* Poll cmd on the high prio queue */
	NPQ		= 0x40,		/* Poll cmd on the low prio queue */
	FSW_INT		= 0x01,		/* Forced software interrupt */

	/* Cfg9346Bits */
	CFG9346_LOCK	= 0x00,
	CFG9346_UNLOCK	= 0xc0,

	/* rx_mode_bits */
	ACCEPT_ERR		= 0x20,
	ACCEPT_RUNT		= 0x10,
#define RX_CONFIG_ACCEPT_ERR_MASK	0x30
	ACCEPT_BROADCAST	= 0x08,
	ACCEPT_MULTICAST	= 0x04,
	ACCEPT_MY_PHYS		= 0x02,
	ACCEPT_ALL_PHYS		= 0x01,
#define RX_CONFIG_ACCEPT_OK_MASK	0x0f
#define RX_CONFIG_ACCEPT_MASK		0x3f

	/* TxConfigBits */
	TX_INTER_FRAME_GAP_SHIFT = 24,
	TX_DMA_SHIFT = 8,	/* DMA burst value (0-7)
				 * is shift this many bits
				 */

	/* CONFIG1 register p.24 */
	LEDS1		= BIT(7),
	LEDS0		= BIT(6),
	SPEED_DOWN	= BIT(4),
	MEMMAP		= BIT(3),
	IOMAP		= BIT(2),
	VPD		= BIT(1),
	PM_ENABLE	= BIT(0),	/* Power Management Enable */

	/* CONFIG2 register p. 25 */
	CLK_REQ_EN	= BIT(7),	/* Clock Request Enable */
	MSI_ENABLE	= BIT(5),	/* 8169 only. Reserved in the 8168. */
	PCI_CLOCK_66MHZ = 0x01,
	PCI_CLOCK_33MHZ = 0x00,

	/* CONFIG3 register p.25 */
	MAGIC_PKT	= BIT(5),	/* Wake up when receives a Magic Pkt */
	LINK_UP		= BIT(4),	/* Wake up when the cable connection
					 * is re-established
					 */
	JUMBO_EN0	= BIT(2),	/* 8168 only. Reserved in the 8168b */
	BEACON_EN	= BIT(0),	/* 8168 only. Reserved in the 8168b */

	/* CONFIG4 register */
	JUMBO_EN1	= BIT(1),	/* 8168 only. Reserved in the 8168b */

	/* CONFIG5 register p.27 */
	BWF		= BIT(6),	/* Accept Broadcast wakeup frame */
	MWF		= BIT(5),	/* Accept Multicast wakeup frame */
	UWF		= BIT(4),	/* Accept Unicast wakeup frame */
	SPI_EN		= BIT(3),
	LAN_WAKE	= BIT(1),	/* LAN_WAKE enable/disable */
	PME_STATUS	= BIT(0),	/* PME status can be reset by PCI RST */
	ASPM_EN		= BIT(0),	/* ASPM enable */

	/* C_PLUS_CMD p.31 */
	ENABLE_BIST		= BIT(15),	/* 8168 8101 */
	MAC_DBGO_OE		= BIT(14),	/* 8168 8101 */
	NORMAL_MODE		= BIT(13),	/* unused */
	FORCE_HALF_DUP		= BIT(12),	/* 8168 8101 */
	FORCE_RXFLOW_EN		= BIT(11),	/* 8168 8101 */
	FORCE_TXFLOW_EN		= BIT(10),	/* 8168 8101 */
	CXPL_DBG_SEL		= BIT(9),	/* 8168 8101 */
	ASF			= BIT(8),	/* 8168 8101 */
	PKT_CNTR_DISABLE	= BIT(7),	/* 8168 8101 */
	MAC_DBGO_SEL		= 0x001c,	/* 8168 */
	RX_VLAN			= BIT(6),
	RX_CHK_SUM		= BIT(5),
	PCIDAC			= BIT(4),
	PCI_MUL_RW		= BIT(3),
	INTT_0			= 0x0000,	/* 8168 */
	INTT_1			= 0x0001,	/* 8168 */
	INTT_2			= 0x0002,	/* 8168 */
	INTT_3			= 0x0003,	/* 8168 */
#define INTT_MASK       GENMASK(1, 0)

	/* rtl8169_PHYstatus */
	PWR_SAVE_STATUS	= 0x80,
	TX_FLOW_CTRL	= 0x40,
	RX_FLOW_CTRL	= 0x20,
	_1000BPSF	= 0x10,
	_100BPS		= 0x08,
	_10BPS		= 0x04,
	LINK_STATUS	= 0x02,
	FULL_DUP	= 0x01,

	/* ResetCounterCommand */
	COUNTER_RESET	= 0x1,

	/* DumpCounterCommand */
	COUNTER_DUMP	= 0x8,
};

enum rtl_desc_bit {
	/* First doubleword. */
	DESC_OWN	= BIT(31), /* Descriptor is owned by NIC */
	RING_END	= BIT(30), /* End of descriptor ring */
	FIRST_FRAG	= BIT(29), /* First segment of a packet */
	LAST_FRAG	= BIT(28), /* Final segment of a packet */
};

/* Generic case. */
enum rtl_tx_desc_bit {
	/* First doubleword. */
	TD_LSO		= BIT(27),		/* Large Send Offload */
#define TD_MSS_MAX			0x07ffu	/* MSS value */

	/* Second doubleword. */
	TX_VLAN_TAG	= BIT(17),		/* Add VLAN tag */
};

enum rtl_tx_desc_bit_1 {
	/* Second doubleword. */
#define TD1_MSS_SHIFT			18	/* MSS position (11 bits) */
	TD1_IP_CS	= BIT(29),		/* Calculate IP checksum */
	TD1_TCP_CS	= BIT(30),		/* Calculate TCP/IP checksum */
	TD1_UDP_CS	= BIT(31),		/* Calculate UDP/IP checksum */
};

enum rtl_rx_desc_bit {
	/* Rx private */
	PID1		= BIT(18), /* Protocol ID bit 1/2 */
	PID0		= BIT(17), /* Protocol ID bit 2/2 */

#define RX_PROTO_UDP	(PID1)
#define RX_PROTO_TCP	(PID0)
#define RX_PROTO_IP	(PID1 | PID0)
#define RX_PROTO_MASK	RX_PROTO_IP

	IP_FAIL		= BIT(16), /* IP checksum failed */
	UDP_FAIL	= BIT(15), /* UDP/IP checksum failed */
	TCP_FAIL	= BIT(14), /* TCP/IP checksum failed */

#define RX_CS_FAIL_MASK	(IP_FAIL | UDP_FAIL | TCP_FAIL)

	RX_VLAN_TAG	= BIT(16), /* VLAN tag available */
};

#define RSVD_MASK	0x3fffc000

struct tx_desc {
	__le32 opts1;
	__le32 opts2;
	__le64 addr;
};

struct rx_desc {
	__le32 opts1;
	__le32 opts2;
	__le64 addr;
};

struct ring_info {
	struct sk_buff	*skb;
	u32		len;
	u8		__pad[sizeof(void *) - sizeof(u32)];
};

enum features {
	RTL_FEATURE_WOL		= BIT(0),
	RTL_FEATURE_GMII	= BIT(2),
	RTL_FEATURE_TX_NO_CLOSE	= BIT(3),
	RTL_FEATURE_RX_NO_COPY	= BIT(4),
	RTL_FEATURE_ADJUST_FIFO	= BIT(5),
	RTL_FEATURE_ACP		= BIT(6),
	RTL_FEATURE_EEE		= BIT(7),
	RTL_FEATURE_OCP_MDIO	= BIT(8),
	RTL_FEATURE_PAT_WAKE	= BIT(9),
	RTL_FEATURE_STORM_CTRL	= BIT(10),
	RTL_FEATURE_MDNS_OFFLOAD = BIT(11),
};

struct rtl8169_counters {
	__le64	tx_packets;
	__le64	rx_packets;
	__le64	tx_errors;
	__le32	rx_errors;
	__le16	rx_missed;
	__le16	align_errors;
	__le32	tx_one_collision;
	__le32	tx_multi_collision;
	__le64	rx_unicast;
	__le64	rx_broadcast;
	__le32	rx_multicast;
	__le16	tx_aborted;
	__le16	tx_underrun;
};

enum rtl_flag {
	RTL_FLAG_TASK_ENABLED = 0,
	RTL_FLAG_TASK_RESET_PENDING,
	RTL_FLAG_TASK_TX_TIMEOUT,
	RTL_FLAG_MAX
};

enum rtl_output_mode {
	OUTPUT_EMBEDDED_PHY,
	OUTPUT_RGMII_TO_MAC,
	OUTPUT_RGMII_TO_PHY,
	OUTPUT_SGMII_TO_MAC,
	OUTPUT_SGMII_TO_PHY,
	OUTPUT_RMII,
	OUTPUT_FORCE_LINK,
	OUTPUT_MAX
};

enum drv_status {
	RTL_STATUS_DOWN	= 0,
	RTL_STATUS_REINIT,
	RTL_STATUS_LOOPBACK,
	RTL_STATUS_HW_FAIL,
	RTL_STATUS_MAX
};

enum lb_mode {
	RTL_MAC_LOOPBACK = 0,
	RTL_INT_PHY_PCS_LOOPBACK,
	RTL_INT_PHY_REMOTE_LOOPBACK,
	RTL_EXT_PHY_PCS_LOOPBACK,
	RTL_EXT_PHY_REMOTE_LOOPBACK,
	RTL_LOOPBACK_MAX
};

/* ISO base addr 0x98007000 */
enum common_iso_registers {
	ISO_UMSK_ISR			= 0x0004,
	ISO_PWRCUT_ETN			= 0x005c,
	ISO_ETN_TESTIO			= 0x0060,
	ISO_PLL_WDOUT			= 0x0070,
	ISO_SOFT_RESET			= 0x0088,
	ISO_CLOCK_ENABLE		= 0x008c,
};

/* RTD119X */
enum rtd119x_iso_registers {
	RTD119X_ISO_MUXPAD0		= 0x0310,
};

/* RTD129X */
/* ISO base addr 0x98007000 */
enum rtd129x_iso_registers {
	RTD129X_ISO_RGMII_MDIO_TO_GMAC	= 0x0064,
	RTD129X_ISO_MUXPAD0		= 0x0310,
	RTD129X_ISO_DBUS_CTRL		= 0x0fc0,
};

/* SB2 base addr 0x9801a000 */
enum rtd129x_sb2_registers {
	RTD129X_SB2_PFUNC_RG0		= 0x0960,
	RTD129X_SB2_PFUNC_RG1		= 0x0964,
	RTD129X_SB2_PFUNC_RG2		= 0x0968,
};

enum rtd129x_rgmii_voltage {
	RTD129X_VOLTAGE_1_DOT_8V = 1,
	RTD129X_VOLTAGE_2_DOT_5V,
	RTD129X_VOLTAGE_3_DOT_3V,
	RTD129X_VOLTAGE_MAX
};

enum rtd129x_rgmii_delay {
	RTD129X_RGMII_DELAY_0NS,
	RTD129X_RGMII_DELAY_2NS,
	RTD129X_RGMII_DELAY_MAX
};

/* RTD139X */
#define RTD139X_R_K_DEFAULT		0x8
#define RTD139X_IDAC_FINE_DEFAULT	0x33

/* SDS base addr 0x981c8000 */
enum rtd139x_sds_registers {
	RTD139X_SDS_REG02	= 0x0008,
	RTD139X_SDS_REG28	= 0x0070,
	RTD139X_SDS_REG29	= 0x0074,
	RTD139X_SDS_MISC	= 0x1804,
	RTD139X_SDS_LINK	= 0x1810,
};

/* ISO testmux base addr 0x9804e000 */
enum rtd139x_testmux_registers {
	RTD139X_ISO_TESTMUX_MUXPAD0	= 0x0000,
	RTD139X_ISO_TESTMUX_MUXPAD1	= 0x0004,
	RTD139X_ISO_TESTMUX_MUXPAD2	= 0x0008,
};

/* SBX base addr 0x9801c000 */
enum rtd139x_sbx_registers {
	RTD139X_SBX_SB3_CHANNEL_REQ_MASK	= 0x020c,
	RTD139X_SBX_SB3_CHANNEL_REQ_BUSY	= 0x0210,
	RTD139X_SBX_ACP_CHANNEL_REQ_MASK	= 0x080c,
	RTD139X_SBX_ACP_CHANNEL_REQ_BUSY	= 0x0810,
	RTD139X_SBX_ACP_MISC_CTRL		= 0x0814,
};

/* SCPU_WRAPPER base addr 0x9801d000 */
enum rtd139x_sc_wrap_registers {
	RTD139X_SC_WRAP_ACP_CRT_CTRL		= 0x0030,
	RTD139X_SC_WRAP_CRT_CTRL		= 0x0100,
	RTD139X_SC_WRAP_INTERFACE_EN		= 0x0124,
	RTD139X_SC_WRAP_ACP_CTRL		= 0x0800,
};

enum rtd139x_phy_addr_e {
	/* embedded PHY ID */
	RTD139X_INT_PHY_ADDR		= 1,
	/* embedded SerDes DPHY ID 0, RL6481_T28_SGMII.doc ANA00~ANA0F */
	RTD139X_SERDES_DPHY_0		= 0,
	/* embedded SerDes DPHY ID 1, RL6481_T28_SGMII.doc ANA20~ANA2F */
	RTD139X_SERDES_DPHY_1		= 1,
	/* external RTL8211FS SGMII PHY ID */
	RTD139X_EXT_PHY_ADDR		= 3,
};

enum rtd139x_sgmii_swing_e {
	RTD139X_TX_SWING_1040MV		= (0X0 << 8),	/* DEFAULT */
	RTD139X_TX_SWING_693MV		= (0X1 << 8),
	RTD139X_TX_SWING_474MV		= (0X2 << 8),
	RTD139X_TX_SWING_352MV		= (0X3 << 8),
	RTD139X_TX_SWING_312MV		= (0X4 << 8),
};

#define RTD139X_SGMII_SWING		(0X3 << 8)

/* RTD16XX */
#define RTD16XX_RC_K_DEFAULT		0x8888
#define RTD16XX_R_K_DEFAULT		0x8888
#define RTD16XX_AMP_K_DEFAULT		0x7777
#define RTD16XX_ADC_BIAS_K_DEFAULT	0x8888

/* SDS base addr 0x981c8000 */
enum rtd16xx_sds_registers {
	RTD16XX_SDS_REG02	= 0x0008,
	RTD16XX_SDS_MISC	= 0x1804,
	RTD16XX_SDS_LINK	= 0x180c,
	RTD16XX_SDS_DEBUG	= 0x1810,
};

/* ISO testmux base addr 0x9804e000 */
enum rtd16xx_testmux_registers {
	RTD16XX_ISO_TESTMUX_MUXPAD0	= 0x0000,
	RTD16XX_ISO_TESTMUX_MUXPAD1	= 0x0004,
	RTD16XX_ISO_TESTMUX_MUXPAD2	= 0x0008,
};

/* SBX base addr 0x9801c000 */
enum rtd16xx_sbx_registers {
	RTD16XX_SBX_SB3_CHANNEL_REQ_MASK	= 0x020c,
	RTD16XX_SBX_SB3_CHANNEL_REQ_BUSY	= 0x0210,
	RTD16XX_SBX_ACP_CHANNEL_REQ_MASK	= 0x080c,
	RTD16XX_SBX_ACP_CHANNEL_REQ_BUSY	= 0x0810,
	RTD16XX_SBX_ACP_MISC_CTRL		= 0x0814,
};

/* SCPU_WRAPPER base addr 0x9801d000 */
enum rtd16xx_sc_wrap_registers {
	RTD16XX_SC_WRAP_ACP_CRT_CTRL		= 0x0030,
	RTD16XX_SC_WRAP_CRT_CTRL		= 0x0100,
	RTD16XX_SC_WRAP_INTERFACE_EN		= 0x0124,
	RTD16XX_SC_WRAP_ACP_CTRL		= 0x0800,
};

enum rtd16xx_phy_addr_e {
	/* embedded PHY ID */
	RTD16XX_INT_PHY_ADDR		= 1,
	/* embedded SerDes DPHY ID 0, RL6481_T28_SGMII.doc ANA00~ANA0F */
	RTD16XX_SERDES_DPHY_0		= 0,
	/* embedded SerDes DPHY ID 1, RL6481_T28_SGMII.doc ANA20~ANA2F */
	RTD16XX_SERDES_DPHY_1		= 1,
	/* external RTL8211FS SGMII PHY ID */
	RTD16XX_EXT_PHY_ADDR		= 3,
};

enum rtd16xx_sgmii_swing_e {
	RTD16XX_TX_SWING_550MV,
	RTD16XX_TX_SWING_380MV,
	RTD16XX_TX_SWING_250MV,
	RTD16XX_TX_SWING_190MV
};

/* RTD13XX */
#define RTD13XX_R_K_DEFAULT		0x8
#define RTD13XX_IDAC_FINE_DEFAULT	0x77

/* ISO testmux base addr 0x9804e000 */
enum rtd13xx_testmux_registers {
	RTD13XX_ISO_TESTMUX_MUXPAD0	= 0x0000,
	RTD13XX_ISO_TESTMUX_MUXPAD1	= 0x0004,
	RTD13XX_ISO_TESTMUX_MUXPAD2	= 0x0008,
	RTD13XX_ISO_TESTMUX_MUXPAD5	= 0x0014,
	RTD13XX_ISO_TESTMUX_MUXPAD6	= 0x0018,
	RTD13XX_ISO_TESTMUX_PFUNC9	= 0x0040, /* MDC/MDIO current */
	RTD13XX_ISO_TESTMUX_PFUNC20	= 0x006c, /* RGMII current */
	RTD13XX_ISO_TESTMUX_PFUNC21	= 0x0070, /* RGMII current */
	RTD13XX_ISO_TESTMUX_PFUNC25	= 0x0090, /* RGMII BIAS */
};

/* SBX base addr 0x9801c000 */
enum rtd13xx_sbx_registers {
	RTD13XX_SBX_SB3_CHANNEL_REQ_MASK	= 0x020c,
	RTD13XX_SBX_SB3_CHANNEL_REQ_BUSY	= 0x0210,
	RTD13XX_SBX_ACP_CHANNEL_REQ_MASK	= 0x080c,
	RTD13XX_SBX_ACP_CHANNEL_REQ_BUSY	= 0x0810,
	RTD13XX_SBX_ACP_MISC_CTRL		= 0x0814,
};

/* SCPU_WRAPPER base addr 0x9801d000 */
enum rtd13xx_sc_wrap_registers {
	RTD13XX_SC_WRAP_ACP_CRT_CTRL		= 0x0030,
	RTD13XX_SC_WRAP_CRT_CTRL		= 0x0100,
	RTD13XX_SC_WRAP_INTERFACE_EN		= 0x0124,
	RTD13XX_SC_WRAP_ACP_CTRL		= 0x0800,
};

enum rtd13xx_phy_addr_e {
	/* embedded PHY ID */
	RTD13XX_INT_PHY_ADDR		= 1,
	/* external PHY ID */
	RTD13XX_EXT_PHY_ADDR		= 1,
};

/* RTD16XXB */
#define RTD16XXB_R_K_DEFAULT		0x8
#define RTD16XXB_AMP_K_DEFAULT		0x7777
#define RTD16XXB_RC_K_DEFAULT		0x8888

/* ISO testmux base addr 0x9804e000 */
enum rtd16xxb_testmux_registers {
	RTD16XXB_ISO_TESTMUX_MUXPAD2	= 0x0008, /* LED */
	RTD16XXB_ISO_TESTMUX_PFUNC12	= 0x0050, /* MDC/MDIO current */
};

enum rtd16xxb_phy_addr_e {
	/* embedded PHY ID */
	RTD16XXB_INT_PHY_ADDR		= 1,
	/* external PHY ID */
	RTD16XXB_EXT_PHY_ADDR		= 1,
};

/* RTD13XXD */
#define RTD13XXD_RS_DEFAULT		0X3	/* TX */
#define RTD13XXD_RSW_DEFAULT		0X4	/* RX */
#define RTD13XXD_IDAC_FINE_MDI_DEFAULT	0X12	/* TX */
#define RTD13XXD_IDAC_FINE_MDIX_DEFAULT	0X12	/* RX */

/* ISO testmux base addr 0x9804e000 */
enum rtd13xxd_testmux_registers {
	RTD13XXD_ISO_TESTMUX_MUXPAD3	= 0x000c, /* LED */
	RTD13XXD_ISO_TESTMUX_PFUNC20	= 0x007c, /* ETN LED0 current */
	RTD13XXD_ISO_TESTMUX_PFUNC21	= 0x0080, /* ETN LED1 current */
};

enum rtd13xxd_phy_addr_e {
	/* embedded PHY ID */
	RTD13XXD_INT_PHY_ADDR		= 1,
	/* external PHY ID */
	RTD13XXD_EXT_PHY_ADDR		= 1,
};

/* RTD13XXE */
#define RTD13XXE_RS_DEFAULT		0X3	/* TX */
#define RTD13XXE_RSW_DEFAULT		0X4	/* RX */
#define RTD13XXE_IDAC_FINE_MDI_DEFAULT	0X12	/* TX */
#define RTD13XXE_IDAC_FINE_MDIX_DEFAULT	0X12	/* RX */

/* ISO testmux base addr 0x9804e000 */
enum rtd13xxe_testmux_registers {
	RTD13XXE_ISO_TESTMUX_MUXPAD2    = 0x0008, /* LED */
	RTD13XXE_ISO_TESTMUX_PFUNC7     = 0x0040, /* ETN LED0/LED1 current */
};

enum rtd13xxe_phy_addr_e {
	/* embedded PHY ID */
	RTD13XXE_INT_PHY_ADDR		= 1,
	/* external PHY ID */
	RTD13XXE_EXT_PHY_ADDR		= 1,
};

/* RTD1625 */
/* ISO testmux base addr 0x9804e000 */
enum rtd1625_testmux_registers {
	RTD1625_ISO_MUXPAD32	= 0x0128, /* PHY CTRL location */
	RTD1625_ISO_PFUNC42	= 0x0188, /* RGMII pad voltage selection */
	RTD1625_ISO_DBG_STATUS	= 0x01a0, /* RGMII voltage detection */
};

/* MAIN2 misc base addr 0x9804f000 */
enum rtd1625_m2tmx_registers {
	RTD1625_M2TMX_PWRCUT_ETN	= 0x0180,
	RTD1625_M2TMX_ETN_DBUS_CTRL	= 0x0188, /* DBUS clock gating */
	RTD1625_M2TMX_ETN_MISC		= 0x0194, /* init_autoload_done */
	RTD1625_M2TMX_PFUNC9		= 0x0238, /* RGMII pad driving */
	RTD1625_M2TMX_PFUNC10		= 0x023c, /* RGMII pad driving */
	RTD1625_M2TMX_PFUNC11		= 0x0240, /* RGMII pad driving */
	RTD1625_M2TMX_PFUNC12		= 0x0244, /* RGMII pad driving */
};

/* ISO sys base addr 0x98129000 */
enum rtd1625_iso_sys_registers {
	RTD1625_ISO_SYS_PWR_CTRL	= 0x0300, /* main2 power */
};

/* end of register locations per chip */

/* wol_enable
 * BIT 0: WoL enable
 * BIT 1: CRC match
 * BIT 2: WPD
 */
enum wol_flags {
	WOL_MAGIC			= 0x1,
	WOL_CRC_MATCH			= 0x2,
	WOL_WPD				= 0x4,
	WOL_MDNS_OFFLOAD		= 0x8,
};

#define WOL_BUF_LEN		128

/* CRC WAKE UP supports 16 rules */
/* EXACTLY PATTERN WAKE UP supports 32 rules */
#define RTL_WAKE_SIZE		32
#define RTL_WAKE_SIZE_CRC	16

/* CRC WAKE UP supports 128-bit mask (16 bytes) */
/* EXACTLY PATTERN WAKE UP supports 128-bit mask + 8-bit padding (17 bytes) */
#define RTL_WAKE_MASK_SIZE	17
#define RTL_WAKE_MASK_SIZE_CRC	16

#define RTL_WAKE_MASK_REG_SIZE	32
#define RTL_WAKE_PATTERN_SIZE	136
struct rtl_wake_rule_s {
	u8 flag;
		#define WAKE_FLAG_ENABLE	0x01
	u8 mask_size; /* 0 ~ 17, include padding */
	u8 pattern_size; /* 0 ~ 136, include padding */
	u16 crc; /* CRC-16 */
	u16 offset; /* 0 ~ 1535 */
	u8 mask[RTL_WAKE_MASK_SIZE];
	u8 pattern[RTL_WAKE_PATTERN_SIZE];
};

struct rtl8169_rgmii_info {
	u8 voltage; /* 1:1.8V, 2: 2.5V, 3:3.3V */
	u8 tx_delay; /* 0: 0ns, 1: 2ns */
	u8 rx_delay; /* 0: 0ns, 1: 2ns */
};

struct rtl8169_sgmii_info {
	u8 swing; /* 0:640mV, 1:380mV, 2:250mV, 3:190mV */
};

struct rtl8169_rmii_info {
	u8 voltage; /* 1:1.8V, 2: 2.5V, 3:3.3V */
	u8 tx_delay; /* 0: 0ns, 1: 2ns */
	u8 rx_delay; /* 0 ~ 7 x 0.5ns */
};

struct rtl8169_storm_ctrl {
	u16 type; /* 0:disable, 1:pkt_limit, 2:rate_limit */
	u16 limit;
};

enum rtl_pkt_type {
	RTL_BROADCAST_PKT,
	RTL_MULTICAST_PKT,
	RTL_UNKNOWN_PKT,
};

enum rtl_limit_type {
	RTL_NO_LIMIT,
	RTL_PKT_LIMIT,
	RTL_RATE_LIMIT,
};

struct rtl8169_private {
	void __iomem *mmio_addr;	/* memory map physical address */
	struct regmap *iso_base;
	struct regmap *sb2_base;
	struct regmap *sbx_base;
	struct regmap *scpu_wrap_base;
	struct regmap *pinctrl_base;
	struct regmap *sds_base;
	struct regmap *m2tmx_base;
	struct regmap *iso_sys_base;

	struct platform_device *pdev;
	struct net_device *dev;
	struct pinctrl *pc;
	struct phy_device *phydev;
	struct napi_struct napi;
	u32 cur_rx; /* Index into the Rx descriptor buffer of next Rx pkt. */
	u32 cur_tx; /* Index into the Tx descriptor buffer of next Rx pkt. */
	u32 dirty_tx;

#if defined(CONFIG_RTL_RX_NO_COPY)
	u32 dirty_rx;
#endif /* CONFIG_RTL_RX_NO_COPY */

	struct tx_desc *tx_desc_array;	/* 256-aligned Tx descriptor ring */
	struct rx_desc *rx_desc_array;	/* 256-aligned Rx descriptor ring */
	dma_addr_t tx_phy_addr;
	dma_addr_t rx_phy_addr;

	/* Rx data buffers */
	#if defined(CONFIG_RTL_RX_NO_COPY)
	struct sk_buff *rx_databuff[NUM_RX_DESC]; /* RTL_FEATURE_RX_NO_COPY */
	#else
	struct page *rx_databuff[NUM_RX_DESC];
	#endif /* CONFIG_RTL_RX_NO_COPY */

	struct ring_info tx_skb[NUM_TX_DESC];	/* Tx data buffers */
	u16 cp_cmd;
	u16 irq_mask;

#ifdef RTL_PROC
	struct proc_dir_entry *dir_dev;
#endif

	struct {
		DECLARE_BITMAP(flags, RTL_FLAG_MAX);
		struct mutex mutex; /* mutex for work */
		struct work_struct work;
	} wk;

	dma_addr_t counters_phys_addr;
	struct rtl8169_counters *counters;

	u8 mac_addr[ETH_ALEN];
	u8 wol_enable;
	u8 wol_crc_cnt;

	u32 saved_wolopts;
	struct rtl_wake_rule_s wol_rule[RTL_WAKE_SIZE];
	struct rtl_wake_rule_s wol_rule_buf;

	u32 led_cfg;

	struct r8169soc_chip_info *chip;
	enum rtl_output_mode output_mode;
	union {
		struct rtl8169_rgmii_info rgmii;
		struct rtl8169_sgmii_info sgmii;
		struct rtl8169_rmii_info rmii;
	};
	u8 ext_phy_id;		/* 0 ~ 31 */
	u8 phy_ctrl_loc;
	u16 force_speed;	/* 0: disable, 100: 100Mbps, 1000: 1Gbps */

	u32 bypass_enable:1;	/* 0: disable, 1: enable */
	u32 eee_enable:1;	/* 0: disable, 1: enable */
	u32 acp_enable:1;	/* 0: disable, 1: enable */
	u32 ext_phy:1;
	u32 pwr_saving:1;	/* power saving of suspend mode */
	u32 netif_is_running:1;
	u32 tc_inited:1;	/* tally counters is inited or not */

	int dco_flag;
	int amp_k_offset;

	struct rtl8169_storm_ctrl sc[3];
	DECLARE_BITMAP(status, RTL_STATUS_MAX);

	/* mDNS offload */
	struct mdns_proto_data_list mdns_data_list;
	struct mdns_offload_passthrough_list passthrough_list;
	u32 mdns_offload_state;
	u32 passthrough_behavior;
	u8 g_wol_ipv4_addr[4];
	u8 g_wol_ipv6_addr[16];
	u16 ocp_mar_bak[4];
	u16 ocp_rcr_0_bak;
};

#define REVISION_A00	0xA00
#define REVISION_A01	0xA01
#define REVISION_A02	0xA02
#define REVISION_B00	0xB00
#define REVISION_B01	0xB01
#define REVISION_B02	0xB02
#define REVISION_NONE	0xFFF

struct r8169soc_chip_info {
	const char *name;
	void (*mdio_write)(struct rtl8169_private *tp, int page, int reg, int value);
	int (*mdio_read)(struct rtl8169_private *tp, int page, int reg);
	void (*mmd_write)(struct rtl8169_private *tp, u32 dev_addr, u32 reg_addr, u32 value);
	u32 (*mmd_read)(struct rtl8169_private *tp, u32 dev_addr, u32 reg_addr);
	void (*mdio_lock)(struct rtl8169_private *tp);
	void (*mdio_unlock)(struct rtl8169_private *tp);
	u32 (*link_ok)(struct rtl8169_private *tp);
	void (*reset_phy_gmac)(struct rtl8169_private *tp);
	void (*acp_init)(struct rtl8169_private *tp);
	void (*pll_clock_init)(struct rtl8169_private *tp);
	void (*mdio_init)(struct rtl8169_private *tp);
	void (*mac_mcu_patch)(struct rtl8169_private *tp);
	void (*hw_phy_config)(struct rtl8169_private *tp);
	void (*wakeup_set)(struct rtl8169_private *tp, bool enable);
	void (*eee_set)(struct rtl8169_private *tp, bool enable);
	void (*led_set)(struct rtl8169_private *tp, bool enable);
	void (*dump_regs)(struct seq_file *m, struct rtl8169_private *tp);
	void (*dump_var)(struct seq_file *m, struct rtl8169_private *tp);

	u32 led_cfg;
	u32 features;
	u16 jumbo_max;
	bool jumbo_tx_csum;
};

#define SIOCDEVPRIVATE_RTLTOOL	(SIOCDEVPRIVATE + 1)
#define SIOCDEVPRIVATE_RTLMDNS  (SIOCDEVPRIVATE + 4)

enum rtl_tool_cmd {
	RTLTOOL_READ_WOL = 0,
	RTLTOOL_WRITE_WOL,

	RTLTOOL_READ_PWR_SAVING,
	RTLTOOL_WRITE_PWR_SAVING,

	RTLTOOL_READ_MAC,
	RTLTOOL_WRITE_MAC,

	RTLTOOL_READ_OCP,
	RTLTOOL_WRITE_OCP,

	RTLTOOL_READ_ERI,
	RTLTOOL_WRITE_ERI,

	RTLTOOL_READ_PHY,
	RTLTOOL_WRITE_PHY,

	RTLTOOL_READ_EEE,
	RTLTOOL_WRITE_EEE,

	RTLTOOL_READ_WAKE_MASK,
	RTLTOOL_WRITE_WAKE_MASK,

	RTLTOOL_READ_WAKE_CRC,
	RTLTOOL_WRITE_WAKE_CRC,

	RTLTOOL_READ_WAKE_OFFSET,
	RTLTOOL_WRITE_WAKE_OFFSET,

	RTLTOOL_READ_WAKE_PATTERN,
	RTLTOOL_WRITE_WAKE_PATTERN,

	RTLTOOL_READ_WAKE_IDX_EN,
	RTLTOOL_WRITE_WAKE_IDX_EN,

	RTLTOOL_READ_STORM_CTRL,
	RTLTOOL_WRITE_STORM_CTRL,

	RTLTOOL_REINIT_MAC,
	RTLTOOL_REINIT_PHY,
	RTLTOOL_WRITE_ETH_LED,
	RTLTOOL_DUMP_WAKE_RULE,
	RTLTOOL_TEST_LOOPBACK,

	RTLTOOL_INVALID
};

enum rtl_mdns_cmd {
	RTLMDNS_READ_PASSTHROUGH = 0,
	RTLMDNS_ADD_PASSTHROUGH,
	RTLMDNS_DEL_PASSTHROUGH,
	RTLMDNS_RESET_PASSTHROUGH,

	RTLMDNS_READ_PASSTHROUGH_BEHAVIOR,
	RTLMDNS_WRITE_PASSTHROUGH_BEHAVIOR,

	RTLMDNS_READ_PROTO_DATA,
	RTLMDNS_ADD_PROTO_DATA,
	RTLMDNS_DEL_PROTO_DATA,
	RTLMDNS_RESET_PROTO_DATA,

	RTLMDNS_READ_WOL_IPV4,
	RTLMDNS_WRITE_WOL_IPV4,

	RTLMDNS_READ_WOL_IPV6,
	RTLMDNS_WRITE_WOL_IPV6,

	RTLMDNS_READ_STATE,
	RTLMDNS_WRITE_STATE,

	RTLMDNS_INVALID
};

struct rtl_storm_ctrl_struct {
	u8 pkt_type;
	u8 limit_type;
	u16 limit;
};

struct rtl_phy_struct {
	u16 page;
	u16 addr;
	u16 val;
};

struct rtl_ioctl_struct {
	u32 cmd;
	short offset;
	u16 len;
	union {
		u32 data;
		struct rtl_storm_ctrl_struct sc;
		struct rtl_phy_struct phy;
		u8 ipv4[4];
		void __user *buf;
	};
};

#endif /* __RTL8169SOC_H__ */

