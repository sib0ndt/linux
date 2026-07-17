/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/* r8169soc_mdns_offload.h: the embedded Realtek STB SoC 8169soc Ethernet driver.
 *
 * Copyright (c) 2002 Realtek Semiconductor Corp.
 * Copyright (c) 2002 ShuChen <shuchen@realtek.com.tw>
 * Copyright (c) 2003 - 2007 Francois Romieu <romieu@fr.zoreil.com>
 * Copyright (c) 2014 YuKuen Wu <yukuen@realtek.com>
 * Copyright (c) 2015 Eric Wang <ericwang@realtek.com>
 *
 * See MAINTAINERS file for support contact information.
 */

#ifndef __RTK_MDNS_OFFLOAD_H__
#define __RTK_MDNS_OFFLOAD_H__

#define MDNS_MAX_PACKET_SIZE 512
#define MDNS_PACKET_MAX 16
#define MDNS_TYPE_MAX 8
#define MDNS_PASSTHROUGH_MAX 32
#define MDNS_QNAME_LEN_MAX 256
#define MAX_PORT_WAKE_SOURCE_CNT  16

#define OCP_RCR_0		0xc010 /* equal to IO RCR_0 */
#define OCP_FTR_MCU_CTRL	0xc0b4
#define OCP_CFG_EN_MAGIC	0xc0b6
#define OCP_PROXY_PORT_MATCH_SEL	0xc0c0
#define OCP_OOB_PORT_MATCH_EN	0xc0be
#define OCP_OOB_IPV4_0		0xcd08
#define OCP_NS0_OOB_LC0_IPV6_0	0xcd10
#define OCP_MAR0		0xcd00 /* equal to IO MAR0 */
#define OCP_NS0_MACID_0		0xcd70
#define OCP_L4_PORT_5		0xcdb2

#define MDNS_NAME_SIZE_MAX	256
#define MDNS_RR_SIZE_MAX	(MDNS_NAME_SIZE_MAX + 2 + 2)
#define SRAM_SIZE		8192
#define SARM_1KB_BOUNDARY	1024
#define SARM_PATCH_WORD_SIZE	2
#define SRAM_PATCH_ARRAY_MAX	(SRAM_SIZE / SARM_PATCH_WORD_SIZE)
#define SRAM_PATCH_PAGE_SIZE	(SARM_1KB_BOUNDARY / SARM_PATCH_WORD_SIZE)
#define SRAM_PATCH_PAGE_MASK	(SRAM_PATCH_PAGE_SIZE - 1)
#define SFF_START_ADDR		0x6000
#define SFF_END_ADDR		0x77FF
#define SFF_SIZE		(SFF_END_ADDR - SFF_START_ADDR)
#define SFF_PASSTHROUGH_START_ADDR	SFF_START_ADDR
#define SFF_PASSTHROUGH_END_ADDR	0x67FF
#define SFF_PASSTHROUGH_SIZE	(SFF_PASSTHROUGH_END_ADDR - SFF_PASSTHROUGH_START_ADDR)
#define SFF_RR_START_ADDR	0x7000
#define SFF_RAW_PKT_START_ADDR	0x7400
#define SFF_RR_SIZE		(SFF_RAW_PKT_START_ADDR - SFF_RR_START_ADDR)
#define SFF_RAW_PKT_SIZE	(SFF_END_ADDR - SFF_RAW_PKT_START_ADDR)
#define SFF_RAW_PKT_OPT_START_ADDR	0x7800
#define SFF_WAKEUP_REASON_ADDR		0x7bc0
#define SFF_PASSTHROUGH_BEHAVIOR_ADDR	0x7bc2
#define SFF_WAKEUP_PKT_LEN_ADDR		0x7bc8
#define SFF_PASSTHROUGH_OPT_START_ADDR	0x7bd0

#define TMP_BUF_SIZE	512

enum passthrough_behavior_type {
	/* Only the queries present in the passthrough list are forwarded
	 * to the system without any modification.
	 */
	PASSTHROUGH_LIST,
	/* All the queries are dropped. */
	DROP_ALL,
	/* All the queries are forwarded to the system without any modification */
	FORWARD_ALL
};

enum mdns_offload_state {
	MDNS_OFFLOAD_STATE_DISSABLE = 0,
	MDNS_OFFLOAD_STATE_ENABLE
};

/*struct*/
struct match_criteria {
	unsigned short qtype;
	unsigned short nameoffset;
};

struct mdns_proto_data {
	unsigned char packet[MDNS_MAX_PACKET_SIZE];
	unsigned int packet_size;
	unsigned int type_size;
	struct match_criteria type[MDNS_TYPE_MAX];
};

struct mdns_proto_data_list {
	unsigned int list_size;
	struct mdns_proto_data list[MDNS_PACKET_MAX];
};

struct mdns_offload_passthrough {
	unsigned int qname_len;
	unsigned char qname[MDNS_QNAME_LEN_MAX];
};

struct mdns_offload_passthrough_list {
	unsigned int list_size;
	struct mdns_offload_passthrough list[MDNS_PASSTHROUGH_MAX];
};

struct rtl8169_private;
int mdns_proc_file_register(struct rtl8169_private *tp);
void mdns_proc_file_unregister(struct rtl8169_private *tp);
void rtd1625_mdns_mac_mcu_patch(struct rtl8169_private *tp);
void rtl8169soc_mdns_mac_offload_set(struct rtl8169_private *tp);
void rtl8169soc_mdns_mac_offload_unset(struct rtl8169_private *tp);
void rtl8169soc_mdns_mac_offload_test(struct rtl8169_private *tp);

extern u16 rtd1625_isram[];
extern u32 rtd1625_isram_len;

#endif /* __RTK_MDNS_OFFLOAD_H__ */

