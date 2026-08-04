// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Motorcomm YT921x Switch Extended CPU Port Tagging
 *
 * Copyright (c) 2025 David Yang <mmyangfl@gmail.com>
 *
 * +----+----+-------+-----+----+---------
 * | DA | SA | TagET | Tag | ET | Payload ...
 * +----+----+-------+-----+----+---------
 *   6    6      2      6    2       N
 *
 * Tag Ethertype: CPU_TAG_TPID_TPID (default: ETH_P_YT921X = 0x9988)
 *   * Hardcoded for the moment, but still configurable. Discuss it if there
 *     are conflicts somewhere and/or you want to change it for some reason.
 * Tag:
 *   2: VLAN Tag
 *   2: Rx Port
 *     15b: Rx Port Valid
 *     14b-11b: Rx Port
 *     10b-0b: Cmd?
 *   2: Tx Port(s)
 *     15b: Tx Port(s) Valid
 *     10b-0b: Tx Port(s) Mask
 */

#include <linux/etherdevice.h>
#include "tag.h"

#define YT921X_TAG_NAME	"yt921x"
#define YT921X_TAG_LEN	8
#define YT921X_TAG_PORT_EN		BIT(15)
#define YT921X_TAG_RX_PORT_M		GENMASK(14, 11)
#define YT921X_TAG_PRIO_M		GENMASK(10, 8)
#define  YT921X_TAG_PRIO(x)			FIELD_PREP(YT921X_TAG_PRIO_M, (x))
#define YT921X_TAG_CODE_EN		BIT(7)
#define YT921X_TAG_CODE_M		GENMASK(6, 1)
#define  YT921X_TAG_CODE(x)			FIELD_PREP(YT921X_TAG_CODE_M, (x))
#define YT921X_TAG_TX_PORTS_M		GENMASK(10, 0)
#define  YT921X_TAG_TX_PORTn(port)	BIT(port)

enum yt921x_tag_code {
	YT921X_TAG_CODE_FORWARD = 0x00,
	YT921X_TAG_CODE_L2_CTRL = 0x18,
	YT921X_TAG_CODE_UNK_UCAST = 0x19,
	YT921X_TAG_CODE_UNK_MCAST = 0x1a,
	YT921X_TAG_CODE_PORT_COPY = 0x1b,
	YT921X_TAG_CODE_FDB_COPY = 0x1c,
};

static struct sk_buff *
yt921x_tag_xmit(struct sk_buff *skb, struct net_device *netdev)
{
	struct dsa_port *dp = dsa_slave_to_port(netdev);
	unsigned int port = dp->index;
	__be16 *tag;
	u16 tx;
	int nh_offset, th_offset;
	bool th_set;

	/* 
	 * 【终极修复 1：强制线性化】
	 * 解决 rk_gmac (stmmac) 驱动在 6.6 内核下处理多 frags 描述符链时
	 * DMA 引擎挂死的已知 Bug。将所有碎片合并到线性数据区。
	 */
	if (skb_is_nonlinear(skb)) {
		if (unlikely(skb_linearize(skb))) {
			kfree_skb(skb);
			return NULL;
		}
	}

	/* 防御性检查，确保 headroom 足够 */
	if (unlikely(skb_cow_head(skb, YT921X_TAG_LEN) < 0)) {
		kfree_skb(skb);
		return NULL;
	}

	/* 记录原始的 Header 偏移量 */
	nh_offset = skb_network_offset(skb);
	th_set = skb_transport_header_was_set(skb);
	if (th_set)
		th_offset = skb_transport_offset(skb);

	/* 插入 8 字节 Tag */
	skb_push(skb, YT921X_TAG_LEN);
	dsa_alloc_etype_header(skb, YT921X_TAG_LEN);

	/* 
	 * 【终极修复 2：修正 SKB Header 指针】
	 * 这是解决 "transmit queue timed out" 的核心！
	 * stmmac 在 TX 时依赖这些指针来定位 IP 头以配置 DMA 描述符。
	 * 如果不修正，stmmac 会读错位置（偏差 8 字节），生成非法 DMA 描述符，
	 * 导致 DMA 引擎 Bus Error 并彻底挂死。
	 */
	skb_reset_mac_header(skb);
	skb_set_network_header(skb, nh_offset + YT921X_TAG_LEN);
	if (th_set)
		skb_set_transport_header(skb, th_offset + YT921X_TAG_LEN);

	/* 
	 * 【终极修复 3：软件校验和】
	 * 必须在修正指针之后调用，确保能正确找到 IP/TCP 头。
	 * 避开 stmmac 硬件校验和引擎因为 Tag 偏移而算错的问题。
	 */
	if (skb->ip_summed == CHECKSUM_PARTIAL) {
		if (unlikely(skb_checksum_help(skb))) {
			kfree_skb(skb);
			return NULL;
		}
	}
	skb->ip_summed = CHECKSUM_NONE;

	tag = dsa_etype_header_pos_tx(skb);
	tag[0] = htons(ETH_P_YT921X);
	tag[1] = 0;
	tag[2] = htons(YT921X_TAG_CODE(YT921X_TAG_CODE_FORWARD) |
		       YT921X_TAG_CODE_EN |
		       YT921X_TAG_PRIO(skb->priority));
	tx = YT921X_TAG_PORT_EN | YT921X_TAG_TX_PORTn(port);
	tag[3] = htons(tx);

	return skb;
}

static struct sk_buff *
yt921x_tag_rcv(struct sk_buff *skb, struct net_device *netdev)
{
	unsigned int port;
	__be16 *tag;
	u16 code;
	u16 rx;

	if (unlikely(!pskb_may_pull(skb, YT921X_TAG_LEN))) {
		kfree_skb(skb);
		return NULL;
	}

	tag = dsa_etype_header_pos_rx(skb);
	if (unlikely(tag[0] != htons(ETH_P_YT921X))) {
		/* 彻底删除了 yt921x_conduit_is_raw 及 Secondary Conduit 逻辑，防止死锁 */
		kfree_skb(skb);
		return NULL;
	}

	rx = ntohs(tag[2]);
	if (unlikely((rx & YT921X_TAG_PORT_EN) == 0)) {
		dev_warn_ratelimited(&netdev->dev,
				     "Unexpected rx tag 0x%04x\n", rx);
		kfree_skb(skb);
		return NULL;
	}
	port = FIELD_GET(YT921X_TAG_RX_PORT_M, rx);
	skb->dev = dsa_master_find_slave(netdev, 0, port);
	if (unlikely(!skb->dev)) {
		dev_warn_ratelimited(&netdev->dev,
				     "Couldn't decode source port %u\n", port);
		kfree_skb(skb);
		return NULL;
	}

	skb->priority = FIELD_GET(YT921X_TAG_PRIO_M, rx);
	if (unlikely(!(rx & YT921X_TAG_CODE_EN))) {
		dev_warn_ratelimited(&netdev->dev,
				     "Tag code not enabled in rx packet\n");
		goto out_strip;
	}
	code = FIELD_GET(YT921X_TAG_CODE_M, rx);
	switch (code) {
	case YT921X_TAG_CODE_FORWARD:
	case YT921X_TAG_CODE_PORT_COPY:
	case YT921X_TAG_CODE_FDB_COPY:
		dsa_default_offload_fwd_mark(skb);
		break;
	case YT921X_TAG_CODE_L2_CTRL:
	case YT921X_TAG_CODE_UNK_UCAST:
	case YT921X_TAG_CODE_UNK_MCAST:
		break;
	default:
		dev_warn_ratelimited(&netdev->dev,
				     "Unknown tag code 0x%02x\n", code);
		break;
	}

out_strip:
	skb_pull_rcsum(skb, YT921X_TAG_LEN);
	dsa_strip_etype_header(skb, YT921X_TAG_LEN);
	return skb;
}

static const struct dsa_device_ops yt921x_netdev_ops = {
	.name	= YT921X_TAG_NAME,
	.proto	= DSA_TAG_PROTO_YT921X,
	.xmit	= yt921x_tag_xmit,
	.rcv	= yt921x_tag_rcv,
	.needed_headroom = YT921X_TAG_LEN,
};

MODULE_DESCRIPTION("DSA tag driver for Motorcomm YT921x switches");
MODULE_LICENSE("GPL");
MODULE_ALIAS_DSA_TAG_DRIVER(DSA_TAG_PROTO_YT921X, "yt921x");
module_dsa_tag_driver(yt921x_netdev_ops);
