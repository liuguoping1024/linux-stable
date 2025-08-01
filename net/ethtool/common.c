// SPDX-License-Identifier: GPL-2.0-only

#include <linux/ethtool_netlink.h>
#include <linux/net_tstamp.h>
#include <linux/phy.h>
#include <linux/rtnetlink.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/phy_link_topology.h>
#include <net/netdev_queues.h>

#include "netlink.h"
#include "common.h"
#include "../core/dev.h"


const char netdev_features_strings[NETDEV_FEATURE_COUNT][ETH_GSTRING_LEN] = {
	[NETIF_F_SG_BIT] =               "tx-scatter-gather",
	[NETIF_F_IP_CSUM_BIT] =          "tx-checksum-ipv4",
	[NETIF_F_HW_CSUM_BIT] =          "tx-checksum-ip-generic",
	[NETIF_F_IPV6_CSUM_BIT] =        "tx-checksum-ipv6",
	[NETIF_F_HIGHDMA_BIT] =          "highdma",
	[NETIF_F_FRAGLIST_BIT] =         "tx-scatter-gather-fraglist",
	[NETIF_F_HW_VLAN_CTAG_TX_BIT] =  "tx-vlan-hw-insert",

	[NETIF_F_HW_VLAN_CTAG_RX_BIT] =  "rx-vlan-hw-parse",
	[NETIF_F_HW_VLAN_CTAG_FILTER_BIT] = "rx-vlan-filter",
	[NETIF_F_HW_VLAN_STAG_TX_BIT] =  "tx-vlan-stag-hw-insert",
	[NETIF_F_HW_VLAN_STAG_RX_BIT] =  "rx-vlan-stag-hw-parse",
	[NETIF_F_HW_VLAN_STAG_FILTER_BIT] = "rx-vlan-stag-filter",
	[NETIF_F_VLAN_CHALLENGED_BIT] =  "vlan-challenged",
	[NETIF_F_GSO_BIT] =              "tx-generic-segmentation",
	[NETIF_F_GRO_BIT] =              "rx-gro",
	[NETIF_F_GRO_HW_BIT] =           "rx-gro-hw",
	[NETIF_F_LRO_BIT] =              "rx-lro",

	[NETIF_F_TSO_BIT] =              "tx-tcp-segmentation",
	[NETIF_F_GSO_ROBUST_BIT] =       "tx-gso-robust",
	[NETIF_F_TSO_ECN_BIT] =          "tx-tcp-ecn-segmentation",
	[NETIF_F_GSO_ACCECN_BIT] =	 "tx-tcp-accecn-segmentation",
	[NETIF_F_TSO_MANGLEID_BIT] =	 "tx-tcp-mangleid-segmentation",
	[NETIF_F_TSO6_BIT] =             "tx-tcp6-segmentation",
	[NETIF_F_FSO_BIT] =              "tx-fcoe-segmentation",
	[NETIF_F_GSO_GRE_BIT] =		 "tx-gre-segmentation",
	[NETIF_F_GSO_GRE_CSUM_BIT] =	 "tx-gre-csum-segmentation",
	[NETIF_F_GSO_IPXIP4_BIT] =	 "tx-ipxip4-segmentation",
	[NETIF_F_GSO_IPXIP6_BIT] =	 "tx-ipxip6-segmentation",
	[NETIF_F_GSO_UDP_TUNNEL_BIT] =	 "tx-udp_tnl-segmentation",
	[NETIF_F_GSO_UDP_TUNNEL_CSUM_BIT] = "tx-udp_tnl-csum-segmentation",
	[NETIF_F_GSO_PARTIAL_BIT] =	 "tx-gso-partial",
	[NETIF_F_GSO_TUNNEL_REMCSUM_BIT] = "tx-tunnel-remcsum-segmentation",
	[NETIF_F_GSO_SCTP_BIT] =	 "tx-sctp-segmentation",
	[NETIF_F_GSO_ESP_BIT] =		 "tx-esp-segmentation",
	[NETIF_F_GSO_UDP_L4_BIT] =	 "tx-udp-segmentation",
	[NETIF_F_GSO_FRAGLIST_BIT] =	 "tx-gso-list",

	[NETIF_F_FCOE_CRC_BIT] =         "tx-checksum-fcoe-crc",
	[NETIF_F_SCTP_CRC_BIT] =        "tx-checksum-sctp",
	[NETIF_F_NTUPLE_BIT] =           "rx-ntuple-filter",
	[NETIF_F_RXHASH_BIT] =           "rx-hashing",
	[NETIF_F_RXCSUM_BIT] =           "rx-checksum",
	[NETIF_F_NOCACHE_COPY_BIT] =     "tx-nocache-copy",
	[NETIF_F_LOOPBACK_BIT] =         "loopback",
	[NETIF_F_RXFCS_BIT] =            "rx-fcs",
	[NETIF_F_RXALL_BIT] =            "rx-all",
	[NETIF_F_HW_L2FW_DOFFLOAD_BIT] = "l2-fwd-offload",
	[NETIF_F_HW_TC_BIT] =		 "hw-tc-offload",
	[NETIF_F_HW_ESP_BIT] =		 "esp-hw-offload",
	[NETIF_F_HW_ESP_TX_CSUM_BIT] =	 "esp-tx-csum-hw-offload",
	[NETIF_F_RX_UDP_TUNNEL_PORT_BIT] =	 "rx-udp_tunnel-port-offload",
	[NETIF_F_HW_TLS_RECORD_BIT] =	"tls-hw-record",
	[NETIF_F_HW_TLS_TX_BIT] =	 "tls-hw-tx-offload",
	[NETIF_F_HW_TLS_RX_BIT] =	 "tls-hw-rx-offload",
	[NETIF_F_GRO_FRAGLIST_BIT] =	 "rx-gro-list",
	[NETIF_F_HW_MACSEC_BIT] =	 "macsec-hw-offload",
	[NETIF_F_GRO_UDP_FWD_BIT] =	 "rx-udp-gro-forwarding",
	[NETIF_F_HW_HSR_TAG_INS_BIT] =	 "hsr-tag-ins-offload",
	[NETIF_F_HW_HSR_TAG_RM_BIT] =	 "hsr-tag-rm-offload",
	[NETIF_F_HW_HSR_FWD_BIT] =	 "hsr-fwd-offload",
	[NETIF_F_HW_HSR_DUP_BIT] =	 "hsr-dup-offload",
};

const char
rss_hash_func_strings[ETH_RSS_HASH_FUNCS_COUNT][ETH_GSTRING_LEN] = {
	[ETH_RSS_HASH_TOP_BIT] =	"toeplitz",
	[ETH_RSS_HASH_XOR_BIT] =	"xor",
	[ETH_RSS_HASH_CRC32_BIT] =	"crc32",
};

const char
tunable_strings[__ETHTOOL_TUNABLE_COUNT][ETH_GSTRING_LEN] = {
	[ETHTOOL_ID_UNSPEC]     = "Unspec",
	[ETHTOOL_RX_COPYBREAK]	= "rx-copybreak",
	[ETHTOOL_TX_COPYBREAK]	= "tx-copybreak",
	[ETHTOOL_PFC_PREVENTION_TOUT] = "pfc-prevention-tout",
	[ETHTOOL_TX_COPYBREAK_BUF_SIZE] = "tx-copybreak-buf-size",
};

const char
phy_tunable_strings[__ETHTOOL_PHY_TUNABLE_COUNT][ETH_GSTRING_LEN] = {
	[ETHTOOL_ID_UNSPEC]     = "Unspec",
	[ETHTOOL_PHY_DOWNSHIFT]	= "phy-downshift",
	[ETHTOOL_PHY_FAST_LINK_DOWN] = "phy-fast-link-down",
	[ETHTOOL_PHY_EDPD]	= "phy-energy-detect-power-down",
};

#define __LINK_MODE_NAME(speed, type, duplex) \
	#speed "base" #type "/" #duplex
#define __DEFINE_LINK_MODE_NAME(speed, type, duplex) \
	[ETHTOOL_LINK_MODE(speed, type, duplex)] = \
	__LINK_MODE_NAME(speed, type, duplex)
#define __DEFINE_SPECIAL_MODE_NAME(_mode, _name) \
	[ETHTOOL_LINK_MODE_ ## _mode ## _BIT] = _name

const char link_mode_names[][ETH_GSTRING_LEN] = {
	__DEFINE_LINK_MODE_NAME(10, T, Half),
	__DEFINE_LINK_MODE_NAME(10, T, Full),
	__DEFINE_LINK_MODE_NAME(100, T, Half),
	__DEFINE_LINK_MODE_NAME(100, T, Full),
	__DEFINE_LINK_MODE_NAME(1000, T, Half),
	__DEFINE_LINK_MODE_NAME(1000, T, Full),
	__DEFINE_SPECIAL_MODE_NAME(Autoneg, "Autoneg"),
	__DEFINE_SPECIAL_MODE_NAME(TP, "TP"),
	__DEFINE_SPECIAL_MODE_NAME(AUI, "AUI"),
	__DEFINE_SPECIAL_MODE_NAME(MII, "MII"),
	__DEFINE_SPECIAL_MODE_NAME(FIBRE, "FIBRE"),
	__DEFINE_SPECIAL_MODE_NAME(BNC, "BNC"),
	__DEFINE_LINK_MODE_NAME(10000, T, Full),
	__DEFINE_SPECIAL_MODE_NAME(Pause, "Pause"),
	__DEFINE_SPECIAL_MODE_NAME(Asym_Pause, "Asym_Pause"),
	__DEFINE_LINK_MODE_NAME(2500, X, Full),
	__DEFINE_SPECIAL_MODE_NAME(Backplane, "Backplane"),
	__DEFINE_LINK_MODE_NAME(1000, KX, Full),
	__DEFINE_LINK_MODE_NAME(10000, KX4, Full),
	__DEFINE_LINK_MODE_NAME(10000, KR, Full),
	__DEFINE_SPECIAL_MODE_NAME(10000baseR_FEC, "10000baseR_FEC"),
	__DEFINE_LINK_MODE_NAME(20000, MLD2, Full),
	__DEFINE_LINK_MODE_NAME(20000, KR2, Full),
	__DEFINE_LINK_MODE_NAME(40000, KR4, Full),
	__DEFINE_LINK_MODE_NAME(40000, CR4, Full),
	__DEFINE_LINK_MODE_NAME(40000, SR4, Full),
	__DEFINE_LINK_MODE_NAME(40000, LR4, Full),
	__DEFINE_LINK_MODE_NAME(56000, KR4, Full),
	__DEFINE_LINK_MODE_NAME(56000, CR4, Full),
	__DEFINE_LINK_MODE_NAME(56000, SR4, Full),
	__DEFINE_LINK_MODE_NAME(56000, LR4, Full),
	__DEFINE_LINK_MODE_NAME(25000, CR, Full),
	__DEFINE_LINK_MODE_NAME(25000, KR, Full),
	__DEFINE_LINK_MODE_NAME(25000, SR, Full),
	__DEFINE_LINK_MODE_NAME(50000, CR2, Full),
	__DEFINE_LINK_MODE_NAME(50000, KR2, Full),
	__DEFINE_LINK_MODE_NAME(100000, KR4, Full),
	__DEFINE_LINK_MODE_NAME(100000, SR4, Full),
	__DEFINE_LINK_MODE_NAME(100000, CR4, Full),
	__DEFINE_LINK_MODE_NAME(100000, LR4_ER4, Full),
	__DEFINE_LINK_MODE_NAME(50000, SR2, Full),
	__DEFINE_LINK_MODE_NAME(1000, X, Full),
	__DEFINE_LINK_MODE_NAME(10000, CR, Full),
	__DEFINE_LINK_MODE_NAME(10000, SR, Full),
	__DEFINE_LINK_MODE_NAME(10000, LR, Full),
	__DEFINE_LINK_MODE_NAME(10000, LRM, Full),
	__DEFINE_LINK_MODE_NAME(10000, ER, Full),
	__DEFINE_LINK_MODE_NAME(2500, T, Full),
	__DEFINE_LINK_MODE_NAME(5000, T, Full),
	__DEFINE_SPECIAL_MODE_NAME(FEC_NONE, "None"),
	__DEFINE_SPECIAL_MODE_NAME(FEC_RS, "RS"),
	__DEFINE_SPECIAL_MODE_NAME(FEC_BASER, "BASER"),
	__DEFINE_LINK_MODE_NAME(50000, KR, Full),
	__DEFINE_LINK_MODE_NAME(50000, SR, Full),
	__DEFINE_LINK_MODE_NAME(50000, CR, Full),
	__DEFINE_LINK_MODE_NAME(50000, LR_ER_FR, Full),
	__DEFINE_LINK_MODE_NAME(50000, DR, Full),
	__DEFINE_LINK_MODE_NAME(100000, KR2, Full),
	__DEFINE_LINK_MODE_NAME(100000, SR2, Full),
	__DEFINE_LINK_MODE_NAME(100000, CR2, Full),
	__DEFINE_LINK_MODE_NAME(100000, LR2_ER2_FR2, Full),
	__DEFINE_LINK_MODE_NAME(100000, DR2, Full),
	__DEFINE_LINK_MODE_NAME(200000, KR4, Full),
	__DEFINE_LINK_MODE_NAME(200000, SR4, Full),
	__DEFINE_LINK_MODE_NAME(200000, LR4_ER4_FR4, Full),
	__DEFINE_LINK_MODE_NAME(200000, DR4, Full),
	__DEFINE_LINK_MODE_NAME(200000, CR4, Full),
	__DEFINE_LINK_MODE_NAME(100, T1, Full),
	__DEFINE_LINK_MODE_NAME(1000, T1, Full),
	__DEFINE_LINK_MODE_NAME(400000, KR8, Full),
	__DEFINE_LINK_MODE_NAME(400000, SR8, Full),
	__DEFINE_LINK_MODE_NAME(400000, LR8_ER8_FR8, Full),
	__DEFINE_LINK_MODE_NAME(400000, DR8, Full),
	__DEFINE_LINK_MODE_NAME(400000, CR8, Full),
	__DEFINE_SPECIAL_MODE_NAME(FEC_LLRS, "LLRS"),
	__DEFINE_LINK_MODE_NAME(100000, KR, Full),
	__DEFINE_LINK_MODE_NAME(100000, SR, Full),
	__DEFINE_LINK_MODE_NAME(100000, LR_ER_FR, Full),
	__DEFINE_LINK_MODE_NAME(100000, DR, Full),
	__DEFINE_LINK_MODE_NAME(100000, CR, Full),
	__DEFINE_LINK_MODE_NAME(200000, KR2, Full),
	__DEFINE_LINK_MODE_NAME(200000, SR2, Full),
	__DEFINE_LINK_MODE_NAME(200000, LR2_ER2_FR2, Full),
	__DEFINE_LINK_MODE_NAME(200000, DR2, Full),
	__DEFINE_LINK_MODE_NAME(200000, CR2, Full),
	__DEFINE_LINK_MODE_NAME(400000, KR4, Full),
	__DEFINE_LINK_MODE_NAME(400000, SR4, Full),
	__DEFINE_LINK_MODE_NAME(400000, LR4_ER4_FR4, Full),
	__DEFINE_LINK_MODE_NAME(400000, DR4, Full),
	__DEFINE_LINK_MODE_NAME(400000, CR4, Full),
	__DEFINE_LINK_MODE_NAME(100, FX, Half),
	__DEFINE_LINK_MODE_NAME(100, FX, Full),
	__DEFINE_LINK_MODE_NAME(10, T1L, Full),
	__DEFINE_LINK_MODE_NAME(800000, CR8, Full),
	__DEFINE_LINK_MODE_NAME(800000, KR8, Full),
	__DEFINE_LINK_MODE_NAME(800000, DR8, Full),
	__DEFINE_LINK_MODE_NAME(800000, DR8_2, Full),
	__DEFINE_LINK_MODE_NAME(800000, SR8, Full),
	__DEFINE_LINK_MODE_NAME(800000, VR8, Full),
	__DEFINE_LINK_MODE_NAME(10, T1S, Full),
	__DEFINE_LINK_MODE_NAME(10, T1S, Half),
	__DEFINE_LINK_MODE_NAME(10, T1S_P2MP, Half),
	__DEFINE_LINK_MODE_NAME(10, T1BRR, Full),
	__DEFINE_LINK_MODE_NAME(200000, CR, Full),
	__DEFINE_LINK_MODE_NAME(200000, KR, Full),
	__DEFINE_LINK_MODE_NAME(200000, DR, Full),
	__DEFINE_LINK_MODE_NAME(200000, DR_2, Full),
	__DEFINE_LINK_MODE_NAME(200000, SR, Full),
	__DEFINE_LINK_MODE_NAME(200000, VR, Full),
	__DEFINE_LINK_MODE_NAME(400000, CR2, Full),
	__DEFINE_LINK_MODE_NAME(400000, KR2, Full),
	__DEFINE_LINK_MODE_NAME(400000, DR2, Full),
	__DEFINE_LINK_MODE_NAME(400000, DR2_2, Full),
	__DEFINE_LINK_MODE_NAME(400000, SR2, Full),
	__DEFINE_LINK_MODE_NAME(400000, VR2, Full),
	__DEFINE_LINK_MODE_NAME(800000, CR4, Full),
	__DEFINE_LINK_MODE_NAME(800000, KR4, Full),
	__DEFINE_LINK_MODE_NAME(800000, DR4, Full),
	__DEFINE_LINK_MODE_NAME(800000, DR4_2, Full),
	__DEFINE_LINK_MODE_NAME(800000, SR4, Full),
	__DEFINE_LINK_MODE_NAME(800000, VR4, Full),
};
static_assert(ARRAY_SIZE(link_mode_names) == __ETHTOOL_LINK_MODE_MASK_NBITS);

#define __LINK_MODE_LANES_CR		1
#define __LINK_MODE_LANES_CR2		2
#define __LINK_MODE_LANES_CR4		4
#define __LINK_MODE_LANES_CR8		8
#define __LINK_MODE_LANES_DR		1
#define __LINK_MODE_LANES_DR_2		1
#define __LINK_MODE_LANES_DR2		2
#define __LINK_MODE_LANES_DR2_2		2
#define __LINK_MODE_LANES_DR4		4
#define __LINK_MODE_LANES_DR4_2		4
#define __LINK_MODE_LANES_DR8		8
#define __LINK_MODE_LANES_KR		1
#define __LINK_MODE_LANES_KR2		2
#define __LINK_MODE_LANES_KR4		4
#define __LINK_MODE_LANES_KR8		8
#define __LINK_MODE_LANES_SR		1
#define __LINK_MODE_LANES_SR2		2
#define __LINK_MODE_LANES_SR4		4
#define __LINK_MODE_LANES_SR8		8
#define __LINK_MODE_LANES_ER		1
#define __LINK_MODE_LANES_KX		1
#define __LINK_MODE_LANES_KX4		4
#define __LINK_MODE_LANES_LR		1
#define __LINK_MODE_LANES_LR4		4
#define __LINK_MODE_LANES_LR4_ER4	4
#define __LINK_MODE_LANES_LR_ER_FR	1
#define __LINK_MODE_LANES_LR2_ER2_FR2	2
#define __LINK_MODE_LANES_LR4_ER4_FR4	4
#define __LINK_MODE_LANES_LR8_ER8_FR8	8
#define __LINK_MODE_LANES_LRM		1
#define __LINK_MODE_LANES_MLD2		2
#define __LINK_MODE_LANES_T		1
#define __LINK_MODE_LANES_T1		1
#define __LINK_MODE_LANES_X		1
#define __LINK_MODE_LANES_FX		1
#define __LINK_MODE_LANES_T1L		1
#define __LINK_MODE_LANES_T1S		1
#define __LINK_MODE_LANES_T1S_P2MP	1
#define __LINK_MODE_LANES_VR		1
#define __LINK_MODE_LANES_VR2		2
#define __LINK_MODE_LANES_VR4		4
#define __LINK_MODE_LANES_VR8		8
#define __LINK_MODE_LANES_DR8_2		8
#define __LINK_MODE_LANES_T1BRR		1

#define __DEFINE_LINK_MODE_PARAMS(_speed, _type, _duplex)	\
	[ETHTOOL_LINK_MODE(_speed, _type, _duplex)] = {		\
		.speed  = SPEED_ ## _speed, \
		.lanes  = __LINK_MODE_LANES_ ## _type, \
		.duplex	= __DUPLEX_ ## _duplex \
	}
#define __DUPLEX_Half DUPLEX_HALF
#define __DUPLEX_Full DUPLEX_FULL
#define __DEFINE_SPECIAL_MODE_PARAMS(_mode) \
	[ETHTOOL_LINK_MODE_ ## _mode ## _BIT] = { \
		.speed	= SPEED_UNKNOWN, \
		.lanes	= 0, \
		.duplex	= DUPLEX_UNKNOWN, \
	}

const struct link_mode_info link_mode_params[] = {
	__DEFINE_LINK_MODE_PARAMS(10, T, Half),
	__DEFINE_LINK_MODE_PARAMS(10, T, Full),
	__DEFINE_LINK_MODE_PARAMS(100, T, Half),
	__DEFINE_LINK_MODE_PARAMS(100, T, Full),
	__DEFINE_LINK_MODE_PARAMS(1000, T, Half),
	__DEFINE_LINK_MODE_PARAMS(1000, T, Full),
	__DEFINE_SPECIAL_MODE_PARAMS(Autoneg),
	__DEFINE_SPECIAL_MODE_PARAMS(TP),
	__DEFINE_SPECIAL_MODE_PARAMS(AUI),
	__DEFINE_SPECIAL_MODE_PARAMS(MII),
	__DEFINE_SPECIAL_MODE_PARAMS(FIBRE),
	__DEFINE_SPECIAL_MODE_PARAMS(BNC),
	__DEFINE_LINK_MODE_PARAMS(10000, T, Full),
	__DEFINE_SPECIAL_MODE_PARAMS(Pause),
	__DEFINE_SPECIAL_MODE_PARAMS(Asym_Pause),
	__DEFINE_LINK_MODE_PARAMS(2500, X, Full),
	__DEFINE_SPECIAL_MODE_PARAMS(Backplane),
	__DEFINE_LINK_MODE_PARAMS(1000, KX, Full),
	__DEFINE_LINK_MODE_PARAMS(10000, KX4, Full),
	__DEFINE_LINK_MODE_PARAMS(10000, KR, Full),
	[ETHTOOL_LINK_MODE_10000baseR_FEC_BIT] = {
		.speed	= SPEED_10000,
		.lanes	= 1,
		.duplex = DUPLEX_FULL,
	},
	__DEFINE_LINK_MODE_PARAMS(20000, MLD2, Full),
	__DEFINE_LINK_MODE_PARAMS(20000, KR2, Full),
	__DEFINE_LINK_MODE_PARAMS(40000, KR4, Full),
	__DEFINE_LINK_MODE_PARAMS(40000, CR4, Full),
	__DEFINE_LINK_MODE_PARAMS(40000, SR4, Full),
	__DEFINE_LINK_MODE_PARAMS(40000, LR4, Full),
	__DEFINE_LINK_MODE_PARAMS(56000, KR4, Full),
	__DEFINE_LINK_MODE_PARAMS(56000, CR4, Full),
	__DEFINE_LINK_MODE_PARAMS(56000, SR4, Full),
	__DEFINE_LINK_MODE_PARAMS(56000, LR4, Full),
	__DEFINE_LINK_MODE_PARAMS(25000, CR, Full),
	__DEFINE_LINK_MODE_PARAMS(25000, KR, Full),
	__DEFINE_LINK_MODE_PARAMS(25000, SR, Full),
	__DEFINE_LINK_MODE_PARAMS(50000, CR2, Full),
	__DEFINE_LINK_MODE_PARAMS(50000, KR2, Full),
	__DEFINE_LINK_MODE_PARAMS(100000, KR4, Full),
	__DEFINE_LINK_MODE_PARAMS(100000, SR4, Full),
	__DEFINE_LINK_MODE_PARAMS(100000, CR4, Full),
	__DEFINE_LINK_MODE_PARAMS(100000, LR4_ER4, Full),
	__DEFINE_LINK_MODE_PARAMS(50000, SR2, Full),
	__DEFINE_LINK_MODE_PARAMS(1000, X, Full),
	__DEFINE_LINK_MODE_PARAMS(10000, CR, Full),
	__DEFINE_LINK_MODE_PARAMS(10000, SR, Full),
	__DEFINE_LINK_MODE_PARAMS(10000, LR, Full),
	__DEFINE_LINK_MODE_PARAMS(10000, LRM, Full),
	__DEFINE_LINK_MODE_PARAMS(10000, ER, Full),
	__DEFINE_LINK_MODE_PARAMS(2500, T, Full),
	__DEFINE_LINK_MODE_PARAMS(5000, T, Full),
	__DEFINE_SPECIAL_MODE_PARAMS(FEC_NONE),
	__DEFINE_SPECIAL_MODE_PARAMS(FEC_RS),
	__DEFINE_SPECIAL_MODE_PARAMS(FEC_BASER),
	__DEFINE_LINK_MODE_PARAMS(50000, KR, Full),
	__DEFINE_LINK_MODE_PARAMS(50000, SR, Full),
	__DEFINE_LINK_MODE_PARAMS(50000, CR, Full),
	__DEFINE_LINK_MODE_PARAMS(50000, LR_ER_FR, Full),
	__DEFINE_LINK_MODE_PARAMS(50000, DR, Full),
	__DEFINE_LINK_MODE_PARAMS(100000, KR2, Full),
	__DEFINE_LINK_MODE_PARAMS(100000, SR2, Full),
	__DEFINE_LINK_MODE_PARAMS(100000, CR2, Full),
	__DEFINE_LINK_MODE_PARAMS(100000, LR2_ER2_FR2, Full),
	__DEFINE_LINK_MODE_PARAMS(100000, DR2, Full),
	__DEFINE_LINK_MODE_PARAMS(200000, KR4, Full),
	__DEFINE_LINK_MODE_PARAMS(200000, SR4, Full),
	__DEFINE_LINK_MODE_PARAMS(200000, LR4_ER4_FR4, Full),
	__DEFINE_LINK_MODE_PARAMS(200000, DR4, Full),
	__DEFINE_LINK_MODE_PARAMS(200000, CR4, Full),
	__DEFINE_LINK_MODE_PARAMS(100, T1, Full),
	__DEFINE_LINK_MODE_PARAMS(1000, T1, Full),
	__DEFINE_LINK_MODE_PARAMS(400000, KR8, Full),
	__DEFINE_LINK_MODE_PARAMS(400000, SR8, Full),
	__DEFINE_LINK_MODE_PARAMS(400000, LR8_ER8_FR8, Full),
	__DEFINE_LINK_MODE_PARAMS(400000, DR8, Full),
	__DEFINE_LINK_MODE_PARAMS(400000, CR8, Full),
	__DEFINE_SPECIAL_MODE_PARAMS(FEC_LLRS),
	__DEFINE_LINK_MODE_PARAMS(100000, KR, Full),
	__DEFINE_LINK_MODE_PARAMS(100000, SR, Full),
	__DEFINE_LINK_MODE_PARAMS(100000, LR_ER_FR, Full),
	__DEFINE_LINK_MODE_PARAMS(100000, DR, Full),
	__DEFINE_LINK_MODE_PARAMS(100000, CR, Full),
	__DEFINE_LINK_MODE_PARAMS(200000, KR2, Full),
	__DEFINE_LINK_MODE_PARAMS(200000, SR2, Full),
	__DEFINE_LINK_MODE_PARAMS(200000, LR2_ER2_FR2, Full),
	__DEFINE_LINK_MODE_PARAMS(200000, DR2, Full),
	__DEFINE_LINK_MODE_PARAMS(200000, CR2, Full),
	__DEFINE_LINK_MODE_PARAMS(400000, KR4, Full),
	__DEFINE_LINK_MODE_PARAMS(400000, SR4, Full),
	__DEFINE_LINK_MODE_PARAMS(400000, LR4_ER4_FR4, Full),
	__DEFINE_LINK_MODE_PARAMS(400000, DR4, Full),
	__DEFINE_LINK_MODE_PARAMS(400000, CR4, Full),
	__DEFINE_LINK_MODE_PARAMS(100, FX, Half),
	__DEFINE_LINK_MODE_PARAMS(100, FX, Full),
	__DEFINE_LINK_MODE_PARAMS(10, T1L, Full),
	__DEFINE_LINK_MODE_PARAMS(800000, CR8, Full),
	__DEFINE_LINK_MODE_PARAMS(800000, KR8, Full),
	__DEFINE_LINK_MODE_PARAMS(800000, DR8, Full),
	__DEFINE_LINK_MODE_PARAMS(800000, DR8_2, Full),
	__DEFINE_LINK_MODE_PARAMS(800000, SR8, Full),
	__DEFINE_LINK_MODE_PARAMS(800000, VR8, Full),
	__DEFINE_LINK_MODE_PARAMS(10, T1S, Full),
	__DEFINE_LINK_MODE_PARAMS(10, T1S, Half),
	__DEFINE_LINK_MODE_PARAMS(10, T1S_P2MP, Half),
	__DEFINE_LINK_MODE_PARAMS(10, T1BRR, Full),
	__DEFINE_LINK_MODE_PARAMS(200000, CR, Full),
	__DEFINE_LINK_MODE_PARAMS(200000, KR, Full),
	__DEFINE_LINK_MODE_PARAMS(200000, DR, Full),
	__DEFINE_LINK_MODE_PARAMS(200000, DR_2, Full),
	__DEFINE_LINK_MODE_PARAMS(200000, SR, Full),
	__DEFINE_LINK_MODE_PARAMS(200000, VR, Full),
	__DEFINE_LINK_MODE_PARAMS(400000, CR2, Full),
	__DEFINE_LINK_MODE_PARAMS(400000, KR2, Full),
	__DEFINE_LINK_MODE_PARAMS(400000, DR2, Full),
	__DEFINE_LINK_MODE_PARAMS(400000, DR2_2, Full),
	__DEFINE_LINK_MODE_PARAMS(400000, SR2, Full),
	__DEFINE_LINK_MODE_PARAMS(400000, VR2, Full),
	__DEFINE_LINK_MODE_PARAMS(800000, CR4, Full),
	__DEFINE_LINK_MODE_PARAMS(800000, KR4, Full),
	__DEFINE_LINK_MODE_PARAMS(800000, DR4, Full),
	__DEFINE_LINK_MODE_PARAMS(800000, DR4_2, Full),
	__DEFINE_LINK_MODE_PARAMS(800000, SR4, Full),
	__DEFINE_LINK_MODE_PARAMS(800000, VR4, Full),
};
static_assert(ARRAY_SIZE(link_mode_params) == __ETHTOOL_LINK_MODE_MASK_NBITS);
EXPORT_SYMBOL_GPL(link_mode_params);

const char netif_msg_class_names[][ETH_GSTRING_LEN] = {
	[NETIF_MSG_DRV_BIT]		= "drv",
	[NETIF_MSG_PROBE_BIT]		= "probe",
	[NETIF_MSG_LINK_BIT]		= "link",
	[NETIF_MSG_TIMER_BIT]		= "timer",
	[NETIF_MSG_IFDOWN_BIT]		= "ifdown",
	[NETIF_MSG_IFUP_BIT]		= "ifup",
	[NETIF_MSG_RX_ERR_BIT]		= "rx_err",
	[NETIF_MSG_TX_ERR_BIT]		= "tx_err",
	[NETIF_MSG_TX_QUEUED_BIT]	= "tx_queued",
	[NETIF_MSG_INTR_BIT]		= "intr",
	[NETIF_MSG_TX_DONE_BIT]		= "tx_done",
	[NETIF_MSG_RX_STATUS_BIT]	= "rx_status",
	[NETIF_MSG_PKTDATA_BIT]		= "pktdata",
	[NETIF_MSG_HW_BIT]		= "hw",
	[NETIF_MSG_WOL_BIT]		= "wol",
};
static_assert(ARRAY_SIZE(netif_msg_class_names) == NETIF_MSG_CLASS_COUNT);

const char wol_mode_names[][ETH_GSTRING_LEN] = {
	[const_ilog2(WAKE_PHY)]		= "phy",
	[const_ilog2(WAKE_UCAST)]	= "ucast",
	[const_ilog2(WAKE_MCAST)]	= "mcast",
	[const_ilog2(WAKE_BCAST)]	= "bcast",
	[const_ilog2(WAKE_ARP)]		= "arp",
	[const_ilog2(WAKE_MAGIC)]	= "magic",
	[const_ilog2(WAKE_MAGICSECURE)]	= "magicsecure",
	[const_ilog2(WAKE_FILTER)]	= "filter",
};
static_assert(ARRAY_SIZE(wol_mode_names) == WOL_MODE_COUNT);

const char sof_timestamping_names[][ETH_GSTRING_LEN] = {
	[const_ilog2(SOF_TIMESTAMPING_TX_HARDWARE)]  = "hardware-transmit",
	[const_ilog2(SOF_TIMESTAMPING_TX_SOFTWARE)]  = "software-transmit",
	[const_ilog2(SOF_TIMESTAMPING_RX_HARDWARE)]  = "hardware-receive",
	[const_ilog2(SOF_TIMESTAMPING_RX_SOFTWARE)]  = "software-receive",
	[const_ilog2(SOF_TIMESTAMPING_SOFTWARE)]     = "software-system-clock",
	[const_ilog2(SOF_TIMESTAMPING_SYS_HARDWARE)] = "hardware-legacy-clock",
	[const_ilog2(SOF_TIMESTAMPING_RAW_HARDWARE)] = "hardware-raw-clock",
	[const_ilog2(SOF_TIMESTAMPING_OPT_ID)]       = "option-id",
	[const_ilog2(SOF_TIMESTAMPING_TX_SCHED)]     = "sched-transmit",
	[const_ilog2(SOF_TIMESTAMPING_TX_ACK)]       = "ack-transmit",
	[const_ilog2(SOF_TIMESTAMPING_OPT_CMSG)]     = "option-cmsg",
	[const_ilog2(SOF_TIMESTAMPING_OPT_TSONLY)]   = "option-tsonly",
	[const_ilog2(SOF_TIMESTAMPING_OPT_STATS)]    = "option-stats",
	[const_ilog2(SOF_TIMESTAMPING_OPT_PKTINFO)]  = "option-pktinfo",
	[const_ilog2(SOF_TIMESTAMPING_OPT_TX_SWHW)]  = "option-tx-swhw",
	[const_ilog2(SOF_TIMESTAMPING_BIND_PHC)]     = "bind-phc",
	[const_ilog2(SOF_TIMESTAMPING_OPT_ID_TCP)]   = "option-id-tcp",
	[const_ilog2(SOF_TIMESTAMPING_OPT_RX_FILTER)] = "option-rx-filter",
	[const_ilog2(SOF_TIMESTAMPING_TX_COMPLETION)] = "tx-completion",
};
static_assert(ARRAY_SIZE(sof_timestamping_names) == __SOF_TIMESTAMPING_CNT);

const char ts_tx_type_names[][ETH_GSTRING_LEN] = {
	[HWTSTAMP_TX_OFF]		= "off",
	[HWTSTAMP_TX_ON]		= "on",
	[HWTSTAMP_TX_ONESTEP_SYNC]	= "onestep-sync",
	[HWTSTAMP_TX_ONESTEP_P2P]	= "onestep-p2p",
};
static_assert(ARRAY_SIZE(ts_tx_type_names) == __HWTSTAMP_TX_CNT);

const char ts_rx_filter_names[][ETH_GSTRING_LEN] = {
	[HWTSTAMP_FILTER_NONE]			= "none",
	[HWTSTAMP_FILTER_ALL]			= "all",
	[HWTSTAMP_FILTER_SOME]			= "some",
	[HWTSTAMP_FILTER_PTP_V1_L4_EVENT]	= "ptpv1-l4-event",
	[HWTSTAMP_FILTER_PTP_V1_L4_SYNC]	= "ptpv1-l4-sync",
	[HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ]	= "ptpv1-l4-delay-req",
	[HWTSTAMP_FILTER_PTP_V2_L4_EVENT]	= "ptpv2-l4-event",
	[HWTSTAMP_FILTER_PTP_V2_L4_SYNC]	= "ptpv2-l4-sync",
	[HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ]	= "ptpv2-l4-delay-req",
	[HWTSTAMP_FILTER_PTP_V2_L2_EVENT]	= "ptpv2-l2-event",
	[HWTSTAMP_FILTER_PTP_V2_L2_SYNC]	= "ptpv2-l2-sync",
	[HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ]	= "ptpv2-l2-delay-req",
	[HWTSTAMP_FILTER_PTP_V2_EVENT]		= "ptpv2-event",
	[HWTSTAMP_FILTER_PTP_V2_SYNC]		= "ptpv2-sync",
	[HWTSTAMP_FILTER_PTP_V2_DELAY_REQ]	= "ptpv2-delay-req",
	[HWTSTAMP_FILTER_NTP_ALL]		= "ntp-all",
};
static_assert(ARRAY_SIZE(ts_rx_filter_names) == __HWTSTAMP_FILTER_CNT);

const char ts_flags_names[][ETH_GSTRING_LEN] = {
	[const_ilog2(HWTSTAMP_FLAG_BONDED_PHC_INDEX)] = "bonded-phc-index",
};
static_assert(ARRAY_SIZE(ts_flags_names) == __HWTSTAMP_FLAG_CNT);

const char udp_tunnel_type_names[][ETH_GSTRING_LEN] = {
	[ETHTOOL_UDP_TUNNEL_TYPE_VXLAN]		= "vxlan",
	[ETHTOOL_UDP_TUNNEL_TYPE_GENEVE]	= "geneve",
	[ETHTOOL_UDP_TUNNEL_TYPE_VXLAN_GPE]	= "vxlan-gpe",
};
static_assert(ARRAY_SIZE(udp_tunnel_type_names) ==
	      __ETHTOOL_UDP_TUNNEL_TYPE_CNT);

/* return false if legacy contained non-0 deprecated fields
 * maxtxpkt/maxrxpkt. rest of ksettings always updated
 */
bool
convert_legacy_settings_to_link_ksettings(
	struct ethtool_link_ksettings *link_ksettings,
	const struct ethtool_cmd *legacy_settings)
{
	bool retval = true;

	memset(link_ksettings, 0, sizeof(*link_ksettings));

	/* This is used to tell users that driver is still using these
	 * deprecated legacy fields, and they should not use
	 * %ETHTOOL_GLINKSETTINGS/%ETHTOOL_SLINKSETTINGS
	 */
	if (legacy_settings->maxtxpkt ||
	    legacy_settings->maxrxpkt)
		retval = false;

	ethtool_convert_legacy_u32_to_link_mode(
		link_ksettings->link_modes.supported,
		legacy_settings->supported);
	ethtool_convert_legacy_u32_to_link_mode(
		link_ksettings->link_modes.advertising,
		legacy_settings->advertising);
	ethtool_convert_legacy_u32_to_link_mode(
		link_ksettings->link_modes.lp_advertising,
		legacy_settings->lp_advertising);
	link_ksettings->base.speed
		= ethtool_cmd_speed(legacy_settings);
	link_ksettings->base.duplex
		= legacy_settings->duplex;
	link_ksettings->base.port
		= legacy_settings->port;
	link_ksettings->base.phy_address
		= legacy_settings->phy_address;
	link_ksettings->base.autoneg
		= legacy_settings->autoneg;
	link_ksettings->base.mdio_support
		= legacy_settings->mdio_support;
	link_ksettings->base.eth_tp_mdix
		= legacy_settings->eth_tp_mdix;
	link_ksettings->base.eth_tp_mdix_ctrl
		= legacy_settings->eth_tp_mdix_ctrl;
	return retval;
}

int __ethtool_get_link(struct net_device *dev)
{
	if (!dev->ethtool_ops->get_link)
		return -EOPNOTSUPP;

	return netif_running(dev) && dev->ethtool_ops->get_link(dev);
}

static int ethtool_get_rxnfc_rule_count(struct net_device *dev)
{
	const struct ethtool_ops *ops = dev->ethtool_ops;
	struct ethtool_rxnfc info = {
		.cmd = ETHTOOL_GRXCLSRLCNT,
	};
	int err;

	err = ops->get_rxnfc(dev, &info, NULL);
	if (err)
		return err;

	return info.rule_cnt;
}

/* Max offset for one RSS context */
static u32 ethtool_get_rss_ctx_max_channel(struct ethtool_rxfh_context *ctx)
{
	u32 max_ring = 0;
	u32 i, *tbl;

	if (WARN_ON_ONCE(!ctx))
		return 0;
	tbl = ethtool_rxfh_context_indir(ctx);
	for (i = 0; i < ctx->indir_size; i++)
		max_ring = max(max_ring, tbl[i]);
	return max_ring;
}

static int ethtool_get_max_rxnfc_channel(struct net_device *dev, u64 *max)
{
	const struct ethtool_ops *ops = dev->ethtool_ops;
	struct ethtool_rxnfc *info;
	int err, i, rule_cnt;
	u64 max_ring = 0;

	if (!ops->get_rxnfc)
		return -EOPNOTSUPP;

	rule_cnt = ethtool_get_rxnfc_rule_count(dev);
	if (rule_cnt <= 0)
		return -EINVAL;

	info = kvzalloc(struct_size(info, rule_locs, rule_cnt), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->cmd = ETHTOOL_GRXCLSRLALL;
	info->rule_cnt = rule_cnt;
	err = ops->get_rxnfc(dev, info, info->rule_locs);
	if (err)
		goto err_free_info;

	for (i = 0; i < rule_cnt; i++) {
		struct ethtool_rxnfc rule_info = {
			.cmd = ETHTOOL_GRXCLSRULE,
			.fs.location = info->rule_locs[i],
		};

		err = ops->get_rxnfc(dev, &rule_info, NULL);
		if (err)
			goto err_free_info;

		if (rule_info.fs.ring_cookie != RX_CLS_FLOW_DISC &&
		    rule_info.fs.ring_cookie != RX_CLS_FLOW_WAKE &&
		    !ethtool_get_flow_spec_ring_vf(rule_info.fs.ring_cookie)) {
			u64 ring = rule_info.fs.ring_cookie;

			if (rule_info.flow_type & FLOW_RSS) {
				struct ethtool_rxfh_context *ctx;

				ctx = xa_load(&dev->ethtool->rss_ctx,
					      rule_info.rss_context);
				ring += ethtool_get_rss_ctx_max_channel(ctx);
			}
			max_ring = max_t(u64, max_ring, ring);
		}
	}

	kvfree(info);
	*max = max_ring;
	return 0;

err_free_info:
	kvfree(info);
	return err;
}

/* Max offset across all of a device's RSS contexts */
static u32 ethtool_get_max_rss_ctx_channel(struct net_device *dev)
{
	struct ethtool_rxfh_context *ctx;
	unsigned long context;
	u32 max_ring = 0;

	mutex_lock(&dev->ethtool->rss_lock);
	xa_for_each(&dev->ethtool->rss_ctx, context, ctx)
		max_ring = max(max_ring, ethtool_get_rss_ctx_max_channel(ctx));
	mutex_unlock(&dev->ethtool->rss_lock);

	return max_ring;
}

static u32 ethtool_get_max_rxfh_channel(struct net_device *dev)
{
	struct ethtool_rxfh_param rxfh = {};
	u32 dev_size, current_max = 0;
	int ret;

	/* While we do track whether RSS context has an indirection
	 * table explicitly set by the user, no driver looks at that bit.
	 * Assume drivers won't auto-regenerate the additional tables,
	 * to be safe.
	 */
	current_max = ethtool_get_max_rss_ctx_channel(dev);

	if (!netif_is_rxfh_configured(dev))
		return current_max;

	if (!dev->ethtool_ops->get_rxfh_indir_size ||
	    !dev->ethtool_ops->get_rxfh)
		return current_max;
	dev_size = dev->ethtool_ops->get_rxfh_indir_size(dev);
	if (dev_size == 0)
		return current_max;

	rxfh.indir = kcalloc(dev_size, sizeof(rxfh.indir[0]), GFP_USER);
	if (!rxfh.indir)
		return U32_MAX;

	mutex_lock(&dev->ethtool->rss_lock);
	ret = dev->ethtool_ops->get_rxfh(dev, &rxfh);
	mutex_unlock(&dev->ethtool->rss_lock);
	if (ret) {
		current_max = U32_MAX;
		goto out_free;
	}

	while (dev_size--)
		current_max = max(current_max, rxfh.indir[dev_size]);

out_free:
	kfree(rxfh.indir);
	return current_max;
}

int ethtool_check_max_channel(struct net_device *dev,
			      struct ethtool_channels channels,
			      struct genl_info *info)
{
	u64 max_rxnfc_in_use;
	u32 max_rxfh_in_use;
	int max_mp_in_use;

	/* ensure the new Rx count fits within the configured Rx flow
	 * indirection table/rxnfc settings
	 */
	if (ethtool_get_max_rxnfc_channel(dev, &max_rxnfc_in_use))
		max_rxnfc_in_use = 0;
	max_rxfh_in_use = ethtool_get_max_rxfh_channel(dev);
	if (channels.combined_count + channels.rx_count <= max_rxfh_in_use) {
		if (info)
			GENL_SET_ERR_MSG_FMT(info, "requested channel counts are too low for existing indirection table (%d)", max_rxfh_in_use);
		return -EINVAL;
	}
	if (channels.combined_count + channels.rx_count <= max_rxnfc_in_use) {
		if (info)
			GENL_SET_ERR_MSG(info, "requested channel counts are too low for existing ntuple filter settings");
		return -EINVAL;
	}

	max_mp_in_use = dev_get_min_mp_channel_count(dev);
	if (channels.combined_count + channels.rx_count <= max_mp_in_use) {
		if (info)
			GENL_SET_ERR_MSG_FMT(info, "requested channel counts are too low for existing memory provider setting (%d)", max_mp_in_use);
		return -EINVAL;
	}

	return 0;
}

int ethtool_check_rss_ctx_busy(struct net_device *dev, u32 rss_context)
{
	const struct ethtool_ops *ops = dev->ethtool_ops;
	struct ethtool_rxnfc *info;
	int rc, i, rule_cnt;

	if (!ops->get_rxnfc)
		return 0;

	rule_cnt = ethtool_get_rxnfc_rule_count(dev);
	if (!rule_cnt)
		return 0;

	if (rule_cnt < 0)
		return -EINVAL;

	info = kvzalloc(struct_size(info, rule_locs, rule_cnt), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->cmd = ETHTOOL_GRXCLSRLALL;
	info->rule_cnt = rule_cnt;
	rc = ops->get_rxnfc(dev, info, info->rule_locs);
	if (rc)
		goto out_free;

	for (i = 0; i < rule_cnt; i++) {
		struct ethtool_rxnfc rule_info = {
			.cmd = ETHTOOL_GRXCLSRULE,
			.fs.location = info->rule_locs[i],
		};

		rc = ops->get_rxnfc(dev, &rule_info, NULL);
		if (rc)
			goto out_free;

		if (rule_info.fs.flow_type & FLOW_RSS &&
		    rule_info.rss_context == rss_context) {
			rc = -EBUSY;
			goto out_free;
		}
	}

out_free:
	kvfree(info);
	return rc;
}

struct ethtool_rxfh_context *
ethtool_rxfh_ctx_alloc(const struct ethtool_ops *ops,
		       u32 indir_size, u32 key_size)
{
	size_t indir_bytes, flex_len, key_off, size;
	struct ethtool_rxfh_context *ctx;
	u32 priv_bytes, indir_max;
	u16 key_max;

	key_max = max(key_size, ops->rxfh_key_space);
	indir_max = max(indir_size, ops->rxfh_indir_space);

	priv_bytes = ALIGN(ops->rxfh_priv_size, sizeof(u32));
	indir_bytes = array_size(indir_max, sizeof(u32));

	key_off = size_add(priv_bytes, indir_bytes);
	flex_len = size_add(key_off, key_max);
	size = struct_size_t(struct ethtool_rxfh_context, data, flex_len);

	ctx = kzalloc(size, GFP_KERNEL_ACCOUNT);
	if (!ctx)
		return NULL;

	ctx->indir_size = indir_size;
	ctx->key_size = key_size;
	ctx->key_off = key_off;
	ctx->priv_size = ops->rxfh_priv_size;

	ctx->hfunc = ETH_RSS_HASH_NO_CHANGE;
	ctx->input_xfrm = RXH_XFRM_NO_CHANGE;

	return ctx;
}

/* Check if fields configured for flow hash are symmetric - if src is included
 * so is dst and vice versa.
 */
int ethtool_rxfh_config_is_sym(u64 rxfh)
{
	bool sym;

	sym = rxfh == (rxfh & (RXH_IP_SRC | RXH_IP_DST |
			       RXH_L4_B_0_1 | RXH_L4_B_2_3));
	sym &= !!(rxfh & RXH_IP_SRC)   == !!(rxfh & RXH_IP_DST);
	sym &= !!(rxfh & RXH_L4_B_0_1) == !!(rxfh & RXH_L4_B_2_3);

	return sym;
}

int ethtool_check_ops(const struct ethtool_ops *ops)
{
	if (WARN_ON(ops->set_coalesce && !ops->supported_coalesce_params))
		return -EINVAL;
	if (WARN_ON(ops->rxfh_max_num_contexts == 1))
		return -EINVAL;
	if (WARN_ON(ops->supported_input_xfrm && !ops->get_rxfh_fields))
		return -EINVAL;
	if (WARN_ON(ops->supported_input_xfrm &&
		    ops->rxfh_per_ctx_fields != ops->rxfh_per_ctx_key))
		return -EINVAL;

	/* NOTE: sufficiently insane drivers may swap ethtool_ops at runtime,
	 * the fact that ops are checked at registration time does not
	 * mean the ops attached to a netdev later on are sane.
	 */
	return 0;
}

void ethtool_ringparam_get_cfg(struct net_device *dev,
			       struct ethtool_ringparam *param,
			       struct kernel_ethtool_ringparam *kparam,
			       struct netlink_ext_ack *extack)
{
	memset(param, 0, sizeof(*param));
	memset(kparam, 0, sizeof(*kparam));

	param->cmd = ETHTOOL_GRINGPARAM;
	dev->ethtool_ops->get_ringparam(dev, param, kparam, extack);

	/* Driver gives us current state, we want to return current config */
	kparam->tcp_data_split = dev->cfg->hds_config;
	kparam->hds_thresh = dev->cfg->hds_thresh;
}

static void ethtool_init_tsinfo(struct kernel_ethtool_ts_info *info)
{
	memset(info, 0, sizeof(*info));
	info->cmd = ETHTOOL_GET_TS_INFO;
	info->phc_index = -1;
}

int ethtool_net_get_ts_info_by_phc(struct net_device *dev,
				   struct kernel_ethtool_ts_info *info,
				   struct hwtstamp_provider_desc *hwprov_desc)
{
	const struct ethtool_ops *ops = dev->ethtool_ops;
	int err;

	if (!ops->get_ts_info)
		return -ENODEV;

	/* Does ptp comes from netdev */
	ethtool_init_tsinfo(info);
	info->phc_qualifier = hwprov_desc->qualifier;
	err = ops->get_ts_info(dev, info);
	if (err)
		return err;

	if (info->phc_index == hwprov_desc->index &&
	    net_support_hwtstamp_qualifier(dev, hwprov_desc->qualifier))
		return 0;

	return -ENODEV;
}

struct phy_device *
ethtool_phy_get_ts_info_by_phc(struct net_device *dev,
			       struct kernel_ethtool_ts_info *info,
			       struct hwtstamp_provider_desc *hwprov_desc)
{
	int err;

	/* Only precise qualifier is supported in phydev */
	if (hwprov_desc->qualifier != HWTSTAMP_PROVIDER_QUALIFIER_PRECISE)
		return ERR_PTR(-ENODEV);

	/* Look in the phy topology */
	if (dev->link_topo) {
		struct phy_device_node *pdn;
		unsigned long phy_index;

		xa_for_each(&dev->link_topo->phys, phy_index, pdn) {
			if (!phy_has_tsinfo(pdn->phy))
				continue;

			ethtool_init_tsinfo(info);
			err = phy_ts_info(pdn->phy, info);
			if (err)
				return ERR_PTR(err);

			if (info->phc_index == hwprov_desc->index)
				return pdn->phy;
		}
		return ERR_PTR(-ENODEV);
	}

	/* Look on the dev->phydev */
	if (phy_has_tsinfo(dev->phydev)) {
		ethtool_init_tsinfo(info);
		err = phy_ts_info(dev->phydev, info);
		if (err)
			return ERR_PTR(err);

		if (info->phc_index == hwprov_desc->index)
			return dev->phydev;
	}

	return ERR_PTR(-ENODEV);
}

int ethtool_get_ts_info_by_phc(struct net_device *dev,
			       struct kernel_ethtool_ts_info *info,
			       struct hwtstamp_provider_desc *hwprov_desc)
{
	int err;

	err = ethtool_net_get_ts_info_by_phc(dev, info, hwprov_desc);
	if (err == -ENODEV) {
		struct phy_device *phy;

		phy = ethtool_phy_get_ts_info_by_phc(dev, info, hwprov_desc);
		if (IS_ERR(phy))
			return PTR_ERR(phy);

		/* Report the phc source only if we have a real
		 * phc source with an index.
		 */
		if (info->phc_index >= 0) {
			info->phc_source = HWTSTAMP_SOURCE_PHYLIB;
			info->phc_phyindex = phy->phyindex;
		}
		err = 0;
	} else if (!err && info->phc_index >= 0) {
		info->phc_source = HWTSTAMP_SOURCE_NETDEV;
	}

	info->so_timestamping |= SOF_TIMESTAMPING_RX_SOFTWARE |
				 SOF_TIMESTAMPING_SOFTWARE;

	return err;
}

int __ethtool_get_ts_info(struct net_device *dev,
			  struct kernel_ethtool_ts_info *info)
{
	struct hwtstamp_provider *hwprov;
	int err = 0;

	rcu_read_lock();
	hwprov = rcu_dereference(dev->hwprov);
	/* No provider specified, use default behavior */
	if (!hwprov) {
		const struct ethtool_ops *ops = dev->ethtool_ops;
		struct phy_device *phydev = dev->phydev;

		ethtool_init_tsinfo(info);
		if (phy_is_default_hwtstamp(phydev) &&
		    phy_has_tsinfo(phydev)) {
			err = phy_ts_info(phydev, info);
			/* Report the phc source only if we have a real
			 * phc source with an index.
			 */
			if (!err && info->phc_index >= 0) {
				info->phc_source = HWTSTAMP_SOURCE_PHYLIB;
				info->phc_phyindex = phydev->phyindex;
			}
		} else if (ops->get_ts_info) {
			err = ops->get_ts_info(dev, info);
			if (!err && info->phc_index >= 0)
				info->phc_source = HWTSTAMP_SOURCE_NETDEV;
		}

		info->so_timestamping |= SOF_TIMESTAMPING_RX_SOFTWARE |
					 SOF_TIMESTAMPING_SOFTWARE;

		rcu_read_unlock();
		return err;
	}

	err = ethtool_get_ts_info_by_phc(dev, info, &hwprov->desc);
	rcu_read_unlock();
	return err;
}

bool net_support_hwtstamp_qualifier(struct net_device *dev,
				    enum hwtstamp_provider_qualifier qualifier)
{
	const struct ethtool_ops *ops = dev->ethtool_ops;

	if (!ops)
		return false;

	/* Return true with precise qualifier and with NIC without
	 * qualifier description to not break the old behavior.
	 */
	if (!ops->supported_hwtstamp_qualifiers &&
	    qualifier == HWTSTAMP_PROVIDER_QUALIFIER_PRECISE)
		return true;

	if (ops->supported_hwtstamp_qualifiers & BIT(qualifier))
		return true;

	return false;
}

int ethtool_get_phc_vclocks(struct net_device *dev, int **vclock_index)
{
	struct kernel_ethtool_ts_info info = { };
	int num = 0;

	if (!__ethtool_get_ts_info(dev, &info))
		num = ptp_get_vclocks_index(info.phc_index, vclock_index);

	return num;
}
EXPORT_SYMBOL(ethtool_get_phc_vclocks);

int ethtool_get_ts_info_by_layer(struct net_device *dev, struct kernel_ethtool_ts_info *info)
{
	return __ethtool_get_ts_info(dev, info);
}
EXPORT_SYMBOL(ethtool_get_ts_info_by_layer);

const struct ethtool_phy_ops *ethtool_phy_ops;

void ethtool_set_ethtool_phy_ops(const struct ethtool_phy_ops *ops)
{
	ASSERT_RTNL();
	ethtool_phy_ops = ops;
}
EXPORT_SYMBOL_GPL(ethtool_set_ethtool_phy_ops);

void
ethtool_params_from_link_mode(struct ethtool_link_ksettings *link_ksettings,
			      enum ethtool_link_mode_bit_indices link_mode)
{
	const struct link_mode_info *link_info;

	if (WARN_ON_ONCE(link_mode >= __ETHTOOL_LINK_MODE_MASK_NBITS))
		return;

	link_info = &link_mode_params[link_mode];
	link_ksettings->base.speed = link_info->speed;
	link_ksettings->lanes = link_info->lanes;
	link_ksettings->base.duplex = link_info->duplex;
}
EXPORT_SYMBOL_GPL(ethtool_params_from_link_mode);

/**
 * ethtool_forced_speed_maps_init
 * @maps: Pointer to an array of Ethtool forced speed map
 * @size: Array size
 *
 * Initialize an array of Ethtool forced speed map to Ethtool link modes. This
 * should be called during driver module init.
 */
void
ethtool_forced_speed_maps_init(struct ethtool_forced_speed_map *maps, u32 size)
{
	for (u32 i = 0; i < size; i++) {
		struct ethtool_forced_speed_map *map = &maps[i];

		linkmode_set_bit_array(map->cap_arr, map->arr_size, map->caps);
		map->cap_arr = NULL;
		map->arr_size = 0;
	}
}
EXPORT_SYMBOL_GPL(ethtool_forced_speed_maps_init);

void ethtool_rxfh_context_lost(struct net_device *dev, u32 context_id)
{
	struct ethtool_rxfh_context *ctx;

	WARN_ONCE(!rtnl_is_locked() &&
		  !lockdep_is_held_type(&dev->ethtool->rss_lock, -1),
		  "RSS context lock assertion failed\n");

	netdev_err(dev, "device error, RSS context %d lost\n", context_id);
	ctx = xa_erase(&dev->ethtool->rss_ctx, context_id);
	kfree(ctx);
	ethtool_rss_notify(dev, ETHTOOL_MSG_RSS_DELETE_NTF, context_id);
}
EXPORT_SYMBOL(ethtool_rxfh_context_lost);
