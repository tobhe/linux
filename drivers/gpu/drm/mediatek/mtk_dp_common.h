/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2024 MediaTek Inc.
 */

#ifndef __MTK_DP_COMMON_H__
#define __MTK_DP_COMMON_H__

#include <drm/drm_bridge.h>
#include <drm/drm_encoder.h>
#include <drm/display/drm_dp_helper.h>
#include <drm/display/drm_dp_mst_helper.h>
#include <drm/display/drm_hdcp_helper.h>

#include <sound/hdmi-codec.h>

#include <linux/media-bus-format.h>

#include "tlc_dp_hdcp.h"

#define SUPPORT_YUV422		1

#define DP_ENCODER_NUM		2
#define DP_CONNECTOR_NUM	16
#define DP_PAYLOAD_MAX		8
#define DP_FIRST_CON		0
#define DP_SST_ENCODER_PORT	0
#define DP_ENCODER_ENDPOINT	0

#define DP_PHY_LEVEL_COUNT	10
#define DP_REG_OFFSET(a)	((a) * 0xa00)

#define HPD_CONNECT			BIT(0)
#define HPD_DISCONNECT		BIT(10)
#define HPD_INITIAL_STATE	0
#define HPD_INT_EVNET		BIT(2)

#define DP_HPD_DEBOUNCE		100

#define DP_DSC_BPP			8

#define DP_BITWIDTH_16		BIT(0)
#define DP_BITWIDTH_20		BIT(1)
#define DP_BITWIDTH_24		BIT(2)

#define PHY_READ_BYTE(mtk_dp, reg) mtk_dp_phy_read_v2(mtk_dp, reg)
#define PHY_WRITE_BYTE(mtk_dp, reg, val) \
	mtk_dp_phy_write_byte_v2(mtk_dp, reg, val, 0xFF)
#define PHY_WRITE_2BYTE(mtk_dp, reg, val) \
	mtk_dp_phy_mask_v2(mtk_dp, reg, val, 0xFFFF)
#define PHY_WRITE_4BYTE(mtk_dp, reg, val) \
	mtk_dp_phy_write_v2(mtk_dp, reg, val)
#define PHY_WRITE_BYTE_MASK(mtk_dp, addr, val, mask) \
	mtk_dp_phy_write_byte_v2(mtk_dp, addr, val, mask)
#define PHY_WRITE_2BYTE_MASK(mtk_dp, addr, val, mask) \
	mtk_dp_phy_mask_v2(mtk_dp, addr, val, mask)
#define PHY_WRITE_4BYTE_MASK(mtk_dp, addr, val, mask) \
	mtk_dp_phy_mask_v2(mtk_dp, addr, val, mask)
#define READ_BYTE(mtk_dp, reg) \
	(mtk_dp_read_v2(mtk_dp, reg) & 0xFF)
#define READ_2BYTE(mtk_dp, reg) \
	(mtk_dp_read_v2(mtk_dp, reg) & 0xFFFF)
#define READ_4BYTE(mtk_dp, reg) \
	mtk_dp_read_v2(mtk_dp, reg)
#define WRITE_BYTE(mtk_dp, reg, val) \
	mtk_dp_write_byte_v2(mtk_dp, reg, val, 0xFF)
#define WRITE_2BYTE(mtk_dp, reg, val) \
	mtk_dp_mask_v2(mtk_dp, reg, val, 0xFFFF)
#define WRITE_4BYTE(mtk_dp, reg, val) \
	mtk_dp_write_v2(mtk_dp, reg, val)
#define WRITE_BYTE_MASK(mtk_dp, addr, val, mask) \
	mtk_dp_write_byte_v2(mtk_dp, addr, val, mask)
#define WRITE_2BYTE_MASK(mtk_dp, addr, val, mask) \
	mtk_dp_mask_v2(mtk_dp, addr, val, mask)
#define WRITE_4BYTE_MASK(mtk_dp, addr, val, mask) \
	mtk_dp_mask_v2(mtk_dp, addr, val, mask)

struct mtk_dp;

enum audio_m_div {
	DP_AUDIO_M_DIV_M,
	DP_AUDIO_M_DIV_M2,
	DP_AUDIO_M_DIV_M4,
	DP_AUDIO_M_DIV_M8,
	DP_AUDIO_M_DIV_D2,
	DP_AUDIO_M_DIV_D4,
	DP_AUDIO_M_DIV_NA,
	DP_AUDIO_M_DIV_D8,
};

enum dp_color_depth {
	DP_COLOR_DEPTH_6BIT,
	DP_COLOR_DEPTH_8BIT,
	DP_COLOR_DEPTH_10BIT,
	DP_COLOR_DEPTH_12BIT,
	DP_COLOR_DEPTH_16BIT,
	DP_COLOR_DEPTH_UNKNOWN,
};

enum dp_encoder_id {
	DP_ENCODER_ID_0 = 0,
	DP_ENCODER_ID_1 = 1,
	DP_ENCODER_ID_MAX
};

enum dp_lane_count {
	DP_1LANE = 0x01,
	DP_2LANE = 0x02,
	DP_4LANE = 0x04,
};

enum dp_lane_num {
	DP_LANE0,
	DP_LANE1,
	DP_LANE2,
	DP_LANE3,
	DP_LANE_MAX,
};

enum dp_link_rate {
	DP_LINK_RATE_RBR = 0x6,
	DP_LINK_RATE_HBR = 0xa,
	DP_LINK_RATE_HBR2 = 0x14,
	DP_LINK_RATE_HBR25 = 0x19,
	DP_LINK_RATE_HBR3 = 0x1e,
	DP_LINK_RATE_UHBR10  = 0x20,
};

enum dp_lt_pattern {
	DP_0 = 0,
	DP_TPS1 = BIT(4),
	DP_TPS2 = BIT(5),
	DP_TPS3 = BIT(6),
	DP_TPS4 = BIT(7),
	DP_20_TPS1 = BIT(14),
	DP_20_TPS2 = BIT(15),
};

enum dp_pg_typesel {
	DP_PG_NONE,
	DP_PG_PURE_COLOR,
	DP_PG_VERTICAL_RAMPING,
	DP_PG_HORIZONTAL_RAMPING,
	DP_PG_VERTICAL_COLOR_BAR ,
	DP_PG_HORIZONTAL_COLOR_BAR,
	DP_PG_CHESSBOARD_PATTERN,
	DP_PG_SUB_PIXEL_PATTERN,
	DP_PG_FRAME_PATTERN,
	DP_PG_MAX,
};

enum dp_preemphasis_num {
	DP_PREEMPHASIS0,
	DP_PREEMPHASIS1,
	DP_PREEMPHASIS2,
	DP_PREEMPHASIS3,
};

enum dp_swing_num {
	DP_SWING0,
	DP_SWING1,
	DP_SWING2,
	DP_SWING3,
};

enum dp_return_status {
	DP_RET_NOERR,
	DP_RET_PLUG_OUT,
	DP_RET_TIMEOU,
	DP_RET_AUTH_FAIL,
	DP_RET_EDID_FAIL,
	DP_RET_TRANING_FAIL,
	DP_RET_RETRANING,
};

enum dp_video_mode {
	DP_VIDEO_INTERLACE,
	DP_VIDEO_PROGRESSIVE,
};

union dp_misc {
	struct {
		u8 is_sync_clock : 1;
		u8 color_format : 2;
		u8 spec_def1 : 2;
		u8 color_depth : 3;

		u8 interlaced : 1;
		u8 stereo_attr : 2;
		u8 reserved : 3;
		u8 is_vsc_sdp : 1;
		u8 spec_def2 : 1;
	} misc;
	u8 misc_raw[2];
};

union dp_pps {
	struct{
		u8 major : 4;
		u8 minor : 4;               /* pps0 */
		u8 pps_id : 8;              /* pps1 */
		u8 reserved1 : 8;           /* pps2 */
		u8 color_depth : 4;
		u8 buffer_depth : 4;        /* pps3 */
		u8 reserved2 : 2;
		bool bp_enable : 1;
		bool convert_rgb : 1;
		bool simple_422 : 1;
		bool vbr_enable : 1;
		u16 bit_per_pixel : 10;      /* pps4-5 */
		u16 pic_height : 16;         /* pps6-7 */
		u16 pic_width : 16;          /* pps8-9 */
		u16 slice_height : 16;       /* pps10-11 */
		u16 slice_width : 16;        /* pps12-13 */
		u16 chunk_size : 16;         /* pps14-15 */
		u8 reserved3 : 6;
		u16 init_xmit_delay : 10;    /* pps16-17 */
		u16 init_dec_delay : 16;     /* pps18-19 */
		u16 reserved4 : 10;
		u8 init_scale_val : 6;       /* pps20-21 */
		u16 scale_inc_interval : 16; /* pps22-23 */
		u8 reserved5 : 4;
		u16 scale_dec_interval : 12; /* pps24-25 */
		u16 reserved6 : 11;
		u8 first_line_offset : 5;    /* pps26-27 */
		u16 nfl_bpg_offset : 16;     /* pps28-29 */
		u16 slice_bpg_offset : 16;   /* pps30-31 */
		u16 init_offset : 16;        /* pps32-33 */
		u16 final_offset : 16;       /* pps34-35 */
		u8 reserved7 : 3;
		u8 min_qp : 5;               /* pps36 */
		u8 reserved8 : 3;
		u8 max_qp : 5;               /* pps37 */
		u8 rc_param_set[50];         /* pps38-87 */

		u8 reserved9 : 6;
		bool native_420 : 1;
		bool native_422 : 1;         /* pps88 */
		u8 reserved10 : 3;
		u8 sec_line_bpg_offset : 5;  /* pps89 */
		u16 nsl_bpg_offset : 16;     /* pps90-91 */
		u16 sec_line_offset : 16;    /* pps92-93 */
		u8 reserved11[34];           /* pps94-127 */
	} pps;

	u8 pps_raw[128];
};

struct mtk_dp_efuse_fmt {
	unsigned short idx;
	unsigned short shift;
	unsigned short mask;
	unsigned short min_val;
	unsigned short max_val;
	unsigned short default_val;
};

struct mtk_dp_data {
	int bridge_type;
	unsigned int smc_cmd;
	const struct mtk_dp_efuse_fmt *efuse_fmt;
	bool audio_supported;
	bool audio_pkt_in_hblank_area;
	u16 audio_m_div2_bit;
	bool ssc_support;
	bool dsc_support;
	bool mst_support;
	u16 max_hdisplay;
	u16 max_vdisplay;
	u16 min_hblanking;
	u16 max_fps_at_max_resolution;
	u16 min_hdisplay;
	u16 min_vdisplay;
	u32 patch_phy_settings[6];
	u32 txfir_settings[6];
	void (*set_param)(struct mtk_dp *mtk_dp);
	u32 phyd_dig_glb_offset;
	u32 phyd_dig_lan_offset[4];
	u32 phyd_ana_glb_offset;
	u32 phyd_ana_lan_offset[4];
	u8 max_link_rate;
	u8 max_lane_count;
	u8 encoder_num;
	bool need_phy_lane_enable_set;
	bool need_phy_flip_set;
	u32 phy_4lane_ctrl_bit;
	u32 phy_flip_ctrl_bit;
};

struct dp_timing_parameter {
	u16 htt;
	u16 hde;
	u16 hbk;
	u16 hfp;
	u16 hsw;

	bool hsp;
	u16 hbp;
	u16 vtt;
	u16 vde;
	u16 vbk;
	u16 vfp;
	u16 vsw;

	bool vsp;
	u16 vbp;
	u8 frame_rate;
	u64 pixcel_rate;
	int video_ip_mode;
	union dp_misc misc;
};

struct mtk_dp_audio_cfg {
	bool detect_monitor;
	int sad_count;
	int sample_rate;
	int word_length_bits;
	int channels;
};

struct dp_info {
	struct dp_timing_parameter dp_output_timing;
	struct mtk_dp_audio_cfg audio_cur_cfg;
	union dp_pps pps;
	bool pattern_gen : 1;
	bool audio_mute : 1;
	bool video_mute : 1;
	u8 depth;
	u8 format;
	u32 video_m;
	u32 video_n;
};

struct dp_training_info {
	bool sink_ext_cap_en : 1;
	bool tps3_support : 1;
	bool tps4_support : 1;
	bool sink_ssc_en : 1;
	bool dp_tx_auto_test_en : 1;
	bool cable_plug_in : 1;
	bool cable_state_change : 1;
	bool dp_mst_cap : 1;
	bool dp_mst_en : 1;
	bool dp_mst_branch : 1;
	bool dwn_strm_port_present : 1;
	bool cr_done : 1;
	bool eq_done : 1;

	u8 dp_version;
	u8 max_link_rate;
	u8 max_link_lane_count;
	u8 link_rate;
	u8 link_lane_count;
	u16 phy_status;
	u8 dpcd_rev;
	u8 sink_count;
	u8 check_cap_times;
};

struct dp_phy_parameter {
	unsigned char c0;
	unsigned char cp1;
};

struct mtk_dp_con {
	struct mtk_dp *mtk_dp;
	struct drm_dp_mst_port *port;
	struct drm_connector connector;
	struct drm_encoder *encoder;
	enum dp_encoder_id encoder_id;

	struct drm_dp_aux *dsc_aux;
	u8 dsc_dpcd[DP_DSC_RECEIVER_CAP_SIZE];
	u8 fec_cap;
	u8 dsc_hblank_expansion_quirk:1;
	u8 dsc_decompression_enabled:1;

	struct edid *edid;
	enum drm_dp_mst_mode dp_mode;
	bool video_enable;
};

struct mtk_dp_bridge {
	struct mtk_dp *mtk_dp;
	struct drm_bridge bridge;
	enum dp_encoder_id encoder_id;
};

struct mtk_dp {
	struct device *dev;
	struct drm_device *drm_dev;
	struct drm_bridge bridge;
	const struct mtk_dp_data *data;
	int id;
	struct drm_dp_aux aux;
	u8 rx_cap[DP_RECEIVER_CAP_SIZE];
	struct drm_display_mode mode[DP_ENCODER_NUM];
	struct dp_info info[DP_ENCODER_NUM];
	struct dp_training_info training_info;
	struct notifier_block notifier;
	struct clk *pclk;
	struct clk *pclk_src;
	atomic_t refcount;
	int irq;

	struct device *genpd_dp_tx;
	struct device *genpd_dp_phy;
	struct device_link *genpd_dl_dp_tx;
	struct device_link *genpd_dl_dp_phy;

	/* irq_thread_lock is used to protect phy_status */
	spinlock_t irq_thread_lock;

	struct mtk_hdcp_info hdcp_info;
	struct work_struct hdcp_enable_work;
	struct work_struct hdcp_disable_work;
	struct work_struct prop_work;
	struct work_struct audio_work;
	struct delayed_work check_work;
	struct workqueue_struct *hdcp_workqueue;
	struct drm_connector_state con_state[DP_ENCODER_NUM];
	/* This mutex is used to synchronize HDCP operations in the driver */
	struct mutex hdcp_mutex;
	/* This mutex is used to synchronize MST operations in the driver */
	struct mutex mst_mutex;

	void __iomem *regs;
	void __iomem *phyd_regs;
	void __iomem *phy_mux_regs;
	void __iomem *mac_power_regs;

	int disp_state;
	bool dp_ready;

	struct mtk_drm_private *priv;
	struct dp_phy_parameter phy_params[DP_PHY_LEVEL_COUNT];

	bool phy_by_link_rate;
	u32 phy_settings[6];
	u32 phy_settings_hbr2[6];

	bool swap_enable;
	bool phy_flip_enable;
	bool need_debounce;

	bool init_property;
	struct drm_property *prop_dsc_enable[DP_ENCODER_NUM];
	struct drm_property *prop_dsc_cfg[DP_ENCODER_NUM];
	bool dsc_enable[DP_ENCODER_NUM];

	bool mst_enable;
	bool mst_start;
	u8 mst_enable_count;
	struct drm_dp_mst_topology_mgr mgr;
	struct mtk_dp_con *mtk_con[DP_CONNECTOR_NUM];
	struct mtk_dp_bridge *mtk_bridge[DP_ENCODER_NUM];

	/* For audio */
	bool audio_enable;
	hdmi_codec_plugged_cb plugged_cb;
	struct platform_device *audio_pdev[DP_ENCODER_NUM];
	bool audio_codec_on[DP_ENCODER_NUM];
	struct platform_device *pseudo_audio_pdev;
	struct device *codec_dev[DP_ENCODER_NUM];
	struct device *vdisp_ao_dev;
	/* protect the plugged_cb as it's used in both bridge ops and audio */
	struct mutex update_plugged_status_lock;

	struct gpio_desc *phy_flip_pin;
};
#endif
