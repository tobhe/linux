// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015 MediaTek Inc.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/minmax.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/units.h>

#include <video/mipi_display.h>
#include <video/videomode.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>

#include "mtk_ddp_comp.h"
#include "mtk_disp_drv.h"
#include "mtk_drm_drv.h"

#define REG_FLD_SHIFT(field) ((unsigned int)((field) & 0xff))
#define REG_FLD_WIDTH(field) ((unsigned int)(((field) >> 16) & 0xff))
#define REG_FLD_MASK(field)							\
	((unsigned int)((1ULL << REG_FLD_WIDTH(field)) - 1)			\
	 << REG_FLD_SHIFT(field))
#define REG_FLD_VAL(field, val)							\
	(((val) << REG_FLD_SHIFT(field)) & REG_FLD_MASK(field))
#define REG_FLD(width, shift)							\
	((unsigned int)((((width) & 0xff) << 16) | ((shift) & 0xff)))
#define REG_FLD_MSB_LSB(msb, lsb) REG_FLD((msb) - (lsb) + 1, (lsb))

#define DSI_START		0x00

#define DSI_INTEN		0x08

#define DSI_INTSTA		0x0c
#define LPRX_RD_RDY_INT_FLAG		BIT(0)
#define CMD_DONE_INT_FLAG		BIT(1)
#define TE_RDY_INT_FLAG			BIT(2)
#define VM_DONE_INT_FLAG		BIT(3)
#define EXT_TE_RDY_INT_FLAG		BIT(4)
#define DSI_BUSY			BIT(31)

#define DSI_CON_CTRL(data)		(0x10 + (data)->reg_20_ofs)
#define DSI_RESET			BIT(0)
#define DSI_EN				BIT(1)
#define DPHY_RESET			BIT(2)
#define DSI_CM_MODE_WAIT_DATA_EVERY_LINE_EN	BIT(24)

#define DSI_MODE_CTRL(data)		(0x14 + (data)->reg_20_ofs)
#define MODE				(3)
#define CMD_MODE			0
#define SYNC_PULSE_MODE			1
#define SYNC_EVENT_MODE			2
#define BURST_MODE			3
#define FRM_MODE			BIT(16)
#define MIX_MODE			BIT(17)

#define DSI_TXRX_CTRL(data)		(0x18 + (data)->reg_20_ofs)
#define VC_NUM				BIT(1)
#define LANE_NUM			GENMASK(5, 2)
#define DIS_EOT				BIT(6)
#define NULL_EN				BIT(7)
#define TE_FREERUN			BIT(8)
#define EXT_TE_EN			BIT(9)
#define EXT_TE_EDGE			BIT(10)
#define MAX_RTN_SIZE			GENMASK(15, 12)
#define HSTX_CKLP_EN			BIT(16)

#define DSI_PSCTRL(data)		(0x1c + (data)->reg_20_ofs)
#define DSI_PS_WC			GENMASK(13, 0)
#define DSI_PS_SEL			GENMASK(19, 16)
#define PACKED_PS_16BIT_RGB565		0
#define PACKED_PS_18BIT_RGB666		1
#define LOOSELY_PS_24BIT_RGB666		2
#define PACKED_PS_24BIT_RGB888		3
#define COMPRESSED_PIXEL_STREAM		5

#define DSI_VSA_NL(data)		(0x20 + (data)->reg_40_ofs)
#define DSI_VBP_NL(data)		(0x24 + (data)->reg_40_ofs)
#define DSI_VFP_NL(data)		(0x28 + (data)->reg_40_ofs)
#define DSI_VACT_NL(data)		(0x2c + (data)->reg_40_ofs)
#define VACT_NL				GENMASK(14, 0)
#define DSI_SIZE_CON(data)		((data)->dsi_size_con ? \
					(data)->dsi_size_con : 0x38)
#define DSI_HEIGHT				GENMASK(30, 16)
#define DSI_WIDTH				GENMASK(14, 0)
#define DSI_HSA_WC(data)		(0x50 + (data)->reg_30_ofs)
#define DSI_HBP_WC(data)		(0x54 + (data)->reg_30_ofs)
#define DSI_HFP_WC(data)		(0x58 + (data)->reg_30_ofs)
#define HFP_HS_VB_PS_WC		GENMASK(30, 16)
#define HFP_HS_EN			BIT(31)

#define DSI_CMDQ_SIZE(data)		((data)->dsi_cmdq_con ? \
					(data)->dsi_cmdq_con : 0x60)
#define CMDQ_SIZE			0x3f
#define CMDQ_SIZE_SEL		BIT(15)

#define DSI_HSTX_CKL_WC(data)		((data)->dsi_hstx_ckl_wc ? \
					(data)->dsi_hstx_ckl_wc : 0x64)
#define HSTX_CKL_WC			GENMASK(15, 2)

#define DSI_BLLP_WC		0x5c

#define DSI_CMD_TYPE1_HS		0x6c
#define CPHY_6BYE_EN			BIT(18)

#define DSI_RX_DATA0(data)		(0x74 + (data)->reg_30_ofs)
#define DSI_RX_DATA1(data)		(0x78 + (data)->reg_30_ofs)
#define DSI_RX_DATA2(data)		(0x7c + (data)->reg_30_ofs)
#define DSI_RX_DATA3(data)		(0x80 + (data)->reg_30_ofs)

#define DSI_RACK(data)			(0x84 + (data)->reg_30_ofs)
#define RACK				BIT(0)

#define DSI_PHY_LCCON(data)		((data)->reg_phy_base ? 0x1d0 : 0x104)
#define LC_HS_TX_EN			BIT(0)
#define LC_ULPM_EN			BIT(1)
#define LC_WAKEUP_EN			BIT(2)

#define DSI_PHY_LD0CON(data)		((data)->reg_phy_base ? 0x1d4 : 0x108)
#define LD0_HS_TX_EN			BIT(0)
#define LD0_ULPM_EN			BIT(1)
#define LD0_WAKEUP_EN			BIT(2)

#define DSI_PHY_TIMECON0(data)		((data)->reg_phy_base ? \
					(data)->reg_phy_base : 0x110)
#define LPX				(0xff << 0)
#define HS_PREP				(0xff << 8)
#define HS_ZERO				(0xff << 16)
#define HS_TRAIL			(0xff << 24)
#define FLD_LPX				REG_FLD_MSB_LSB(7, 0)
#define FLD_HS_PREP			REG_FLD_MSB_LSB(15, 8)
#define FLD_HS_ZERO			REG_FLD_MSB_LSB(23, 16)
#define FLD_HS_TRAIL			REG_FLD_MSB_LSB(31, 24)

#define DSI_PHY_TIMECON1(data)		(DSI_PHY_TIMECON0(data) + 0x4)
#define TA_GO				(0xff << 0)
#define TA_SURE				(0xff << 8)
#define TA_GET				(0xff << 16)
#define DA_HS_EXIT			(0xff << 24)
#define FLD_TA_GO			REG_FLD_MSB_LSB(7, 0)
#define FLD_TA_SURE			REG_FLD_MSB_LSB(15, 8)
#define FLD_TA_GET			REG_FLD_MSB_LSB(23, 16)
#define FLD_DA_HS_EXIT			REG_FLD_MSB_LSB(31, 24)

#define DSI_PHY_TIMECON2(data)		(DSI_PHY_TIMECON0(data) + 0x8)
#define CONT_DET			(0xff << 0)
#define DA_HS_SYNC			(0xff << 8)
#define CLK_ZERO			(0xff << 16)
#define CLK_TRAIL			(0xff << 24)

#define DSI_PHY_TIMECON3(data)		(DSI_PHY_TIMECON0(data) + 0xc)
#define CLK_HS_PREP			(0xff << 0)
#define CLK_HS_POST			(0xff << 8)
#define CLK_HS_EXIT			(0xff << 16)

#define VM_CMD_EN			BIT(0)
#define TS_VFP_EN			BIT(5)

#define DSI_CPHY_CON0			0x120
#define CPHY_EN			BIT(0)
#define CPHY_CON0_VAL			0x012c0003
#define DSI_SHADOW_DEBUG(data)		((data)->dsi_shadow_dbg ? \
					(data)->dsi_shadow_dbg : 0x190)
#define FORCE_COMMIT			BIT(0)
#define BYPASS_SHADOW			BIT(1)

/* CMDQ related bits */
#define CONFIG				GENMASK(7, 0)
#define SHORT_PACKET			0
#define LONG_PACKET			2
#define BTA				BIT(2)
#define DATA_ID				GENMASK(15, 8)
#define DATA_0				GENMASK(23, 16)
#define DATA_1				GENMASK(31, 24)

#define DSI_BUF_CON0			0x300
#define BUF_BUF_EN			BIT(0)
#define DSI_BUF_CON1			0x304
#define DSI_TX_BUF_RW_TIMES		(DSI_BUF_CON0 + 0x10)
#define DSI_BUF_SODI_HIGH		(DSI_BUF_CON0 + 0x14)
#define DSI_BUF_SODI_LOW		(DSI_BUF_CON0 + 0x18)
#define DSI_BUF_PREULTRA_HIGH		(DSI_BUF_CON0 + 0x24)
#define DSI_BUF_PREULTRA_LOW		(DSI_BUF_CON0 + 0x28)
#define DSI_BUF_ULTRA_HIGH		(DSI_BUF_CON0 + 0x2c)
#define DSI_BUF_ULTRA_LOW		(DSI_BUF_CON0 + 0x30)
#define DSI_BUF_URGENT_HIGH		(DSI_BUF_CON0 + 0x34)
#define DSI_BUF_URGENT_LOW		(DSI_BUF_CON0 + 0x38)
#define DSI_BUF_PREURGENT_HIGH		(DSI_BUF_CON0 + 0x3c)
#define DSI_BUF_MASK			GENMASK(14, 0)
#define DSI_DEBUG_SEL			0x274
#define MM_RST_SEL			BIT(10)
#define DSI_RESERVED			0x3F8
#define DSI_VDE_BLOCK_ULTRA 		BIT(29)

#define MMSYS_CLK_RATE_DEFAULT 208

#define NS_TO_CYCLE(n, c)    ((n) / (c) + (((n) % (c)) ? 1 : 0))

#define MTK_DSI_HOST_IS_READ(type) \
	((type == MIPI_DSI_GENERIC_READ_REQUEST_0_PARAM) || \
	(type == MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM) || \
	(type == MIPI_DSI_GENERIC_READ_REQUEST_2_PARAM) || \
	(type == MIPI_DSI_DCS_READ))

struct mtk_phy_timing {
	u32 lpx;
	u32 da_hs_prepare;
	u32 da_hs_zero;
	u32 da_hs_trail;

	u32 ta_go;
	u32 ta_sure;
	u32 ta_get;
	u32 da_hs_exit;

	u32 clk_hs_zero;
	u32 clk_hs_trail;

	u32 clk_hs_prepare;
	u32 clk_hs_post;
	u32 clk_hs_exit;
};

struct phy;

struct mtk_dsi_driver_data {
	const u32 reg_cmdq_off;
	bool has_shadow_ctl;
	bool has_size_ctl;
	bool cmdq_long_packet_ctl;
	bool support_per_frame_lp;
	bool support_dsi_buffer;
	const u32 reg_phy_base;
	const u32 reg_20_ofs;
	const u32 reg_30_ofs;
	const u32 reg_40_ofs;
	const u32 reg_100_ofs;
	const u32 dsi_size_con;
	const u32 dsi_vfp_early_stop;
	const u32 dsi_lfr_con;
	const u32 dsi_cmdq_con;
	const u32 dsi_type1_hs;
	const u32 dsi_hstx_ckl_wc;
	const u32 dsi_mem_conti;
	const u32 dsi_time_con;
	const u32 dsi_reserved;
	const u32 dsi_state_dbg6;
	const u32 dsi_dbg_sel;
	const u32 dsi_shadow_dbg;
	const u32 dsi_vm_cmd_con;
	const u32 max_linkrate_kbps;
	const u32 buffer_unit;
	const u32 sram_unit;
	const u32 urgent_lo_fifo_us;
	const u32 urgent_hi_fifo_us;
	const u32 output_valid_fifo_us;
};

struct mtk_dsi {
	struct device *dev;
	struct mipi_dsi_host host;
	struct mtk_encoder mtk_encoder;
	struct drm_bridge bridge;
	struct drm_bridge *next_bridge;
	struct drm_connector *connector;
	struct phy *phy;

	void __iomem *regs;

	struct clk *engine_clk;
	struct clk *digital_clk;
	struct clk *hs_clk;

	u32 data_rate;

	unsigned long mode_flags;
	enum mipi_dsi_pixel_format format;
	unsigned int lanes;
	struct videomode vm;
	struct mtk_phy_timing phy_timing;
	int refcount;
	bool enabled;
	bool lanes_ready;
	u32 irq_data;
	wait_queue_head_t irq_wait_queue;
	const struct mtk_dsi_driver_data *driver_data;
	struct drm_dsc_config *dsc_config;
	bool dsc_enable;
	u32 data_phy_cycle;
	bool is_cphy;
};

static inline struct mtk_dsi *bridge_to_dsi(struct drm_bridge *b)
{
	return container_of(b, struct mtk_dsi, bridge);
}

static inline struct mtk_dsi *host_to_dsi(struct mipi_dsi_host *h)
{
	return container_of(h, struct mtk_dsi, host);
}

static void mtk_dsi_mask(struct mtk_dsi *dsi, u32 offset, u32 mask, u32 data)
{
	u32 temp = readl(dsi->regs + offset);

	writel((temp & ~mask) | (data & mask), dsi->regs + offset);
}

static int mtk_get_dsi_buffer_bpp(int format) {
	switch (format) {
	case MIPI_DSI_FMT_RGB565:
		return 2;
	case MIPI_DSI_FMT_RGB888:
		return 3;
	default:
		return 0;
	}
}

static void mtk_dsi_cphy_timconfig(struct mtk_dsi *dsi)
{
	u32 ui = 0, cycle_time = 0;
	u32 value = 0;
	u32 data_rate = dsi->data_rate / 1000000;
	struct mtk_phy_timing *timing = &dsi->phy_timing;

	ui = (1000 / data_rate > 0) ? 1000 / data_rate : 1;
	cycle_time = 7000 / data_rate;
	/* spec. lpx > 50ns */
	timing->lpx = (data_rate * 80) / 7000 + 1;
	round_up(timing->lpx, 2);
	/* spec.  38ns < hs_prpr < 95ns */
	timing->da_hs_prepare = (data_rate * 50) / 7000 + 1;
	round_up(timing->da_hs_prepare, 2);
	/* spec.  7ui < hs_zero(prebegin) < 448ui
	 * The PreBegin part of the Preamble may
	 * range from 1 to 64 Words, or 7 to 448 symbols.
	 */
	timing->da_hs_zero = 48;

	/* spec.  7ui < hs_trail(post) < 224ui
	 * The Post field may range from 1 to 32 Words,
	 * or 7 to 224 symbols.
	 */
	timing->da_hs_trail = 32;

	/* spec. ta_get = 5*lpx */
	timing->ta_get = 5 * timing->lpx;
	/* spec. ta_sure = 1.5*lpx */
	timing->ta_sure = 3 * timing->lpx / 2;
	/* spec. ta_go = 4*lpx */
	timing->ta_go = 4 * timing->lpx;
	/* spec. da_hs_exit > 100ns */
	timing->da_hs_exit = round_up((data_rate * 118) / 7000 + 1, 2) + 1;
	dsi->data_phy_cycle = timing->da_hs_prepare + timing->da_hs_zero + timing->da_hs_exit + timing->lpx + 5;
	value = REG_FLD_VAL(FLD_LPX, timing->lpx)
		| REG_FLD_VAL(FLD_HS_PREP, timing->da_hs_prepare)
		| REG_FLD_VAL(FLD_HS_ZERO, timing->da_hs_zero)
		| REG_FLD_VAL(FLD_HS_TRAIL, timing->da_hs_trail);

	writel(value, dsi->regs + DSI_PHY_TIMECON0(dsi->driver_data));

	value = REG_FLD_VAL(FLD_TA_GO, timing->ta_go)
		| REG_FLD_VAL(FLD_TA_SURE, timing->ta_sure)
		| REG_FLD_VAL(FLD_TA_GET, timing->ta_get)
		| REG_FLD_VAL(FLD_DA_HS_EXIT, timing->da_hs_exit);

	writel(value, dsi->regs + DSI_PHY_TIMECON1(dsi->driver_data));
	writel(CPHY_CON0_VAL, dsi->regs + DSI_CPHY_CON0);
	writel(16 * dsi->lanes, dsi->regs + DSI_BLLP_WC);
}

static void mtk_dsi_phy_timconfig(struct mtk_dsi *dsi)
{
	u32 timcon0, timcon1, timcon2, timcon3;
	u32 data_rate_mhz = DIV_ROUND_UP(dsi->data_rate, HZ_PER_MHZ);
	struct mtk_phy_timing *timing = &dsi->phy_timing;

	timing->lpx = (60 * data_rate_mhz / (8 * 1000)) + 1;
	timing->da_hs_prepare = (80 * data_rate_mhz + 4 * 1000) / 8000;
	timing->da_hs_zero = (170 * data_rate_mhz + 10 * 1000) / 8000 + 1 -
			     timing->da_hs_prepare;
	timing->da_hs_trail = timing->da_hs_prepare + 1;

	timing->ta_go = 4 * timing->lpx - 2;
	timing->ta_sure = timing->lpx + 2;
	timing->ta_get = 4 * timing->lpx;
	timing->da_hs_exit = 2 * timing->lpx + 1;

	timing->clk_hs_prepare = 70 * data_rate_mhz / (8 * 1000);
	timing->clk_hs_post = timing->clk_hs_prepare + 8;
	timing->clk_hs_trail = timing->clk_hs_prepare;
	timing->clk_hs_zero = timing->clk_hs_trail * 4;
	timing->clk_hs_exit = 2 * timing->clk_hs_trail;

	timcon0 = FIELD_PREP(LPX, timing->lpx) |
		  FIELD_PREP(HS_PREP, timing->da_hs_prepare) |
		  FIELD_PREP(HS_ZERO, timing->da_hs_zero) |
		  FIELD_PREP(HS_TRAIL, timing->da_hs_trail);

	timcon1 = FIELD_PREP(TA_GO, timing->ta_go) |
		  FIELD_PREP(TA_SURE, timing->ta_sure) |
		  FIELD_PREP(TA_GET, timing->ta_get) |
		  FIELD_PREP(DA_HS_EXIT, timing->da_hs_exit);

	timcon2 = FIELD_PREP(DA_HS_SYNC, 1) |
		  FIELD_PREP(CLK_ZERO, timing->clk_hs_zero) |
		  FIELD_PREP(CLK_TRAIL, timing->clk_hs_trail);

	timcon3 = FIELD_PREP(CLK_HS_PREP, timing->clk_hs_prepare) |
		  FIELD_PREP(CLK_HS_POST, timing->clk_hs_post) |
		  FIELD_PREP(CLK_HS_EXIT, timing->clk_hs_exit);

	writel(timcon0, dsi->regs + DSI_PHY_TIMECON0(dsi->driver_data));
	writel(timcon1, dsi->regs + DSI_PHY_TIMECON1(dsi->driver_data));
	writel(timcon2, dsi->regs + DSI_PHY_TIMECON2(dsi->driver_data));
	writel(timcon3, dsi->regs + DSI_PHY_TIMECON3(dsi->driver_data));
}

static void mtk_dsi_enable(struct mtk_dsi *dsi)
{
	mtk_dsi_mask(dsi, DSI_CON_CTRL(dsi->driver_data), DSI_EN, DSI_EN);
}

static void mtk_dsi_disable(struct mtk_dsi *dsi)
{
	mtk_dsi_mask(dsi, DSI_CON_CTRL(dsi->driver_data), DSI_EN, 0);
}

static void mtk_dsi_reset_engine(struct mtk_dsi *dsi)
{
	mtk_dsi_mask(dsi, DSI_CON_CTRL(dsi->driver_data), DSI_RESET, DSI_RESET);
	mtk_dsi_mask(dsi, DSI_CON_CTRL(dsi->driver_data), DSI_RESET, 0);
}

static void mtk_dsi_reset_dphy(struct mtk_dsi *dsi)
{
	mtk_dsi_mask(dsi, DSI_CON_CTRL(dsi->driver_data), DPHY_RESET, DPHY_RESET);
	mtk_dsi_mask(dsi, DSI_CON_CTRL(dsi->driver_data), DPHY_RESET, 0);
}

static void mtk_dsi_clk_ulp_mode_enter(struct mtk_dsi *dsi)
{
	mtk_dsi_mask(dsi, DSI_PHY_LCCON(dsi->driver_data), LC_HS_TX_EN, 0);
	mtk_dsi_mask(dsi, DSI_PHY_LCCON(dsi->driver_data), LC_ULPM_EN, 0);
}

static void mtk_dsi_clk_ulp_mode_leave(struct mtk_dsi *dsi)
{
	mtk_dsi_mask(dsi, DSI_PHY_LCCON(dsi->driver_data), LC_ULPM_EN, 0);
	mtk_dsi_mask(dsi, DSI_PHY_LCCON(dsi->driver_data), LC_WAKEUP_EN, LC_WAKEUP_EN);
	mtk_dsi_mask(dsi, DSI_PHY_LCCON(dsi->driver_data), LC_WAKEUP_EN, 0);
}

static void mtk_dsi_lane0_ulp_mode_enter(struct mtk_dsi *dsi)
{
	mtk_dsi_mask(dsi, DSI_PHY_LD0CON(dsi->driver_data), LD0_HS_TX_EN, 0);
	mtk_dsi_mask(dsi, DSI_PHY_LD0CON(dsi->driver_data), LD0_ULPM_EN, 0);
}

static void mtk_dsi_lane0_ulp_mode_leave(struct mtk_dsi *dsi)
{
	mtk_dsi_mask(dsi, DSI_PHY_LD0CON(dsi->driver_data), LD0_ULPM_EN, 0);
	mtk_dsi_mask(dsi, DSI_PHY_LD0CON(dsi->driver_data), LD0_WAKEUP_EN, LD0_WAKEUP_EN);
	mtk_dsi_mask(dsi, DSI_PHY_LD0CON(dsi->driver_data), LD0_WAKEUP_EN, 0);
}

static bool mtk_dsi_clk_hs_state(struct mtk_dsi *dsi)
{
	return readl(dsi->regs + DSI_PHY_LCCON(dsi->driver_data)) & LC_HS_TX_EN;
}

static void mtk_dsi_clk_hs_mode(struct mtk_dsi *dsi, bool enter)
{
	if (enter && !mtk_dsi_clk_hs_state(dsi))
		mtk_dsi_mask(dsi, DSI_PHY_LCCON(dsi->driver_data), LC_HS_TX_EN, LC_HS_TX_EN);
	else if (!enter && mtk_dsi_clk_hs_state(dsi))
		mtk_dsi_mask(dsi, DSI_PHY_LCCON(dsi->driver_data), LC_HS_TX_EN, 0);
}

static void mtk_dsi_set_mode(struct mtk_dsi *dsi)
{
	u32 vid_mode = CMD_MODE;

	if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO) {
		if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_BURST)
			vid_mode = BURST_MODE;
		else if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE)
			vid_mode = SYNC_PULSE_MODE;
		else
			vid_mode = SYNC_EVENT_MODE;
	}

	writel(vid_mode, dsi->regs + DSI_MODE_CTRL(dsi->driver_data));
}

static void mtk_dsi_set_vm_cmd(struct mtk_dsi *dsi)
{
	mtk_dsi_mask(dsi, dsi->driver_data->dsi_vm_cmd_con, VM_CMD_EN, VM_CMD_EN);
	mtk_dsi_mask(dsi, dsi->driver_data->dsi_vm_cmd_con, TS_VFP_EN, TS_VFP_EN);
}

static void mtk_dsi_rxtx_control(struct mtk_dsi *dsi)
{
	u32 regval, tmp_reg = 0;
	u8 i;

	/* Number of DSI lanes (max 4 lanes), each bit enables one DSI lane. */
	for (i = 0; i < dsi->lanes; i++)
		tmp_reg |= BIT(i);

	regval = FIELD_PREP(LANE_NUM, tmp_reg);

	if (dsi->mode_flags & MIPI_DSI_CLOCK_NON_CONTINUOUS)
		regval |= HSTX_CKLP_EN;

	if (dsi->mode_flags & MIPI_DSI_MODE_NO_EOT_PACKET)
		regval |= DIS_EOT;

	writel(regval, dsi->regs + DSI_TXRX_CTRL(dsi->driver_data));
}

static void mtk_dsi_ps_control(struct mtk_dsi *dsi, bool config_vact)
{
	u32 dsi_buf_bpp, ps_val, ps_wc, vact_nl;

	if (dsi->format == MIPI_DSI_FMT_RGB565)
		dsi_buf_bpp = 2;
	else
		dsi_buf_bpp = 3;

	/* Word count */
	ps_wc = FIELD_PREP(DSI_PS_WC, dsi->vm.hactive * dsi_buf_bpp);
	ps_val = ps_wc;

	/* Pixel Stream type */
	if (dsi->dsc_enable) {
		ps_val |= FIELD_PREP(DSI_PS_SEL, COMPRESSED_PIXEL_STREAM);
	} else {
		switch (dsi->format) {
		default:
			fallthrough;
		case MIPI_DSI_FMT_RGB888:
			ps_val |= FIELD_PREP(DSI_PS_SEL, PACKED_PS_24BIT_RGB888);
			break;
		case MIPI_DSI_FMT_RGB666:
			ps_val |= FIELD_PREP(DSI_PS_SEL, LOOSELY_PS_24BIT_RGB666);
			break;
		case MIPI_DSI_FMT_RGB666_PACKED:
			ps_val |= FIELD_PREP(DSI_PS_SEL, PACKED_PS_18BIT_RGB666);
			break;
		case MIPI_DSI_FMT_RGB565:
			ps_val |= FIELD_PREP(DSI_PS_SEL, PACKED_PS_16BIT_RGB565);
			break;
		}
	}
	if (config_vact) {
		vact_nl = FIELD_PREP(VACT_NL, dsi->vm.vactive);
		writel(vact_nl, dsi->regs + DSI_VACT_NL(dsi->driver_data));
		writel(ps_wc, dsi->regs + DSI_HSTX_CKL_WC(dsi->driver_data));
	}
	writel(ps_val, dsi->regs + DSI_PSCTRL(dsi->driver_data));
	if (dsi->is_cphy)
		mtk_dsi_mask(dsi, DSI_CMD_TYPE1_HS, CPHY_6BYE_EN, 0);
}

static void mtk_dsi_config_vdo_timing_per_frame_lp(struct mtk_dsi *dsi)
{
	u32 horizontal_sync_active_byte;
	u32 horizontal_backporch_byte;
	u32 horizontal_frontporch_byte;
	u32 hfp_byte_adjust = 0;;
	u32 v_active_adjust = 0;
	u32 cklp_wc_min_adjust = 0;
	u32 cklp_wc_max_adjust = 0;;
	u32 dsi_tmp_buf_bpp = 0;;
	unsigned int da_hs_trail;
	unsigned int ps_wc, hs_vb_ps_wc;
	u32 v_active_roundup, hstx_cklp_wc;
	u32 hstx_cklp_wc_max, hstx_cklp_wc_min;
	struct videomode *vm = &dsi->vm;
	struct mtk_phy_timing *timing = &dsi->phy_timing;
	u32 tmp = 0;

	if (dsi->format == MIPI_DSI_FMT_RGB565)
		dsi_tmp_buf_bpp = 2;
	else
		dsi_tmp_buf_bpp = 3;

	da_hs_trail = dsi->phy_timing.da_hs_trail;
	ps_wc = vm->hactive * dsi_tmp_buf_bpp;

	if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE) {
		horizontal_sync_active_byte =
			vm->hsync_len * dsi_tmp_buf_bpp - 10;
		horizontal_backporch_byte =
			vm->hback_porch * dsi_tmp_buf_bpp - 10;
		if (dsi->is_cphy) {
			if (vm->hsync_len * dsi_tmp_buf_bpp < 10 * dsi->lanes + 26 + 5)
				horizontal_sync_active_byte = 4;
			else
				horizontal_frontporch_byte =
					vm->hsync_len * dsi_tmp_buf_bpp - 10 * dsi->lanes - 26;

			if (vm->hback_porch * dsi_tmp_buf_bpp < 12 * dsi->lanes + 26 + 5)
				horizontal_backporch_byte = 4;
			else
				horizontal_backporch_byte =
					vm->hback_porch * dsi_tmp_buf_bpp -
					12 * dsi->lanes - 26;
		}
		hfp_byte_adjust = 12;
		v_active_adjust = 32 + horizontal_sync_active_byte;
		cklp_wc_min_adjust = 12 + 2 + 4 + horizontal_sync_active_byte;
		cklp_wc_max_adjust = 20 + 6 + 4 + horizontal_sync_active_byte;
	} else {
		horizontal_sync_active_byte = vm->hsync_len * dsi_tmp_buf_bpp - 4;
		horizontal_backporch_byte = (vm->hback_porch + vm->hsync_len) *
			dsi_tmp_buf_bpp - 10;
		cklp_wc_min_adjust = 4;
		cklp_wc_max_adjust = 12 + 4 + 4;
		if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_BURST) {
			hfp_byte_adjust = 18;
			v_active_adjust = 28;
		} else {
			hfp_byte_adjust = 12;
			v_active_adjust = 22;
		}
	}
	horizontal_frontporch_byte = vm->hfront_porch * dsi_tmp_buf_bpp - hfp_byte_adjust;
	if (dsi->is_cphy) {
		tmp = 8 * dsi->lanes + 28 + 2 * dsi->data_phy_cycle * dsi->lanes;
		if (vm->hfront_porch * dsi_tmp_buf_bpp < tmp + 9)
			horizontal_frontporch_byte = 8;
		else if ((vm->hfront_porch * dsi_tmp_buf_bpp > tmp  + 8) &&
				(vm->hfront_porch * dsi_tmp_buf_bpp < tmp + 2 *
				(timing->da_hs_trail + 1) * dsi->lanes - 6 * dsi->lanes - 14))
			horizontal_frontporch_byte = 2 * (timing->da_hs_trail + 1) *
						     dsi->lanes - 6 * dsi->lanes - 14;
		else
			horizontal_frontporch_byte = vm->hfront_porch * dsi_tmp_buf_bpp - tmp;
	}

	v_active_roundup = (v_active_adjust + horizontal_backporch_byte + ps_wc +
			   horizontal_frontporch_byte) % dsi->lanes;
	if (v_active_roundup)
		horizontal_backporch_byte += dsi->lanes - v_active_roundup;
	hstx_cklp_wc_min = (DIV_ROUND_UP(cklp_wc_min_adjust, dsi->lanes) + da_hs_trail + 1)
			   * dsi->lanes / 6 - 1;
	hstx_cklp_wc_max = (DIV_ROUND_UP((cklp_wc_max_adjust + horizontal_backporch_byte +
			   ps_wc), dsi->lanes) + da_hs_trail + 1) * dsi->lanes / 6 - 1;

	hstx_cklp_wc = FIELD_PREP(HSTX_CKL_WC, (hstx_cklp_wc_min + hstx_cklp_wc_max) / 2);
	writel(hstx_cklp_wc, dsi->regs + DSI_HSTX_CKL_WC(dsi->driver_data));

	if (dsi->is_cphy)
		hs_vb_ps_wc = ps_wc - (dsi->phy_timing.lpx + dsi->phy_timing.da_hs_exit +
			dsi->phy_timing.da_hs_prepare + dsi->phy_timing.da_hs_zero + 4)
			* 2 * dsi->lanes;
	else
		hs_vb_ps_wc = ps_wc - (dsi->phy_timing.lpx + dsi->phy_timing.da_hs_exit +
			dsi->phy_timing.da_hs_prepare + dsi->phy_timing.da_hs_zero + 2)
			* dsi->lanes;
	horizontal_frontporch_byte |= FIELD_PREP(HFP_HS_EN, 1) |
				      FIELD_PREP(HFP_HS_VB_PS_WC, hs_vb_ps_wc);

	writel(horizontal_sync_active_byte, dsi->regs + DSI_HSA_WC(dsi->driver_data));
	writel(horizontal_backporch_byte, dsi->regs + DSI_HBP_WC(dsi->driver_data));
	writel(horizontal_frontporch_byte, dsi->regs + DSI_HFP_WC(dsi->driver_data));
}

static void mtk_dsi_config_vdo_timing_per_line_lp(struct mtk_dsi *dsi)
{
	u32 horizontal_sync_active_byte;
	u32 horizontal_backporch_byte;
	u32 horizontal_frontporch_byte;
	u32 horizontal_front_back_byte;
	u32 data_phy_cycles_byte;
	u32 dsi_tmp_buf_bpp, data_phy_cycles;
	u32 delta;
	struct mtk_phy_timing *timing = &dsi->phy_timing;
	struct videomode *vm = &dsi->vm;

	if (dsi->format == MIPI_DSI_FMT_RGB565)
		dsi_tmp_buf_bpp = 2;
	else
		dsi_tmp_buf_bpp = 3;

	horizontal_sync_active_byte = (vm->hsync_len * dsi_tmp_buf_bpp - 10);

	if (dsi->mode_flags & MIPI_DSI_MODE_VIDEO_SYNC_PULSE)
		horizontal_backporch_byte = vm->hback_porch * dsi_tmp_buf_bpp - 10;
	else
		horizontal_backporch_byte = (vm->hback_porch + vm->hsync_len) *
					    dsi_tmp_buf_bpp - 10;

	data_phy_cycles = timing->lpx + timing->da_hs_prepare +
			  timing->da_hs_zero + timing->da_hs_exit + 3;

	delta = dsi->mode_flags & MIPI_DSI_MODE_VIDEO_BURST ? 18 : 12;
	delta += dsi->mode_flags & MIPI_DSI_MODE_NO_EOT_PACKET ? 0 : 2;

	horizontal_frontporch_byte = vm->hfront_porch * dsi_tmp_buf_bpp;
	horizontal_front_back_byte = horizontal_frontporch_byte + horizontal_backporch_byte;
	data_phy_cycles_byte = data_phy_cycles * dsi->lanes + delta;

	if (horizontal_front_back_byte > data_phy_cycles_byte) {
		horizontal_frontporch_byte -= data_phy_cycles_byte *
					      horizontal_frontporch_byte /
					      horizontal_front_back_byte;

		horizontal_backporch_byte -= data_phy_cycles_byte *
					     horizontal_backporch_byte /
					     horizontal_front_back_byte;
	} else {
		DRM_WARN("HFP + HBP less than d-phy, FPS will under 60Hz\n");
	}

	if ((dsi->mode_flags & MIPI_DSI_HS_PKT_END_ALIGNED) &&
	    (dsi->lanes == 4)) {
		horizontal_sync_active_byte =
			roundup(horizontal_sync_active_byte, dsi->lanes) - 2;
		horizontal_frontporch_byte =
			roundup(horizontal_frontporch_byte, dsi->lanes) - 2;
		horizontal_backporch_byte =
			roundup(horizontal_backporch_byte, dsi->lanes) - 2;
		horizontal_backporch_byte -=
			(vm->hactive * dsi_tmp_buf_bpp + 2) % dsi->lanes;
	}

	writel(horizontal_sync_active_byte, dsi->regs + DSI_HSA_WC(dsi->driver_data));
	writel(horizontal_backporch_byte, dsi->regs + DSI_HBP_WC(dsi->driver_data));
	writel(horizontal_frontporch_byte, dsi->regs + DSI_HFP_WC(dsi->driver_data));
}

static void mtk_dsi_config_vdo_timing(struct mtk_dsi *dsi)
{
	struct videomode *vm = &dsi->vm;

	writel(vm->vsync_len, dsi->regs + DSI_VSA_NL(dsi->driver_data));
	writel(vm->vback_porch, dsi->regs + DSI_VBP_NL(dsi->driver_data));
	writel(vm->vfront_porch, dsi->regs + DSI_VFP_NL(dsi->driver_data));
	writel(vm->vactive, dsi->regs + DSI_VACT_NL(dsi->driver_data));

	if (dsi->driver_data->has_size_ctl)
		writel(FIELD_PREP(DSI_HEIGHT, vm->vactive) |
			FIELD_PREP(DSI_WIDTH, vm->hactive),
			dsi->regs + DSI_SIZE_CON(dsi->driver_data));

	if (dsi->driver_data->support_per_frame_lp)
		mtk_dsi_config_vdo_timing_per_frame_lp(dsi);
	else
		mtk_dsi_config_vdo_timing_per_line_lp(dsi);

	mtk_dsi_ps_control(dsi, false);
}

static void mtk_dsi_start(struct mtk_dsi *dsi)
{
	writel(0, dsi->regs + DSI_START);
	writel(1, dsi->regs + DSI_START);
}

static void mtk_dsi_stop(struct mtk_dsi *dsi)
{
	writel(0, dsi->regs + DSI_START);
}

static void mtk_dsi_set_cmd_mode(struct mtk_dsi *dsi)
{
	writel(CMD_MODE, dsi->regs + DSI_MODE_CTRL(dsi->driver_data));
}

static void mtk_dsi_set_interrupt_enable(struct mtk_dsi *dsi)
{
	u32 inten = LPRX_RD_RDY_INT_FLAG | CMD_DONE_INT_FLAG | VM_DONE_INT_FLAG;

	writel(inten, dsi->regs + DSI_INTEN);
}

static void mtk_dsi_irq_data_set(struct mtk_dsi *dsi, u32 irq_bit)
{
	dsi->irq_data |= irq_bit;
}

static void mtk_dsi_irq_data_clear(struct mtk_dsi *dsi, u32 irq_bit)
{
	dsi->irq_data &= ~irq_bit;
}

static s32 mtk_dsi_wait_for_irq_done(struct mtk_dsi *dsi, u32 irq_flag,
				     unsigned int timeout)
{
	s32 ret = 0;
	unsigned long jiffies = msecs_to_jiffies(timeout);

	ret = wait_event_interruptible_timeout(dsi->irq_wait_queue,
					       dsi->irq_data & irq_flag,
					       jiffies);
	if (ret == 0) {
		DRM_WARN("Wait DSI IRQ(0x%08x) Timeout\n", irq_flag);

		mtk_dsi_enable(dsi);
		mtk_dsi_reset_engine(dsi);
	}

	return ret;
}

static int mtk_dsi_calculate_rw_times(struct mtk_dsi *dsi, u32 width, u32 height)
{
	u32 rw_times = 0;
	u32 ps_wc = 0, in_width = 16;
	u32 dsi_buf_bpp;

	dsi_buf_bpp = mtk_get_dsi_buffer_bpp(dsi->format);
	ps_wc = width * dsi_buf_bpp;

	rw_times = DIV_ROUND_UP(ps_wc * height, in_width);

	return rw_times;
}

static void mtk_dsi_tx_buf_rw(struct mtk_dsi *dsi)
{
	struct device *dev = dsi->host.dev;
	u32 mmsys_clk_rate;
	u32 tmp = 0, rw_times;
	u32 preultra_hi, preultra_lo, ultra_hi, ultra_lo, urgent_hi, urgent_lo;
	u32 fill_rate;
	u32 sodi_hi, sodi_lo;
	u32 sram_unit, buffer_unit;
	u32 urgent_lo_fifo_us, urgent_hi_fifo_us, output_valid_us;
	u32 dsi_buf_bpp;
	/* line counter mode for vdo mode */
	int dli_relay_1tnp = 2;
	u32 buf_con = 1554;
	u32 data_rate_per_buf;

	dsi_buf_bpp = mtk_get_dsi_buffer_bpp(dsi->format);
	mmsys_clk_rate = dsi->vm.pixelclock / 1000000;
	if (!mmsys_clk_rate) {
		dev_err(dev, "mmclk is zero\n");
		mmsys_clk_rate = MMSYS_CLK_RATE_DEFAULT;
	}

	buffer_unit = dsi->driver_data->buffer_unit;
	sram_unit = dsi->driver_data->sram_unit;
	urgent_lo_fifo_us = dsi->driver_data->urgent_lo_fifo_us ?
				dsi->driver_data->urgent_lo_fifo_us : 11;
	urgent_hi_fifo_us = dsi->driver_data->urgent_hi_fifo_us ?
				dsi->driver_data->urgent_hi_fifo_us : 12;
	output_valid_us = dsi->driver_data->output_valid_fifo_us ?
				dsi->driver_data->output_valid_fifo_us : 25;
	if (dsi->is_cphy)
		data_rate_per_buf = dsi->data_rate * 2 * dsi->lanes / 7 / buffer_unit;
	else
		data_rate_per_buf = dsi->data_rate * dsi->lanes / 8 / buffer_unit;

	if (dsi->driver_data->support_per_frame_lp)
		mtk_dsi_mask(dsi, DSI_CON_CTRL(dsi->driver_data),
			     DSI_CM_MODE_WAIT_DATA_EVERY_LINE_EN,
			     DSI_CM_MODE_WAIT_DATA_EVERY_LINE_EN);
	else
		mtk_dsi_mask(dsi, DSI_CON_CTRL(dsi->driver_data),
		             DSI_CM_MODE_WAIT_DATA_EVERY_LINE_EN, 0);
	if (dsi->is_cphy)
		tmp = 25 * data_rate_per_buf;
	else
		tmp = output_valid_us * data_rate_per_buf;

	/* check output valid threshold exceed FIFO size if FIFO size is pre-defined */
	if (buf_con)
		tmp = min(tmp, buf_con - 1);

	rw_times = mtk_dsi_calculate_rw_times(dsi, dsi->vm.vactive, dsi->vm.hactive);
	mtk_dsi_mask(dsi, DSI_BUF_CON1,  GENMASK(14, 0), tmp);
	mtk_dsi_mask(dsi, DSI_DEBUG_SEL, MM_RST_SEL, MM_RST_SEL);

	/* enable ultra signal between SOF to VACT */
	mtk_dsi_mask(dsi, DSI_RESERVED, DSI_VDE_BLOCK_ULTRA, 0);
	/* 1TNP */
	fill_rate = mmsys_clk_rate * dli_relay_1tnp * dsi_buf_bpp / buffer_unit;

	if (buf_con)
		tmp = buf_con * sram_unit / buffer_unit;
	else
		tmp = (readl(dsi->regs + DSI_BUF_CON1) >> 16) * sram_unit / buffer_unit;

	sodi_hi = tmp - (12 * (fill_rate - data_rate_per_buf) / 10);
	/*dsi->driver_data*/
	sodi_lo = (23 + 5) * data_rate_per_buf;
	preultra_hi = 36 * data_rate_per_buf;
	preultra_lo = 35 * data_rate_per_buf;
	ultra_hi = 26 * data_rate_per_buf;
	ultra_lo = 25 * data_rate_per_buf;
	urgent_hi = urgent_hi_fifo_us * data_rate_per_buf;
	urgent_lo = urgent_lo_fifo_us * data_rate_per_buf;

	writel((sodi_hi & DSI_BUF_MASK), dsi->regs + DSI_BUF_SODI_HIGH);
	writel((sodi_lo & DSI_BUF_MASK), dsi->regs + DSI_BUF_SODI_LOW);
	writel((preultra_hi & DSI_BUF_MASK), dsi->regs + DSI_BUF_PREULTRA_HIGH);
	writel((preultra_lo & DSI_BUF_MASK), dsi->regs + DSI_BUF_PREULTRA_LOW);
	writel((ultra_hi & DSI_BUF_MASK), dsi->regs + DSI_BUF_ULTRA_HIGH);
	writel((ultra_lo & DSI_BUF_MASK), dsi->regs + DSI_BUF_ULTRA_LOW);
	writel((urgent_hi & DSI_BUF_MASK), dsi->regs + DSI_BUF_URGENT_HIGH);
	writel((urgent_lo & DSI_BUF_MASK), dsi->regs + DSI_BUF_URGENT_LOW);
	writel(rw_times, dsi->regs + DSI_TX_BUF_RW_TIMES);
	mtk_dsi_mask(dsi, DSI_BUF_CON0, BUF_BUF_EN, BUF_BUF_EN);
}

static irqreturn_t mtk_dsi_irq(int irq, void *dev_id)
{
	struct mtk_dsi *dsi = dev_id;
	u32 status, tmp;
	u32 flag = LPRX_RD_RDY_INT_FLAG | CMD_DONE_INT_FLAG | VM_DONE_INT_FLAG;

	status = readl(dsi->regs + DSI_INTSTA) & flag;

	if (status) {
		do {
			mtk_dsi_mask(dsi, DSI_RACK(dsi->driver_data), RACK, RACK);
			tmp = readl(dsi->regs + DSI_INTSTA);
		} while (tmp & DSI_BUSY);

		mtk_dsi_mask(dsi, DSI_INTSTA, status, 0);
		mtk_dsi_irq_data_set(dsi, status);
		wake_up_interruptible(&dsi->irq_wait_queue);
	}

	return IRQ_HANDLED;
}

static s32 mtk_dsi_switch_to_cmd_mode(struct mtk_dsi *dsi, u8 irq_flag, u32 t)
{
	mtk_dsi_irq_data_clear(dsi, irq_flag);
	mtk_dsi_set_cmd_mode(dsi);

	if (!mtk_dsi_wait_for_irq_done(dsi, irq_flag, t)) {
		DRM_ERROR("failed to switch cmd mode\n");
		return -ETIME;
	} else {
		return 0;
	}
}

static int mtk_dsi_poweron(struct mtk_dsi *dsi)
{
	struct device *dev = dsi->host.dev;
	int ret;
	u32 bit_per_pixel;

	if (++dsi->refcount != 1)
		return 0;

	ret = mipi_dsi_pixel_format_to_bpp(dsi->format);
	if (ret < 0) {
		dev_err(dev, "Unknown MIPI DSI format %d\n", dsi->format);
		return ret;
	}
	bit_per_pixel = ret;

	dsi->data_rate = DIV_ROUND_UP_ULL(dsi->vm.pixelclock * bit_per_pixel,
					  dsi->lanes);
	if (dsi->is_cphy)
		dsi->data_rate = DIV_ROUND_UP_ULL(dsi->vm.pixelclock * bit_per_pixel * 7,
					  dsi->lanes * 16);
	else
		dsi->data_rate = DIV_ROUND_UP_ULL(dsi->vm.pixelclock * bit_per_pixel,
					  dsi->lanes);
	if (dsi->dsc_enable) {
		dsi->vm.hactive = DIV_ROUND_UP_ULL(dsi->vm.hactive, 3);
		dsi->vm.pixelclock = (dsi->vm.hactive + dsi->vm.hfront_porch
			+ dsi->vm.hback_porch
			+ dsi->vm.hsync_len) *
			dsi->vm.pixelclock /
			(dsi->vm.hactive * 3
			+ dsi->vm.hfront_porch
			+ dsi->vm.hback_porch
			+ dsi->vm.hsync_len);
		dsi->data_rate = DIV_ROUND_UP_ULL(dsi->vm.pixelclock * bit_per_pixel,
					  dsi->lanes);
	}
	pm_runtime_get_sync(dsi->host.dev);
	ret = clk_set_rate(dsi->hs_clk, dsi->data_rate);
	if (ret < 0) {
		dev_err(dev, "Failed to set data rate: %d\n", ret);
		goto err_refcount;
	}

	phy_power_on(dsi->phy);

	ret = clk_prepare_enable(dsi->engine_clk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable engine clock: %d\n", ret);
		goto err_phy_power_off;
	}

	ret = clk_prepare_enable(dsi->digital_clk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable digital clock: %d\n", ret);
		goto err_disable_engine_clk;
	}

	mtk_dsi_enable(dsi);

	if (dsi->driver_data->has_shadow_ctl)
		writel(FORCE_COMMIT | BYPASS_SHADOW,
		       dsi->regs + DSI_SHADOW_DEBUG(dsi->driver_data));

	mtk_dsi_reset_engine(dsi);
	if (dsi->is_cphy)
		mtk_dsi_cphy_timconfig(dsi);
	else
		mtk_dsi_phy_timconfig(dsi);
	if (dsi->driver_data->support_dsi_buffer)
		mtk_dsi_tx_buf_rw(dsi);

	mtk_dsi_ps_control(dsi, true);
	mtk_dsi_set_vm_cmd(dsi);
	mtk_dsi_config_vdo_timing(dsi);
	mtk_dsi_set_interrupt_enable(dsi);

	return 0;
err_disable_engine_clk:
	clk_disable_unprepare(dsi->engine_clk);
err_phy_power_off:
	phy_power_off(dsi->phy);
err_refcount:
	dsi->refcount--;
	return ret;
}

static void mtk_dsi_poweroff(struct mtk_dsi *dsi)
{
	if (WARN_ON(dsi->refcount == 0))
		return;

	if (--dsi->refcount != 0)
		return;

	/*
	 * mtk_dsi_stop() and mtk_dsi_start() is asymmetric, since
	 * mtk_dsi_stop() should be called after mtk_crtc_atomic_disable(),
	 * which needs irq for vblank, and mtk_dsi_stop() will disable irq.
	 * mtk_dsi_start() needs to be called in mtk_output_dsi_enable(),
	 * after dsi is fully set.
	 */
	mtk_dsi_stop(dsi);

	mtk_dsi_switch_to_cmd_mode(dsi, VM_DONE_INT_FLAG, 500);
	mtk_dsi_reset_engine(dsi);
	mtk_dsi_lane0_ulp_mode_enter(dsi);
	mtk_dsi_clk_ulp_mode_enter(dsi);
	/* set the lane number as 0 to pull down mipi */
	writel(0, dsi->regs + DSI_TXRX_CTRL(dsi->driver_data));

	mtk_dsi_disable(dsi);

	clk_disable_unprepare(dsi->engine_clk);
	clk_disable_unprepare(dsi->digital_clk);

	phy_power_off(dsi->phy);
	pm_runtime_put_sync(dsi->host.dev);

	dsi->lanes_ready = false;
}

static void mtk_dsi_lane_ready(struct mtk_dsi *dsi)
{
	if (!dsi->lanes_ready) {
		dsi->lanes_ready = true;
		mtk_dsi_rxtx_control(dsi);
		usleep_range(30, 100);
		mtk_dsi_reset_dphy(dsi);
		mtk_dsi_clk_ulp_mode_leave(dsi);
		mtk_dsi_lane0_ulp_mode_leave(dsi);
		mtk_dsi_clk_hs_mode(dsi, 0);
		usleep_range(1000, 3000);
		/* The reaction time after pulling up the mipi signal for dsi_rx */
	}
}

static void mtk_output_dsi_enable(struct mtk_dsi *dsi)
{
	if (dsi->enabled)
		return;

	mtk_dsi_lane_ready(dsi);
	mtk_dsi_set_mode(dsi);
	mtk_dsi_clk_hs_mode(dsi, 1);

	mtk_dsi_start(dsi);

	dsi->enabled = true;
}

static void mtk_output_dsi_disable(struct mtk_dsi *dsi)
{
	if (!dsi->enabled)
		return;

	dsi->enabled = false;
}

static int mtk_dsi_bridge_attach(struct drm_bridge *bridge,
				 enum drm_bridge_attach_flags flags)
{
	struct mtk_dsi *dsi = bridge_to_dsi(bridge);

	/* Attach the panel or bridge to the dsi bridge */
	return drm_bridge_attach(bridge->encoder, dsi->next_bridge,
				 &dsi->bridge, flags);
}

static void mtk_dsi_bridge_mode_set(struct drm_bridge *bridge,
				    const struct drm_display_mode *mode,
				    const struct drm_display_mode *adjusted)
{
	struct mtk_dsi *dsi = bridge_to_dsi(bridge);

	drm_display_mode_to_videomode(adjusted, &dsi->vm);
}

static void mtk_dsi_bridge_atomic_disable(struct drm_bridge *bridge,
					  struct drm_bridge_state *old_bridge_state)
{
	struct mtk_dsi *dsi = bridge_to_dsi(bridge);

	mtk_output_dsi_disable(dsi);
}

static void mtk_dsi_bridge_atomic_enable(struct drm_bridge *bridge,
					 struct drm_bridge_state *old_bridge_state)
{
	struct mtk_dsi *dsi = bridge_to_dsi(bridge);

	if (dsi->refcount == 0)
		return;

	mtk_output_dsi_enable(dsi);
}

static void mtk_dsi_bridge_atomic_pre_enable(struct drm_bridge *bridge,
					     struct drm_bridge_state *old_bridge_state)
{
	struct mtk_dsi *dsi = bridge_to_dsi(bridge);
	int ret;

	ret = mtk_dsi_poweron(dsi);
	if (ret < 0)
		DRM_ERROR("failed to power on dsi\n");
}

static void mtk_dsi_bridge_atomic_post_disable(struct drm_bridge *bridge,
					       struct drm_bridge_state *old_bridge_state)
{
	struct mtk_dsi *dsi = bridge_to_dsi(bridge);

	mtk_dsi_poweroff(dsi);
}

static enum drm_mode_status
mtk_dsi_bridge_mode_valid(struct drm_bridge *bridge,
			  const struct drm_display_info *info,
			  const struct drm_display_mode *mode)
{
	struct mtk_dsi *dsi = bridge_to_dsi(bridge);
	int bpp;

	bpp = mipi_dsi_pixel_format_to_bpp(dsi->format);
	if (bpp < 0)
		return MODE_ERROR;

	if (dsi->dsc_config && dsi->dsc_config->dsc_version_major == 1) {
		if (mode->hdisplay > 3840)
			return MODE_BAD_HVALUE;
	} else {
		if (mode->hdisplay > 4096)
			return MODE_BAD_HVALUE;
	}
	if (mode->vdisplay > 2560)
		return MODE_BAD_VVALUE;

	if (mode->clock * bpp / dsi->lanes > dsi->driver_data->max_linkrate_kbps)
		return MODE_CLOCK_HIGH;

	return MODE_OK;
}

static const struct drm_bridge_funcs mtk_dsi_bridge_funcs = {
	.attach = mtk_dsi_bridge_attach,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_disable = mtk_dsi_bridge_atomic_disable,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_enable = mtk_dsi_bridge_atomic_enable,
	.atomic_pre_enable = mtk_dsi_bridge_atomic_pre_enable,
	.atomic_post_disable = mtk_dsi_bridge_atomic_post_disable,
	.atomic_reset = drm_atomic_helper_bridge_reset,
	.mode_valid = mtk_dsi_bridge_mode_valid,
	.mode_set = mtk_dsi_bridge_mode_set,
};

void mtk_dsi_ddp_start(struct device *dev)
{
	struct mtk_dsi *dsi = dev_get_drvdata(dev);

	mtk_dsi_poweron(dsi);
}

void mtk_dsi_ddp_stop(struct device *dev)
{
	struct mtk_dsi *dsi = dev_get_drvdata(dev);

	mtk_dsi_poweroff(dsi);
}

static int mtk_dsi_encoder_init(struct drm_device *drm, struct mtk_dsi *dsi)
{
	int ret;

	ret = drm_simple_encoder_init(drm, &dsi->mtk_encoder.encoder,
				      DRM_MODE_ENCODER_DSI);
	if (ret) {
		DRM_ERROR("Failed to encoder init to drm\n");
		return ret;
	}

	ret = mtk_find_possible_crtcs(drm, dsi->host.dev);
	if (ret < 0)
		goto err_cleanup_encoder;
	dsi->mtk_encoder.encoder.possible_crtcs = ret;

	ret = drm_bridge_attach(&dsi->mtk_encoder.encoder, &dsi->bridge, NULL,
				DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (ret)
		goto err_cleanup_encoder;

	dsi->connector = drm_bridge_connector_init(drm, &dsi->mtk_encoder.encoder);
	if (IS_ERR(dsi->connector)) {
		DRM_ERROR("Unable to create bridge connector\n");
		ret = PTR_ERR(dsi->connector);
		goto err_cleanup_encoder;
	}
	drm_connector_attach_encoder(dsi->connector, &dsi->mtk_encoder.encoder);

	return 0;

err_cleanup_encoder:
	drm_encoder_cleanup(&dsi->mtk_encoder.encoder);
	return ret;
}

unsigned int mtk_dsi_encoder_index(struct device *dev)
{
	struct mtk_dsi *dsi = dev_get_drvdata(dev);
	unsigned int encoder_index = drm_encoder_index(&dsi->mtk_encoder.encoder);

	dev_dbg(dev, "encoder index:%d\n", encoder_index);
	return encoder_index;
}

void mtk_dsi_get_dsc_info(struct device *dev, struct dsc_info *dsc_info)
{
	struct mtk_dsi *dsi = dev_get_drvdata(dev);

	if (!dsi || !dsc_info) {
		dev_err(dev,"Invalid dsi or dsc_info is NULL\n");
		return;
	}
	if (!dsi->dsc_config || dsi->dsc_config->dsc_version_major == 0) {
		dev_err(dev,"Invalid dsc version or dsc_config is NULL\n");
		dsc_info->compression_enable = false;
		dsi->dsc_enable = false;
		memset(&dsc_info->dsc_config, 0, sizeof(dsc_info->dsc_config));
		return;
	}
	dsc_info->compression_enable = true;
	dsi->dsc_enable = true;
	memcpy(&dsc_info->dsc_config, dsi->dsc_config, sizeof(dsc_info->dsc_config));
}

void mtk_dsi_get_hrt_bw_by_datarate(struct device *dev, unsigned int *bw_base)
{
	struct mtk_dsi *dsi = dev_get_drvdata(dev);
	int htotal, vtotal, vrefresh;
	struct drm_display_mode mode;

	drm_display_mode_from_videomode(&dsi->vm, &mode);
	htotal = mode.htotal;
	vtotal = mode.vtotal;
	vrefresh = drm_mode_vrefresh(&mode);

	*bw_base = (unsigned int)div_u64((unsigned long long)vtotal * htotal * vrefresh * 4,
					 1000000);
}

static int mtk_dsi_bind(struct device *dev, struct device *master, void *data)
{
	int ret;
	struct drm_device *drm = data;
	struct mtk_dsi *dsi = dev_get_drvdata(dev);

	ret = mtk_dsi_encoder_init(drm, dsi);
	if (ret)
		return ret;

	return device_reset_optional(dev);
}

static void mtk_dsi_unbind(struct device *dev, struct device *master,
			   void *data)
{
	struct mtk_dsi *dsi = dev_get_drvdata(dev);

	drm_encoder_cleanup(&dsi->mtk_encoder.encoder);
}

static const struct component_ops mtk_dsi_component_ops = {
	.bind = mtk_dsi_bind,
	.unbind = mtk_dsi_unbind,
};

static int mtk_dsi_host_attach(struct mipi_dsi_host *host,
			       struct mipi_dsi_device *device)
{
	struct mtk_dsi *dsi = host_to_dsi(host);
	struct device *dev = host->dev;
	int ret;

	dsi->lanes = device->lanes;
	dsi->format = device->format;
	dsi->mode_flags = device->mode_flags;
	dsi->dsc_config = device->dsc;
	dsi->next_bridge = devm_drm_of_get_bridge(dev, dev->of_node, 1, 0);
	if (IS_ERR(dsi->next_bridge)) {
		ret = PTR_ERR(dsi->next_bridge);
		if (ret == -EPROBE_DEFER)
			return ret;

		/* Old devicetree has only one endpoint */
		dsi->next_bridge = devm_drm_of_get_bridge(dev, dev->of_node, 0, 0);
		if (IS_ERR(dsi->next_bridge))
			return PTR_ERR(dsi->next_bridge);
	}

	drm_bridge_add(&dsi->bridge);

	ret = component_add(host->dev, &mtk_dsi_component_ops);
	if (ret) {
		DRM_ERROR("failed to add dsi_host component: %d\n", ret);
		drm_bridge_remove(&dsi->bridge);
		return ret;
	}

	return 0;
}

static int mtk_dsi_host_detach(struct mipi_dsi_host *host,
			       struct mipi_dsi_device *device)
{
	struct mtk_dsi *dsi = host_to_dsi(host);

	component_del(host->dev, &mtk_dsi_component_ops);
	drm_bridge_remove(&dsi->bridge);
	return 0;
}

static void mtk_dsi_wait_for_idle(struct mtk_dsi *dsi)
{
	int ret;
	u32 val;

	ret = readl_poll_timeout(dsi->regs + DSI_INTSTA, val, !(val & DSI_BUSY),
				 4, 2000000);
	if (ret) {
		DRM_WARN("polling dsi wait not busy timeout!\n");

		mtk_dsi_enable(dsi);
		mtk_dsi_reset_engine(dsi);
	}
}

static u32 mtk_dsi_recv_cnt(u8 type, u8 *read_data)
{
	switch (type) {
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_1BYTE:
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_1BYTE:
		return 1;
	case MIPI_DSI_RX_GENERIC_SHORT_READ_RESPONSE_2BYTE:
	case MIPI_DSI_RX_DCS_SHORT_READ_RESPONSE_2BYTE:
		return 2;
	case MIPI_DSI_RX_GENERIC_LONG_READ_RESPONSE:
	case MIPI_DSI_RX_DCS_LONG_READ_RESPONSE:
		return read_data[1] + read_data[2] * 16;
	case MIPI_DSI_RX_ACKNOWLEDGE_AND_ERROR_REPORT:
		DRM_INFO("type is 0x02, try again\n");
		break;
	default:
		DRM_INFO("type(0x%x) not recognized\n", type);
		break;
	}

	return 0;
}

static void mtk_dsi_cmdq(struct mtk_dsi *dsi, const struct mipi_dsi_msg *msg)
{
	const char *tx_buf = msg->tx_buf;
	u8 config, cmdq_size, cmdq_off, type = msg->type;
	u32 reg_val, cmdq_mask, i;
	u32 reg_cmdq_off = dsi->driver_data->reg_cmdq_off;

	if (MTK_DSI_HOST_IS_READ(type))
		config = BTA;
	else
		config = (msg->tx_len > 2) ? LONG_PACKET : SHORT_PACKET;

	if (msg->tx_len > 2) {
		cmdq_size = 1 + (msg->tx_len + 3) / 4;
		cmdq_off = 4;
		cmdq_mask = CONFIG | DATA_ID | DATA_0 | DATA_1;
		reg_val = (msg->tx_len << 16) | (type << 8) | config;
	} else {
		cmdq_size = 1;
		cmdq_off = 2;
		cmdq_mask = CONFIG | DATA_ID;
		reg_val = (type << 8) | config;
	}
	writel(0x0, dsi->regs + reg_cmdq_off);
	for (i = 0; i < msg->tx_len; i++)
		mtk_dsi_mask(dsi, (reg_cmdq_off + cmdq_off + i) & (~0x3U),
			     (0xffUL << (((i + cmdq_off) & 3U) * 8U)),
			     tx_buf[i] << (((i + cmdq_off) & 3U) * 8U));

	mtk_dsi_mask(dsi, reg_cmdq_off, cmdq_mask, reg_val);
	mtk_dsi_mask(dsi, DSI_CMDQ_SIZE(dsi->driver_data), CMDQ_SIZE, cmdq_size);
	if (dsi->driver_data->cmdq_long_packet_ctl) {
		/* Disable setting cmdq_size automatically for long packets */
		mtk_dsi_mask(dsi, DSI_CMDQ_SIZE(dsi->driver_data), CMDQ_SIZE_SEL, CMDQ_SIZE_SEL);
	}
}

static ssize_t mtk_dsi_host_send_cmd(struct mtk_dsi *dsi,
				     const struct mipi_dsi_msg *msg, u8 flag)
{
	mtk_dsi_wait_for_idle(dsi);
	mtk_dsi_irq_data_clear(dsi, flag);
	mtk_dsi_cmdq(dsi, msg);
	mtk_dsi_start(dsi);
	/* unset DSI_START after trigger */
	writel(0, dsi->regs + DSI_START);

	if (!mtk_dsi_wait_for_irq_done(dsi, flag, 2000))
		return -ETIME;
	else
		return 0;
}

static ssize_t mtk_dsi_host_transfer(struct mipi_dsi_host *host,
				     const struct mipi_dsi_msg *msg)
{
	struct mtk_dsi *dsi = host_to_dsi(host);
	ssize_t recv_cnt;
	u8 read_data[16];
	void *src_addr;
	u8 irq_flag = CMD_DONE_INT_FLAG;
	u32 dsi_mode;
	int ret, i;

	dsi_mode = readl(dsi->regs + DSI_MODE_CTRL(dsi->driver_data));
	if (dsi_mode & MODE) {
		mtk_dsi_stop(dsi);
		ret = mtk_dsi_switch_to_cmd_mode(dsi, VM_DONE_INT_FLAG, 500);
		if (ret)
			goto restore_dsi_mode;
	}

	mtk_dsi_lane_ready(dsi);

	if (MTK_DSI_HOST_IS_READ(msg->type)) {
		struct mipi_dsi_msg set_rd_msg = {
		.tx_buf = (u8 [1]) { msg->rx_len},
		.tx_len = 0x1,
		.type = MIPI_DSI_SET_MAXIMUM_RETURN_PACKET_SIZE,
		};

		if (mtk_dsi_host_send_cmd(dsi, &set_rd_msg, irq_flag) < 0) {
			goto restore_dsi_mode;
		}
		irq_flag |= LPRX_RD_RDY_INT_FLAG;
	}
	ret = mtk_dsi_host_send_cmd(dsi, msg, irq_flag);
	if (ret)
		goto restore_dsi_mode;

	if (!MTK_DSI_HOST_IS_READ(msg->type)) {
		recv_cnt = 0;
		goto restore_dsi_mode;
	}

	if (!msg->rx_buf) {
		DRM_ERROR("dsi receive buffer size may be NULL\n");
		ret = -EINVAL;
		goto restore_dsi_mode;
	}

	for (i = 0; i < 16; i++)
		*(read_data + i) = readb(dsi->regs + DSI_RX_DATA0(dsi->driver_data) + i);

	recv_cnt = mtk_dsi_recv_cnt(read_data[0], read_data);

	if (recv_cnt > 2)
		src_addr = &read_data[4];
	else
		src_addr = &read_data[1];

	if (recv_cnt > 10)
		recv_cnt = 10;

	if (recv_cnt > msg->rx_len)
		recv_cnt = msg->rx_len;

	if (recv_cnt)
		memcpy(msg->rx_buf, src_addr, recv_cnt);

	DRM_INFO("dsi get %zd byte data from the panel address(0x%x)\n",
		 recv_cnt, *((u8 *)(msg->tx_buf)));

restore_dsi_mode:
	if (dsi_mode & MODE) {
		mtk_dsi_set_mode(dsi);
		mtk_dsi_start(dsi);
	}

	return ret < 0 ? ret : recv_cnt;
}

static const struct mipi_dsi_host_ops mtk_dsi_ops = {
	.attach = mtk_dsi_host_attach,
	.detach = mtk_dsi_host_detach,
	.transfer = mtk_dsi_host_transfer,
};

static int mtk_dsi_probe(struct platform_device *pdev)
{
	struct mtk_dsi *dsi;
	struct device *dev = &pdev->dev;
	struct resource *regs;
	int irq_num;
	int ret;

	dsi = devm_kzalloc(dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return -ENOMEM;

	dsi->driver_data = of_device_get_match_data(dev);

	dsi->engine_clk = devm_clk_get(dev, "engine");
	if (IS_ERR(dsi->engine_clk))
		return dev_err_probe(dev, PTR_ERR(dsi->engine_clk),
				     "Failed to get engine clock\n");


	dsi->digital_clk = devm_clk_get(dev, "digital");
	if (IS_ERR(dsi->digital_clk))
		return dev_err_probe(dev, PTR_ERR(dsi->digital_clk),
				     "Failed to get digital clock\n");

	dsi->hs_clk = devm_clk_get(dev, "hs");
	if (IS_ERR(dsi->hs_clk))
		return dev_err_probe(dev, PTR_ERR(dsi->hs_clk), "Failed to get hs clock\n");

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dsi->regs = devm_ioremap_resource(dev, regs);
	if (IS_ERR(dsi->regs))
		return dev_err_probe(dev, PTR_ERR(dsi->regs), "Failed to ioremap memory\n");

	dsi->phy = devm_phy_get(dev, "dphy");
	if (IS_ERR(dsi->phy))
		return dev_err_probe(dev, PTR_ERR(dsi->phy), "Failed to get MIPI-DPHY\n");

	dsi->is_cphy = of_property_read_bool(dev->of_node, "mediatek,is-cphy");
	irq_num = platform_get_irq(pdev, 0);
	if (irq_num < 0)
		return irq_num;

	dsi->host.ops = &mtk_dsi_ops;
	dsi->host.dev = dev;
	ret = mipi_dsi_host_register(&dsi->host);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to register DSI host\n");

	ret = devm_request_irq(&pdev->dev, irq_num, mtk_dsi_irq,
			       IRQF_TRIGGER_NONE, dev_name(&pdev->dev), dsi);
	if (ret) {
		mipi_dsi_host_unregister(&dsi->host);
		return dev_err_probe(&pdev->dev, ret, "Failed to request DSI irq\n");
	}

	init_waitqueue_head(&dsi->irq_wait_queue);

	platform_set_drvdata(pdev, dsi);

	dsi->bridge.funcs = &mtk_dsi_bridge_funcs;
	dsi->bridge.of_node = dev->of_node;
	dsi->bridge.type = DRM_MODE_CONNECTOR_DSI;
	pm_runtime_enable(dev);

	return 0;
}

static void mtk_dsi_remove(struct platform_device *pdev)
{
	struct mtk_dsi *dsi = platform_get_drvdata(pdev);

	mtk_output_dsi_disable(dsi);
	mipi_dsi_host_unregister(&dsi->host);
	pm_runtime_disable(&pdev->dev);
}

static const struct mtk_dsi_driver_data mt8173_dsi_driver_data = {
	.reg_cmdq_off = 0x200,
	.dsi_vm_cmd_con = 0x130,
	.max_linkrate_kbps = 1500000,
};

static const struct mtk_dsi_driver_data mt2701_dsi_driver_data = {
	.reg_cmdq_off = 0x180,
	.dsi_vm_cmd_con = 0x130,
	.max_linkrate_kbps = 1500000,
};

static const struct mtk_dsi_driver_data mt8183_dsi_driver_data = {
	.reg_cmdq_off = 0x200,
	.has_shadow_ctl = true,
	.has_size_ctl = true,
	.dsi_vm_cmd_con = 0x130,
	.max_linkrate_kbps = 1500000,
};

static const struct mtk_dsi_driver_data mt8186_dsi_driver_data = {
	.reg_cmdq_off = 0xd00,
	.has_shadow_ctl = true,
	.has_size_ctl = true,
	.dsi_vm_cmd_con = 0x130,
	.max_linkrate_kbps = 1500000,
};

static const struct mtk_dsi_driver_data mt8188_dsi_driver_data = {
	.reg_cmdq_off = 0xd00,
	.has_shadow_ctl = true,
	.has_size_ctl = true,
	.cmdq_long_packet_ctl = true,
	.support_per_frame_lp = true,
	.dsi_vm_cmd_con = 0x130,
	.max_linkrate_kbps = 1500000,
};

static const struct mtk_dsi_driver_data mt8189_dsi_driver_data = {
	.reg_cmdq_off = 0xd00,
	.has_shadow_ctl = true,
	.has_size_ctl = true,
	.cmdq_long_packet_ctl = true,
	.support_per_frame_lp = true,
	.dsi_vm_cmd_con = 0x200,
	.max_linkrate_kbps = 2500000,
};

static const struct mtk_dsi_driver_data mt8196_dsi_driver_data = {
	.reg_cmdq_off = 0x400,
	.has_shadow_ctl = true,
	.has_size_ctl = true,
	.cmdq_long_packet_ctl = true,
	.support_per_frame_lp = true,
	.reg_phy_base = 0x600,
	.reg_20_ofs = 0x020,
	.reg_30_ofs = 0x030,
	.reg_40_ofs = 0x040,
	.reg_100_ofs = 0x100,
	.dsi_size_con = 0x02c,
	.dsi_vfp_early_stop = 0x170,
	.dsi_lfr_con = 0x1a0,
	.dsi_cmdq_con = 0x44,
	.dsi_type1_hs = 0x50,
	.dsi_hstx_ckl_wc = 0x100,
	.dsi_mem_conti = 0x048,
	.dsi_time_con = 0x200,
	.dsi_reserved = 0x3f8,
	.dsi_state_dbg6 = 0x274,
	.dsi_dbg_sel = 0x274,
	.dsi_shadow_dbg = 0x0d0,
	.dsi_vm_cmd_con = 0x110,
	.max_linkrate_kbps = 2000000,
	.buffer_unit = 32,
	.sram_unit = 32,
	.urgent_lo_fifo_us = 11,
	.urgent_hi_fifo_us = 12,
	.output_valid_fifo_us = 35,
	.support_dsi_buffer = true,
};

static const struct of_device_id mtk_dsi_of_match[] = {
	{ .compatible = "mediatek,mt2701-dsi", .data = &mt2701_dsi_driver_data },
	{ .compatible = "mediatek,mt8173-dsi", .data = &mt8173_dsi_driver_data },
	{ .compatible = "mediatek,mt8183-dsi", .data = &mt8183_dsi_driver_data },
	{ .compatible = "mediatek,mt8186-dsi", .data = &mt8186_dsi_driver_data },
	{ .compatible = "mediatek,mt8188-dsi", .data = &mt8188_dsi_driver_data },
	{ .compatible = "mediatek,mt8189-dsi", .data = &mt8189_dsi_driver_data },
	{ .compatible = "mediatek,mt8196-dsi", .data = &mt8196_dsi_driver_data },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtk_dsi_of_match);

struct platform_driver mtk_dsi_driver = {
	.probe = mtk_dsi_probe,
	.remove_new = mtk_dsi_remove,
	.driver = {
		.name = "mtk-dsi",
		.of_match_table = mtk_dsi_of_match,
	},
};
