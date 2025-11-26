// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2014 MediaTek Inc.
 * Author: Jie Qiu <jie.qiu@mediatek.com>
 */

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_edid.h>
#include <drm/drm_of.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_print.h>

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/media-bus-format.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/soc/mediatek/mtk-mmsys.h>
#include <linux/types.h>

#include <video/videomode.h>

#include "mtk_ddp_comp.h"
#include "mtk_disp_drv.h"
#include "mtk_crtc.h"
#include "mtk_dpi_regs_v2.h"
#include "mtk_drm_drv.h"

enum mtk_dpi_out_bit_num {
	MTK_DPI_OUT_BIT_NUM_8BITS,
	MTK_DPI_OUT_BIT_NUM_10BITS,
	MTK_DPI_OUT_BIT_NUM_12BITS,
	MTK_DPI_OUT_BIT_NUM_16BITS
};

enum mtk_dpi_out_yc_map {
	MTK_DPI_OUT_YC_MAP_RGB,
	MTK_DPI_OUT_YC_MAP_CYCY,
	MTK_DPI_OUT_YC_MAP_YCYC,
	MTK_DPI_OUT_YC_MAP_CY,
	MTK_DPI_OUT_YC_MAP_YC
};

enum mtk_dpi_out_channel_swap {
	MTK_DPI_OUT_CHANNEL_SWAP_RGB,
	MTK_DPI_OUT_CHANNEL_SWAP_GBR,
	MTK_DPI_OUT_CHANNEL_SWAP_BRG,
	MTK_DPI_OUT_CHANNEL_SWAP_RBG,
	MTK_DPI_OUT_CHANNEL_SWAP_GRB,
	MTK_DPI_OUT_CHANNEL_SWAP_BGR
};

enum mtk_dpi_out_color_format {
	MTK_DPI_COLOR_FORMAT_RGB,
	MTK_DPI_COLOR_FORMAT_YCBCR_422
};

enum TVDPLL_CLK {
	TCK_26M,
	TVDPLL_D2,
	TVDPLL_D4,
	TVDPLL_D8,
	TVDPLL_D16,
	TVDPLL_PLL,
};

struct mtk_dpi {
	struct drm_encoder encoder;
	struct drm_bridge bridge;
	struct drm_bridge *next_bridge;
	void __iomem *regs;
	struct device *dev;
	struct device *mmsys_dev;
	struct clk *engine_clk;
	struct clk *pixel_clk;
	struct clk *tvd_clk;
	struct clk *hf_fmm_ck;
	struct clk *hf_fdp_ck;
	struct clk *pclk;
	struct clk *pclk_src[6];
	int irq;
	struct drm_display_mode mode;
	const struct mtk_dpi_conf *conf;
	enum mtk_dpi_out_color_format color_format;
	enum mtk_dpi_out_yc_map yc_map;
	enum mtk_dpi_out_bit_num bit_num;
	enum mtk_dpi_out_channel_swap channel_swap;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_gpio;
	struct pinctrl_state *pins_dpi;
	u32 output_fmt;
	int refcount;
	bool dsc_enable;
	struct drm_dsc_config dsc_config;
	struct drm_property *prop_dsc_enable;
	struct drm_property *prop_dsc_cfg;
};

static inline struct mtk_dpi *bridge_to_dpi_v2(struct drm_bridge *b)
{
	return container_of(b, struct mtk_dpi, bridge);
}

enum mtk_dpi_polarity {
	MTK_DPI_POLARITY_RISING,
	MTK_DPI_POLARITY_FALLING,
};

struct mtk_dpi_polarities {
	enum mtk_dpi_polarity de_pol;
	enum mtk_dpi_polarity ck_pol;
	enum mtk_dpi_polarity hsync_pol;
	enum mtk_dpi_polarity vsync_pol;
};

struct mtk_dpi_sync_param {
	u32 sync_width;
	u32 front_porch;
	u32 back_porch;
	bool shift_half_line;
};

struct mtk_dpi_yc_limit {
	u16 y_top;
	u16 y_bottom;
	u16 c_top;
	u16 c_bottom;
};

/**
 * struct mtk_dpi_conf - Configuration of mediatek dpi.
 * @cal_factor: Callback function to calculate factor value.
 * @reg_h_fre_con: Register address of frequency control.
 * @max_clock_khz: Max clock frequency supported for this SoCs in khz units.
 * @edge_sel_en: Enable of edge selection.
 * @output_fmts: Array of supported output formats.
 * @num_output_fmts: Quantity of supported output formats.
 * @is_ck_de_pol: Support CK/DE polarity.
 * @swap_input_support: Support input swap function.
 * @support_direct_pin: IP supports direct connection to dpi panels.
 * @input_2pixel: Input pixel of dp_intf is 2 pixel per round, so enable this
 *		  config to enable this feature.
 * @dimension_mask: Mask used for HWIDTH, HPORCH, VSYNC_WIDTH and VSYNC_PORCH
 *		    (no shift).
 * @hvsize_mask: Mask of HSIZE and VSIZE mask (no shift).
 * @channel_swap_shift: Shift value of channel swap.
 * @yuv422_en_bit: Enable bit of yuv422.
 * @csc_enable_bit: Enable bit of CSC.
 * @pixels_per_iter: Quantity of transferred pixels per iteration.
 * @edge_cfg_in_mmsys: If the edge configuration for DPI's output needs to be set in MMSYS.
 */
struct mtk_dpi_conf {
	unsigned int (*cal_factor)(int clock);
	u32 reg_h_fre_con;
	u32 max_clock_khz;
	bool edge_sel_en;
	const u32 *output_fmts;
	u32 num_output_fmts;
	bool is_ck_de_pol;
	bool swap_input_support;
	bool support_direct_pin;
	bool input_2pixel;
	u32 dimension_mask;
	u32 hvsize_mask;
	u32 channel_swap_shift;
	u32 yuv422_en_bit;
	u32 csc_enable_bit;
	u32 pixels_per_iter;
	bool edge_cfg_in_mmsys;
	bool dsc_support;
	bool use_version_2;
	bool pol_positive_default;
	bool hfp_adjustment_for_bs;
};

bool mtk_dpi_v2;

static void mtk_dpi_mask_v2(struct mtk_dpi *dpi, u32 offset, u32 val, u32 mask)
{
	u32 tmp = readl(dpi->regs + offset) & ~mask;

	tmp |= (val & mask);
	writel(tmp, dpi->regs + offset);
}

static void mtk_dpi_sw_reset_v2(struct mtk_dpi *dpi, bool reset)
{
	mtk_dpi_mask_v2(dpi, DPI_RET, reset ? RST : 0, RST);
}

static void mtk_dpi_enable_v2(struct mtk_dpi *dpi)
{
	drm_dbg_kms(dpi->bridge.dev, "[DPTX] enable dpi");
	mtk_dpi_mask_v2(dpi, DPI_EN, EN, EN);
}

static void mtk_dpi_disable_v2(struct mtk_dpi *dpi)
{
	mtk_dpi_mask_v2(dpi, DPI_EN, 0, EN);
}

static void mtk_dpi_config_hsync_v2(struct mtk_dpi *dpi,
				 struct mtk_dpi_sync_param *sync)
{
	mtk_dpi_mask_v2(dpi, DPI_TGEN_HWIDTH, sync->sync_width << HPW,
			dpi->conf->dimension_mask << HPW);
	mtk_dpi_mask_v2(dpi, DPI_TGEN_HPORCH, sync->back_porch << HBP,
			dpi->conf->dimension_mask << HBP);
	mtk_dpi_mask_v2(dpi, DPI_TGEN_HPORCH, sync->front_porch << HFP,
			dpi->conf->dimension_mask << HFP);
}

static void mtk_dpi_config_vsync_v2(struct mtk_dpi *dpi,
				 struct mtk_dpi_sync_param *sync,
				 u32 width_addr, u32 porch_addr)
{
	mtk_dpi_mask_v2(dpi, width_addr,
			sync->shift_half_line << VSYNC_HALF_LINE_SHIFT,
			VSYNC_HALF_LINE_MASK);
	mtk_dpi_mask_v2(dpi, width_addr,
			sync->sync_width << VSYNC_WIDTH_SHIFT,
			dpi->conf->dimension_mask << VSYNC_WIDTH_SHIFT);
	mtk_dpi_mask_v2(dpi, porch_addr,
			sync->back_porch << VSYNC_BACK_PORCH_SHIFT,
			dpi->conf->dimension_mask << VSYNC_BACK_PORCH_SHIFT);
	mtk_dpi_mask_v2(dpi, porch_addr,
			sync->front_porch << VSYNC_FRONT_PORCH_SHIFT,
			dpi->conf->dimension_mask << VSYNC_FRONT_PORCH_SHIFT);
}

static void mtk_dpi_config_vsync_lodd_v2(struct mtk_dpi *dpi,
				      struct mtk_dpi_sync_param *sync)
{
	mtk_dpi_config_vsync_v2(dpi, sync, DPI_TGEN_VWIDTH, DPI_TGEN_VPORCH);
}

static void mtk_dpi_config_vsync_leven_v2(struct mtk_dpi *dpi,
				       struct mtk_dpi_sync_param *sync)
{
	mtk_dpi_config_vsync_v2(dpi, sync, DPI_TGEN_VWIDTH_LEVEN,
			     DPI_TGEN_VPORCH_LEVEN);
}

static void mtk_dpi_config_vsync_rodd_v2(struct mtk_dpi *dpi,
				      struct mtk_dpi_sync_param *sync)
{
	mtk_dpi_config_vsync_v2(dpi, sync, DPI_TGEN_VWIDTH_RODD,
			     DPI_TGEN_VPORCH_RODD);
}

static void mtk_dpi_config_vsync_reven_v2(struct mtk_dpi *dpi,
				       struct mtk_dpi_sync_param *sync)
{
	mtk_dpi_config_vsync_v2(dpi, sync, DPI_TGEN_VWIDTH_REVEN,
			     DPI_TGEN_VPORCH_REVEN);
}

static void mtk_dpi_config_pol_v2(struct mtk_dpi *dpi,
			       struct mtk_dpi_polarities *dpi_pol)
{
	unsigned int pol = 0;
	unsigned int mask;

	mask = HSYNC_POL | VSYNC_POL;

	pol = (dpi_pol->hsync_pol == MTK_DPI_POLARITY_RISING ? 0 : HSYNC_POL) |
	      (dpi_pol->vsync_pol == MTK_DPI_POLARITY_RISING ? 0 : VSYNC_POL);
	if (dpi->conf->is_ck_de_pol) {
		mask |= CK_POL | DE_POL;
		pol |= (dpi_pol->ck_pol == MTK_DPI_POLARITY_RISING ?
			0 : CK_POL) |
		       (dpi_pol->de_pol == MTK_DPI_POLARITY_RISING ?
			0 : DE_POL);
	}

	mtk_dpi_mask_v2(dpi, DPI_OUTPUT_SETTING, pol, mask);
}

static void mtk_dpi_config_3d_v2(struct mtk_dpi *dpi, bool en_3d)
{
	mtk_dpi_mask_v2(dpi, DPI_CON, en_3d ? TDFP_EN : 0, TDFP_EN);
}

static void mtk_dpi_config_interface_v2(struct mtk_dpi *dpi, bool inter)
{
	mtk_dpi_mask_v2(dpi, DPI_CON, inter ? INTL_EN : 0, INTL_EN);
}

static void mtk_dpi_config_fb_size_v2(struct mtk_dpi *dpi, u32 width, u32 height)
{
	mtk_dpi_mask_v2(dpi, DPI_SIZE, width << HSIZE,
			dpi->conf->hvsize_mask << HSIZE);
	mtk_dpi_mask_v2(dpi, DPI_SIZE, height << VSIZE,
			dpi->conf->hvsize_mask << VSIZE);
}

static void mtk_dpi_config_channel_limit_v2(struct mtk_dpi *dpi)
{
	struct mtk_dpi_yc_limit limit;

	if (drm_default_rgb_quant_range(&dpi->mode) ==
	    HDMI_QUANTIZATION_RANGE_LIMITED) {
		limit.y_bottom = 0x10;
		limit.y_top = 0xfe0;
		limit.c_bottom = 0x10;
		limit.c_top = 0xfe0;
	} else {
		limit.y_bottom = 0;
		limit.y_top = 0xfff;
		limit.c_bottom = 0;
		limit.c_top = 0xfff;
	}

	mtk_dpi_mask_v2(dpi, DPI_Y_LIMIT, limit.y_bottom << Y_LIMINT_BOT,
		     Y_LIMINT_BOT_MASK);
	mtk_dpi_mask_v2(dpi, DPI_Y_LIMIT, limit.y_top << Y_LIMINT_TOP,
		     Y_LIMINT_TOP_MASK);
	mtk_dpi_mask_v2(dpi, DPI_C_LIMIT, limit.c_bottom << C_LIMIT_BOT,
		     C_LIMIT_BOT_MASK);
	mtk_dpi_mask_v2(dpi, DPI_C_LIMIT, limit.c_top << C_LIMIT_TOP,
		     C_LIMIT_TOP_MASK);
}

static void mtk_dpi_config_bit_num_v2(struct mtk_dpi *dpi,
				   enum mtk_dpi_out_bit_num num)
{
	u32 val;

	switch (num) {
	case MTK_DPI_OUT_BIT_NUM_8BITS:
		val = OUT_BIT_8;
		break;
	case MTK_DPI_OUT_BIT_NUM_10BITS:
		val = OUT_BIT_10;
		break;
	case MTK_DPI_OUT_BIT_NUM_12BITS:
		val = OUT_BIT_12;
		break;
	case MTK_DPI_OUT_BIT_NUM_16BITS:
		val = OUT_BIT_16;
		break;
	default:
		val = OUT_BIT_8;
		break;
	}
	mtk_dpi_mask_v2(dpi, DPI_OUTPUT_SETTING, val << OUT_BIT,
			OUT_BIT_MASK);
}

static void mtk_dpi_config_yc_map_v2(struct mtk_dpi *dpi,
				  enum mtk_dpi_out_yc_map map)
{
	u32 val;

	switch (map) {
	case MTK_DPI_OUT_YC_MAP_RGB:
		val = YC_MAP_RGB;
		break;
	case MTK_DPI_OUT_YC_MAP_CYCY:
		val = YC_MAP_CYCY;
		break;
	case MTK_DPI_OUT_YC_MAP_YCYC:
		val = YC_MAP_YCYC;
		break;
	case MTK_DPI_OUT_YC_MAP_CY:
		val = YC_MAP_CY;
		break;
	case MTK_DPI_OUT_YC_MAP_YC:
		val = YC_MAP_YC;
		break;
	default:
		val = YC_MAP_RGB;
		break;
	}

	mtk_dpi_mask_v2(dpi, DPI_OUTPUT_SETTING, val << YC_MAP, YC_MAP_MASK);
}

static void mtk_dpi_config_channel_swap_v2(struct mtk_dpi *dpi,
					enum mtk_dpi_out_channel_swap swap)
{
	u32 val;

	switch (swap) {
	case MTK_DPI_OUT_CHANNEL_SWAP_RGB:
		val = SWAP_RGB;
		break;
	case MTK_DPI_OUT_CHANNEL_SWAP_GBR:
		val = SWAP_GBR;
		break;
	case MTK_DPI_OUT_CHANNEL_SWAP_BRG:
		val = SWAP_BRG;
		break;
	case MTK_DPI_OUT_CHANNEL_SWAP_RBG:
		val = SWAP_RBG;
		break;
	case MTK_DPI_OUT_CHANNEL_SWAP_GRB:
		val = SWAP_GRB;
		break;
	case MTK_DPI_OUT_CHANNEL_SWAP_BGR:
		val = SWAP_BGR;
		break;
	default:
		val = SWAP_RGB;
		break;
	}

	mtk_dpi_mask_v2(dpi, DPI_OUTPUT_SETTING,
			val << dpi->conf->channel_swap_shift,
			CH_SWAP_MASK << dpi->conf->channel_swap_shift);
}

static void mtk_dpi_config_yuv422_enable_v2(struct mtk_dpi *dpi)
{
	mtk_dpi_mask_v2(dpi, DPI_YUV, DPI_YUV_EN, DPI_YUV_EN);
	mtk_dpi_mask_v2(dpi, DPI_YUV, DPI_YUV422_EN, DPI_YUV422_EN);
}

static void mtk_dpi_config_yuv422_disable_v2(struct mtk_dpi *dpi)
{
	mtk_dpi_mask_v2(dpi, DPI_YUV, 0, DPI_YUV_EN);
	mtk_dpi_mask_v2(dpi, DPI_YUV, 0, DPI_YUV422_EN);
}

static void mtk_dpi_config_swap_input_v2(struct mtk_dpi *dpi, bool enable)
{
	mtk_dpi_mask_v2(dpi, DPI_CON, enable ? IN_RB_SWAP : 0, IN_RB_SWAP);
}

static void mtk_dpi_config_2n_h_fre_v2(struct mtk_dpi *dpi)
{
	mtk_dpi_mask_v2(dpi, dpi->conf->reg_h_fre_con, H_FRE_2N, H_FRE_2N);
}

static void mtk_dpi_config_disable_edge_v2(struct mtk_dpi *dpi)
{
	if (dpi->conf->edge_sel_en)
		mtk_dpi_mask_v2(dpi, dpi->conf->reg_h_fre_con, 0, EDGE_SEL_EN);
}

static void mtk_dpi_config_color_format_v2(struct mtk_dpi *dpi,
					enum mtk_dpi_out_color_format format)
{
	mtk_dpi_config_channel_swap_v2(dpi, MTK_DPI_OUT_CHANNEL_SWAP_RGB);

	drm_dbg_kms(dpi->bridge.dev, "[DPTX] format:%d", format);

	if (format == MTK_DPI_COLOR_FORMAT_YCBCR_422) {
		mtk_dpi_config_yuv422_enable_v2(dpi);

		/*
		 * If height is smaller than 720, we need to use RGB_TO_BT601
		 * to transfer to yuv422. Otherwise, we use RGB_TO_JPEG.
		 */
		mtk_dpi_mask_v2(dpi, DPI_MATRIX_SET, dpi->mode.hdisplay <= 720 ?
			     MATRIX_SEL_RGB_TO_BT601 : MATRIX_SEL_RGB_TO_JPEG,
			     INT_MATRIX_SEL_MASK);
	} else {
		mtk_dpi_config_yuv422_disable_v2(dpi);
		if (dpi->conf->swap_input_support)
			mtk_dpi_config_swap_input_v2(dpi, false);
	}
}

static void mtk_dpi_dual_edge_v2(struct mtk_dpi *dpi)
{
	if (dpi->output_fmt == MEDIA_BUS_FMT_RGB888_2X12_LE ||
	    dpi->output_fmt == MEDIA_BUS_FMT_RGB888_2X12_BE) {
		mtk_dpi_mask_v2(dpi, DPI_DDR_SETTING, DDR_EN | DDR_4PHASE,
			     DDR_EN | DDR_4PHASE);
		mtk_dpi_mask_v2(dpi, DPI_OUTPUT_SETTING,
			     dpi->output_fmt == MEDIA_BUS_FMT_RGB888_2X12_LE ?
			     EDGE_SEL : 0, EDGE_SEL);
		if (dpi->conf->edge_cfg_in_mmsys)
			mtk_mmsys_ddp_dpi_fmt_config(dpi->mmsys_dev, MTK_DPI_RGB888_DDR_CON);
	} else {
		mtk_dpi_mask_v2(dpi, DPI_DDR_SETTING, DDR_EN | DDR_4PHASE, 0);
		if (dpi->conf->edge_cfg_in_mmsys)
			mtk_mmsys_ddp_dpi_fmt_config(dpi->mmsys_dev, MTK_DPI_RGB888_SDR_CON);
	}
}

static void mtk_dpi_power_off_v2(struct mtk_dpi *dpi)
{
	if (WARN_ON(dpi->refcount == 0))
		return;

	if (--dpi->refcount != 0)
		return;

	drm_dbg_kms(dpi->bridge.dev, "[DPTX] power off\n");

	mtk_dpi_disable_v2(dpi);

	clk_disable_unprepare(dpi->pclk);
	clk_disable_unprepare(dpi->pclk_src[TVDPLL_PLL]);
	clk_disable_unprepare(dpi->hf_fdp_ck);
	clk_disable_unprepare(dpi->hf_fmm_ck);
}

static int mtk_dpi_power_on_v2(struct mtk_dpi *dpi)
{
	int ret;

	if (++dpi->refcount != 1)
		return 0;

	drm_dbg_kms(dpi->bridge.dev, "[DPTX] power on\n");

	ret = clk_prepare_enable(dpi->hf_fmm_ck);
	if (ret) {
		dev_err(dpi->dev, "[DPTX] Failed to enable hf_fmm_ck clock: %d\n", ret);
		goto err_refcount;
	}

	ret = clk_prepare_enable(dpi->hf_fdp_ck);
	if (ret) {
		dev_err(dpi->dev, "[DPTX] Failed to enable hf_fdp_ck clock: %d\n", ret);
		clk_disable_unprepare(dpi->hf_fmm_ck);
		goto err_refcount;
	}

	ret = clk_prepare_enable(dpi->pclk_src[TVDPLL_PLL]);
	if (ret) {
		dev_err(dpi->dev, "[DPTX] Failed to enable pclk_src: %d\n", ret);
		clk_disable_unprepare(dpi->hf_fmm_ck);
		clk_disable_unprepare(dpi->hf_fdp_ck);
		goto err_refcount;
	}

	ret = clk_prepare_enable(dpi->pclk);
	if (ret) {
		dev_err(dpi->dev, "[DPTX] Failed to enable pclk:%d\n", ret);
		clk_disable_unprepare(dpi->hf_fmm_ck);
		clk_disable_unprepare(dpi->hf_fdp_ck);
		clk_disable_unprepare(dpi->pclk_src[TVDPLL_PLL]);
		goto err_refcount;
	}

	return 0;

err_refcount:
	dpi->refcount--;
	return ret;
}

static void mtk_dpi_set_golden_setting_v2(struct mtk_dpi *dpi, u32 hsize, u32 vsize)
{
	u32 dp_buf_sodi_high = 5255;
	u32 dp_buf_sodi_low = 3899;

	unsigned int rw_times = 0;

	drm_dbg_kms(dpi->bridge.dev, "[DPTX] HV:%d x %d, sodi_high:%d, sodi_low:%d\n",
		    hsize, vsize,
		    dp_buf_sodi_high,
		    dp_buf_sodi_low);

	mtk_dpi_mask_v2(dpi, DPI_BUF_SODI_HIGH, dp_buf_sodi_high, GENMASK(31,0));

	mtk_dpi_mask_v2(dpi, DPI_BUF_SODI_LOW, dp_buf_sodi_low, GENMASK(31,0));

	if (hsize & 0x3)
		rw_times = ((hsize >> 2) + 1) * vsize;
	else
		rw_times = (hsize >> 2) * vsize;

	mtk_dpi_mask_v2(dpi, DPI_BUF_RW_TIMES, rw_times, GENMASK(31,0));
	mtk_dpi_mask_v2(dpi, DPI_BUF_CON0, BUF_BUF_EN, BUF_BUF_EN);
	mtk_dpi_mask_v2(dpi, DPI_BUF_CON0, BUF_BUF_FIFO_UNDERFLOW_DONT_BLOCK,
		     BUF_BUF_FIFO_UNDERFLOW_DONT_BLOCK);
}

static int mtk_dpi_set_display_mode_v2(struct mtk_dpi *dpi,
				    struct drm_display_mode *mode)
{
	struct mtk_dpi_polarities dpi_pol;
	struct mtk_dpi_sync_param hsync;
	struct mtk_dpi_sync_param vsync_lodd = { 0 };
	struct mtk_dpi_sync_param vsync_leven = { 0 };
	struct mtk_dpi_sync_param vsync_rodd = { 0 };
	struct mtk_dpi_sync_param vsync_reven = { 0 };
	struct videomode vm = { 0 };
	unsigned long pll_rate;
	unsigned int clksrc = TCK_26M;
	unsigned int hsize;
	u16 hblank;
	u64 htotal;
	u64 mode_htotal;
	int ret = 0;

	drm_display_mode_to_videomode(mode, &vm);

	if (dpi->dsc_enable) {
		hblank = mode->htotal - mode->hdisplay;
		hsize = DIV_ROUND_UP(vm.hactive, 3);
		vm.hactive = ((vm.hactive * 8 + (12 * 8 - 1)) / (12 * 8)) * 4;
		htotal = hblank + vm.hactive;
		mode_htotal =  mode->htotal;
		vm.pixelclock = div_u64(mode->clock * 1000 * htotal, mode_htotal);

		drm_dbg_kms(dpi->bridge.dev,
			    "[DPTX] DSC compress mode, hactive:%d(%d), pixelclock:%lu\n",
			    vm.hactive, hsize, vm.pixelclock);
		dpi->color_format = MTK_DPI_COLOR_FORMAT_RGB;
	} else {
		vm.pixelclock = mode->clock * 1000;
		hsize = vm.hactive;
	}

	if (vm.pixelclock < 70000000)
		clksrc = TVDPLL_D16;
	else if (vm.pixelclock < 200000000)
		clksrc = TVDPLL_D8;
	else
		clksrc = TVDPLL_D4;

	pll_rate = vm.pixelclock * (1 << clksrc);

	drm_dbg_kms(dpi->bridge.dev, "[DPTX] pixel:%lu, clksrc:%d, pll_rate/4:%lu\n",
		     vm.pixelclock, clksrc, pll_rate / 4);

	ret = clk_set_rate(dpi->pclk_src[TVDPLL_PLL], pll_rate / 4);
	if (ret) {
		dev_err(dpi->dev, "[DPTX] cannot set pclk_src[TVDPLL_PLL]: err=%d\n",
			 ret);
	}

	ret = clk_set_parent(dpi->pclk, dpi->pclk_src[clksrc]);
	if (ret) {
		dev_err(dpi->dev, "[DPTX] clk_set_parent dp_intf->pclk: err=%d\n",
			 ret);
	}

	drm_dbg_kms(dpi->bridge.dev, "[DPTX] pclk_src[TVDPLL_PLL]:%ld\n",
		    clk_get_rate(dpi->pclk_src[TVDPLL_PLL]));
	drm_dbg_kms(dpi->bridge.dev, "[DPTX] pclk_src[clksrc]:%ld\n",
		    clk_get_rate(dpi->pclk_src[clksrc]));
	drm_dbg_kms(dpi->bridge.dev, "[DPTX] pclk:%ld\n",
		    clk_get_rate(dpi->pclk));
	drm_dbg_kms(dpi->bridge.dev, "[DPTX] hf_fmm_ck:%ld\n",
		    clk_get_rate(dpi->hf_fmm_ck));
	drm_dbg_kms(dpi->bridge.dev, "[DPTX] hf_fdp_ck:%ld\n",
		    clk_get_rate(dpi->hf_fdp_ck));

	dpi_pol.ck_pol = MTK_DPI_POLARITY_FALLING;
	dpi_pol.de_pol = MTK_DPI_POLARITY_RISING;
	dpi_pol.hsync_pol = vm.flags & DISPLAY_FLAGS_HSYNC_HIGH ?
			    MTK_DPI_POLARITY_FALLING : MTK_DPI_POLARITY_RISING;
	dpi_pol.vsync_pol = vm.flags & DISPLAY_FLAGS_VSYNC_HIGH ?
			    MTK_DPI_POLARITY_FALLING : MTK_DPI_POLARITY_RISING;

	if (dpi->conf->pol_positive_default) {
		/* Due to the HW design, force DPI polarity positive */
		dpi_pol.hsync_pol = MTK_DPI_POLARITY_RISING;
		dpi_pol.vsync_pol = MTK_DPI_POLARITY_RISING;
	}

	/*
	 * Depending on the IP version, we may output a different amount of
	 * pixels for each iteration: divide the clock by this number and
	 * adjust the display porches accordingly.
	 */
	hsync.sync_width = ((vm.hsync_len / dpi->conf->pixels_per_iter) == 0) ?
		1 : (vm.hsync_len / dpi->conf->pixels_per_iter);
	hsync.back_porch = ((vm.hback_porch / dpi->conf->pixels_per_iter) == 0) ?
		1 : (vm.hback_porch / dpi->conf->pixels_per_iter);
	hsync.front_porch = ((vm.hfront_porch / dpi->conf->pixels_per_iter) == 0) ?
		1 : (vm.hfront_porch / dpi->conf->pixels_per_iter);

	if (dpi->conf->hfp_adjustment_for_bs) {
		/* Tuning the DPI H front porch to generate BS normally */
		u16 hfp_adjustment = dpi->dsc_enable ? 5 : 1;
		hsync.back_porch += (hsync.front_porch - hfp_adjustment);
		hsync.front_porch = hfp_adjustment;
	}

	hsync.shift_half_line = false;
	vsync_lodd.sync_width = vm.vsync_len;
	vsync_lodd.back_porch = vm.vback_porch;
	vsync_lodd.front_porch = vm.vfront_porch;
	vsync_lodd.shift_half_line = false;

	if (vm.flags & DISPLAY_FLAGS_INTERLACED &&
	    mode->flags & DRM_MODE_FLAG_3D_MASK) {
		vsync_leven = vsync_lodd;
		vsync_rodd = vsync_lodd;
		vsync_reven = vsync_lodd;
		vsync_leven.shift_half_line = true;
		vsync_reven.shift_half_line = true;
	} else if (vm.flags & DISPLAY_FLAGS_INTERLACED &&
		   !(mode->flags & DRM_MODE_FLAG_3D_MASK)) {
		vsync_leven = vsync_lodd;
		vsync_leven.shift_half_line = true;
	} else if (!(vm.flags & DISPLAY_FLAGS_INTERLACED) &&
		   mode->flags & DRM_MODE_FLAG_3D_MASK) {
		vsync_rodd = vsync_lodd;
	}
	mtk_dpi_sw_reset_v2(dpi, true);
	mtk_dpi_config_pol_v2(dpi, &dpi_pol);

	mtk_dpi_set_golden_setting_v2(dpi, vm.hactive, vm.vactive);

	mtk_dpi_config_hsync_v2(dpi, &hsync);
	mtk_dpi_config_vsync_lodd_v2(dpi, &vsync_lodd);
	mtk_dpi_config_vsync_rodd_v2(dpi, &vsync_rodd);
	mtk_dpi_config_vsync_leven_v2(dpi, &vsync_leven);
	mtk_dpi_config_vsync_reven_v2(dpi, &vsync_reven);

	mtk_dpi_config_3d_v2(dpi, !!(mode->flags & DRM_MODE_FLAG_3D_MASK));
	mtk_dpi_config_interface_v2(dpi, !!(vm.flags &
					 DISPLAY_FLAGS_INTERLACED));
	if (vm.flags & DISPLAY_FLAGS_INTERLACED)
		mtk_dpi_config_fb_size_v2(dpi, hsize, vm.vactive >> 1);
	else
		mtk_dpi_config_fb_size_v2(dpi, hsize, vm.vactive);

	mtk_dpi_config_channel_limit_v2(dpi);
	mtk_dpi_config_bit_num_v2(dpi, dpi->bit_num);
	mtk_dpi_config_channel_swap_v2(dpi, dpi->channel_swap);
	mtk_dpi_config_color_format_v2(dpi, dpi->color_format);
	if (dpi->conf->support_direct_pin) {
		mtk_dpi_config_yc_map_v2(dpi, dpi->yc_map);
		mtk_dpi_config_2n_h_fre_v2(dpi);
		mtk_dpi_dual_edge_v2(dpi);
		mtk_dpi_config_disable_edge_v2(dpi);
	}
	if (dpi->conf->input_2pixel) {
		mtk_dpi_mask_v2(dpi, DPI_CONFIG_1TNP, DPI_CONFIG_1T2P,
			     DPI_CONFIG_1TNP_MASK);
	}
	mtk_dpi_sw_reset_v2(dpi, false);

	return 0;
}

static int mtk_dpi_bridge_atomic_check_v2(struct drm_bridge *bridge,
					  struct drm_bridge_state *bridge_state,
					  struct drm_crtc_state *crtc_state,
					  struct drm_connector_state *conn_state)
{
	struct mtk_dpi *dpi = bridge_to_dpi_v2(bridge);
	unsigned int out_bus_format;

	out_bus_format = bridge_state->output_bus_cfg.format;

	if (out_bus_format == MEDIA_BUS_FMT_FIXED)
		if (dpi->conf->num_output_fmts)
			out_bus_format = dpi->conf->output_fmts[0];

	drm_dbg_kms(dpi->bridge.dev, "[DPTX] input format 0x%04x, output format 0x%04x\n",
		    bridge_state->input_bus_cfg.format,
		    bridge_state->output_bus_cfg.format);

	dpi->output_fmt = out_bus_format;
	dpi->bit_num = MTK_DPI_OUT_BIT_NUM_8BITS;
	dpi->channel_swap = MTK_DPI_OUT_CHANNEL_SWAP_RGB;
	dpi->yc_map = MTK_DPI_OUT_YC_MAP_RGB;
	if (out_bus_format == MEDIA_BUS_FMT_YUYV8_1X16)
		dpi->color_format = MTK_DPI_COLOR_FORMAT_YCBCR_422;
	else
		dpi->color_format = MTK_DPI_COLOR_FORMAT_RGB;

	dpi->dsc_enable = dpi->prop_dsc_enable->values[0];
	memcpy(&dpi->dsc_config, dpi->prop_dsc_cfg->values, sizeof(struct drm_dsc_config));
	drm_dbg_kms(dpi->bridge.dev, "[DPTX] dsc enable:%d, dsc version:%d\n", dpi->dsc_enable, dpi->dsc_config.dsc_version_minor);

	return 0;
}

static u32 *mtk_dpi_bridge_atomic_get_output_bus_fmts_v2(struct drm_bridge *bridge,
						      struct drm_bridge_state *bridge_state,
						      struct drm_crtc_state *crtc_state,
						      struct drm_connector_state *conn_state,
						      unsigned int *num_output_fmts)
{
	struct mtk_dpi *dpi = bridge_to_dpi_v2(bridge);
	u32 *output_fmts;

	*num_output_fmts = 0;

	if (!dpi->conf->output_fmts) {
		dev_err(dpi->dev, "[DPTX] output_fmts should not be null\n");
		return NULL;
	}

	output_fmts = kcalloc(dpi->conf->num_output_fmts, sizeof(*output_fmts),
			      GFP_KERNEL);
	if (!output_fmts)
		return NULL;

	*num_output_fmts = dpi->conf->num_output_fmts;

	memcpy(output_fmts, dpi->conf->output_fmts,
	       sizeof(*output_fmts) * dpi->conf->num_output_fmts);

	return output_fmts;
}

static u32 *mtk_dpi_bridge_atomic_get_input_bus_fmts_v2(struct drm_bridge *bridge,
						     struct drm_bridge_state *bridge_state,
						     struct drm_crtc_state *crtc_state,
						     struct drm_connector_state *conn_state,
						     u32 output_fmt,
						     unsigned int *num_input_fmts)
{
	u32 *input_fmts;

	*num_input_fmts = 0;

	input_fmts = kcalloc(1, sizeof(*input_fmts),
			     GFP_KERNEL);
	if (!input_fmts)
		return NULL;

	*num_input_fmts = 1;
	input_fmts[0] = MEDIA_BUS_FMT_RGB888_1X24;

	return input_fmts;
}

static int mtk_dpi_bridge_attach_v2(struct drm_bridge *bridge,
				    enum drm_bridge_attach_flags flags)
{
	struct mtk_dpi *dpi = bridge_to_dpi_v2(bridge);

	return drm_bridge_attach(bridge->encoder, dpi->next_bridge,
				 &dpi->bridge, flags);
}

static void mtk_dpi_bridge_mode_set_v2(struct drm_bridge *bridge,
				       const struct drm_display_mode *mode,
				       const struct drm_display_mode *adjusted_mode)
{
	struct mtk_dpi *dpi = bridge_to_dpi_v2(bridge);

	drm_mode_copy(&dpi->mode, adjusted_mode);

	drm_dbg_kms(dpi->bridge.dev, "[DPTX] mode set, Htt:%d, Vtt:%d, Hact:%d, Vact:%d, fps:%d\n",
		    dpi->mode.htotal, dpi->mode.vtotal,
		    dpi->mode.hdisplay, dpi->mode.vdisplay, drm_mode_vrefresh(mode));
}

static void mtk_dpi_bridge_disable_v2(struct drm_bridge *bridge)
{
	struct mtk_dpi *dpi = bridge_to_dpi_v2(bridge);

	mtk_dpi_power_off_v2(dpi);

	if (dpi->pinctrl && dpi->pins_gpio)
		pinctrl_select_state(dpi->pinctrl, dpi->pins_gpio);
}

static void mtk_dpi_bridge_enable_v2(struct drm_bridge *bridge)
{
	struct mtk_dpi *dpi = bridge_to_dpi_v2(bridge);

	drm_dbg_kms(dpi->bridge.dev, "[DPTX] dpi enabled\n");

	if (dpi->pinctrl && dpi->pins_dpi)
		pinctrl_select_state(dpi->pinctrl, dpi->pins_dpi);

	if (dpi->dsc_enable) {
		dpi->color_format = MTK_DPI_COLOR_FORMAT_RGB;
		drm_dbg_kms(dpi->bridge.dev, "[DPTX] dsc enable with the output format as RGB\n");
	}

	mtk_dpi_power_on_v2(dpi);
	mtk_dpi_set_display_mode_v2(dpi, &dpi->mode);

	/* TODO: Remove temporary solutions and follow API specifications */
	mtk_dpi_enable_v2(dpi);
	/* TODO: Use DPI IRQ to implement 3 vsync delays */
	/* HW implementation requires 3 vsync delays to wait for dpi to stabilize */
	mdelay(1000 / drm_mode_vrefresh(&dpi->mode) * 3);
}

static enum drm_mode_status
mtk_dpi_bridge_mode_valid_v2(struct drm_bridge *bridge,
			  const struct drm_display_info *info,
			  const struct drm_display_mode *mode)
{
	struct mtk_dpi *dpi = bridge_to_dpi_v2(bridge);

	if (mode->clock > dpi->conf->max_clock_khz)
		return MODE_CLOCK_HIGH;

	return MODE_OK;
}

static const struct drm_bridge_funcs mtk_dpi_bridge_funcs = {
	.attach = mtk_dpi_bridge_attach_v2,
	.mode_set = mtk_dpi_bridge_mode_set_v2,
	.mode_valid = mtk_dpi_bridge_mode_valid_v2,
	.disable = mtk_dpi_bridge_disable_v2,
	.enable = mtk_dpi_bridge_enable_v2,
	.atomic_check = mtk_dpi_bridge_atomic_check_v2,
	.atomic_get_output_bus_fmts = mtk_dpi_bridge_atomic_get_output_bus_fmts_v2,
	.atomic_get_input_bus_fmts = mtk_dpi_bridge_atomic_get_input_bus_fmts_v2,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_reset = drm_atomic_helper_bridge_reset,
};

void mtk_dpi_start_v2(struct device *dev)
{
	struct mtk_dpi *dpi = dev_get_drvdata(dev);

	drm_dbg_kms(dpi->bridge.dev, "[DPTX] start\n");

	mtk_dpi_power_on_v2(dpi);
}

void mtk_dpi_stop_v2(struct device *dev)
{
	struct mtk_dpi *dpi = dev_get_drvdata(dev);

	drm_dbg_kms(dpi->bridge.dev, "[DPTX] stop\n");

	mtk_dpi_power_off_v2(dpi);
}

unsigned int mtk_dpi_encoder_index_v2(struct device *dev)
{
	struct mtk_dpi *dpi = dev_get_drvdata(dev);
	unsigned int encoder_index = drm_encoder_index(&dpi->encoder);

	drm_dbg_kms(dpi->bridge.dev, "[DPTX] encoder index:%d\n", encoder_index);
	return encoder_index;
}

void mtk_dpi_get_hrt_bw_by_datarate(struct device *dev, unsigned int *bw_base)
{
	struct mtk_dpi *dpi = dev_get_drvdata(dev);
	int htotal, vtotal, vrefresh;

	htotal = dpi->mode.htotal;
	vtotal = dpi->mode.vtotal;
	vrefresh = drm_mode_vrefresh(&dpi->mode);

	*bw_base = (unsigned int)div_u64((unsigned long long)vtotal * htotal * vrefresh * 4,
					 1000000);
}

void mtk_dpi_get_dsc_info_v2(struct device *dev, struct dsc_info *dsc_info)
{
	struct mtk_dpi *dpi = dev_get_drvdata(dev);

	if (dsc_info) {
		dsc_info->compression_enable = dpi->dsc_enable;
		memcpy(&dsc_info->dsc_config, &dpi->dsc_config, sizeof(struct drm_dsc_config));
	}

	drm_dbg_kms(dpi->bridge.dev,
		    "[DPTX] get dsc info, compression_enable:%d\n", dsc_info->compression_enable);
}

static int mtk_dpi_bind_v2(struct device *dev, struct device *master, void *data)
{
	struct mtk_dpi *dpi = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	struct mtk_drm_private *priv = drm_dev->dev_private;
	struct drm_property *prop;
	char dsc_enable[] = "dp_dsc_enable";
	char dsc_cfg[] = "dp_dsc_cfg";
	char result[20];
	int id, ret;

	drm_dbg_kms(dpi->bridge.dev, "[DPTX] encoder init\n");
	dpi->mmsys_dev = priv->mmsys_dev;
	ret = drm_simple_encoder_init(drm_dev, &dpi->encoder,
				      DRM_MODE_ENCODER_TMDS);
	if (ret) {
		dev_err(dev, "[DPTX] Failed to initialize decoder:%d\n", ret);
		return ret;
	}

	id = of_alias_get_id(dev->of_node, "dp-intf");
	if (id < 0) {
		dev_err(dev, "[DPTX] Failed to get id: %d\n", ret);
		return ret;
	}

	snprintf(result, sizeof(result), "%s%d", dsc_enable, id);
	prop = drm_property_create_bool(dpi->encoder.dev,
					DRM_MODE_PROP_ATOMIC, result);
	if (!prop) {
		dev_err(dev, "[DPTX] failed to create property dp_dsc_enable\n");
		return ret;
	}
	dpi->prop_dsc_enable = prop;

	snprintf(result, sizeof(result), "%s%d", dsc_cfg, id);
	prop = drm_property_create(dpi->encoder.dev,
				   DRM_MODE_PROP_BLOB | DRM_MODE_PROP_IMMUTABLE,
				   result, sizeof(struct drm_dsc_config));
	if (!prop) {
		dev_err(dev, "[DPTX] failed to create property dp_dsc_cfg\n");
		return ret;
	}
	dpi->prop_dsc_cfg = prop;

	ret = mtk_find_possible_crtcs(drm_dev, dpi->dev);
	if (ret < 0)
		goto err_cleanup;
	dpi->encoder.possible_crtcs = ret;

	ret = drm_bridge_attach(&dpi->encoder, &dpi->bridge, NULL, 0);
	if (ret)
		goto err_cleanup;

	return 0;

err_cleanup:
	drm_encoder_cleanup(&dpi->encoder);
	return ret;
}

static void mtk_dpi_unbind_v2(struct device *dev, struct device *master,
			   void *data)
{
	struct mtk_dpi *dpi = dev_get_drvdata(dev);

	drm_encoder_cleanup(&dpi->encoder);
}

static const struct component_ops mtk_dpi_component_ops = {
	.bind = mtk_dpi_bind_v2,
	.unbind = mtk_dpi_unbind_v2,
};

static unsigned int mt8196_dpintf_calculate_factor_v2(int clock)
{
	if (clock < 70000)
		return 4;
	else if (clock < 200000)
		return 2;
	else
		return 1;
}

static const u32 mt8196_output_fmts[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_YUYV8_1X16,
};

static const struct mtk_dpi_conf mt8196_dpintf_conf = {
	.cal_factor = mt8196_dpintf_calculate_factor_v2,
	.max_clock_khz = 600000,
	.output_fmts = mt8196_output_fmts,
	.num_output_fmts = ARRAY_SIZE(mt8196_output_fmts),
	.pixels_per_iter = 4,
	.input_2pixel = true,
	.dimension_mask = DPINTF_HPW_MASK,
	.hvsize_mask = DPINTF_HSIZE_MASK,
	.channel_swap_shift = DPINTF_CH_SWAP,
	.yuv422_en_bit = DPINTF_YUV422_EN,
	.csc_enable_bit = DPINTF_CSC_ENABLE,
	.dsc_support = true,
	.use_version_2 = true,
	.pol_positive_default = true,
	.hfp_adjustment_for_bs = true,
};

static void mt8196_get_clk_v2(struct mtk_dpi *dp_intf)
{
	drm_dbg_kms(dp_intf->bridge.dev, "[DPTX] get clk\n");

	dp_intf->pclk = devm_clk_get(dp_intf->dev, "mux_dp");
	dp_intf->pclk_src[TCK_26M] = devm_clk_get(dp_intf->dev, "dpi_26m");
	dp_intf->pclk_src[TVDPLL_D4] = devm_clk_get(dp_intf->dev, "tvdpll_d4");
	dp_intf->pclk_src[TVDPLL_D8] = devm_clk_get(dp_intf->dev, "tvdpll_d8");
	dp_intf->pclk_src[TVDPLL_D16] = devm_clk_get(dp_intf->dev, "tvdpll_d16");
	dp_intf->pclk_src[TVDPLL_PLL] = devm_clk_get(dp_intf->dev, "dpi_ck");

	if (IS_ERR(dp_intf->pclk) ||
	    IS_ERR(dp_intf->pclk_src[TCK_26M]) ||
		IS_ERR(dp_intf->pclk_src[TVDPLL_D4]) ||
		IS_ERR(dp_intf->pclk_src[TVDPLL_D8]) ||
		IS_ERR(dp_intf->pclk_src[TVDPLL_D16]) ||
		IS_ERR(dp_intf->pclk_src[TVDPLL_PLL]))
		dev_err(dp_intf->dev, "[DPTX] Failed to get pclk andr src clock, -%d-%d-%d-%d-%d-%d-\n",
			IS_ERR(dp_intf->pclk),
			IS_ERR(dp_intf->pclk_src[TCK_26M]),
			IS_ERR(dp_intf->pclk_src[TVDPLL_D4]),
			IS_ERR(dp_intf->pclk_src[TVDPLL_D8]),
			IS_ERR(dp_intf->pclk_src[TVDPLL_D16]),
			IS_ERR(dp_intf->pclk_src[TVDPLL_PLL]));
}

static int mtk_dpi_probe_v2(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *endpoint, *port;
	struct mtk_dpi *dpi;
	int ret;

	dpi = devm_kzalloc(dev, sizeof(*dpi), GFP_KERNEL);
	if (!dpi)
		return -ENOMEM;

	dpi->dev = dev;
	dpi->conf = (struct mtk_dpi_conf *)of_device_get_match_data(dev);
	dpi->output_fmt = MEDIA_BUS_FMT_RGB888_1X24;

	/* To support MST, the DP bridge (dp-tx) driver registers one bridge per input port. */
	endpoint = of_graph_get_endpoint_by_regs(dev->of_node, 0, 0);
	if (!endpoint) {
		dev_err(dev, "[DPTX] fail to find endpoint!\n");
		return -ENODEV;
	}

	port = of_graph_get_remote_port(endpoint);
	of_node_put(endpoint);
	if (!port) {
		dev_err(dev, "[DPTX] fail to find remote port!\n");
		return -ENODEV;
	}

	dpi->next_bridge = of_drm_find_bridge(port);
	of_node_put(port);
	if (!dpi->next_bridge) {
		dev_err(dev, "[DPTX] fail to find next bridge\n");
		return -EPROBE_DEFER;
	}
	dev_dbg(dev, "[DPTX] find next_bridge:%pOF", dpi->next_bridge->of_node);

	dpi->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(dpi->pinctrl)) {
		dpi->pinctrl = NULL;
		dev_dbg(&pdev->dev, "[DPTX] Cannot find pinctrl!\n");
	}
	if (dpi->pinctrl) {
		dpi->pins_gpio = pinctrl_lookup_state(dpi->pinctrl, "sleep");
		if (IS_ERR(dpi->pins_gpio)) {
			dpi->pins_gpio = NULL;
			dev_dbg(&pdev->dev, "[DPTX] Cannot find pinctrl idle!\n");
		}
		if (dpi->pins_gpio)
			pinctrl_select_state(dpi->pinctrl, dpi->pins_gpio);

		dpi->pins_dpi = pinctrl_lookup_state(dpi->pinctrl, "default");
		if (IS_ERR(dpi->pins_dpi)) {
			dpi->pins_dpi = NULL;
			dev_dbg(&pdev->dev, "[DPTX] Cannot find pinctrl active!\n");
		}
	}
	dpi->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dpi->regs))
		return dev_err_probe(dev, PTR_ERR(dpi->regs),
				     "[DPTX] Failed to ioremap mem resource\n");

	dpi->hf_fmm_ck = devm_clk_get(dev, "hf_fmm_ck");
	if (IS_ERR(dpi->hf_fmm_ck)) {
		ret = PTR_ERR(dpi->hf_fmm_ck);
		dev_err(dev, "[DPTX] Failed to get hf_fmm_ck clock: %d\n", ret);
		return ret;
	}
	dpi->hf_fdp_ck = devm_clk_get(dev, "hf_fdp_ck");
	if (IS_ERR(dpi->hf_fdp_ck)) {
		ret = PTR_ERR(dpi->hf_fdp_ck);
		dev_err(dev, "[DPTX] Failed to get hf_fdp_ck clock: %d\n", ret);
		return ret;
	}

	mt8196_get_clk_v2(dpi);

	mtk_dpi_v2 = dpi->conf->use_version_2;

	dpi->irq = platform_get_irq(pdev, 0);
	if (dpi->irq < 0)
		return dpi->irq;

	platform_set_drvdata(pdev, dpi);

	dpi->bridge.funcs = &mtk_dpi_bridge_funcs;
	dpi->bridge.of_node = dev->of_node;
	dpi->bridge.type = DRM_MODE_CONNECTOR_DPI;

	ret = devm_drm_bridge_add(dev, &dpi->bridge);
	if (ret)
		return ret;

	ret = component_add(dev, &mtk_dpi_component_ops);
	if (ret)
		return dev_err_probe(dev, ret, "[DPTX] Failed to add component.\n");

	return 0;
}

static void mtk_dpi_remove_v2(struct platform_device *pdev)
{
	component_del(&pdev->dev, &mtk_dpi_component_ops);
}

static const struct of_device_id mtk_dpi_of_ids_v2[] = {
	{ .compatible = "mediatek,mt8196-dp-intf", .data = &mt8196_dpintf_conf },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mtk_dpi_of_ids_v2);

struct platform_driver mtk_dpi_driver_v2 = {
	.probe = mtk_dpi_probe_v2,
	.remove_new = mtk_dpi_remove_v2,
	.driver = {
		.name = "mediatek-dpi-v2",
		.of_match_table = mtk_dpi_of_ids_v2,
	},
};
