// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung S6E3FC3 DriverIC Panels driver
 *
 * Copyright (c) 2025 Collabora Ltd.
 *                    AngeloGioacchino Del Regno <angelogioacchino.delregno@collabora.com>
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#define MCS_LEVEL1_KEY			0x9f
#define MCS_LEVEL2_KEY			0xf0
#define MCS_LEVEL3_KEY			0xfc

#define S6E3FC3_WRCTRLD_DIMMING		BIT(3)
#define S6E3FC3_WRCTRLD_LOCAL_HBM	BIT(4)
#define S6E3FC3_WRCTRLD_BCTRL		BIT(5)
#define S6E3FC3_WRCTRLD_GLOBAL_HBM	GENMASK(7, 6)

#define S6E3FC3_CMD(...)				\
	{						\
		.len = sizeof((u8[]) { __VA_ARGS__ }),	\
		.data = (u8[]) { __VA_ARGS__ }		\
	}

struct s6e3fc3_cmd {
	size_t len;
	const u8 *data;
};

struct s6e3fc3_panel_desc {
	u16 width_mm;
	u16 height_mm;

	const struct mipi_dsi_device_info dsi_info;
	const struct drm_display_mode *modes;
	size_t num_modes;

	const struct s6e3fc3_cmd *ffc_setting;
	size_t num_ffc_setting;
};

struct s6e3fc3 {
	struct device *dev;
	struct drm_panel panel;
	struct mipi_dsi_device *dsi[2];
	struct backlight_device *bl_dev;
	struct s6e3fc3_panel_desc *pdata;

	struct regulator_bulk_data supplies[2];
	struct gpio_desc *reset_gpio;
	struct drm_dsc_config dsc;

	bool dimming_on;
	bool hbm_on;

	const struct drm_display_mode *mode;
};

static inline struct s6e3fc3 *to_s63efc3(struct drm_panel *panel)
{
	return container_of(panel, struct s6e3fc3, panel);
}

static int s6e3fc3_key_unlock(struct s6e3fc3 *ctx, u8 key_level, bool unlock)
{
	struct mipi_dsi_device *dsi = ctx->dsi[0];
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };
	bool cmd_sel = unlock;
	u8 cmd_key[3];

	switch (key_level) {
		case 1:
			cmd_key[0] = MCS_LEVEL1_KEY;
			break;
		case 2:
			cmd_key[0] = MCS_LEVEL2_KEY;
			cmd_sel = !cmd_sel;
			break;
		case 3:
			cmd_key[0] = MCS_LEVEL3_KEY;
			cmd_sel = !cmd_sel;
			break;
		default:
			return -EINVAL;
	}

	/*
	 * Level 1 unlocks with 0xa5 and locks with 0x5a, while levels 2 and 3
	 * want the inverse (so they unlock with 0xa5 and lock with 0x5a).
	 */
	if (cmd_sel) {
		cmd_key[1] = 0xa5;
		cmd_key[2] = 0xa5;
	} else {
		cmd_key[1] = 0x5a;
		cmd_key[2] = 0x5a;
	}

	mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, cmd_key, ARRAY_SIZE(cmd_key));
	if (dsi_ctx.accum_err)
		return dsi_ctx.accum_err;

	return 0;
}

static int s6e3fc3_set_fps(struct s6e3fc3 *ctx, u8 freq_hz)
{
	struct mipi_dsi_device *dsi = ctx->dsi[0];
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };
	const u8 s6e3fc3_cmd_fps_ltps_update[] = { 0xf7, 0x0f };
	u8 s6e3fc3_cmd_fps_mode[] = { 0x60, 0 };
	int ret;

	/* Unlock level 2 settings */
	ret = s6e3fc3_key_unlock(ctx, 2, true);
	if (ret)
		return ret;

	if (freq_hz > 60 && freq_hz <= 120)
		s6e3fc3_cmd_fps_mode[1] = 0x08;

	mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, s6e3fc3_cmd_fps_mode,
					ARRAY_SIZE(s6e3fc3_cmd_fps_mode));
	mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, s6e3fc3_cmd_fps_ltps_update,
					ARRAY_SIZE(s6e3fc3_cmd_fps_ltps_update));

	/* Lock again unconditionally */
	ret = s6e3fc3_key_unlock(ctx, 2, false);
	if (ret)
		return ret;

	if (dsi_ctx.accum_err)
		return dsi_ctx.accum_err;

	/* Wait until setting changed */
	usleep_range(15000, 16000);

	return 0;
}

static int s6e5fc3_set_freq_con(struct s6e3fc3 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi[0];
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };
	const u8 s6e3fc3_cmd_fq_con1[] = { 0xb0, 0x27, 0xf2 };
	const u8 s6e3fc3_cmd_fq_set[] = { 0xf2, 0 };
	int ret;

	/* Unlock level 2 setting */
	ret = s6e3fc3_key_unlock(ctx, 2, true);
	if (ret)
		return ret;

	mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, s6e3fc3_cmd_fq_con1,
					ARRAY_SIZE(s6e3fc3_cmd_fq_con1));
	mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, s6e3fc3_cmd_fq_set,
					ARRAY_SIZE(s6e3fc3_cmd_fq_set));

	/* Lock again unconditionally */
	ret = s6e3fc3_key_unlock(ctx, 2, false);
	if (ret)
		return ret;

	if (dsi_ctx.accum_err)
		return dsi_ctx.accum_err;

	return 0;
}

static int s6e5fc3_set_ddi_ffc(struct s6e3fc3 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi[0];
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };
	int i, ret, ret_b;

	/* Unlock level 2 settings */
	ret = s6e3fc3_key_unlock(ctx, 2, true);
	if (ret)
		return ret;

	ret = s6e3fc3_key_unlock(ctx, 3, true);
	if (ret) {
		s6e3fc3_key_unlock(ctx, 2, false);
		return ret;
	}

	for (i = 0; i < ctx->pdata->num_ffc_setting; i++) {
		const struct s6e3fc3_cmd *ffc_cmds = &ctx->pdata->ffc_setting[i];
		mipi_dsi_dcs_write_buffer_multi(&dsi_ctx,
						ffc_cmds->data, ffc_cmds->len);
		if (dsi_ctx.accum_err)
			break;
	};

	/* Lock again unconditionally */
	ret = s6e3fc3_key_unlock(ctx, 3, false);
	ret_b = s6e3fc3_key_unlock(ctx, 2, false);
	if (ret)
		return ret;
	if (ret_b)
		return ret_b;
	if (dsi_ctx.accum_err)
		return dsi_ctx.accum_err;

	return 0;
}


static void s6e3fc3_reset(struct s6e3fc3 *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(2000, 3000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
}

static int s6e3fc3_set_bctrl(struct s6e3fc3 *ctx)
{
	struct mipi_dsi_device *dsi = ctx->dsi[0];
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };
	u8 s6e3fc3_cmd_wrctrld[2] = { MIPI_DCS_WRITE_CONTROL_DISPLAY,
				      S6E3FC3_WRCTRLD_BCTRL };

	if (ctx->hbm_on)
		s6e3fc3_cmd_wrctrld[1] |= S6E3FC3_WRCTRLD_GLOBAL_HBM;

	if (ctx->dimming_on)
		s6e3fc3_cmd_wrctrld[1] |= S6E3FC3_WRCTRLD_DIMMING;

	mipi_dsi_dcs_write_buffer_multi(&dsi_ctx, s6e3fc3_cmd_wrctrld,
					ARRAY_SIZE(s6e3fc3_cmd_wrctrld));
	if (dsi_ctx.accum_err)
		return dsi_ctx.accum_err;

	return 0;
}

static int s6e3fc3_prepare(struct drm_panel *panel)
{
	struct s6e3fc3 *ctx = to_s63efc3(panel);
	struct mipi_dsi_device *dsi = ctx->dsi[0];
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };
	struct drm_dsc_picture_parameter_set pps;
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
	if (ret < 0)
		return ret;

	/* Wait for DDIC boot */
	usleep_range(5000, 6000);

	s6e3fc3_reset(ctx);

	/* DSC params setting doesn't need any unlock key */
	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);
	mipi_dsi_picture_parameter_set_multi(&dsi_ctx, &pps);
	mipi_dsi_compression_mode_ext_multi(&dsi_ctx, true,
					    MIPI_DSI_COMPRESSION_DSC, 1);

	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	if (dsi_ctx.accum_err)
		return dsi_ctx.accum_err;

	/* Wait for sleep-out */
	mipi_dsi_msleep(&dsi_ctx, 30);

	/* Set TE mode */
	mipi_dsi_dcs_set_tear_on_multi(&dsi_ctx, MIPI_DSI_DCS_TEAR_MODE_VBLANK);

	/* CASET/PASET Setting */
	mipi_dsi_dcs_set_column_address_multi(&dsi_ctx, 0, ctx->mode->hdisplay - 1);
	mipi_dsi_dcs_set_page_address_multi(&dsi_ctx, 0, ctx->mode->vdisplay - 1);
	if (dsi_ctx.accum_err)
		return dsi_ctx.accum_err;

	/* FQ CON Setting */
	ret = s6e5fc3_set_freq_con(ctx);
	if (ret)
		return ret;

	/* Set wrctrld: HBM off, dimming off at startup */
	ret = s6e3fc3_set_bctrl(ctx);
	if (ret)
		return ret;

	/* FPS setting */
	ret = s6e3fc3_set_fps(ctx, 90);
	if (ret)
		return ret;

	/* FFC Setting */
	ret = s6e5fc3_set_ddi_ffc(ctx);
	if (ret)
		return ret;

	return 0;
}

static int s6e3fc3_enable(struct drm_panel *panel)
{
	struct s6e3fc3 *ctx = to_s63efc3(panel);
	struct mipi_dsi_device *dsi = ctx->dsi[0];
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };

	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 120);

	return dsi_ctx.accum_err;
}

static int s6e3fc3_disable(struct drm_panel *panel)
{
	struct s6e3fc3 *ctx = to_s63efc3(panel);
	struct mipi_dsi_device *dsi = ctx->dsi[0];
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 20);

	return dsi_ctx.accum_err;
}

static int s6e3fc3_unprepare(struct drm_panel *panel)
{
	struct s6e3fc3 *ctx = to_s63efc3(panel);
	struct mipi_dsi_device *dsi = ctx->dsi[0];
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = dsi };

	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	return regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
}

static int s6e3fc3_get_modes(struct drm_panel *panel,
				struct drm_connector *connector)
{
	struct s6e3fc3 *ctx = to_s63efc3(panel);
	int i;

	for (i = 0; i < ctx->pdata->num_modes; i++) {
		const struct drm_display_mode *m = &ctx->pdata->modes[i];
		struct drm_display_mode *mode;

		mode = drm_mode_duplicate(connector->dev, m);
		if (!mode)
			return -ENOMEM;

		drm_mode_set_name(mode);

		mode->type |= DRM_MODE_TYPE_DRIVER;
		if (ctx->pdata->num_modes == 1)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_probed_add(connector, mode);
	}

	connector->display_info.bpc = 8;
	connector->display_info.height_mm = ctx->pdata->height_mm;
	connector->display_info.width_mm = ctx->pdata->width_mm;

	return ctx->pdata->num_modes;
}

static const struct drm_panel_funcs s6e3fc3_funcs = {
	.disable = s6e3fc3_disable,
	.unprepare = s6e3fc3_unprepare,
	.prepare = s6e3fc3_prepare,
	.enable = s6e3fc3_enable,
	.get_modes = s6e3fc3_get_modes,
};

static int s6e3fc3_backlight_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness = backlight_get_brightness(bl);
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
	ret = mipi_dsi_dcs_set_display_brightness_large(dsi, brightness);
	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	if (ret) {
		dev_err(&dsi->dev, "Failed to write display brightness: %d", ret);
		return ret;
	}

	return 0;
}

static int s6e3fc3_backlight_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	u16 brightness;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
	ret = mipi_dsi_dcs_get_display_brightness_large(dsi, &brightness);
	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	return ret < 0 ? ret : brightness;
}

static const struct backlight_ops s6e3fc3_bl_ops = {
	.update_status = s6e3fc3_backlight_update_status,
	.get_brightness = s6e3fc3_backlight_get_brightness,
};

static struct backlight_device *s6e3fc3_create_backlight(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = 1023,
		.max_brightness = 1023,
		.scale = BACKLIGHT_SCALE_NON_LINEAR,
	};

	return devm_backlight_device_register(dev, dev_name(dev), dev, dsi,
					      &s6e3fc3_bl_ops, &props);
}

static int s6e3fc3_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi_r;
	struct s6e3fc3 *ctx;
	int i, ret;

	ctx = devm_kzalloc(dev, sizeof(struct s6e3fc3), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = dev;
	ctx->dsi[0] = dsi;
	ctx->dsi[0]->lanes = 4;
	ctx->dsi[0]->format = MIPI_DSI_FMT_RGB888;
	ctx->dsi[0]->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM;
	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->supplies[0].supply = "vddio";
	ctx->supplies[1].supply = "vci";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->supplies),
				      ctx->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "cannot get reset-gpio\n");

	dsi_r = of_graph_get_remote_node(dsi->dev.of_node, 1, -1);
	if (dsi_r) {
		const struct mipi_dsi_device_info *info = &ctx->pdata->dsi_info;
		struct mipi_dsi_host *dsi_r_host;

		dsi_r_host = of_find_mipi_dsi_host_by_node(dsi_r);
		of_node_put(dsi_r);
		if (!dsi_r_host)
			return dev_err_probe(dev, -EPROBE_DEFER,
					     "Cannot get secondary DSI host\n");

		ctx->dsi[1] = devm_mipi_dsi_device_register_full(dev, dsi_r_host, info);
		if (IS_ERR(ctx->dsi[1]))
			return dev_err_probe(dev, PTR_ERR(ctx->dsi[1]),
					     "Cannot get secondary DSI node\n");

		ctx->dsi[1]->lanes = ctx->dsi[0]->lanes;
		ctx->dsi[1]->format = ctx->dsi[0]->format;
		ctx->dsi[1]->mode_flags = ctx->dsi[0]->mode_flags;
		mipi_dsi_set_drvdata(ctx->dsi[1], ctx);
	}

	if (!dsi_r)
		return dev_err_probe(dev, -ENODEV, "Cannot get secondary DSI node.\n");

	drm_panel_init(&ctx->panel, dev, &s6e3fc3_funcs, DRM_MODE_CONNECTOR_DSI);
	ctx->panel.prepare_prev_first = true;

	ctx->bl_dev = s6e3fc3_create_backlight(dsi);
	if (IS_ERR(ctx->bl_dev))
		return dev_err_probe(dev, PTR_ERR(ctx->bl_dev),
				     "failed to register backlight device\n");

	drm_panel_add(&ctx->panel);

	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 1;
	ctx->dsc.pic_width = 1080;
	ctx->dsc.pic_height = 2400;
	ctx->dsc.slice_height = 30;
	ctx->dsc.slice_width = 540;
	ctx->dsc.slice_chunk_size = 540;
	ctx->dsc.slice_count = ctx->dsc.pic_width / ctx->dsc.slice_width;
	ctx->dsc.line_buf_depth = 9;
	ctx->dsc.bits_per_component = 8;
	ctx->dsc.bits_per_pixel = 8 << 4; /* 4 fractional bits */
	ctx->dsc.convert_rgb = true;
	ctx->dsc.block_pred_enable = true;

	ctx->dsc.rc_edge_factor = 6;
	ctx->dsc.rc_tgt_offset_high = 3;
	ctx->dsc.rc_tgt_offset_low = 3;
	ctx->dsc.initial_xmit_delay = 200;
	ctx->dsc.initial_dec_delay = 526;
	ctx->dsc.initial_scale_value = 32;
	ctx->dsc.scale_increment_interval = 739;
	ctx->dsc.scale_decrement_interval = 7;
	ctx->dsc.first_line_bpg_offset = 12;
	ctx->dsc.nfl_bpg_offset = 848;
	ctx->dsc.slice_bpg_offset = 868;
	ctx->dsc.initial_offset = 6144;
	ctx->dsc.final_offset = 4336;
	ctx->dsc.flatness_min_qp = 3;
	ctx->dsc.flatness_max_qp = 12;
	ctx->dsc.rc_model_size = 8192;
	ctx->dsc.rc_quant_incr_limit0 = 11;
	ctx->dsc.rc_quant_incr_limit1 = 11;
/*
	ctx->dsc.rc_buf_thresh = (u16[]){
		14, 28, 42, 56, 70, 84, 98, 105,
		112, 119, 121, 123, 125, 126
	};
*/
/*
	ctx->dsc.rc_range_params = {
		258, 256, 2368, 2494, 6652, 6650, 6648,
		6712, 6776 6838 10998 11060 11124 15220 27636
	};
*/
	for (i = 0; i < 2; i++) {
		if (!ctx->dsi[i])
			continue;

		/* This panel only supports DSC; unconditionally enable it */
		dsi[i].dsc = &ctx->dsc;

		ret = devm_mipi_dsi_attach(dev, ctx->dsi[i]);
		if (ret < 0) {
			drm_panel_remove(&ctx->panel);
			return ret;
		}
	}

	return 0;
}

static void s6e3fc3_remove(struct mipi_dsi_device *dsi)
{
	struct s6e3fc3 *ctx = mipi_dsi_get_drvdata(dsi);

	drm_panel_remove(&ctx->panel);
}

static const struct s6e3fc3_cmd s6e3fc3_ams643ye05_1711mbps_ffc[] = {
	S6E3FC3_CMD(0xb0, 0x00, 0x2a, 0xc5),
	S6E3FC3_CMD(0xc5, 0x0d, 0x10, 0x80, 0x45),
	S6E3FC3_CMD(0xb0, 0x00, 0x3e, 0xc5),
	S6E3FC3_CMD(0xc5, 0x32, 0xb1),
};

static const struct s6e3fc3_cmd s6e3fc3_ams646yd04_1711mbps_ffc[] = {
	S6E3FC3_CMD(0xb0, 0x00, 0x2a, 0xc5),
	S6E3FC3_CMD(0xc5, 0x0d, 0x10, 0x80, 0x45),
	S6E3FC3_CMD(0xb0, 0x00, 0x3e, 0xc5),
	S6E3FC3_CMD(0xc5, 0x36, 0x41),
};

static const struct s6e3fc3_cmd s6e3fc3_ams667ym01_1711mbps_ffc[] = {
	S6E3FC3_CMD(0xb0, 0x66, 0xc5),
	S6E3FC3_CMD(0xc5, 0x00, 0x8c),
	S6E3FC3_CMD(0xb0, /*0x00*/ 0x2a, 0xc5),
	S6E3FC3_CMD(0xc5, 0x0d, 0x10, 0x80, 0x45),
	S6E3FC3_CMD(0xb0, /*0x00*/ 0x3e, 0xc5),
	S6E3FC3_CMD(0xc5, 0x4d, 0xa6), //0x8e, 0x82),
};

static const struct drm_display_mode ams667ym01_modes[] = {
	{	/* 90 Hz */
		.clock = (1080 + 80 + 84 + 88) * (2400 + 15 + 2 + 2) * 90 / 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + 80,
		.hsync_end = 1080 + 80 + 84,
		.htotal = 1080 + 80 + 84 + 88,
		.vdisplay = 2400,
		.vsync_start = 2400 + 15,
		.vsync_end = 2400 + 15 + 2,
		.vtotal = 2400 + 15 + 2 + 2,
	},
	{	/* 60 Hz */
		.clock = (1080 + 80 + 84 + 88) * (2400 + 16 + 2 + 2) * 60 / 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + 80,
		.hsync_end = 1080 + 80 + 84,
		.htotal = 1080 + 80 + 84 + 88,
		.vdisplay = 2400,
		.vsync_start = 2400 + 16,
		.vsync_end = 2400 + 16 + 2,
		.vtotal = 2400 + 16 + 2 + 2,
	},
};

static const struct drm_display_mode ams643ye05_modes[] = {
	{	/* 90 Hz */
		.clock = (1080 + 40 + 10 + 20) * (2400 + 20 + 2 + 8) * 90 / 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + 40,
		.hsync_end = 1080 + 40 + 10,
		.htotal = 1080 + 40 + 10 + 20,
		.vdisplay = 2400,
		.vsync_start = 2400 + 20,
		.vsync_end = 2400 + 20 + 2,
		.vtotal = 2400 + 20 + 2 + 8,
	},
	{	/* 60 Hz */
		.clock = (1080 + 80 + 84 + 88) * (2400 + 16 + 2 + 2) * 60 / 1000,
		.hdisplay = 1080,
		.hsync_start = 1080 + 40,
		.hsync_end = 1080 + 40 + 10,
		.htotal = 1080 + 40 + 10 + 20,
		.vdisplay = 2400,
		.vsync_start = 2400 + 20,
		.vsync_end = 2400 + 20 + 2,
		.vtotal = 2400 + 20 + 2 + 8,
	},
};

static const struct s6e3fc3_panel_desc ams643ye05_desc = {
	.modes = ams643ye05_modes,
	.num_modes = ARRAY_SIZE(ams643ye05_modes),
	.dsi_info = {
		.type = "s6e3fc3-ams643ye05",
		.channel = 0,
		.node = NULL,
	},
	.width_mm = 68,
	.height_mm = 152,
	.ffc_setting = s6e3fc3_ams643ye05_1711mbps_ffc,
	.num_ffc_setting = ARRAY_SIZE(s6e3fc3_ams643ye05_1711mbps_ffc),
};

static const struct s6e3fc3_panel_desc ams646yd04_desc = {
	.modes = ams667ym01_modes,
	.num_modes = ARRAY_SIZE(ams667ym01_modes),
	.dsi_info = {
		.type = "s6e3fc3-ams646yd04",
		.channel = 0,
		.node = NULL,
	},
	.width_mm = 67,
	.height_mm = 150,
	.ffc_setting = s6e3fc3_ams646yd04_1711mbps_ffc,
	.num_ffc_setting = ARRAY_SIZE(s6e3fc3_ams646yd04_1711mbps_ffc),
};

static const struct s6e3fc3_panel_desc ams667ym01_desc = {
	.modes = ams667ym01_modes,
	.num_modes = ARRAY_SIZE(ams667ym01_modes),
	.dsi_info = {
		.type = "s6e3fc3-ams667ym01",
		.channel = 0,
		.node = NULL,
	},
	.width_mm = 70,
	.height_mm = 155,
	.ffc_setting = s6e3fc3_ams667ym01_1711mbps_ffc,
	.num_ffc_setting = ARRAY_SIZE(s6e3fc3_ams667ym01_1711mbps_ffc),
};

static const struct of_device_id s6e3fc3_of_match[] = {
	{ .compatible = "samsung,ams643ye05", .data = &ams643ye05_desc },
	{ .compatible = "samsung,ams646yd04", .data = &ams646yd04_desc },
	{ .compatible = "samsung,ams667ym01", .data = &ams667ym01_desc },
	{ }
};
MODULE_DEVICE_TABLE(of, s6e3fc3_of_match);

static struct mipi_dsi_driver s6e3fc3_driver = {
	.probe = s6e3fc3_probe,
	.remove = s6e3fc3_remove,
	.driver = {
		.name = "panel_samsung_s6e3fc3",
		.of_match_table = s6e3fc3_of_match,
	},
};
module_mipi_dsi_driver(s6e3fc3_driver);

MODULE_AUTHOR("Inki Dae <inki.dae@samsung.com>");
MODULE_AUTHOR("Hoegeun Kwon <hoegeun.kwon@samsung.com>");
MODULE_DESCRIPTION("MIPI-DSI based s6e3fc3 AMOLED LCD Panel Driver");
MODULE_LICENSE("GPL v2");
