// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 Collabora Ltd.
 * Author: AngeloGioacchino Del Regno <angelogioacchino.delregno@collabora.com>
 */

#include <dt-bindings/clock/mediatek,mt8173-clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include "clk-gate.h"
#include "clk-mtk.h"

#define MFG_ACTIVE_PWR_CON0		0x24
 #define PWR_ON_RST_B_DELAY_CNT		GENMASK(7, 0)
 #define CLK_EN_DELAY_CNT		GENMASK(15, 8)
 #define CLK_DIS_DELAY_CNT		GENMASK(23, 16)
 #define EVT_FORCE_ABORT		BIT(30)
 #define ACTIVE_PWRCTL_EN		BIT(31)
#define MFG_ACTIVE_PWR_CON1		0x28
 #define PWR_ON_S_DELAY_CNT		GENMASK(7, 0)
 #define PWR_OFF_ISO_DELAY_CNT		GENMASK(15, 8)
 #define PWR_ON_ISO_DELAY_CNT		GENMASK(23, 16)
 #define PWR_OFF_RST_B_DELAY_CNT	GENMASK(31, 24)

/* Number of cycles for clock controller power on/off, clken/clkdis */
#define PWR_ON_RST_B_DELAY_NUM_CYC	77
#define PWR_OFF_RST_B_DELAY_NUM_CYC	77
#define CLK_EN_DELAY_NUM_CYC		61
#define CLK_DIS_DELAY_NUM_CYC		60
#define PWR_ON_S_DELAY_NUM_CYC		11
#define PWR_OFF_ISO_DELAY_NUM_CYC	68
#define PWR_ON_ISO_DELAY_NUM_CYC	69

static const struct mtk_gate_regs mfg_cg_regs = {
	.set_ofs = 0x4,
	.clr_ofs = 0x8,
	.sta_ofs = 0x0,
};

#define GATE_MFG(_id, _name, _parent, _shift)			\
	GATE_MTK(_id, _name, _parent, &mfg_cg_regs, _shift, &mtk_clk_gate_ops_setclr)

static const struct mtk_gate mfg_clks[] = {
	GATE_MFG(CLK_MFG_BAXI, "mfg_baxi", "axi_mfg_in_sel", 0),
	GATE_MFG(CLK_MFG_BMEM, "mfg_bmem", "mem_mfg_in_sel", 1),
	GATE_MFG(CLK_MFG_BG3D, "mfg_bg3d", "mfg_sel", 2),
	GATE_MFG(CLK_MFG_B26M, "mfg_b26m", "clk26m", 3),
};

static const struct mtk_clk_desc mfg_desc = {
	.clks = mfg_clks,
	.num_clks = ARRAY_SIZE(mfg_clks),
};

static int clk_mt8173_mfgclk_controller_setup(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	const struct mtk_clk_desc *mcd;
	void __iomem *base = NULL;
	u32 conx;
	int ret;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR_OR_NULL(base))
		return IS_ERR(base) ? PTR_ERR(base) : -ENOMEM;

	val = FIELD_PREP(PWR_ON_RST_B_DELAY_CNT, PWR_ON_RST_B_DELAY_NUM_CYC) |
	      FIELD_PREP(CLK_EN_DELAY_CNT, CLK_EN_DELAY_NUM_CYC) |
	      FIELD_PREP(CLK_DIS_DELAY_CNT, CLK_DIS_DELAY_NUM_CYC) |
	      FIELD_PREP(EVT_FORCE_ABORT, 0) |
	      FIELD_PREP(ACTIVE_PWRCTL_EN, 0);
	writel(val, base + MFG_ACTIVE_PWR_CON0);

	val = FIELD_PREP(PWR_ON_S_DELAY_CNT, PWR_ON_S_DELAY_NUM_CYC) |
	      FIELD_PREP(PWR_OFF_ISO_DELAY_CNT, PWR_OFF_ISO_DELAY_NUM_CYC) |
	      FIELD_PREP(PWR_ON_ISO_DELAY_CNT, PWR_ON_ISO_DELAY_NUM_CYC) |
	      FIELD_PREP(PWR_OFF_RST_B_DELAY, PWR_OFF_RST_B_DELAY_NUM_CYC);
	writel(val, base + MFG_ACTIVE_PWR_CON1);

	return 0;
}

static int clk_mt8173_mfg_probe(struct platform_device *pdev)
{
	int ret;

	ret = mtk_clk_simple_probe(pdev);
	if (ret)
		return ret;

	ret = clk_mt8173_mfgclk_controller_setup(pdev);
	if (ret) {
		mtk_clk_simple_remove(pdev);
		return ret;
	}

	return 0;
}

static const struct of_device_id of_match_clk_mt8173_mfg[] = {
	{ .compatible = "mediatek,mt8173-mfgcfg", .data = &mfg_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt8173_mfg);

static struct platform_driver clk_mt8173_mfg_drv = {
	.driver = {
		.name = "clk-mt8173-mfg",
		.of_match_table = of_match_clk_mt8173_mfg,
	},
	.probe = mtk_clk_simple_probe,
	.remove_new = mtk_clk_simple_remove,
};
module_platform_driver(clk_mt8173_mfg_drv);

MODULE_DESCRIPTION("MediaTek MT8173 mfg clocks driver");
MODULE_LICENSE("GPL");
