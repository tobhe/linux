// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 MediaTek Inc.
 */
#include <linux/bitops.h>
#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/printk.h>

#define POSDIV_PURIFY	BIT(2)
#define PREDIV		7
#define REF_FREQUENCY	26
#define SHUFFLE_OFFSET	0x700

/*--------------------------------------------------------------------------*/
/* Register Offset                                                          */
/*--------------------------------------------------------------------------*/
#define DPHY_DVFS_STA		0x0e98
#define APHY_PHYPLL2		0x0908
#define APHY_CLRPLL2		0x0928
#define APHY_PHYPLL3		0x090c
#define APHY_CLRPLL3		0x092c
#define APHY_PHYPLL4		0x0910
#define APHY_ARPI0		0x0d94
#define APHY_CA_ARDLL1		0x0d08
#define APHY_B0_TX0		0x0dc4

/*--------------------------------------------------------------------------*/
/* Register Mask                                                            */
/*--------------------------------------------------------------------------*/
#define DPHY_DVFS_SHU_LV	GENMASK(15, 14)
#define DPHY_DVFS_PLL_SEL	GENMASK(25, 25)
#define APHY_PLL2_SDMPCW	GENMASK(18, 3)
#define APHY_PLL3_POSDIV	GENMASK(13, 11)
#define APHY_PLL4_FBKSEL	GENMASK(6, 6)
#define APHY_ARPI0_SOPEN	GENMASK(26, 26)
#define APHY_ARDLL1_CK_EN	GENMASK(0, 0)
#define APHY_B0_TX0_SER_MODE	GENMASK(4, 3)

static unsigned int read_reg_field(void __iomem *base, unsigned int offset, unsigned int mask)
{
	unsigned int val = readl(base + offset);
	unsigned int shift = __ffs(mask);

	return (val & mask) >> shift;
}

struct mtk_dramc_pdata {
	unsigned int fmeter_version;
};

struct mtk_dramc_dev_t {
	void __iomem *anaphy_base;
	void __iomem *ddrphy_base;
	const struct mtk_dramc_pdata *pdata;
};

static int mtk_dramc_probe(struct platform_device *pdev)
{
	struct mtk_dramc_dev_t *dramc;
	const struct mtk_dramc_pdata *pdata;
	int ret;

	dramc = devm_kzalloc(&pdev->dev, sizeof(struct mtk_dramc_dev_t), GFP_KERNEL);
	if (!dramc)
		return dev_err_probe(&pdev->dev, -ENOMEM, "Failed to allocate memory\n");

	pdata = of_device_get_match_data(&pdev->dev);
	if (!pdata)
		return dev_err_probe(&pdev->dev, -EINVAL, "No platform data available\n");

	dramc->pdata = pdata;

	dramc->anaphy_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dramc->anaphy_base)) {
		ret = PTR_ERR(dramc->anaphy_base);
		return dev_err_probe(&pdev->dev, ret, "Unable to map DDRPHY NAO base\n");
	}

	dramc->ddrphy_base = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(dramc->ddrphy_base)) {
		ret = PTR_ERR(dramc->ddrphy_base);
		return dev_err_probe(&pdev->dev, ret, "Unable to map DDRPHY AO base\n");
	}

	platform_set_drvdata(pdev, dramc);
	return 0;
}

static unsigned int mtk_fmeter_v1(struct mtk_dramc_dev_t *dramc)
{
	unsigned int shu_level, pll_sel, offset;
	unsigned int sdmpcw, posdiv, ckdiv4, fbksel, sopen, async_ca, ser_mode;
	unsigned int perdiv_freq, posdiv_freq, vco_freq;
	unsigned int final_rate;

	shu_level = read_reg_field(dramc->ddrphy_base, DPHY_DVFS_STA, DPHY_DVFS_SHU_LV);
	pll_sel = read_reg_field(dramc->ddrphy_base, DPHY_DVFS_STA, DPHY_DVFS_PLL_SEL);
	offset = SHUFFLE_OFFSET * shu_level;

	sdmpcw = read_reg_field(dramc->anaphy_base,
				((pll_sel == 0) ? APHY_PHYPLL2 : APHY_CLRPLL2) + offset,
				APHY_PLL2_SDMPCW);
	posdiv = read_reg_field(dramc->anaphy_base,
				((pll_sel == 0) ? APHY_PHYPLL3 : APHY_CLRPLL3) + offset,
				APHY_PLL3_POSDIV);
	fbksel = read_reg_field(dramc->anaphy_base, APHY_PHYPLL4 + offset, APHY_PLL4_FBKSEL);
	sopen = read_reg_field(dramc->anaphy_base, APHY_ARPI0 + offset, APHY_ARPI0_SOPEN);
	async_ca = read_reg_field(dramc->anaphy_base, APHY_CA_ARDLL1 + offset, APHY_ARDLL1_CK_EN);
	ser_mode = read_reg_field(dramc->anaphy_base, APHY_B0_TX0 + offset, APHY_B0_TX0_SER_MODE);

	ckdiv4 = (ser_mode == 1) ? 1 : 0;
	posdiv &= ~(POSDIV_PURIFY);

	perdiv_freq = REF_FREQUENCY * (sdmpcw >> PREDIV);
	posdiv_freq = (perdiv_freq >> posdiv) >> 1;
	vco_freq = posdiv_freq << fbksel;
	final_rate = vco_freq >> ckdiv4;

	if (sopen == 1 && async_ca == 1)
		final_rate >>= 1;

	return final_rate;
}

/*
 * mtk_dramc_get_data_rate - calculate DRAM data rate
 *
 * Returns DRAM data rate (MB/s)
 */
static unsigned int mtk_dramc_get_data_rate(struct device *dev)
{
	struct mtk_dramc_dev_t *dramc_dev = dev_get_drvdata(dev);

	if (!dramc_dev) {
		dev_err(dev, "DRAMC device data not found\n");
		return -EINVAL;
	}

	if (dramc_dev->pdata) {
		if (dramc_dev->pdata->fmeter_version == 1)
			return mtk_fmeter_v1(dramc_dev);

		dev_err(dev, "Unsupported fmeter version\n");
		return -EINVAL;
	}
	dev_err(dev, "DRAMC platform data not found\n");
	return -EINVAL;
}

static ssize_t dram_data_rate_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "DRAM data rate = %u\n",
			mtk_dramc_get_data_rate(dev));
}

static DEVICE_ATTR_RO(dram_data_rate);

static struct attribute *mtk_dramc_attrs[] = {
	&dev_attr_dram_data_rate.attr,
	NULL
};
ATTRIBUTE_GROUPS(mtk_dramc);

static const struct mtk_dramc_pdata dramc_pdata_mt8196 = {
	.fmeter_version = 1
};

static const struct of_device_id mtk_dramc_of_ids[] = {
	{ .compatible = "mediatek,mt8196-dramc", .data = &dramc_pdata_mt8196 },
	{}
};
MODULE_DEVICE_TABLE(of, mtk_dramc_of_ids);

static struct platform_driver mtk_dramc_driver = {
	.probe = mtk_dramc_probe,
	.driver = {
		.name = "mtk_dramc_drv",
		.of_match_table = mtk_dramc_of_ids,
		.dev_groups = mtk_dramc_groups,
	},
};

module_platform_driver(mtk_dramc_driver);

MODULE_AUTHOR("Crystal Guo <crystal.guo@mediatek.com>");
MODULE_DESCRIPTION("MediaTek DRAM Controller Driver");
MODULE_LICENSE("GPL");
