// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek Secure System Power Manager driver
 *
 * Copyright (c) 2011-2015 MediaTek Inc.
 * Copyright (c) 2025 Collabora Ltd
 *                    AngeloGioacchino Del Regno <angelogioacchino.delregno@collabora.com>
 */

#include <clocksource/arm_arch_timer.h>
#include <linux/clk.h>
#include <linux/clocksource.h>
#include <linux/devm-helpers.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/sched/clock.h>
#include <linux/timecounter.h>
#include <linux/workqueue.h>

#define MTK_SSPM_TIMESYNC_REG_TICK_H		0
#define MTK_SSPM_TIMESYNC_REG_TICK_L		0x4
#define MTK_SSPM_TIMESYNC_REG_TS_H		0x8
#define MTK_SSPM_TIMESYNC_REG_TS_L		0xc
#define MTK_SSPM_TIMESYNC_HDR_CNTR_MASK		GENMASK(30, 28)
#define MTK_SSPM_TIMESYNC_HDR_FREEZE		BIT(31)
#define MTK_SSPM_TIMESYNC_TIME_HI_MASK		GENMASK(27, 0)

/* Synchronize time with the SSPM every 60 seconds */
#define MTK_SSPM_TIMESYNC_SYNC_TIME_MSEC	(60 * MSEC_PER_SEC)

/* This timecounter wraps every hour */
#define MTK_SSPM_CYCLECOUNTER_MAX_SECS		(60 * 60)

struct mtk_sspm {
	void __iomem *base;

	struct cyclecounter timesync_cc;
	struct timecounter timesync_tc;
	struct delayed_work timesync_work;

	/* Note: We rely on this variable wrapping, counter is 0..7 */
	u8 timesync_cur_cnt : 3;
};

static void mtk_sspm_timesync(struct mtk_sspm *sspm, u64 cntr, u64 tick, u8 flags)
{
	volatile u32 tick_lo, tick_hi, stime_lo, stime_hi;
	u32 header;

	/* Prepare the header */
	sspm->timesync_cur_cnt++;
	header = FIELD_PREP(MTK_SSPM_TIMESYNC_HDR_CNTR_MASK, sspm->timesync_cur_cnt);
	header |= flags;

	/* The upper 32 bits of both tick and ts regs also contain the header */
	tick_hi = FIELD_PREP(MTK_SSPM_TIMESYNC_TIME_HI_MASK, upper_32_bits(tick));
	tick_hi |= header;
	tick_lo = lower_32_bits(tick);

	stime_hi = FIELD_PREP(MTK_SSPM_TIMESYNC_TIME_HI_MASK, upper_32_bits(cntr));
	stime_hi |= header;
	stime_lo = lower_32_bits(cntr);

	/* Write all values: TICK hi/lo and TS lo/hi in this order */
	writel(tick_hi, sspm->base + MTK_SSPM_TIMESYNC_REG_TICK_H);
	writel(tick_lo, sspm->base + MTK_SSPM_TIMESYNC_REG_TICK_L);
	writel(stime_lo, sspm->base + MTK_SSPM_TIMESYNC_REG_TS_L);
	writel(stime_hi, sspm->base + MTK_SSPM_TIMESYNC_REG_TS_H);
}

static void mtk_sspm_timesync_update_work(struct work_struct *work)
{
	struct mtk_sspm *sspm = container_of(to_delayed_work(work),
					     struct mtk_sspm, timesync_work);
	u64 cntr, tick;

	cntr = timecounter_read(&sspm->timesync_tc);
	tick = sspm->timesync_tc.cycle_last;

	mtk_sspm_timesync(sspm, cntr, tick, 0);

	queue_delayed_work(system_power_efficient_wq, &sspm->timesync_work,
			   msecs_to_jiffies(MTK_SSPM_TIMESYNC_SYNC_TIME_MSEC));
}

static void mtk_sspm_timesync_start_timecounter(struct mtk_sspm *sspm)
{
	u64 start_timestamp = sched_clock();

	timecounter_init(&sspm->timesync_tc, &sspm->timesync_cc, start_timestamp);
}

static u64 mtk_sspm_timesync_tick_read(struct cyclecounter *cc)
{
	return arch_timer_read_counter();
}

static int mtk_sspm_probe(struct platform_device *pdev)
{
	struct mtk_sspm *sspm;
	int ret;

	sspm = devm_kzalloc(&pdev->dev, sizeof(*sspm), GFP_KERNEL);
	if (!sspm)
		return -ENOMEM;

	sspm->base = devm_platform_get_and_ioremap_resource(pdev, 0, NULL);
	if (IS_ERR(sspm->base))
		return PTR_ERR(sspm->base);

	ret = devm_delayed_work_autocancel(&pdev->dev, &sspm->timesync_work,
					   mtk_sspm_timesync_update_work);
	if (ret)
		return ret;

	dev_set_drvdata(&pdev->dev, sspm);

	sspm->timesync_cc = (struct cyclecounter) {
		.read = mtk_sspm_timesync_tick_read,
		.mask = CYCLECOUNTER_MASK(56),
	};

	/* Initialize multiplier and shift of the cyclecounter */
	clocks_calc_mult_shift(&sspm->timesync_cc.mult, &sspm->timesync_cc.shift,
			       arch_timer_get_rate(), NSEC_PER_SEC,
			       MTK_SSPM_CYCLECOUNTER_MAX_SECS);

	/* Timecounter init and first timesync must be at the end */
	mtk_sspm_timesync_start_timecounter(sspm);

	/* Synchronize all co-processors time immediately */
	queue_delayed_work(system_power_efficient_wq, &sspm->timesync_work, 0);

	return 0;
}

static int mtk_sspm_suspend(struct device *dev)
{
	struct mtk_sspm *sspm = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&sspm->timesync_work);

	return 0;
}

static int mtk_sspm_resume(struct device *dev)
{
	struct mtk_sspm *sspm = dev_get_drvdata(dev);

	/* Reinitialize the timecounter to sync with sched clock */
	mtk_sspm_timesync_start_timecounter(sspm);

	/* Synchronize all co-processors time at resume */
	queue_delayed_work(system_power_efficient_wq, &sspm->timesync_work, 0);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(mtk_sspm_pm_ops, mtk_sspm_suspend, mtk_sspm_resume);

static const struct of_device_id mtk_sspm_match[] = {
	{ .compatible = "mediatek,mt8196-sspm" },
	{ /* sentinel */ }
};

static struct platform_driver mtk_sspm = {
	.probe = mtk_sspm_probe,
	.driver = {
		.name = "mtk-sspm",
		.of_match_table = mtk_sspm_match,
		.pm = &mtk_sspm_pm_ops,
	},
};
module_platform_driver(mtk_sspm);
MODULE_AUTHOR("AngeloGioacchino Del Regno <angelogioacchino.delregno@collabora.com>");
MODULE_DESCRIPTION("MediaTek Secured Power Manager driver");
MODULE_LICENSE("GPL");
