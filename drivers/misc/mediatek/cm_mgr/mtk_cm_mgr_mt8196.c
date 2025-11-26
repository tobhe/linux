// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2023 MediaTek Inc.
 */

/* system includes */
#include <linux/atomic.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/interconnect.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_opp.h>
#include <linux/pm_qos.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/suspend.h>
#include <linux/time.h>
#include <linux/topology.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>
#include "mtk_cm_ipi.h"
#include "mtk_cm_mgr_common.h"
#include "mtk_cm_mgr_mt8196.h"

/*****************************************************************************
 *  Variables
 *****************************************************************************/
static spinlock_t cm_mgr_lock;
static unsigned int prev_freq_idx[CM_MGR_CPU_CLUSTER];
static unsigned int prev_freq[CM_MGR_CPU_CLUSTER];

static struct cm_mgr_hook *local_hk;

static void __iomem *csram_base;

#define CM_CHIP_VER		0x0002
/*****************************************************************************
 *  Platform functions
 *****************************************************************************/

static int cm_get_base_addr(struct device *dev)
{
	int ret = 0;
	u32 cpuhvfs_id = 0;
	struct device_node *dn = NULL;
	struct platform_device *pdev = NULL;
	struct resource *csram_res = NULL;

	/* get cpufreq driver base address */
	if (of_property_read_u32(dev->of_node, "cpuhvfs", &cpuhvfs_id)) {
		dev_err(dev, "Fail to read couhvfs phandle\n");
		return -EINVAL;
	}
	dn = of_find_node_by_phandle(cpuhvfs_id);
	if (!dn) {
		ret = -ENOMEM;
		dev_err(dev, "find cpuhvfs node failed\n");
		goto ERROR;
	}

	pdev = of_find_device_by_node(dn);
	of_node_put(dn);
	if (!pdev) {
		ret = -ENODEV;
		dev_err(dev, "cpuhvfs is not ready\n");
		goto ERROR;
	}

	csram_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!csram_res) {
		ret = -ENODEV;
		dev_err(dev, "cpuhvfs resource is not found\n");
		goto ERROR;
	}

	csram_base = ioremap(csram_res->start, resource_size(csram_res));
	if (IS_ERR_OR_NULL((void *)csram_base)) {
		ret = -ENOMEM;
		dev_err(dev, "find csram base failed\n");
		goto ERROR;
	}

ERROR:
	return ret;
}

unsigned int csram_read(unsigned int offs)
{
	if (IS_ERR_OR_NULL((void *)csram_base))
		return 0;
	return __raw_readl(csram_base + (offs));
}

void csram_write(unsigned int offs, unsigned int val)
{
	if (IS_ERR_OR_NULL((void *)csram_base))
		return;
	__raw_writel(val, csram_base + (offs));
}

static void check_cm_mgr_status_mt8196(unsigned int cluster, unsigned int freq,
					   unsigned int idx)
{
	unsigned int bcpu_opp_max;
	unsigned long spinlock_save_flag;

	if (!cm_mgr_get_enable())
		return;

	spin_lock_irqsave(&cm_mgr_lock, spinlock_save_flag);

	prev_freq_idx[cluster] = idx;
	prev_freq[cluster] = freq;

	if (prev_freq_idx[CM_MGR_B] < prev_freq_idx[CM_MGR_BB])
		bcpu_opp_max = prev_freq_idx[CM_MGR_B];
	else
		bcpu_opp_max = prev_freq_idx[CM_MGR_BB];

	spin_unlock_irqrestore(&cm_mgr_lock, spinlock_save_flag);
	cm_mgr_update_dram_by_cpu_opp(bcpu_opp_max);
}

static int cm_mgr_check_dts_setting_mt8196(struct platform_device *pdev)
{
	int ret = 0;
	int temp = 0;
	struct device_node *node = pdev->dev.of_node;
	struct icc_path *bw_path;

	ret = of_count_phandle_with_args(node, "required-opps", NULL);
	if (ret > 0) {
		cm_mgr_set_num_perf(ret);
		dev_dbg(&pdev->dev, "required_opps count %d\n", ret);
	} else {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "fail to get required_opps count from dts.\n");
		goto ERROR;
	}

	ret = of_property_read_u32(node, "mediatek,cm-mgr-num-array", &temp);
	if (ret) {
		dev_err(&pdev->dev, "fail to get cm_mgr_num_array from dts. ret %d\n", ret);
		goto ERROR;
	} else {
		cm_mgr_set_num_array(temp);
		dev_dbg(&pdev->dev, "cm_mgr_num_array %d\n", cm_mgr_get_num_array());
	}

	bw_path = of_icc_get(&pdev->dev, "cm-perf-bw");
	if (IS_ERR(bw_path)) {
		cm_mgr_set_bw_path(NULL);
		dev_dbg(&pdev->dev, "fail to get cm_perf_bw path from dts.\n");
		ret = -ENOMEM;
		goto ERROR;
	} else
		cm_mgr_set_bw_path(bw_path);

	ret = of_property_read_bool(node, "mediatek,cm-perf-mode-enable");
	if (ret) {
		cm_mgr_set_perf_mode_enable(1);
		dev_dbg(&pdev->dev, "cm_perf_mode_enable %d\n", cm_mgr_get_perf_mode_enable());
	} else {
		cm_mgr_set_perf_mode_enable(0);
		dev_err_probe(&pdev->dev, -ENODEV, "not enable cm_perf_mode!\n");
	}

	ret = of_property_read_u32(node, "mediatek,cm-perf-mode-ceiling-opp", &temp);
	if (ret) {
		dev_err(&pdev->dev, "fail to get cm_perf_mode_ceiling_opp from dts. ret %d\n", ret);
		goto ERROR;
	} else {
		cm_mgr_set_perf_mode_ceiling_opp(temp);
		dev_dbg(&pdev->dev, "cm_perf_mode_ceiling_opp %d\n", cm_mgr_get_perf_mode_ceiling_opp());
	}

	ret = of_property_read_u32(node, "mediatek,cm-perf-mode-thd", &temp);
	if (ret) {
		dev_err(&pdev->dev, "fail to get cm_perf_mode_thd from dts. ret %d\n", ret);
		goto ERROR;
	} else {
		cm_mgr_set_perf_mode_thd(temp);
		dev_dbg(&pdev->dev, "cm_perf_mode_thd %d\n", cm_mgr_get_perf_mode_thd());
	}

	return 0;

ERROR:
	return ret;
}

static int platform_cm_mgr_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;

	spin_lock_init(&cm_mgr_lock);
	ret = cm_mgr_check_dts_setting_mt8196(pdev);
	if (ret) {
		dev_err(dev, "%s(%d): fail to get platform data from dts. ret %d\n",
			__func__, __LINE__, ret);
		goto ERROR;
	}

	ret = cm_mgr_check_dts_setting(pdev);
	if (ret) {
		dev_err(dev, "%s(%d): fail to get common data from dts. ret %d\n",
			__func__, __LINE__, ret);
		goto ERROR;
	}

	ret = cm_mgr_common_init(pdev);
	if (ret) {
		dev_err(dev, "%s(%d): fail to common init. ret %d\n", __func__,
			__LINE__, ret);
		goto ERROR;
	}

	ret = cm_get_base_addr(dev);
	if (ret) {
		dev_err(dev, "%s(%d): fail to get cm csram base. ret %d\n", __func__,
			__LINE__, ret);
		goto ERROR;
	}

	local_hk = devm_kzalloc(&pdev->dev, sizeof(struct cm_mgr_hook), GFP_KERNEL);

	local_hk->check_cm_mgr_status = check_cm_mgr_status_mt8196;
#if IS_REACHABLE(CONFIG_MTK_CM_IPI)
	local_hk->cm_mgr_to_sspm_command = cm_mgr_to_sspm_command_ipi;
#else
	local_hk->cm_mgr_to_sspm_command = cm_mgr_to_sspm_command_common;
#endif
	cm_mgr_register_hook(local_hk);

	dev_pm_genpd_set_performance_state(&pdev->dev, 0);
#if IS_REACHABLE(CONFIG_MTK_CM_IPI)
	cm_mgr_get_sspm_version();
#endif

	local_hk->cm_mgr_to_sspm_command(IPI_CM_MGR_PERF_MODE_ENABLE,
				   cm_mgr_get_perf_mode_enable());
	local_hk->cm_mgr_to_sspm_command(IPI_CM_MGR_PERF_MODE_CEILING_OPP,
				   cm_mgr_get_perf_mode_ceiling_opp());
	local_hk->cm_mgr_to_sspm_command(IPI_CM_MGR_PERF_MODE_THD,
				   cm_mgr_get_perf_mode_thd());
	local_hk->cm_mgr_to_sspm_command(IPI_CM_MGR_CHIP_VER, CM_CHIP_VER);
	local_hk->cm_mgr_to_sspm_command(IPI_CM_MGR_ENABLE, cm_mgr_get_enable());

	dev_info(dev, "%s(%d): platform-cm_mgr_probe Done.\n", __func__, __LINE__);

	return 0;

ERROR:
	return ret;
}

static int platform_cm_mgr_remove(struct platform_device *pdev)
{
	cm_mgr_unregister_hook(local_hk);
	icc_put(cm_mgr_get_bw_path());

	return 0;
}

static const struct of_device_id platform_cm_mgr_of_match[] = {
	{
		.compatible = "mediatek,mt8196-cm-mgr",
	},
	{},
};
MODULE_DEVICE_TABLE(of, platform_cm_mgr_of_match);

static struct platform_driver mtk_platform_cm_mgr_driver = {
	.probe = platform_cm_mgr_probe,
	.remove = platform_cm_mgr_remove,
	.driver = {
		.name = "mt8196-cm-mgr",
		.owner = THIS_MODULE,
		.of_match_table = platform_cm_mgr_of_match,
	},
};

module_platform_driver(mtk_platform_cm_mgr_driver);

MODULE_SOFTDEP("post: mtk-dvfsrc pre: tinysys-scmi");
MODULE_DESCRIPTION("Mediatek cm_mgr driver");
MODULE_AUTHOR("Carlos Hung <carlos.hung@mediatek.com>");
MODULE_LICENSE("GPL");
