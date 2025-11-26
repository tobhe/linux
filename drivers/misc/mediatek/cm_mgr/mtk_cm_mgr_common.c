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
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/sched.h>
#include <linux/sched/clock.h>
#include <linux/sched/rt.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/suspend.h>
#include <linux/sysfs.h>
#include <linux/time.h>
#include <linux/topology.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#if IS_REACHABLE(CONFIG_MTK_DVFSRC)
#include "mtk_dvfsrc.h"
#endif /* CONFIG_MTK_DVFSRC */

#include <linux/kallsyms.h>
#include <linux/tracepoint.h>
#include <trace/events/power.h>

#include "mtk_cm_mgr_common.h"
#if IS_REACHABLE(CONFIG_MTK_CM_IPI)
#include "mtk_cm_ipi.h"
#endif /* CONFIG_MTK_CM_IPI */

/*****************************************************************************
 *  Variables
 *****************************************************************************/
static struct cm_mgr_common_data cmmgr_data = {
	.cm_mgr_disable_fb = 1,
	.cm_mgr_emi_demand_check = 1,
	.cm_mgr_dram_opp_base = -1,
	.cm_mgr_dram_opp_ceiling = -1,
	.cm_mgr_dram_opp_floor = -1,
	.cm_mgr_loading_level = 1000,
	.debounce_times_perf_down = 50,
	.debounce_times_perf_force_down = 100,
	.debounce_times_perf_down_local = -1,
	.debounce_times_perf_down_force_local = -1,
};

static struct cm_mgr_cp_data cmmgr_data_cp = {
	.cm_mgr_enable = 1,
	.cm_mgr_use_cpu_to_dram_map = 1,
	.cpu_power_bcpu_weight_max = 100,
	.cpu_power_bcpu_weight_min = 100,
	.cpu_power_bbcpu_weight_max = 100,
	.cpu_power_bbcpu_weight_min = 100,
};

/*****************************************************************************
 *  Common api
 *****************************************************************************/
int cm_mgr_get_enable(void)
{
	return cmmgr_data_cp.cm_mgr_enable;
}
EXPORT_SYMBOL_GPL(cm_mgr_get_enable);

struct icc_path *cm_mgr_get_bw_path(void)
{
	return cmmgr_data.cm_mgr_bw_path;
}
EXPORT_SYMBOL_GPL(cm_mgr_get_bw_path);

struct icc_path *cm_mgr_set_bw_path(struct icc_path *bw_path)
{
	return cmmgr_data.cm_mgr_bw_path = bw_path;
}
EXPORT_SYMBOL_GPL(cm_mgr_set_bw_path);

int debounce_times_perf_force_down_get(void)
{
	return cmmgr_data.debounce_times_perf_force_down;
}
EXPORT_SYMBOL_GPL(debounce_times_perf_force_down_get);

#if IS_REACHABLE(CONFIG_MTK_CM_IPI)
void cm_mgr_get_sspm_version(void)
{
	cmmgr_data.cm_mgr_sspm_version = cmmgr_data.hk.cm_mgr_to_sspm_command(
		IPI_CM_MGR_SSPM_VER | IPI_CM_MGR_SCMI_GET, 0);
}
EXPORT_SYMBOL_GPL(cm_mgr_get_sspm_version);
#endif

void cm_mgr_set_num_perf(int num)
{
	cmmgr_data.cm_mgr_num_perf = num;
}
EXPORT_SYMBOL_GPL(cm_mgr_set_num_perf);

int cm_mgr_get_num_array(void)
{
	return cmmgr_data.cm_mgr_num_array;
}
EXPORT_SYMBOL_GPL(cm_mgr_get_num_array);

void cm_mgr_set_num_array(int num)
{
	cmmgr_data.cm_mgr_num_array = num;
}
EXPORT_SYMBOL_GPL(cm_mgr_set_num_array);

void cm_mgr_perf_set_status(int enable)
{
	if (cmmgr_data.hk.cm_mgr_perf_set_status)
		cmmgr_data.hk.cm_mgr_perf_set_status(enable);
}
EXPORT_SYMBOL_GPL(cm_mgr_perf_set_status);

void cm_mgr_set_perf_mode_enable(int enable)
{
	cmmgr_data.cm_perf_mode_enable = enable;
}
EXPORT_SYMBOL_GPL(cm_mgr_set_perf_mode_enable);

int cm_mgr_get_perf_mode_enable(void)
{
	return cmmgr_data.cm_perf_mode_enable;
}
EXPORT_SYMBOL_GPL(cm_mgr_get_perf_mode_enable);

void cm_mgr_set_perf_mode_ceiling_opp(int opp)
{
	cmmgr_data.cm_perf_mode_ceiling_opp = opp;
}
EXPORT_SYMBOL_GPL(cm_mgr_set_perf_mode_ceiling_opp);

int cm_mgr_get_perf_mode_ceiling_opp(void)
{
	return cmmgr_data.cm_perf_mode_ceiling_opp;
}
EXPORT_SYMBOL_GPL(cm_mgr_get_perf_mode_ceiling_opp);

void cm_mgr_set_perf_mode_thd(int thd)
{
	cmmgr_data.cm_perf_mode_thd = thd;
}
EXPORT_SYMBOL_GPL(cm_mgr_set_perf_mode_thd);

int cm_mgr_get_perf_mode_thd(void)
{
	return cmmgr_data.cm_perf_mode_thd;
}
EXPORT_SYMBOL_GPL(cm_mgr_get_perf_mode_thd);

void cm_mgr_register_hook(struct cm_mgr_hook *hook)
{
	cmmgr_data.hk.cm_mgr_get_perfs = hook->cm_mgr_get_perfs;
	cmmgr_data.hk.cm_mgr_perf_set_force_status = hook->cm_mgr_perf_set_force_status;
	cmmgr_data.hk.check_cm_mgr_status = hook->check_cm_mgr_status;
	cmmgr_data.hk.cm_mgr_perf_platform_set_status =
		hook->cm_mgr_perf_platform_set_status;
	cmmgr_data.hk.cm_mgr_perf_set_status = hook->cm_mgr_perf_set_status;
#if IS_REACHABLE(CONFIG_MTK_CM_IPI)
	cmmgr_data.hk.cm_mgr_to_sspm_command = cm_mgr_to_sspm_command_ipi;
#else
	cmmgr_data.hk.cm_mgr_to_sspm_command = cm_mgr_to_sspm_command_common;
#endif
}
EXPORT_SYMBOL_GPL(cm_mgr_register_hook);

void cm_mgr_unregister_hook(struct cm_mgr_hook *hook)
{
	cmmgr_data.hk.cm_mgr_get_perfs = NULL;
	cmmgr_data.hk.cm_mgr_perf_set_force_status = NULL;
	cmmgr_data.hk.check_cm_mgr_status = NULL;
	cmmgr_data.hk.cm_mgr_perf_platform_set_status = NULL;
	cmmgr_data.hk.cm_mgr_perf_set_status = NULL;
	cmmgr_data.hk.cm_mgr_to_sspm_command = NULL;
}
EXPORT_SYMBOL_GPL(cm_mgr_unregister_hook);

int cm_mgr_check_dts_setting(struct platform_device *pdev)
{
	const char *buf;
	int ret;
	int opp_count;
	struct resource *res;
	struct device *dev = &pdev->dev;
	struct device_node *node = pdev->dev.of_node;

	ret = of_property_read_bool(node, "mediatek,cm-mgr-enable");
	if (ret) {
		cmmgr_data_cp.cm_mgr_enable = 1;
		dev_dbg(dev, "cm_mgr_enable %d\n", cmmgr_data_cp.cm_mgr_enable);
	} else {
		cmmgr_data_cp.cm_mgr_enable = 0;
		dev_err_probe(dev, -ENODEV, "not enable cm_mgr!\n");
	}

	ret = of_property_read_string(node, "mediatek,cm-mgr-arch", &buf);
	if (!ret) {
		if (!strcmp(buf, "v1p"))
			cmmgr_data_cp.cm_mgr_arch = CM_MGR_ARCH_V1P;
		else {
			dev_err_probe(dev, ret, "fail to get correct cm_mgr_arch from dts.\n");
			ret = -ENODEV;
			goto ERROR;
		}
	} else
		cmmgr_data_cp.cm_mgr_arch = CM_MGR_ARCH_V1;
	dev_dbg(dev, "cm_mgr_arch %d\n", cmmgr_data_cp.cm_mgr_arch);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cm_mgr_base");
	cmmgr_data.cm_mgr_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(cmmgr_data.cm_mgr_base)) {
		dev_err_probe(dev, ret, "fail to ioremap registers.\n");
		ret = -EFAULT;
		goto ERROR;
	} else
		dev_dbg(dev, "cm_mgr_base %p\n", cmmgr_data.cm_mgr_base);

	opp_count = of_count_phandle_with_args(node, "mediatek,cm-mgr-cpu-opp-to-dram",
					       NULL);
	if (opp_count > 0) {
		dev_dbg(dev, "opp_count %d\n", opp_count);
		cmmgr_data.cm_mgr_cpu_opp_size = opp_count;
	} else {
		dev_err_probe(dev, opp_count, "fail to get opp_count from dts.\n");
		ret = -ENODEV;
		goto ERROR;
	}

	cmmgr_data.cm_mgr_cpu_opp_to_dram = devm_kzalloc(
		dev, sizeof(int) * cmmgr_data.cm_mgr_cpu_opp_size, GFP_KERNEL);
	if (!cmmgr_data.cm_mgr_cpu_opp_to_dram) {
		ret = -ENOMEM;
		goto ERROR;
	}

	ret = of_property_read_u32_array(node, "mediatek,cm-mgr-cpu-opp-to-dram",
					 cmmgr_data.cm_mgr_cpu_opp_to_dram,
					 cmmgr_data.cm_mgr_cpu_opp_size);
	if (ret) {
		dev_err_probe(dev, ret, "fail to get cmmgr_data.cm_mgr_cpu_opp_to_dram from dts. ret %d\n", ret);
		goto ERROR;
	}

	ret = of_property_read_bool(node, "mediatek,use-cpu-to-dram-map");
	if (ret) {
		cmmgr_data_cp.cm_mgr_use_cpu_to_dram_map = 1;
		dev_dbg(dev, "cm_mgr_use_cpu_to_dram_map %d\n", cmmgr_data_cp.cm_mgr_use_cpu_to_dram_map);
	} else {
		cmmgr_data_cp.cm_mgr_use_cpu_to_dram_map = 0;
		dev_err_probe(dev, -ENODEV, "not enable cm_mgr_cpu_opp_to_dram!\n");
	}

	ret = of_property_read_bool(node, "mediatek,use-cpu-to-dram-map-new");
	if (ret) {
		cmmgr_data_cp.cm_mgr_use_cpu_to_dram_map_new = 1;
		dev_dbg(dev, "cm_mgr_use_cpu_to_dram_map_new %d\n", cmmgr_data_cp.cm_mgr_use_cpu_to_dram_map_new);
	} else {
		cmmgr_data_cp.cm_mgr_use_cpu_to_dram_map_new = 0;
		dev_err_probe(dev, -ENODEV, "not enable cm_mgr_use_cpu_to_dram_map_new!\n");
	}

	if (cmmgr_data_cp.cm_mgr_arch == CM_MGR_ARCH_V1) {
		ret = of_property_read_bool(node, "use-bcpu-weight");
		if (ret) {
			cmmgr_data_cp.cm_mgr_use_bcpu_weight = 1;
			dev_dbg(dev, "cm_mgr_use_bcpu_weight %d\n", cmmgr_data_cp.cm_mgr_use_bcpu_weight);
		} else {
			cmmgr_data_cp.cm_mgr_use_bcpu_weight = 0;
			dev_err_probe(dev, -ENODEV, "not enable cm_mgr_use_bcpu_weight!\n");
		}

		ret = of_property_read_s32(node, "cpu-power-bcpu-weight-max",
					   &cmmgr_data_cp.cpu_power_bcpu_weight_max);
		if (ret)
			cmmgr_data_cp.cpu_power_bcpu_weight_max = 100;
		dev_dbg(dev, "cpu_power_bcpu_weight_max %d\n", cmmgr_data_cp.cpu_power_bcpu_weight_max);

		ret = of_property_read_s32(node, "cpu-power-bcpu-weight-min",
					   &cmmgr_data_cp.cpu_power_bcpu_weight_min);
		if (ret)
			cmmgr_data_cp.cpu_power_bcpu_weight_min = 100;
		dev_dbg(dev, "cpu_power_bcpu_weight_min %d\n", cmmgr_data_cp.cpu_power_bcpu_weight_min);

		ret = of_property_read_s32(node, "cpu-power-bbcpu-weight-max",
					   &cmmgr_data_cp.cpu_power_bbcpu_weight_max);
		if (ret)
			cmmgr_data_cp.cpu_power_bbcpu_weight_max = 100;

		dev_dbg(dev, "cpu_power_bbcpu_weight_max %d\n", cmmgr_data_cp.cpu_power_bbcpu_weight_max);

		ret = of_property_read_s32(node, "cpu-power-bbcpu-weight-min",
					   &cmmgr_data_cp.cpu_power_bbcpu_weight_min);
		if (ret)
			cmmgr_data_cp.cpu_power_bbcpu_weight_min = 100;
		dev_dbg(dev, "cpu_power_bbcpu_weight_min %d\n", cmmgr_data_cp.cpu_power_bbcpu_weight_min);
	}

	cmmgr_data.cm_mgr_buf = devm_kzalloc(dev, sizeof(int) * 6 * cmmgr_data.cm_mgr_num_array,
				  GFP_KERNEL);

	if (!cmmgr_data.cm_mgr_buf) {
		ret = -ENOMEM;
		goto ERROR;
	}

	cmmgr_data.cpu_power_ratio_down = cmmgr_data.cm_mgr_buf;
	cmmgr_data.cpu_power_ratio_up = cmmgr_data.cpu_power_ratio_down + cmmgr_data.cm_mgr_num_array;
	cmmgr_data.debounce_times_down_adb = cmmgr_data.cpu_power_ratio_up + cmmgr_data.cm_mgr_num_array;
	cmmgr_data.debounce_times_up_adb = cmmgr_data.debounce_times_down_adb + cmmgr_data.cm_mgr_num_array;
	cmmgr_data.vcore_power_ratio_down = cmmgr_data.debounce_times_up_adb + cmmgr_data.cm_mgr_num_array;
	cmmgr_data.vcore_power_ratio_up = cmmgr_data.vcore_power_ratio_down + cmmgr_data.cm_mgr_num_array;

	ret = of_property_read_u32_array(
		node, "mediatek,cm-mgr,cp-down", cmmgr_data.cpu_power_ratio_down, cmmgr_data.cm_mgr_num_array);
	ret = of_property_read_u32_array(node, "mediatek,cm-mgr,cp-up",
					 cmmgr_data.cpu_power_ratio_up, cmmgr_data.cm_mgr_num_array);
	ret = of_property_read_u32_array(node, "mediatek,cm-mgr,dt-down",
					 cmmgr_data.debounce_times_down_adb,
					 cmmgr_data.cm_mgr_num_array);
	ret = of_property_read_u32_array(
		node, "mediatek,cm-mgr,dt-up", cmmgr_data.debounce_times_up_adb, cmmgr_data.cm_mgr_num_array);
	ret = of_property_read_u32_array(node, "mediatek,cm-mgr,vp-down",
					 cmmgr_data.vcore_power_ratio_down,
					 cmmgr_data.cm_mgr_num_array);
	ret = of_property_read_u32_array(
		node, "mediatek,cm-mgr,vp-up", cmmgr_data.vcore_power_ratio_up, cmmgr_data.cm_mgr_num_array);

	return 0;

ERROR:
	return ret;
}
EXPORT_SYMBOL_GPL(cm_mgr_check_dts_setting);

void cm_mgr_update_dram_by_cpu_opp(int cpu_opp)
{
	int dram_opp = 0;

	if (!cmmgr_data_cp.cm_mgr_use_cpu_to_dram_map || !cmmgr_data.cm_work_flag)
		return;

	if (cmmgr_data_cp.cm_mgr_arch == CM_MGR_ARCH_V1 && cmmgr_data.cm_mgr_disable_fb == 1 &&
	    cmmgr_data.cm_mgr_blank_status == 1) {
		if (cmmgr_data.cm_mgr_cpu_to_dram_opp != cmmgr_data.cm_mgr_num_perf) {
			cmmgr_data.cm_mgr_cpu_to_dram_opp = cmmgr_data.cm_mgr_num_perf;
			schedule_delayed_work(&cmmgr_data.cm_mgr_work, 1);
		}
		return;
	}

	if (!cmmgr_data_cp.cm_mgr_cpu_map_dram_enable) {
		if (cmmgr_data.cm_mgr_cpu_to_dram_opp != cmmgr_data.cm_mgr_num_perf) {
			cmmgr_data.cm_mgr_cpu_to_dram_opp = cmmgr_data.cm_mgr_num_perf;
			schedule_delayed_work(&cmmgr_data.cm_mgr_work, 1);
		}
		return;
	}

	if ((cpu_opp >= 0) && (cpu_opp < cmmgr_data.cm_mgr_cpu_opp_size))
		dram_opp = cmmgr_data.cm_mgr_cpu_opp_to_dram[cpu_opp];

	cmmgr_data.cm_mgr_cpu_to_dram_opp = dram_opp;
	schedule_delayed_work(&cmmgr_data.cm_mgr_work, 1);
}
EXPORT_SYMBOL_GPL(cm_mgr_update_dram_by_cpu_opp);

static void cm_mgr_cpu_frequency_tracer(void *ignore, unsigned int frequency,
					unsigned int cpu_id)
{
	int cpu = 0, cluster = 0;
	struct cpufreq_policy *policy = NULL;
	unsigned int idx = 0;

	if (!cmmgr_data_cp.cm_mgr_cpu_map_dram_enable) {
		if (cmmgr_data.cm_work_flag && cmmgr_data.cm_mgr_cpu_to_dram_opp != cmmgr_data.cm_mgr_num_perf) {
			cmmgr_data.cm_mgr_cpu_to_dram_opp = cmmgr_data.cm_mgr_num_perf;
			schedule_delayed_work(&cmmgr_data.cm_mgr_work, 1);
		}
		return;
	}

	policy = cpufreq_cpu_get(cpu_id);
	if (!policy)
		return;
	if (cpu_id != cpumask_first(policy->related_cpus)) {
		cpufreq_cpu_put(policy);
		return;
	}
	cpufreq_cpu_put(policy);

	for_each_possible_cpu(cpu) {
		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			break;
		cpu = cpumask_first(policy->related_cpus);
		if (cpu == cpu_id)
			break;
		cpu = cpumask_last(policy->related_cpus);
		cluster++;
		cpufreq_cpu_put(policy);
	}

	if (policy) {
		idx = cpufreq_frequency_table_target(policy, frequency,
						     policy->min, policy->max,
						     CPUFREQ_RELATION_L);
		if (cmmgr_data.hk.check_cm_mgr_status)
			cmmgr_data.hk.check_cm_mgr_status(cluster, frequency, idx);
		cpufreq_cpu_put(policy);
	}
}

struct tracepoints_table cm_mgr_tracepoints = {
	.name = "cpu_frequency",
	.func = cm_mgr_cpu_frequency_tracer,
};

static void lookup_tracepoints(struct tracepoint *tp, void *ignore)
{
	if (strcmp(cm_mgr_tracepoints.name, tp->name) == 0)
		cm_mgr_tracepoints.tp = tp;
}

void tracepoint_cleanup(void)
{
	if (cm_mgr_tracepoints.registered) {
		tracepoint_probe_unregister(cm_mgr_tracepoints.tp,
						cm_mgr_tracepoints.func,
						NULL);
		cm_mgr_tracepoints.registered = false;
	}
}

void cm_mgr_process(struct work_struct *work)
{
	if (cmmgr_data.hk.cm_mgr_get_perfs)
		icc_set_bw(cmmgr_data.cm_mgr_bw_path, 0,
			   cmmgr_data.hk.cm_mgr_get_perfs(cmmgr_data.cm_mgr_cpu_to_dram_opp));
}
EXPORT_SYMBOL_GPL(cm_mgr_process);

int cm_mgr_to_sspm_command_common(unsigned int cmd, unsigned int val)
{
	dev_err(cmmgr_data.dev, "send cmd to sspm fail\n");
	return -ENODEV;
}
EXPORT_SYMBOL_GPL(cm_mgr_to_sspm_command_common);

int cm_mgr_common_init(struct platform_device *pdev)
{
	int ret;

	dev_dbg(&pdev->dev, "CM COMMON INIT\n");

	cmmgr_data.dev = &pdev->dev;

#if IS_REACHABLE(CONFIG_MTK_CM_IPI)
	dev_dbg(&pdev->dev,"CM_IPI_INIT CALL\n");
	ret = cm_ipi_init(cmmgr_data.dev);
	if (ret)
		return ret;
#endif

	dev_dbg(&pdev->dev,"CM KERNEL TRACEPOINT\n");
	for_each_kernel_tracepoint(lookup_tracepoints, NULL);

	if (cm_mgr_tracepoints.tp == NULL) {
		dev_err_probe(&pdev->dev, -ENODEV, "%s not found.\n", cm_mgr_tracepoints.name);
		tracepoint_cleanup();
		return -ENODEV;
	}

	ret = tracepoint_probe_register(cm_mgr_tracepoints.tp,
					cm_mgr_tracepoints.func, NULL);
	if (ret) {
		dev_err_probe(&pdev->dev, ret, "fail to activate tracepoint.\n");
		goto fail_reg_cpu_frequency_entry;
	}
	cm_mgr_tracepoints.registered = true;

fail_reg_cpu_frequency_entry:

	if (cmmgr_data_cp.cm_mgr_arch == CM_MGR_ARCH_V1) {
		cmmgr_data.hk.cm_mgr_to_sspm_command(IPI_CM_MGR_ENABLE, cmmgr_data_cp.cm_mgr_enable);

		cmmgr_data.hk.cm_mgr_to_sspm_command(IPI_CM_MGR_EMI_DEMAND_CHECK,
				       cmmgr_data.cm_mgr_emi_demand_check);

		cmmgr_data.hk.cm_mgr_to_sspm_command(IPI_CM_MGR_LOADING_LEVEL,
				       cmmgr_data.cm_mgr_loading_level);

		cmmgr_data.hk.cm_mgr_to_sspm_command(IPI_CM_MGR_LOADING_ENABLE,
				       cmmgr_data.cm_mgr_loading_enable);

		cmmgr_data.hk.cm_mgr_to_sspm_command(IPI_CM_MGR_DEBOUNCE_TIMES_RESET_ADB,
				       cmmgr_data.debounce_times_reset_adb);

		if (cmmgr_data_cp.cm_mgr_use_bcpu_weight) {
			cmmgr_data.hk.cm_mgr_to_sspm_command(IPI_CM_MGR_BCPU_WEIGHT_MAX_SET,
					       cmmgr_data_cp.cpu_power_bcpu_weight_max);

			cmmgr_data.hk.cm_mgr_to_sspm_command(IPI_CM_MGR_BCPU_WEIGHT_MIN_SET,
					       cmmgr_data_cp.cpu_power_bcpu_weight_min);

			cmmgr_data.hk.cm_mgr_to_sspm_command(IPI_CM_MGR_BBCPU_WEIGHT_MAX_SET,
					       cmmgr_data_cp.cpu_power_bbcpu_weight_max);

			cmmgr_data.hk.cm_mgr_to_sspm_command(IPI_CM_MGR_BBCPU_WEIGHT_MIN_SET,
					       cmmgr_data_cp.cpu_power_bbcpu_weight_min);
		}
	}

	INIT_DELAYED_WORK(&cmmgr_data.cm_mgr_work, cm_mgr_process);
	cmmgr_data.cm_work_flag = 1;

	return 0;
}
EXPORT_SYMBOL_GPL(cm_mgr_common_init);
MODULE_LICENSE("GPL");
