// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 MediaTek Inc.
 * Author: Echo.Zhang <echo.zhang@mediatek.com>
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iommu.h>
#if IS_ENABLED(CONFIG_MTK_VCP_RPROC)
#include <linux/remoteproc.h>
#include <linux/remoteproc/mtk_vcp_public.h>
#endif
#include <linux/rpmsg.h>
#include <linux/rpmsg/mtk_rpmsg.h>

#include "mmqos-vcp.h"

static DEFINE_MUTEX(mmqos_vcp_ipi_mutex);

#define MAX_RETRY_COUNT            (100)
#define MAX_RETRY_COUNT_WAIT_VCP   (1000)

static int vcp_mmqos_log;
static int vcp_smi_log;

void *mmqos_get_vcp_base(phys_addr_t *pa)
{
	struct mtk_mmqos *mmqos = NULL;

	mmqos = mtk_mmqos_get_drv_data();
	if (IS_ERR_OR_NULL(mmqos)) {
		mmqos_err(mmqos->dev, "not ready");
		*pa = (u64)NULL;
	}

	if (pa)
		*pa = mmqos->mmqos_memory_pa;

	return mmqos->mmqos_memory_va;
}
EXPORT_SYMBOL_GPL(mmqos_get_vcp_base);

bool mmqos_is_init_done(void)
{
	struct mtk_mmqos *mmqos = NULL;
	u32 mmqos_state;

	mmqos = mtk_mmqos_get_drv_data();
	if (IS_ERR_OR_NULL(mmqos)) {
		mmqos_err(mmqos->dev, "not ready");
		return false;
	}
	mmqos_state = mmqos->mmqos_state;

	return (mmqos_state != MMQOS_DISABLE) ? mmqos->mmqos_vcp_init_done : false;
}
EXPORT_SYMBOL_GPL(mmqos_is_init_done);

int mmqos_vcp_ipi_send(enum ipi_func_id func, const u8 idx, u32 *data)
{
	struct mmqos_ipi_data slot = {func, idx, 0, 0};
	struct mtk_ipi_device *vcp_ipi_dev;
	struct mtk_vcp_device  *vcp_device;
	struct mtk_mmqos *mmqos = NULL;
	int ret = 0, retry = 0;
	u32 mmqos_state;
	u32 val;

	mmqos = mtk_mmqos_get_drv_data();
	if (IS_ERR_OR_NULL(mmqos)) {
		mmqos_err(mmqos->dev, "not ready");
		return -ENODEV;
	}

	if (!mmqos_is_init_done())
		return -ENODEV;

	slot.ack = (mmqos->mmqos_memory_iova) >> 32;
	slot.base = mmqos->mmqos_memory_iova & GENMASK(31, 0);

	vcp_device = mmqos->vcp_device;
	mmqos_state = mmqos->mmqos_state;

	mutex_lock(&mmqos_vcp_ipi_mutex);
	writel(vcp_mmqos_log, MEM_LOG_FLAG);
	writel(vcp_smi_log, MEM_SMI_LOG_FLAG);
	writel(mmqos_state, MEM_MMQOS_STATE);

	switch (func) {
	case FUNC_MMQOS_INIT:
		/* trigger mmqos in vcp to create topology */
		break;
	case FUNC_TEST:
		writel(idx, MEM_TEST);
		break;
	case FUNC_SYNC_STATE:
		/* change mmqos_state by adb command, should trigger sync state */
		break;
	default:
		mmqos_err(mmqos->dev, "Unhandled enum value: %d\n", func);
		break;
	}

	val = readl(MEM_IPI_SYNC_FUNC);
	mutex_unlock(&mmqos_vcp_ipi_mutex);

	while (!vcp_device->data->vcp_is_ready(MMQOS_FEATURE_ID) ||
	       (!(mmqos->mmqos_vcp_cb_ready) && func != FUNC_MMQOS_INIT)) {
		if (++retry > VCP_SYNC_TIMEOUT_MS) {
			ret = -ETIMEDOUT;
			mmqos_err(mmqos->dev, "ret:%d retry:%d ready:%d cb_ready:%d",
				  ret, retry,
				  vcp_device->data->vcp_is_ready(MMQOS_FEATURE_ID),
				  mmqos->mmqos_vcp_cb_ready);
			goto ipi_send_end;
		}

		mdelay(1);
	}

	vcp_ipi_dev = vcp_device->ipi_dev;
	if (!vcp_ipi_dev)
		goto ipi_lock_end;

	mutex_lock(&mmqos_vcp_ipi_mutex);
	writel(0, MEM_IPI_SYNC_DATA);
	writel(val | (1 << func), MEM_IPI_SYNC_FUNC);

	ret = vcp_device->ipi_ops->ipi_send(vcp_ipi_dev, IPI_OUT_MMQOS, &slot,
					    PIN_OUT_SIZE_MMQOS, IPI_TIMEOUT_MS);
	if (ret != IPI_ACTION_DONE)
		goto ipi_lock_end;

	retry = 0;
	while (!(readl(MEM_IPI_SYNC_DATA) & (1 << func))) {
		if (++retry > VCP_SYNC_TIMEOUT_MS) {
			ret = IPI_COMPL_TIMEOUT;
			mmqos_err(mmqos->dev,
				  "ret:%d rty:%d cb_rdy:%d slot:%#llx power:%d func:%#x",
				  ret, retry, mmqos->mmqos_vcp_cb_ready, *(u64 *)&slot,
				  mmqos->vcp_power, val);
			break;
		}

		if (!vcp_device->data->vcp_is_ready(MMQOS_FEATURE_ID)) {
			ret = -ETIMEDOUT;
			mmqos_err(mmqos->dev,
				  "ret:%d rty:%d cb_rdy:%d slot:%#llx power:%d func:%#x",
				  ret, retry, mmqos->mmqos_vcp_cb_ready, *(u64 *)&slot,
				  mmqos->vcp_power, val);
			break;
		}

		mdelay(1);
	}

	if (!ret)
		writel(val & ~readl(MEM_IPI_SYNC_DATA), MEM_IPI_SYNC_FUNC);

ipi_lock_end:
	val = readl(MEM_IPI_SYNC_FUNC);
	mutex_unlock(&mmqos_vcp_ipi_mutex);

ipi_send_end:
	if (ret)
		mmqos_err(mmqos->dev, "ret:%d rty:%d cb_rdy:%d slot:%#llx power:%d func:%#x",
			  ret, retry, mmqos->mmqos_vcp_cb_ready, *(u64 *)&slot,
			  mmqos->vcp_power, val);
	else
		mmqos_dbg(mmqos->dev, LOG_IPI,
			  "ret:%d rty:%d cb_rdy:%d slot:%#llx power:%d func:%#x",
			  ret, retry, mmqos->mmqos_vcp_cb_ready,
			  *(u64 *)&slot, mmqos->vcp_power, val);

	mmqos->mmqos_ipi_status = ret;

	return ret;
}
EXPORT_SYMBOL_GPL(mmqos_vcp_ipi_send);

static int mmqos_vcp_notifier_callback(struct notifier_block *nb, unsigned long action, void *data)
{
	struct mtk_mmqos *mmqos = NULL;

	mmqos = mtk_mmqos_get_drv_data();
	if (IS_ERR_OR_NULL(mmqos)) {
		mmqos_err(mmqos->dev, "not ready");
		return 0;
	}

	switch (action) {
	case VCP_EVENT_READY:
		mmqos_dbg(mmqos->dev, LOG_BW,
			  "receive VCP_EVENT_READY IPI_SYNC_FUNC=%#x IPI_SYNC_DATA=%#x",
			  readl(MEM_IPI_SYNC_FUNC), readl(MEM_IPI_SYNC_DATA));
		mmqos_vcp_ipi_send(FUNC_MMQOS_INIT, 0, NULL);
		mmqos->mmqos_vcp_cb_ready = true;
		break;
	case VCP_EVENT_STOP:
	case VCP_EVENT_SUSPEND:
		mmqos->mmqos_vcp_cb_ready = false;
		break;
	}

	return NOTIFY_DONE;
}

int mmqos_vcp_init_thread(void *data)
{
	struct platform_device *pdev = data;
	struct mtk_mmqos *mmqos = NULL;
	phandle  vcp_phandle;
	u32 mmqos_state;
	int retry = 0;

	mmqos = mtk_mmqos_get_drv_data();
	if (IS_ERR_OR_NULL(mmqos)) {
		mmqos_err(mmqos->dev, "not ready");
		return -ENODEV;
	}

	mmqos_state = mmqos->mmqos_state;
	while (request_module("mtk-vcp")) {
		if (++retry > MAX_RETRY_COUNT_WAIT_VCP) {
			mmqos_err(mmqos->dev, "failed to load mtk-vcp module");
			return -ENODEV;
		}

		ssleep(1);
	}

	if (of_property_read_u32(pdev->dev.of_node, "mediatek,vcp", &vcp_phandle)) {
		mmqos_err(mmqos->dev, "can't get vcp handle.\n");
		return -ENODEV;
	}

	mmqos->vcp_device = mtk_vcp_get_by_phandle(vcp_phandle);
	if (!mmqos->vcp_device) {
		mmqos_err(mmqos->dev, "get vcp device failed\n");
		return -ENODEV;
	}

	retry = 0;
	while (!mmqos->vcp_device->data->vcp_is_ready(MMQOS_FEATURE_ID)) {
		if (++retry > VCP_SYNC_TIMEOUT_MS) {
			mmqos_err(mmqos->dev, "VCP not ready");
			return -ETIMEDOUT;
		}
		mdelay(1);
	}

	mmqos->vcp_device->data->vcp_register_feature(mmqos->vcp_device, MMQOS_FEATURE_ID);

	mmqos->mmqos_memory_iova = mmqos->vcp_device->data->vcp_get_mem_iova(MMQOS_MEM_ID);
	mmqos->mmqos_memory_va = (void *)mmqos->vcp_device->data->vcp_get_mem_virt(MMQOS_MEM_ID);

	writel_relaxed(mmqos_state, MEM_MMQOS_STATE);

	mmqos->mmqos_vcp_init_done = true;

	mmqos_dbg(mmqos->dev, LOG_BW, "shared memory iova:%pa pa:%pa va:%#lx init_done:%d",
		  &mmqos->mmqos_memory_iova, &mmqos->mmqos_memory_pa,
		  (unsigned long)mmqos->mmqos_memory_va, mmqos->mmqos_vcp_init_done);

	mmqos->vcp_ready_notifier.notifier_call = mmqos_vcp_notifier_callback;
	mmqos->vcp_device->data->vcp_register_notify(MMQOS_FEATURE_ID, &mmqos->vcp_ready_notifier);

	mmqos->vcp_device->data->vcp_deregister_feature(mmqos->vcp_device, MMQOS_FEATURE_ID);

	return 0;
}
EXPORT_SYMBOL_GPL(mmqos_vcp_init_thread);

int mmqos_set_state_to_vcp(void)
{
	struct mtk_mmqos *mmqos = mtk_mmqos_get_drv_data();
	struct mtk_vcp_device  *vcp_device;
	int ret;

	vcp_device = mmqos->vcp_device;
	if (mmqos->mmqos_state & VCP_ENABLE) {
		ret = vcp_device->data->vcp_register_feature(vcp_device, MMQOS_FEATURE_ID);
		if (ret)
			goto err;
		ret = mmqos_vcp_ipi_send(FUNC_SYNC_STATE, mmqos->mmqos_state, NULL);
		ret = vcp_device->data->vcp_deregister_feature(vcp_device, MMQOS_FEATURE_ID);
		if (ret)
			goto err;
	}
err:
	return 0;
}

static int mmqos_get_vcp_mmqos_log(char *buf, const struct kernel_param *kp)
{
	int len = 0, ret;

	if (!mmqos_is_init_done())
		return 0;

	ret = readl(MEM_LOG_FLAG);
	len += snprintf(buf + len, PAGE_SIZE - len, "MEM_LOG_FLAG:%#x", ret);

	return len;
}

static int mmqos_set_vcp_mmqos_log(const char *val, const struct kernel_param *kp)
{
	struct mtk_mmqos *mmqos = mtk_mmqos_get_drv_data();
	struct mtk_vcp_device  *vcp_device;
	u32 mmqos_state;
	u32 log = 0;
	int ret;

	if (IS_ERR_OR_NULL(mmqos)) {
		mmqos_dbg(mmqos->dev, LOG_BW, "not ready");
		return 0;
	}

	if (!mmqos_is_init_done())
		return 0;

	ret = kstrtou32(val, 0, &log);
	if (ret) {
		mmqos_err(mmqos->dev, "failed:%d log:%#x", ret, log);
		return ret;
	}

	vcp_device = mmqos->vcp_device;
	mmqos_state = mmqos->mmqos_state;
	vcp_mmqos_log = log;
	ret = vcp_device->data->vcp_register_feature(vcp_device, MMQOS_FEATURE_ID);
	if (ret)
		goto err;
	ret = mmqos_vcp_ipi_send(FUNC_SYNC_STATE, mmqos_state, NULL);
	ret = vcp_device->data->vcp_deregister_feature(vcp_device, MMQOS_FEATURE_ID);
	if (ret)
		goto err;
err:
	return 0;
}

static const struct kernel_param_ops mmqos_set_vcp_mmqos_log_ops = {
	.get = mmqos_get_vcp_mmqos_log,
	.set = mmqos_set_vcp_mmqos_log,
};

module_param_cb(vcp_mmqos_log, &mmqos_set_vcp_mmqos_log_ops, NULL, 0644);
MODULE_PARM_DESC(vcp_mmqos_log, "mmqos vcp log");

static int mmqos_get_vcp_smi_log(char *buf, const struct kernel_param *kp)
{
	int len = 0, ret;

	if (!mmqos_is_init_done())
		return 0;

	ret = readl(MEM_SMI_LOG_FLAG);
	len += snprintf(buf + len, PAGE_SIZE - len, "MEM_SMI_LOG_FLAG:%#x", ret);

	return len;
}

static int mmqos_set_vcp_smi_log(const char *val, const struct kernel_param *kp)
{
	struct mtk_mmqos *mmqos = mtk_mmqos_get_drv_data();
	struct mtk_vcp_device  *vcp_device;
	u32 mmqos_state;
	u32 log = 0;
	int ret;

	if (IS_ERR_OR_NULL(mmqos)) {
		mmqos_dbg(mmqos->dev, LOG_BW, "not ready");
		return 0;
	}

	if (!mmqos_is_init_done())
		return 0;

	ret = kstrtou32(val, 0, &log);
	if (ret) {
		mmqos_err(mmqos->dev, "failed:%d log:%#x", ret, log);
		return ret;
	}

	vcp_device = mmqos->vcp_device;
	mmqos_state = mmqos->mmqos_state;
	vcp_smi_log = log;
	ret = vcp_device->data->vcp_register_feature(vcp_device, MMQOS_FEATURE_ID);
	if (ret)
		goto err;
	ret = mmqos_vcp_ipi_send(FUNC_SYNC_STATE, mmqos_state, NULL);
	ret = vcp_device->data->vcp_deregister_feature(vcp_device, MMQOS_FEATURE_ID);
	if (ret)
		goto err;
err:
	return 0;
}

static const struct kernel_param_ops mmqos_set_vcp_smi_log_ops = {
	.get = mmqos_get_vcp_smi_log,
	.set = mmqos_set_vcp_smi_log,
};
module_param_cb(vcp_smi_log, &mmqos_set_vcp_smi_log_ops, NULL, 0644);
MODULE_PARM_DESC(vcp_smi_log, "smi vcp log");

MODULE_LICENSE("GPL");
