// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 MediaTek Inc.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#if IS_ENABLED(CONFIG_MTK_VCP_RPROC)
#include <linux/remoteproc/mtk_vcp_public.h>
#include <linux/remoteproc.h>
#endif
#include <linux/workqueue.h>
#include <soc/mediatek/mmdebug.h>
#include <soc/mediatek/smi.h>

#include "mtk-mmdebug-vcp.h"

const char *kernel_warn_type_str[] = {
	"MMDEBUG-DPSW_CHECK_VMM_OFF",
	"MMDEBUG-DPSW_CHECK_VLOGIC",
	"MMDEBUG-DPSW_TIMEOUT",
	"MMDEBUG-HFLV_CHECK_CLOCK",
	"MMDEBUG-HFLV_CHECK_SINGLE_CLOCK",
	"MMDEBUG-HFLV_CHECK_VOLTAGE",
	"MMDEBUG-HFLV_CHECK_VOLTAGE_BIN",
	"MMDEBUG-POLL_RC_TIMEOUT",
	"MMDEBUG-RPOFILE_TIMEOUT",
};


#define REQUEST_MAX_RETRY_COUNT 10000
#define REQUEST_VCP_MAX_RETRY_COUNT 100

bool mmdebug_is_init_done(void)
{
	struct mmdebug_dev *mmdebug = get_mmdebug_dev();
	if (!mmdebug)
		return true;

	return mmdebug->mmdebug_init_done;
}
EXPORT_SYMBOL_GPL(mmdebug_is_init_done);

int mtk_mmdebug_status_dump_register_notifier(struct notifier_block *nb)
{
	struct mmdebug_dev *mmdebug = get_mmdebug_dev();
	if (!mmdebug)
		return false;

	return raw_notifier_chain_register(&mmdebug->mmdebug_status_dump_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(mtk_mmdebug_status_dump_register_notifier);

static void mmdebug_work_impl(struct work_struct *work)
{
	struct mmdebug_dev *mmdebug = get_mmdebug_dev();
	if (!mmdebug)
		return;

	raw_notifier_call_chain(&mmdebug->mmdebug_status_dump_notifier_list, 0, NULL);
}

static int mmdebug_vcp_ipi_cb(unsigned int ipi_id, void *prdata, void *data, unsigned int len)
{
	struct mmdebug_ipi_data slot;
	struct mmdebug_dev *mmdebug = prdata;
	struct device *dev = &mmdebug->pdev->dev;

	if (ipi_id != IPI_IN_MMDEBUG || !data)
		return 0;

	slot = *(struct mmdebug_ipi_data *)data;

	switch (slot.func) {
	case MMDEBUG_FUNC_SMI_DUMP:
		break;
	case MMDEBUG_FUNC_KERNEL_WARN:
		if (slot.idx >= ARRAY_SIZE(kernel_warn_type_str)) {
			MMDEBUG_ERR(dev, "MMDEBUG invalid idx:%hhu ack:%hhu base:%hhu",
				    slot.idx, slot.ack, slot.base);
			break;
		}

		MMDEBUG_ERR(dev, "MMDEBUG str:%s idx:%hhu ack:%hhu base:%hhu",
			    kernel_warn_type_str[slot.idx], slot.idx, slot.ack, slot.base);

		if (!work_pending(&mmdebug->mmdebug_work))
			queue_work(mmdebug->mmdebug_workq, &mmdebug->mmdebug_work);
		else
			MMDEBUG_ERR(dev, "queue work fail");
		break;
	default:
		MMDEBUG_ERR(dev, "ipi_id:%u func:%hhu idx:%hhu ack:%hhu base:%hhu",
			    ipi_id, slot.func, slot.idx, slot.ack, slot.base);
		break;
	}

	return 0;
}

static int mmdebug_vcp_init_thread(void *data)
{
	struct mmdebug_dev *mmdebug = data;
	struct platform_device *pdev = mmdebug->pdev;
	int retry = 0, ret = 0;
	phandle  vcp_phandle;

	mmdebug->vcp_device = NULL;
	while (mmdebug->vcp_device == NULL) {
		if (++retry > REQUEST_MAX_RETRY_COUNT) {
			MMDEBUG_ERR(&pdev->dev, "failed to load mtk-vcp module");
			return -ENODEV;
		}

		if (of_property_read_u32(pdev->dev.of_node, "mediatek,vcp", &vcp_phandle))
			continue;

		mmdebug->vcp_device = mtk_vcp_get_by_phandle(vcp_phandle);
		ssleep(1);
	}

	retry = 0;
	while (!mmdebug->vcp_device->data->vcp_is_ready(MMDEBUG_FEATURE_ID)) {
		if (++retry > REQUEST_VCP_MAX_RETRY_COUNT) {
			MMDEBUG_ERR(&pdev->dev, "mmup is not ready yet");
			return -ETIMEDOUT;
		}
		ssleep(1);
	}

	dev_err(&pdev->dev, "MMDEBUG: vcp and mmup are ready!\n");
	mmdebug->vcp_device->data->vcp_register_feature(mmdebug->vcp_device, MMDEBUG_FEATURE_ID);

	ret = mmdebug->vcp_device->ipi_ops->ipi_register(mmdebug->vcp_device->ipi_dev,
							 IPI_IN_MMDEBUG,
							 mmdebug_vcp_ipi_cb, mmdebug,
							 &mmdebug->mmdebug_vcp_ipi_data);
	if (ret) {
		MMDEBUG_ERR(&pdev->dev, "ipi register failed:%d ipi_id:%d", ret, IPI_IN_MMDEBUG);
		return ret;
	}

	mmdebug->vcp_device->data->vcp_deregister_feature(mmdebug->vcp_device, MMDEBUG_FEATURE_ID);

	mmdebug->mmdebug_init_done = true;

	MMDEBUG_DBG(&pdev->dev, "mmdebug_init_done:%d", mmdebug->mmdebug_init_done);

	return 0;
}

static int mmdebug_vcp_probe(struct platform_device *pdev)
{
	struct task_struct *kthr_vcp;
	struct mmdebug_dev *mmdebug;

	mmdebug = devm_kzalloc(&pdev->dev, sizeof(struct mmdebug_dev), GFP_KERNEL);
	if (!mmdebug)
		return -ENOMEM;

	mmdebug->mmdebug_ena = true;

	mmdebug->mmdebug_workq = create_singlethread_workqueue("mmdebug_workq");
	INIT_WORK(&mmdebug->mmdebug_work, mmdebug_work_impl);

	RAW_INIT_NOTIFIER_HEAD(&mmdebug->mmdebug_status_dump_notifier_list);

	mmdebug->pdev = pdev;
	platform_set_drvdata(pdev, mmdebug);

	kthr_vcp = kthread_run(mmdebug_vcp_init_thread, mmdebug, "mmdebug-vcp");
	if (IS_ERR(kthr_vcp))
		MMDEBUG_ERR(&pdev->dev, "create kthread mmdebug_vcp_init_thread failed");

	MMDEBUG_DBG(&pdev->dev, "probe success");
	return 0;
}

static const struct of_device_id of_mmdebug_vcp_match_tbl[] = {
	{
		.compatible = "mediatek,mmdebug-vcp",
	},
	{}
};

static struct platform_driver mmdebug_vcp_drv = {
	.probe = mmdebug_vcp_probe,
	.driver = {
		.name = "mtk-mmdebug-vcp",
		.of_match_table = of_mmdebug_vcp_match_tbl,
	},
};

module_platform_driver(mmdebug_vcp_drv)
MODULE_DESCRIPTION("MMDEBUG vcp Driver");
MODULE_LICENSE("GPL");
