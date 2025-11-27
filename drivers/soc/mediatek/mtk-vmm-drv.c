// SPDX-License-Identifier: (GPL-2.0 OR MIT)
/*
 * Copyright (c) 2025 MediaTek Inc.
 * Author: Yunfei Dong <yunfei.dong@mediatek.com>
 */

#include "mtk-vmm-drv.h"

#define MTK_VMM_ID_VDEC_VCORE	0

#define DELAY_US      1
#define POLL_TIMEOUT  10000

#define MTK_VMM_REGULAR(match, _name)              \
[MTK_VMM_ID_##_name] = {                           \
	.desc = {                                  \
		.name = match,                     \
		.of_match = of_match_ptr(match),   \
		.ops = &mtk_vmm_regulator_ops,     \
		.type = REGULATOR_VOLTAGE,         \
		.id = MTK_VMM_ID_##_name,          \
		.owner = THIS_MODULE,              \
	},	\
}

/*
 * struct mtk_vmm_regulator - mtk vmm regulators' information
 *
 * @desc: standard fields of regulator description.
 */
struct mtk_vmm_regulator {
	struct regulator_desc	desc;
};

/*
 * struct mtk_vmm_regulator_init_data - mtk vmm regulators' init data
 *
 * @size: num of regulators
 * @regulator_info: regulator info.
 */
struct mtk_vmm_regulator_init_data {
	u32 size;
	struct mtk_vmm_regulator *regulator_info;
};

static void mtk_vmm_debug_dump(struct mtk_vmm_drv *vmm_drv, int line)
{
	u32 value0, value1, value2, value3, value4, value5, value6, value7;

	regmap_read(vmm_drv->hwccf_regmap, HW_CCF_XPU0_BACKUP2_SET, &value0);
	regmap_read(vmm_drv->hwccf_regmap, HW_CCF_XPU0_BACKUP2_CLR, &value1);
	regmap_read(vmm_drv->hwccf_regmap, HW_CCF_BACKUP2_ENABLE, &value2);
	regmap_read(vmm_drv->hwccf_regmap, HW_CCF_BACKUP2_STATUS, &value3);
	regmap_read(vmm_drv->hwccf_regmap, HW_CCF_BACKUP2_STATUS_DBG, &value4);
	regmap_read(vmm_drv->hwccf_regmap, HW_CCF_BACKUP2_DONE, &value5);
	regmap_read(vmm_drv->hwccf_regmap, HW_CCF_BACKUP2_SET_STATUS, &value6);
	regmap_read(vmm_drv->hwccf_regmap, HW_CCF_BACKUP2_CLR_STATUS, &value7);

	mtk_vmm_err(vmm_drv->dev, "%d: (0x%x, 0x%x), (0x%x) (0x%x 0x%x) (0x%x), (0x%x 0x%x)\n",
		    line, value0, value1, value2, value3, value4, value5, value6, value7);
}

static int mtk_vmm_hwccf_ctrl(struct mtk_vmm_drv *vmm_drv, bool enable, unsigned int vote_bit)
{
	u32 ctrl_reg = enable ? HW_CCF_XPU0_BACKUP2_SET : HW_CCF_XPU0_BACKUP2_CLR;
	u32 hwccf_ctrl_status = enable ? HW_CCF_BACKUP2_SET_STATUS : HW_CCF_BACKUP2_CLR_STATUS;
	u32 hwccf_done = HW_CCF_BACKUP2_DONE;
	unsigned int bit_val = BIT(vote_bit);
	int tmp, ret = 0;

	if (IS_ERR_OR_NULL(vmm_drv->hwccf_regmap)) {
		mtk_vmm_err(vmm_drv->dev, "hwccf_reg_base is null, need to map first.");
		return -ENODEV;
	}

	if (!vmm_drv->vcp_is_active) {
		vmm_drv->need_update = true;
		goto ctrl_end;
	}

	/* polling hwccf write data done */
	ret = regmap_read_poll_timeout(vmm_drv->hwccf_regmap, hwccf_done, tmp,
				       ((tmp & bit_val) == bit_val), DELAY_US, POLL_TIMEOUT);
	if (ret < 0) {
		mtk_vmm_debug_dump(vmm_drv, __LINE__);
		goto ctrl_end;
	}

	/* set and clr data */
	ret = regmap_write(vmm_drv->hwccf_regmap, ctrl_reg, bit_val);
	if (ret) {
		mtk_vmm_debug_dump(vmm_drv, __LINE__);
		goto ctrl_end;
	}

	/* polling hwccf write data done */
	ret = regmap_read_poll_timeout(vmm_drv->hwccf_regmap, hwccf_done, tmp,
				       ((tmp & bit_val) == bit_val), DELAY_US, POLL_TIMEOUT);
	if (ret < 0) {
		mtk_vmm_debug_dump(vmm_drv, __LINE__);
		goto ctrl_end;
	}

	/* wait for current done */
	ret = regmap_read_poll_timeout(vmm_drv->hwccf_regmap, hwccf_ctrl_status, tmp,
				       !(tmp & bit_val), DELAY_US, POLL_TIMEOUT);
	if (ret < 0) {
		mtk_vmm_debug_dump(vmm_drv, __LINE__);
		goto ctrl_end;
	}

ctrl_end:

	return ret;
}

static int mtk_vmm_vdec_is_enabled(struct regulator_dev *rdev)
{
	struct mtk_vmm_drv *vmm_drv = rdev_get_drvdata(rdev);
	int ret;
	u32 val;

	ret = regmap_read(vmm_drv->hwccf_regmap, HW_CCF_BACKUP2_ENABLE, &val);
	if (ret < 0) {
		mtk_vmm_err(vmm_drv->dev, "vmm read buck status failed\n");
		return ret;
	}

	return !!(val & BIT(HW_CCF_AP_VOTER_BIT));
}

static int mtk_vmm_vdec_enable(struct regulator_dev *rdev)
{
	struct mtk_vmm_drv *vmm_drv = rdev_get_drvdata(rdev);
	int ret;

	mtk_vmm_dbg(vmm_drv->dev, "enable vmm driver %d", vmm_drv->vcp_is_active);

	ret = mtk_vmm_hwccf_ctrl(vmm_drv, true, HW_CCF_AP_VOTER_BIT);
	if (ret < 0)
		mtk_vmm_err(vmm_drv->dev, "failed to enable hwccf, ret=%d\n", ret);

	return ret;
}

static int mtk_vmm_vdec_disable(struct regulator_dev *rdev)
{
	struct mtk_vmm_drv *vmm_drv = rdev_get_drvdata(rdev);
	int ret = 0;

	mtk_vmm_dbg(vmm_drv->dev, "disable vmm driver %d", vmm_drv->vcp_is_active);

	ret = mtk_vmm_hwccf_ctrl(vmm_drv, false, HW_CCF_AP_VOTER_BIT);
	if (ret < 0)
		mtk_vmm_err(vmm_drv->dev, "failed to disable hwccf, ret=%d\n", ret);

	return ret;
}

#if IS_ENABLED(CONFIG_MTK_VCP_RPROC)
static int vmm_vcp_notifier(struct notifier_block *nb, unsigned long vcp_event, void *unused)
{
	struct mtk_vmm_drv *vmm_drv = container_of(nb, struct mtk_vmm_drv, vcp_nb);
	int ret = NOTIFY_DONE;

	switch (vcp_event) {
	case VCP_EVENT_SUSPEND:
	case VCP_EVENT_STOP:
		mtk_vmm_dbg(vmm_drv->dev, "vcp notifier suspend");
		break;
	case VCP_EVENT_READY:
	case VCP_EVENT_RESUME:
		if (vmm_drv->need_update) {
			ret = mtk_vmm_hwccf_ctrl(vmm_drv, true, HW_CCF_AP_VOTER_BIT);
			if (ret < 0)
				mtk_vmm_err(vmm_drv->dev, "failed to enalbe hwccf %d\n", ret);

			ret = mtk_vmm_hwccf_ctrl(vmm_drv, false, HW_CCF_AP_VOTER_BIT);
			if (ret < 0)
				mtk_vmm_err(vmm_drv->dev, "failed to disable hwccf %d\n", ret);

			vmm_drv->need_update = false;
		}

		mtk_vmm_dbg(vmm_drv->dev, "vcp notifier ready");
		break;
	}

	return ret;
}



static int mtk_vmm_vcp_init_thread(void *arg)
{
	struct mtk_vmm_drv *vmm_drv = arg;
	struct device *dev = vmm_drv->dev;
	struct device_link *dev_link;
	int retry = 0, retry_cnt = 10000;

	vmm_drv->vcp_device = NULL;
	while (vmm_drv->vcp_device == NULL) {
		if (++retry > retry_cnt) {
			mtk_vmm_err(vmm_drv->dev, "failed to load mtk-vcp module");
			return -ENODEV;
		}

		if (of_property_read_u32(dev->of_node, "mediatek,vcp", &vmm_drv->vcp_phandle))
			continue;

		vmm_drv->vcp_device = mtk_vcp_get_by_phandle(vmm_drv->vcp_phandle);

		ssleep(1);
	}

	dev_link = device_link_add(vmm_drv->dev, vmm_drv->vcp_device->dev, 0);
	if (!dev_link) {
		mtk_vmm_err(vmm_drv->dev, "device link is NULL\n");
		return -EINVAL;
	}

	retry = 0;
	retry_cnt = 10000;
	while (!vmm_drv->vcp_device->data->vcp_is_ready(MMDVFS_MMUP_FEATURE_ID) ||
	       !vmm_drv->vcp_device->data->vcp_is_ready(MMDVFS_VCP_FEATURE_ID)) {
		if (++retry > retry_cnt) {
			mtk_vmm_err(vmm_drv->dev, "vcp and mmup is not ready yet.");
			return -ETIMEDOUT;
		}
		ssleep(1);
	}

	vmm_drv->vcp_is_active = true;
	vmm_drv->vcp_device->data->vcp_register_feature(vmm_drv->vcp_device, VMM_FEATURE_ID);

	vmm_drv->vcp_nb.notifier_call = vmm_vcp_notifier;
	vmm_drv->vcp_device->data->vcp_register_notify(VMM_FEATURE_ID, &vmm_drv->vcp_nb);

	return 0;
}
#endif

static const struct regulator_ops mtk_vmm_regulator_ops = {
	.enable = mtk_vmm_vdec_enable,
	.disable = mtk_vmm_vdec_disable,
	.is_enabled = mtk_vmm_vdec_is_enabled,
};

static struct mtk_vmm_regulator mt8196_vmm_regulators[] = {
	MTK_VMM_REGULAR("vdec-vcore", VDEC_VCORE),
};

static const struct mtk_vmm_regulator_init_data mt8196_regulator_data = {
	.size = ARRAY_SIZE(mt8196_vmm_regulators),
	.regulator_info = mt8196_vmm_regulators,
};

static const struct of_device_id of_vmm_match_tbl[] = {
	{
		.compatible = "mediatek,mt8196-vmm",
		.data = &mt8196_regulator_data,
	},
	{},
};
MODULE_DEVICE_TABLE(of, of_vmm_match_tbl);

static int mtk_vmm_probe(struct platform_device *pdev)
{
	struct regulator_config config = { };
	struct regulator_dev *rdev;
	const struct mtk_vmm_regulator_init_data *regulator_init_data;
	struct mtk_vmm_regulator *mtk_regulators;
	struct mtk_vmm_drv *vmm_drv;
	int i;

	vmm_drv = devm_kzalloc(&pdev->dev, sizeof(*vmm_drv), GFP_KERNEL);
	if (!vmm_drv)
		return -ENOMEM;

	vmm_drv->dev = &pdev->dev;
	platform_set_drvdata(pdev, vmm_drv);

	vmm_drv->hwccf_regmap =
		syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "mediatek,hw-ccf");
	if (IS_ERR(vmm_drv->hwccf_regmap))
		return dev_err_probe(&pdev->dev, PTR_ERR(vmm_drv->hwccf_regmap),
				     "cannot find hwccf controller.\n");

	regulator_init_data = of_device_get_match_data(&pdev->dev);

	mtk_regulators = regulator_init_data->regulator_info;
	for (i = 0; i < regulator_init_data->size; i++) {
		config.dev = &pdev->dev;
		config.driver_data = vmm_drv;
		rdev = devm_regulator_register(&pdev->dev, &(mtk_regulators + i)->desc, &config);
		if (IS_ERR(rdev)) {
			dev_err(&pdev->dev, "failed to register %d\n", i);
			return PTR_ERR(rdev);
		}
	}

#if IS_ENABLED(CONFIG_MTK_VCP_RPROC)
	vmm_drv->kthr_vcp = kthread_run(mtk_vmm_vcp_init_thread, vmm_drv, "vmm-vcp");
	if (IS_ERR(vmm_drv->kthr_vcp))
		dev_err(&pdev->dev, "create kthread failed");
#endif

	mtk_vmm_dbg(&pdev->dev, "vmm prob done.");
	return 0;
}

static int mtk_vmm_remove(struct platform_device *pdev)
{
#if IS_ENABLED(CONFIG_MTK_VCP_RPROC)
	struct mtk_vmm_drv *vmm_drv = dev_get_drvdata(&pdev->dev);
	struct mtk_vcp_device *vcp_device = vmm_drv->vcp_device;

	vcp_device->data->vcp_deregister_feature(vcp_device, VMM_FEATURE_ID);
	device_link_remove(vmm_drv->dev, vcp_device->dev);
#endif

	return 0;
}

static struct platform_driver mtk_vmm_plat_drv = {
	.probe = mtk_vmm_probe,
	.remove = mtk_vmm_remove,
	.driver = {
		.name = "mtk-vmm-drv",
		.of_match_table = of_vmm_match_tbl,
	},
};
module_platform_driver(mtk_vmm_plat_drv);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Mediatek vmm driver");
