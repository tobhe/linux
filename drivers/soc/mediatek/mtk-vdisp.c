// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2023 MediaTek Inc.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#if IS_ENABLED(CONFIG_MTK_VCP_RPROC)
#include <linux/remoteproc/mtk_vcp_public.h>
#endif

#define RETRY_COUNT_MAX			10000
#define VOTE_DELAY_US			2
#define HW_CCF_AP_VOTER_BIT		0
#define HW_CCF_XPU0_BACKUP1_SET		0x230
#define HW_CCF_XPU0_BACKUP1_CLR		0x234
#define HW_CCF_BACKUP1_ENABLE		0x1430
#define HW_CCF_BACKUP1_DONE		0x143c
#define HW_CCF_BACKUP1_SET_STATUS	0x1484
#define HW_CCF_BACKUP1_CLR_STATUS	0x1488

struct mtk_vdisp_ctrl {
	struct device		*dev;
	struct regmap		*hwccf_regmap;
	/* lock for hwccf control */
	struct mutex		ctrl_lock;
#if IS_ENABLED(CONFIG_MTK_VCP_RPROC)
	phandle			vcp_phandle;
	struct mtk_vcp_device	*vcp_device;
	bool			vcp_ready;
#endif
};

static int vdisp_hwccf_ctrl(struct mtk_vdisp_ctrl *priv, bool enable)
{
	u32 val;
	int ret;
	u32 ctrl_reg = (enable) ? HW_CCF_XPU0_BACKUP1_SET : HW_CCF_XPU0_BACKUP1_CLR;
#if IS_ENABLED(CONFIG_MTK_VCP_RPROC)
	u32 hwccf_ctrl_status = (enable) ? HW_CCF_BACKUP1_SET_STATUS : HW_CCF_BACKUP1_CLR_STATUS;
#endif

	if (IS_ERR_OR_NULL(priv->hwccf_regmap))
		return -ENODEV;

	dev_dbg(priv->dev, "vdisp buck enable:%d", enable);

	mutex_lock(&priv->ctrl_lock);

	/* These settings still need to be voted before the vcp ready */
	ret = regmap_read_poll_timeout(priv->hwccf_regmap, HW_CCF_BACKUP1_DONE, val,
				       (val & BIT(HW_CCF_AP_VOTER_BIT)), VOTE_DELAY_US, 100000);
	if (ret) {
		dev_err(priv->dev, "vdisp hwccf backup1-1 done timeout");
		goto end;
	}
	ret = regmap_write(priv->hwccf_regmap, ctrl_reg, BIT(HW_CCF_AP_VOTER_BIT));
	if (ret) {
		dev_err(priv->dev, "vdisp hwccf voter fail");
		goto end;
	}

#if IS_ENABLED(CONFIG_MTK_VCP_RPROC)
	if (priv->vcp_ready) {
		ret = regmap_read_poll_timeout(priv->hwccf_regmap, HW_CCF_BACKUP1_DONE, val,
					(val & BIT(HW_CCF_AP_VOTER_BIT)), VOTE_DELAY_US, 100000);
		if (ret) {
			dev_err(priv->dev, "vdisp hwccf backup1-2 done timeout");
			goto end;
		}
		ret = regmap_read_poll_timeout(priv->hwccf_regmap, hwccf_ctrl_status, val,
					!(val & BIT(HW_CCF_AP_VOTER_BIT)), VOTE_DELAY_US, 100000);
		if (ret) {
			dev_err(priv->dev, "vdisp wait ctrl status cleared timeout");
			goto end;
		}
	}
#endif

end:
	mutex_unlock(&priv->ctrl_lock);

	return ret;
}

static int vdisp_enable(struct regulator_dev *rdev)
{
	struct mtk_vdisp_ctrl *priv = rdev_get_drvdata(rdev);
	int ret;

	ret = vdisp_hwccf_ctrl(priv, true);
	if (ret < 0)
		dev_err(priv->dev, "%s: failed to enable hwccf, ret=%d\n", __func__, ret);

	return ret;
}

static int vdisp_disable(struct regulator_dev *rdev)
{
	struct mtk_vdisp_ctrl *priv = rdev_get_drvdata(rdev);
	int ret;

	ret = vdisp_hwccf_ctrl(priv, false);
	if (ret < 0)
		dev_err(priv->dev, "%s: failed to disable hwccf, ret=%d\n", __func__, ret);

	return ret;
}

static int vdisp_is_enabled(struct regulator_dev *rdev)
{
	struct mtk_vdisp_ctrl *priv = rdev_get_drvdata(rdev);
	int ret;
	u32 val;

	mutex_lock(&priv->ctrl_lock);

	ret = regmap_read(priv->hwccf_regmap, HW_CCF_BACKUP1_ENABLE, &val);
	if (ret < 0) {
		dev_err(priv->dev, "%s: vdisp read buck status failed, ret=%d\n", __func__, ret);
		return ret;
	}
	dev_dbg(priv->dev, "%s: %lu\n", __func__, val & BIT(HW_CCF_AP_VOTER_BIT));

	mutex_unlock(&priv->ctrl_lock);

	return !!(val & BIT(HW_CCF_AP_VOTER_BIT));
}

static const struct regulator_ops vdisp_ops = {
	.is_enabled = vdisp_is_enabled,
	.enable = vdisp_enable,
	.disable = vdisp_disable,
};

static const struct regulator_desc vdisp_desc = {
	.name = "vdisp",
	.of_match = of_match_ptr("vdisp"),
	.ops = &vdisp_ops,
	.type = REGULATOR_VOLTAGE,
	.owner = THIS_MODULE,
};

static int vdisp_vcp_init_thread(void *data)
{
#if IS_ENABLED(CONFIG_MTK_VCP_RPROC)
	struct mtk_vdisp_ctrl *priv = data;
	struct device_link *dev_link;
	int retry = 0;

	priv->vcp_device = NULL;
	while (priv->vcp_device == NULL) {
		if (++retry > RETRY_COUNT_MAX) {
			dev_err(priv->dev, "failed to load mtk-vcp module");
			return -ENODEV;
		}

		if (of_property_read_u32(priv->dev->of_node, "mediatek,vcp", &priv->vcp_phandle))
			continue;

		priv->vcp_device = mtk_vcp_get_by_phandle(priv->vcp_phandle);

		ssleep(1);
	}

	dev_dbg(priv->dev, "get mtk-vcp device success!\n");

	/* vcp needs to be put into suspend state later than vdisp */
	dev_link = device_link_add(priv->dev, priv->vcp_device->dev, 0);
	if (!dev_link) {
		dev_err(priv->dev, "device link is NULL\n");
		return -EINVAL;
	}
	dev_dbg(priv->dev, "add device link success!\n");

	retry = 0;
	while (!priv->vcp_device->data->vcp_is_ready(MMDVFS_MMUP_FEATURE_ID) ||
	       !priv->vcp_device->data->vcp_is_ready(MMDVFS_VCP_FEATURE_ID)) {
		if (++retry > RETRY_COUNT_MAX) {
			dev_err(priv->dev, "Failed to wait vcp and mmup ready\n");
			return -ETIMEDOUT;
		}
		ssleep(1);
	}

	priv->vcp_ready = true;
	dev_info(priv->dev, "@@@@@@@@@@@@@@@@ vdisp: vcp and mmup are ready! @@@@@@@@@@@@@@@@\n");
	priv->vcp_device->data->vcp_register_feature(priv->vcp_device, VDISP_FEATURE_ID);
#endif
	return 0;
}

static int mtk_vdisp_ctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct mtk_vdisp_ctrl *priv;
	struct regmap *regmap;
	int ret;
	struct task_struct *kthr_vcp;
	struct regulator_dev *regulator;
	struct regulator_config config = { };

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;

	config.dev = dev;
	config.driver_data = priv;

	regmap = syscon_regmap_lookup_by_phandle(np, "mediatek,hw-ccf");
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);
	priv->hwccf_regmap = regmap;

	mutex_init(&priv->ctrl_lock);
	platform_set_drvdata(pdev, priv);

	 /*
	 * VDISP is the supplier of the Power Domain, so it cannot be registered
	 * for the regulator too late. However, due to limited flash storage,
	 * the VCP can only be loaded in the kernel, and it has to wait for UFS
	 * to be ready.
	 * Therefore, VDISP has to register the regulator first and open a
	 * vdisp_vcp_init_thread to wait for the VCP to be loaded.
	 *
	 * Registering VDISP as a regulator before the VCP is loaded is fine because
	 * the firmware leaves the VDISP power in an enabled state, and without
	 * the VCP, nothing can actually turn it off.
	 * Once the VCP is loaded, it must always trigger actions from the VCP,
	 * and return an error if that fails.
	 */
	regulator = devm_regulator_register(dev, &vdisp_desc, &config);
	if (IS_ERR(regulator)) {
		ret = PTR_ERR(regulator);
		dev_err(dev, "Failed to register vdisp regulator: %d\n", ret);
		return ret;
	}

	kthr_vcp = kthread_run(vdisp_vcp_init_thread, priv, "vdisp-vcp");
	if (IS_ERR(kthr_vcp))
		dev_err(dev, "create kthread vdisp-vcp init_thread failed");

	return 0;
}

static int mtk_vdisp_ctrl_remove(struct platform_device *pdev)
{
#if IS_ENABLED(CONFIG_MTK_VCP_RPROC)
	struct mtk_vdisp_ctrl *priv = dev_get_drvdata(&pdev->dev);

	priv->vcp_device->data->vcp_deregister_feature(priv->vcp_device, VDISP_FEATURE_ID);
	device_link_remove(priv->dev, priv->vcp_device->dev);
#endif
	return 0;
}

static const struct of_device_id mtk_vdisp_ctrl_dt_match[] = {
	{.compatible = "mediatek,mt8196-vdisp-ctrl"},
	{},
};
MODULE_DEVICE_TABLE(of, mtk_vdisp_ctrl_dt_match);

struct platform_driver mtk_vdisp_ctrl_driver = {
	.probe = mtk_vdisp_ctrl_probe,
	.remove = mtk_vdisp_ctrl_remove,
	.driver = {
		.name = "mediatek-vdisp-ctrl",
		.owner = THIS_MODULE,
		.of_match_table = mtk_vdisp_ctrl_dt_match,
	},
};

module_platform_driver(mtk_vdisp_ctrl_driver);

MODULE_AUTHOR("Nancy Lin <nancy.lin@mediatek.com>");
MODULE_DESCRIPTION("MediaTek VDISP Driver");
MODULE_LICENSE("GPL");
