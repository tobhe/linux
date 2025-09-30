// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek CPU Memory Latency Manager (CM_MGR) Interconnect driver
 *
 * Copyright (c) 2023 MediaTek Inc.
 * Copyright (c) 2025 Collabora Ltd.
 *                    AngeloGioacchino Del Regno <angelogioacchino.delregno@collabora.com>
 */

#include <linux/bitfield.h>
#include <linux/interconnect.h>
#include <linux/interconnect-provider.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/soc/mediatek/dvfsrc.h>
#include <linux/scmi_protocol.h>
#include <linux/scmi_mtk_protocol.h>

#include "icc-emi.h"

#define MTK_CM_MGR_SSPM_VER_MAJOR	GENMASK(23, 16)
#define MTK_CM_MGR_SSPM_VER_MINOR	GENMASK(15, 8)
#define MTK_CM_MGR_SSPM_VER_PATCH	GENMASK(7, 0)

#define MTK_CM_MGR_SCMI_CTRL_ID		9

/* Note: This enumeration is firmware ABI for SSPMv3! */
enum mtk_cm_mgr_ipi_cmds {
	MTK_CM_MGR_CMD_INIT,
	MTK_CM_MGR_CMD_ENABLE,
	MTK_CM_MGR_CMD_OPP_ENABLE = 3,
	MTK_CM_MGR_CMD_DRAM_LEVEL = 17,
	MTK_CM_MGR_CMD_SSPM_VER = 47,
	MTK_CM_MGR_CMD_CHIP_VER = 52
};

struct mtk_cm_mgr {
	const struct scmi_mtk_tinysys_proto_ops *spops;
	const struct scmi_protocol_handle *sph;
	struct device *dev;
	u32 sspm_version;
};

static int mtk_cm_mgr_get_sspm_version(struct mtk_cm_mgr *cm)
{
	struct scmi_mtk_tinysys_status scmi_rval;
	u32 fwver;
	int ret;

	ret = cm->spops->common_get(cm->sph, 9, MTK_CM_MGR_CMD_SSPM_VER, &scmi_rval);
	if (ret) {
		pr_err("Cannot get SSPM Version.\n");
		return ret;
	}
	fwver = scmi_rval.reply[0];

	dev_info(cm->dev,
		 "MediaTek System Security Processor Manager (SSPM) Version %lu.%lu.%lu\n",
		 FIELD_GET(MTK_CM_MGR_SSPM_VER_MAJOR, fwver),
		 FIELD_GET(MTK_CM_MGR_SSPM_VER_MINOR, fwver),
		 FIELD_GET(MTK_CM_MGR_SSPM_VER_PATCH, fwver));

	return fwver;
}

static int scmi_mtk_cm_mgr_icc_probe(struct scmi_device *sdev)
{
	const struct scmi_handle *scmi = sdev->handle;
	struct scmi_protocol_handle *sph;
	struct device *dev = &sdev->dev;
	struct mtk_cm_mgr *cm;
	int ret;

	if (!scmi)
		return -ENODEV;

	cm = devm_kzalloc(dev, sizeof(*cm), GFP_KERNEL);
	if (!cm)
		return -ENOMEM;

	cm->spops = scmi->devm_protocol_get(sdev, SCMI_PROTOCOL_MTK_TINYSYS, &sph);
	if (IS_ERR(cm->spops))
		return PTR_ERR(cm->spops);

	cm->dev = dev;
	cm->sph = sph;
	dev_set_drvdata(dev, cm);

	ret = mtk_cm_mgr_get_sspm_version(cm);
	if (ret < 0)
		return ret;
	cm->sspm_version = ret;

	/* Currently, the only supported SSPM firmware is v3 */
	if (FIELD_GET(MTK_CM_MGR_SSPM_VER_MAJOR, cm->sspm_version) != 3)
		return dev_err_probe(dev, -ENOTSUPP,
				     "The current SSPM firmware is not yet supported.\n");

	

	return 0;
}

static const struct scmi_device_id scmi_mtk_cm_mgr_id_table[] = {
	{ SCMI_PROTOCOL_MTK_TINYSYS, "mtk-cm-mgr" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(scmi, scmi_mtk_cm_mgr_id_table);

static struct scmi_driver scmi_mtk_cm_mgr_driver = {
	.name = "scmi-mtk-cm-mgr",
	.probe = scmi_mtk_cm_mgr_icc_probe,
	.id_table = scmi_mtk_cm_mgr_id_table,
};
module_scmi_driver(scmi_mtk_cm_mgr_driver);

MODULE_AUTHOR("AngeloGioacchino Del Regno <angelogioacchino.delregno@collabora.com>");
MODULE_DESCRIPTION("MediaTek CPU Memory Latency Manager driver");
MODULE_LICENSE("GPL");
