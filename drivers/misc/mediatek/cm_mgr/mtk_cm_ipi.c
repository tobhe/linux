// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2023 MediaTek Inc.
 */

#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/scmi_protocol.h>
#include <tinysys-scmi.h>
#include "mtk_cm_ipi.h"

static struct cm_ipi_common_data  cm_ipi_data_c = {
	.cm_ipi_enable = 1,
};

int cm_mgr_to_sspm_command_ipi(unsigned int cmd, unsigned int val)
{
	struct cm_ipi_data cm_ipi_d;
	struct scmi_tinysys_status rvalue;
	unsigned int ret = 0;
	unsigned int type;
	unsigned int mask = GENMASK(31, 28);

	type = cmd & mask;

	cm_ipi_d.cmd = cmd & ~mask;
	cm_ipi_d.arg = val;

	if (cm_ipi_data_c.cm_ipi_enable != 1) {
		dev_err(cm_ipi_data_c.dev, "cm ipi not ready, skip cmd=%d\n", cm_ipi_d.cmd);
		goto error;
	}

	dev_dbg(cm_ipi_data_c.dev, "cmd 0x%x, arg 0x%x\n", cm_ipi_d.cmd, cm_ipi_d.arg);

	switch (type) {
	case IPI_CM_MGR_SCMI_SET:
		ret = scmi_tinysys_common_set(cm_ipi_data_c.tinfo->ph, cm_ipi_data_c.scmi_cm_id,
					      cm_ipi_d.cmd, cm_ipi_d.arg, 0, 0,
					      0);
		if (ret) {
			dev_err(cm_ipi_data_c.dev, "cm ipi cmd %d send fail, ret = %d\n",
				cm_ipi_d.cmd, ret);
			goto error;
		}
		break;
	case IPI_CM_MGR_SCMI_GET:
		ret = scmi_tinysys_common_get(cm_ipi_data_c.tinfo->ph, cm_ipi_data_c.scmi_cm_id,
					      cm_ipi_d.cmd, &rvalue);
		if (ret) {
			dev_err(cm_ipi_data_c.dev, "cm ipi cmd %d send fail, ret = %d rvalue %d\n",
				cm_ipi_d.cmd, ret, rvalue.r1);
			goto error;
		} else {
			ret = rvalue.r1;
		}
		break;
	default:
		dev_err(cm_ipi_data_c.dev, "wrong cmd type(0x%x)!!!\n", type);
		break;
	}

	return ret;
error:
	return -ENOMEM;
}
EXPORT_SYMBOL(cm_mgr_to_sspm_command_ipi);

int cm_ipi_init(struct device *dev)
{
	unsigned int ret;

	cm_ipi_data_c.dev = dev;
	cm_ipi_data_c.tinfo = get_scmi_tinysys_info();
	if (!cm_ipi_data_c.tinfo) {
		dev_err(cm_ipi_data_c.dev, "get scmi-tinysys-info fail, ret\n");
		return -EPROBE_DEFER;
	}

	ret = of_property_read_u32(cm_ipi_data_c.tinfo->sdev->dev.of_node, "scmi-cm",
				   &cm_ipi_data_c.scmi_cm_id);
	if (ret) {
		dev_err(cm_ipi_data_c.dev, "get scmi-cm fail, ret %d\n", ret);
		cm_ipi_data_c.cm_ipi_enable = 0;
		return -EINVAL;
	}
	dev_dbg(cm_ipi_data_c.dev, "scmi-cm_id %d\n", cm_ipi_data_c.scmi_cm_id);

	cm_ipi_data_c.cm_ipi_enable = 1;
	return 0;
}
EXPORT_SYMBOL(cm_ipi_init);
MODULE_DESCRIPTION("CM ipi Driver v0.1");
MODULE_LICENSE("GPL");
