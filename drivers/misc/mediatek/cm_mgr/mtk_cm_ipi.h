/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2023 MediaTek Inc.
 */

#ifndef __MTK_CM_IPI_H__
#define __MTK_CM_IPI_H__

enum {
	IPI_CM_MGR_SCMI_SET = 0x00000000,
	IPI_CM_MGR_SCMI_GET = 0x10000000,
};

struct cm_ipi_data {
	unsigned int cmd;
	unsigned int arg;
};

struct cm_ipi_common_data {
	struct device *dev;
	int cm_ipi_enable;
	int scmi_cm_id;
	struct scmi_tinysys_info_st *tinfo;
};

int cm_mgr_to_sspm_command_ipi(unsigned int cmd, unsigned int val);
int cm_ipi_init(struct device *dev);

#endif /* __MTK_CM_MGR_IPI_H__ */
