/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2025 MediaTek Inc.
 * Author: Echo.Zhang <echo.zhang@mediatek.com>
 */
#ifndef __MMQOS_GLOBAL_H__
#define __MMQOS_GLOBAL_H__

extern u32 log_level;

#define mmqos_dbg(dev, level, fmt, args...)                                      \
			dev_err(dev, "[mmqos dbg]level=%d %s(),%d: " fmt "\n",   \
				(level), __func__, __LINE__, ##args)       

#define mmqos_err(dev, fmt, args...) \
	dev_err(dev, "[mmqos err]%s %d: " fmt "\n", __func__, __LINE__, ##args)

enum mmqos_state_level {
	MMQOS_DISABLE = 0,
	OSTD_ENABLE = BIT(0),
	BWL_ENABLE = BIT(1),
	DVFSRC_ENABLE = BIT(2),
	COMM_OSTDL_ENABLE = BIT(3),
	DISP_BY_LARB_ENABLE = BIT(4),
	VCP_ENABLE = BIT(5),
	VCODEC_BW_BYPASS = BIT(6),
	DPC_ENABLE = BIT(7),
	VMMRC_ENABLE = BIT(8),
	VMMRC_VCP_ENABLE = BIT(9),
	CAM_NO_MAX_OSTDL = BIT(10),
	MMPC_ENABLE = BIT(11),
	REAL_TIME_PERM_ENABLE = BIT(12),
	FORCE_BW_TO_OSTDL = BIT(13),
};

enum mmqos_log_level {
	LOG_BW = 0,
	LOG_COMM_FREQ,
	LOG_V2_DBG,
	LOG_VCP_PWR,
	LOG_IPI,
	LOG_DEBUG,
};
#endif /* MMQOS_GLOBAL_H */
