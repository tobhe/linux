// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024 MediaTek Inc.
 * Author: Yong Wu <yong.wu@mediatek.com>
 */

#include "arm-smmu-v3.h"

struct arm_smmu_device *arm_smmu_v3_impl_init(struct arm_smmu_device *smmu)
{
	if (IS_ENABLED(CONFIG_ARM_SMMU_V3_MEDIATEK))
		smmu = arm_smmu_v3_impl_mtk_init(smmu);
	return smmu;
}
