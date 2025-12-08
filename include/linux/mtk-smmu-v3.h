/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef _MTK_SMMU_V3_H_
#define _MTK_SMMU_V3_H_

unsigned int mtk_smmu_v3_get_type(struct device *dev);
u64 mtk_smmu_v3_get_smmu_tab_id(struct device *dev);

#endif
