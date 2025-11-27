// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 MediaTek Inc.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#if IS_ENABLED(CONFIG_MTK_VCP_RPROC)
#include <linux/remoteproc.h>
#include <linux/remoteproc/mtk_vcp_public.h>
#endif
#include <linux/rpmsg.h>
#include <linux/rpmsg/mtk_rpmsg.h>
#include <linux/sched/clock.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/workqueue.h>
#include <soc/mediatek/mmdvfs_v3.h>
#include <soc/mediatek/mmdvfs_v3_memory.h>
#include <soc/mediatek/smi.h>

#if IS_ENABLED(CONFIG_MTK_MMDEBUG)
#include <soc/mediatek/mmdebug.h>
#endif
#include "mtk-mmdvfs-v3.h"
#include "mtk-mmdvfs-v3-user.h"

static bool mmdvfs_rst_clk_high_rate;

/* If mmup_ena is true, MMDVFS_MMUP_FEATURE_ID is used.
 * Otherwise, if mmup_ena is false, MMDVFS_VCP_FEATURE_ID should be used.
 * Since the code never sets mmup_ena to false,
 * it's simplified to directly use MMDVFS_MMUP_FEATURE_ID.
 */
#define MMDVFS_HFRP_FEATURE_ID MMDVFS_MMUP_FEATURE_ID

static DEFINE_MUTEX(mmdvfs_vcp_pwr_mutex);
static DEFINE_MUTEX(mmdvfs_vcp_cb_mutex);
static DEFINE_MUTEX(mmdvfs_vcp_ipi_mutex);
static DEFINE_MUTEX(mmdvfs_vmm_pwr_mutex);

#define MMDVFS_IPI_SEND_TIMEOUT       1000000
#define MMDVFS_IPI_DELAY_US           100
#define MMDVFS_REQUEST_VCP_MAX_COUNT  10000
#define MMDVFS_MAX_RETRY_COUNT        100
#define MMDVFS_DEFAULT_RATE           26000000UL
#define DEFAULT_DPSW_THR_VALUE        1

int mmdvfs_log_level;
module_param(mmdvfs_log_level, uint, 0644);
MODULE_PARM_DESC(mmdvfs_log_level, "mmdvfs log level");

enum {
	LOG_PWR,
	LOG_IPI,
	LOG_CLK_OPS,
	LOG_DBG,
	LOG_RST_CLK,
	LOG_NUM,
};

void *mmdvfs_get_mmup_base(void)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();

	return (IS_ERR_OR_NULL(mmdvfs_dev))? NULL : mmdvfs_dev->mmdvfs_mmup_va;
}
EXPORT_SYMBOL_GPL(mmdvfs_get_mmup_base);

void *mmdvfs_get_vcp_base(void)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();

	return (IS_ERR_OR_NULL(mmdvfs_dev))? NULL : mmdvfs_dev->mmdvfs_vcp_va;
}
EXPORT_SYMBOL_GPL(mmdvfs_get_vcp_base);

bool mmdvfs_get_mmup_sram_enable(void)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();

	return (IS_ERR_OR_NULL(mmdvfs_dev))? false : mmdvfs_dev->mmdvfs_mmup_sram;
}
EXPORT_SYMBOL_GPL(mmdvfs_get_mmup_sram_enable);

void __iomem *mmdvfs_get_mmup_sram(void)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();

	return (IS_ERR_OR_NULL(mmdvfs_dev))? NULL : mmdvfs_dev->mmdvfs_mmup_sram_va;
}
EXPORT_SYMBOL_GPL(mmdvfs_get_mmup_sram);

bool mmdvfs_is_init_done(void)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();

	return (IS_ERR_OR_NULL(mmdvfs_dev))? false :
		(mmdvfs_dev->mmdvfs_free_run ? mmdvfs_dev->mmdvfs_init_done : false);
}
EXPORT_SYMBOL_GPL(mmdvfs_is_init_done);

int mtk_mmdvfs_enable_vcp(const bool enable, enum VCP_PWR_USR idx)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	struct mtk_vcp_device  *vcp_device;
	int ret = 0;

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return -EINVAL;

	vcp_device = mmdvfs_dev->vcp_device;

	if (IS_ERR_OR_NULL(vcp_device)) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "Invalid vcp_device!");
		return -EINVAL;
	}

	if (!mmdvfs_dev->mmdvfs_free_run)
		return 0;

	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_PWR, "+++ vcp_power:%d", mmdvfs_dev->vcp_power);
	if (vcp_device->data->vcp_is_suspending(vcp_device))
		return -EBUSY;

	mutex_lock(&mmdvfs_vcp_pwr_mutex);
	if (enable) {
		if (!mmdvfs_dev->vcp_power) {
			ret = vcp_device->data->vcp_register_feature(vcp_device,
								     MMDVFS_HFRP_FEATURE_ID);
			if (ret)
				goto enable_vcp_end;
		}
		mmdvfs_dev->vcp_power += 1;
		mmdvfs_dev->vcp_pwr_usage[idx] += 1;
	} else {
		if (!mmdvfs_dev->vcp_pwr_usage[idx] || !mmdvfs_dev->vcp_power) {
			ret = -EINVAL;
			goto enable_vcp_end;
		}
		if (mmdvfs_dev->vcp_power == 1) {
			ret = vcp_device->data->vcp_deregister_feature(vcp_device,
								       MMDVFS_HFRP_FEATURE_ID);
			if (ret)
				goto enable_vcp_end;
		}
		mmdvfs_dev->vcp_pwr_usage[idx] -= 1;
		mmdvfs_dev->vcp_power -= 1;
	}

enable_vcp_end:
	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_PWR, "ret:%d enable:%d vcp_power:%d idx:%u usage:%d",
			 ret, enable, mmdvfs_dev->vcp_power, idx,
			 mmdvfs_dev->vcp_pwr_usage[idx]);
	mutex_unlock(&mmdvfs_vcp_pwr_mutex);

	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_PWR, "--- vcp_power:%d", mmdvfs_dev->vcp_power);

	return ret;
}
EXPORT_SYMBOL_GPL(mtk_mmdvfs_enable_vcp);

static int mmdvfs_vcp_ipi_send_ex(struct mtk_mmdvfs_dev *mmdvfs_dev, enum ipi_func_id func,
				  const u8 idx, const u8 opp, u32 *data, const bool vcp)
{
	struct platform_device *pdev = mmdvfs_dev->pdev;
	const u32 feature_id = vcp ? MMDVFS_VCP_FEATURE_ID : MMDVFS_MMUP_FEATURE_ID;
	struct mtk_vcp_device  *vcp_device = mmdvfs_dev->vcp_device;
	struct mtk_ipi_device *vcp_ipi_dev;
	struct mmdvfs_ipi_data slot = {func, idx, opp,
		(vcp ? mmdvfs_dev->mmdvfs_vcp_iova : mmdvfs_dev->mmdvfs_mmup_iova) >> 32,
		(u32)(vcp ? mmdvfs_dev->mmdvfs_vcp_iova : mmdvfs_dev->mmdvfs_mmup_iova)};
	int ret = 0, retry = 0;
	u32 val;

	if (!mmdvfs_is_init_done())
		return -ENODEV;

	mutex_lock(&mmdvfs_vcp_ipi_mutex);
	switch (func) {
	case FUNC_VMM_CEIL_ENABLE:
		val = readl(MEM_VMM_CEIL_ENABLE);
		if (opp)
			writel(val | BIT(idx), MEM_VMM_CEIL_ENABLE);
		else
			writel(val & ~BIT(idx), MEM_VMM_CEIL_ENABLE);
		break;
	case FUNC_VCORE_CEIL_LEVEL:
		if (!data || *data >= PWR_MMDVFS_NUM) {
			ret = -EINVAL;
			goto ipi_lock_end;
		}
		val = readl(MEM_CEIL_LEVEL(*data));
		val &= ~GENMASK(idx * 4 + 3, idx * 4); /* clear pre ceil level */
		val |= opp << (idx * 4); /* set cur ceil level */
		writel(val, MEM_CEIL_LEVEL(*data));
		break;
	case FUNC_VMM_GENPD_NOTIFY:
		if (idx >= VMM_USR_NUM) {
			ret = -EINVAL;
			goto ipi_lock_end;
		}
		writel(opp, MEM_GENPD_ENABLE_USR(idx));
		break;
	case FUNC_VMM_AVS_UPDATE:
		if (idx >= VMM_USR_NUM) {
			ret = -EINVAL;
			goto ipi_lock_end;
		}
		writel(data[0], MEM_AGING_CNT_USR(idx));
		writel(data[1], MEM_FRESH_CNT_USR(idx));
		break;
	case FUNC_FORCE_OPP:
		writel(opp, MEM_FORCE_OPP_PWR(idx));
		break;
	case FUNC_VOTE_OPP:
		writel(opp, MEM_VOTE_OPP_USR(idx));
		break;
	case FUNC_MMDVFS_LP_MODE:
		writel(idx, MEM_MMDVFS_LP_MODE);
		break;
	case FUNC_MMDVFS_PROFILE:
		writel(*data, MEM_PROFILE_TIMES);
		break;
	default:
		break;
	}
	val = readl(MEM_IPI_SYNC_FUNC(vcp));
	mutex_unlock(&mmdvfs_vcp_ipi_mutex);

	while (!vcp_device->data->vcp_is_ready(feature_id) || !mmdvfs_dev->mmdvfs_vcp_cb_ready ||
	       mmdvfs_dev->mmdvfs_rst_clk_done) {
		if (!mmdvfs_dev->mmdvfs_vcp_cb_ready &&
		    (func == FUNC_MMDVFS_INIT || func == FUNC_MMDVFSRC_INIT))
			break;
		if (mmdvfs_dev->mmdvfs_rst_clk_done && func == FUNC_CLKMUX_ENABLE)
			break;
		if (func == FUNC_VMM_GENPD_NOTIFY || func == FUNC_VMM_CEIL_ENABLE ||
		    func == FUNC_MMDVFS_LP_MODE || (func == FUNC_CLKMUX_ENABLE && !opp))
			goto ipi_send_end;
		if (++retry > VCP_SYNC_TIMEOUT_MS) {
			ret = -ETIMEDOUT;
			goto ipi_send_end;
		}
		mdelay(1);
	}

	mutex_lock(&mmdvfs_vcp_ipi_mutex);
	writel(0, MEM_IPI_SYNC_DATA(vcp));
	writel(val | BIT(func), MEM_IPI_SYNC_FUNC(vcp));

	mutex_lock(&mmdvfs_vcp_cb_mutex);
	if (!mmdvfs_dev->mmdvfs_vcp_cb_ready && func != FUNC_MMDVFS_INIT &&
	    func != FUNC_MMDVFSRC_INIT) {
		mutex_unlock(&mmdvfs_vcp_cb_mutex);
		ret = -ETIMEDOUT;
		goto ipi_lock_end;
	}
	mutex_unlock(&mmdvfs_vcp_cb_mutex);

	vcp_ipi_dev = vcp_device->ipi_dev;
	if (!vcp_ipi_dev)
		goto ipi_lock_end;

	ret = vcp_device->ipi_ops->ipi_send(vcp_ipi_dev,
					    vcp ? IPI_OUT_MMDVFS_VCP : IPI_OUT_MMDVFS_MMUP,
					    &slot, PIN_OUT_SIZE_MMDVFS, IPI_TIMEOUT_MS);
	if (ret != IPI_ACTION_DONE)
		goto ipi_lock_end;


	ret = readl_poll_timeout(MEM_IPI_SYNC_DATA(vcp), val, (val & (1 << func)),
				 MMDVFS_IPI_DELAY_US, MMDVFS_IPI_SEND_TIMEOUT);
	if (ret) {
		if (!vcp_device->data->vcp_is_ready(feature_id))
			ret = -ETIMEDOUT;
		else
			ret = IPI_COMPL_TIMEOUT;
	}

	if (!ret)
		writel(val & ~readl(MEM_IPI_SYNC_DATA(vcp)), MEM_IPI_SYNC_FUNC(vcp));

ipi_lock_end:
	val = readl(MEM_IPI_SYNC_FUNC(vcp));
	mutex_unlock(&mmdvfs_vcp_ipi_mutex);

ipi_send_end:
	mtk_mmdvfs_debug(pdev, LOG_IPI, "ret:%d %d vcp:%d id:%u %d %d %llx vcp_power:%d func:%#x",
			 ret, retry, vcp, feature_id,
			 vcp_device->data->vcp_is_ready(feature_id),
			 mmdvfs_dev->mmdvfs_vcp_cb_ready, *(u64 *)&slot,
			 mmdvfs_dev->vcp_power, val);
	mtk_mmdvfs_debug(pdev, LOG_IPI, "rst_clk:%d data:%#x pm time:%llu time:%llu ready:%llu",
			 mmdvfs_dev->mmdvfs_rst_clk_done, readl(MEM_IPI_SYNC_DATA(vcp)),
			 mmdvfs_dev->cb_timestamp[0], mmdvfs_dev->cb_timestamp[1],
			 mmdvfs_dev->cb_timestamp[2]);
	mmdvfs_dev->mmdvfs_ipi_status = ret;

	return ret;
}

static int mmdvfs_vcp_ipi_send(struct mtk_mmdvfs_dev *mmdvfs_dev, enum ipi_func_id func,
			       const u8 idx, const u8 opp, u32 *data)
{
	return mmdvfs_vcp_ipi_send_ex(mmdvfs_dev, func, idx, opp, data, !(mmdvfs_dev->mmup_ena));
}

static inline int rate_to_opp(struct mtk_mmdvfs_clk *clk, int rate)
{
	int opp;
	opp = (clk->freq_num - ((rate == clk->freq_num) ? (rate - 1) : rate) - 1);
	return opp;
}

static int mtk_mmdvfs_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	struct mtk_vcp_device  *vcp_device = mmdvfs_dev->vcp_device;
	struct mtk_mmdvfs_clk *clk = container_of(hw, typeof(*clk), clk_hw);
	u8 opp, pwr_opp = MMDVFS_MAX_OPP, user_opp = MMDVFS_MAX_OPP;
	u32 img_clk = rate / 1000000UL;
	int i, ret = 0, retry = 0;

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return 0;

	if (!mmdvfs_is_init_done())
		return 0;

	for (i = 0; i < clk->freq_num; i++) {
		if (rate <= clk->freqs[i])
			break;
	}

	opp = rate_to_opp(clk, i);
	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_CLK_OPS,
			 "user_id:%u clk_id:%u opp:%u rate:%lu opp:%u",
			 clk->user_id, clk->clk_id, clk->opp, rate, opp);

	if (clk->opp == opp)
		return 0;
	clk->opp = opp;

	/* Choose max step among all users of special independence */
	if (clk->spec_type == SPEC_MMDVFS_ALONE) {
		for (i = 0; i < mmdvfs_dev->mmdvfs_clk_num; i++) {
			if (clk->user_id == mmdvfs_dev->mtk_mmdvfs_clks[i].user_id &&
			    mmdvfs_dev->mtk_mmdvfs_clks[i].opp < user_opp)
				user_opp = mmdvfs_dev->mtk_mmdvfs_clks[i].opp;
		}
		ret = mmdvfs_vcp_ipi_send(mmdvfs_dev, FUNC_VOTE_OPP, clk->user_id, user_opp, NULL);
		goto set_rate_end;
	}

	/* spec_type != SPEC_MMDVFS_ALONE */
	for (i = 0; i < mmdvfs_dev->mmdvfs_clk_num; i++)
		if (clk->pwr_id == mmdvfs_dev->mtk_mmdvfs_clks[i].pwr_id &&
		    mmdvfs_dev->mtk_mmdvfs_clks[i].opp < pwr_opp &&
		    mmdvfs_dev->mtk_mmdvfs_clks[i].spec_type != SPEC_MMDVFS_ALONE)
			pwr_opp = mmdvfs_dev->mtk_mmdvfs_clks[i].opp;

	if (pwr_opp == mmdvfs_dev->mmdvfs_pwr_opp[clk->pwr_id])
		return 0;

	mmdvfs_dev->mmdvfs_pwr_opp[clk->pwr_id] = pwr_opp;

	while (!vcp_device->data->vcp_is_ready(MMDVFS_HFRP_FEATURE_ID) ||
	       !mmdvfs_dev->mmdvfs_vcp_cb_ready) {
		if (++retry > VCP_SYNC_TIMEOUT_MS) {
			ret = -ETIMEDOUT;
			goto set_rate_end;
		}
		mdelay(1);
	}

	ret = mmdvfs_vcp_ipi_send(mmdvfs_dev, FUNC_VOTE_OPP, clk->user_id, pwr_opp, NULL);
set_rate_end:
	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_CLK_OPS,
		       "ret:%d retry:%d ready(%d %d) %u %u %u %lu opp(%u %u %u) clk:%u",
		       ret, retry, vcp_device->data->vcp_is_ready(MMDVFS_HFRP_FEATURE_ID),
		       mmdvfs_dev->mmdvfs_vcp_cb_ready, clk->user_id, clk->clk_id, clk->opp,
		       rate, opp, pwr_opp, user_opp, img_clk);
	return ret;
}

static long mtk_mmdvfs_round_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long *parent_rate)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	struct mtk_mmdvfs_clk *clk = container_of(hw, typeof(*clk), clk_hw);
	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return 0;

	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_CLK_OPS, "clk_id:%u opp:%u rate:%lu",
			 clk->clk_id, clk->opp, rate);

	return rate;
}

static unsigned long mtk_mmdvfs_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	struct mtk_mmdvfs_clk *clk = container_of(hw, typeof(*clk), clk_hw);

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return 0;

	if (clk->opp >= clk->freq_num)
		return 0;

	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_CLK_OPS, "clk_id:%u opp:%u rate:%lu freq:%u",
			 clk->clk_id, clk->opp, parent_rate,
			 clk->freqs[clk->freq_num - clk->opp - 1]);

	return clk->freqs[clk->freq_num - clk->opp - 1];
}

static const struct clk_ops mtk_mmdvfs_req_ops = {
	.set_rate	= mtk_mmdvfs_set_rate,
	.round_rate	= mtk_mmdvfs_round_rate,
	.recalc_rate	= mtk_mmdvfs_recalc_rate,
};

int mtk_mmdvfs_force_vcore_notify(const u32 val)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	u32 pwr = PWR_MMDVFS_VCORE;
	int ret;

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return -ENODEV;

	if (!mmdvfs_dev->vcore_level_count || val >= mmdvfs_dev->vcore_level_count)
		return -EINVAL;

	ret = mmdvfs_vcp_ipi_send(mmdvfs_dev, FUNC_VCORE_CEIL_LEVEL, VCORE_CEIL_USR_VCORE,
				  mmdvfs_dev->vcore_level[val], &pwr);
	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_DBG, "ret:%d force vcore:%u final vcore:%u",
			 ret, val, mmdvfs_dev->vcore_level[val]);

	return ret;
}
EXPORT_SYMBOL_GPL(mtk_mmdvfs_force_vcore_notify);

static int mmdvfs_vcore_ceil_step(const char *val, const struct kernel_param *kp)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	int result;
	u32 level;
	u32 pwr = PWR_MMDVFS_VCORE;

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return -ENODEV;

	result = kstrtou32(val, 0, &level);
	if (result) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "fail ret:%d\n", result);
		return result;
	}

	return mmdvfs_vcp_ipi_send(mmdvfs_dev, FUNC_VCORE_CEIL_LEVEL,
				   VCORE_CEIL_USR_ADB, level, &pwr);
}

static const struct kernel_param_ops mmdvfs_vcore_ceil_ops = {
	.set = mmdvfs_vcore_ceil_step,
};
module_param_cb(vcore_ceil, &mmdvfs_vcore_ceil_ops, NULL, 0644);
MODULE_PARM_DESC(vcore_ceil, "vcore ceil level");

void vmm_notify_work_func(struct work_struct *work)
{
	struct mmdvfs_vmm_notify_work *vmm_notify_work =
		container_of(work, struct mmdvfs_vmm_notify_work, vmm_notify_work);

	mtk_mmdvfs_enable_vcp(vmm_notify_work->enable, VCP_PWR_USR_MMDVFS_GENPD);
	kfree(vmm_notify_work);
}

int mtk_mmdvfs_genpd_notify(const u8 idx, const bool enable)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	struct mmdvfs_vmm_notify_work *work;

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return -ENODEV;

	if (!mmdvfs_is_init_done())
		return 0;

	mmdvfs_vcp_ipi_send(mmdvfs_dev, FUNC_VMM_GENPD_NOTIFY, idx, enable ? 1 : 0, NULL);

	if (!mmdvfs_dev->vmm_notify_wq)
		return 0;

	work = kzalloc(sizeof(*work), GFP_KERNEL);
	if (!work)
		return -ENOMEM;

	work->enable = enable;
	INIT_WORK(&work->vmm_notify_work, vmm_notify_work_func);
	queue_work(mmdvfs_dev->vmm_notify_wq, &work->vmm_notify_work);

	return 0;
}
EXPORT_SYMBOL_GPL(mtk_mmdvfs_genpd_notify);

int mtk_mmdvfs_set_avs(const u8 idx, const u32 aging, const u32 fresh)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	u32 data[2] = {aging, fresh};
	int ret;

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return -ENODEV;

	ret = mmdvfs_vcp_ipi_send(mmdvfs_dev, FUNC_VMM_AVS_UPDATE, idx, MMDVFS_MAX_OPP, (u32 *)&data);
	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_PWR, "ret:%d idx:%u aging:%u fresh:%u",
			 ret, idx, aging, fresh);

	return ret;
}
EXPORT_SYMBOL_GPL(mtk_mmdvfs_set_avs);

int vmm_avs_debug_dump(char *buf, const struct kernel_param *kp)
{
	int len = 0;
	u32 avs, i;

	if (!MEM_BASE)
		return 0;

	/* power opp */
	len += scnprintf(buf + len, PAGE_SIZE - len, "efuse_low:%#x, efuse_high:%#x\n",
			readl(MEM_VMM_EFUSE_LOW), readl(MEM_VMM_EFUSE_HIGH));
	for (i = 0; i < 8; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "opp_level%u: %u\n", i,
				readl(MEM_VMM_OPP_VOLT(i)));

	i = readl(MEM_REC_VMM_DBG_CNT);
	if (i > 0)
		i = (i - 1) % MEM_REC_CNT_MAX;

	avs = readl(MEM_REC_VMM_AVS(i));
	len += scnprintf(buf + len, PAGE_SIZE - len,
			"temp:%#x volt:%u zone:%u opp_level:%u vde_on:%u isp_on:%u\n",
			readl(MEM_REC_VMM_TEMP(i)), readl(MEM_REC_VMM_VOLT(i)),
			avs & 0xff, (avs >> 8) & 0xff, (avs >> 16) & 0x1, (avs >> 17) & 0x1);

	return len;
}

static const struct kernel_param_ops vmm_avs_debug_dump_ops = {
	.get = vmm_avs_debug_dump,
};
module_param_cb(avs_debug, &vmm_avs_debug_dump_ops, NULL, 0644);
MODULE_PARM_DESC(avs_debug, "dump avs debug information");

int mtk_mmdvfs_camsv_dc_enable(const u8 idx, const bool enable)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	int ret;

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return 0;

	ret = mmdvfs_vcp_ipi_send(mmdvfs_dev, FUNC_CAMSV_DC_ENABLE, idx, enable ? 1 : 0, NULL);
	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_PWR, "ret:%d idx:%u enable:%d", ret, idx, enable);

	return ret;
}
EXPORT_SYMBOL_GPL(mtk_mmdvfs_camsv_dc_enable);

static int set_camsv_dc_enable(const char *val, const struct kernel_param *kp)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	int ena, idx, ret;

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return 0;

	ret = sscanf(val, "%d %d", &idx, &ena);
	if (ret != 2) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "input failed:%d idx:%d ena:%d", ret, idx, ena);
		return -EINVAL;
	}
	return mtk_mmdvfs_camsv_dc_enable(idx, ena);
}

static const struct kernel_param_ops camsv_dc_ops = {
	.set = set_camsv_dc_enable,
};
module_param_cb(camsv_dc, &camsv_dc_ops, NULL, 0644);
MODULE_PARM_DESC(camsv_dc, "camsv dc enable");

static int mtk_mmdvfs_enable_vmm(const bool enable)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	struct device *mmdvfs_v3_dev = NULL;
	int ret = 0;

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return 0;

	mmdvfs_v3_dev = mmdvfs_dev->mmdvfs_v3_dev;

	mutex_lock(&mmdvfs_vmm_pwr_mutex);
	if (enable) {
		if (!mmdvfs_dev->vmm_power) {
			ret = mmdvfs_v3_dev ? pm_runtime_resume_and_get(mmdvfs_v3_dev) :
				mmdvfs_vcp_ipi_send_ex(mmdvfs_dev, FUNC_VMM_BUCK_ENABLE,
						       VMM_USR_VDE, 1, NULL, true);
			if (ret)
				goto enable_vmm_end;
		}
		mmdvfs_dev->vmm_power += 1;
	} else {
		if (!mmdvfs_dev->vmm_power) {
			ret = -EINVAL;
			goto enable_vmm_end;
		}
		if (mmdvfs_dev->vmm_power == 1) {
			ret = mmdvfs_v3_dev ? pm_runtime_put_sync(mmdvfs_v3_dev) :
				mmdvfs_vcp_ipi_send_ex(mmdvfs_dev, FUNC_VMM_BUCK_ENABLE,
						       VMM_USR_VDE, 0, NULL, true);
			if (ret)
				goto enable_vmm_end;
		}
		mmdvfs_dev->vmm_power -= 1;
	}

enable_vmm_end:
	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_PWR, "ret:%d enable:%d vmm_power:%d dev:%p", ret,
		       enable, mmdvfs_dev->vmm_power, mmdvfs_dev->mmdvfs_v3_dev);
	mutex_unlock(&mmdvfs_vmm_pwr_mutex);

	return ret;
}

static int set_vmm_enable(const char *val, const struct kernel_param *kp)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	bool enable;
	int ret;

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return 0;

	ret = kstrtobool(val, &enable);
	if (ret) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "failed:%d enable:%d", ret, enable);
		return ret;
	}
	return mtk_mmdvfs_enable_vmm(enable);
}

static const struct kernel_param_ops enable_vmm_ops = {
	.set = set_vmm_enable,
};
module_param_cb(enable_vmm, &enable_vmm_ops, NULL, 0644);
MODULE_PARM_DESC(enable_vmm, "enable vmm");

static int mtk_mmdvfs_v3_set_vmm_ceil_step(const bool enable)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return 0;

	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_IPI, "enable:%u start", enable);

	mtk_mmdvfs_enable_vcp(true, VCP_PWR_USR_MMQOS);
	mmdvfs_vcp_ipi_send(mmdvfs_dev, FUNC_VMM_CEIL_ENABLE, VMM_CEIL_USR_ADB,
			    enable ? 1 : 0, NULL);
	mtk_mmdvfs_enable_vcp(false, VCP_PWR_USR_MMQOS);

	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_IPI, "enable:%u end", enable);

	return 0;
}

int mmdvfs_vmm_ceil_step(const char *val, const struct kernel_param *kp)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	int result;
	bool enable;

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return 0;

	result = kstrtobool(val, &enable);
	if (result) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "fail ret:%d\n", result);
		return result;
	}

	mtk_mmdvfs_v3_set_vmm_ceil_step(enable);
	return 0;
}

static const struct kernel_param_ops mmdvfs_vmm_ceil_ops = {
	.set = mmdvfs_vmm_ceil_step,
};
module_param_cb(vmm_ceil, &mmdvfs_vmm_ceil_ops, NULL, 0644);
MODULE_PARM_DESC(vmm_ceil, "enable vmm ceiling");

int mmdvfs_rst_clk_rate(const char *val, const struct kernel_param *kp)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	u8 mmdvfs_rst_clk_num;
	u8 idx;
	u32 rate;
	int ret;

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return 0;

	mmdvfs_rst_clk_num = mmdvfs_dev->mmdvfs_rst_clk_num;

	ret = sscanf(val, "%hhu %u", &idx, &rate);
	if (ret != 2 || (mmdvfs_rst_clk_num && idx >= mmdvfs_rst_clk_num)) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "input failed:%d idx:%u rate:%u rst_clk_num:%u",
			       ret, idx, rate, mmdvfs_rst_clk_num);
		return -EINVAL;
	}

	if (mmdvfs_rst_clk_num) {
		mtk_mmdvfs_enable_vcp(true, VCP_PWR_USR_MMDVFS_GENPD);
		ret = clk_set_rate(mmdvfs_dev->mmdvfs_rst_clk[idx], rate);
		mtk_mmdvfs_enable_vcp(false, VCP_PWR_USR_MMDVFS_GENPD);
		if (rate)
			mmdvfs_rst_clk_high_rate = true;
	}

	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_DBG, "ret:%d idx:%u rate:%u", ret, idx, rate);

	return ret;
}

static const struct kernel_param_ops mmdvfs_rst_clk_rate_ops = {
	.set = mmdvfs_rst_clk_rate,
};
module_param_cb(rst_clk_rate, &mmdvfs_rst_clk_rate_ops, NULL, 0644);
MODULE_PARM_DESC(rst_clk_rate, "set rst clk rate");

int mmdvfs_get_vcp_log(char *buf, const struct kernel_param *kp)
{
	int len = 0, ret;

	if (!mmdvfs_is_init_done())
		return 0;

	ret = readl(MEM_LOG_FLAG);
	len += scnprintf(buf + len, PAGE_SIZE - len, "MEM_LOG_FLAG:%#x", ret);
	return len;
}

static int mtk_mmdvfs_v3_set_vcp_log(const u32 log)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return 0;

	if (!mmdvfs_is_init_done()) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "mmdvfs is not init done");
		return -ENODEV;
	}

	writel(log, MEM_LOG_FLAG);
	return 0;
}

int mmdvfs_set_vcp_log(const char *val, const struct kernel_param *kp)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	u32 log = 0;
	int ret;

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return 0;

	ret = kstrtou32(val, 0, &log);
	if (ret || log >= (1 << LOG_NUM)) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "failed:%d log:%#x", ret, log);
		return ret;
	}

	ret = mtk_mmdvfs_v3_set_vcp_log(log);
	if (ret)
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "set vcp log failed:%d log:%#x", ret, log);

	return ret;
}

static const struct kernel_param_ops mmdvfs_set_vcp_log_ops = {
	.get = mmdvfs_get_vcp_log,
	.set = mmdvfs_set_vcp_log,
};
module_param_cb(vcp_log, &mmdvfs_set_vcp_log_ops, NULL, 0644);
MODULE_PARM_DESC(vcp_log, "mmdvfs vcp log");

int mmdvfs_get_vmrc_log(char *buf, const struct kernel_param *kp)
{
	int len = 0, ret;

	if (!mmdvfs_is_init_done())
		return 0;

	ret = readl(MEM_VMRC_LOG_FLAG);
	len += scnprintf(buf + len, PAGE_SIZE - len, "MEM_VMRC_LOG_FLAG:%#x", ret);
	return len;
}

static int mtk_mmdvfs_v3_set_vmrc_log(const u32 log)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return 0;

	if (!mmdvfs_is_init_done()) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "mmdvfs is not init done");
		return -ENODEV;
	}

	writel(log, MEM_VMRC_LOG_FLAG);
	return 0;
}

int mmdvfs_set_vmrc_log(const char *val, const struct kernel_param *kp)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	u32 log = 0;
	int ret;

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return 0;

	ret = kstrtou32(val, 0, &log);
	if (ret || log >= (1 << LOG_NUM)) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "failed:%d log:%#x", ret, log);
		return ret;
	}

	ret = mtk_mmdvfs_v3_set_vmrc_log(log);
	if (ret)
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "set vmrc log failed:%d log:%#x", ret, log);

	return ret;
}

static const struct kernel_param_ops mmdvfs_set_vmrc_log_ops = {
	.get = mmdvfs_get_vmrc_log,
	.set = mmdvfs_set_vmrc_log,
};
module_param_cb(vmrc_log, &mmdvfs_set_vmrc_log_ops, NULL, 0644);
MODULE_PARM_DESC(vmrc_log, "mmdvfs vmrc log");

module_param(mmdvfs_rst_clk_high_rate, bool, 0644);
MODULE_PARM_DESC(mmdvfs_rst_clk_high_rate, "mmdvfs reset clk high rate");

static DEFINE_SPINLOCK(mmdvfs_mux_lock);

static inline int opp_to_level(struct mmdvfs_mux *mmdvfs_mux, int mux, int opp)
{
	int level;
	level = (opp >= mmdvfs_mux[mux].freq_num) ? 0 : (mmdvfs_mux[mux].freq_num - 1 - opp);
	return level;
}

int mmdvfs_set_lp_mode_by_vcp(const bool enable)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	int ret;

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return -EINVAL;

	mmdvfs_dev->mmdvfs_lp_mode = enable;
	ret = mmdvfs_vcp_ipi_send(mmdvfs_dev, FUNC_MMDVFS_LP_MODE, enable ? 1 : 0,
				  MMDVFS_MAX_OPP, NULL);
	if (ret)
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "failed:%d enable:%d", ret, enable);

	return ret;
}
EXPORT_SYMBOL_GPL(mmdvfs_set_lp_mode_by_vcp);

void mmdvfs_rc_enable_set_fp(rc_enable fp)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return;

	mmdvfs_dev->dpc_fp = fp;
}
EXPORT_SYMBOL_GPL(mmdvfs_rc_enable_set_fp);

bool mmdvfs_get_mmup_enable(void)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();

	return (IS_ERR_OR_NULL(mmdvfs_dev))? false : mmdvfs_dev->mmup_ena;
}
EXPORT_SYMBOL_GPL(mmdvfs_get_mmup_enable);

bool mmdvfs_is_mux_version(void)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();

	return (IS_ERR_OR_NULL(mmdvfs_dev))? false : mmdvfs_dev->mmdvfs_mux_version;
}
EXPORT_SYMBOL_GPL(mmdvfs_is_mux_version);

static int mmdvfs_force_step_by_vcp_ipi(const u8 pwr_idx, const s8 opp)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	u8 idx = pwr_idx + MMDVFS_USER_VCORE;
	struct mmdvfs_mux *mux;
	int ret, *last;

	if (!mmdvfs_dev->mmdvfs_mux_version || idx >= MMDVFS_USER_NUM) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "invalid:%d pwr_idx:%u idx:%u",
			       mmdvfs_dev->mmdvfs_mux_version, pwr_idx, idx);
		return -EINVAL;
	}
	mux = &mmdvfs_dev->mmdvfs_mux[mmdvfs_dev->mmdvfs_user[idx].target_id];

	if (opp >= mux->freq_num) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "invalid opp:%d idx:%u mux:%u freq_num:%u",
			       opp, idx, mux->id, mux->freq_num);
		return -EINVAL;
	}

	last = &mmdvfs_dev->last_force_step[pwr_idx];
	if (mmdvfs_dev->dpsw_thr && mux->id >= MMDVFS_MUX_VDE && mux->id <= MMDVFS_MUX_CAM &&
	    opp >= 0 && opp < mmdvfs_dev->dpsw_thr && (*last < 0 || *last >= mmdvfs_dev->dpsw_thr))
		mtk_mmdvfs_enable_vmm(true);

	ret = mmdvfs_vcp_ipi_send(mmdvfs_dev, FUNC_FORCE_OPP, pwr_idx, opp, NULL);
	if (mmdvfs_dev->dpsw_thr && mux->id >= MMDVFS_MUX_VDE && mux->id <= MMDVFS_MUX_CAM &&
	    (opp < 0 || opp >= mmdvfs_dev->dpsw_thr) && *last >= 0 && *last < mmdvfs_dev->dpsw_thr)
		mtk_mmdvfs_enable_vmm(false);
	*last = opp;

	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_DBG, "pwr_idx:%u idx:%u mux:%u opp:%d",
			 pwr_idx, idx, mux->id, opp);

	return ret;
}

int mmdvfs_force_step_by_vcp(const u8 pwr_idx, const s8 opp)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	struct mmdvfs_mux *mux;
	u8 idx = pwr_idx + MMDVFS_USER_VCORE;
	int ret;

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return -EINVAL;

	if (!mmdvfs_dev->mmdvfs_mux_version || idx >= MMDVFS_USER_NUM) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "invalid:%d pwr_idx:%u idx:%u",
			       mmdvfs_dev->mmdvfs_mux_version, pwr_idx, idx);
		return -EINVAL;
	}
	mux = &mmdvfs_dev->mmdvfs_mux[mmdvfs_dev->mmdvfs_user[idx].target_id];

	if (opp >= mux->freq_num) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "invalid opp:%d idx:%u mux:%u freq_num:%u",
			       opp, idx, mux->id, mux->freq_num);
		return -EINVAL;
	}

	mtk_mmdvfs_enable_vcp(true, VCP_PWR_USR_MMDVFS_FORCE);
	ret = mmdvfs_force_step_by_vcp_ipi(pwr_idx, opp);
	mtk_mmdvfs_enable_vcp(false, VCP_PWR_USR_MMDVFS_FORCE);

	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_DBG, "pwr_idx:%u idx:%u mux:%u opp:%d",
			 pwr_idx, idx, mux->id, opp);

	return ret;
}
EXPORT_SYMBOL_GPL(mmdvfs_force_step_by_vcp);

static int mtk_mmdvfs_v3_set_force_step_ipi(const u16 pwr_idx, const s16 opp)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	int *last, ret;

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return -EINVAL;

	if (pwr_idx >= PWR_MMDVFS_NUM || opp >= MMDVFS_MAX_OPP) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "wrong pwr_idx:%u opp:%d", pwr_idx, opp);
		return -EINVAL;
	}

	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_DBG, "mux_version:%d pwr_idx:%d opp:%d dpsw_thr:%d",
			 mmdvfs_dev->mmdvfs_mux_version, pwr_idx, opp, mmdvfs_dev->dpsw_thr);

	if (mmdvfs_dev->mmdvfs_mux_version)
		return mmdvfs_force_step_by_vcp_ipi(pwr_idx, opp);

	last = &mmdvfs_dev->last_force_step[pwr_idx];

	if (*last == opp || (*last < 0 && opp < 0))
		return 0;

	if (mmdvfs_dev->dpsw_thr > 0 && (*last < 0 || *last >= mmdvfs_dev->dpsw_thr) &&
	    opp >= 0 && opp < mmdvfs_dev->dpsw_thr && pwr_idx == PWR_MMDVFS_VMM)
		mtk_mmdvfs_enable_vmm(true);

	ret = mmdvfs_vcp_ipi_send(mmdvfs_dev, FUNC_FORCE_OPP, pwr_idx,
				  opp <= -1 ? MMDVFS_MAX_OPP : opp, NULL);

	if (mmdvfs_dev->dpsw_thr > 0 && *last >= 0 && *last < mmdvfs_dev->dpsw_thr &&
	    (opp < 0 || opp >= mmdvfs_dev->dpsw_thr) && pwr_idx == PWR_MMDVFS_VMM)
		mtk_mmdvfs_enable_vmm(false);
	*last = opp;

	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_DBG, "pwr_idx:%u opp:%d ret:%d", pwr_idx, opp, ret);
	return ret;
}

int mtk_mmdvfs_v3_set_force_step(const u16 pwr_idx,
				 const s16 opp, const bool requested_by_cmd)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	int *last, ret;

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return -EINVAL;

	if (pwr_idx >= PWR_MMDVFS_NUM || opp >= MMDVFS_MAX_OPP) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "wrong pwr_idx:%u opp:%d", pwr_idx, opp);
		return -EINVAL;
	}

	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_DBG, "mmdvfs_mux_version:%d pwr_idx:%d",
			 mmdvfs_dev->mmdvfs_mux_version, pwr_idx);
	if (requested_by_cmd && mmdvfs_dev->mmdvfs_release_step_done)
		return 0;

	if (mmdvfs_dev->mmdvfs_mux_version)
		return mmdvfs_force_step_by_vcp(pwr_idx, opp);

	last = &mmdvfs_dev->last_force_step[pwr_idx];

	if (*last == opp || (*last < 0 && opp < 0))
		return 0;

	mtk_mmdvfs_enable_vcp(true, VCP_PWR_USR_MMDVFS_FORCE);
	ret = mtk_mmdvfs_v3_set_force_step_ipi(pwr_idx, opp);
	mtk_mmdvfs_enable_vcp(false, VCP_PWR_USR_MMDVFS_FORCE);

	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_DBG, "pwr_idx:%u opp:%d ret:%d", pwr_idx, opp, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(mtk_mmdvfs_v3_set_force_step);

static int mmdvfs_set_force_step(const char *val, const struct kernel_param *kp)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	u16 idx = 0;
	s16 opp;
	int ret;

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return -EINVAL;

	ret = sscanf(val, "%hu %hd", &idx, &opp);
	if (ret != 2 || idx >= PWR_MMDVFS_NUM || opp >= MMDVFS_MAX_OPP ||
	    mmdvfs_dev->mmdvfs_release_step_done) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "input failed:%d idx:%u opp:%d release_step:%d",
			       ret, idx, opp, mmdvfs_dev->mmdvfs_release_step_done);
		return -EINVAL;
	}

	ret = mtk_mmdvfs_v3_set_force_step(idx, opp, true);
	if (ret)
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "failed:%d idx:%u opp:%d", ret, idx, opp);
	return ret;
}

static const struct kernel_param_ops mmdvfs_force_step_ops = {
	.set = mmdvfs_set_force_step,
};
module_param_cb(force_step, &mmdvfs_force_step_ops, NULL, 0644);
MODULE_PARM_DESC(force_step, "force mmdvfs to specified step");

int mmdvfs_force_voltage_by_vcp(const u8 pwr_idx, const s8 opp)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	struct mmdvfs_mux *mux;
	u8 idx = pwr_idx + MMDVFS_USER_VCORE;
	int ret, *last;

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return -EINVAL;

	if (!mmdvfs_dev->mmdvfs_mux_version || idx >= MMDVFS_USER_NUM) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "invalid:%d pwr_idx:%u idx:%u",
			       mmdvfs_dev->mmdvfs_mux_version, pwr_idx, idx);
		return -EINVAL;
	}
	mux = &mmdvfs_dev->mmdvfs_mux[mmdvfs_dev->mmdvfs_user[idx].target_id];

	if (opp >= mux->freq_num) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "invalid opp:%d idx:%u mux:%u freq_num:%u",
			       opp, idx, mux->id, mux->freq_num);
		return -EINVAL;
	}

	last = &mmdvfs_dev->last_force_volt[pwr_idx];
	mtk_mmdvfs_enable_vcp(true, VCP_PWR_USR_MMDVFS_FORCE);
	if (mmdvfs_dev->dpsw_thr && mux->id >= MMDVFS_MUX_VDE && mux->id <= MMDVFS_MUX_CAM &&
	    opp >= 0 && opp < mmdvfs_dev->dpsw_thr && (*last < 0 || *last >= mmdvfs_dev->dpsw_thr))
		mtk_mmdvfs_enable_vmm(true);

	ret = mmdvfs_vcp_ipi_send(mmdvfs_dev, FUNC_FORCE_VOL, pwr_idx, opp, NULL);

	if (mmdvfs_dev->dpsw_thr && mux->id >= MMDVFS_MUX_VDE && mux->id <= MMDVFS_MUX_CAM &&
	    (opp < 0 || opp >= mmdvfs_dev->dpsw_thr) && *last >= 0 && *last < mmdvfs_dev->dpsw_thr)
		mtk_mmdvfs_enable_vmm(false);

	mtk_mmdvfs_enable_vcp(false, VCP_PWR_USR_MMDVFS_FORCE);
	*last = opp;

	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_DBG, "pwr_idx:%u idx:%u mux:%u opp:%d",
			 pwr_idx, idx, mux->id, opp);

	return ret;
}
EXPORT_SYMBOL_GPL(mmdvfs_force_voltage_by_vcp);

int mmdvfs_force_rc_clock_by_vcp(const u8 pwr_idx, const s8 opp)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	struct mmdvfs_mux *mux;
	u8 idx = pwr_idx + MMDVFS_USER_VCORE;
	int ret;

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return -EINVAL;

	if (!mmdvfs_dev->mmdvfs_mux_version || idx >= MMDVFS_USER_NUM) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "invalid:%d pwr_idx:%u idx:%u",
			       mmdvfs_dev->mmdvfs_mux_version, pwr_idx, idx);
		return -EINVAL;
	}
	mux = &mmdvfs_dev->mmdvfs_mux[mmdvfs_dev->mmdvfs_user[idx].target_id];

	if (opp >= mux->freq_num) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "invalid opp:%d idx:%u mux:%u freq_num:%u",
			       opp, idx, mux->id, mux->freq_num);
		return -EINVAL;
	}

	mtk_mmdvfs_enable_vcp(true, VCP_PWR_USR_MMDVFS_FORCE);
	ret = mmdvfs_vcp_ipi_send(mmdvfs_dev, FUNC_FORCE_CLK, pwr_idx, opp, NULL);
	mtk_mmdvfs_enable_vcp(false, VCP_PWR_USR_MMDVFS_FORCE);

	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_DBG, "pwr_idx:%u idx:%u mux:%u opp:%d",
			 pwr_idx, idx, mux->id, opp);

	return ret;
}
EXPORT_SYMBOL_GPL(mmdvfs_force_rc_clock_by_vcp);

int mmdvfs_force_single_clock_by_vcp(const u8 mux_idx, const s8 opp)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	struct mmdvfs_mux *mux;
	int ret;

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return -EINVAL;

	if (!mmdvfs_dev->mmdvfs_mux_version || mux_idx >= MMDVFS_MUX_NUM) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "invalid:%d mux_idx:%u",
			       mmdvfs_dev->mmdvfs_mux_version, mux_idx);
		return -EINVAL;
	}
	mux = &mmdvfs_dev->mmdvfs_mux[mux_idx];

	if (opp >= mux->freq_num) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev,
			       "invalid opp:%d mux_idx:%u mux:%u freq_num:%u",
			       opp, mux_idx, mux->id, mux->freq_num);
		return -EINVAL;
	}

	mtk_mmdvfs_enable_vcp(true, VCP_PWR_USR_MMDVFS_FORCE);
	ret = mmdvfs_vcp_ipi_send(mmdvfs_dev, FUNC_FORCE_SINGLE_CLK, mux->vcp_mux_id, opp, NULL);
	mtk_mmdvfs_enable_vcp(false, VCP_PWR_USR_MMDVFS_FORCE);

	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_DBG, "mux_idx:%u mux:%u opp:%d",
			 mux_idx, mux->id, opp);
	return ret;
}
EXPORT_SYMBOL_GPL(mmdvfs_force_single_clock_by_vcp);

static int mmdvfs_vote_step_by_vcp_ipi(const u8 pwr_idx, const s8 opp)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	struct mmdvfs_mux *mux;
	u8 idx = pwr_idx + MMDVFS_USER_VCORE;
	s8 level;
	int ret, *last;

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return -EINVAL;

	if (!mmdvfs_dev->mmdvfs_mux_version || idx >= MMDVFS_USER_NUM) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "invalid:%d pwr_idx:%u idx:%u",
			       mmdvfs_dev->mmdvfs_mux_version, pwr_idx, idx);
		return -EINVAL;
	}
	mux = &mmdvfs_dev->mmdvfs_mux[mmdvfs_dev->mmdvfs_user[idx].target_id];

	if (opp >= mux->freq_num) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "invalid opp:%d idx:%u mux:%u freq_num:%u",
			       opp, idx, mux->id, mux->freq_num);
		return -EINVAL;
	}
	level = mux->freq_num - 1 - opp;

	last = &mmdvfs_dev->last_vote_step[pwr_idx];
	if (mmdvfs_dev->dpsw_thr && mux->id >= MMDVFS_MUX_VDE && mux->id <= MMDVFS_MUX_CAM &&
	    opp >= 0 && opp < mmdvfs_dev->dpsw_thr && (*last < 0 || *last >= mmdvfs_dev->dpsw_thr))
		mtk_mmdvfs_enable_vmm(true);
	ret = mmdvfs_user_set_rate(idx, mux->freq[level]);
	if (mmdvfs_dev->dpsw_thr && mux->id >= MMDVFS_MUX_VDE && mux->id <= MMDVFS_MUX_CAM &&
	    (opp < 0 || opp >= mmdvfs_dev->dpsw_thr) && *last >= 0 && *last < mmdvfs_dev->dpsw_thr)
		mtk_mmdvfs_enable_vmm(false);
	*last = opp;

	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_DBG, "pwr_idx:%u idx:%u mux:%u opp:%d level:%d",
			 pwr_idx, idx, mux->id, opp, level);

	return ret;
}

int mmdvfs_vote_step_by_vcp(const u8 pwr_idx, const s8 opp)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	struct mmdvfs_mux *mux;
	u8 idx = pwr_idx + MMDVFS_USER_VCORE;
	int ret;

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return -EINVAL;

	if (!mmdvfs_dev->mmdvfs_mux_version || idx >= MMDVFS_USER_NUM) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "invalid:%d pwr_idx:%u idx:%u",
			       mmdvfs_dev->mmdvfs_mux_version, pwr_idx, idx);
		return -EINVAL;
	}
	mux = &mmdvfs_dev->mmdvfs_mux[mmdvfs_dev->mmdvfs_user[idx].target_id];

	if (opp >= mux->freq_num) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "invalid opp:%d idx:%u mux:%u freq_num:%u",
			       opp, idx, mux->id, mux->freq_num);
		return -EINVAL;
	}

	mtk_mmdvfs_enable_vcp(true, VCP_PWR_USR_MMDVFS_VOTE);
	ret = mmdvfs_vote_step_by_vcp_ipi(pwr_idx, opp);
	mtk_mmdvfs_enable_vcp(false, VCP_PWR_USR_MMDVFS_VOTE);

	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_DBG, "pwr_idx:%u idx:%u mux:%u opp:%d",
			 pwr_idx, idx, mux->id, opp);
	return ret;
}
EXPORT_SYMBOL_GPL(mmdvfs_vote_step_by_vcp);

static int mtk_mmdvfs_v3_set_vote_step_ipi(const u16 pwr_idx, const s16 opp)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	struct mtk_mmdvfs_clk *mtk_mmdvfs_clks = mmdvfs_dev->mtk_mmdvfs_clks;
	int i, *last, ret = 0;
	u32 freq;

	if (pwr_idx > PWR_MMDVFS_NUM || opp >= MMDVFS_MAX_OPP) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "failed:%d pwr_idx:%u opp:%d",
			       ret, pwr_idx, opp);
		return -EINVAL;
	}

	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_DBG, "mmdvfs_mux_version:%d ",
			 mmdvfs_dev->mmdvfs_mux_version);

	if (mmdvfs_dev->mmdvfs_mux_version)
		return mmdvfs_vote_step_by_vcp_ipi(pwr_idx, opp);

	if (pwr_idx == PWR_MMDVFS_NUM) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "failed:%d pwr_idx:%u opp:%d",
			       ret, pwr_idx, opp);
		return -EINVAL;
	}

	last = &mmdvfs_dev->last_vote_step[pwr_idx];

	if (*last == opp || (*last < 0 && opp < 0))
		return 0;

	if (mmdvfs_dev->dpsw_thr > 0 && (*last < 0 || *last >= mmdvfs_dev->dpsw_thr) &&
	    opp >= 0 && opp < mmdvfs_dev->dpsw_thr && pwr_idx == PWR_MMDVFS_VMM)
		mtk_mmdvfs_enable_vmm(true);

	for (i = mmdvfs_dev->mmdvfs_clk_num - 1; i >= 0; i--)
		if (pwr_idx == mmdvfs_dev->mtk_mmdvfs_clks[i].pwr_id) {
			if (opp >= mtk_mmdvfs_clks[i].freq_num) {
				mtk_mmdvfs_err(mmdvfs_dev->pdev, "i:%d inval opp:%d freq_num:%u",
					       i, opp, mtk_mmdvfs_clks[i].freq_num);
				break;
			}

			freq = (opp <= -1) ? 0 :
				mtk_mmdvfs_clks[i].freqs[mtk_mmdvfs_clks[i].freq_num - 1 - opp];
			ret = clk_set_rate(mmdvfs_dev->mmdvfs_pwr_clk[pwr_idx], freq);
			break;
		}

	if (mmdvfs_dev->dpsw_thr > 0 && *last >= 0 && *last < mmdvfs_dev->dpsw_thr &&
	    (opp < 0 || opp >= mmdvfs_dev->dpsw_thr) && pwr_idx == PWR_MMDVFS_VMM)
		mtk_mmdvfs_enable_vmm(false);
	*last = opp;

	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_DBG, "pwr_idx:%u opp:%d i:%d freq:%u ret:%d", pwr_idx,
			 opp, i, freq, ret);

	return ret;
}

int mtk_mmdvfs_v3_set_vote_step(const u16 pwr_idx, const s16 opp, const bool requested_by_cmd)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	int *last, ret = 0;

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return -EINVAL;

	if (pwr_idx > PWR_MMDVFS_NUM || opp >= MMDVFS_MAX_OPP) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "failed:%d pwr_idx:%u opp:%d", ret,
			       pwr_idx, opp);
		return -EINVAL;
	}

	if (requested_by_cmd && mmdvfs_dev->mmdvfs_release_step_done)
		return 0;

	if (mmdvfs_dev->mmdvfs_mux_version)
		return mmdvfs_vote_step_by_vcp(pwr_idx, opp);

	if (pwr_idx == PWR_MMDVFS_NUM) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "failed:%d pwr_idx:%u opp:%d", ret,
			       pwr_idx, opp);
		return -EINVAL;
	}

	last = &mmdvfs_dev->last_vote_step[pwr_idx];

	if (*last == opp || (*last < 0 && opp < 0))
		return 0;

	mtk_mmdvfs_enable_vcp(true, VCP_PWR_USR_MMDVFS_VOTE);
	ret = mtk_mmdvfs_v3_set_vote_step_ipi(pwr_idx, opp);
	mtk_mmdvfs_enable_vcp(false, VCP_PWR_USR_MMDVFS_VOTE);

	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_DBG, "pwr_idx:%u opp:%d ret:%d", pwr_idx, opp, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(mtk_mmdvfs_v3_set_vote_step);

static int mmdvfs_set_vote_step(const char *val, const struct kernel_param *kp)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	int ret;
	u16 idx;
	s16 opp;

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return -EINVAL;

	ret = sscanf(val, "%hu %hd", &idx, &opp);
	if (ret != 2 || idx >= PWR_MMDVFS_NUM || opp >= MMDVFS_MAX_OPP ||
	    mmdvfs_dev->mmdvfs_release_step_done) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "failed:%d idx:%u opp:%d release_step:%d",
			       ret, idx, opp, mmdvfs_dev->mmdvfs_release_step_done);
		return -EINVAL;
	}

	ret = mtk_mmdvfs_v3_set_vote_step(idx, opp, true);
	if (ret)
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "failed:%d idx:%u opp:%d", ret, idx, opp);
	return ret;
}

static const struct kernel_param_ops mmdvfs_vote_step_ops = {
	.set = mmdvfs_set_vote_step,
};
module_param_cb(vote_step, &mmdvfs_vote_step_ops, NULL, 0644);
MODULE_PARM_DESC(vote_step, "vote mmdvfs to specified step");

static inline void mmdvfs_reset_clk(bool enable_vcp)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	int i, ret;

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return;

	if (!mmdvfs_dev->mmdvfs_rst_clk_num || mmdvfs_dev->mmdvfs_rst_clk_done)
		return;

	if (enable_vcp)
		mtk_mmdvfs_enable_vcp(true, VCP_PWR_USR_MMDVFS_RST);

	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_RST_CLK, "Before reset clk:");

	for (i = 0; i < mmdvfs_dev->mmdvfs_rst_clk_num; i++) {
		if (!IS_ERR_OR_NULL(mmdvfs_dev->mmdvfs_rst_clk[i])) {
			ret = clk_set_rate(mmdvfs_dev->mmdvfs_rst_clk[i], 0);
			if (ret)
				mtk_mmdvfs_err(mmdvfs_dev->pdev, "reset clk:%d to 0 failed:%d",
					       i, ret);
		}
	}

	mmdvfs_dev->mmdvfs_rst_clk_done = true;
	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_RST_CLK, "After reset clk:");

	if (enable_vcp)
		mtk_mmdvfs_enable_vcp(false, VCP_PWR_USR_MMDVFS_RST);
}

static inline void mmdvfs_reset_vcp(struct mtk_mmdvfs_dev *mmdvfs_dev)
{
	int i;

	mmdvfs_reset_clk(false);

	for (i = 0; i < VCP_PWR_USR_NUM; i++) {
		if (mmdvfs_dev->vcp_pwr_usage[i])
			mtk_mmdvfs_err(mmdvfs_dev->pdev, "i:%d usage:%d not disable",
				       i, mmdvfs_dev->vcp_pwr_usage[i]);
	}
}

static void mmdvfs_v3_release_step(struct mtk_mmdvfs_dev *mmdvfs_dev, bool enable_vcp)
{
	int last, i;
	void __iomem *vcore_check_rg = mmdvfs_dev->vcore_check_rg;

	if (enable_vcp)
		mtk_mmdvfs_enable_vcp(true, VCP_PWR_USR_MMDVFS_VOTE);

	for (i = 0; i < PWR_MMDVFS_NUM; i++) {
		if (mmdvfs_dev->last_vote_step[i] != -1) {
			last = mmdvfs_dev->last_vote_step[i];
			mtk_mmdvfs_v3_set_vote_step_ipi(i, -1);
			mmdvfs_dev->last_vote_step[i] = last;
		}

		if (mmdvfs_dev->last_force_step[i] != -1) {
			if (i == PWR_MMDVFS_VCORE && vcore_check_rg &&
			    (readl(vcore_check_rg) & (0x1 << mmdvfs_dev->vcore_check_offset))) {
				mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_DBG,
						 "skip release vcore force");
				continue;
			}

			last = mmdvfs_dev->last_force_step[i];
			mtk_mmdvfs_v3_set_force_step_ipi(i, -1);
			mmdvfs_dev->last_force_step[i] = last;
		}
	}

	if (enable_vcp)
		mtk_mmdvfs_enable_vcp(false, VCP_PWR_USR_MMDVFS_VOTE);

	if (mmdvfs_dev->vmm_power) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "enable:%d vmm_power:%d should be zero",
			       false, mmdvfs_dev->vmm_power);
		if (mmdvfs_dev->vmm_power > 1) {
			mutex_lock(&mmdvfs_vmm_pwr_mutex);
			mmdvfs_dev->vmm_power = 1;
			mutex_unlock(&mmdvfs_vmm_pwr_mutex);
		}
		mtk_mmdvfs_enable_vmm(false);
	}
}

static void mmdvfs_v3_restore_step(struct mtk_mmdvfs_dev *mmdvfs_dev)
{
	int i, last;

	for (i = 0; i < PWR_MMDVFS_NUM; i++) {
		if (mmdvfs_dev->last_force_step[i] != -1) {
			last = mmdvfs_dev->last_force_step[i];
			mmdvfs_dev->last_force_step[i] = -1;
			mtk_mmdvfs_v3_set_force_step_ipi(i, last);
		}

		if (mmdvfs_dev->last_vote_step[i] != -1) {
			last = mmdvfs_dev->last_vote_step[i];
			mmdvfs_dev->last_vote_step[i] = -1;
			mtk_mmdvfs_v3_set_vote_step_ipi(i, last);
		}
	}
}

void mmdvfs_vcp_cb_mutex_lock(void)
{
	mutex_lock(&mmdvfs_vcp_cb_mutex);
}
EXPORT_SYMBOL_GPL(mmdvfs_vcp_cb_mutex_lock);

void mmdvfs_vcp_cb_mutex_unlock(void)
{
	mutex_unlock(&mmdvfs_vcp_cb_mutex);
}
EXPORT_SYMBOL_GPL(mmdvfs_vcp_cb_mutex_unlock);

bool mmdvfs_vcp_cb_ready_get(void)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();

	return (IS_ERR_OR_NULL(mmdvfs_dev))? false: mmdvfs_dev->mmdvfs_vcp_cb_ready;
}
EXPORT_SYMBOL_GPL(mmdvfs_vcp_cb_ready_get);

static int mmdvfs_mmup_notifier_callback(struct notifier_block *nb, unsigned long action,
					 void *data)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	struct mtk_vcp_device  *vcp_device = NULL;
	struct mmdvfs_mux *mmdvfs_mux = NULL;
	static bool sram_init;
	int target_id;

	if (IS_ERR_OR_NULL(mmdvfs_dev)) {
		pr_err("@@@@@@@@@ mmdvfs_dev is NULL!!!!\n");
		return -EINVAL;
	}

	vcp_device = mmdvfs_dev->vcp_device;
	mmdvfs_mux = mmdvfs_dev->mmdvfs_mux;

	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_DBG, "action:%ld dpb_fp:%d sram:%d", action,
			 mmdvfs_dev->dpc_fp ? 1 : 0, mmdvfs_dev->mmdvfs_mmup_sram);
	switch (action) {
	case VCP_EVENT_READY:
		mmdvfs_dev->cb_timestamp[2] = sched_clock();
		mmdvfs_dev->mmdvfs_rst_clk_done = false;
		mmdvfs_dev->mmdvfs_release_step_done = false;
		mmdvfs_dev->mmdvfs_init_done = true;
		mmdvfs_vcp_ipi_send(mmdvfs_dev, FUNC_MMDVFS_INIT, MMDVFS_MAX_OPP,
				    MMDVFS_MAX_OPP, NULL);
		if (mmdvfs_dev->dpc_fp)
			mmdvfs_dev->dpc_fp(true, mmdvfs_dev->mmdvfs_vcp_stop);
		mmdvfs_dev->mmdvfs_vcp_stop = false;
		mmdvfs_vcp_ipi_send(mmdvfs_dev, FUNC_MMDVFSRC_INIT, MMDVFS_MAX_OPP,
				    MMDVFS_MAX_OPP, NULL);
		if (mmdvfs_dev->mmdvfs_mmup_sram && unlikely(!sram_init)) {
			mmdvfs_dev->mmdvfs_mmup_sram_va =
				vcp_device->data->vcp_get_sram_virt(vcp_device) +
								    readl(MEM_SRAM_OFFSET);
			sram_init = true;
			mtk_mmdvfs_err(mmdvfs_dev->pdev, "sram_init:%d offset:%x va:%lx",
				       sram_init, readl(MEM_SRAM_OFFSET),
				       (unsigned long)mmdvfs_dev->mmdvfs_mmup_sram_va);
		}
		mutex_lock(&mmdvfs_vcp_cb_mutex);
		mmdvfs_dev->mmdvfs_vcp_cb_ready = true;
		mutex_unlock(&mmdvfs_vcp_cb_mutex);
		if (mmdvfs_dev->hqa_enable)
			mtk_mmdvfs_enable_vmm(true);

		break;
	case VCP_EVENT_RESUME:
		if (mmdvfs_dev->mmdvfs_restore_step)
			mmdvfs_v3_restore_step(mmdvfs_dev);
		break;
	case VCP_EVENT_STOP:
		if (mmdvfs_dev->dpc_fp)
			mmdvfs_dev->dpc_fp(false, true);
		mmdvfs_dev->mmdvfs_vcp_stop = true;
		mutex_lock(&mmdvfs_vcp_cb_mutex);
		mmdvfs_dev->mmdvfs_vcp_cb_ready = false;
		mutex_unlock(&mmdvfs_vcp_cb_mutex);
		break;
	case VCP_EVENT_SUSPEND:
		mmdvfs_dev->mmdvfs_release_step_done = true;
		mmdvfs_v3_release_step(mmdvfs_dev, false);

		if (mmdvfs_dev->dpc_fp)
			mmdvfs_dev->dpc_fp(false, false);

		if (mmdvfs_dev->hqa_enable) {
			/* set vmm to lowest step for HQA test */
			target_id = mmdvfs_dev->mmdvfs_user[0].target_id;
			mmdvfs_force_step_by_vcp(1,
						 mmdvfs_mux[target_id].freq_num - 1);
			mtk_mmdvfs_enable_vmm(false);
		}
		mmdvfs_dev->cb_timestamp[1] = sched_clock();
		mmdvfs_reset_vcp(mmdvfs_dev);
		mutex_lock(&mmdvfs_vcp_cb_mutex);
		mmdvfs_dev->mmdvfs_vcp_cb_ready = false;
		mutex_unlock(&mmdvfs_vcp_cb_mutex);
		break;
	}

	return NOTIFY_DONE;
}

static int mmdvfs_vcp_init_thread(void *data)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = data;
	struct platform_device *pdev = mmdvfs_dev->pdev;
	struct mtk_vcp_device  *vcp_device = NULL;
	struct platform_device *vcp_pdev = NULL;
	struct device_node *np;
	phandle vcp_phandle;
	int i, retry = 0;

	mmdvfs_dev->vcp_device = NULL;
	while (mmdvfs_dev->vcp_device == NULL) {
		if (++retry > MMDVFS_REQUEST_VCP_MAX_COUNT) {
			mtk_mmdvfs_err(pdev, "failed to load mtk-vcp module");
			return -ENODEV;
		}

		if (of_property_read_u32(pdev->dev.of_node, "mediatek,vcp", &vcp_phandle))
			continue;

		mmdvfs_dev->vcp_device = mtk_vcp_get_by_phandle(vcp_phandle);

		ssleep(1);
	}
	vcp_device = mmdvfs_dev->vcp_device;
	pr_err("mmdvfs: VCP device found!\n");

	retry = 0;
	while (!vcp_device->data->vcp_is_ready(MMDVFS_MMUP_FEATURE_ID) ||
	       !vcp_device->data->vcp_is_ready(MMDVFS_VCP_FEATURE_ID)) {
		if (++retry > MMDVFS_MAX_RETRY_COUNT) {
			mtk_mmdvfs_err(pdev, "vcp and mmup is not ready yet");
			return -ETIMEDOUT;
		}
		pr_err("mmdvfs: waiting for vcp_is_ready...\n");
		ssleep(1);
	}
	pr_err("mmdvfs: VCP is ready!\n");


#if IS_ENABLED(CONFIG_MTK_MMDEBUG)
	retry = 0;
	while (!mmdebug_is_init_done()) {
		if (++retry > MMDVFS_MAX_RETRY_COUNT) {
			mtk_mmdvfs_err(pdev, "mmdebug is not ready yet");
			return -ETIMEDOUT;
		}
		pr_err("mmdvfs: waiting for mmdebug...\n");
		ssleep(1);
	}
	pr_err("mmdvfs: mmdebug init is done!\n");
#endif
	retry = 0;
	while (mtk_mmdvfs_enable_vcp(true, VCP_PWR_USR_MMDVFS_INIT)) {
		if (++retry > MMDVFS_MAX_RETRY_COUNT) {
			mtk_mmdvfs_err(pdev, "vcp is not powered on yet");
			return -ETIMEDOUT;
		}
		pr_err("mmdvfs: waiting for mtk_mmdvfs_enable_vcp...\n");
		ssleep(1);
	}
	pr_err("mmdvfs: VCP enabled!\n");

	mmdvfs_dev->mmdvfs_mmup_iova = vcp_device->data->vcp_get_mem_iova(MMDVFS_MMUP_MEM_ID);
	mmdvfs_dev->mmdvfs_mmup_va = (void *)vcp_device->data->vcp_get_mem_virt(MMDVFS_MMUP_MEM_ID);
	pr_err("mmdvfs: MMUP memory done!\n");

	mmdvfs_dev->mmdvfs_vcp_iova = vcp_device->data->vcp_get_mem_iova(MMDVFS_VCP_MEM_ID);
	mmdvfs_dev->mmdvfs_vcp_va = (void *)vcp_device->data->vcp_get_mem_virt(MMDVFS_VCP_MEM_ID);
	pr_err("mmdvfs: VCP memory done!\n");

	writel_relaxed(mmdvfs_dev->mmdvfs_free_run ? 1 : 0, MEM_FREERUN);
	for (i = 0; i < PWR_MMDVFS_NUM; i++) {
		writel_relaxed(MMDVFS_MAX_OPP, MEM_FORCE_OPP_PWR(i));
		writel_relaxed(MMDVFS_MAX_OPP, MEM_VOTE_OPP_PWR(i));
		writel_relaxed(MMDVFS_MAX_OPP, MEM_PWR_OPP(i));
		writel_relaxed(MMDVFS_MAX_OPP, MEM_PWR_CUR_GEAR(i));
		writel_relaxed(MMDVFS_MAX_OPP, MEM_FORCE_VOL(i));
		writel_relaxed(UINT_MAX, MEM_CEIL_LEVEL(i));
	}
	for (i = 0; i < USER_NUM; i++) {
		writel_relaxed(MMDVFS_MAX_OPP, MEM_VOTE_OPP_USR(i));
		writel_relaxed(MMDVFS_MAX_OPP, MEM_MUX_OPP(i));
		writel_relaxed(MMDVFS_MAX_OPP, MEM_MUX_MIN(i));
		writel_relaxed(MMDVFS_MAX_OPP, MEM_FORCE_CLK(i));
	}
	for (i = 0; i < MMDVFS_USER_NUM; i++) {
		writel_relaxed(MMDVFS_MAX_OPP, MEM_USR_OPP(i, false));
		writel_relaxed(MMDVFS_MAX_OPP, MEM_USR_OPP(i, true));
	}
	pr_err("mmdvfs: Votes write OK!\n");

	if (mmdvfs_dev->mmdvfs_lp_mode)
		writel_relaxed(1, MEM_MMDVFS_LP_MODE);

//	mtk_mmdvfs_debug(pdev, LOG_DBG, "mmup: iova:%pa va:%#lx vcp: iova:%pa va:%#lx init_done:%d",
	mtk_mmdvfs_err(pdev, "mmup: iova:%pa va:%#lx vcp: iova:%pa va:%#lx init_done:%d",
			 &mmdvfs_dev->mmdvfs_mmup_iova, (unsigned long)mmdvfs_dev->mmdvfs_mmup_va,
			 &mmdvfs_dev->mmdvfs_vcp_iova, (unsigned long)mmdvfs_dev->mmdvfs_vcp_va,
			 mmdvfs_dev->mmdvfs_init_done);

	mtk_mmdvfs_debug(pdev, LOG_DBG, "log level:%d %d %d", mmdvfs_dev->vcp_log_level,
			 mmdvfs_dev->vmrc_log_level, mmdvfs_dev->vmm_ceil_step);

	if (mmdvfs_dev->vcp_log_level)
		mtk_mmdvfs_v3_set_vcp_log(mmdvfs_dev->vcp_log_level);

	if (mmdvfs_dev->vmrc_log_level)
		mtk_mmdvfs_v3_set_vmrc_log(mmdvfs_dev->vmrc_log_level);

	if (mmdvfs_dev->vmm_ceil_step)
		mtk_mmdvfs_v3_set_vmm_ceil_step(mmdvfs_dev->vmm_ceil_step);

	vcp_device->data->vcp_register_feature(vcp_device, MMDVFS_HFRP_FEATURE_ID);
	pr_err("mmdvfs: Registered MMUP HFRP feature\n");

	mmdvfs_dev->mmdvfs_mmup_notifier.notifier_call = mmdvfs_mmup_notifier_callback;
	vcp_device->data->vcp_register_notify(MMDVFS_HFRP_FEATURE_ID,
					      &mmdvfs_dev->mmdvfs_mmup_notifier);
	pr_err("mmdvfs: Registered MMUP HFRP notification\n");

	if (mmdvfs_dev->force_vol != 0xff)
		mmdvfs_force_voltage_by_vcp(mmdvfs_dev->force_vol >> 4 & 0xf,
					    mmdvfs_dev->force_vol & 0xf);

	if (mmdvfs_dev->force_rc_clk != 0xff)
		mmdvfs_force_rc_clock_by_vcp(mmdvfs_dev->force_rc_clk >> 4 & 0xf,
					     mmdvfs_dev->force_rc_clk & 0xf);

	if (mmdvfs_dev->force_single_clk != 0xff)
		mmdvfs_force_single_clock_by_vcp(mmdvfs_dev->force_single_clk >> 4 & 0xf,
						 mmdvfs_dev->force_single_clk & 0xf);

	mtk_mmdvfs_debug(pdev, LOG_DBG, "clk level:%d %d %d", mmdvfs_dev->force_vol,
			 mmdvfs_dev->force_rc_clk, mmdvfs_dev->force_single_clk);

	return 0;
}

int mmdvfs_mux_set_opp(const char *name, unsigned long rate)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	struct mtk_mux_user *user;
	struct mmdvfs_mux *mux;
	int i;
	u64 ns = sched_clock(), sec = ns / 1000000000, usec = (ns / 1000) % 1000000;

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return -EINVAL;

	for (i = 0; i < MMDVFS_USER_NUM; i++)
		if (!strncmp(mmdvfs_dev->mmdvfs_user[i].name, name, 16))
			break;

	if (i >= MMDVFS_USER_NUM) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "invalid name:%s rate:%lu", name, rate);
		return -EINVAL;
	}

	user = &mmdvfs_dev->mmdvfs_user[i];
	mux = &mmdvfs_dev->mmdvfs_mux[user->target_id];

	if (user->undo_rate <= MMDVFS_DEFAULT_RATE && user->rate == rate)
		return 0;

	user->rate = rate;
	mux->rate = 0ULL;

	for (i = 0; i < mux->user_num; i++) {
		if (mux->rate < mux->user[i]->rate)
			mux->rate = mux->user[i]->rate;
	}

	for (i = 0; i < mux->freq_num; i++) {
		if (mux->rate <= mux->freq[i])
			break;
	}

	mux->opp = (mux->freq_num - ((i == mux->freq_num) ? (i - 1) : i) - 1);

	if (MEM_BASE) {
		writel_relaxed(rate, MEM_USR_FREQ(user->id, false));
		writel_relaxed(mux->opp, MEM_USR_OPP(user->id, false));
		writel_relaxed(sec, MEM_USR_OPP_SEC(user->id, false));
		writel_relaxed(usec, MEM_USR_OPP_USEC(user->id, false));
	}

	if (mux->opp == mux->last)
		return 0;

	user->undo_rate = 0UL;
	mux->last = mux->opp;

	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_CLK_OPS,
		       "mux:%u name:%s rate:%llu freq_num:%u opp:%d last:%d",
		       mux->id, mux->name, mux->rate, mux->freq_num, mux->opp, mux->last);

	return 0;
}
EXPORT_SYMBOL(mmdvfs_mux_set_opp);

static int mmdvfs_mux_get_opp(const char *name)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	u8 id = MMDVFS_MUX_NUM, opp = 0xff;
	int i;

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return -EINVAL;

	for (i = 0; i < MMDVFS_MUX_NUM; i++) {
		if (!strncmp(mmdvfs_dev->mmdvfs_mux[i].target_name, name, 16))
			if (mmdvfs_dev->mmdvfs_mux[i].opp < opp) {
				id = i;
				opp = mmdvfs_dev->mmdvfs_mux[i].opp;
			}
	}

	if (id >= MMDVFS_MUX_NUM) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "invalid name:%s id:%d", name, id);
		return -EINVAL;
	}

	if (mmdvfs_dev->mmdvfs_vcp_stop) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "vote opp after vcp stop, name:%s id:%d",
			       mmdvfs_dev->mmdvfs_mux[id].name, id);
		return -EINVAL;
	}

	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_CLK_OPS,
			 "name:%s opp:%d mux:%d name:%s rate:%llu opp:%d level:%u",
			 name, opp, id, mmdvfs_dev->mmdvfs_mux[id].name,
			 mmdvfs_dev->mmdvfs_mux[id].rate,
			 mmdvfs_dev->mmdvfs_mux[id].opp,
			 opp_to_level(mmdvfs_dev->mmdvfs_mux, id, opp));

	return opp_to_level(mmdvfs_dev->mmdvfs_mux, id, opp);
}

static unsigned long mmdvfs_mux_get_rate(const char *name)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev = mtk_mmdvfs_get_drv_data();
	int i;

	if (IS_ERR_OR_NULL(mmdvfs_dev))
		return -EINVAL;

	for (i = 0; i < MMDVFS_USER_NUM; i++) {
		if (!strncmp(mmdvfs_dev->mmdvfs_user[i].name, name, 16))
			break;
	}

	if (i >= MMDVFS_USER_NUM) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "invalid name:%s", name);
		return -EINVAL;
	}

	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_CLK_OPS, "name:%s user:%d name:%s rate:%lu",
			 name, mmdvfs_dev->mmdvfs_user[i].id,
			 mmdvfs_dev->mmdvfs_user[i].name,
			 mmdvfs_dev->mmdvfs_user[i].rate);

	return mmdvfs_dev->mmdvfs_user[i].rate;
}

static struct dfs_ops mmdvfs_mux_dfs_ops = {
	.set_opp = mmdvfs_mux_set_opp,
	.get_opp = mmdvfs_mux_get_opp,
	.get_rate = mmdvfs_mux_get_rate,
};

static struct clk_hw_onecell_data *mmdvfs_alloc_clk_data(struct platform_device *pdev,
							 unsigned int clk_num)
{
	struct clk_hw_onecell_data *clk_data;
	int i;

	clk_data = devm_kzalloc(&pdev->dev, struct_size(clk_data, hws, clk_num), GFP_KERNEL);
	if (!clk_data)
		return NULL;

	clk_data->num = clk_num;

	for (i = 0; i < clk_num; i++)
		clk_data->hws[i] = ERR_PTR(-ENOENT);

	return clk_data;
}

static int mmdvfs_mux_probe(struct platform_device *pdev)
{
	struct mtk_mmdvfs_dev *mmdvfs_dev;
	struct device_node *node = pdev->dev.of_node, *larb;
	struct platform_device *larb_pdev;
	struct clk_hw_onecell_data *data;
	struct task_struct *kthr_vcp;
	u32 vcp_mux_id, user_target_id;
	const char *name = NULL;
	int i, j, ret, ret_read;
	u8 user_num;
	u32 val;

	mmdvfs_dev = devm_kzalloc(&pdev->dev, sizeof(*mmdvfs_dev), GFP_KERNEL);
	if (!mmdvfs_dev)
		return -ENOMEM;

	mmdvfs_dev->pdev = pdev;
	platform_set_drvdata(pdev, mmdvfs_dev);

	mmdvfs_dev->mmdvfs_mux_version = true;
	mmdvfs_dev->mmup_ena = true;
	mmdvfs_dev->mmdvfs_free_run = of_property_read_bool(node, "mediatek,free-run");
	ret_read = of_property_read_s32(node, "mediatek,dpsw-thres", &mmdvfs_dev->dpsw_thr);
	if (ret_read < 0) {
		mtk_mmdvfs_err(mmdvfs_dev->pdev, "Failed to read dpsw-thres, %d\n", ret_read);
		mmdvfs_dev->dpsw_thr = DEFAULT_DPSW_THR_VALUE;
	}
	mmdvfs_dev->mmdvfs_restore_step = of_property_read_bool(node, "mediatek,restore-step");
	mmdvfs_dev->mmdvfs_mmup_sram = of_property_read_bool(node, "mediatek,mmup-sram");

	mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_DBG,
			 "version:%d free_run:%d dpsw_thr:%d restore_step:%d mmup_sram:%d",
			 mmdvfs_dev->mmdvfs_mux_version, mmdvfs_dev->mmdvfs_free_run,
			 mmdvfs_dev->dpsw_thr,
			 mmdvfs_dev->mmdvfs_restore_step, mmdvfs_dev->mmdvfs_mmup_sram);

	larb = of_parse_phandle(pdev->dev.of_node, "mediatek,vdec-larb", 0);
	if (larb) {
		mmdvfs_dev->mmdvfs_v3_dev = &pdev->dev;
		larb_pdev = of_find_device_by_node(larb);
		if (!device_link_add(mmdvfs_dev->mmdvfs_v3_dev, &larb_pdev->dev,
				     DL_FLAG_PM_RUNTIME | DL_FLAG_STATELESS)) {
			mtk_mmdvfs_err(pdev, "device_link_add larb failed");
			of_node_put(larb);
			return -EINVAL;
		}
		pm_runtime_enable(mmdvfs_dev->mmdvfs_v3_dev);
		of_node_put(larb);
	}

	if (!of_property_read_u32(pdev->dev.of_node, "mediatek,vcore-check-rg", &val)) {
		mmdvfs_dev->vcore_check_rg = ioremap(val, 4);
		if (of_property_read_u32(pdev->dev.of_node, "mediatek,vcore-check-offset",
					 &mmdvfs_dev->vcore_check_offset))
			mtk_mmdvfs_err(pdev, "mediatek,vcore-check-offset missing");
	}

	ret = of_property_count_u8_elems(pdev->dev.of_node, "mediatek,vcore-level");
	if (ret > 0) {
		mmdvfs_dev->vcore_level_count = ret;
		mmdvfs_dev->vcore_level = kcalloc(mmdvfs_dev->vcore_level_count,
						  sizeof(*mmdvfs_dev->vcore_level), GFP_KERNEL);
		if (!mmdvfs_dev->vcore_level)
			return -ENOMEM;

		ret = of_property_read_u8_array(pdev->dev.of_node, "mediatek,vcore-level",
						mmdvfs_dev->vcore_level,
						mmdvfs_dev->vcore_level_count);
		if (ret)
			mtk_mmdvfs_err(mmdvfs_dev->pdev, "get vcore level failed");
	}

	for (i = 0; i < MMDVFS_MUX_NUM; i++) {
		struct device_node *table, *opp = NULL;
		struct clk *clk;
		phandle handle;
		u64 freq;

		mmdvfs_dev->mmdvfs_mux[i].id = i;

		ret = of_property_read_string_index(node, "clock-names", i, &name);
		if (ret) {
			mtk_mmdvfs_err(pdev, "failed:%d i:%d name:%s", ret, i, name);
			return ret;
		}
		mmdvfs_dev->mmdvfs_mux[i].name = name;

		clk = of_clk_get(node, i);
		if (IS_ERR_OR_NULL(clk)) {
			mtk_mmdvfs_err(pdev, "failed:%d i:%d", PTR_ERR_OR_ZERO(clk), i);
			return PTR_ERR_OR_ZERO(clk);
		}

		name = __clk_get_name(clk);
		if (!name) {
			mtk_mmdvfs_err(pdev, "failed:%d name:%s clk:%p", PTR_ERR_OR_ZERO(name),
				       name, clk);
			return PTR_ERR_OR_ZERO(clk);
		}
		clk_put(clk);
		mmdvfs_dev->mmdvfs_mux[i].target_name = name;

		ret = of_property_read_u32_index(node, "mediatek,mmdvfs-opp-table", i, &handle);
		if (ret) {
			mtk_mmdvfs_err(pdev, "failed:%d i:%d handle:%u", ret, i, handle);
			return ret;
		}

		table = of_find_node_by_phandle(handle);
		if (!table)
			return -EINVAL;

		j = 0;
		do {
			opp = of_get_next_available_child(table, opp);
			if (opp) {
				ret = of_property_read_u64(opp, "opp-hz", &freq);
				if (ret) {
					mtk_mmdvfs_err(pdev,
						       "failed:%d i:%d freq:%llu", ret, i, freq);
					return ret;
				}
				mmdvfs_dev->mmdvfs_mux[i].freq[j] = freq;
				j += 1;
				if (j >= MMDVFS_MAX_OPP)
					break;
			}
		} while (opp);
		of_node_put(table);

		mmdvfs_dev->mmdvfs_mux[i].freq_num = j;
		mmdvfs_dev->mmdvfs_mux[i].rate = MMDVFS_DEFAULT_RATE;
		mmdvfs_dev->mmdvfs_mux[i].opp = MMDVFS_MAX_OPP;
		mmdvfs_dev->mmdvfs_mux[i].last = MMDVFS_MAX_OPP;

		ret = of_property_read_u32_index(node, "mediatek,mmdvfs-vcp-mux-id",
						 i, &vcp_mux_id);
		if (ret) {
			mtk_mmdvfs_err(pdev, "failed:%d i:%d vcp_mux_id:%d", ret, i, vcp_mux_id);
			return ret;
		}
		mmdvfs_dev->mmdvfs_mux[i].vcp_mux_id = vcp_mux_id;
	}

	for (i = 0; i < MMDVFS_USER_NUM; i++) {
		mmdvfs_dev->mmdvfs_user[i].id = i;

		ret = of_property_read_string_index(node, "mediatek,mmdvfs-user-names", i, &name);
		if (ret) {
			mtk_mmdvfs_err(pdev, "failed:%d i:%d name:%s", ret, i, name);
			return ret;
		}
		mmdvfs_dev->mmdvfs_user[i].name = name;

		ret = of_property_read_u32_index(node, "mediatek,mmdvfs-user-target",
						 i, &user_target_id);
		if (ret || user_target_id >= MMDVFS_MUX_NUM) {
			mtk_mmdvfs_err(pdev, "failed:%d i:%d user_target_id:%d",
				       ret, i, user_target_id);
			return ret;
		}
		mmdvfs_dev->mmdvfs_user[i].target_name =
			mmdvfs_dev->mmdvfs_mux[user_target_id].name;
		mmdvfs_dev->mmdvfs_user[i].target_id = user_target_id;

		user_num = mmdvfs_dev->mmdvfs_mux[user_target_id].user_num;
		mmdvfs_dev->mmdvfs_mux[user_target_id].user[user_num] =
			&mmdvfs_dev->mmdvfs_user[i];
		mmdvfs_dev->mmdvfs_mux[user_target_id].user_num += 1;

		mmdvfs_dev->mmdvfs_user[i].rate = MMDVFS_DEFAULT_RATE;
		mmdvfs_dev->mmdvfs_user[i].undo_rate = MMDVFS_DEFAULT_RATE;
		mmdvfs_dev->mmdvfs_user[i].ops = &mtk_mux_user_ops;
		mmdvfs_dev->mmdvfs_user[i].flags = 0;

		mtk_mmdvfs_debug(mmdvfs_dev->pdev, LOG_DBG,
				 "user:%u %s target:%s mux:%u name:%s target:%s freq:%u num:%u",
				 mmdvfs_dev->mmdvfs_user[i].id, mmdvfs_dev->mmdvfs_user[i].name,
				 mmdvfs_dev->mmdvfs_user[i].target_name,
				 mmdvfs_dev->mmdvfs_mux[user_target_id].id,
				 mmdvfs_dev->mmdvfs_mux[user_target_id].name,
				 mmdvfs_dev->mmdvfs_mux[user_target_id].target_name,
				 mmdvfs_dev->mmdvfs_mux[user_target_id].freq_num,
				 mmdvfs_dev->mmdvfs_mux[user_target_id].user_num);
	}

	data = mmdvfs_alloc_clk_data(pdev, MMDVFS_USER_NUM);
	if (!data)
		return -ENOMEM;

	ret = mtk_clk_mux_register_user_clks(mmdvfs_dev->mmdvfs_user, MMDVFS_USER_NUM,
					     &mmdvfs_mux_lock, data, &pdev->dev);
	if (ret) {
		mtk_mmdvfs_err(pdev, "failed:%d user:%p size:%d data:%p",
			       ret, mmdvfs_dev->mmdvfs_user, MMDVFS_USER_NUM, data);
		return ret;
	}

	ret = of_clk_add_hw_provider(node, of_clk_hw_onecell_get, data);
	if (ret) {
		mtk_mmdvfs_err(pdev, "failed:%d data:%p", ret, data);
		return ret;
	}
	mtk_clk_mux_register_callback(&mmdvfs_mux_dfs_ops);

	if (!mmdvfs_dev->vmm_notify_wq)
		mmdvfs_dev->vmm_notify_wq = create_singlethread_workqueue("vmm_notify_wq");

	for (i = 0; i < PWR_MMDVFS_NUM; i++) {
		mmdvfs_dev->last_vote_step[i] = -1;
		mmdvfs_dev->last_force_step[i] = -1;
		mmdvfs_dev->last_force_volt[i] = -1;
	}

	ret_read = of_property_read_s32(node, "kernel-log-level", &mmdvfs_log_level);
	if (ret_read < 0)
		mmdvfs_log_level = 0;
	of_property_read_s32(node, "vcp-log-level", &mmdvfs_dev->vcp_log_level);
	of_property_read_s32(node, "vmrc-log-level", &mmdvfs_dev->vmrc_log_level);
	of_property_read_s32(node, "vmm-ceil-step", &mmdvfs_dev->vmm_ceil_step);
	mmdvfs_dev->force_vol = 0xff;
	of_property_read_u32(node, "force-voltage", &mmdvfs_dev->force_vol);
	mmdvfs_dev->force_rc_clk = 0xff;
	of_property_read_u32(node, "force-rc-clk", &mmdvfs_dev->force_rc_clk);
	mmdvfs_dev->force_single_clk = 0xff;
	of_property_read_u32(node, "force-single-clk", &mmdvfs_dev->force_single_clk);

	dev_dbg(&pdev->dev, "prob successfully.");
	kthr_vcp = kthread_run(mmdvfs_vcp_init_thread, mmdvfs_dev, "mmdvfs-vcp");
	if (IS_ERR(kthr_vcp))
		mtk_mmdvfs_err(pdev, "create kthread mmdvfs_vcp_init_thread failed");

	return 0;
}

static const struct of_device_id of_match_mmdvfs_mux[] = {
	{
		.compatible = "mediatek,mtk-mmdvfs-mux",
	},
	{}
};

static struct platform_driver mmdvfs_mux_drv = {
	.probe = mmdvfs_mux_probe,
	.driver = {
		.name = "mtk-mmdvfs-mux",
		.of_match_table = of_match_mmdvfs_mux,
	},
};

module_platform_driver(mmdvfs_mux_drv)

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MediaTek MMDVFS");
MODULE_AUTHOR("MediaTek Inc.");
