// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 MediaTek Inc.
 */

#include <linux/device.h>
#include <linux/kconfig.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>

#include "slbc.h"
#include "slbc_ops.h"
#include "slbc_ipi.h"
#include "mtk_slbc_sram.h"
#include <mtk_heap.h>

#include <linux/slab.h>
#include <linux/pm_qos.h>
#include <linux/cpuidle.h>
#include <linux/wait.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/scmi_protocol.h>

#include <tinysys-scmi.h>

#include <linux/arm-smccc.h>    /* for Kernel Native SMC API */
#include <linux/soc/mediatek/mtk_sip_svc.h> /* for SMC ID table */

#define MTK_SLBC_KERNEL_OP_CPU_DCC 0

#define slbc_smc_send(_opid, _val1, _val2)                   \
({                                                           \
	struct arm_smccc_res res;                            \
	arm_smccc_smc(MTK_SIP_KERNEL_SLBC_CONTROL,           \
		      _opid, _val1, _val2, 0, 0, 0, 0, &res);\
	res.a0;                                              \
})

#define SLBC_WAY_A_BASE			0x0f000000
#define SLBC_WAY_B_BASE			0x800000000
#define SLBC_PADDR_MASK			0x00ffffff
#define SLBC_UID_VALID(uid)		(((uid) > UID_ZERO) && ((uid) < UID_MAX))
#define SLBC_SID_VALID(sid)		((sid) < ARRAY_SIZE(p_config))
#define SLBC_UID_STR(uid)		((SLBC_UID_VALID(uid)) ? slbc_uid_str[uid] : "UID_ERROR")
#define SLBC_CHECK_TIME			msecs_to_jiffies(1000)
#define SLBC_CHECK_TIMEOUT		msecs_to_jiffies(5000)
#define SLBC_TIMEOUT_LIMIT		500

#define GID_MAX					64
#define GID_REQ					(-1)

enum slc_gid_list {
	GID_GPU = 7,
	GID_GPU_OVL,
	GID_VDEC_FRAME,
	GID_VDEC_UBE,
	GID_SMMU_0,
	GID_SMMU_1,
	GID_SMMU_2,
	GID_SMMU_3,
	GID_MD,
	GID_ADSP,
	GID_AOV,
	GID_IMG_SMT,
	GID_IMG_MCNR,
	GID_CAM,
	GID_MAE,
	GID_DMR,
	GID_OD,
	GID_DBI,
};

#define BUF_ID_NOT_CARE			0x00000000
#define BUF_ID_GPU				0x0000000f
#define BUF_ID_OVL				0x000000f0
#define BUF_ID_VDEC				0x00000f00

static struct mtk_slbc *slbc;

static int venc_count;

#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCMI)
static int slc_disable;
#endif
static int slbc_sram_enable;
static u32 slbc_force;
static int buffer_ref;
static u32 slbc_ref;
static u32 slbc_sspm_major_ver;
static u32 slbc_sspm_minor_ver;
static u32 slbc_sspm_patch_ver;
static int uid_ref[UID_MAX];
static int slbc_mic_num = 3;
static int slbc_inner = 5;
static int slbc_outer = 5;
static int gid_ref[GID_MAX];
static int gid_vld_cnt[GID_MAX];

static u64 req_val_count;
static u64 rel_val_count;
static u64 req_val_min;
static u64 req_val_max;
static u64 req_val_total;
static u64 rel_val_min;
static u64 rel_val_max;
static u64 rel_val_total;

static LIST_HEAD(slbc_ops_list);
static DEFINE_MUTEX(slbc_ops_lock);
static DEFINE_MUTEX(slbc_req_lock);
static DEFINE_MUTEX(slbc_rel_lock);
static DEFINE_MUTEX(slbc_ref_lock);
DECLARE_WAIT_QUEUE_HEAD(slbc_wq);

/* 1 in bit is for CB fail sid */
static unsigned long slbc_sid_cb_fail;
/* 1 in bit is under wait flow */
static unsigned long slbc_sid_wait_q;
/* 1 in bit is for wait result(fail) */
static unsigned long slbc_sid_wait_fail;
/* 1 in bit is for wait result(done) */
static unsigned long slbc_sid_wait_done;

enum slbc_cb_res {
	RES_ACTIVE = 0,
	RES_DEACTIVE,
	RES_LOW_PRIORITY,
};

static struct slbc_config p_config[] = {
	/* SLBC_ENTRY(id,          sid, max, fix, p, extra, res,   cache) */
	SLBC_ENTRY(UID_AOV_DC,     0,   0,   0,   0, 0x0,   0x07c, 0),
	SLBC_ENTRY(UID_SH_P1,      1,   0,   0,   0, 0x0,   0x007, 0),
	SLBC_ENTRY(UID_SH_APU,     2,   0,   0,   0, 0x0,   0x007, 0),
	SLBC_ENTRY(UID_MM_VENC_8K, 3,   0,   0,   0, 0x0,   0x040, 0),
	SLBC_ENTRY(UID_MM_VENC,    4,   0,   0,   0, 0x0,   0x380, 0),
	SLBC_ENTRY(UID_AOV_APU,    5,   0,   0,   0, 0x0,   0x003, 0),
	SLBC_ENTRY(UID_AINR,       6,   0,   0,   0, 0x0,   0x040, 0),
	SLBC_ENTRY(UID_APU,        7,   0,   0,   0, 0x0,   0x038, 0),
};

struct mtk_slbc_pdata {
	unsigned int slbc_enable;
	unsigned int slbc_all_cache_enable;
};

static struct slbc_data ops_d[ARRAY_SIZE(p_config)];
static struct slbc_ops ops_config[ARRAY_SIZE(p_config)];
static struct task_struct *slbc_activate_task[ARRAY_SIZE(p_config)];
static struct task_struct *slbc_deactivate_task[ARRAY_SIZE(p_config)];

static u32 _slbc_sram_read(u32 offset)
{
	if (!slbc_sram_enable)
		return 0;

	if (!slbc->sram_vaddr || offset >= slbc->regsize)
		return 0;

	return readl(slbc->sram_vaddr + offset);
}

static void _slbc_sram_write(u32 offset, u32 val)
{
	if (!slbc_sram_enable)
		return;

	if (!slbc->sram_vaddr || offset >= slbc->regsize)
		return;

	writel(val, slbc->sram_vaddr + offset);
}

void slbc_sram_init(struct mtk_slbc *slbc)
{
	int i;

	pr_debug("#@# %s(%d) [LVL_QOS] slbc_sram addr:0x%p len:%d\n",
		__func__, __LINE__, slbc->regs, slbc->regsize);

	for (i = 0; i < slbc->regsize; i += 4)
		writel(0x0, slbc->sram_vaddr + i);
}

static int _slbc_test_bit(unsigned long nr , unsigned long *_addr)
{
	unsigned long addr[1] = {*_addr};

	return test_bit(nr, addr);
}

static void _slbc_set_sram_data(struct slbc_data *d)
{
	pr_debug("#@# %s(%d) [LVL_NORM] set pa:0x%p va:0x%p\n",
		__func__, __LINE__, d->paddr, d->vaddr);
}

static void _slbc_clr_sram_data(struct slbc_data *d)
{
	pr_debug("#@# %s(%d) [LVL_NORM] clr pa:0x%p va:0x%p\n",
		__func__, __LINE__, d->paddr, d->vaddr);
}

static int _slbc_get_sid_by_uid(enum slbc_uid uid)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(p_config); i++) {
		if (p_config[i].uid == uid)
			return p_config[i].slot_id;
	}

	return SID_NOT_FOUND;
}

static void _slbc_dcc_ctrl(u32 dcc_flag)
{
	mutex_lock(&slbc_ref_lock);
	if (dcc_flag) {
		if (venc_count == 0) {
			slbc_smc_send(MTK_SLBC_KERNEL_OP_CPU_DCC, 0, 0);
			pr_debug("[slbc] disable DCC\n");
		}
		venc_count++;
	} else {
		venc_count--;
		if (venc_count == 0) {
			slbc_smc_send(MTK_SLBC_KERNEL_OP_CPU_DCC, 1, 1);
			pr_debug("[slbc] enable DCC\n");
		}
	}
	pr_debug("#@# %s(%d) venc_count %d\n",
		__func__, __LINE__, venc_count);
	_slbc_sram_write(SLBC_DCC_COUNT, venc_count);
	_slbc_sram_write(SLBC_DCC_CTRL, dcc_flag);
	mutex_unlock(&slbc_ref_lock);
}

void _slbc_force_cmd(unsigned int force)
{
	slbc_force = force;
	slbc_force_scmi_cmd(force);
}

static int _slbc_force_cache_ratio(enum slc_ach_uid uid, unsigned int ratio)
{
	unsigned int force_cmd;

	pr_debug("#@# %s(%d) [LVL_QOS] uid:%d, ratio:%u\n",
		__func__, __LINE__, uid, ratio);

	/* set force_cmd[31] = 0x1 to indicate setting cache ratio */
	force_cmd = (0x1 << 31 | (ratio & 0x7fff) << 16) | (uid & 0xffff);
	slbc_force = force_cmd;

	return slbc_force_scmi_cmd(force_cmd);
}

static int _slbc_force_cache(enum slc_ach_uid uid, unsigned int size)
{
	unsigned int force_cmd;

	pr_debug("#@# %s(%d) [LVL_QOS] uid:%d, size:%u\n",
		__func__, __LINE__, uid, size);

	/* set force_cmd[31] = 0x0 to indicate setting cache size */
	force_cmd = (0x0 << 31 | (size & 0x7fff) << 16) | (uid & 0xffff);
	slbc_force = force_cmd;

	return slbc_force_scmi_cmd(force_cmd);
}

static int _slbc_activate_thread(void *arg)
{
	struct slbc_ops *ops = arg;
	struct slbc_data *d;
	unsigned int uid;

	if (slbc_enable == 0)
		return -EDISABLED;

	if (!ops)
		return -EFAULT;
	d = ops->data;

	if (d->uid <= UID_ZERO || d->uid >= UID_MAX)
		return -EINVAL;
	uid = d->uid;

	if (ops->activate) {
		pr_debug("#@# %s(%d) [LVL_QOS] %s activate CB run!\n",
			__func__, __LINE__, SLBC_UID_STR(uid));

		if (ops->activate(d) == CB_DONE)
			pr_err("#@# %s(%d) [LVL_WARN] %s activate CB run!\n",
				__func__, __LINE__, SLBC_UID_STR(uid));
		else
			pr_debug("#@# %s(%d) [LVL_QOS] %s activate CB done!\n",
				__func__, __LINE__, SLBC_UID_STR(uid));

		return 0;
	}

	pr_err("#@# %s(%d) [LVL_WARN] %s activate CB not found!\n",
		__func__, __LINE__, SLBC_UID_STR(uid));

	return -EFAULT;
}

static int _slbc_deactivate_thread(void *arg)
{
	struct slbc_ops *ops = arg;
	struct slbc_data *d;
	unsigned int i, uid, sid;

	if (slbc_enable == 0)
		return -EDISABLED;

	if (!ops)
		return -EFAULT;
	d = ops->data;

	if (d->uid <= UID_ZERO || d->uid >= UID_MAX)
		return -EINVAL;
	uid = d->uid;

	sid = _slbc_get_sid_by_uid((enum slbc_uid)uid);
	if (!SLBC_SID_VALID(sid))
		return -EINVAL;

	if (ops->deactivate) {
		pr_debug("#@# %s(%d) [LVL_QOS] %s deactivate CB run!\n",
			__func__, __LINE__, SLBC_UID_STR(uid));

		if (ops->deactivate(d) == CB_DONE) {
			pr_err("#@# %s(%d) [LVL_WARN] %s deactivate CB fail!\n",
				__func__, __LINE__, SLBC_UID_STR(uid));

			/* add user to cb_fail */
			set_bit(sid, &slbc_sid_cb_fail);

			/* trigger wait sid to wakeup */
			for (i = 0; i < ARRAY_SIZE(p_config); i++) {
				if (_slbc_test_bit(i, &slbc_sid_wait_q) &&
					(p_config[i].res_slot & p_config[sid].res_slot)) {
					set_bit(i, &slbc_sid_wait_fail);
				}
			}
		} else {
			pr_debug("#@# %s(%d) [LVL_QOS] %s deactivate CB done!\n",
				__func__, __LINE__, SLBC_UID_STR(uid));

			/* clr user from cb_fail */
			clear_bit(sid, &slbc_sid_cb_fail);
		}
		return 0;
	}

	pr_err("#@# %s(%d) [LVL_WARN] %s deactivate CB not found!\n",
		__func__, __LINE__, SLBC_UID_STR(uid));

	return -EFAULT;
}

void slbc_buffer_cb_notify(u32 res, u32 sid, u32 sid_list)
{
	u32 i, fail;

	if (!SLBC_SID_VALID(sid))
		return;

	pr_debug("#@# %s(%d) [LVL_QOS] res %u sid %u sid_list 0x%x\n",
		__func__, __LINE__, res, sid, sid_list);

	switch (res) {
	case RES_ACTIVE:
		for (i = 0; i < ARRAY_SIZE(p_config); i++) {
			if ((sid_list & (1UL << i)) == 0)
				continue;

			if (_slbc_test_bit(i, &slbc_sid_wait_q))
				set_bit(i, &slbc_sid_wait_done);
			else if (ops_config[i].activate)
				slbc_activate_task[i] = kthread_run(_slbc_activate_thread,
					&ops_config[i], "_slbc_activate_thread");
		}
		break;
	case RES_DEACTIVE:
		fail = 0;
		for (i = 0; i < ARRAY_SIZE(p_config); i++) {
			if (sid_list & (1UL << i) && !ops_config[i].deactivate) {
				pr_err("#@# %s(%d) [LVL_ERR] %s block by no deactivate cb user %s\n",
					__func__, __LINE__, slbc_uid_str[p_config[sid].uid],
					slbc_uid_str[p_config[i].uid]);
				fail++;
			}
		}

		if (!fail && _slbc_test_bit(sid, &slbc_sid_wait_q)) {
			for (i = 0; i < ARRAY_SIZE(p_config); i++) {
				if (sid_list & (1UL << i)) {
					slbc_deactivate_task[i] =
						kthread_run(_slbc_deactivate_thread,
							&ops_config[i], "_slbc_deactivate_thread");
				}
			}
		}
		break;
	case RES_LOW_PRIORITY:
		for (i = 0; i < ARRAY_SIZE(p_config); i++) {
			if (!(sid_list & (1UL << i)))
				continue;

			pr_err("#@# %s(%d) [LVL_ERR] %s block by high priority user %s\n",
				__func__, __LINE__, slbc_uid_str[p_config[sid].uid],
				slbc_uid_str[p_config[i].uid]);
		}
		break;
	default:
		break;
	}
}

static int _slbc_register_activate_ops(struct slbc_ops *ops)
{
	u32 sid;

	if (!ops)
		return -EFAULT;

	if (!ops->data)
		return -EFAULT;

	if (ops->data->uid == UID_MM_VENC || ops->data->uid == UID_MM_VENC_SL) {
		pr_err("#@# %s(%d) [LVL_QOS] register slbc ops fail: venc no need to register slb callback\n",
			__func__, __LINE__);
		return -EFAULT;
	}

	sid = _slbc_get_sid_by_uid((enum slbc_uid)ops->data->uid);
	if (sid == SID_NOT_FOUND) {
		pr_err("#@# %s(%d) [LVL_ERR] register slbc ops fail: invalid uid %d!\n",
			__func__, __LINE__, ops->data->uid);
		return -EFAULT;
	}

	if (ops->data->type != TP_BUFFER) {
		pr_err("#@# %s(%d) [LVL_ERR] %s register slbc ops fail: invalid type %d!\n",
			__func__, __LINE__, slbc_uid_str[ops->data->uid], ops->data->type);
		return -EFAULT;
	}

	if (ops_config[sid].data) {
		pr_debug("#@# %s(%d) [LVL_QOS] %s register slbc ops has been done!\n",
			__func__, __LINE__, slbc_uid_str[ops->data->uid]);
		return 0;
	}

	ops_d[sid] = *(ops->data);
	ops_config[sid].data = &ops_d[sid];
	ops_config[sid].activate = ops->activate;
	ops_config[sid].deactivate = ops->deactivate;

	pr_debug("#@# %s(%d) [LVL_QOS] %s register slbc ops success!\n",
		__func__, __LINE__, slbc_uid_str[ops->data->uid]);

	return 0;
}

static int _slbc_request_status(struct slbc_data *d)
{
	int ret = 0;
	int uid = d->uid;

	if (uid <= UID_ZERO || uid > UID_MAX)
		ret = -EINVAL;
	else
		ret = _slbc_buffer_status_scmi(d);

	return ret;
}

static int _slbc_cb_timeout(int err, struct slbc_data *d)
{
	int ret = 0;
	int uid = d->uid;
	unsigned int i = 0;

	if (!SLBC_UID_VALID(d->uid) || !SLBC_SID_VALID(d->sid))
		return err;

	if (err != -EWAIT_RELEASE)
		return err;

	/*  timeout handle */
	if (d->timeout == 0) {
		pr_err("#@# %s(%d) [LVL_WARN] %s need to wait slb release!\n",
			__func__, __LINE__, slbc_uid_str[uid]);
		return -ENOT_AVAILABLE;
	} else if (d->timeout >= SLBC_TIMEOUT_LIMIT) {
		pr_err("#@# %s(%d) [LVL_WARN] %s need to wait slb release (invalid timeout: %ums)!\n",
			__func__, __LINE__, slbc_uid_str[uid], d->timeout);
		return -ENOT_AVAILABLE;
	}

	/* cb timeout policy */
	pr_debug("#@# %s(%d) [LVL_QOS] %s start to wait slb, timeout %ums\n",
		__func__, __LINE__, slbc_uid_str[uid], d->timeout);

	ret = 0;
	for (i = 0; i < d->timeout; i++) {
		mdelay(1);
		if (_slbc_test_bit(d->sid, &slbc_sid_wait_done) ||
			_slbc_test_bit(d->sid, &slbc_sid_wait_fail)) {
			ret = 1;
			break;
		}
	}

	pr_err("#@# %s(%d) [LVL_QOS] %s stop to wait slb, slbc_sid_wait_done 0x%lx, slbc_sid_wait_fail 0x%lx\n",
		__func__, __LINE__, slbc_uid_str[uid], slbc_sid_wait_done, slbc_sid_wait_fail);

	if (ret) {
		if (_slbc_test_bit(d->sid, &slbc_sid_wait_done)) {
			pr_info("#@# %s(%d) [LVL_QOS] %s wait slb success\n",
				__func__, __LINE__, slbc_uid_str[uid]);
			ret = _slbc_request_buffer_scmi(d);
		} else {
			pr_err("#@# %s(%d) [LVL_ERR] %s wait slb fail, %s 0x%lx %s 0x%lx %s 0x%lx\n",
				__func__, __LINE__, slbc_uid_str[uid],
				"slbc_sid_wait_fail", slbc_sid_wait_fail,
				"slbc_sid_wait_done", slbc_sid_wait_done,
				"slbc_sid_cb_fail", slbc_sid_cb_fail);
			ret = -EREQ_WAIT_FAIL;
		}
	} else {
		pr_err("#@# %s(%d) [LVL_ERR] %s wait timeout!\n",
			__func__, __LINE__, slbc_uid_str[uid]);
		ret = -EREQ_TIMEOUT;
	}

	return ret;
}

static void _slbc_set_sid_wait_q(struct slbc_data *d)
{
	if (!SLBC_SID_VALID(d->sid))
		return;
	if (d->timeout > 0 && d->timeout < SLBC_TIMEOUT_LIMIT) {
		set_bit(d->sid, &slbc_sid_wait_q);
		clear_bit(d->sid, &slbc_sid_wait_done);
		clear_bit(d->sid, &slbc_sid_wait_fail);
	}
}

static void _slbc_clr_sid_wait_q(struct slbc_data *d)
{
	if (!SLBC_SID_VALID(d->sid))
		return;
	if (d->timeout > 0 && d->timeout < SLBC_TIMEOUT_LIMIT)
		clear_bit(d->sid, &slbc_sid_wait_q);
}

static int _slbc_request_buffer(struct slbc_data *d)
{
	int ret = 0;
	int uid = d->uid;
	int sid;

	if (uid <= UID_ZERO || uid > UID_MAX)
		d->config = NULL;
	else {
		pr_debug("#@# %s(%d) [LVL_QOS] %s req TP_BUFFER\n",
			__func__, __LINE__, SLBC_UID_STR(uid));

		sid = _slbc_get_sid_by_uid((enum slbc_uid)uid);
		if (sid != SID_NOT_FOUND) {
			d->sid = sid;
			d->config = &p_config[sid];
		}
	}

	_slbc_set_sid_wait_q(d);
	mutex_lock(&slbc_req_lock);
	ret = _slbc_request_buffer_scmi(d);
	mutex_unlock(&slbc_req_lock);

	if (ret == -ENOT_AVAILABLE) {
		pr_err("#@# %s(%d) [LVL_WARN] %s need to wait slb release!\n",
			__func__, __LINE__, SLBC_UID_STR(uid));
	} else if (ret == -EWAIT_RELEASE) {
		ret = _slbc_cb_timeout(ret, d);
	}
	_slbc_clr_sid_wait_q(d);

	if (!ret) {
		_slbc_set_sram_data(d);

#if IS_ENABLED(CONFIG_MTK_SLBC_IPI)
		buffer_ref = _slbc_sram_read(SLBC_BUFFER_REF);
#else
		buffer_ref++;
#endif /* CONFIG_MTK_SLBC_IPI */
	}

	return ret;
}

static int _slbc_status(struct slbc_data *d)
{
	int ret = 0;

	if ((d->type) == TP_BUFFER)
		ret = _slbc_request_status(d);

	pr_debug("#@# %s(%d) [LVL_QOS] uid 0x%x ret %d\n",
		__func__, __LINE__, d->uid, ret);

	return ret;
}

static int _slbc_request(struct slbc_data *d)
{
	int ret = 0;
	int sid;
	u64 begin, val;

	if (slbc_enable == 0)
		return -EDISABLED;

	begin = ktime_get_ns();

	if ((d->type) == TP_BUFFER) {
		ret = _slbc_request_buffer(d);

		sid = _slbc_get_sid_by_uid((enum slbc_uid)d->uid);
		if (sid != SID_NOT_FOUND)
			d->slot_used = p_config[sid].res_slot;

		if (!d->paddr)
			ret = -1;
		else
			d->emi_paddr = (void __iomem *)((((unsigned long)d->paddr)
					 & SLBC_PADDR_MASK) | SLBC_WAY_B_BASE);
	}

	pr_debug("#@# %s(%d) [LVL_QOS] uid 0x%x ret %d d->ret %d pa 0x%p emipa 0x%p size 0x%zx\n",
		__func__, __LINE__, d->uid, ret, d->ret, d->paddr, d->emi_paddr, d->size);

	if (!ret) {
#if IS_ENABLED(CONFIG_MTK_SLBC_IPI)
		slbc_ref = _slbc_sram_read(SLBC_REF);
#else
		slbc_ref++;
#endif /* CONFIG_MTK_SLBC_IPI */
	}

	val = div_u64((ktime_get_ns() - begin), 1000000);
	req_val_count++;
	req_val_total += val;
	req_val_max = max(val, req_val_max);
	if (!req_val_min)
		req_val_min = val;
	else
		req_val_min = min(val, req_val_min);

	return ret;
}

static int _slbc_release_buffer(struct slbc_data *d)
{
	int ret = 0;

	mutex_lock(&slbc_req_lock);
	ret = _slbc_release_buffer_scmi(d);
	mutex_unlock(&slbc_req_lock);

	if (!ret) {
		_slbc_clr_sram_data(d);

#if IS_ENABLED(CONFIG_MTK_SLBC_IPI)
		buffer_ref = _slbc_sram_read(SLBC_BUFFER_REF);
#else
		buffer_ref--;
#endif /* CONFIG_MTK_SLBC_IPI */
		WARN_ON(buffer_ref < 0);
	}

	return ret;
}

static int _slbc_release(struct slbc_data *d)
{
	int ret = 0;
	u64 begin, val;

	begin = ktime_get_ns();

	if ((d->type) == TP_BUFFER) {
		ret = _slbc_release_buffer(d);
		d->size = 0;
	}

	pr_debug("#@# %s(%d) [LVL_QOS] uid 0x%x ret %d d->ret %d pa 0x%p size 0x%zx\n",
		__func__, __LINE__, d->uid, ret, d->ret, d->paddr, d->size);

	if (!ret) {
#if IS_ENABLED(CONFIG_MTK_SLBC_IPI)
		slbc_ref = _slbc_sram_read(SLBC_REF);
#else
		slbc_ref--;
#endif /* CONFIG_MTK_SLBC_IPI */
	}

	val = div_u64((ktime_get_ns() - begin), 1000000);
	rel_val_count++;
	rel_val_total += val;
	rel_val_max = max(val, rel_val_max);
	if (!rel_val_min)
		rel_val_min = val;
	else
		rel_val_min = min(val, rel_val_min);

	return ret;
}

static int _slbc_power_on(struct slbc_data *d)
{
	unsigned int uid;

	if (slbc_enable == 0)
		return -EDISABLED;

	if (!d)
		return -EINVAL;

	uid = d->uid;
	if (uid <= UID_ZERO || uid >= UID_MAX)
		return -EINVAL;

	return 0;
}

static int _slbc_power_off(struct slbc_data *d)
{
	unsigned int uid;

	if (slbc_enable == 0)
		return -EDISABLED;

	if (!d)
		return -EINVAL;

	uid = d->uid;
	if (uid <= UID_ZERO || uid >= UID_MAX)
		return -EINVAL;

	return 0;
}

static int _slbc_secure_on(struct slbc_data *d)
{
	unsigned int uid;

	if (slbc_enable == 0)
		return -EDISABLED;

	if (!d)
		return -EINVAL;

	uid = d->uid;
	if (uid <= UID_ZERO || uid >= UID_MAX)
		return -EINVAL;

	pr_debug("#@# %s(%d) %s flag %x\n", __func__, __LINE__, slbc_uid_str[uid], d->flag);

	return 0;
}

static int _slbc_secure_off(struct slbc_data *d)
{
	unsigned int uid;

	if (slbc_enable == 0)
		return -EDISABLED;

	if (!d)
		return -EINVAL;

	uid = d->uid;
	if (uid <= UID_ZERO || uid >= UID_MAX)
		return -EINVAL;

	pr_debug("#@# %s(%d) %s flag %x\n", __func__, __LINE__, slbc_uid_str[uid], d->flag);

	return 0;
}

static void _slbc_update_mm_bw(unsigned int bw)
{
	_slbc_sram_write(SLBC_MM_EST_BW, bw);
}

static void _slbc_update_mic_num(unsigned int num)
{
	int i;

	slbc_mic_num = num;
	slbc_mic_num_cmd(num);

	if (!uid_ref[UID_HIFI3]) {
		for (i = 0; i < ARRAY_SIZE(p_config); i++) {
			if (p_config[i].uid == UID_HIFI3) {
				if (num == 3)
					p_config[i].res_slot = 0xe00;
				else
					p_config[i].res_slot = 0x200;
			}
		}
	}
}

void slbc_update_inner(unsigned int inner)
{
	slbc_inner = inner;
	slbc_inner_cmd(inner);
}

void slbc_update_outer(unsigned int outer)
{
	slbc_outer = outer;
	slbc_outer_cmd(outer);
}

static int _slbc_gid_val(enum slc_ach_uid uid)
{
	int ret = -EINVAL;

	switch (uid) {
	case ID_DBI:
		ret = GID_DBI;
		break;
	case ID_OD:
		ret = GID_OD;
		break;
	case ID_DMR:
		ret = GID_DMR;
		break;
	case ID_MAE:
		ret = GID_MAE;
		break;
	case ID_IMG:
		break;
	case ID_CAM:
		break;
	case ID_AOV:
		ret = GID_AOV;
		break;
	case ID_ADSP:
		ret = GID_ADSP;
		break;
	case ID_MD:
		ret = GID_MD;
		break;
	case ID_VDEC_FRAME:
		ret = GID_VDEC_FRAME;
		break;
	case ID_VDEC_UBE:
		ret = GID_VDEC_UBE;
		break;
	case ID_GPU:
		ret = GID_GPU;
		break;
	case ID_GPU_W:
	case ID_OVL_R:
		ret = GID_GPU_OVL;
		break;
	default:
		pr_err("#@# %s(%d) [LVL_ERR] unrecognized uid\n", __func__, __LINE__);
		break;
	}

	return ret;
}

static int _slbc_check_uid_gid(enum slc_ach_uid uid, int gid)
{
	int ret = 0;

	switch (uid) {
	case ID_DBI:
		if (gid != GID_DBI)
			ret = -EINVAL;
		break;
	case ID_OD:
		if (gid != GID_OD)
			ret = -EINVAL;
		break;
	case ID_DMR:
		if (gid != GID_DMR)
			ret = -EINVAL;
		break;
	case ID_MAE:
		if (gid != GID_MAE)
			ret = -EINVAL;
		break;
	case ID_IMG:
		if (gid != GID_IMG_SMT && gid != GID_IMG_MCNR)
			ret = -EINVAL;
		break;
	case ID_CAM:
		if (gid != GID_CAM)
			ret = -EINVAL;
		break;
	case ID_AOV:
		if (gid != GID_AOV)
			ret = -EINVAL;
		break;
	case ID_ADSP:
		if (gid != GID_ADSP)
			ret = -EINVAL;
		break;
	case ID_MD:
		if (gid != GID_MD)
			ret = -EINVAL;
		break;
	case ID_VDEC_FRAME:
		if (gid != GID_VDEC_FRAME)
			ret = -EINVAL;
		break;
	case ID_VDEC_UBE:
		if (gid != GID_VDEC_UBE)
			ret = -EINVAL;
		break;
	case ID_GPU:
		if (gid != GID_GPU)
			ret = -EINVAL;
		break;
	case ID_GPU_W:
	case ID_OVL_R:
		if (gid != GID_GPU_OVL)
			ret = -EINVAL;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static void _slbc_get_gid_by_req(enum slc_ach_uid uid, int *gid)
{
	if (*gid != GID_REQ)
		return;

	switch (uid) {
	case ID_AOV:
		*gid = GID_AOV;
		break;
	case ID_ADSP:
		*gid = GID_ADSP;
		break;
	case ID_MD:
		*gid = GID_MD;
		break;
	case ID_VDEC_UBE:
		*gid = GID_VDEC_UBE;
		break;
	case ID_GPU:
		*gid = GID_GPU;
		break;
	case ID_OD:
		*gid = GID_OD;
		break;
	case ID_DBI:
		*gid = GID_DBI;
		break;
	case ID_DMR:
		*gid = GID_DMR;
		break;
	case ID_MAE:
		*gid = GID_MAE;
		break;
	case ID_CAM:
		*gid = GID_CAM;
		break;
	case ID_IMG:
		*gid = GID_IMG_MCNR; /* use 1 GID of ID_IMG to represent its 2 GIDs (GID_IMG_SMT/GID_IMG_MCNR) */
		break;
	default:
		break;
	}
}

static int _slbc_gid_request(enum slc_ach_uid uid, int *gid, struct slbc_gid_data *data)
{
	int local_cnt = 0;
	int ret = 0;

	/* block GPU_OVL */
	if (*gid == GID_GPU_OVL)
		return -EINVAL;

	if (data->sign != SLC_DATA_MAGIC) {
		pr_err("#@# %s(%d) [LVL_ERR] uid:%d ,invalid sign:%#x\n",
			__func__, __LINE__, uid, data->sign);
		return -EINVAL;
	}

	if (uid == ID_MAE && gid_ref[GID_AOV]) {
		pr_err("#@# %s(%d) [LVL_ERR] MAE conflicts with AOV\n",
			__func__, __LINE__);
		return -EINVAL;
	} else if (uid == ID_AOV && gid_ref[GID_MAE]) {
		pr_err("#@# %s(%d) [LVL_ERR] AOV conflicts with MAE\n",
			__func__, __LINE__);
		return -EINVAL;
	}

	_slbc_get_gid_by_req(uid, gid);
	ret = _slbc_check_uid_gid(uid, *gid);
	if (ret) {
		pr_err("#@# %s(%d) [LVL_ERR] uid:%d or gid:%d unrecognized\n",
			__func__, __LINE__, uid, *gid);
		return ret;
	}

	local_cnt = atomic_inc_return((atomic_t *) &gid_ref[*gid]);
	if (uid == ID_IMG)
		atomic_inc((atomic_t *) &gid_ref[GID_IMG_SMT]);

	if (local_cnt == 1) {
		/* Set M/G and G/P tables */
		mutex_lock(&slbc_req_lock);
		ret = _slbc_ach_scmi(IPI_SLBC_GID_REQUEST_FROM_AP, uid, *gid, data);
		if (uid == ID_IMG)
			ret = _slbc_ach_scmi(IPI_SLBC_GID_REQUEST_FROM_AP, uid, GID_IMG_SMT, data);
		mutex_unlock(&slbc_req_lock);

		if (ret) {
			pr_err("#@# %s(%d) [LVL_ERR] ach scmi req fail, uid:%d, gid:%d\n",
				__func__, __LINE__, uid, *gid);
			return ret;
		}
	}

	pr_debug("#@# %s(%d) [LVL_NORM] gid:%d\n", __func__, __LINE__, *gid);

	return ret;
}

static int _slbc_gid_release(enum slc_ach_uid uid, int gid)
{
	int local_cnt = 0;
	int ret = 0;

	/* block GPU_OVL */
	if (gid == GID_GPU_OVL)
		return -EINVAL;

	ret = _slbc_check_uid_gid(uid, gid);
	if (ret) {
		pr_err("#@# %s(%d) [LVL_ERR] uid:%d or gid:%d unrecognized\n",
			__func__, __LINE__, uid, gid);
		return ret;
	}

	local_cnt = atomic_dec_return((atomic_t *) &gid_ref[gid]);
	if (uid == ID_IMG)
		atomic_dec((atomic_t *) &gid_ref[GID_IMG_SMT]);

	if (local_cnt == 0) {
		/* Clear M/G and G/P tables */
		mutex_lock(&slbc_req_lock);
		ret = _slbc_ach_scmi(IPI_SLBC_GID_RELEASE_FROM_AP, uid, gid, NULL);
		if (uid == ID_IMG)
			ret = _slbc_ach_scmi(IPI_SLBC_GID_RELEASE_FROM_AP, uid, GID_IMG_SMT, NULL);
		mutex_unlock(&slbc_req_lock);

		if (ret) {
			pr_err("#@# %s(%d) [LVL_ERR] ach scmi rel fail, uid:%d, gid:%d\n",
				__func__, __LINE__, uid, gid);
			return ret;
		}
	}

	pr_debug("#@# %s(%d) [LVL_NORM] gid:%d\n", __func__, __LINE__, gid);

	return ret;
}

static int _slbc_roi_update(enum slc_ach_uid uid, int gid, struct slbc_gid_data *data)
{
	int ret = 0;

	if (gid >= GID_MAX)
		return -EINVAL;

	mutex_lock(&slbc_req_lock);
	ret = _slbc_ach_scmi(IPI_SLBC_ROI_UPDATE_FROM_AP, uid, gid, data);
	if (ret) {
		pr_err("#@# %s(%d) [LVL_ERR] ach scmi vld fail, uid:%d, gid:%d\n",
			__func__, __LINE__, uid, gid);
		return ret;
	}

	mutex_unlock(&slbc_req_lock);

	return ret;
}

static int _slbc_validate(enum slc_ach_uid uid, int gid)
{
	int local_cnt = 0;
	int ret = 0;

	/* block GPU_OVL */
	if (gid == GID_GPU_OVL)
		return -EINVAL;

	ret = _slbc_check_uid_gid(uid, gid);
	if (ret) {
		pr_err("#@# %s(%d) [LVL_ERR] uid:%d or gid:%d unrecognized\n",
			__func__, __LINE__, uid, gid);
		return ret;
	}

	local_cnt = atomic_inc_return((atomic_t *) &gid_vld_cnt[gid]);
	if (uid == ID_IMG)
		atomic_inc((atomic_t *) &gid_vld_cnt[GID_IMG_SMT]);

	if (local_cnt == 1) {
		mutex_lock(&slbc_req_lock);
		ret = _slbc_ach_scmi(IPI_SLBC_GID_VALID_FROM_AP, uid, gid, NULL);
		if (uid == ID_IMG)
			ret = _slbc_ach_scmi(IPI_SLBC_GID_VALID_FROM_AP, uid, GID_IMG_SMT, NULL);
		mutex_unlock(&slbc_req_lock);

		if (ret) {
			pr_err("#@# %s(%d) [LVL_ERR] ach scmi vld fail, uid:%d, gid:%d\n",
				__func__, __LINE__, uid, gid);
			return ret;
		}
	}

	pr_debug("#@# %s(%d) [LVL_NORM] gid:%d\n", __func__, __LINE__, gid);

	return ret;
}

static int _slbc_invalidate(enum slc_ach_uid uid, int gid)
{
	int local_cnt = 0;
	int ret = 0;

	/* block GPU_OVL */
	if (gid == GID_GPU_OVL)
		return -EINVAL;

	ret = _slbc_check_uid_gid(uid, gid);
	if (ret) {
		pr_err("#@# %s(%d) [LVL_ERR] uid:%d or gid:%d unrecognized\n",
			__func__, __LINE__, uid, gid);
		return ret;
	}

	local_cnt = atomic_dec_return((atomic_t *) &gid_vld_cnt[gid]);
	if (uid == ID_IMG)
		atomic_dec((atomic_t *) &gid_vld_cnt[GID_IMG_SMT]);

	if (local_cnt == 0) {
		mutex_lock(&slbc_req_lock);
		ret = _slbc_ach_scmi(IPI_SLBC_GID_INVALID_FROM_AP, uid, gid, NULL);
		if (uid == ID_IMG)
			ret = _slbc_ach_scmi(IPI_SLBC_GID_INVALID_FROM_AP, uid, GID_IMG_SMT, NULL);
		mutex_unlock(&slbc_req_lock);

		if (ret) {
			pr_err("#@# %s(%d) [LVL_ERR] ach scmi invld fail, uid:%d, gid:%d\n",
				__func__, __LINE__, uid, gid);
			return ret;
		}
	}

	pr_debug("#@# %s(%d) [LVL_NORM] gid:%d\n", __func__, __LINE__, gid);

	return ret;
}

static int _slbc_read_invalidate(enum slc_ach_uid uid, int gid, int enable)
{
	struct slbc_gid_data data;
	int ret = 0;

	pr_debug("#@# %s(%d) [LVL_NORM] gid:%d\n", __func__, __LINE__, gid);
	if (gid >= GID_MAX)
		return -EINVAL;

	data.bw = enable;
	mutex_lock(&slbc_req_lock);
	ret = _slbc_ach_scmi(IPI_SLBC_GID_READ_INVALID_FROM_AP, uid, gid, &data);
	if (ret) {
		pr_err("#@# %s(%d) [LVL_ERR] ach read invld fail, uid:%d, gid:%d\n",
			__func__, __LINE__, uid, gid);
		return ret;
	}
	mutex_unlock(&slbc_req_lock);

	return ret;
}

static int _slbc_ceil(enum slc_ach_uid uid, unsigned int ceil)
{
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCMI)
	int ret = 0;
	struct scmi_tinysys_slbc_ctrl_status rvalue = {0};

	pr_debug("#@# %s(%d) [LVL_QOS] uid:%d, ceil:%u\n", __func__, __LINE__, uid, ceil);
	ret = slbc_ctrl_scmi_info(IPI_SLBC_CACHE_USER_CEIL_SET, uid ,ceil, 0, 0, &rvalue);

	return ret;
#else
	return 0;
#endif /* CONFIG_MTK_TINYSYS_SCMI */
}

static int _slbc_total_ceil(unsigned int ceil)
{
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCMI)
	int ret = 0;
	struct scmi_tinysys_slbc_ctrl_status rvalue = {0};

	pr_debug("#@# %s(%d) [LVL_QOS] total ceil:%u\n", __func__, __LINE__, ceil);
	ret = slbc_ctrl_scmi_info(IPI_SLBC_CACHE_USER_CEIL_SET, 0 ,ceil, 0, 0, &rvalue);

	return ret;
#else
	return 0;
#endif /* CONFIG_MTK_TINYSYS_SCMI */
}

static int _slbc_window(unsigned int window)
{
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCMI)
	int ret = 0;
	struct scmi_tinysys_slbc_ctrl_status rvalue = {0};

	pr_debug("#@# %s(%d) [LVL_QOS] window:%d\n", __func__, __LINE__, window);
	ret = slbc_ctrl_scmi_info(IPI_SLBC_CACHE_WINDOW_SET, window, 0, 0, 0, &rvalue);

	return ret;
#else
	return 0;
#endif /* CONFIG_MTK_TINYSYS_SCMI */
}

static int _slbc_cg_priority(bool gpu_first)
{
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCMI)
	int ret = 0;
	struct scmi_tinysys_slbc_ctrl_status rvalue = {0};

	pr_debug("#@# %s(%d) [LVL_QOS] CG priority:%d\n", __func__, __LINE__, gpu_first);
	ret = slbc_ctrl_scmi_info(IPI_SLBC_CG_PRIORITY_SET, gpu_first, 0, 0, 0, &rvalue);

	return ret;
#else
	return 0;
#endif /* CONFIG_MTK_TINYSYS_SCMI */
}

static int _slbc_disable_dcc(bool disable)
{
	mutex_lock(&slbc_ref_lock);
	if (disable) {
		if (venc_count == 0)
			slbc_smc_send(MTK_SLBC_KERNEL_OP_CPU_DCC, 0, 0);
		venc_count++;
	} else {
		venc_count--;
		if (venc_count == 0)
			slbc_smc_send(MTK_SLBC_KERNEL_OP_CPU_DCC, 1, 1);
	}
	pr_debug("#@# %s(%d) venc_count %d\n", __func__, __LINE__, venc_count);
	_slbc_sram_write(SLBC_DCC_COUNT, venc_count);
	mutex_unlock(&slbc_ref_lock);
	return 0;
}

static int _slbc_disable_slc(bool disable)
{
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCMI)
	slc_disable = (int)disable;
	pr_debug("slc disable %d\n", slc_disable);

	return slbc_sspm_slc_disable(slc_disable);
#else
	return 0;
#endif /* CONFIG_MTK_TINYSYS_SCMI */
}

static int _slbc_get_cache_size(enum slc_ach_uid uid)
{
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCMI)
	int ret = 0;
	struct scmi_tinysys_slbc_ctrl_status rvalue = {0};

	ret = slbc_ctrl_scmi_info(IPI_SLBC_CACHE_USER_INFO, uid, 0, 0, 0, &rvalue);
	if (ret)
		return ret;

	return rvalue.slbc_resv1;
#else
	return 0;
#endif /* CONFIG_MTK_TINYSYS_SCMI */
}

static int _slbc_get_cache_hit_rate(enum slc_ach_uid uid)
{
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCMI)
	int ret = 0;
	struct scmi_tinysys_slbc_ctrl_status rvalue = {0};

	ret = slbc_ctrl_scmi_info(IPI_SLBC_CACHE_USER_INFO, uid, 0, 0, 0, &rvalue);
	if (ret)
		return ret;

	return rvalue.slbc_resv2;
#else
	return 0;
#endif /* CONFIG_MTK_TINYSYS_SCMI */
}

static int _slbc_get_cache_hit_bw(enum slc_ach_uid uid)
{
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCMI)
	int ret = 0;
	struct scmi_tinysys_slbc_ctrl_status rvalue = {0};

	ret = slbc_ctrl_scmi_info(IPI_SLBC_CACHE_USER_INFO, uid, 0, 0, 0, &rvalue);
	if (ret)
		return ret;

	return rvalue.slbc_resv3;
#else
	return 0;
#endif /* CONFIG_MTK_TINYSYS_SCMI */
}

static int _slbc_get_cache_usage(int *cpu, int *gpu, int *other)
{
#if IS_ENABLED(CONFIG_MTK_TINYSYS_SCMI)
	int ret = 0;
	struct scmi_tinysys_slbc_ctrl_status rvalue = {0};

	ret = slbc_ctrl_scmi_info(IPI_SLBC_CACHE_USAGE, 0, 0, 0, 0, &rvalue);
	if (ret)
		return ret;

	*cpu = (int) rvalue.slbc_resv1;
	*gpu = (int) rvalue.slbc_resv2;
	*other = (int) rvalue.slbc_resv3;

	return 0;
#else
	return 0;
#endif /* CONFIG_MTK_TINYSYS_SCMI */
}

static struct slbc_common_ops common_ops = {
	.slbc_status = _slbc_status,
	.slbc_request = _slbc_request,
	.slbc_release = _slbc_release,
	.slbc_power_on = _slbc_power_on,
	.slbc_power_off = _slbc_power_off,
	.slbc_secure_on = _slbc_secure_on,
	.slbc_secure_off = _slbc_secure_off,
	.slbc_register_activate_ops = _slbc_register_activate_ops,
	.slbc_activate_status = slbc_activate_status,
	.slbc_sram_read = _slbc_sram_read,
	.slbc_sram_write = _slbc_sram_write,
	.slbc_update_mm_bw = _slbc_update_mm_bw,
	.slbc_update_mic_num = _slbc_update_mic_num,
	.slbc_gid_val = _slbc_gid_val,
	.slbc_gid_request = _slbc_gid_request,
	.slbc_gid_release = _slbc_gid_release,
	.slbc_roi_update = _slbc_roi_update,
	.slbc_validate = _slbc_validate,
	.slbc_invalidate = _slbc_invalidate,
	.slbc_read_invalidate = _slbc_read_invalidate,
	.slbc_force_cache = _slbc_force_cache,
	.slbc_force_cache_ratio = _slbc_force_cache_ratio,
	.slbc_ceil = _slbc_ceil,
	.slbc_total_ceil = _slbc_total_ceil,
	.slbc_window = _slbc_window,
	.slbc_cg_priority = _slbc_cg_priority,
	.slbc_disable_dcc = _slbc_disable_dcc,
	.slbc_disable_slc = _slbc_disable_slc,
	.slbc_get_cache_size = _slbc_get_cache_size,
	.slbc_get_cache_hit_rate = _slbc_get_cache_hit_rate,
	.slbc_get_cache_hit_bw = _slbc_get_cache_hit_bw,
	.slbc_get_cache_usage = _slbc_get_cache_usage,
};

static struct slbc_ipi_ops ipi_ops = {
	.slbc_buffer_cb_notify = slbc_buffer_cb_notify,
	.slbc_dcc_ctrl = _slbc_dcc_ctrl,
};

void slbc_get_gid_for_dma(struct dma_buf *dmabuf_2)
{
	int gid = GID_REQ;
	struct slbc_gid_data *gid_data;
	unsigned int buffer_fd, producer, consumer;
	struct iosys_map map;
	struct dma_buf *dmabuf_1;
	int ret;

	memset(&map, 0, sizeof(map));
	ret = dma_buf_vmap_unlocked(dmabuf_2, &map);
	if (ret) {
		pr_err("#@# %s(%d) [LVL_ERR] dma_buf_vmap_unlocked failed\n",
			__func__, __LINE__);
		goto err1;
	}

	gid_data = (struct slbc_gid_data *)map.vaddr;
	buffer_fd = gid_data->buffer_fd;
	producer = gid_data->producer;
	consumer = gid_data->consumer;
	pr_debug("#@# %s(%d) [LVL_QOS] buffer_fd:%d  producer:0x%x  consumer:0x%x\n",
		__func__, __LINE__, buffer_fd, producer, consumer);

	if (gid_data->sign != SLC_DATA_MAGIC) {
		pr_err("#@# %s(%d) [LVL_QOS] invalid sign:%#x\n",
			__func__, __LINE__, gid_data->sign);
		goto err1;
	}

	/* mapping producre/consumer to GID */
	switch (producer) {
	case BUF_ID_GPU | BUF_ID_OVL:
	case BUF_ID_GPU:
		if (consumer & BUF_ID_OVL)	/* GPU to OVL */
			gid = GID_GPU_OVL;
		else                        /* GPU only */
			gid = GID_GPU;
		break;
	case BUF_ID_VDEC:
		gid = GID_VDEC_FRAME;
		break;
	default:
		if (consumer & BUF_ID_GPU)  /* GPU only */
			gid = GID_GPU;
	}

	dmabuf_1 = dma_buf_get(buffer_fd);
	if (IS_ERR(dmabuf_1)) {
		pr_err("#@# %s(%d) [LVL_ERR] dma_buf_get failed\n",
			__func__, __LINE__);
		goto err1;
	}
	ret = dma_buf_set_gid(dmabuf_1, gid);
	if (ret)
		pr_err("#@# %s(%d) [LVL_ERR] dma_buf_set_gid failed\n",
			__func__, __LINE__);

	dma_buf_put(dmabuf_1);
err1:
	dma_buf_vunmap_unlocked(dmabuf_2, &map);
}

static int slbc_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	int ret = 0;
	uint32_t reg[4] = {0, 0, 0, 0};
	const struct mtk_slbc_pdata *pdata;

#if IS_ENABLED(CONFIG_MTK_SLBC_IPI)
	ret = slbc_scmi_init();
	if (ret < 0)
		return ret;
#endif /* CONFIG_MTK_SLBC_IPI */

	pdata = of_device_get_match_data(dev);
	if (!pdata)
		return dev_err_probe(dev, -EINVAL, "No platform data available\n");

	slbc_enable = pdata->slbc_enable;
	dev_info(dev, "#@# %s(%d) [LVL_QOS] slbc_enable %d\n",
		__func__, __LINE__, slbc_enable);
	slbc_all_cache_mode = pdata->slbc_all_cache_enable;
	dev_info(dev, "#@# %s(%d) [LVL_QOS] slbc_all_cache_mode %d\n",
		__func__, __LINE__, slbc_all_cache_mode);

	slbc = devm_kzalloc(dev, sizeof(struct mtk_slbc), GFP_KERNEL);
	if (!slbc)
		return -ENOMEM;

	slbc->dev = dev;

	ret = of_property_read_u32_array(node, "reg", reg, 4);
	if (ret < 0) {
		slbc_sram_enable = 0;

		dev_err(dev, "#@# %s(%d) [LVL_ERR] slbc of_property_read_u32_array ERR: %d\n",
			__func__, __LINE__, ret);
	} else {

		slbc->regs = (void *)(long)reg[1];
		slbc->regsize = reg[3];
		slbc->sram_vaddr = (void __iomem *) devm_memremap(dev,
				(resource_size_t)reg[1], slbc->regsize,
				MEMREMAP_WT);
		if (IS_ERR(slbc->sram_vaddr)) {
			slbc_sram_enable = 0;

			dev_notice(dev, "slbc could not ioremap resource for memory\n");
		} else {
			slbc_sram_enable = 1;

			dev_info(dev, "#@# %s(%d) [LVL_QOS] slbc->regs 0x%p slbc->regsize 0x%x\n",
				__func__, __LINE__, slbc->regs, slbc->regsize);
			slbc_sram_init(slbc);
		}
	}

#if IS_ENABLED(CONFIG_PM_SLEEP)
	slbc->ws = wakeup_source_register(NULL, "slbc");
	if (!slbc->ws)
		dev_err(dev, "#@# %s(%d) [LVL_ERR] slbc wakelock register fail!\n",
			__func__, __LINE__);
#endif /* CONFIG_PM_SLEEP */

	slbc_register_ipi_ops(&ipi_ops);
	slbc_register_common_ops(&common_ops);
	if (slbc_all_cache_mode) {
		ret = mtk_dmaheap_register_slc_callback(slbc_get_gid_for_dma);
		dev_info(dev, "#@# %s(%d) [LVL_NORM] mtk_dmaheap_register_slc_callback done\n",
			__func__, __LINE__);
	}

	if (slbc_enable) {
		ret = slbc_sspm_enable(slbc_enable);
		if (ret < 0)
			dev_err(dev, "slbc_sspm_enable failed.\n");

		ret = slbc_get_sspm_ver(&slbc_sspm_major_ver, &slbc_sspm_minor_ver, &slbc_sspm_patch_ver);
		if (ret < 0)
			dev_err(dev, "slbc_get_sspm_ver failed.\n");
	}

	return 0;
}

static int slbc_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	slbc_unregister_ipi_ops(&ipi_ops);
	slbc_unregister_common_ops(&common_ops);
	if (slbc_all_cache_mode)
		mtk_dmaheap_unregister_slc_callback();
	devm_kfree(dev, slbc);

	return 0;
}

static int slbc_suspend(struct device *dev)
{
#if IS_ENABLED(CONFIG_MTK_SLBC_IPI)
	slbc_suspend_resume_notify(1);
#endif /* CONFIG_MTK_SLBC_IPI */
	return 0;
}

static int slbc_resume(struct device *dev)
{
#if IS_ENABLED(CONFIG_MTK_SLBC_IPI)
	slbc_suspend_resume_notify(0);
#endif /* CONFIG_MTK_SLBC_IPI */
	return 0;
}

static int slbc_suspend_cb(struct device *dev)
{
	int i;
	int sid;

	for (i = 0; i < UID_MAX; i++) {
		sid = _slbc_get_sid_by_uid(i);
		if (sid != SID_NOT_FOUND && uid_ref[i]) {
			if (sid == UID_HIFI3) {
				pr_debug("#@# %s(%d) [LVL_QOS] uid_ref %s %x\n",
					__func__, __LINE__, slbc_uid_str[i], uid_ref[i]);
			} else if (sid == UID_AOV |
					sid == UID_AOV_DC |
					sid == UID_AOV_APU) {
				pr_debug("#@# %s(%d) [LVL_QOS] uid_ref %s %x, screen off user\n",
					__func__, __LINE__, slbc_uid_str[i], uid_ref[i]);
			} else {
				pr_debug("#@# %s(%d) [LVL_QOS] uid_ref %s %x, screen on user\n",
					__func__, __LINE__, slbc_uid_str[i], uid_ref[i]);
				WARN_ON(uid_ref[i]);
			}
		}
	}

	return 0;
}

static int slbc_resume_cb(struct device *dev)
{
	int i;
	int sid;

	for (i = 0; i < UID_MAX; i++) {
		sid = _slbc_get_sid_by_uid(i);
		if (sid != SID_NOT_FOUND && uid_ref[i]) {
			if (sid == UID_HIFI3) {
				pr_debug("#@# %s(%d) [LVL_QOS] uid_ref %s %x\n",
					__func__, __LINE__, slbc_uid_str[i], uid_ref[i]);
			} else if (sid == UID_AOV |
					sid == UID_AOV_DC |
					sid == UID_AOV_APU) {
				pr_debug("#@# %s(%d) [LVL_QOS] uid_ref %s %x, screen off user\n",
					__func__, __LINE__, slbc_uid_str[i], uid_ref[i]);
				WARN_ON(uid_ref[i]);
			} else {
				pr_debug("#@# %s(%d) [LVL_QOS] uid_ref %s %x, screen on user\n",
					__func__, __LINE__, slbc_uid_str[i], uid_ref[i]);
			}
		}
	}

	return 0;
}

static const struct dev_pm_ops slbc_pm_ops = {
	.suspend = slbc_suspend,
	.resume = slbc_resume,
	.suspend_late = slbc_suspend_cb,
	.resume_early = slbc_resume_cb,
};

static const struct mtk_slbc_pdata slbc_pdata_mt8196 = {
	.slbc_enable = 1,
	.slbc_all_cache_enable = 1
};

static const struct of_device_id slbc_of_match[] = {
	{ .compatible = "mediatek,mt8196-slbc", .data = &slbc_pdata_mt8196 },
	{}
};

static struct platform_driver slbc_pdrv = {
	.probe = slbc_probe,
	.remove = slbc_remove,
	.driver = {
		.name = "slbc_mt8196",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(slbc_of_match),
		.pm = &slbc_pm_ops,
	},
};

module_platform_driver(slbc_pdrv);
MODULE_SOFTDEP("pre: tinysys-scmi");
MODULE_DESCRIPTION("MT8196 SLBC driver");
MODULE_VERSION("1.0");
MODULE_IMPORT_NS(DMA_BUF);
MODULE_LICENSE("GPL");
