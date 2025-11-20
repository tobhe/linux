// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 MediaTek Inc.
 */

#include <linux/device.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/suspend.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/vmalloc.h>
#include <linux/remoteproc.h>
#include <linux/rpmsg/mtk_rpmsg.h>
#include <linux/rpmsg/mtk_vcp_rpmsg.h>

#include "mtk_vcp_reg.h"
#include "mtk_vcp_rproc.h"
#include "remoteproc_internal.h"

#if IS_ENABLED(CONFIG_OF_RESERVED_MEM)
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <linux/dma-mapping.h>
#include <linux/iommu.h>
#include <uapi/linux/dma-heap.h>
#include <linux/of_reserved_mem.h>
#endif

/* vcp ready status for notify */
static bool vcp_ready[VCP_CORE_TOTAL];
static DEFINE_MUTEX(vcp_ready_mutex);
static DEFINE_MUTEX(vcp_pw_clk_mutex);
static DEFINE_MUTEX(vcp_A_notify_mutex);
static DEFINE_MUTEX(vcp_feature_mutex);
static DEFINE_SPINLOCK(vcp_awake_spinlock);

/* vcp awake variable */
static int vcp_awake_counts[VCP_CORE_TOTAL];

static dma_addr_t vcp_mem_base_iova;
static size_t vcp_mem_size;

static struct workqueue_struct *vcp_workqueue;
static BLOCKING_NOTIFIER_HEAD(mmup_notifier_list);
static BLOCKING_NOTIFIER_HEAD(vcp_notifier_list);

struct mtk_mbox_device vcp_mboxdev = {
	.name = "vcp_mboxdev",
	.pin_recv_table = 0,
	.pin_send_table = 0,
	.info_table = 0,
	.recv_count = 0,
	.send_count = 0,
};

struct mtk_ipi_device vcp_ipidev = {
	.name = "vcp_ipidev",
	.id = IPI_DEV_VCP,
	.mbdev = &vcp_mboxdev,
	.pre_cb = (ipi_tx_cb_t)vcp_awake_lock,
	.post_cb = (ipi_tx_cb_t)vcp_awake_unlock,
	.prdata = 0,
};

static const struct mtk_vcp_ipi_ops mt8196_vcp_ipi_ops = {
	.ipi_send = vcp_ipi_send,
	.ipi_send_compl = vcp_ipi_send_compl,
	.ipi_register = vcp_mbox_ipi_register,
	.ipi_unregister = vcp_mbox_ipi_unregister,
};

static struct vcp_reserve_mblock vcp_reserve_mblock[NUMS_MEM_ID] = {
	{
		.num = VCP_RTOS_MEM_ID,
		.start_phys = 0x0,
		.start_iova = 0x0,
		.start_virt = 0x0,
		.size = 0x0,
	},
	{
		.num = VDEC_MEM_ID,
		.start_phys = 0x0,
		.start_iova = 0x0,
		.start_virt = 0x0,
		.size = 0x0,
	},
	{
		.num = VENC_MEM_ID,
		.start_phys = 0x0,
		.start_iova = 0x0,
		.start_virt = 0x0,
		.size = 0x0,
	},
	{
		.num = MMDVFS_VCP_MEM_ID,
		.start_phys = 0x0,
		.start_iova = 0x0,
		.start_virt = 0x0,
		.size = 0x0,
	},
	{
		.num = MMDVFS_MMUP_MEM_ID,
		.start_phys = 0x0,
		.start_iova = 0x0,
		.start_virt = 0x0,
		.size = 0x0,
	},
	{
		.num = MMQOS_MEM_ID,
		.start_phys = 0x0,
		.start_iova = 0x0,
		.start_virt = 0x0,
		.size = 0x0,
	},
};

/* vcp feature list */
struct vcp_feature_tb feature_table[NUM_FEATURE_ID] = {
	{
		.feature    = RTOS_FEATURE_ID,
		.core_id    = VCP_CORE_TOTAL,
		.enable     = 0,
	},
	{
		.feature    = VDEC_FEATURE_ID,
		.core_id    = VCP_ID,
		.enable     = 0,
	},
	{
		.feature    = VENC_FEATURE_ID,
		.core_id    = VCP_ID,
		.enable     = 0,
	},
	{
		.feature    = MMDVFS_MMUP_FEATURE_ID,
		.core_id    = MMUP_ID,
		.enable	    = 0,
	},
	{
		.feature    = MMDVFS_VCP_FEATURE_ID,
		.core_id    = VCP_ID,
		.enable	    = 0,
	},
	{
		.feature    = MMDEBUG_FEATURE_ID,
		.core_id    = MMUP_ID,
		.enable	    = 0,
	},
	{
		.feature    = VMM_FEATURE_ID,
		.core_id    = MMUP_ID,
		.enable	    = 0,
	},
	{
		.feature    = VDISP_FEATURE_ID,
		.core_id    = MMUP_ID,
		.enable	    = 0,
	},
	{
		.feature    = MMQOS_FEATURE_ID,
		.core_id    = VCP_ID,
		.enable	    = 0,
	},
};

/**
 * vcp_get() - get a reference to VCP.
 *
 * @pdev:	the platform device of the module requesting VCP platform
 *		device for using VCP API.
 *
 * Return: Return NULL if failed.  otherwise reference to VCP.
 **/
struct mtk_vcp_device *vcp_get(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *vcp_node;
	struct platform_device *vcp_pdev;

	vcp_node = of_parse_phandle(dev->of_node, "mediatek,vcp", 0);
	if (!vcp_node) {
		dev_err(dev, "can't get VCP node\n");
		return NULL;
	}

	vcp_pdev = of_find_device_by_node(vcp_node);
	of_node_put(vcp_node);

	if (WARN_ON(!vcp_pdev)) {
		dev_err(dev, "VCP pdev failed\n");
		return NULL;
	}

	return platform_get_drvdata(vcp_pdev);
}
EXPORT_SYMBOL_GPL(vcp_get);

/**
 * vcp_put() - "free" the VCP
 *
 * @scp:	mtk_vcp_device structure from vcp_put().
 **/
void vcp_put(struct mtk_vcp_device *vcp)
{
	put_device(vcp->dev);
}
EXPORT_SYMBOL_GPL(vcp_put);

struct mtk_ipi_device *vcp_get_ipidev(struct mtk_vcp_device *vcp)
{
	return vcp->ipi_dev;
}
EXPORT_SYMBOL_GPL(vcp_get_ipidev);

/*
 * @return: true if vcp is ready for running tasks
 */
static bool is_vcp_ready_by_coreid(enum vcp_core_id core_id)
{
	switch (core_id) {
	case VCP_ID:
		return vcp_ready[VCP_ID];
	case MMUP_ID:
		return vcp_ready[MMUP_ID];
	case VCP_CORE_TOTAL:
	default:
		return vcp_ready[VCP_ID] == true &&
		       vcp_ready[MMUP_ID] == true;
	}
}

static enum vcp_core_id get_core_by_feature(enum feature_id id)
{
	int i = 0;

	for (i = 0; i < NUM_FEATURE_ID; i++) {
		if (feature_table[i].feature == id)
			return feature_table[i].core_id;
	}

	return VCP_ID;
}

static bool is_vcp_ready(enum feature_id id)
{
	enum vcp_core_id core_id = get_core_by_feature(id);

	return is_vcp_ready_by_coreid(core_id);
}

static u32 wait_core_hart_shutdown(struct mtk_vcp_device *vcp,
				   enum vcp_core_id core_id)
{
	u32 retry;
	bool twohart_support;
	u32 core_hart0 = 0;
	u32 core_hart1 = 0;

	twohart_support = vcp->vcp_cluster->twohart[core_id];

	for (retry = VCP_AWAKE_TIMEOUT; retry > 0; retry--) {
		switch (core_id) {
		case VCP_ID:
			core_hart0 = readl(vcp->vcp_cluster->cfg + VCP_C0_GPR5_H0_REBOOT);
			if (twohart_support)
				core_hart1 = readl(vcp->vcp_cluster->cfg + VCP_C0_GPR6_H1_REBOOT);
			break;
		case MMUP_ID:
			core_hart0 = readl(vcp->vcp_cluster->cfg + VCP_C1_GPR5_H0_REBOOT);
			if (twohart_support)
				core_hart1 = readl(vcp->vcp_cluster->cfg + VCP_C1_GPR6_H1_REBOOT);
			break;
		case VCP_CORE_TOTAL:
		default:
			break;
		}

		if (twohart_support) {
			if ((core_hart0 == CORE_RDY_TO_REBOOT)
			     && (core_hart1 == CORE_RDY_TO_REBOOT))
				break;
		} else {
			if (core_hart0 == CORE_RDY_TO_REBOOT)
				break;
		}
		usleep_range(USDELAY_RANGE_MIN, USDELAY_RANGE_MAX);
	}

	return retry;
}

static void vcp_A_register_notify(enum feature_id id,
				  struct notifier_block *nb)
{
	enum vcp_core_id core_id = get_core_by_feature(id);

	mutex_lock(&vcp_A_notify_mutex);
	switch (core_id) {
	case VCP_ID:
		blocking_notifier_chain_register(&vcp_notifier_list, nb);
		if (is_vcp_ready_by_coreid(VCP_ID))
			nb->notifier_call(nb, VCP_EVENT_READY, NULL);
		break;
	case MMUP_ID:
		blocking_notifier_chain_register(&mmup_notifier_list, nb);
		if (is_vcp_ready_by_coreid(MMUP_ID))
			nb->notifier_call(nb, VCP_EVENT_READY, NULL);
		break;
	default:
		break;
	}
	mutex_unlock(&vcp_A_notify_mutex);
}

static void vcp_A_unregister_notify(enum feature_id id,
				    struct notifier_block *nb)
{
	enum vcp_core_id core_id = get_core_by_feature(id);

	mutex_lock(&vcp_A_notify_mutex);
	switch (core_id) {
	case VCP_ID:
		blocking_notifier_chain_unregister(&vcp_notifier_list, nb);
		break;
	case MMUP_ID:
		blocking_notifier_chain_unregister(&mmup_notifier_list, nb);
		break;
	default:
		break;
	}
	mutex_unlock(&vcp_A_notify_mutex);
}

static void vcp_extern_notify(enum vcp_core_id core_id,
			      enum VCP_NOTIFY_EVENT notify_status)
{
	switch (core_id) {
	case VCP_ID:
		blocking_notifier_call_chain(&vcp_notifier_list, notify_status, NULL);
		break;
	case MMUP_ID:
		blocking_notifier_call_chain(&mmup_notifier_list, notify_status, NULL);
		break;
	default:
		break;
	}
}

static void vcp_reset_awake_counts(void)
{
	int i;

	/* vcp ready static flag initialise */
	for (i = 0; i < VCP_CORE_TOTAL ; i++)
		vcp_awake_counts[i] = 0;
}

static void vcp_wait_awake_count(struct mtk_vcp_device *vcp)
{
	int i = 0;
	unsigned long spin_flags;

	while (vcp_awake_counts[VCP_ID] != 0 || vcp_awake_counts[MMUP_ID] != 0) {
		i++;
		if (i > WAKE_POLL_TIMES) {
			dev_info(vcp->dev, "wait vcp_awake_counts timeout %d %d\n",
				 vcp_awake_counts[VCP_ID], vcp_awake_counts[MMUP_ID]);
			break;
		}
		usleep_range(USDELAY_RANGE_MIN, USDELAY_RANGE_MAX);
	}

	spin_lock_irqsave(&vcp_awake_spinlock, spin_flags);
	vcp_reset_awake_counts();
	spin_unlock_irqrestore(&vcp_awake_spinlock, spin_flags);
}

static void vcp_schedule_work(struct vcp_work_struct *vcp_ws)
{
	if (!vcp_workqueue)
		dev_err(vcp_ws->dev, "vcp_workqueue is NULL\n");
	else
		queue_work(vcp_workqueue, &vcp_ws->work);
}

/*
 * callback function for work struct
 * notify apps to start their tasks
 * @param ws:   work struct
 */
static void vcp_A_notify_ws(struct work_struct *ws)
{
	struct vcp_work_struct *sws =
		container_of(ws, struct vcp_work_struct, work);
	enum vcp_core_id core_id = sws->flags;

	if (core_id < VCP_CORE_TOTAL) {
		mutex_lock(&vcp_ready_mutex);
		vcp_ready[core_id] = 1;
		vcp_ipidev.prdata = (void*)core_id;
		mutex_unlock(&vcp_ready_mutex);

		mutex_lock(&vcp_A_notify_mutex);

		vcp_extern_notify(core_id, VCP_EVENT_READY);
		mutex_unlock(&vcp_A_notify_mutex);

		/*clear reset status and unlock wake lock*/
		dev_info(sws->dev, "%s core id %u ready\n", __func__, core_id);
	} else
		dev_warn(sws->dev, "%s wrong core id %u\n", __func__, core_id);
}

static void vcp_A_set_ready(struct mtk_vcp_device *vcp,
			    enum vcp_core_id core_id)
{
	if (core_id < VCP_CORE_TOTAL)
		vcp_schedule_work(&vcp->vcp_cluster->vcp_ready_notify_wk[core_id]);
}

/*
 * handle notification from vcp
 * mark vcp is ready for running tasks
 * It is important to call vcp_ram_dump_init() in this IPI handler. This
 * timing is necessary to ensure that the region_info has been initialized.
 * @param id:   ipi id
 * @param prdata: ipi handler parameter
 * @param data: ipi data
 * @param len:  length of ipi data
 */
static int vcp_A_ready_ipi_handler(u32 id, void *prdata, void *data, u32 len)
{
	struct mtk_vcp_device *vcp = (struct mtk_vcp_device *)prdata;

	switch (id) {
	case IPI_IN_VCP_READY_0:
		if (!is_vcp_ready_by_coreid(VCP_ID))
			vcp_A_set_ready(vcp, VCP_ID);
		break;
	case IPI_IN_VCP_READY_1:
		if (!is_vcp_ready_by_coreid(MMUP_ID))
			vcp_A_set_ready(vcp, MMUP_ID);
		break;
	default:
		break;
	}

	return 0;
}

static void vcp_wait_ready_sync(struct mtk_vcp_device *vcp)
{
	u32 feature_id = 0;
	bool core_statue = false;

	read_poll_timeout_atomic(is_vcp_ready_by_coreid,
				 core_statue, core_statue,
				 USEC_PER_MSEC,
				 VCP_SYNC_TIMEOUT_MS * USEC_PER_MSEC,
				 false, VCP_CORE_TOTAL);
	if (!core_statue) {
		dev_warn(vcp->dev, "wait VCP_CORE_TOTAL ready timeout!\n");
		for (feature_id = 0; feature_id < NUM_FEATURE_ID; feature_id++)
			if (feature_table[feature_id].enable)
				dev_dbg(vcp->dev, "feature id %d cnt %d\n",
					feature_table[feature_id].feature,
					feature_table[feature_id].enable);
	}
}

static void vcp_wait_suspend_resume(struct mtk_vcp_device *vcp, bool suspend)
{
	int timeout = SUSPEND_WAIT_TIMEOUT_MS;

	if (suspend) {
		writel(B_CORE0_SUSPEND, vcp->vcp_cluster->cfg_core + AP_R_GPR2);
		writel(B_CORE1_SUSPEND, vcp->vcp_cluster->cfg_core + AP_R_GPR3);
		writel(SUSPEND_IPI_MAGIC, vcp->vcp_cluster->cfg + VCP_C0_GPR0_SUSPEND_RESUME_FLAG);
		writel(SUSPEND_IPI_MAGIC, vcp->vcp_cluster->cfg + VCP_C1_GPR0_SUSPEND_RESUME_FLAG);
		if (!readl(vcp->vcp_cluster->cfg_core + AP_R_GPR2) ||
		    !readl(vcp->vcp_cluster->cfg_core + AP_R_GPR3)) {
			dev_info(vcp->dev, "[%s] AP_R_GPR2/3 is null %x %x %x %x flag %x %x\n",
				 suspend ? "suspend" : "resume",
				 readl(vcp->vcp_cluster->cfg_core + AP_R_GPR2),
				 readl(vcp->vcp_cluster->cfg_core + AP_R_GPR3),
				 readl(vcp->vcp_cluster->cfg + VCP_C0_GPR0_SUSPEND_RESUME_FLAG),
				 readl(vcp->vcp_cluster->cfg + VCP_C1_GPR0_SUSPEND_RESUME_FLAG),
				 readl(vcp->vcp_cluster->mbox_init + R_GPR2_CFGREG_SEC),
				 readl(vcp->vcp_cluster->mbox_init + R_GPR3_CFGREG_SEC));
		}
		writel(B_GIPC4_SETCLR_3, vcp->vcp_cluster->cfg_core + R_GIPC_IN_SET);
	} else {
		writel(B_CORE0_RESUME, vcp->vcp_cluster->cfg_core + AP_R_GPR2);
		writel(B_CORE1_RESUME, vcp->vcp_cluster->cfg_core + AP_R_GPR3);
		writel(RESUME_IPI_MAGIC, vcp->vcp_cluster->cfg + VCP_C0_GPR0_SUSPEND_RESUME_FLAG);
		writel(RESUME_IPI_MAGIC, vcp->vcp_cluster->cfg + VCP_C1_GPR0_SUSPEND_RESUME_FLAG);
		if (!readl(vcp->vcp_cluster->cfg_core + AP_R_GPR2) ||
		    !readl(vcp->vcp_cluster->cfg_core + AP_R_GPR3)) {
			dev_info(vcp->dev, "[%s] AP_R_GPR2/3 is null %x %x %x %x flag %x %x\n",
				 suspend ? "suspend" : "resume",
				 readl(vcp->vcp_cluster->cfg_core + AP_R_GPR2),
				 readl(vcp->vcp_cluster->cfg_core + AP_R_GPR3),
				 readl(vcp->vcp_cluster->cfg + VCP_C0_GPR0_SUSPEND_RESUME_FLAG),
				 readl(vcp->vcp_cluster->cfg + VCP_C1_GPR0_SUSPEND_RESUME_FLAG),
				 readl(vcp->vcp_cluster->mbox_init + R_GPR2_CFGREG_SEC),
				 readl(vcp->vcp_cluster->mbox_init + R_GPR3_CFGREG_SEC));
		}
		writel(B_GIPC4_SETCLR_3, vcp->vcp_cluster->cfg_core + R_GIPC_IN_SET);
	}

	while (--timeout) {
		if (suspend && (readl(vcp->vcp_cluster->mbox_init + R_GPR3_CFGREG_SEC) & (VCP_AP_SUSPEND))
		    && (readl(vcp->vcp_cluster->mbox_init + R_GPR2_CFGREG_SEC) & (MMUP_AP_SUSPEND)))
			break;
		else if (!suspend && !(readl(vcp->vcp_cluster->mbox_init + R_GPR3_CFGREG_SEC) & (VCP_AP_SUSPEND))
			 && !(readl(vcp->vcp_cluster->mbox_init + R_GPR2_CFGREG_SEC) & (MMUP_AP_SUSPEND)))
			break;
		usleep_range(USDELAY_RANGE_MIN, USDELAY_RANGE_MAX);
	}
	if (timeout <= 0) {
		dev_info(vcp->dev, "vcp %s timeout GPIC 0x%x 0x%x 0x%x 0x%x flag 0x%x 0x%x\n",
			 suspend ? "suspend" : "resume",
			 readl(vcp->vcp_cluster->cfg_core + R_GIPC_IN_SET),
			 readl(vcp->vcp_cluster->cfg_core + R_GIPC_IN_CLR),
			 readl(vcp->vcp_cluster->cfg_core + AP_R_GPR2),
			 readl(vcp->vcp_cluster->cfg_core + AP_R_GPR3),
			 readl(vcp->vcp_cluster->mbox_init + R_GPR2_CFGREG_SEC),
			 readl(vcp->vcp_cluster->mbox_init + R_GPR3_CFGREG_SEC));
	}
}

static void vcp_wait_core_stop(struct mtk_vcp_device *vcp, enum vcp_core_id core_id)
{
	u32 core_halt = 1;
	u32 core_axi = 0;
	u32 core_status = 0;
	u32 status = 0;

	/* make sure vcp is in idle state */
	int timeout = SUSPEND_WAIT_TIMEOUT_MS;

	while (--timeout) {
		switch (core_id) {
		case VCP_ID:
			core_status = readl(vcp->vcp_cluster->cfg + R_CORE0_STATUS);
			status = (vcp->vcp_cluster->twohart[VCP_ID] ?
				 (B_CORE_GATED | B_HART0_HALT | B_HART1_HALT) :
				 (B_CORE_GATED | B_HART0_HALT));
			break;
		case MMUP_ID:
			core_status = readl(vcp->vcp_cluster->cfg + R_CORE1_STATUS);
			status = (vcp->vcp_cluster->twohart[MMUP_ID] ?
				 (B_CORE_GATED | B_HART0_HALT | B_HART1_HALT) :
				 (B_CORE_GATED | B_HART0_HALT));
			break;
		case VCP_CORE_TOTAL:
		default:
			dev_info(vcp->dev, "unexcepted core_id = %d\n", core_id);
			break;
		}

		core_halt = ((core_status & status) == status);
		core_axi = core_status & (B_CORE_AXIS_BUSY);

		if (core_halt && (!core_axi)) {
			dev_err(vcp->dev, "[%s] core status 0x%x, GPIC 0x%x flag 0x%x\n",
				core_id ? "MMUP_ID" : "VCP_ID", core_status,
				readl(vcp->vcp_cluster->cfg_core + R_GIPC_IN_SET),
				readl(vcp->vcp_cluster->mbox_init + R_GPR3_CFGREG_SEC));
			break;
		}
		usleep_range(USDELAY_RANGE_MIN, USDELAY_RANGE_MAX);
	}

	if (timeout == 0) {
		dev_err(vcp->dev, "wait [%s] core stop timeout, current status 0x%x\n",
			core_id ? "MMUP_ID" : "VCP_ID", core_status);
	}
}

static void vcp_wait_rdy_signal(struct mtk_vcp_device *vcp, bool rdy)
{
	u32 rdy_signal;
	int timeout = SUSPEND_WAIT_TIMEOUT_MS;

	if (!IS_ERR((void const *) vcp->vcp_cluster->vcp_rdy)) {
		while (--timeout) {
			rdy_signal = readl(vcp->vcp_cluster->vcp_rdy + VLP_AO_RSVD7) & (READY_BIT);
			if (rdy && rdy_signal)
				break;
			else if (!rdy && !rdy_signal)
				break;
			usleep_range(USDELAY_RANGE_MIN, USDELAY_RANGE_MAX);
		}
		if (timeout <= 0)
			dev_err(vcp->dev, "wait vcp %s timeout 0x%x\n",
				rdy ? "set rdy bit" : "clr rdy bit",
				readl(vcp->vcp_cluster->vcp_rdy + VLP_AO_RSVD7));
	} else {
		dev_err(vcp->dev, "illegal vcp rdy signal\n");
	}
}

/*
 * reset vcp and waiting for vcp notify
 * @param reset:    bit[0-3]=0 for vcp enable, =1 for reboot
 *                  bit[4-7]=0 for All, =1 for vcp_A, =2 for vcp_B
 * @return:         0 if success
 */
static int reset_vcp(struct mtk_vcp_device *vcp)
{
	struct arm_smccc_res res;
	bool mmup_status = false;
	bool vcp_status = false;

	if (vcp->vcp_cluster->core_nums >= MMUP_ID) {
		/* write vcp reserved memory address/size to GRP1/GRP2
		 * to let vcp setup MPU
		 */
		writel((u32)VCP_PACK_IOVA(vcp_mem_base_iova),
			vcp->vcp_cluster->cfg + VCP_C1_GPR1_DRAM_RESV_ADDR);
		writel((u32)vcp_mem_size,
			vcp->vcp_cluster->cfg + VCP_C1_GPR2_DRAM_RESV_SIZE);

		arm_smccc_smc(MTK_SIP_TINYSYS_VCP_CONTROL,
			      MTK_TINYSYS_MMUP_KERNEL_OP_RESET_RELEASE,
			      1, 0, 0, 0, 0, 0, &res);

		read_poll_timeout_atomic(is_vcp_ready_by_coreid,
					 mmup_status, mmup_status,
					 USEC_PER_MSEC,
					 VCP_READY_TIMEOUT_MS * USEC_PER_MSEC,
					 false, MMUP_ID);
		if (!mmup_status) {
			dev_info(vcp->dev, "MMUP_ID bootup timeout. Stop vcp booting\n");
			return -EINVAL;
		}
	}

	/* write vcp reserved memory address/size to GRP1/GRP2
	 * to let vcp setup MPU
	 */
	writel((u32)VCP_PACK_IOVA(vcp_mem_base_iova),
		vcp->vcp_cluster->cfg + VCP_C0_GPR1_DRAM_RESV_ADDR);
	writel((u32)vcp_mem_size,
		vcp->vcp_cluster->cfg + VCP_C0_GPR2_DRAM_RESV_SIZE);

	arm_smccc_smc(MTK_SIP_TINYSYS_VCP_CONTROL,
		      MTK_TINYSYS_VCP_KERNEL_OP_RESET_RELEASE,
		      1, 0, 0, 0, 0, 0, &res);

	read_poll_timeout_atomic(is_vcp_ready_by_coreid,
				 vcp_status, vcp_status,
				 USEC_PER_MSEC,
				 VCP_READY_TIMEOUT_MS * USEC_PER_MSEC,
				 false, VCP_ID);
	if (!vcp_status) {
		dev_info(vcp->dev, "VCP_ID bootup timeout. Stop vcp booting\n");
		return -EINVAL;
	}

	return 0;
}

static bool is_vcp_suspending(struct mtk_vcp_device *vcp)
{
	return vcp->vcp_cluster->is_suspending ? true : false;
}

static int vcp_enable_pm_clk(struct mtk_vcp_device *vcp, enum feature_id id)
{
	int ret = 0;
	bool suspend_status = false;
	struct slp_ctrl_data ipi_data;

	mutex_lock(&vcp_pw_clk_mutex);
	read_poll_timeout_atomic(is_vcp_suspending,
				 suspend_status, !suspend_status,
				 USEC_PER_MSEC,
				 SUSPEND_WAIT_TIMEOUT_MS * USEC_PER_MSEC,
				 false, vcp);
	if (suspend_status) {
		dev_warn(vcp->dev, "%s blocked by vcp suspend, pwclkcnt(%d)\n",
			 __func__,
			 vcp->vcp_cluster->pwclkcnt);
		return -ETIMEDOUT;
	}

	if (vcp->vcp_cluster->pwclkcnt == 0) {
		if (!is_vcp_ready_by_coreid(VCP_CORE_TOTAL)) {
			if (reset_vcp(vcp)) {
				mutex_unlock(&vcp_pw_clk_mutex);
				return -EINVAL;
			}
		}
	}
	vcp->vcp_cluster->pwclkcnt++;
	if (id != RTOS_FEATURE_ID) {
		ipi_data.cmd = SLP_WAKE_LOCK;
		ipi_data.feature = id;
		ret = vcp_ipi_send_compl(&vcp_ipidev, IPI_OUT_C_SLEEP_0,
					 &ipi_data, PIN_OUT_C_SIZE_SLEEP_0, 500);
		if (ret < 0) {
			dev_warn(vcp->dev, "%s vcp_ipi_send_compl failed. ret %d\n",
				 __func__, ret);
			return ret;
		}
	}
	mutex_unlock(&vcp_pw_clk_mutex);

	return ret;
}

static int vcp_disable_pm_clk(struct mtk_vcp_device *vcp, enum feature_id id)
{
	int ret = 0;
	int i = 0;
	bool suspend_status = false;
	struct slp_ctrl_data ipi_data;

	mutex_lock(&vcp_pw_clk_mutex);
	read_poll_timeout_atomic(is_vcp_suspending,
				 suspend_status, !suspend_status,
				 USEC_PER_MSEC,
				 SUSPEND_WAIT_TIMEOUT_MS * USEC_PER_MSEC,
				 false, vcp);
	if (suspend_status) {
		dev_warn(vcp->dev, "%s blocked by vcp suspend, pwclkcnt(%d)\n",
			 __func__,
			 vcp->vcp_cluster->pwclkcnt);
		return -ETIMEDOUT;
	}

	dev_dbg(vcp->dev, "%s id %d entered %d ready %d %d\n",
		__func__, id,
		vcp->vcp_cluster->pwclkcnt,
		is_vcp_ready_by_coreid(VCP_ID),
		is_vcp_ready_by_coreid(MMUP_ID));

	if (id != RTOS_FEATURE_ID) {
		ipi_data.cmd = SLP_WAKE_UNLOCK;
		ipi_data.feature = id;
		ret = vcp_ipi_send_compl(&vcp_ipidev, IPI_OUT_C_SLEEP_0,
					 &ipi_data, PIN_OUT_C_SIZE_SLEEP_0, 500);
		if (ret < 0) {
			dev_warn(vcp->dev, "%s vcp_ipi_send_compl failed. ret %d\n",
				 __func__, ret);
			return ret;
		}
	}
	vcp->vcp_cluster->pwclkcnt--;

	if (vcp->vcp_cluster->pwclkcnt <= 0) {
		for (i = 0; i < NUM_FEATURE_ID; i++)
			dev_warn(vcp->dev, "%s Check feature id %d enable cnt %d\n",
				 __func__, feature_table[i].feature, feature_table[i].enable);
		vcp->vcp_cluster->pwclkcnt = 0;
	}
	mutex_unlock(&vcp_pw_clk_mutex);

	return 0;
}

static int mtk_vcp_suspend(struct device *dev)
{
	struct mtk_vcp_device *vcp = platform_get_drvdata(to_platform_device(dev));
	u32 i;

	mutex_lock(&vcp_A_notify_mutex);
	vcp_extern_notify(VCP_ID, VCP_EVENT_SUSPEND);
	vcp_extern_notify(MMUP_ID, VCP_EVENT_SUSPEND);
	mutex_unlock(&vcp_A_notify_mutex);

	mutex_lock(&vcp_pw_clk_mutex);
	if (!IS_ERR((void const *) vcp->vcp_cluster->vcp_rdy))
		dev_info(vcp->dev, "vcp suspend entered %d %d rdy %x\n",
			 vcp->vcp_cluster->pwclkcnt,
			 vcp->vcp_cluster->is_suspending,
			 readl(vcp->vcp_cluster->vcp_rdy + VLP_AO_RSVD7));
	else
		dev_info(vcp->dev, "vcp suspend entered %d %d\n",
			 vcp->vcp_cluster->pwclkcnt,
			 vcp->vcp_cluster->is_suspending);
	if ((!vcp->vcp_cluster->is_suspending) &&
	     vcp->vcp_cluster->pwclkcnt) {
		vcp->vcp_cluster->is_suspending = true;

		vcp_wait_ready_sync(vcp);
		flush_workqueue(vcp_workqueue);

		mutex_lock(&vcp_ready_mutex);
		for (i = 0; i < VCP_CORE_TOTAL; i++)
			vcp_ready[i] = 0;
		mutex_unlock(&vcp_ready_mutex);

		vcp_wait_suspend_resume(vcp, true);
		vcp_wait_core_stop(vcp, VCP_ID);
		vcp_wait_core_stop(vcp, MMUP_ID);
		vcp_wait_awake_count(vcp);

		pm_runtime_put_sync(dev);

		/* wait vcp clr rdy bit */
		vcp_wait_rdy_signal(vcp, false);
	}
	vcp->vcp_cluster->is_suspending = true;
	mutex_unlock(&vcp_pw_clk_mutex);

	return 0;
}

static int mtk_vcp_resume(struct device *dev)
{
	struct mtk_vcp_device *vcp = platform_get_drvdata(to_platform_device(dev));

	mutex_lock(&vcp_pw_clk_mutex);
	if (!IS_ERR((void const *) vcp->vcp_cluster->vcp_rdy))
		dev_info(vcp->dev, "vcp suspend entered %d %d rdy %x\n",
			 vcp->vcp_cluster->pwclkcnt,
			 vcp->vcp_cluster->is_suspending,
			 readl(vcp->vcp_cluster->vcp_rdy + VLP_AO_RSVD7));
	else
		dev_info(vcp->dev, "vcp suspend entered %d %d\n",
			 vcp->vcp_cluster->pwclkcnt,
			 vcp->vcp_cluster->is_suspending);

	if (vcp->vcp_cluster->is_suspending &&
	    vcp->vcp_cluster->pwclkcnt) {
		pm_runtime_get_sync(dev);

		/* wait vcp set rdy bit */
		vcp_wait_rdy_signal(vcp, true);
		vcp_wait_suspend_resume(vcp, false);
	}
	vcp->vcp_cluster->is_suspending = false;
	mutex_unlock(&vcp_pw_clk_mutex);

	mutex_lock(&vcp_A_notify_mutex);
	vcp_extern_notify(MMUP_ID, VCP_EVENT_RESUME);
	vcp_extern_notify(VCP_ID, VCP_EVENT_RESUME);
	mutex_unlock(&vcp_A_notify_mutex);

	return 0;
}

static phys_addr_t vcp_get_reserve_mem_phys(enum vcp_reserve_mem_id_t id)
{
	if (id >= 0 && id < NUMS_MEM_ID)
		return vcp_reserve_mblock[id].start_phys;

	return 0;
}

static dma_addr_t vcp_get_reserve_mem_iova(enum vcp_reserve_mem_id_t id)
{
	if (id >= 0 && id < NUMS_MEM_ID)
		return vcp_reserve_mblock[id].start_iova;

	return 0;
}

static void __iomem *vcp_get_reserve_mem_virt(enum vcp_reserve_mem_id_t id)
{
	if (id >= 0 && id < NUMS_MEM_ID)
		return vcp_reserve_mblock[id].start_virt;

	return NULL;
}

static u32 vcp_get_reserve_mem_size(enum vcp_reserve_mem_id_t id)
{
	if (id >= 0 && id < NUMS_MEM_ID)
		return vcp_reserve_mblock[id].size;

	return 0;
}

static void __iomem *vcp_get_internal_sram_virt(struct mtk_vcp_device *vcp)
{
	return vcp->vcp_cluster->sram_base;
}

#if IS_ENABLED(CONFIG_OF_RESERVED_MEM)
static int vcp_reserve_memory_ioremap(struct mtk_vcp_device *vcp)
{
#define MEMORY_TBL_ELEM_NUM (2)
	u32 num = (u32)(sizeof(vcp_reserve_mblock)
			/ sizeof(vcp_reserve_mblock[0]));
	enum vcp_reserve_mem_id_t id;
	u32 vcp_mem_num = 0;
	u32 i = 0, m_idx = 0, m_size = 0;
	u32 offset;
	struct device_node *rmem_node;
	struct resource res;
	struct iommu_domain *domain;
	void __iomem *share_memory_virt;
	phys_addr_t mblock_start_phys;
	dma_addr_t share_memory_iova;
	size_t share_memory_size;
	size_t mblock_start_size;
	int ret;

	if (num != NUMS_MEM_ID) {
		dev_err(vcp->dev, "actual memory num(%u) is not match mem ID table (%u)\n",
			num, NUMS_MEM_ID);
		WARN_ON(1);
		return -EINVAL;
	}

	rmem_node = of_parse_phandle(vcp->dev->of_node, "memory-region", 0);
	if (!rmem_node) {
		dev_err(vcp->dev, "No reserved memory region found.\n");
		return -EINVAL;
	}

	ret = of_address_to_resource(rmem_node, 0, &res);
	if (ret) {
		dev_err(vcp->dev, "failed to parse reserved memory: %d\n", ret);
		return ret;
	}

	mblock_start_phys = (phys_addr_t)res.start;
	mblock_start_size = (u32)resource_size(&res);

	/* Set reserved memory table */
	vcp_mem_num = of_property_count_u32_elems(vcp->dev->of_node, "vcp-mem-tbl")
		      / MEMORY_TBL_ELEM_NUM;
	if (vcp_mem_num <= 0) {
		dev_info(vcp->dev, "vcp-mem-tbl not found\n");
		vcp_mem_num = 0;
	}

	for (i = 0; i < vcp_mem_num; i++) {
		ret = of_property_read_u32_index(vcp->dev->of_node, "vcp-mem-tbl",
						 i * MEMORY_TBL_ELEM_NUM, &m_idx);
		if (ret) {
			dev_err(vcp->dev, "cannot get memory index(%d)\n", i);
			return -EINVAL;
		}

		ret = of_property_read_u32_index(vcp->dev->of_node, "vcp-mem-tbl",
						 (i * MEMORY_TBL_ELEM_NUM) + 1, &m_size);
		if (ret) {
			dev_err(vcp->dev, "Cannot get memory size(%d)(%d)\n", i, m_idx);
			return -EINVAL;
		}

		if (m_idx >= NUMS_MEM_ID) {
			dev_dbg(vcp->dev, "skip unexpected index, %d\n", m_idx);
			continue;
		}

		vcp_reserve_mblock[m_idx].size = m_size;
		dev_dbg(vcp->dev, "reserved: <%d  %d>\n", m_idx, m_size);
	}

	vcp_reserve_mblock[VCP_RTOS_MEM_ID].start_phys = mblock_start_phys;
	vcp_reserve_mblock[VCP_RTOS_MEM_ID].start_virt = devm_ioremap(vcp->dev,
				vcp_reserve_mblock[VCP_RTOS_MEM_ID].start_phys,
				vcp_reserve_mblock[VCP_RTOS_MEM_ID].size);
	domain = iommu_get_domain_for_dev(vcp->dev);
	ret = iommu_map(domain, IMG_MEMORY_STATIC_IOVA,
			vcp_reserve_mblock[VCP_RTOS_MEM_ID].start_phys,
			vcp_reserve_mblock[VCP_RTOS_MEM_ID].size,
			IOMMU_READ | IOMMU_WRITE | IOMMU_PRIV, GFP_KERNEL);
	if (ret) {
		dev_err(vcp->dev, "%s iommu map fail, ret:%d.\n", __func__, ret);
		return ret;
	}
	vcp_reserve_mblock[VCP_RTOS_MEM_ID].start_iova = IMG_MEMORY_STATIC_IOVA;

	share_memory_size = 0;
	for (id = VDEC_MEM_ID; id < NUMS_MEM_ID; id++) {
		if (vcp_reserve_mblock[id].size == 0)
			continue;
		share_memory_size += vcp_reserve_mblock[id].size;
	}

	ret = dma_set_mask_and_coherent(vcp->dev, DMA_BIT_MASK(DMA_MAX_MASK_BIT));
	if (ret) {
		dev_err(vcp->dev, "64-bit DMA enable failed\n");
		return ret;
	}

	if (!vcp->dev->dma_parms) {
		vcp->dev->dma_parms = devm_kzalloc(vcp->dev, sizeof(*vcp->dev->dma_parms), GFP_KERNEL);
		if (vcp->dev->dma_parms) {
			ret = dma_set_max_seg_size(vcp->dev, (u32)DMA_BIT_MASK(33));
			if (ret) {
				dev_err(vcp->dev, "Failed to set DMA segment size\n");
				return ret;
			}
		} else {
			dev_err(vcp->dev, "Failed to set DMA parms\n");
			return -EINVAL;
		}
	}
	share_memory_virt = dma_alloc_coherent(vcp->dev, share_memory_size,
					       &share_memory_iova, GFP_KERNEL);
	if (!share_memory_virt)
		return -ENOMEM;
	offset = 0;
	for (id = VDEC_MEM_ID; id < NUMS_MEM_ID; id++)  {
		if (vcp_reserve_mblock[id].size == 0)
			continue;

		vcp_reserve_mblock[id].start_phys = vcp_reserve_mblock[VCP_RTOS_MEM_ID].start_phys +
						    vcp_reserve_mblock[VCP_RTOS_MEM_ID].size + offset;
		vcp_reserve_mblock[id].start_iova = share_memory_iova + offset;
		vcp_reserve_mblock[id].start_virt = share_memory_virt + offset;
		offset += (u32)vcp_reserve_mblock[id].size;

		dev_dbg(vcp->dev, "share memory [%d] pa:%pa, iova:%pa, virt:%p, len:0x%zx\n",
			id, &vcp_reserve_mblock[id].start_phys,
			&vcp_reserve_mblock[id].start_iova,
			vcp_reserve_mblock[id].start_virt,
			vcp_reserve_mblock[id].size);
	}

	vcp_mem_base_iova = share_memory_iova;
	vcp_mem_size = share_memory_size;

	return 0;
}
#endif

static int vcp_A_register_feature(struct mtk_vcp_device *vcp, enum feature_id id)
{
	u32 i;
	int ret = 0;

	if (id >= NUM_FEATURE_ID) {
		dev_info(vcp->dev, "%s unsupported feature id %d\n",
			__func__, id);
		return -EINVAL;
	}
	mutex_lock(&vcp_feature_mutex);
	for (i = 0; i < NUM_FEATURE_ID; i++) {
		if (feature_table[i].feature == id)
			feature_table[i].enable++;
	}
	ret = vcp_enable_pm_clk(vcp, id);
	mutex_unlock(&vcp_feature_mutex);

	return ret;
}

static int vcp_A_deregister_feature(struct mtk_vcp_device *vcp, enum feature_id id)
{
	u32 i;
	int ret = 0;

	if (id >= NUM_FEATURE_ID) {
		dev_info(vcp->dev, "%s unsupported feature id %d\n",
			__func__, id);
		return -EINVAL;
	}
	mutex_lock(&vcp_feature_mutex);
	for (i = 0; i < NUM_FEATURE_ID; i++) {
		if (feature_table[i].feature == id) {
			if (feature_table[i].enable == 0) {
				dev_warn(vcp->dev, "%s unbalanced feature id %d enable cnt %d\n",
					__func__, id, feature_table[i].enable);
				mutex_unlock(&vcp_feature_mutex);
				return -EINVAL;
			}
			feature_table[i].enable--;
		}
	}
	ret = vcp_disable_pm_clk(vcp, id);
	mutex_unlock(&vcp_feature_mutex);

	return ret;
}

/*
 * acquire vcp lock flag, keep vcp awake
 * @param vcp_core_id: vcp core id
 * return  0 :get lock success
 *        -EINVAL :get lock timeout
 */
int vcp_awake_lock(void *vcp_core_id)
{
	enum vcp_core_id core_id = (enum vcp_core_id) vcp_core_id;
	unsigned long spin_flags;
	int *vcp_awake_count;

	if (core_id >= VCP_CORE_TOTAL)
		return -EINVAL;

	vcp_awake_count = (int *)&vcp_awake_counts[core_id];

	if (!is_vcp_ready_by_coreid(core_id))
		return -EINVAL;

	/* vcp unlock awake */
	spin_lock_irqsave(&vcp_awake_spinlock, spin_flags);

	/* vcp lock awake success*/
	*vcp_awake_count = *vcp_awake_count + 1;

	spin_unlock_irqrestore(&vcp_awake_spinlock, spin_flags);

	return 0;
}

/*
 * release vcp awake lock flag
 * @param vcp_core_id: vcp core id
 * return  0 :release lock success
 *        -EINVAL :release lock fail
 */
int vcp_awake_unlock(void *vcp_core_id)
{
	enum vcp_core_id core_id = (enum vcp_core_id) vcp_core_id;
	unsigned long spin_flags;
	int *vcp_awake_count;

	if (core_id >= VCP_CORE_TOTAL)
		return -EINVAL;

	vcp_awake_count = (int *)&vcp_awake_counts[core_id];

	if (!is_vcp_ready_by_coreid(core_id))
		return -EINVAL;

	/* vcp unlock awake */
	spin_lock_irqsave(&vcp_awake_spinlock, spin_flags);
	if (*vcp_awake_count > 0)
		*vcp_awake_count = *vcp_awake_count - 1;

	/* spinlock context safe */
	spin_unlock_irqrestore(&vcp_awake_spinlock, spin_flags);

	return 0;
}

static void vcp_awake_init(void)
{
	vcp_reset_awake_counts();
}

static int mtk_vcp_start(struct rproc *rproc)
{
	struct mtk_vcp_device *vcp = (struct mtk_vcp_device *)rproc->priv;
	struct arm_smccc_res res;

	/* core 0 */
	arm_smccc_smc(MTK_SIP_TINYSYS_VCP_CONTROL,
		      MTK_TINYSYS_VCP_KERNEL_OP_RESET_SET,
		      1, 0, 0, 0, 0, 0, &res);

	/* core 1 */
	arm_smccc_smc(MTK_SIP_TINYSYS_VCP_CONTROL,
		      MTK_TINYSYS_MMUP_KERNEL_OP_RESET_SET,
		      1, 0, 0, 0, 0, 0, &res);

	if (vcp_A_register_feature(vcp, RTOS_FEATURE_ID) < 0) {
		dev_err(vcp->dev, "bootup fail\n");
		vcp_A_deregister_feature(vcp, RTOS_FEATURE_ID);
	} else
		dev_info(vcp->dev, "bootup successfully\n");

	dev_info(vcp->dev, "%s core0 status: 0x%x, core1 status: 0x%x\n",
		 __func__,
		 readl(vcp->vcp_cluster->cfg + R_CORE0_STATUS),
		 readl(vcp->vcp_cluster->cfg + R_CORE1_STATUS));

	return 0;
}

static int mtk_vcp_stop(struct rproc *rproc)
{
	struct mtk_vcp_device *vcp = (struct mtk_vcp_device *)rproc->priv;

	vcp_A_deregister_feature(vcp, RTOS_FEATURE_ID);

	mutex_lock(&vcp_A_notify_mutex);
	vcp_extern_notify(VCP_ID, VCP_EVENT_STOP);
	vcp_extern_notify(MMUP_ID, VCP_EVENT_STOP);
	mutex_unlock(&vcp_A_notify_mutex);

	return 0;
}
static const struct rproc_ops mtk_vcp_ops = {
	.load		= mtk_vcp_load,
	.start		= mtk_vcp_start,
	.stop		= mtk_vcp_stop,
};

static int vcp_register_ipi(struct platform_device *pdev, u32 id,
			    ipi_handler_t handler, void *priv)
{
	struct mtk_vcp_device *vcp;
	struct rpmsg_channel_info *chinfo = NULL;

	vcp = platform_get_drvdata(pdev);
	if (!vcp)
		return -ENXIO;
	if (id == VCP_IPI_NS_SERVICE) {
		chinfo = kzalloc(sizeof(*chinfo), GFP_KERNEL);
		if (!chinfo)
			return -ENOMEM;
		snprintf(chinfo->name, RPMSG_NAME_SIZE, "%s", dev_name(vcp->dev));
		handler(chinfo, sizeof(*chinfo) - sizeof(u32), priv);
		chinfo->dst = RPMSG_ADDR_ANY;
		vcp->ipi_dev->chinfo = chinfo;
	}

	return 0;
}

static void vcp_unregister_ipi(struct platform_device *pdev, u32 id)
{
	struct mtk_vcp_device *vcp = platform_get_drvdata(pdev);

	vcp_mbox_ipi_unregister(vcp->ipi_dev, id);
}

static int vcp_send_ipi(struct platform_device *pdev, u32 id, void *buf,
			u32 len, u32 wait)
{
	struct mtk_vcp_device *vcp = platform_get_drvdata(pdev);

	if (!vcp || !vcp->ipi_dev)
		return -EINVAL;

	return vcp_mbox_send(vcp->ipi_dev, id, buf, len, wait);
}

static struct mtk_rpmsg_info mtk_vcp_rpmsg_info = {
	.send_ipi = vcp_send_ipi,
	.register_ipi = vcp_register_ipi,
	.unregister_ipi = vcp_unregister_ipi,
	.ns_ipi_id = VCP_IPI_NS_SERVICE,
};

static void vcp_add_rpmsg_subdev(struct mtk_vcp_device *vcp)
{
	vcp->rpmsg_subdev =
		mtk_rpmsg_create_rproc_subdev(to_platform_device(vcp->dev),
					      &mtk_vcp_rpmsg_info);
	if (!vcp->rpmsg_subdev)
		dev_err(vcp->dev, "create rpmsg subdev failed");
	else
		vcp->rpmsg_subdev->prepare(vcp->rpmsg_subdev);
}

static void vcp_remove_rpmsg_subdev(struct mtk_vcp_device *vcp)
{
	if (vcp->rpmsg_subdev) {
		vcp->rpmsg_subdev->unprepare(vcp->rpmsg_subdev);
		rproc_remove_subdev(vcp->rproc, vcp->rpmsg_subdev);
		mtk_rpmsg_destroy_rproc_subdev(vcp->rpmsg_subdev);
		vcp->rpmsg_subdev = NULL;
	}
}

static int vcp_ipi_mbox_init(struct mtk_vcp_device *vcp)
{
	int ret = 0;

	ret = vcp_ipi_table_init(vcp->vcp_cluster->mbox_base,
				 vcp->vcp_cluster->mbox_init,
				 MBOX_COUNT, &vcp_mboxdev, vcp->pdev);
	if (ret) {
		dev_err(vcp->dev, "vcp_ipi_table_init failed, ret %d\n", ret);
		return ret;
	}

	ret = vcp_ipi_device_register(vcp->ipi_dev, VCP_IPI_COUNT, vcp->pdev, &vcp_mboxdev);
	if (ret) {
		dev_err(vcp->dev, "ipi_dev_register failed, ret %d\n", ret);
		return ret;
	}

	return ret;
}

static int vcp_multi_core_init(struct platform_device *pdev,
			       struct mtk_vcp_of_cluster *vcp_cluster,
			       enum vcp_core_id core_id)
{
	int ret;

	ret = of_property_read_u32(pdev->dev.of_node, "twohart",
				   &vcp_cluster->twohart[core_id]);
	if (ret) {
		dev_err(&pdev->dev, "failed to read twohart property\n");
		return ret;
	}
	ret = of_property_read_u32(pdev->dev.of_node, "sram-offset",
				   &vcp_cluster->sram_offset[core_id]);
	if (ret) {
		dev_err(&pdev->dev, "failed to read sram-offset property\n");
		return ret;
	}

	vcp_cluster->vcp_ready_notify_wk[core_id].flags = core_id;

	return ret;
}

static bool vcp_is_single_core(struct platform_device *pdev,
			       struct mtk_vcp_of_cluster *vcp_cluster)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev_of_node(dev);
	struct device_node *child;
	u32 num_cores = 0;

	for_each_available_child_of_node(np, child) {
		num_cores++;
	}
	vcp_cluster->core_nums = num_cores;

	return num_cores < VCP_CORE_TOTAL ? true : false;
}

static int vcp_add_single_core(struct platform_device *pdev,
			       struct mtk_vcp_of_cluster *vcp_cluster)
{
	return 0;
}

static int vcp_add_multi_core(struct platform_device *pdev,
			      struct mtk_vcp_of_cluster *vcp_cluster)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev_of_node(dev);
	struct device_node *child;
	struct platform_device *cpdev;
	struct mtk_vcp_device *vcp;
	struct rproc *rproc;
	const struct mtk_vcp_of_data *vcp_of_data;
	const char *fw_name = "mediatek/mt8196/vcp.img";
	u32 core_id;
	int ret = 0;

	vcp_of_data = of_device_get_match_data(dev);
	rproc = devm_rproc_alloc(dev, np->name, &mtk_vcp_ops,
				 fw_name, sizeof(struct mtk_vcp_device));
	if (!rproc)
		return dev_err_probe(dev, -ENOMEM, "unable to allocate remoteproc\n");

	vcp  = rproc->priv;
	vcp->rproc = rproc;
	vcp->pdev = pdev;
	vcp->dev = dev;
	vcp->data = vcp_of_data;
	vcp->ipi_dev = &vcp_ipidev;
	vcp->ipi_ops = &mt8196_vcp_ipi_ops;
	vcp->vcp_cluster = vcp_cluster;

	rproc->auto_boot = true;
	rproc->sysfs_read_only = true;
	platform_set_drvdata(pdev, vcp);
	vcp_add_rpmsg_subdev(vcp);

#if IS_ENABLED(CONFIG_OF_RESERVED_MEM)
	ret = vcp_reserve_memory_ioremap(vcp);
	if (ret) {
		dev_err(dev, "vcp_reserve_memory_ioremap failed ret = %d\n", ret);
		goto remove_subdev;
	}
#endif

	for_each_available_child_of_node(np, child) {
		if (of_device_is_compatible(child, "mediatek,vcp-core")) {
			cpdev = of_find_device_by_node(child);
			if (!cpdev) {
				ret = -ENODEV;
				dev_err(dev, "Not found platform device for core\n");
				return ret;
			}
			ret = vcp_multi_core_init(cpdev, vcp_cluster, VCP_ID);
		} else if (of_device_is_compatible(child, "mediatek,mmup-core")) {
			cpdev = of_find_device_by_node(child);
			if (!cpdev) {
				ret = -ENODEV;
				dev_err(dev, "Not found platform device for core\n");
				return ret;
			}
			ret = vcp_multi_core_init(cpdev, vcp_cluster, MMUP_ID);
		}
	}

	ret = vcp_ipi_mbox_init(vcp);
	if (ret) {
		dev_err(dev, "Failed to init vcp ipi-mbox\n");
		goto remove_subdev;
	}

	ret = vcp->ipi_ops->ipi_register(vcp->ipi_dev, IPI_OUT_C_SLEEP_0,
					 NULL, NULL, &vcp->vcp_cluster->slp_ipi_ack_data);
	if (ret) {
		dev_err(dev, "Failed to register IPI_OUT_C_SLEEP_0\n");
		goto slp_ipi_unregister;
	}

	ret = vcp->ipi_ops->ipi_register(vcp->ipi_dev, IPI_IN_VCP_READY_0,
					 (void *)vcp_A_ready_ipi_handler,
					 vcp, &vcp->vcp_cluster->msg_vcp_ready0);
	if (ret) {
		dev_err(dev, "Failed to register IPI_IN_VCP_READY_0\n");
		goto vcp0_ready_ipi_unregister;
	}

	ret = vcp->ipi_ops->ipi_register(vcp->ipi_dev, IPI_IN_VCP_READY_1,
					 (void *)vcp_A_ready_ipi_handler,
					 vcp, &vcp->vcp_cluster->msg_vcp_ready1);
	if (ret) {
		dev_err(dev, "Failed to register IPI_IN_VCP_READY_1\n");
		goto vcp1_ready_ipi_unregister;
	}

	vcp_awake_init();
	vcp_workqueue = create_singlethread_workqueue("VCP_WQ");
	if (!vcp_workqueue)
		dev_info(dev, "vcp_workqueue create fail\n");

	for (core_id = 0; core_id < VCP_CORE_TOTAL; core_id++) {
		vcp->vcp_cluster->vcp_ready_notify_wk[core_id].dev = dev;
		INIT_WORK(&vcp->vcp_cluster->vcp_ready_notify_wk[core_id].work, vcp_A_notify_ws);
	}

	ret = vcp_wdt_irq_init(vcp);
	if (ret)
		dev_err(dev, "vcp_wdt_irq_init failed\n");

	pm_runtime_get_sync(dev);

	ret = rproc_add(rproc);
	if (ret)
		goto rproc_err;
	return ret;

rproc_err:
vcp1_ready_ipi_unregister:
	vcp_mbox_ipi_unregister(vcp->ipi_dev, IPI_IN_VCP_READY_1);
vcp0_ready_ipi_unregister:
	vcp_mbox_ipi_unregister(vcp->ipi_dev, IPI_IN_VCP_READY_0);
slp_ipi_unregister:
	vcp_mbox_ipi_unregister(vcp->ipi_dev, IPI_OUT_C_SLEEP_0);
remove_subdev:
	vcp_remove_rpmsg_subdev(vcp);
	return ret;
}

static int vcp_cluster_init(struct platform_device *pdev,
			    struct mtk_vcp_of_cluster *vcp_cluster)
{
	int ret;

	if (vcp_is_single_core(pdev, vcp_cluster))
		ret = vcp_add_single_core(pdev, vcp_cluster);
	else
		ret = vcp_add_multi_core(pdev, vcp_cluster);

	return ret;
}

static int vcp_device_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct device *dev = &pdev->dev;
	struct mtk_vcp_of_cluster *vcp_cluster;
	int ret;

	pm_runtime_enable(dev);

	vcp_cluster = devm_kzalloc(dev, sizeof(*vcp_cluster), GFP_KERNEL);
	if (!vcp_cluster)
		return -ENOMEM;

	vcp_cluster->cfg = devm_platform_ioremap_resource_byname(pdev, "cfg");
	if (IS_ERR(vcp_cluster->cfg))
		return dev_err_probe(dev, PTR_ERR(vcp_cluster->cfg),
				     "Failed to parse and map cfg memory\n");

	vcp_cluster->mbox_base = devm_platform_ioremap_resource_byname(pdev, "mbox_base");
	if (IS_ERR(vcp_cluster->mbox_base))
		return dev_err_probe(dev, PTR_ERR(vcp_cluster->mbox_base),
				     "Failed to parse and map mbox_base memory\n");

	vcp_cluster->mbox_init = devm_platform_ioremap_resource_byname(pdev, "mbox_init");
	if (IS_ERR(vcp_cluster->mbox_init))
		return dev_err_probe(dev, PTR_ERR(vcp_cluster->mbox_init),
				     "Failed to parse and map mbox_init memory\n");

	vcp_cluster->cfg_core = devm_platform_ioremap_resource_byname(pdev, "cfg_core");
	if (IS_ERR(vcp_cluster->cfg_core))
		return dev_err_probe(dev, PTR_ERR(vcp_cluster->cfg_core),
				     "Failed to parse and map cfg_core memory\n");

	vcp_cluster->vcp_rdy = devm_platform_ioremap_resource_byname(pdev, "vcp_vlp_ao_rsvd7");
	if (IS_ERR(vcp_cluster->vcp_rdy))
		return dev_err_probe(dev, PTR_ERR(vcp_cluster->vcp_rdy),
				     "Failed to parse and map vcp_rdy memory\n");

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sram");
	vcp_cluster->sram_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(vcp_cluster->sram_base))
		return dev_err_probe(dev, PTR_ERR(vcp_cluster->sram_base),
				     "Failed to parse and map sram memory\n");
	vcp_cluster->sram_size = (u32)resource_size(res);

	ret = devm_of_platform_populate(dev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to populate platform devices\n");

	ret = vcp_cluster_init(pdev, vcp_cluster);
	if (ret)
		return ret;

	return 0;
}

static int vcp_device_remove(struct platform_device *pdev)
{
	struct mtk_vcp_device *vcp = platform_get_drvdata(pdev);

	flush_workqueue(vcp_workqueue);
	destroy_workqueue(vcp_workqueue);
	vcp_remove_rpmsg_subdev(vcp);
	pm_runtime_disable(&pdev->dev);

	rproc_del(vcp->rproc);

	return 0;
}

static void vcp_device_shutdown(struct platform_device *pdev)
{
	struct mtk_vcp_device *vcp = platform_get_drvdata(pdev);
	u32 ret;
	int i;

	mutex_lock(&vcp_ready_mutex);
	for (i = 0; i < VCP_CORE_TOTAL ; i++)
		vcp_ready[i] = 0;
	mutex_unlock(&vcp_ready_mutex);

	mutex_lock(&vcp_A_notify_mutex);
	vcp_extern_notify(VCP_ID, VCP_EVENT_STOP);
	vcp_extern_notify(MMUP_ID, VCP_EVENT_STOP);
	mutex_unlock(&vcp_A_notify_mutex);

	writel(GIPC_VCP_HART0_SHUT, vcp->vcp_cluster->cfg_core + R_GIPC_IN_SET);
	if (vcp->vcp_cluster->core_nums > VCP_ID) {
		ret = wait_core_hart_shutdown(vcp, VCP_ID);
		if (!ret) {
			dev_warn(&pdev->dev,
				 "wait VCP_ID core hart shutdown timeout\n");
		} else {
			writel(GIPC_MMUP_SHUT, vcp->vcp_cluster->cfg_core + R_GIPC_IN_SET);
		}
	}
}

static const struct mtk_vcp_of_data mt8196_of_data = {
	.vcp_is_ready = is_vcp_ready,
	.vcp_is_suspending = is_vcp_suspending,
	.vcp_register_notify = vcp_A_register_notify,
	.vcp_unregister_notify = vcp_A_unregister_notify,
	.vcp_register_feature = vcp_A_register_feature,
	.vcp_deregister_feature = vcp_A_deregister_feature,
	.vcp_get_mem_phys = vcp_get_reserve_mem_phys,
	.vcp_get_mem_iova = vcp_get_reserve_mem_iova,
	.vcp_get_mem_virt = vcp_get_reserve_mem_virt,
	.vcp_get_mem_size = vcp_get_reserve_mem_size,
	.vcp_get_sram_virt = vcp_get_internal_sram_virt,
};

static const struct dev_pm_ops mtk_vcp_rproc_pm_ops = {
	.suspend_noirq = mtk_vcp_suspend,
	.resume_noirq = mtk_vcp_resume,
};

static const struct of_device_id vcp_of_ids[] = {
	{ .compatible = "mediatek,vcp", .data = &mt8196_of_data},
	{}
};
MODULE_DEVICE_TABLE(of, vcp_of_ids);

static struct platform_driver mtk_vcp_device = {
	.probe = vcp_device_probe,
	.remove = vcp_device_remove,
	.shutdown = vcp_device_shutdown,
	.driver = {
		.name = "mtk-vcp",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(vcp_of_ids),
		.pm = pm_ptr(&mtk_vcp_rproc_pm_ops),
	},
};

module_platform_driver(mtk_vcp_device);

MODULE_DESCRIPTION("MEDIATEK Module VCP driver");
MODULE_AUTHOR("Mediatek");
MODULE_IMPORT_NS(DMA_BUF);
MODULE_LICENSE("GPL");
