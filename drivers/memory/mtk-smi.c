// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2016 MediaTek Inc.
 * Author: Yong Wu <yong.wu@mediatek.com>
 */
#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>
#include <soc/mediatek/smi.h>
#include <dt-bindings/memory/mt2701-larb-port.h>
#include <dt-bindings/memory/mtk-memory-port.h>

/* SMI COMMON */
#define SMI_L1LEN			0x100
#define SMI_L1ARB0			0x104
#define SMI_L1ARB(larbid)		(SMI_L1ARB0 + ((larbid) << 2))
#define MAX_GNT_CNT_MSK			0xfff
#define BW_FILTER_EN			BIT(12)
#define BW_HFILTER_EN			BIT(13)
#define OSTDL_EN			BIT(14)
#define WR_LIMIT_LSB			24
#define RD_LIMIT_LSB			16
#define OSTDL_MASK			GENMASK(6, 0)

#define SMI_L1_ARB			0x200
#define SMI_BUS_SEL			0x220
#define SMI_BUS_LARB_SHIFT(larbid)	((larbid) << 1)
/* All are MMU0 defaultly. Only specialize mmu1 here. */
#define F_MMU1_LARB(larbid)		(0x1 << SMI_BUS_LARB_SHIFT(larbid))

#define SMI_READ_FIFO_TH		0x230
#define SMI_M4U_TH			0x234
#define SMI_FIFO_TH1			0x238
#define SMI_FIFO_TH2			0x23c
#define SMI_PREULTRA_MASK1		0x244
#define SMI_DCM				0x300
#define SMI_DUMMY			0x444

/* SMI LARB */
#define SMI_LARB_SLP_CON                0xc
#define SLP_PROT_EN                     BIT(0)
#define SLP_PROT_RDY                    BIT(16)

#define SMI_LARB_CMD_THRT_CON		0x24
#define SMI_LARB_THRT_RD_NU_LMT_MSK	GENMASK(7, 4)
#define SMI_LARB_THRT_RD_NU_LMT		(5 << 4)

#define SMI_LARB_SW_FLAG		0x40
#define SMI_LARB_SW_FLAG_1		0x1

#define SMI_LARB_DISABLE_ULTRA		0x70

#define SMI_LARB_OSTDL_PORT		0x200
#define SMI_LARB_OSTDL_PORTx(id)	(SMI_LARB_OSTDL_PORT + (((id) & 0x1f) << 2))

/* Below are about mmu enable registers, they are different in SoCs */
/* gen1: mt2701 */
#define REG_SMI_SECUR_CON_BASE		0x5c0

/* every register control 8 port, register offset 0x4 */
#define REG_SMI_SECUR_CON_OFFSET(id)	(((id) >> 3) << 2)
#define REG_SMI_SECUR_CON_ADDR(id)	\
	(REG_SMI_SECUR_CON_BASE + REG_SMI_SECUR_CON_OFFSET(id))

/*
 * every port have 4 bit to control, bit[port + 3] control virtual or physical,
 * bit[port + 2 : port + 1] control the domain, bit[port] control the security
 * or non-security.
 */
#define SMI_SECUR_CON_VAL_MSK(id)	(~(0xf << (((id) & 0x7) << 2)))
#define SMI_SECUR_CON_VAL_VIRT(id)	BIT((((id) & 0x7) << 2) + 3)
/* mt2701 domain should be set to 3 */
#define SMI_SECUR_CON_VAL_DOMAIN(id)	(0x3 << ((((id) & 0x7) << 2) + 1))

/* gen2: */
/* mt8167 */
#define MT8167_SMI_LARB_MMU_EN		0xfc0

/* mt8173 */
#define MT8173_SMI_LARB_MMU_EN		0xf00

/* general */
#define SMI_LARB_NONSEC_CON(id)		(0x380 + ((id) * 4))
#define F_MMU_EN			BIT(0)
#define BW_TH_EN			BIT(3)
#define BANK_SEL(id)			({		\
	u32 _id = (id) & 0x3;				\
	(_id << 8 | _id << 10 | _id << 12 | _id << 14);	\
})

#define SMI_COMMON_INIT_REGS_NR		12
#define SMI_LARB_PORT_NR_MAX		32
#define SMI_COMMON_LARB_NR_MAX		8
#define MTK_COMMON_NR_MAX		33

#define MTK_SMI_FLAG_THRT_UPDATE	BIT(0)
#define MTK_SMI_FLAG_SW_FLAG		BIT(1)
#define MTK_SMI_FLAG_SLEEP_CTL		BIT(2)
#define MTK_SMI_FLAG_CFG_PORT_SEC_CTL	BIT(3)
#define MTK_SMI_FLAG_CONNECT_SMMUV3	BIT(4)
#define MTK_SMI_CAPS(flags, _x)		(!!((flags) & (_x)))

struct mtk_smi_reg_pair {
	unsigned int		offset;
	u32			value;
};

enum mtk_smi_type {
	MTK_SMI_GEN1,
	MTK_SMI_GEN2,		/* gen2 smi common */
	MTK_SMI_GEN2_SUB_COMM,	/* gen2 smi sub common */
};

enum mtk_smi_real_time_type {
	ALWAYS_HRT = 0,
	ALWAYS_SRT,
	HRT_SRT_SWITCH,
	NON_APMCU,
};

/* larbs: Require apb/smi clocks while gals is optional. */
static const char * const mtk_smi_larb_clks[] = {"apb", "smi", "gals"};
#define MTK_SMI_LARB_REQ_CLK_NR		2
#define MTK_SMI_LARB_OPT_CLK_NR		1

/*
 * common: Require these four clocks in has_gals case. Otherwise, only apb/smi are required.
 * sub common: Require apb/smi/gals0 clocks in has_gals case. Otherwise, only apb/smi are required.
 */
static const char * const mtk_smi_common_clks[] = {"apb", "smi", "gals0", "gals1"};
#define MTK_SMI_CLK_NR_MAX		ARRAY_SIZE(mtk_smi_common_clks)
#define MTK_SMI_COM_REQ_CLK_NR		2
#define MTK_SMI_COM_GALS_REQ_CLK_NR	MTK_SMI_CLK_NR_MAX
#define MTK_SMI_SUB_COM_GALS_REQ_CLK_NR 3

struct mtk_smi_common_plat {
	enum mtk_smi_type	type;
	bool			has_gals;
	u32			bus_sel; /* Balance some larbs to enter mmu0 or mmu1 */
	bool			has_init;
	bool			has_p2_ostdl;
	const struct mtk_smi_reg_pair	(*init)[SMI_COMMON_INIT_REGS_NR];
};

struct mtk_smi_larb_gen {
	int port_in_larb[MTK_LARB_NR_MAX + 1];
	int				(*config_port)(struct device *dev);
	unsigned int			larb_direct_to_common_mask;
	unsigned int			flags_general;
	u8				(*ostd)[SMI_LARB_PORT_NR_MAX];
	u8				port_in_larb_gen2[MTK_LARB_NR_MAX + 1];
};

struct mtk_smi {
	struct device			*dev;
	unsigned int			clk_num;
	struct clk_bulk_data		clks[MTK_SMI_CLK_NR_MAX];
	struct clk			*clk_async; /*only needed by mt2701*/
	union {
		void __iomem		*smi_ao_base; /* only for gen1 */
		void __iomem		*base;	      /* only for gen2 */
	};
	struct device			*smi_common_dev; /* for sub common */
	const struct mtk_smi_common_plat *plat;
	u32				commid;
	u32				common_bwl[SMI_COMMON_LARB_NR_MAX]; /* common's bandwidth limit for each larb */
	bool				skip_rpm;
};

struct mtk_smi_larb { /* larb: local arbiter */
	struct mtk_smi			smi;
	void __iomem			*base;
	struct device			*smi_common_dev; /* common or sub-common dev */
	const struct mtk_smi_larb_gen	*larb_gen;
	int				larbid;
	u32				*mmu;
	unsigned char			*bank;

	bool				is_real_time_perm;
	/* Porting from dts, type align smi_real_time_type */
	u32				larb_port_real_time_type[SMI_LARB_PORT_NR_MAX];
	/* larb port real time type is bit array, 1 means SRT, align disable_ultra value */
	u32				real_time_type;
};

/**
 * mtk_smi_larb_bw_set - Set the bandwidth value for a specific port
 * @larbdev: Pointer to the LARB device
 * @port: Port number
 * @val: Bandwidth value
 *
 * This function sets the bandwidth value for a specific port.
 *
 * Return: 0 on success, negative error code on failure.
 */
int mtk_smi_larb_bw_set(struct device *larbdev, const u8 port, const u8 val)
{
	struct mtk_smi_larb *larb = dev_get_drvdata(larbdev);
	u8 *larbostd;

	if (port >= SMI_LARB_PORT_NR_MAX || !val || !larb->larb_gen->ostd)
		return -EINVAL;
	if (!pm_runtime_active(larbdev))
		return -ENONET;

	larbostd = larb->larb_gen->ostd[larb->larbid];
	larbostd[port] = val;
	writel_relaxed(val, larb->base + SMI_LARB_OSTDL_PORTx(port));
	return 0;
}
EXPORT_SYMBOL_NS_GPL(mtk_smi_larb_bw_set, MTK_SMI);

/**
 * mtk_smi_common_bw_set - Set the common bandwidth value for a specific LARB
 * @larbdev: Pointer to the LARB device
 * @larbid: LARB ID
 * @hard_limit: Whether to enable hard limit mode
 * @grant_val: Grant value
 *
 * This function sets the common bandwidth value for a specific LARB.It updates
 * the bandwidth value according to the grant value.If the hard limit mode is enabled,
 * the bandwidth limit must be strictly followed. On the contrary, if other Larbs
 * are not being used, it is allowed to exceed this limit.
 *
 * Return: 0 on success, negative error code on failure.
 */
int mtk_smi_common_bw_set(struct device *larbdev, const u8 larbid, bool hard_limit, const u32 grant_val)
{
	struct mtk_smi_larb *larb; // = dev_get_drvdata(larbdev);
	struct mtk_smi *common; // = dev_get_drvdata(larb->smi_common_dev);
	u32 orig_val, write_val;
	u32 *common_bwl; // = common->common_bwl;

	larb = dev_get_drvdata(larbdev);
	if (!larb) {
		pr_err("%s: For dev %s -- struct mtk_smi_larb *larb is NULL! Returning failure.\n", __func__, dev_name(larbdev));
		WARN(1, "X");
		return -EINVAL;
	}

	common = dev_get_drvdata(larb->smi_common_dev);
	if (!common) {
		pr_err("%s: struct mtk_smi *common is NULL! Returning failure.\n", __func__);
		return -EINVAL;
	}

	common_bwl = common->common_bwl;

	if (larbid >= SMI_COMMON_LARB_NR_MAX || !common_bwl)
		return -EINVAL;
	if (!pm_runtime_active(larbdev))
		return -ENONET;

	orig_val = common_bwl[larbid];
	write_val = orig_val;
	if (grant_val) {
		write_val &= ~(MAX_GNT_CNT_MSK);
		write_val |= grant_val & MAX_GNT_CNT_MSK;
		write_val |= BW_FILTER_EN;
		if (hard_limit)
			write_val |= BW_HFILTER_EN;
	} else {
		write_val &= ~BW_FILTER_EN;
		write_val &= ~BW_HFILTER_EN;
	}

	common_bwl[larbid] = write_val;
	writel_relaxed(write_val, common->base + SMI_L1ARB(larbid));
	return 0;
}
EXPORT_SYMBOL_NS_GPL(mtk_smi_common_bw_set, MTK_SMI);

static bool is_other_ostd_existed(const u32 value, bool is_write)
{
	return ((value >> (is_write ? WR_LIMIT_LSB : RD_LIMIT_LSB)) & OSTDL_MASK) > 0;
}
/**
 * mtk_smi_common_ostdl_set - Set the write or read OSTDL (Outstanding Transaction Depth Limit) for a specific LARB
 * @larbdev: The device structure for the LARB
 * @larbid: The ID of the LARB
 * @is_write: Boolean indicating if the limit is for write transactions
 * @limit_val: The limit value to be set
 *
 * This function sets the write or read OSTDL for a specific LARB. It takes into account whether the limit is for
 * write or read transactions and updates the corresponding register value. If the limit value is zero, it disables
 * the OSTDL.
 *
 * Return: 0 on success, -EINVAL if the larbid is invalid or the platform does not support OSTDL,
 *         -ENONET if the runtime PM is not active.
 */
int mtk_smi_common_ostdl_set(struct device *larbdev, const u8 larbid, bool is_write, const u32 limit_val)
{
	struct mtk_smi_larb *larb = dev_get_drvdata(larbdev);
	struct mtk_smi *common = dev_get_drvdata(larb->smi_common_dev);
	u32 orig_val, write_val;
	u32 *common_bwl = common->common_bwl;

	if (larbid >= SMI_COMMON_LARB_NR_MAX || !common->plat->has_p2_ostdl)
		return -EINVAL;

	if (!pm_runtime_active(larbdev))
		return -ENONET;

	orig_val = common_bwl[larbid];
	write_val = orig_val;
	if (limit_val) {
		if (is_other_ostd_existed(orig_val, !is_write))
			write_val |= OSTDL_EN;
		write_val |= (limit_val & OSTDL_MASK) << (is_write ? WR_LIMIT_LSB : RD_LIMIT_LSB);
	} else {
		write_val &= ~OSTDL_EN;
		write_val &= ~((OSTDL_MASK) << (is_write ? WR_LIMIT_LSB : RD_LIMIT_LSB));
	}

	common_bwl[larbid] = write_val;
	writel(write_val, common->base + SMI_L1ARB(larbid));
	return 0;
}
EXPORT_SYMBOL_NS_GPL(mtk_smi_common_ostdl_set, MTK_SMI);

/* make sure the larb's power is active. */
static int mtk_smi_larb_port_dis_ultra(struct device *larbdev, const u8 port, bool is_dis_ultra)
{
	struct mtk_smi_larb *larb = dev_get_drvdata(larbdev);

	if (port >= SMI_LARB_PORT_NR_MAX) /* max: 32 ports for a larb */
		return -EINVAL;

	if (is_dis_ultra)
		larb->real_time_type |= BIT(port);
	else
		larb->real_time_type &= ~BIT(port);

	writel_relaxed(larb->real_time_type, larb->base + SMI_LARB_DISABLE_ULTRA);
	return 0;
}

/* Make sure the larb's power is active. */
static void mtk_smi_larb_bw_thr(struct device *larbdev, const u8 port, bool is_bw_thr)
{
	struct mtk_smi_larb *larb = dev_get_drvdata(larbdev);
	u32 reg;

	reg = readl_relaxed(larb->base + SMI_LARB_NONSEC_CON(port));
	if (is_bw_thr)
		reg |= BW_TH_EN;
	else
		reg &= ~(BW_TH_EN);
	writel_relaxed(reg, larb->base + SMI_LARB_NONSEC_CON(port));
}

static inline bool is_larb_port_can_be_hrt(struct device *dev, const u8 port)
{
	struct mtk_smi_larb *larb = dev_get_drvdata(dev);

	switch (larb->larb_port_real_time_type[port]) {
	case ALWAYS_HRT:
	case HRT_SRT_SWITCH:
		return true;
	case ALWAYS_SRT:
	case NON_APMCU:
		return false;
	default:
		break;
	}
	return false;
}

/**
 * mtk_smi_set_hrt_perm - Set the Hardware Real Time (HRT) permission for a specific LARB port
 * @larbdev: The device structure for the LARB
 * @port: The port number within the LARB
 * @is_hrt: Boolean indicating if HRT permission should be enabled
 *
 * This function sets the HRT permission for a specific port in the LARB. If HRT permission is enabled,
 * it disables dis_ultra signal and bandwidth throttling for the port.
 * If HRT permission is disabled, it enables dis_ultra signal and bandwidth throttling for the port.
 *
 * Return: 0 on success, -EINVAL if the port number is invalid or the port cannot be allowed to have HRT permission.
 */
int mtk_smi_set_hrt_perm(struct device *larbdev, const u8 port, bool is_hrt)
{
	if (port >= SMI_LARB_PORT_NR_MAX)
		return -EINVAL;

	if (is_hrt) {
		if (!is_larb_port_can_be_hrt(larbdev, port)) {
			dev_err(larbdev, "this port %d is not allowed to HRT.\n", port);
			return -EINVAL;
		}

		/* disable dis_ultra */
		mtk_smi_larb_port_dis_ultra(larbdev, port, false);
		/* disable bw thr */
		mtk_smi_larb_bw_thr(larbdev, port, false);
	} else {
		/* enable dis_ultra */
		mtk_smi_larb_port_dis_ultra(larbdev, port, true);
		/* enable bw thr */
		mtk_smi_larb_bw_thr(larbdev, port, true);
	}
	return 0;
}
EXPORT_SYMBOL_NS_GPL(mtk_smi_set_hrt_perm, MTK_SMI);

static int
mtk_smi_larb_bind(struct device *dev, struct device *master, void *data)
{
	struct mtk_smi_larb *larb = dev_get_drvdata(dev);
	struct mtk_smi_larb_iommu *larb_mmu = data;
	unsigned int         i;

	for (i = 0; i < MTK_LARB_NR_MAX; i++) {
		if (dev == larb_mmu[i].dev) {
			larb->larbid = i;
			larb->mmu = &larb_mmu[i].mmu;
			larb->bank = larb_mmu[i].bank;
			return 0;
		}
	}
	return -ENODEV;
}

static void
mtk_smi_larb_unbind(struct device *dev, struct device *master, void *data)
{
	/* Do nothing as the iommu is always enabled. */
}

static const struct component_ops mtk_smi_larb_component_ops = {
	.bind = mtk_smi_larb_bind,
	.unbind = mtk_smi_larb_unbind,
};

static int mtk_smi_larb_config_port_gen1(struct device *dev)
{
	struct mtk_smi_larb *larb = dev_get_drvdata(dev);
	const struct mtk_smi_larb_gen *larb_gen = larb->larb_gen;
	struct mtk_smi *common = dev_get_drvdata(larb->smi_common_dev);
	int i, m4u_port_id, larb_port_num;
	u32 sec_con_val, reg_val;

	m4u_port_id = larb_gen->port_in_larb[larb->larbid];
	larb_port_num = larb_gen->port_in_larb[larb->larbid + 1]
			- larb_gen->port_in_larb[larb->larbid];

	for (i = 0; i < larb_port_num; i++, m4u_port_id++) {
		if (*larb->mmu & BIT(i)) {
			/* bit[port + 3] controls the virtual or physical */
			sec_con_val = SMI_SECUR_CON_VAL_VIRT(m4u_port_id);
		} else {
			/* do not need to enable m4u for this port */
			continue;
		}
		reg_val = readl(common->smi_ao_base
			+ REG_SMI_SECUR_CON_ADDR(m4u_port_id));
		reg_val &= SMI_SECUR_CON_VAL_MSK(m4u_port_id);
		reg_val |= sec_con_val;
		reg_val |= SMI_SECUR_CON_VAL_DOMAIN(m4u_port_id);
		writel(reg_val,
			common->smi_ao_base
			+ REG_SMI_SECUR_CON_ADDR(m4u_port_id));
	}
	return 0;
}

static int mtk_smi_larb_config_port_mt8167(struct device *dev)
{
	struct mtk_smi_larb *larb = dev_get_drvdata(dev);

	writel(*larb->mmu, larb->base + MT8167_SMI_LARB_MMU_EN);
	return 0;
}

static int mtk_smi_larb_config_port_mt8173(struct device *dev)
{
	struct mtk_smi_larb *larb = dev_get_drvdata(dev);

	writel(*larb->mmu, larb->base + MT8173_SMI_LARB_MMU_EN);
	return 0;
}

static int mtk_smi_larb_config_port_gen2_general(struct device *dev)
{
	struct mtk_smi_larb *larb = dev_get_drvdata(dev);
	u32 reg, flags_general = larb->larb_gen->flags_general;
	const u8 *larbostd = larb->larb_gen->ostd ? larb->larb_gen->ostd[larb->larbid] : NULL;
	struct arm_smccc_res res;
	int i;

	if (BIT(larb->larbid) & larb->larb_gen->larb_direct_to_common_mask)
		return 0;

	if (MTK_SMI_CAPS(flags_general, MTK_SMI_FLAG_THRT_UPDATE)) {
		reg = readl_relaxed(larb->base + SMI_LARB_CMD_THRT_CON);
		reg &= ~SMI_LARB_THRT_RD_NU_LMT_MSK;
		reg |= SMI_LARB_THRT_RD_NU_LMT;
		writel_relaxed(reg, larb->base + SMI_LARB_CMD_THRT_CON);
	}

	if (MTK_SMI_CAPS(flags_general, MTK_SMI_FLAG_SW_FLAG))
		writel_relaxed(SMI_LARB_SW_FLAG_1, larb->base + SMI_LARB_SW_FLAG);

	for (i = 0; i < SMI_LARB_PORT_NR_MAX && larbostd && !!larbostd[i]; i++)
		writel_relaxed(larbostd[i], larb->base + SMI_LARB_OSTDL_PORTx(i));

	if (larb->is_real_time_perm) {
		/* disable ultra */
		writel_relaxed(larb->real_time_type, larb->base + SMI_LARB_DISABLE_ULTRA);
		/* enable larb bw thr */
		for (i = 0; i < larb->larb_gen->port_in_larb_gen2[larb->larbid]; i++) {
			if ((larb->real_time_type >> i) & 1)
				mtk_smi_larb_bw_thr(larb->smi.dev, i, true);
		}
	}
	/*
	 * When mmu_en bits are in security world, the bank_sel still is in the
	 * LARB_NONSEC_CON below. And the mmu_en bits of LARB_NONSEC_CON have no
	 * effect in this case.
	 */
	if (MTK_SMI_CAPS(flags_general, MTK_SMI_FLAG_CFG_PORT_SEC_CTL)) {
		arm_smccc_smc(MTK_SIP_KERNEL_IOMMU_CONTROL, IOMMU_ATF_CMD_CONFIG_SMI_LARB,
			      larb->larbid, *larb->mmu, 0, 0, 0, 0, &res);
		if (res.a0 != 0) {
			dev_err(dev, "Enable iommu fail, ret %ld\n", res.a0);
			return -EINVAL;
		}
	}

	if (!MTK_SMI_CAPS(larb->larb_gen->flags_general, MTK_SMI_FLAG_CONNECT_SMMUV3)) {
		for_each_set_bit(i, (unsigned long *)larb->mmu, 32) {
			reg = readl_relaxed(larb->base + SMI_LARB_NONSEC_CON(i));
			reg |= F_MMU_EN;
			reg |= BANK_SEL(larb->bank[i]);
			writel(reg, larb->base + SMI_LARB_NONSEC_CON(i));
		}
	}
	return 0;
}

static u8 mtk_smi_larb_mt8188_ostd[][SMI_LARB_PORT_NR_MAX] = {
	[0] = {0x02, 0x18, 0x22, 0x22, 0x01, 0x02, 0x0a,},
	[1] = {0x12, 0x02, 0x14, 0x14, 0x01, 0x18, 0x0a,},
	[2] = {0x12, 0x12, 0x12, 0x12, 0x0a,},
	[3] = {0x12, 0x12, 0x12, 0x12, 0x28, 0x28, 0x0a,},
	[4] = {0x06, 0x01, 0x17, 0x06, 0x0a, 0x07, 0x07,},
	[5] = {0x02, 0x01, 0x04, 0x02, 0x06, 0x01, 0x06, 0x0a,},
	[6] = {0x06, 0x01, 0x06, 0x0a,},
	[7] = {0x0c, 0x0c, 0x12,},
	[8] = {0x0c, 0x01, 0x0a, 0x05, 0x02, 0x03, 0x01, 0x01, 0x14, 0x14,
	       0x0a, 0x14, 0x1e, 0x01, 0x0c, 0x0a, 0x05, 0x02, 0x02, 0x05,
	       0x03, 0x01, 0x1e, 0x01, 0x05,},
	[9] = {0x1e, 0x01, 0x0a, 0x0a, 0x01, 0x01, 0x03, 0x1e, 0x1e, 0x10,
	       0x07, 0x01, 0x0a, 0x06, 0x03, 0x03, 0x0e, 0x01, 0x04, 0x28,},
	[10] = {0x03, 0x20, 0x01, 0x20, 0x01, 0x01, 0x14, 0x0a, 0x0a, 0x0c,
		0x0a, 0x05, 0x02, 0x03, 0x02, 0x14, 0x0a, 0x0a, 0x14, 0x14,
		0x14, 0x01, 0x01, 0x14, 0x1e, 0x01, 0x05, 0x03, 0x02, 0x28,},
	[11] = {0x03, 0x20, 0x01, 0x20, 0x01, 0x01, 0x14, 0x0a, 0x0a, 0x0c,
		0x0a, 0x05, 0x02, 0x03, 0x02, 0x14, 0x0a, 0x0a, 0x14, 0x14,
		0x14, 0x01, 0x01, 0x14, 0x1e, 0x01, 0x05, 0x03, 0x02, 0x28,},
	[12] = {0x03, 0x20, 0x01, 0x20, 0x01, 0x01, 0x14, 0x0a, 0x0a, 0x0c,
		0x0a, 0x05, 0x02, 0x03, 0x02, 0x14, 0x0a, 0x0a, 0x14, 0x14,
		0x14, 0x01, 0x01, 0x14, 0x1e, 0x01, 0x05, 0x03, 0x02, 0x28,},
	[13] = {0x07, 0x02, 0x04, 0x02, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05,
		0x07, 0x02, 0x04, 0x02, 0x05, 0x05,},
	[14] = {0x02, 0x02, 0x0c, 0x0c, 0x0c, 0x0c, 0x01, 0x01, 0x02, 0x02,
		0x02, 0x02, 0x0c, 0x0c, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
		0x02, 0x02, 0x01, 0x01,},
	[15] = {0x0c, 0x0c, 0x02, 0x02, 0x02, 0x02, 0x01, 0x01, 0x0c, 0x0c,
		0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x01, 0x02,
		0x0c, 0x01, 0x01,},
	[16] = {0x28, 0x28, 0x03, 0x01, 0x01, 0x03, 0x14, 0x14, 0x0a, 0x0d,
		0x03, 0x05, 0x0e, 0x01, 0x01, 0x05, 0x06, 0x0d, 0x01,},
	[17] = {0x28, 0x02, 0x02, 0x12, 0x02, 0x12, 0x10, 0x02, 0x02, 0x0a,
		0x12, 0x02, 0x02, 0x0a, 0x16, 0x02, 0x04,},
	[18] = {0x28, 0x02, 0x02, 0x12, 0x02, 0x12, 0x10, 0x02, 0x02, 0x0a,
		0x12, 0x02, 0x02, 0x0a, 0x16, 0x02, 0x04,},
	[19] = {0x1a, 0x0e, 0x0a, 0x0a, 0x0c, 0x0e, 0x10,},
	[20] = {0x1a, 0x0e, 0x0a, 0x0a, 0x0c, 0x0e, 0x10,},
	[21] = {0x01, 0x04, 0x01, 0x01, 0x01, 0x01, 0x01, 0x04, 0x04, 0x01,
		0x01, 0x01, 0x04, 0x0a, 0x06, 0x01, 0x01, 0x01, 0x0a, 0x06,
		0x01, 0x01, 0x05, 0x03, 0x03, 0x04, 0x01,},
	[22] = {0x28, 0x19, 0x0c, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x04,
		0x01,},
	[23] = {0x01, 0x01, 0x04, 0x01, 0x01, 0x01, 0x18, 0x01, 0x01,},
	[24] = {0x12, 0x06, 0x12, 0x06,},
	[25] = {0x01},
};

static u8 mtk_smi_larb_mt8189_ostd[][SMI_LARB_PORT_NR_MAX] = {
	[0] = {0x8, 0x20, 0x20, 0x20, 0x20, 0x20, 0x10, 0x0,},
	[1] = {0x8, 0x20, 0x20, 0x20, 0x20, 0x20, 0x10, 0x0,},
	[2] = {0x7, 0x7, 0x4, 0x4, 0x0, 0x0, 0x2, 0x2, 0x7, 0x7, 0x0,},
	[4] = {0x2F, 0x1E, 0x9, 0x1, 0x1, 0x1, 0x1, 0x2, 0x2, 0x5, 0x1, 0x17,},
	[7] = {0x20, 0x2, 0x1, 0x1, 0x1, 0x4, 0x2, 0x1, 0x1, 0x2, 0x3, 0x2,
	       0xA, 0xF, 0x4, 0x6, 0x5, 0x1,},
	[9] = {0x6, 0x3, 0xC, 0x6, 0x1, 0x4, 0x3, 0x1, 0x2, 0x4, 0x5, 0x2,
	       0x4, 0x2, 0x3, 0xB, 0x1, 0x4, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	       0x1, 0x1,},
	[11] = {0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
		0x1, 0x1, 0x1, 0xB, 0x1, 0x4, 0x6, 0x5, 0x6, 0x1, 0x5, 0x2,
		0x9, 0x5,},
	[13] = {0x2, 0x8, 0x8, 0x8, 0x4, 0x4, 0x4, 0x4, 0x4, 0xE, 0x4, 0x1,
		0x6, 0x6, 0x2,},
	[14] = {0x1, 0x1, 0x1, 0x20, 0xE, 0x4, 0x8, 0x8, 0x6, 0x4,},
	[16] = {0x1E, 0xC, 0x2, 0x8, 0xE, 0x2, 0x1E, 0x10, 0x4, 0x2, 0x2, 0x2,
		0x2, 0x2, 0x4, 0x2, 0x4,},
	[17] = {0x1E, 0xC, 0x2, 0x8, 0xE, 0x2, 0x1E, 0x10, 0x4, 0x2, 0x2, 0x2,
		0x2, 0x2, 0x4, 0x2, 0x4,},
	[19] = {0x2, 0x1, 0x3, 0x1,},
	[20] = {0x7, 0x7, 0x3, 0x3, 0x1, 0x1,},
};

static u8 mtk_smi_larb_mt8195_ostd[][SMI_LARB_PORT_NR_MAX] = {
	[0] = {0x0a, 0xc, 0x22, 0x22, 0x01, 0x0a,}, /* larb0 */
	[1] = {0x0a, 0xc, 0x22, 0x22, 0x01, 0x0a,}, /* larb1 */
	[2] = {0x12, 0x12, 0x12, 0x12, 0x0a,},      /* ... */
	[3] = {0x12, 0x12, 0x12, 0x12, 0x28, 0x28, 0x0a,},
	[4] = {0x06, 0x01, 0x17, 0x06, 0x0a,},
	[5] = {0x06, 0x01, 0x17, 0x06, 0x06, 0x01, 0x06, 0x0a,},
	[6] = {0x06, 0x01, 0x06, 0x0a,},
	[7] = {0x0c, 0x0c, 0x12,},
	[8] = {0x0c, 0x0c, 0x12,},
	[9] = {0x0a, 0x08, 0x04, 0x06, 0x01, 0x01, 0x10, 0x18, 0x11, 0x0a,
		0x08, 0x04, 0x11, 0x06, 0x02, 0x06, 0x01, 0x11, 0x11, 0x06,},
	[10] = {0x18, 0x08, 0x01, 0x01, 0x20, 0x12, 0x18, 0x06, 0x05, 0x10,
		0x08, 0x08, 0x10, 0x08, 0x08, 0x18, 0x0c, 0x09, 0x0b, 0x0d,
		0x0d, 0x06, 0x10, 0x10,},
	[11] = {0x0e, 0x0e, 0x0e, 0x0e, 0x0e, 0x0e, 0x01, 0x01, 0x01, 0x01,},
	[12] = {0x09, 0x09, 0x05, 0x05, 0x0c, 0x18, 0x02, 0x02, 0x04, 0x02,},
	[13] = {0x02, 0x02, 0x12, 0x12, 0x02, 0x02, 0x02, 0x02, 0x08, 0x01,},
	[14] = {0x12, 0x12, 0x02, 0x02, 0x02, 0x02, 0x16, 0x01, 0x16, 0x01,
		0x01, 0x02, 0x02, 0x08, 0x02,},
	[15] = {},
	[16] = {0x28, 0x02, 0x02, 0x12, 0x02, 0x12, 0x10, 0x02, 0x02, 0x0a,
		0x12, 0x02, 0x0a, 0x16, 0x02, 0x04,},
	[17] = {0x1a, 0x0e, 0x0a, 0x0a, 0x0c, 0x0e, 0x10,},
	[18] = {0x12, 0x06, 0x12, 0x06,},
	[19] = {0x01, 0x04, 0x01, 0x01, 0x01, 0x01, 0x01, 0x04, 0x04, 0x01,
		0x01, 0x01, 0x04, 0x0a, 0x06, 0x01, 0x01, 0x01, 0x0a, 0x06,
		0x01, 0x01, 0x05, 0x03, 0x03, 0x04, 0x01,},
	[20] = {0x01, 0x04, 0x01, 0x01, 0x01, 0x01, 0x01, 0x04, 0x04, 0x01,
		0x01, 0x01, 0x04, 0x0a, 0x06, 0x01, 0x01, 0x01, 0x0a, 0x06,
		0x01, 0x01, 0x05, 0x03, 0x03, 0x04, 0x01,},
	[21] = {0x28, 0x19, 0x0c, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x04,},
	[22] = {0x28, 0x19, 0x0c, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x04,},
	[23] = {0x18, 0x01,},
	[24] = {0x01, 0x01, 0x04, 0x01, 0x01, 0x01, 0x01, 0x01, 0x04, 0x01,
		0x01, 0x01,},
	[25] = {0x02, 0x02, 0x02, 0x28, 0x16, 0x02, 0x02, 0x02, 0x12, 0x16,
		0x02, 0x01,},
	[26] = {0x02, 0x02, 0x02, 0x28, 0x16, 0x02, 0x02, 0x02, 0x12, 0x16,
		0x02, 0x01,},
	[27] = {0x02, 0x02, 0x02, 0x28, 0x16, 0x02, 0x02, 0x02, 0x12, 0x16,
		0x02, 0x01,},
	[28] = {0x1a, 0x0e, 0x0a, 0x0a, 0x0c, 0x0e, 0x10,},
};

static u8 mtk_smi_larb_mt8196_ostd[][SMI_LARB_PORT_NR_MAX] = {
	[0] = {0x4, 0x4, 0x40, 0x40, 0x1, 0x1, 0x2, 0x2, 0x4, 0x4,
	 0x1, 0x1, 0x1,}, /* LARB0 */
	[1] = {0x4, 0x4, 0x40, 0x40, 0x32, 0x1, 0x2, 0x2, 0x2, 0x4,
	 0x4, 0x2, 0x1, 0x1, 0x1, 0x1,}, /* LARB1 */
	[2] = {0x1, 0x1, 0x1, 0x1, 0x9, 0xb, 0x2a, 0x1, 0x1, 0x1,
	 0x1, 0x1, 0x1, 0x1, 0x3, 0x1c, 0x1, 0x1,}, /* LARB2 */
	[3] = {0x2, 0x2, 0x2, 0x2, 0x1a, 0x20, 0x2a, 0x2, 0x1, 0x1,
	 0x1, 0x1, 0x1, 0x2, 0x8, 0x1c, 0x1, 0x1,}, /* LARB3 */
	[4] = {0x40, 0x10, 0x10, 0x1, 0x4, 0x10, 0x8, 0x8,}, /* LARB4 */
	[5] = {0x10, 0x8, 0x40, 0x1e, 0x8, 0x8, 0x4, 0x1,}, /* LARB5 */
	[6] ={0x40, 0x12, 0x1,}, /* LARB6 */
	[7] ={0x20, 0x6, 0x6, 0x1, 0x1, 0x24, 0x2b, 0x7, 0x4, 0x1,
	 0x1, 0xf, 0x3, 0x5, 0x8, 0x8, 0x3, 0x8, 0x5, 0x23,
	 0x24, 0x4, 0x2, 0xb, 0x10, 0x17, 0x4, 0x8, 0x5, 0x1,
	 0x1, 0x6,}, /* LARB7 */
	[8] ={0x20, 0x6, 0x6, 0x1, 0x1, 0x24, 0x2b, 0x7, 0x4, 0x1,
	 0x1, 0xf, 0x3, 0x5, 0x8, 0x8, 0x3, 0x8, 0x5, 0x23,
	 0x24, 0x4, 0x2, 0xb, 0x10, 0x17, 0x4, 0x8, 0x5, 0x1,
	 0x1, 0x6,}, /* LARB8 */
	[9] = {0x2b, 0x8, 0x9, 0x31, 0x10, 0x26, 0x15, 0x13, 0x7, 0x4,
	 0x1, 0x1, 0x7, 0xa, 0xb, 0x6, 0x1, 0x1, 0x1, 0x1,
	 0x1, 0x1, 0xf, 0x9, 0x6, 0x3,}, /*LARB9*/
	[10] = {0x2b, 0x8, 0x20, 0x1d, 0x19, 0xf, 0x1, 0x3,}, /* LARB10 */
	[11] = {0x8, 0x16, 0x16, 0x24, 0x1, 0x1, 0x1, 0x3, 0x32, 0x1,
	 0x8, 0x10, 0x16, 0x2, 0x38,}, /* LARB11 */
	[12] = {0xa, 0xa, 0x1,}, /* LARB12 */
	[13] = {0x2, 0x20, 0x14, 0x1, 0x1, 0x2, 0x2,}, /* LARB13 */
	[14] = {0x2, 0x20, 0x14, 0x1, 0x2, 0x2,}, /* LARB14 */
	[15] = {0x2b, 0x7, 0x31, 0xa, 0x10, 0x10, 0x2b, 0x29, 0x7, 0x1,}, /* LARB15 */
	[16] = {0x4, 0x4, 0x12, 0x8, 0x8, 0x16, 0x8, 0x6, 0xe, 0x6,
	 0x1e, 0x18, 0x16, 0xe, 0x8, 0xe, 0x8, 0x2, 0x2,}, /* LARB16 */
	[17] = {0x18, 0x18, 0x8, 0x8, 0xc, 0x4, 0x2,}, /* LARB17 */
	[18] = {0xb, 0x1, 0x10, 0x1, 0x2,}, /* LARB18 */
	[19] = {0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x4, 0x2, 0x1, 0x1,
	 0x4, 0x2, 0x1,}, /* LARB19 */
	[20] = {0x2, 0x2, 0x2, 0x40, 0x40, 0x40, 0x1, 0x2, 0x2, 0x2,
	 0x4, 0x4, 0x4, 0x1, 0x1,}, /* LARB20 */
	[21] = {0x2, 0x2, 0x2, 0x40, 0x40, 0x40, 0x1, 0x32, 0x32, 0x32,
	 0x2, 0x2, 0x2, 0x4, 0x4, 0x4, 0x2, 0x1,}, /* LARB21 */
	[22] = {0x8, 0x16, 0x16, 0x24, 0x1, 0x1, 0x1, 0x3, 0x32, 0x1,
	 0x8, 0x10, 0x16, 0x2, 0x38,}, /* LARB22 */
	[23] = {0x8, 0x16, 0x16, 0x24, 0x1, 0x1, 0x1, 0x3, 0x32, 0x1,
	 0x8, 0x10, 0x16, 0x2, 0x38,}, /* LARB23 */
	[24] = {0x20, 0x6, 0x6, 0x1, 0x1, 0x24, 0x2b, 0x7, 0x4, 0x1,
	 0x1, 0xf, 0x3, 0x5, 0x8, 0x8, 0x3, 0x8, 0x5, 0x23,
	 0x24, 0x4, 0x2, 0xb, 0x10, 0x17, 0x4, 0x8, 0x5, 0x1,
	 0x1, 0x6,}, /* LARB24 */
	[25] = {0x2, 0xc, 0x2, 0xc, 0x6, 0x6, 0x3, 0x3, 0x3, 0x1,
	 0x1, 0x2, 0x2,}, /* LARB25 */
	[26] = {0x2, 0xc, 0x2, 0xc, 0x6, 0x6, 0x3, 0x3, 0x3, 0x1,
	 0x1, 0x2, 0x2,}, /* LARB26 */
	[27] = {0x6, 0x2, 0xe, 0x6, 0x2, 0x14, 0x14, 0x4, 0x6,}, /* LARB27 */
	[28] = {0x2b, 0x8, 0x31, 0x10, 0x26, 0x15, 0x1, 0x10,}, /* LARB28 */
	[29] = {0x2, 0x2, 0x2, 0x2, 0x10, 0xe, 0x6, 0x6, 0x1, 0x1,
	 0x2, 0x2, 0x2, 0x2,}, /* LARB29 */
	[30] = {0x2, 0x2, 0x2, 0x2,}, /* LARB30 */
	{}, /* LARB31 */
	{0x1, 0x1, 0x1, 0x1, 0x2, 0x2, 0x32, 0x32, 0x1, 0x2,}, /* LARB32 */
	{0xa, 0x1, 0x1, 0x1, 0xa, 0xa, 0xa, 0x1, 0x26, 0x32,
	 0x32, 0x32, 0x32, 0x32, 0x2, 0x1,}, /* LARB33 */
	{0x4, 0x4, 0x40, 0x40, 0x1, 0x1, 0x2, 0x2, 0x4, 0x4,
	 0x1, 0x1, 0x1,}, /* LARB34 */
	{0x4, 0x4, 0x40, 0x40, 0x32, 0x1, 0x2, 0x2, 0x2, 0x4,
	 0x4, 0x2, 0x1, 0x1, 0x1, 0x1,}, /* LARB35 */
	{0x2, 0x2, 0x2, 0x40, 0x40, 0x40, 0x1, 0x2, 0x2, 0x2,
	 0x4, 0x4, 0x4, 0x1, 0x1,}, /* LARB36 */
	{0x2, 0x2, 0x2, 0x40, 0x40, 0x40, 0x1, 0x32, 0x32, 0x32,
	 0x2, 0x2, 0x2, 0x4, 0x4, 0x4, 0x2, 0x1,}, /* LARB37 */
	{0x29, 0x40, 0x40, 0x7, 0x4, 0x40, 0x4, 0x18, 0x1, 0x1,
	 0x1, 0x7, 0x4,}, /* LARB38 */
	{0x16, 0x4, 0x4, 0x8, 0x4, 0x6, 0x6, 0x13, 0x11, 0x20,
	 0x11, 0x1, 0x1, 0x1, 0x9, 0x8, 0x4, 0x6, 0x6,}, /* LARB39 */
	{0x9, 0x7, 0x7, 0xb, 0xf, 0x1d, 0x13, 0x6, 0x1, 0x1,
	 0x1, 0x6, 0x9, 0x7, 0xe, 0x3,}, /* LARB40 */
	{0x40, 0x8, 0x1, 0x1, 0x2, 0x1, 0x1, 0x1, 0x1, 0x1,
	 0x8, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	 0x1, 0x8, 0x8, 0x8, 0x8, 0x8, 0x1, 0x1, 0x1, 0x1,
	 0x1, 0x1,}, /* LARB41 */
	{0x1, 0x8, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	 0x8, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	 0x1, 0x8, 0x8, 0x8, 0x8, 0x8, 0x1, 0x1, 0x1, 0x1,
	 0x1, 0x1,}, /* LARB42 */
	{0x4, 0x4, 0x12, 0x8, 0x8, 0x16, 0x8, 0x6, 0xe, 0x6,
	 0x1e, 0x18, 0x16, 0xe, 0x8, 0xe, 0x8, 0x1, 0x1,}, /* LARB43 */
	{0x4, 0x4, 0x12, 0x8, 0x8, 0x16, 0x8, 0x6, 0xe, 0x6,
	 0x1e, 0x18, 0x16, 0xe, 0x8, 0xe, 0x8, 0x1, 0x1,}, /* LARB44 */
	{0x18, 0x18, 0x8, 0x8, 0xc, 0x4, 0x1,}, /* LARB45 */
	{0x18, 0x18, 0x8, 0x8, 0xc, 0x4, 0x1,}, /* LARB46 */
	{0x1, 0x8, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	 0x8, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1,
	 0x1, 0x8, 0x8, 0x8, 0x8, 0x8, 0x1, 0x1, 0x1, 0x1,
	 0x1, 0x1,}, /* LARB47 */
};

static const struct mtk_smi_larb_gen mtk_smi_larb_mt2701 = {
	.port_in_larb = {
		LARB0_PORT_OFFSET, LARB1_PORT_OFFSET,
		LARB2_PORT_OFFSET, LARB3_PORT_OFFSET
	},
	.config_port = mtk_smi_larb_config_port_gen1,
};

static const struct mtk_smi_larb_gen mtk_smi_larb_mt2712 = {
	.config_port                = mtk_smi_larb_config_port_gen2_general,
	.larb_direct_to_common_mask = BIT(8) | BIT(9),      /* bdpsys */
};

static const struct mtk_smi_larb_gen mtk_smi_larb_mt6779 = {
	.config_port  = mtk_smi_larb_config_port_gen2_general,
	.larb_direct_to_common_mask =
		BIT(4) | BIT(6) | BIT(11) | BIT(12) | BIT(13),
		/* DUMMY | IPU0 | IPU1 | CCU | MDLA */
};

static const struct mtk_smi_larb_gen mtk_smi_larb_mt8167 = {
	/* mt8167 do not need the port in larb */
	.config_port = mtk_smi_larb_config_port_mt8167,
};

static const struct mtk_smi_larb_gen mtk_smi_larb_mt8173 = {
	/* mt8173 do not need the port in larb */
	.config_port = mtk_smi_larb_config_port_mt8173,
};

static const struct mtk_smi_larb_gen mtk_smi_larb_mt8183 = {
	.config_port                = mtk_smi_larb_config_port_gen2_general,
	.larb_direct_to_common_mask = BIT(2) | BIT(3) | BIT(7),
				      /* IPU0 | IPU1 | CCU */
};

static const struct mtk_smi_larb_gen mtk_smi_larb_mt8186 = {
	.config_port                = mtk_smi_larb_config_port_gen2_general,
	.flags_general	            = MTK_SMI_FLAG_SLEEP_CTL,
};

static const struct mtk_smi_larb_gen mtk_smi_larb_mt8188 = {
	.config_port                = mtk_smi_larb_config_port_gen2_general,
	.flags_general	            = MTK_SMI_FLAG_THRT_UPDATE | MTK_SMI_FLAG_SW_FLAG |
				      MTK_SMI_FLAG_SLEEP_CTL | MTK_SMI_FLAG_CFG_PORT_SEC_CTL,
	.ostd		            = mtk_smi_larb_mt8188_ostd,
};

static const struct mtk_smi_larb_gen mtk_smi_larb_mt8189 = {
	.config_port                = mtk_smi_larb_config_port_gen2_general,
	.flags_general	            = MTK_SMI_FLAG_THRT_UPDATE | MTK_SMI_FLAG_SW_FLAG |
				      MTK_SMI_FLAG_SLEEP_CTL | MTK_SMI_FLAG_CFG_PORT_SEC_CTL,
	.ostd		            = mtk_smi_larb_mt8189_ostd,
};

static const struct mtk_smi_larb_gen mtk_smi_larb_mt8192 = {
	.config_port                = mtk_smi_larb_config_port_gen2_general,
};

static const struct mtk_smi_larb_gen mtk_smi_larb_mt8195 = {
	.config_port                = mtk_smi_larb_config_port_gen2_general,
	.flags_general	            = MTK_SMI_FLAG_THRT_UPDATE | MTK_SMI_FLAG_SW_FLAG |
				      MTK_SMI_FLAG_SLEEP_CTL,
	.ostd		            = mtk_smi_larb_mt8195_ostd,
};

static const struct mtk_smi_larb_gen mtk_smi_larb_mt8196 = {
	.config_port                = mtk_smi_larb_config_port_gen2_general,
	.flags_general	            = MTK_SMI_FLAG_THRT_UPDATE | MTK_SMI_FLAG_SW_FLAG |
				      MTK_SMI_FLAG_SLEEP_CTL | MTK_SMI_FLAG_CONNECT_SMMUV3,
	.ostd		            = mtk_smi_larb_mt8196_ostd,
	.port_in_larb_gen2 	    = {13, 16, 18, 18, 8, 8, 3, 32, 32, 26,
				       8, 15, 3, 7, 6, 10, 19, 7, 5, 13, 15, 18,
				       15, 15, 32, 13, 13, 9, 8, 14, 4, 0, 10,
				       16, 13, 16, 15, 18, 14, 19, 16, 32, 32, 19,
				       19, 7, 7, 32,},
};

static const struct of_device_id mtk_smi_larb_of_ids[] = {
	{.compatible = "mediatek,mt2701-smi-larb", .data = &mtk_smi_larb_mt2701},
	{.compatible = "mediatek,mt2712-smi-larb", .data = &mtk_smi_larb_mt2712},
	{.compatible = "mediatek,mt6779-smi-larb", .data = &mtk_smi_larb_mt6779},
	{.compatible = "mediatek,mt6795-smi-larb", .data = &mtk_smi_larb_mt8173},
	{.compatible = "mediatek,mt8167-smi-larb", .data = &mtk_smi_larb_mt8167},
	{.compatible = "mediatek,mt8173-smi-larb", .data = &mtk_smi_larb_mt8173},
	{.compatible = "mediatek,mt8183-smi-larb", .data = &mtk_smi_larb_mt8183},
	{.compatible = "mediatek,mt8186-smi-larb", .data = &mtk_smi_larb_mt8186},
	{.compatible = "mediatek,mt8188-smi-larb", .data = &mtk_smi_larb_mt8188},
	{.compatible = "mediatek,mt8189-smi-larb", .data = &mtk_smi_larb_mt8189},
	{.compatible = "mediatek,mt8192-smi-larb", .data = &mtk_smi_larb_mt8192},
	{.compatible = "mediatek,mt8195-smi-larb", .data = &mtk_smi_larb_mt8195},
	{.compatible = "mediatek,mt8196-smi-larb", .data = &mtk_smi_larb_mt8196},
	{}
};

static int mtk_smi_larb_sleep_ctrl_enable(struct mtk_smi_larb *larb)
{
	int ret;
	u32 tmp;

	writel_relaxed(SLP_PROT_EN, larb->base + SMI_LARB_SLP_CON);
	ret = readl_poll_timeout_atomic(larb->base + SMI_LARB_SLP_CON,
					tmp, !!(tmp & SLP_PROT_RDY), 10, 1000);
	if (ret) {
		/* TODO: Reset this larb if it fails here. */
		dev_err(larb->smi.dev, "sleep ctrl is not ready(0x%x).\n", tmp);
	}
	return ret;
}

static void mtk_smi_larb_sleep_ctrl_disable(struct mtk_smi_larb *larb)
{
	writel_relaxed(0, larb->base + SMI_LARB_SLP_CON);
}

static int mtk_smi_device_link_common(struct device *dev, struct device **com_dev)
{
	struct platform_device *smi_com_pdev;
	struct device_node *smi_com_node;
	struct device *smi_com_dev;
	struct device_link *link;

	smi_com_node = of_parse_phandle(dev->of_node, "mediatek,smi", 0);
	if (!smi_com_node)
		return -EINVAL;

	smi_com_pdev = of_find_device_by_node(smi_com_node);
	of_node_put(smi_com_node);
	if (!smi_com_pdev) {
		dev_err(dev, "Failed to get the smi_common device\n");
		return -EINVAL;
	}

	/* smi common is the supplier, Make sure it is ready before */
	if (!platform_get_drvdata(smi_com_pdev)) {
		put_device(&smi_com_pdev->dev);
		return -EPROBE_DEFER;
	}

	smi_com_dev = &smi_com_pdev->dev;
	link = device_link_add(dev, smi_com_dev, DL_FLAG_PM_RUNTIME | DL_FLAG_STATELESS);
	if (!link) {
		dev_err(dev, "Unable to link smi-common dev\n");
		put_device(&smi_com_pdev->dev);
		return -ENODEV;
	}

	*com_dev = smi_com_dev;

	return 0;
}

static int mtk_smi_dts_clk_init(struct device *dev, struct mtk_smi *smi,
				const char * const clks[],
				unsigned int clk_nr_required,
				unsigned int clk_nr_optional)
{
	int i, ret;

	if (smi->skip_rpm)
		return 0;

	for (i = 0; i < clk_nr_required; i++)
		smi->clks[i].id = clks[i];
	ret = devm_clk_bulk_get(dev, clk_nr_required, smi->clks);
	if (ret)
		return ret;

	for (i = clk_nr_required; i < clk_nr_required + clk_nr_optional; i++)
		smi->clks[i].id = clks[i];
	ret = devm_clk_bulk_get_optional(dev, clk_nr_optional,
					 smi->clks + clk_nr_required);
	smi->clk_num = clk_nr_required + clk_nr_optional;
	return ret;
}

static int mtk_smi_larb_probe(struct platform_device *pdev)
{
	struct mtk_smi_larb *larb;
	struct device *dev = &pdev->dev;
	bool connect_with_smmuv3;
	const char *hrt_type;
	int ret, i;

	larb = devm_kzalloc(dev, sizeof(*larb), GFP_KERNEL);
	if (!larb)
		return -ENOMEM;

	larb->larb_gen = of_device_get_match_data(dev);
	larb->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(larb->base))
		return PTR_ERR(larb->base);

	if (of_property_read_bool(dev->of_node, "mediatek,skip-rpm-cb")) {
		larb->smi.skip_rpm = true;
		dev_info(dev, "skip rpm callback\n");
	}

	connect_with_smmuv3 = MTK_SMI_CAPS(larb->larb_gen->flags_general, MTK_SMI_FLAG_CONNECT_SMMUV3);
	if (connect_with_smmuv3 && !IS_ENABLED(CONFIG_ARM_SMMU_V3)) {
		dev_err(dev, " SMMU property conflict.\n");
		return -EINVAL;
	}
	ret = of_property_read_u32(dev->of_node, "mediatek,larb-id", &larb->larbid);
	if (connect_with_smmuv3 && ret)
		return ret;

	ret = mtk_smi_dts_clk_init(dev, &larb->smi, mtk_smi_larb_clks,
				   MTK_SMI_LARB_REQ_CLK_NR, MTK_SMI_LARB_OPT_CLK_NR);
	if (ret)
		return ret;

	larb->smi.dev = dev;

	hrt_type = of_get_property(dev->of_node, "larb-port-real-time-type", NULL);
	if (hrt_type) {
		larb->is_real_time_perm = true;
		for (i = 0; i < larb->larb_gen->port_in_larb_gen2[larb->larbid]; i++) {
			ret = of_property_read_u32_index(dev->of_node, "larb-port-real-time-type",
							 i, &larb->larb_port_real_time_type[i]);
			if (ret)
				larb->larb_port_real_time_type[i] = ALWAYS_SRT;
			 /* Default as SRT */
			if (larb->larb_port_real_time_type[i] == ALWAYS_HRT ||
			    larb->larb_port_real_time_type[i] == ALWAYS_SRT ||
			    larb->larb_port_real_time_type[i] == HRT_SRT_SWITCH)
			    	larb->real_time_type = larb->real_time_type | BIT(i);
		}
	}

	if (!larb->smi.skip_rpm) {
		ret = mtk_smi_device_link_common(dev, &larb->smi_common_dev);
		if (ret < 0) {
			dev_err(dev, "lnk fail %d\n", ret);
			return ret;
		}

		pm_runtime_enable(dev);
	}

	platform_set_drvdata(pdev, larb);
	if (!connect_with_smmuv3) {
		ret = component_add(dev, &mtk_smi_larb_component_ops);
		if (ret)
			goto err_pm_disable;
	}
	return 0;

err_pm_disable:
	if (!larb->smi.skip_rpm)
		pm_runtime_disable(dev);
	device_link_remove(dev, larb->smi_common_dev);
	put_device(larb->smi_common_dev);
	return ret;
}

static int mtk_smi_larb_remove(struct platform_device *pdev)
{
	struct mtk_smi_larb *larb = platform_get_drvdata(pdev);

	device_link_remove(&pdev->dev, larb->smi_common_dev);
	pm_runtime_disable(&pdev->dev);
	if (!MTK_SMI_CAPS(larb->larb_gen->flags_general, MTK_SMI_FLAG_CONNECT_SMMUV3))
		component_del(&pdev->dev, &mtk_smi_larb_component_ops);
	put_device(larb->smi_common_dev);
	return 0;
}

static int __maybe_unused mtk_smi_larb_resume(struct device *dev)
{
	struct mtk_smi_larb *larb = dev_get_drvdata(dev);
	const struct mtk_smi_larb_gen *larb_gen = larb->larb_gen;
	int ret;

	ret = clk_bulk_prepare_enable(larb->smi.clk_num, larb->smi.clks);
	if (ret)
		return ret;

	if (MTK_SMI_CAPS(larb->larb_gen->flags_general, MTK_SMI_FLAG_SLEEP_CTL))
		mtk_smi_larb_sleep_ctrl_disable(larb);

	/* Configure the basic setting for this larb */
	return larb_gen->config_port(dev);
}

static int __maybe_unused mtk_smi_larb_suspend(struct device *dev)
{
	struct mtk_smi_larb *larb = dev_get_drvdata(dev);
	int ret;

	if (MTK_SMI_CAPS(larb->larb_gen->flags_general, MTK_SMI_FLAG_SLEEP_CTL)) {
		ret = mtk_smi_larb_sleep_ctrl_enable(larb);
		if (ret)
			return ret;
	}

	clk_bulk_disable_unprepare(larb->smi.clk_num, larb->smi.clks);
	return 0;
}

static const struct dev_pm_ops smi_larb_pm_ops = {
	SET_RUNTIME_PM_OPS(mtk_smi_larb_suspend, mtk_smi_larb_resume, NULL)
	SET_LATE_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				     pm_runtime_force_resume)
};

static struct platform_driver mtk_smi_larb_driver = {
	.probe	= mtk_smi_larb_probe,
	.remove	= mtk_smi_larb_remove,
	.driver	= {
		.name = "mtk-smi-larb",
		.of_match_table = mtk_smi_larb_of_ids,
		.pm             = &smi_larb_pm_ops,
	}
};

static const struct mtk_smi_reg_pair mtk_smi_common_mt6795_init[1][SMI_COMMON_INIT_REGS_NR] = {
	{{SMI_L1_ARB, 0x1b}, {SMI_M4U_TH, 0xce810c85}, {SMI_FIFO_TH1, 0x43214c8}, {SMI_READ_FIFO_TH, 0x191f}},
};

static const struct mtk_smi_reg_pair mtk_smi_common_mt8195_init[1][SMI_COMMON_INIT_REGS_NR] = {
	{{SMI_L1LEN, 0xb}, {SMI_M4U_TH, 0xe100e10}, {SMI_FIFO_TH1, 0x506090a}, {SMI_FIFO_TH2, 0x506090a},
	 {SMI_DCM, 0x4f1}, {SMI_DUMMY, 0x1}},
};

static const struct mtk_smi_reg_pair
mtk_smi_common_mt8196_init[MTK_COMMON_NR_MAX][SMI_COMMON_INIT_REGS_NR] = {
	{{SMI_L1LEN, 0xa}, {SMI_BUS_SEL, 0x114}, {SMI_M4U_TH, 0x28402840},
	 {SMI_FIFO_TH1, 0x1e301e30}, {SMI_FIFO_TH2, 0x1e301e30}, {SMI_DCM, 0x4f1},
	 {SMI_DUMMY, 0x1}, {SMI_L1ARB(2), 0x8000}, {SMI_L1ARB(3), 0x8000},
	 {SMI_L1ARB(4), 0x8000},}, /* COMM0 */
	{{SMI_L1LEN, 0xa}, {SMI_BUS_SEL, 0x1111}, {SMI_M4U_TH, 0x28402840},
	 {SMI_FIFO_TH1, 0x1e301e30}, {SMI_FIFO_TH2, 0x1e301e30}, {SMI_DCM, 0x4f1},
	 {SMI_DUMMY, 0x1}, {SMI_L1ARB(2), 0x8000}, {SMI_L1ARB(3), 0x8000},
	 {SMI_L1ARB(4), 0x8000}, {SMI_L1ARB(5), 0x8000},}, /* COMM1 */
	{{SMI_L1LEN, 0x2}, {SMI_BUS_SEL, 0x444}, {SMI_DCM, 0x4f1},
	 {SMI_DUMMY, 0x1},}, /* COMM2 */
	{{SMI_DUMMY, 0x1},}, /* COMM3 */
	{{SMI_DUMMY, 0x1},}, /* COMM4 */
	{{SMI_DUMMY, 0x1},}, /* COMM5 */
	{{SMI_DUMMY, 0x1},}, /* COMM6 */
	{{SMI_DUMMY, 0x1},}, /* COMM7 */
	{{SMI_DUMMY, 0x1},}, /* COMM8 */
	{{SMI_L1LEN, 0xa}, {SMI_PREULTRA_MASK1, 0x2146}, {SMI_DUMMY, 0x1},}, /* COMM9 */
	{{SMI_L1LEN, 0xa}, {SMI_PREULTRA_MASK1, 0x2146}, {SMI_DUMMY, 0x1},}, /* COMM10 */
	{{SMI_L1LEN, 0xa}, {SMI_PREULTRA_MASK1, 0x2146}, {SMI_DUMMY, 0x1},}, /* COMM11 */
	{{SMI_L1LEN, 0xa}, {SMI_PREULTRA_MASK1, 0x2146}, {SMI_DUMMY, 0x1},}, /* COMM12 */
	{{SMI_DUMMY, 0x1},}, /* COMM13 */
	{{SMI_DUMMY, 0x1},}, /* COMM14 */
	{{SMI_DUMMY, 0x1},}, /* COMM15 */
	{{SMI_DUMMY, 0x1},}, /* COMM16 */
	{{SMI_DUMMY, 0x1},}, /* COMM17 */
	{{SMI_DUMMY, 0x1},}, /* COMM18 */
	{{SMI_DUMMY, 0x1},}, /* COMM19 */
	{{SMI_DUMMY, 0x1},}, /* COMM20 */
	{{SMI_DUMMY, 0x1},}, /* COMM21 */
	{{SMI_DUMMY, 0x1},}, /* COMM22 */
	{{SMI_DUMMY, 0x1},}, /* COMM23 */
	{{SMI_DUMMY, 0x1},}, /* COMM24 */
	{{SMI_DUMMY, 0x1},}, /* COMM25 */
	{{SMI_L1LEN, 0xa}, {SMI_PREULTRA_MASK1, 0x2146}, {SMI_DUMMY, 0x1},}, /* COMM26 */
	{{SMI_L1LEN, 0xa}, {SMI_PREULTRA_MASK1, 0x2146}, {SMI_DUMMY, 0x1},}, /* COMM27 */
	{{SMI_DUMMY, 0x1},}, /* COMM28 */
	{{SMI_L1LEN, 0xa}, {SMI_PREULTRA_MASK1, 0x2146}, {SMI_DUMMY, 0x1},}, /* COMM29 */
	{{SMI_DUMMY, 0x1},}, /* COMM30 */
};

static const struct mtk_smi_common_plat mtk_smi_common_gen1 = {
	.type     = MTK_SMI_GEN1,
};

static const struct mtk_smi_common_plat mtk_smi_common_gen2 = {
	.type	  = MTK_SMI_GEN2,
};

static const struct mtk_smi_common_plat mtk_smi_common_mt6779 = {
	.type	  = MTK_SMI_GEN2,
	.has_gals = true,
	.bus_sel  = F_MMU1_LARB(1) | F_MMU1_LARB(2) | F_MMU1_LARB(4) |
		    F_MMU1_LARB(5) | F_MMU1_LARB(6) | F_MMU1_LARB(7),
};

static const struct mtk_smi_common_plat mtk_smi_common_mt6795 = {
	.type	  = MTK_SMI_GEN2,
	.has_init = true,
	.bus_sel  = F_MMU1_LARB(0),
	.init     = mtk_smi_common_mt6795_init,
};

static const struct mtk_smi_common_plat mtk_smi_common_mt8183 = {
	.type     = MTK_SMI_GEN2,
	.has_gals = true,
	.bus_sel  = F_MMU1_LARB(1) | F_MMU1_LARB(2) | F_MMU1_LARB(5) |
		    F_MMU1_LARB(7),
};

static const struct mtk_smi_common_plat mtk_smi_common_mt8186 = {
	.type     = MTK_SMI_GEN2,
	.has_gals = true,
	.bus_sel  = F_MMU1_LARB(1) | F_MMU1_LARB(4) | F_MMU1_LARB(7),
};

static const struct mtk_smi_common_plat mtk_smi_common_mt8188_vdo = {
	.type     = MTK_SMI_GEN2,
	.has_init = true,
	.bus_sel  = F_MMU1_LARB(1) | F_MMU1_LARB(5) | F_MMU1_LARB(7),
	.init     = mtk_smi_common_mt8195_init,
};

static const struct mtk_smi_common_plat mtk_smi_common_mt8188_vpp = {
	.type     = MTK_SMI_GEN2,
	.has_init = true,
	.bus_sel  = F_MMU1_LARB(1) | F_MMU1_LARB(2) | F_MMU1_LARB(7),
	.init     = mtk_smi_common_mt8195_init,
};

static const struct mtk_smi_common_plat mtk_smi_common_mt8189 = {
	/* SMI common misc setting applied in MMInfra for mt8189 */
	.type      = MTK_SMI_GEN2,
};

static const struct mtk_smi_common_plat mtk_smi_sub_common_mt8189 = {
	.type     = MTK_SMI_GEN2_SUB_COMM,
};

static const struct mtk_smi_common_plat mtk_smi_common_mt8192 = {
	.type     = MTK_SMI_GEN2,
	.has_gals = true,
	.has_init = true,
	.bus_sel  = F_MMU1_LARB(1) | F_MMU1_LARB(2) | F_MMU1_LARB(5) |
		    F_MMU1_LARB(6),
};

static const struct mtk_smi_common_plat mtk_smi_common_mt8195_vdo = {
	.type     = MTK_SMI_GEN2,
	.has_gals = true,
	.has_init = true,
	.bus_sel  = F_MMU1_LARB(1) | F_MMU1_LARB(3) | F_MMU1_LARB(5) |
		    F_MMU1_LARB(7),
	.init     = mtk_smi_common_mt8195_init,
};

static const struct mtk_smi_common_plat mtk_smi_common_mt8195_vpp = {
	.type     = MTK_SMI_GEN2,
	.has_gals = true,
	.has_init = true,
	.bus_sel  = F_MMU1_LARB(1) | F_MMU1_LARB(2) | F_MMU1_LARB(7),
	.init     = mtk_smi_common_mt8195_init,
};

static const struct mtk_smi_common_plat mtk_smi_sub_common_mt8195 = {
	.type     = MTK_SMI_GEN2_SUB_COMM,
	.has_gals = true,
};

static const struct mtk_smi_common_plat mtk_smi_common_mt8365 = {
	.type     = MTK_SMI_GEN2,
	.bus_sel  = F_MMU1_LARB(2) | F_MMU1_LARB(4),
};

static const struct mtk_smi_common_plat mtk_smi_common_mt8196 = {
	.type	      = MTK_SMI_GEN2,
	.has_gals     = true,
	.has_init     = true,
	.has_p2_ostdl = true,
	.init	      = mtk_smi_common_mt8196_init,
};

static const struct mtk_smi_common_plat mtk_smi_sub_common_mt8196 = {
	.type     = MTK_SMI_GEN2_SUB_COMM,
};

static const struct of_device_id mtk_smi_common_of_ids[] = {
	{.compatible = "mediatek,mt2701-smi-common", .data = &mtk_smi_common_gen1},
	{.compatible = "mediatek,mt2712-smi-common", .data = &mtk_smi_common_gen2},
	{.compatible = "mediatek,mt6779-smi-common", .data = &mtk_smi_common_mt6779},
	{.compatible = "mediatek,mt6795-smi-common", .data = &mtk_smi_common_mt6795},
	{.compatible = "mediatek,mt8167-smi-common", .data = &mtk_smi_common_gen2},
	{.compatible = "mediatek,mt8173-smi-common", .data = &mtk_smi_common_gen2},
	{.compatible = "mediatek,mt8183-smi-common", .data = &mtk_smi_common_mt8183},
	{.compatible = "mediatek,mt8186-smi-common", .data = &mtk_smi_common_mt8186},
	{.compatible = "mediatek,mt8188-smi-common-vdo", .data = &mtk_smi_common_mt8188_vdo},
	{.compatible = "mediatek,mt8188-smi-common-vpp", .data = &mtk_smi_common_mt8188_vpp},
	{.compatible = "mediatek,mt8189-smi-common", .data = &mtk_smi_common_mt8189},
	{.compatible = "mediatek,mt8189-smi-sub-common", .data = &mtk_smi_sub_common_mt8189},
	{.compatible = "mediatek,mt8192-smi-common", .data = &mtk_smi_common_mt8192},
	{.compatible = "mediatek,mt8195-smi-common-vdo", .data = &mtk_smi_common_mt8195_vdo},
	{.compatible = "mediatek,mt8195-smi-common-vpp", .data = &mtk_smi_common_mt8195_vpp},
	{.compatible = "mediatek,mt8195-smi-sub-common", .data = &mtk_smi_sub_common_mt8195},
	{.compatible = "mediatek,mt8365-smi-common", .data = &mtk_smi_common_mt8365},
	{.compatible = "mediatek,mt8196-smi-common", .data = &mtk_smi_common_mt8196},
	{.compatible = "mediatek,mt8196-smi-sub-common", .data = &mtk_smi_sub_common_mt8196},
	{}
};

static int mtk_smi_common_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_smi *common;
	int ret, clk_required = MTK_SMI_COM_REQ_CLK_NR;

	common = devm_kzalloc(dev, sizeof(*common), GFP_KERNEL);
	if (!common)
		return -ENOMEM;
	common->dev = dev;
	common->plat = of_device_get_match_data(dev);

	if (of_property_read_bool(dev->of_node, "mediatek,skip-rpm-cb"))
		common->skip_rpm = true;

	if (!common->skip_rpm && common->plat->has_gals) {
		if (common->plat->type == MTK_SMI_GEN2)
			clk_required = MTK_SMI_COM_GALS_REQ_CLK_NR;
		else if (common->plat->type == MTK_SMI_GEN2_SUB_COMM)
			clk_required = MTK_SMI_SUB_COM_GALS_REQ_CLK_NR;
	}
	ret = mtk_smi_dts_clk_init(dev, common, mtk_smi_common_clks, clk_required, 0);
	if (ret)
		return ret;

	/*
	 * for mtk smi gen 1, we need to get the ao(always on) base to config
	 * m4u port, and we need to enable the aync clock for transform the smi
	 * clock into emi clock domain, but for mtk smi gen2, there's no smi ao
	 * base.
	 */
	if (common->plat->type == MTK_SMI_GEN1) {
		common->smi_ao_base = devm_platform_ioremap_resource(pdev, 0);
		if (IS_ERR(common->smi_ao_base))
			return PTR_ERR(common->smi_ao_base);

		common->clk_async = devm_clk_get(dev, "async");
		if (IS_ERR(common->clk_async))
			return PTR_ERR(common->clk_async);

		ret = clk_prepare_enable(common->clk_async);
		if (ret)
			return ret;
	} else {
		common->base = devm_platform_ioremap_resource(pdev, 0);
		if (IS_ERR(common->base))
			return PTR_ERR(common->base);
	}

	if (common->plat->has_init) {
		ret = of_property_read_u32(dev->of_node, "mediatek,common-id", &common->commid);
		if (ret)
			return ret;
	}
	/* link its smi-common if this is smi-sub-common */
	if (common->plat->type == MTK_SMI_GEN2_SUB_COMM && !common->skip_rpm) {
		ret = mtk_smi_device_link_common(dev, &common->smi_common_dev);
		if (ret < 0)
			return ret;
	}

	if (!common->skip_rpm)
		pm_runtime_enable(dev);
	platform_set_drvdata(pdev, common);
	dev_dbg(dev, "comm %d probe done", common->commid);
	return 0;
}

static int mtk_smi_common_remove(struct platform_device *pdev)
{
	struct mtk_smi *common = dev_get_drvdata(&pdev->dev);

	if (common->plat->type == MTK_SMI_GEN2_SUB_COMM)
		device_link_remove(&pdev->dev, common->smi_common_dev);
	pm_runtime_disable(&pdev->dev);
	put_device(common->smi_common_dev);
	return 0;
}

static int __maybe_unused mtk_smi_common_resume(struct device *dev)
{
	struct mtk_smi *common = dev_get_drvdata(dev);
	const struct mtk_smi_reg_pair *init;
	u32 bus_sel = common->plat->bus_sel; /* default is 0 */
	u32 *common_bwl = common->common_bwl;
	int ret, i;

	ret = clk_bulk_prepare_enable(common->clk_num, common->clks);
	if (ret)
		return ret;

	if (common->plat->type != MTK_SMI_GEN2)
		return 0;

	init = common->plat->init[common->commid];
	if (init) {
		for (i = 0; i < SMI_COMMON_INIT_REGS_NR && init[i].offset; i++)
			writel_relaxed(init[i].value, common->base + init[i].offset);
	} else {/* TODO for mt8195/mt8188 */
		writel(bus_sel, common->base + SMI_BUS_SEL);
	}

	for (i = 0; i < SMI_COMMON_LARB_NR_MAX; i++) {
		if (!common_bwl[i])
			continue;
		writel_relaxed(common_bwl[i], common->base + SMI_L1ARB(i));
	}
	return 0;
}

static int __maybe_unused mtk_smi_common_suspend(struct device *dev)
{
	struct mtk_smi *common = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(common->clk_num, common->clks);
	return 0;
}

static const struct dev_pm_ops smi_common_pm_ops = {
	SET_RUNTIME_PM_OPS(mtk_smi_common_suspend, mtk_smi_common_resume, NULL)
	SET_LATE_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				     pm_runtime_force_resume)
};

static struct platform_driver mtk_smi_common_driver = {
	.probe	= mtk_smi_common_probe,
	.remove = mtk_smi_common_remove,
	.driver	= {
		.name = "mtk-smi-common",
		.of_match_table = mtk_smi_common_of_ids,
		.pm             = &smi_common_pm_ops,
	}
};

static struct platform_driver * const smidrivers[] = {
	&mtk_smi_common_driver,
	&mtk_smi_larb_driver,
};

static int __init mtk_smi_init(void)
{
	return platform_register_drivers(smidrivers, ARRAY_SIZE(smidrivers));
}
module_init(mtk_smi_init);

static void __exit mtk_smi_exit(void)
{
	platform_unregister_drivers(smidrivers, ARRAY_SIZE(smidrivers));
}
module_exit(mtk_smi_exit);

MODULE_DESCRIPTION("MediaTek SMI driver");
MODULE_LICENSE("GPL v2");
