// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 MediaTek Inc.
 * Author: Echo.Zhang <echo.zhang@mediatek.com>
 */
#ifndef __MMQOS_MTK_C__
#define __MMQOS_MTK_C__

#include <dt-bindings/interconnect/mtk,mmqos.h>
#include <dt-bindings/memory/mtk-memory-port.h>
#include <linux/debugfs.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interconnect.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/proc_fs.h>
#include <linux/regulator/consumer.h>
#include <linux/sched/clock.h>
#if IS_ENABLED(CONFIG_MTK_MMDVFS)
#include <linux/soc/mediatek/mtk_mmdvfs.h>
#endif
#if IS_ENABLED(CONFIG_MTK_MMDVFS)
#include <soc/mediatek/mmdvfs_v3.h>
#endif
#include <soc/mediatek/smi.h>

#include "mmqos-global.h"
#include "mmqos-mtk.h"

#if IS_ENABLED(CONFIG_MTK_MMQOS_VCP)
#include "mmqos-vcp.h"
#include "mmqos-vcp-memory.h"
#endif

#define SHIFT_ROUND(a, b)                          ((((a) - 1) >> (b)) + 1)
#define ICC_TO_MBPS(x)                             ((x) / 1000)
#define MASK_4(a)                                  ((a) & GENMASK(3, 0))
#define MASK_8(a)                                  ((a) & GENMASK(7, 0))
#define MASK_16(a)                                 ((a) & GENMASK(15, 0))
#define COMM_PORT_COMM_ID(a)                       (((a) >> 8) & GENMASK(7, 0))
#define MULTIPLY_RATIO(value)                      ((value) * 1000)
#define NODE_TYPE(a)                               ((a) >> 16)
#define LARB_ID(a)                                 (MASK_8(a))
#define RATIO_BW(a)                                ((a) * 10 / 7)

#define VIRT_COMM_PORT_ID                          (8)
#define MAX_BW_UNIT                                (1023)

/* channel bw unit is 16MB/s */
#define CHNN_BW_UNIT_SHIFT                         (4)
/* dram bw unit is 64MB/s */
#define DRAM_BW_UNIT_SHIFT                         (6)

#define APMCU_BW_OFFSET(i)                         (4 * (i))

#define MMINFRA_DUMMY                              (0x400)
#define MAX_SUBSYS_NUM                             (6)

#define SUBSYS_HW_BW(sid)                          (0x20 * (sid))
#define SUBSYS_HW_BW_OFFSET(sid, i)                (SUBSYS_HW_BW(sid) + 0x4 * (i))
#define SUBSYS_HW_BW_HRT(sid)                      (0x8 * (sid))
#define SUBSYS_HW_BW_SRT(sid)                      (SUBSYS_HW_BW_HRT(sid) + 0x4)

#define TOTAL_BW(i)                                (4 * (i))
#define TOTAL_SRT_BW                               (4)

#define MAX_COMM_BW                                (0xfff)
#define MIN_COMM_BW                                (0x200)
#define MIX_BW_SHIFT                               (8)
#define DEFAULT_COMM_BW                            (0x1200)
#define PEAK_BW_LOW                                (0x1000)
#define PEAK_BW_HIGH                               (0x3000)
#define HARD_LIMIT_BIT                             (13)
#define CLK_MHZ                                    (1000000)

static u32 freq_mode = BY_REGULATOR;
static u32 mmqos_state;
u32 log_level = U32_MAX;

enum mmqos_rw_type {
	path_no_type = 0,
	path_write,
	path_read,
};

struct larb_port_node {
	struct mmqos_base_node *base;
	u32 old_avg_bw;
	u32 old_peak_bw;
	u16 bw_ratio;
	u8 channel;
	bool is_max_ostd;
	bool is_write;
};

enum hrt_ostdl_policy {
	HRT_OSTDL_1_5 = 0,
	HRT_OSTDL_1,
	HRT_OSTDL_2,
};


struct device *get_larb_dev(int larb_id);

static void mmqos_update_comm_bw(struct device *dev, u32 comm_port, u32 freq,
				 u64 mix_bw, u64 bw_peak, bool qos_bound, bool max_bwl)
{
	u32 comm_bw = 0;
	bool hard_limit;
	u32 value;

	if (!freq || !dev)
		return;

	if (mix_bw)
		comm_bw = (mix_bw << MIX_BW_SHIFT) / freq;

	if (max_bwl)
		comm_bw = MAX_COMM_BW;

	if (comm_bw)
		value = ((comm_bw > MAX_COMM_BW) ? MAX_COMM_BW :
			 ((comm_bw < MIN_COMM_BW) ? MIN_COMM_BW : comm_bw)) |
			 ((bw_peak > 0 || !qos_bound) ? PEAK_BW_LOW : PEAK_BW_HIGH);
	else
		value = DEFAULT_COMM_BW;

	hard_limit = !!(value & BIT(HARD_LIMIT_BIT));
	mtk_smi_common_bw_set(dev, comm_port, hard_limit, value);

	mmqos_dbg(dev, LOG_BW, "comm port=%d bw=%d freq=%d qos_bound=%d value=%#x",
		  comm_port, comm_bw, freq, qos_bound, value);
}

static void mmqos_update_comm_ostdl(struct device *dev, u32 comm_port,
				    u16 max_ratio, struct icc_node *larb)
{
	struct larb_node *larb_node = (struct larb_node *)larb->data;
	u16 bw_ratio;
	u32 value = 0;

	bw_ratio = larb_node->bw_ratio;
	if (larb->avg_bw) {
		value = SHIFT_ROUND(ICC_TO_MBPS(larb->avg_bw), bw_ratio);
		if (value > max_ratio)
			value = max_ratio;
	}

	mtk_smi_common_ostdl_set(dev, comm_port, larb_node->is_write, value);

	mmqos_dbg(dev, LOG_BW,
		  "%s larb_id=%lu comm port=%d is_write=%d bw_ratio=%d avg_bw=%d ostdl=%d",
		  __func__, LARB_ID(larb->id), comm_port, larb_node->is_write,
		  bw_ratio, larb->avg_bw, value);
}

static void mmqos_update_setting(struct mtk_mmqos *mmqos)
{
	struct common_port_node *comm_port;
	struct common_node *comm_node;
	u32 mmqos_state = mmqos->mmqos_state;

	list_for_each_entry(comm_node, &mmqos->comm_list, list) {
		comm_node->freq = clk_get_rate(comm_node->clk) / CLK_MHZ;
		if (mmqos_state & BWL_ENABLE) {
			list_for_each_entry(comm_port,
					    &comm_node->comm_port_list, list) {
				mutex_lock(&comm_port->bw_lock);
				if (comm_port->latest_mix_bw ||
				    comm_port->latest_peak_bw) {
					mmqos_update_comm_bw(comm_port->larb_dev,
							     MASK_8(comm_port->base->icc_node->id),
							     comm_port->common->freq,
							     ICC_TO_MBPS(comm_port->latest_mix_bw),
							     ICC_TO_MBPS(comm_port->latest_peak_bw),
							     mmqos->qos_bound,
							     comm_port->hrt_type == HRT_MAX_BWL);
				}
				mutex_unlock(&comm_port->bw_lock);
			}
		}
	}
}

static int update_mm_clk(struct notifier_block *nb, unsigned long value, void *v)
{
	struct mtk_mmqos *mmqos =
		container_of(nb, struct mtk_mmqos, nb);

	mmqos_update_setting(mmqos);

	return 0;
}

static unsigned long get_volt_by_freq(struct device *dev, unsigned long freq)
{
	struct dev_pm_opp *opp;
	unsigned long ret;

	opp = dev_pm_opp_find_freq_ceil(dev, &freq);

	/* It means freq is over the highest available frequency */
	if (opp == ERR_PTR(-ERANGE))
		opp = dev_pm_opp_find_freq_floor(dev, &freq);

	if (IS_ERR(opp)) {
		mmqos_dbg(dev, LOG_BW, "failed(%ld) freq=%lu", PTR_ERR(opp), freq);
		return 0;
	}

	ret = dev_pm_opp_get_voltage(opp);
	dev_pm_opp_put(opp);

	return ret;
}

static bool is_disp_comm_port(u8 hrt_type)
{
	if (hrt_type == HRT_MAX_BWL || hrt_type == HRT_DISP ||
	    hrt_type == HRT_DISP_BY_LARB)
		return true;

	return false;
}

static bool is_srt_comm_port(u8 hrt_type)
{
	if (hrt_type == SRT_VENC || hrt_type == SRT_MDP || hrt_type == SRT_MML)
		return true;

	return false;
}

static void set_total_bw_to_emi(struct common_node *comm_node, struct mtk_mmqos *mmqos)
{
	u32 avg_bw = 0, peak_bw = 0, total_bw_to_vcp = 0;
	struct common_port_node *comm_port_node;
	u64 normalize_peak_bw;
	u32 comm_id, t_ratio;
	u32 mmqos_state;

	mmqos_state = mmqos->mmqos_state;
	list_for_each_entry(comm_port_node, &comm_node->comm_port_list, list) {
		mutex_lock(&comm_port_node->bw_lock);
		total_bw_to_vcp += comm_port_node->latest_avg_bw + comm_port_node->latest_peak_bw;

		if (mmqos_state & DPC_ENABLE && is_disp_comm_port(comm_port_node->hrt_type)) {
			mmqos_dbg(mmqos->dev, LOG_DEBUG, "ignore disp comm port bw");
		} else if (mmqos_state & MMPC_ENABLE) {
			if (is_srt_comm_port(comm_port_node->hrt_type))
				avg_bw += comm_port_node->latest_avg_bw;
		} else {
			if (mmqos_state & VCODEC_BW_BYPASS &&
			    comm_port_node->hrt_type == SRT_VDEC) {
				mmqos_dbg(mmqos->dev, LOG_DEBUG, "ignore SRT_VDEC comm port bw");
			} else if (mmqos_state & VCODEC_BW_BYPASS &&
				   comm_port_node->hrt_type == SRT_VENC) {
				mmqos_dbg(mmqos->dev, LOG_DEBUG, "ignore SRT_VENC comm port bw");
			} else {
				avg_bw += comm_port_node->latest_avg_bw;
			}

			if (comm_port_node->hrt_type < HRT_TYPE_NUM) {
				t_ratio = mtk_mmqos_get_hrt_ratio(comm_port_node->hrt_type);
				normalize_peak_bw =
					MULTIPLY_RATIO(comm_port_node->latest_peak_bw) / t_ratio;
				peak_bw += normalize_peak_bw;
			}
		}
		mutex_unlock(&comm_port_node->bw_lock);
	}

	comm_id = MASK_8(comm_node->base->icc_node->id);

	mmqos_dbg(mmqos->dev, LOG_BW, "comm%d avg %d peak %d",
		  comm_id, ICC_TO_MBPS(avg_bw), ICC_TO_MBPS(peak_bw));

#if IS_ENABLED(CONFIG_MTK_MMQOS_VCP)
	if (MEM_BASE) {
		writel(total_bw_to_vcp, MEM_APMCU_TOTAL_BW);
		mmqos_dbg(mmqos->dev, LOG_BW, "total_bw_to_vcp:%d", total_bw_to_vcp);
	}
#endif
	icc_set_bw(comm_node->icc_path, avg_bw, 0);
	icc_set_bw(comm_node->icc_hrt_path, peak_bw, 0);
}

static u32 get_max_channel_bw_in_common(u32 comm_id, struct mtk_mmqos *mmqos)
{
	u32 max_bw = 0, i;

	for (i = 0; i < MMQOS_COMM_CHANNEL_NUM; i++) {
		max_bw = max_t(u32, max_bw, RATIO_BW(mmqos->chn_bw->chn_hrt_r_bw[comm_id][i]));
		max_bw = max_t(u32, max_bw, mmqos->chn_bw->chn_srt_r_bw[comm_id][i]);
		max_bw = max_t(u32, max_bw, RATIO_BW(mmqos->chn_bw->chn_hrt_w_bw[comm_id][i]));
		max_bw = max_t(u32, max_bw, mmqos->chn_bw->chn_srt_w_bw[comm_id][i]);
	}

	return max_bw;
}

static u32 get_max_channel_bw(u32 comm_id, u32 freq_mode, struct mtk_mmqos *mmqos)
{
	u32 max_bw = 0, comm_bw, i;

	if (freq_mode == BY_REGULATOR)
		return get_max_channel_bw_in_common(comm_id, mmqos);

	for (i = 0; i < MMQOS_MAX_COMM_NUM; i++) {
		comm_bw = get_max_channel_bw_in_common(i, mmqos);
		max_bw = max_t(u32, max_bw, comm_bw);
	}

	return max_bw;
}

static void set_freq_by_regulator(struct common_node *comm_node,
				  unsigned long smi_clk)
{
	u32 volt;

	volt = get_volt_by_freq(comm_node->comm_dev, smi_clk);
	if (volt > 0 && volt != comm_node->volt) {
		if (IS_ERR_OR_NULL(comm_node->comm_reg)) {
			dev_notice(comm_node->comm_dev, "comm_reg not existed\n");
		} else if (regulator_set_voltage(comm_node->comm_reg, volt, INT_MAX)) {
			dev_notice(comm_node->comm_dev,
				   "regulator_set_voltage failed volt=%u\n", volt);
		}

		comm_node->volt = volt;
	}
}

static u32 change_to_unit(u32 bw, struct mtk_mmqos *mmqos)
{
	/* bw unit is 16MB/s */
	u32 bw_unit;

	bw_unit = (ICC_TO_MBPS(bw)) >> CHNN_BW_UNIT_SHIFT;
	if (bw_unit >= MAX_BW_UNIT) {
		mmqos_err(mmqos->dev, "bw is over %d*16MB/s", MAX_BW_UNIT);
		bw_unit = MAX_BW_UNIT;
	}

	if (bw > 0)
		mmqos_dbg(mmqos->dev, LOG_COMM_FREQ, "bw:%d, bw_unit:%d", bw, bw_unit);

	return bw_unit;
}

static void store_bw_value(const u32 comm_id, const u32 chnn_id, bool is_srt, bool is_write,
			   bool is_on, u32 bw, struct mtk_mmqos *mmqos)
{
	u32 mmqos_state = mmqos->mmqos_state;
	int bw_value_idx;

	if (mmqos_state & VMMRC_ENABLE) {
		bw_value_idx = (comm_id << 2) + (chnn_id << 1) + (is_srt ? 0 : 12)
			+ (is_write ? 1 : 0);
	} else if (mmqos_state & MMPC_ENABLE) {
		bw_value_idx = (comm_id << 3) + (chnn_id << 2) + (is_srt ? 0 : 2)
			+ (is_write ? 1 : 0);
	} else {
		mmqos_err(mmqos->dev, "mmqos_state:%#x, ignore store bw", mmqos_state);
		return;
	}

	if (!is_srt)
		bw = RATIO_BW(bw);

	bw = change_to_unit(bw, mmqos);

	if (is_on)
		mmqos->on_bw_value[bw_value_idx] = bw;
	else
		mmqos->off_bw_value[bw_value_idx] = bw;

	if (bw != 0)
		mmqos_dbg(mmqos->dev, LOG_COMM_FREQ, "bw_value_idx:%d, bw:%d", bw_value_idx, bw);
}

static bool is_bw_value_changed(bool is_on, struct mtk_mmqos *mmqos)
{
	u32 *bw_value;
	u32 *old_bw;
	int i = 0;

	if (is_on) {
		old_bw = mmqos->old_on_bw_value;
		bw_value = mmqos->on_bw_value;
	} else {
		old_bw = mmqos->old_off_bw_value;
		bw_value = mmqos->off_bw_value;
	}

	for (i = 0 ; i < MAX_BW_VALUE_NUM ; i++) {
		if (bw_value[i] != old_bw[i])
			return true;
	}

	return false;
}

static void write_register(u32 offset, u32 value, struct mtk_mmqos *mmqos)
{
	u32 mmqos_state;

	mmqos_state = mmqos->mmqos_state;
	if (value != 0)
		mmqos_dbg(mmqos->dev, LOG_BW, "offset:0x%x, value:0x%x", offset, value);

	if (!(mmqos_state & VMMRC_ENABLE) && !(mmqos_state & MMPC_ENABLE))
		return;

	if (!mmqos->vmmrc_base)
		mmqos_err(mmqos->dev, "write vmmrc fail, vmmrc_base is NULL");
	else
		writel_relaxed(value, mmqos->vmmrc_base + offset);
}

static u32 read_register(u32 offset, struct mtk_mmqos *mmqos)
{
	u32 mmqos_state;

	mmqos_state = mmqos->mmqos_state;
	if (!(mmqos_state & VMMRC_ENABLE) && !(mmqos_state & MMPC_ENABLE))
		return 0;

	if (!mmqos->vmmrc_base)
		mmqos_err(mmqos->dev, "read vmmrc fail, vmmrc_base is NULL");
	else
		return readl_relaxed(mmqos->vmmrc_base + offset);

	return 0;
}

static void start_write_bw(struct mtk_mmqos *mmqos)
{
	u32 apmcu_mask_offset;
	u32 mmqos_state;
	u32 orig = 0;

	apmcu_mask_offset = mmqos->apmcu_mask_offset;
	mmqos_state = mmqos->mmqos_state;

	/* for hfrp timeout debug */
	readl_relaxed(mmqos->mminfra_base + MMINFRA_DUMMY);
	/* enable vcp */
#if IS_ENABLED(CONFIG_MTK_MMDVFS)
	if (mmqos_state & VMMRC_VCP_ENABLE)
		mtk_mmdvfs_enable_vcp(true, VCP_PWR_USR_MMQOS);
#endif

	if (mmqos_state & MMPC_ENABLE) {
		write_register(apmcu_mask_offset, 0, mmqos);
	} else {
		orig = read_register(apmcu_mask_offset, mmqos);
		write_register(apmcu_mask_offset, orig | BIT(mmqos->apmcu_mask_bit), mmqos);
	}
}

static void stop_write_bw(struct mtk_mmqos *mmqos)
{
	u32 apmcu_mask_offset;
	u32 mmqos_state;
	u32 orig = 0;

	apmcu_mask_offset = mmqos->apmcu_mask_offset;
	mmqos_state = mmqos->mmqos_state;

	if (mmqos_state & MMPC_ENABLE) {
		write_register(apmcu_mask_offset, mmqos->mmpc_sw_en_all_on, mmqos);
	} else {
		orig = read_register(apmcu_mask_offset, mmqos);
		write_register(apmcu_mask_offset, orig & ~BIT(mmqos->apmcu_mask_bit), mmqos);
	}

	/* disable vcp */
#if IS_ENABLED(CONFIG_MTK_MMDVFS)
	if (mmqos_state & VMMRC_VCP_ENABLE)
		mtk_mmdvfs_enable_vcp(false, VCP_PWR_USR_MMQOS);
#endif
}

void clear_reg_value(bool is_on, struct mtk_mmqos *mmqos)
{
	if (is_on)
		memset(mmqos->on_reg_value, 0, MAX_REG_VALUE_NUM * sizeof(mmqos->on_reg_value[0]));
	else
		memset(mmqos->off_reg_value, 0,
		       MAX_REG_VALUE_NUM * sizeof(mmqos->off_reg_value[0]));
}

void set_channel_bw_reg_value(bool is_on, struct mtk_mmqos *mmqos)
{
	int i, reg_value_idx;
	u32 *old_bw_value;
	u32 *reg_value;
	u32 *bw_value;

	clear_reg_value(is_on, mmqos);

	if (is_on) {
		reg_value = mmqos->on_reg_value;
		bw_value = mmqos->on_bw_value;
		old_bw_value = mmqos->old_on_bw_value;
	} else {
		reg_value = mmqos->off_reg_value;
		bw_value = mmqos->off_bw_value;
		old_bw_value = mmqos->old_off_bw_value;
	}

	for (i = 0 ; i < MAX_BW_VALUE_NUM ; i++) {
		if (bw_value[i] != 0)
			mmqos_dbg(mmqos->dev, LOG_DEBUG, "is_on:%d, i:%d, bw_value:%d",
				  is_on, i, bw_value[i]);

		reg_value_idx = i / 3;
		reg_value[reg_value_idx] += (bw_value[i] & GENMASK(9, 0)) << ((i % 3) * 10);

		if (reg_value[reg_value_idx] != 0)
			mmqos_dbg(mmqos->dev, LOG_DEBUG, "is_on:%d, idx:%d, reg_value:%#x",
				  is_on, reg_value_idx, reg_value[reg_value_idx]);

		old_bw_value[i] = bw_value[i];
	}
}

static void set_channel_bw_to_hw(struct mtk_mmqos *mmqos)
{
	u32 apmcu_off_bw_offset = mmqos->apmcu_off_bw_offset;
	u32 apmcu_on_bw_offset = mmqos->apmcu_on_bw_offset;
	u32 mmqos_state = mmqos->mmqos_state;
	u32 offset;
	int i;

	start_write_bw(mmqos);

	for (i = 0 ; i < MAX_REG_VALUE_NUM; i++) {
		offset = apmcu_on_bw_offset + APMCU_BW_OFFSET(i);
		write_register(offset, mmqos->on_reg_value[i], mmqos);
	}

	if (mmqos_state & VMMRC_ENABLE) {
		for (i = 0 ; i < MAX_REG_VALUE_NUM; i++) {
			offset = apmcu_off_bw_offset + APMCU_BW_OFFSET(i);
			write_register(offset, mmqos->off_reg_value[i], mmqos);
		}
	}

	stop_write_bw(mmqos);
}

static void set_freq_by_vmmrc(const u32 comm_id, struct mtk_mmqos *mmqos)
{
	struct disp_bw *disp_bw = mmqos->disp_bw;
	struct chn_bw *chn_bw = mmqos->chn_bw;
	u32 mmqos_state = mmqos->mmqos_state;
	bool is_reg_value_changed = false;
	u32 off_s_r_bw;
	u32 off_s_w_bw;
	u32 off_h_r_bw;
	u32 off_h_w_bw;
	int i, j;

	for (i = 0; i < MMQOS_MAX_COMM_NUM; i++) {
		for (j = 0; j < MMQOS_COMM_CHANNEL_NUM; j++) {
			mmqos_dbg(mmqos->dev, LOG_COMM_FREQ,
				  "comm(%d) chn=%d disp s_r=%u h_r=%u s_w=%u h_w=%u",
				  i, j, disp_bw->disp_srt_r_bw[i][j], disp_bw->disp_hrt_r_bw[i][j],
				  disp_bw->disp_srt_w_bw[i][j], disp_bw->disp_hrt_w_bw[i][j]);
		}
	}

	for (i = 0; i < MMQOS_COMM_CHANNEL_NUM; i++) {
		bool is_srt = true;
		bool is_write = true;

		store_bw_value(comm_id, i, is_srt, !is_write, true,
			       chn_bw->chn_srt_r_bw[comm_id][i], mmqos);

		store_bw_value(comm_id, i, is_srt, is_write, true,
			       chn_bw->chn_srt_w_bw[comm_id][i], mmqos);

		store_bw_value(comm_id, i, !is_srt, !is_write, true,
			       chn_bw->chn_hrt_r_bw[comm_id][i], mmqos);

		store_bw_value(comm_id, i, !is_srt, is_write, true,
			       chn_bw->chn_hrt_w_bw[comm_id][i], mmqos);

		if (mmqos_state & VMMRC_ENABLE) {
			off_s_r_bw =
				chn_bw->chn_srt_r_bw[comm_id][i] -
					disp_bw->disp_srt_r_bw[comm_id][i];
			off_s_w_bw =
				chn_bw->chn_srt_w_bw[comm_id][i] -
					disp_bw->disp_srt_w_bw[comm_id][i];
			off_h_r_bw =
				chn_bw->chn_hrt_r_bw[comm_id][i] -
					disp_bw->disp_hrt_r_bw[comm_id][i];
			off_h_w_bw =
				chn_bw->chn_hrt_w_bw[comm_id][i] -
					disp_bw->disp_hrt_w_bw[comm_id][i];

			store_bw_value(comm_id, i, is_srt, !is_write, false,
				       change_to_unit(off_s_r_bw, mmqos), mmqos);

			store_bw_value(comm_id, i, is_srt, is_write, false,
				       change_to_unit(off_s_w_bw, mmqos), mmqos);

			store_bw_value(comm_id, i, !is_srt, !is_write, false,
				       change_to_unit(off_h_r_bw, mmqos), mmqos);

			store_bw_value(comm_id, i, !is_srt, is_write, false,
				       change_to_unit(off_h_w_bw, mmqos), mmqos);

		}
	}

	if (is_bw_value_changed(true, mmqos), mmqos) {
		set_channel_bw_reg_value(true, mmqos);
		is_reg_value_changed = true;
	}

	if (mmqos_state & VMMRC_ENABLE) {
		if (is_bw_value_changed(false, mmqos), mmqos) {
			set_channel_bw_reg_value(false, mmqos);
			is_reg_value_changed = true;
		}
	}

	if (is_reg_value_changed)
		set_channel_bw_to_hw(mmqos);
}

static void set_comm_icc_bw(struct common_node *comm_node, struct mtk_mmqos *mmqos)
{
	unsigned long smi_clk = 0;
	u32 comm_id, i, j;
	u32 max_bw = 0;

	//set_total_bw_to_emi(comm_node, mmqos);

	comm_id = MASK_8(comm_node->base->icc_node->id);

		for (i = 0; i < MMQOS_MAX_COMM_NUM; i++) {
			for (j = 0; j < MMQOS_COMM_CHANNEL_NUM; j++) {
				mmqos_dbg(mmqos->dev, LOG_COMM_FREQ,
					  "comm(%d) chn=%d s_r=%u h_r=%u s_w=%u h_w=%u",
					  i, j, mmqos->chn_bw->chn_srt_r_bw[i][j],
					  mmqos->chn_bw->chn_hrt_r_bw[i][j],
					  mmqos->chn_bw->chn_srt_w_bw[i][j],
					  mmqos->chn_bw->chn_hrt_w_bw[i][j]);
			}
		}

	if (freq_mode == BY_REGULATOR || freq_mode == BY_MMDVFS) {
		max_bw = get_max_channel_bw(comm_id, freq_mode, mmqos);
		if (max_bw)
			smi_clk = SHIFT_ROUND(max_bw, 4) * 1000;
		else
			smi_clk = 0;

		mmqos_dbg(comm_node->comm_dev, LOG_COMM_FREQ,
			  "comm(%d) max_bw=%u smi_clk=%lu freq_mode=%d",
			  comm_id, max_bw, smi_clk, freq_mode);

		if (comm_node->comm_dev && smi_clk != comm_node->smi_clk) {
			if (freq_mode == BY_REGULATOR)
				set_freq_by_regulator(comm_node, smi_clk);
			else
				mmqos_err(mmqos->dev, "freq_mode:%d is wrong", freq_mode);

			comm_node->smi_clk = smi_clk;
		}
	} else if (freq_mode == BY_VMMRC) {
		//set_freq_by_vmmrc(comm_id, mmqos);
	}
}

static void update_hrt_bw(struct mtk_mmqos *mmqos)
{
	struct common_port_node *comm_port;
	u32 hrt_bw[HRT_TYPE_NUM] = {0};
	struct common_node *comm_node;
	u32 i;

	list_for_each_entry(comm_node, &mmqos->comm_list, list) {
		list_for_each_entry(comm_port,
				    &comm_node->comm_port_list, list) {
			if (comm_port->hrt_type < HRT_TYPE_NUM) {
				mutex_lock(&comm_port->bw_lock);
				hrt_bw[comm_port->hrt_type] +=
					ICC_TO_MBPS(comm_port->latest_peak_bw);
				mutex_unlock(&comm_port->bw_lock);
			}
		}
	}

	for (i = 0; i < HRT_TYPE_NUM; i++)
		if (i != HRT_MD)
			mtk_mmqos_set_hrt_bw(i, hrt_bw[i]);
}

static void record_last_larb(u32 node_id, u32 avg_bw, u32 peak_bw, struct mtk_mmqos *mmqos)
{
	mmqos->last_rec->larb_update_time = sched_clock();
	mmqos->last_rec->larb_node_id = node_id;
	mmqos->last_rec->larb_avg_bw = avg_bw;
	mmqos->last_rec->larb_peak_bw = peak_bw;
}

static void record_last_larb_port(u32 node_id, u32 avg_bw, u32 peak_bw, struct mtk_mmqos *mmqos)
{
	mmqos->last_rec->larb_port_update_time = sched_clock();
	mmqos->last_rec->larb_port_node_id = node_id;
	mmqos->last_rec->larb_port_avg_bw = avg_bw;
	mmqos->last_rec->larb_port_peak_bw = peak_bw;
}

static void record_comm_port_bw(u32 comm_id, u32 port_id, u32 larb_id,
				u32 avg_bw, u32 peak_bw,
				u32 l_avg, u32 l_peak,
				struct mtk_mmqos *mmqos)
{
	u32 idx;

	mmqos_dbg(mmqos->dev, LOG_BW, "comm%d port%d larb%d %d %d %d %d",
		  comm_id, port_id, larb_id,
		  ICC_TO_MBPS(avg_bw), ICC_TO_MBPS(peak_bw),
		  ICC_TO_MBPS(l_avg), ICC_TO_MBPS(l_peak));

	idx = mmqos->comm_port_bw_rec->idx[comm_id][port_id];
	mmqos->comm_port_bw_rec->time[comm_id][port_id][idx] = sched_clock();
	mmqos->comm_port_bw_rec->larb_id[comm_id][port_id][idx] = larb_id;
	mmqos->comm_port_bw_rec->avg_bw[comm_id][port_id][idx] = avg_bw;
	mmqos->comm_port_bw_rec->peak_bw[comm_id][port_id][idx] = peak_bw;
	mmqos->comm_port_bw_rec->l_avg_bw[comm_id][port_id][idx] = l_avg;
	mmqos->comm_port_bw_rec->l_peak_bw[comm_id][port_id][idx] = l_peak;
	mmqos->comm_port_bw_rec->idx[comm_id][port_id] = (idx + 1) % RECORD_NUM;
}

static void record_chn_bw(u32 comm_id, u32 chnn_id, u32 srt_r,
			  u32 srt_w, u32 hrt_r, u32 hrt_w,
			  struct mtk_mmqos *mmqos)
{
	u32 idx;

	idx = mmqos->chn_bw_rec->idx[comm_id][chnn_id];
	mmqos->chn_bw_rec->time[comm_id][chnn_id][idx] = sched_clock();
	mmqos->chn_bw_rec->srt_r_bw[comm_id][chnn_id][idx] = srt_r;
	mmqos->chn_bw_rec->srt_w_bw[comm_id][chnn_id][idx] = srt_w;
	mmqos->chn_bw_rec->hrt_r_bw[comm_id][chnn_id][idx] = hrt_r;
	mmqos->chn_bw_rec->hrt_w_bw[comm_id][chnn_id][idx] = hrt_w;
	mmqos->chn_bw_rec->idx[comm_id][chnn_id] = (idx + 1) % RECORD_NUM;
}

static void record_larb_port_bw_ostdl(u32 larb_id, u32 port_id,
				      u32 avg_bw, u32 peak_bw,
				      u32 mix_bw, u8 ostdl,
				      struct mtk_mmqos *mmqos)
{
	u32 idx;

	idx = mmqos->larb_port_bw_rec->idx[larb_id];
	mmqos->larb_port_bw_rec->time[larb_id][idx] = sched_clock();
	mmqos->larb_port_bw_rec->port_id[larb_id][idx] = port_id;
	mmqos->larb_port_bw_rec->avg_bw[larb_id][idx] = avg_bw;
	mmqos->larb_port_bw_rec->peak_bw[larb_id][idx] = peak_bw;
	mmqos->larb_port_bw_rec->mix_bw[larb_id][idx] = mix_bw;
	mmqos->larb_port_bw_rec->ostdl[larb_id][idx] = ostdl;
	mmqos->larb_port_bw_rec->idx[larb_id] = (idx + 1) % RECORD_NUM;
}

static void update_channel_bw(const u32 comm_id, const u32 chnn_id,
		       struct icc_node *src)
{
	struct common_port_node *comm_port_node;
	struct mtk_mmqos *mmqos = NULL;
	u32 half_hrt_r = 0;
	u32 mmqos_state;

	mmqos = mtk_mmqos_get_drv_data();
	if (IS_ERR_OR_NULL(mmqos)) {
		mmqos_err(mmqos->dev, "not ready");
		return;
	}

	comm_port_node = (struct common_port_node *)src->data;
	if (!comm_port_node) {
		mmqos_err(mmqos->dev, "fail, comm_port_node is NULL");
		return;
	}

	mmqos_state = mmqos->mmqos_state;
	if (mmqos_state & DISP_BY_LARB_ENABLE &&
	    comm_port_node->hrt_type == HRT_DISP) {
		mmqos_dbg(mmqos->dev, LOG_BW, "ignore HRT_DISP comm port:%#x", src->id);
		return;
	} else if (!(mmqos_state & DISP_BY_LARB_ENABLE) &&
		   comm_port_node->hrt_type == HRT_DISP_BY_LARB) {
		mmqos_dbg(mmqos->dev, LOG_BW, "ignore HRT_DISP_BY_LARB comm port:%#x", src->id);
		return;
	}

	if ((mmqos_state & MMPC_ENABLE) &&
	    !is_srt_comm_port(comm_port_node->hrt_type)) {
		mmqos_dbg(mmqos->dev, LOG_BW, "ignore not SW mode comm port:%#x", src->id);
		return;
	}

	if ((mmqos_state & DPC_ENABLE) &&
	    is_disp_comm_port(comm_port_node->hrt_type)) {
		/* display bw should be only write on the on table
		 * other mm users need to write on & off table.
		 * so we need to record display chnn bw
		 */
		mmqos->disp_bw->disp_srt_r_bw[comm_id][chnn_id] -= comm_port_node->old_avg_r_bw;
		mmqos->disp_bw->disp_srt_w_bw[comm_id][chnn_id] -= comm_port_node->old_avg_w_bw;
		mmqos->disp_bw->disp_hrt_r_bw[comm_id][chnn_id] -= comm_port_node->old_peak_r_bw;
		mmqos->disp_bw->disp_hrt_w_bw[comm_id][chnn_id] -= comm_port_node->old_peak_w_bw;
		mmqos->disp_bw->disp_srt_r_bw[comm_id][chnn_id] += src->v2_avg_r_bw;
		mmqos->disp_bw->disp_srt_w_bw[comm_id][chnn_id] += src->v2_avg_w_bw;
		mmqos->disp_bw->disp_hrt_r_bw[comm_id][chnn_id] += src->v2_peak_r_bw;
		mmqos->disp_bw->disp_hrt_w_bw[comm_id][chnn_id] += src->v2_peak_w_bw;
	}

	mmqos->chn_bw->v2_chn_hrt_w_bw[comm_id][chnn_id] -= comm_port_node->old_peak_w_bw;
	mmqos->chn_bw->v2_chn_hrt_r_bw[comm_id][chnn_id] -= comm_port_node->old_peak_r_bw;
	mmqos->chn_bw->v2_chn_srt_w_bw[comm_id][chnn_id] -= comm_port_node->old_avg_w_bw;
	mmqos->chn_bw->v2_chn_srt_r_bw[comm_id][chnn_id] -= comm_port_node->old_avg_r_bw;
	mmqos->chn_bw->v2_chn_hrt_w_bw[comm_id][chnn_id] += src->v2_peak_w_bw;
	mmqos->chn_bw->v2_chn_hrt_r_bw[comm_id][chnn_id] += src->v2_peak_r_bw;
	mmqos->chn_bw->v2_chn_srt_w_bw[comm_id][chnn_id] += src->v2_avg_w_bw;
	mmqos->chn_bw->v2_chn_srt_r_bw[comm_id][chnn_id] += src->v2_avg_r_bw;

	if (comm_port_node->hrt_type == HRT_DISP &&
	    mmqos->dual_pipe_enable) {
		half_hrt_r = (src->v2_peak_r_bw / 2);
		mmqos->chn_bw->v2_chn_hrt_r_bw[comm_id][chnn_id] -= half_hrt_r;
		if (mmqos_state & DPC_ENABLE)
			mmqos->disp_bw->disp_hrt_r_bw[comm_id][chnn_id] -= half_hrt_r;
	} else {
		half_hrt_r = (src->v2_peak_r_bw);
	}

	comm_port_node->old_peak_w_bw = src->v2_peak_w_bw;
	comm_port_node->old_peak_r_bw = half_hrt_r;
	comm_port_node->old_avg_w_bw = src->v2_avg_w_bw;
	comm_port_node->old_avg_r_bw = src->v2_avg_r_bw;


	mmqos->chn_bw->chn_hrt_w_bw[comm_id][chnn_id] =
		mmqos->chn_bw->v2_chn_hrt_w_bw[comm_id][chnn_id];
	mmqos->chn_bw->chn_srt_w_bw[comm_id][chnn_id] =
		mmqos->chn_bw->v2_chn_srt_w_bw[comm_id][chnn_id];
	mmqos->chn_bw->chn_hrt_r_bw[comm_id][chnn_id] =
		mmqos->chn_bw->v2_chn_hrt_r_bw[comm_id][chnn_id];
	mmqos->chn_bw->chn_srt_r_bw[comm_id][chnn_id] =
		mmqos->chn_bw->v2_chn_srt_r_bw[comm_id][chnn_id];

	mmqos_dbg(mmqos->dev, LOG_V2_DBG,
		  "[new] hrt_w_bw:%d  hrt_r_bw:%d srt_w_bw:%d srt_r_bw:%d",
		  mmqos->chn_bw->chn_hrt_w_bw[comm_id][chnn_id],
		  mmqos->chn_bw->chn_hrt_r_bw[comm_id][chnn_id],
		  mmqos->chn_bw->chn_srt_w_bw[comm_id][chnn_id],
		  mmqos->chn_bw->chn_srt_r_bw[comm_id][chnn_id]);
}

static inline bool is_max_bw_to_max_ostdl_policy(struct icc_node *src, struct mtk_mmqos *mmqos)
{
	u32 mmqos_state;

	mmqos_state = mmqos->mmqos_state;
	if (mmqos_state & FORCE_BW_TO_OSTDL) {
		mmqos_err(mmqos->dev, "larb=%d port=%d use MTK_MAX_MMQOS_BW",
			  MTK_M4U_TO_LARB(src->id), MTK_M4U_TO_PORT(src->id));

		return false;
	}

	return (!(mmqos_state & DPC_ENABLE) && !(mmqos_state & CAM_NO_MAX_OSTDL));

}

static int mtk_mmqos_set(struct icc_node *src, struct icc_node *dst)
{
	u32 comm_id, chnn_id, port_id;
	struct mtk_mmqos *mmqos = container_of(dst->provider,
					       struct mtk_mmqos, prov);
	struct common_port_node *comm_port_node;
	struct larb_port_node *larb_port_node;
	u32 mmqos_state = mmqos->mmqos_state;
	u32 r_hrt_ostdl = mmqos->r_hrt_ostdl;
	u32 w_hrt_ostdl = mmqos->w_hrt_ostdl;
	struct common_node *comm_node;
	struct larb_node *larb_node;
	u32 value = 1, t_bw, t_bw1;

	switch (NODE_TYPE(dst->id)) {
	case MTK_MMQOS_NODE_COMMON:
		comm_node = (struct common_node *)dst->data;
		comm_port_node = (struct common_port_node *)src->data;
		comm_id = MASK_4(comm_port_node->channel_v2 >> CHNN_BW_UNIT_SHIFT);
		chnn_id = MASK_4(comm_port_node->channel_v2);
		if (chnn_id) {
			chnn_id -= 1;
			update_channel_bw(comm_id, chnn_id, src);

			record_chn_bw(comm_id, chnn_id,
				      mmqos->chn_bw->chn_srt_r_bw[comm_id][chnn_id],
				      mmqos->chn_bw->chn_srt_w_bw[comm_id][chnn_id],
				      mmqos->chn_bw->chn_hrt_r_bw[comm_id][chnn_id],
				      mmqos->chn_bw->chn_hrt_w_bw[comm_id][chnn_id],
				      mmqos);
		}
		if (!comm_node)
			break;

		if (mmqos_state & DVFSRC_ENABLE) {
			set_comm_icc_bw(comm_node, mmqos);
			update_hrt_bw(mmqos);
		}
		break;
	case MTK_MMQOS_NODE_COMMON_PORT:
		comm_port_node = (struct common_port_node *)dst->data;
		larb_node = (struct larb_node *)src->data;

		if (!comm_port_node || !larb_node)
			break;

		comm_id = MASK_4(larb_node->channel >> CHNN_BW_UNIT_SHIFT);
		chnn_id = MASK_4(larb_node->channel_v2);

		if ((mmqos_state & DISP_BY_LARB_ENABLE) == 0 &&
		    (src->id == mmqos->disp_virt_larbs[1] ||
		     src->id == mmqos->disp_virt_larbs[2])) {
			mmqos_dbg(mmqos->dev, LOG_BW, "ignore larb%d", src->id);
			break;
		}

		record_last_larb(src->id, src->avg_bw, src->peak_bw, mmqos);

		mutex_lock(&comm_port_node->bw_lock);
		mmqos_dbg(mmqos->dev, LOG_V2_DBG,
			  "mix,peak,avg:%u,%llu,%u->dst:%u,%u,%u node_name:%s",
			  comm_port_node->latest_mix_bw, comm_port_node->latest_peak_bw,
			  comm_port_node->latest_avg_bw, dst->v2_mix_bw, dst->peak_bw,
			  dst->avg_bw, dst->name);

		if (comm_port_node->latest_mix_bw == dst->v2_mix_bw &&
		    comm_port_node->latest_peak_bw == dst->peak_bw &&
		    comm_port_node->latest_avg_bw == dst->avg_bw) {
			mutex_unlock(&comm_port_node->bw_lock);
			break;
		}

		comm_port_node->latest_mix_bw = dst->v2_mix_bw;
		comm_port_node->latest_peak_bw = dst->peak_bw;
		comm_port_node->latest_avg_bw = dst->avg_bw;
		port_id = MASK_8(dst->id);

		if (mmqos_state & BWL_ENABLE)
			mmqos_update_comm_bw(get_larb_dev(port_id), //comm_port_node->larb_dev,
					     port_id, comm_port_node->common->freq,
					     ICC_TO_MBPS(comm_port_node->latest_mix_bw),
					     ICC_TO_MBPS(comm_port_node->latest_peak_bw),
					     mmqos->qos_bound,
					     comm_port_node->hrt_type == HRT_MAX_BWL);

		mmqos_dbg(mmqos->dev, LOG_BW, "enable comm ostdl:%d, report_bw_larbs:%d",
			  mmqos_state & COMM_OSTDL_ENABLE, larb_node->is_report_bw_larbs);

		if ((mmqos_state & COMM_OSTDL_ENABLE) &&
		    larb_node->is_report_bw_larbs) {
			if (mmqos_state & VCODEC_BW_BYPASS &&
			    comm_port_node->hrt_type == SRT_VDEC) {
				mmqos_dbg(mmqos->dev, LOG_BW,
					  "ignore SRT_VDEC comm port:%#x OSTDL", dst->id);
			} else if (mmqos_state & VCODEC_BW_BYPASS &&
				 comm_port_node->hrt_type == SRT_VENC) {
				mmqos_dbg(mmqos->dev, LOG_BW,
					  "ignore SRT_VENC comm port:%#x OSTDL", dst->id);
			} else {
				mmqos_dbg(mmqos->dev, LOG_BW,
					  "comm=%d port=%lu larb=%lu avg_bw:%d peak_bw:%d",
					  comm_id, MASK_8(dst->id), LARB_ID(src->id),
					  ICC_TO_MBPS(src->avg_bw),
					  ICC_TO_MBPS(src->peak_bw));

				mmqos_update_comm_ostdl(comm_port_node->larb_dev,
							port_id, mmqos->max_ratio, src);
			}
		}

		comm_id = COMM_PORT_COMM_ID(dst->id);

		if (port_id != VIRT_COMM_PORT_ID)
			chnn_id = comm_port_node->channel - 1;
		record_comm_port_bw(comm_id, port_id, LARB_ID(src->id),
				    src->avg_bw, src->peak_bw,
				    comm_port_node->latest_avg_bw,
				    comm_port_node->latest_peak_bw,
				    mmqos);
		mutex_unlock(&comm_port_node->bw_lock);

		break;
	case MTK_MMQOS_NODE_LARB:
		larb_port_node = (struct larb_port_node *)src->data;
		larb_node = (struct larb_node *)dst->data;

		if (!larb_port_node || !larb_node || !larb_node->larb_dev)
			break;

		/* update channel BW */
		comm_id = MASK_4(larb_port_node->channel >> CHNN_BW_UNIT_SHIFT);
		chnn_id = MASK_4(larb_port_node->channel);

		if (ICC_TO_MBPS(src->v2_mix_bw)) {
			value = SHIFT_ROUND(ICC_TO_MBPS(src->v2_mix_bw), larb_port_node->bw_ratio);

			if (src->peak_bw) {
				if ((larb_port_node->is_write && w_hrt_ostdl == HRT_OSTDL_1_5) ||
				    (!larb_port_node->is_write && r_hrt_ostdl == HRT_OSTDL_1_5))
					value = SHIFT_ROUND(value * 3, 1);

				if ((larb_port_node->is_write && w_hrt_ostdl == HRT_OSTDL_2) ||
				    (!larb_port_node->is_write && r_hrt_ostdl == HRT_OSTDL_2))
					value = value * 2;
			}
		} else {
			src->v2_max_ostd = false;
		}

		if (value > mmqos->max_ratio) {
			if (value > mmqos->max_ratio)
				mmqos_dbg(larb_node->larb_dev, LOG_BW,
					  "larb=%d port=%d avg_bw:%d peak_bw:%d mix_bw:%d ostd=%#x",
					  MTK_M4U_TO_LARB(src->id), MTK_M4U_TO_PORT(src->id),
					  ICC_TO_MBPS(larb_port_node->base->icc_node->avg_bw),
					  ICC_TO_MBPS(larb_port_node->base->icc_node->peak_bw),
					  ICC_TO_MBPS(src->v2_mix_bw), value);

			value = mmqos->max_ratio;
		}

		if (src->v2_max_ostd && is_max_bw_to_max_ostdl_policy(src, mmqos)) {
			mmqos_dbg(mmqos->dev, LOG_BW, "larb=%d port=%d use max_disp_ostdl=%#x",
				  MTK_M4U_TO_LARB(src->id), MTK_M4U_TO_PORT(src->id),
				  mmqos->max_disp_ostdl);

			value = mmqos->max_disp_ostdl;
		}

		if (mmqos_state & OSTD_ENABLE) {
			t_bw = ICC_TO_MBPS(larb_port_node->base->icc_node->avg_bw);
			t_bw1 = ICC_TO_MBPS(larb_port_node->base->icc_node->peak_bw);
			record_larb_port_bw_ostdl(MTK_M4U_TO_LARB(src->id),
						  MTK_M4U_TO_PORT(src->id),
						  t_bw, t_bw1, ICC_TO_MBPS(src->v2_mix_bw),
						  value, mmqos);

			if (mmqos_state & REAL_TIME_PERM_ENABLE) {
				if (larb_port_node->base->icc_node->peak_bw)
					mtk_smi_set_hrt_perm(larb_node->larb_dev,
							     MTK_M4U_TO_PORT(src->id), true);
				else
					mtk_smi_set_hrt_perm(larb_node->larb_dev,
							     MTK_M4U_TO_PORT(src->id), false);
			}
			mtk_smi_larb_bw_set(larb_node->larb_dev, MTK_M4U_TO_PORT(src->id), value);
		}

		mmqos_dbg(larb_node->larb_dev, LOG_BW,
			  "larb=%d port=%d avg_bw:%d peak_bw:%d mix_bw:%d ostd=%#x",
			  MTK_M4U_TO_LARB(src->id), MTK_M4U_TO_PORT(src->id),
			  ICC_TO_MBPS(larb_port_node->base->icc_node->avg_bw),
			  ICC_TO_MBPS(larb_port_node->base->icc_node->peak_bw),
			  ICC_TO_MBPS(src->v2_mix_bw), value);

		record_last_larb_port(src->id, larb_port_node->base->icc_node->avg_bw,
				      larb_port_node->base->icc_node->peak_bw, mmqos);
		break;
	default:
		break;
	}

	return 0;
}

static int mtk_mmqos_aggregate(struct icc_node *node, u32 tag, u32 avg_bw, u32 peak_bw,
			       u32 *agg_avg, u32 *agg_peak)
{
	struct mmqos_base_node *base_node = NULL;
	struct larb_port_node *larb_port_node;
	u32 mix_bw = peak_bw;

	if (!node || !node->data)
		return 0;

	switch (NODE_TYPE(node->id)) {
	case MTK_MMQOS_NODE_LARB_PORT:
		larb_port_node = (struct larb_port_node *)node->data;
		base_node = larb_port_node->base;

		if (peak_bw) {
			if (peak_bw == MTK_MMQOS_MAX_BW) {
				larb_port_node->is_max_ostd = true;
				mix_bw = max_t(u32, avg_bw, 1000);
			} else {
				mix_bw = peak_bw;
			}
		}
		break;
	case MTK_MMQOS_NODE_COMMON_PORT:
		base_node = ((struct common_port_node *)node->data)->base;
		break;
	default:
		return 0;
	}

	if (base_node) {
		if (*agg_avg == 0 && *agg_peak == 0)
			base_node->mix_bw = 0;

		base_node->mix_bw += peak_bw ? mix_bw : avg_bw;
	}

	*agg_avg += avg_bw;

	if (peak_bw == MTK_MMQOS_MAX_BW)
		*agg_peak += 1000; /* for BWL soft mode */
	else
		*agg_peak += peak_bw;

	return 0;
}

static bool mtk_mmqos_path_is_write(struct icc_node *node)
{
	struct larb_port_node *larb_port_node = NULL;
	struct larb_node *larb_node = NULL;

	return false;

	switch (NODE_TYPE(node->id)) {
	case MTK_MMQOS_NODE_LARB_PORT:
		larb_port_node = (struct larb_port_node *)node->data;

		if (larb_port_node == NULL) {
			pr_err("LARB PORT is NULL. Faking is_write=false\n");
			return false;
		}

		if (larb_port_node->is_write) {
			if (larb_node->larb_dev == NULL) {
				pr_err("is_write==true, LARB_DEV for PORT IS NULL!!!!\n");
			} else {
				mmqos_dbg(larb_node->larb_dev, LOG_V2_DBG,
					  "[port] node_id:0x%x is_write:%d",
					  node->id, larb_port_node->is_write);
			}

			return true;
		}

		if (larb_node->larb_dev == NULL) {
			pr_err("is_write==false, LARB_DEV for LARB IS NULL!!!!\n");
			return false;
		} else {
			mmqos_dbg(larb_node->larb_dev, LOG_V2_DBG, "[port] node_id:0x%x is_write:%d",
				  node->id, larb_port_node->is_write);
		}
		break;
	case MTK_MMQOS_NODE_LARB:
		larb_node = (struct larb_node *)node->data;

		if (larb_node == NULL) {
			pr_err("LARB is NULL. Faking is_write=false\n");
			return false;
		}

		if (larb_node->is_write) {
			if (larb_node->larb_dev == NULL) {
				pr_err("is_write==true, LARB_DEV for LARB IS NULL!!!!\n");
			} else {
				mmqos_dbg(larb_node->larb_dev, LOG_V2_DBG,
					  "[larb] node_id:0x%x is_write:%d",
					  node->id, larb_node->is_write);
			}

			return true;
		}

		if (larb_node->larb_dev == NULL) {
			pr_err("is_write==false, LARB_DEV for LARB IS NULL!!!!\n");
			return false;
		} else {
			mmqos_dbg(larb_node->larb_dev, LOG_V2_DBG, "[larb] node_id:0x%x is_write:%d",
				  node->id, larb_node->is_write);
		}
		break;
	default:
		break;
	}

	return false;
}

static struct icc_node *mtk_mmqos_xlate(struct of_phandle_args *spec, void *data)
{
	struct icc_onecell_data *icc_data;
	s32 i;

	if (!spec || !data)
		return ERR_PTR(-EPROBE_DEFER);

	icc_data = (struct icc_onecell_data *)data;

	for (i = 0; i < icc_data->num_nodes; i++)
		if (icc_data->nodes[i]->id == spec->args[0])
			return icc_data->nodes[i];

	pr_notice("%s: invalid index %u\n", __func__, spec->args[0]);

	return ERR_PTR(-EINVAL);
}

static void larb_port_ostdl_dump(struct seq_file *file, u32 larb_id, u32 i, struct mtk_mmqos *mmqos)
{
	u64 rem_nsec;
	u64 ts;

	ts = mmqos->larb_port_bw_rec->time[larb_id][i];
	rem_nsec = do_div(ts, 1000000000);

	if (ts == 0 && mmqos->larb_port_bw_rec->port_id[larb_id][i] == 0 &&
	    mmqos->larb_port_bw_rec->avg_bw[larb_id][i] == 0 &&
	    mmqos->larb_port_bw_rec->peak_bw[larb_id][i] == 0 &&
	    mmqos->larb_port_bw_rec->ostdl[larb_id][i] == 0)
		return;

	seq_printf(file, "[%5llu.%06llu] larb%2d port%2d %8d %8d %8d %8d\n",
		   (u64)ts, rem_nsec / 1000, larb_id,
		   mmqos->larb_port_bw_rec->port_id[larb_id][i],
		   mmqos->larb_port_bw_rec->avg_bw[larb_id][i],
		   mmqos->larb_port_bw_rec->peak_bw[larb_id][i],
		   mmqos->larb_port_bw_rec->mix_bw[larb_id][i],
		   mmqos->larb_port_bw_rec->ostdl[larb_id][i]);
}

static void comm_port_bw_dump(struct seq_file *file, u32 comm_id,
			      u32 port_id, u32 i, struct mtk_mmqos *mmqos)
{
	u64 rem_nsec;
	u64 ts;

	ts = mmqos->comm_port_bw_rec->time[comm_id][port_id][i];
	rem_nsec = do_div(ts, 1000000000);

	if (ts == 0 &&
	    mmqos->comm_port_bw_rec->avg_bw[comm_id][port_id][i] == 0 &&
	    mmqos->comm_port_bw_rec->peak_bw[comm_id][port_id][i] == 0 &&
	    mmqos->comm_port_bw_rec->l_avg_bw[comm_id][port_id][i] == 0 &&
	    mmqos->comm_port_bw_rec->l_peak_bw[comm_id][port_id][i] == 0)
		return;

	seq_printf(file, "[%5llu.%06llu] comm%d port%d larb%2d %8d %8d %8d %8d\n",
		   (u64)ts, rem_nsec / 1000, comm_id, port_id,
		   mmqos->comm_port_bw_rec->larb_id[comm_id][port_id][i],
		   ICC_TO_MBPS(mmqos->comm_port_bw_rec->avg_bw[comm_id][port_id][i]),
		   ICC_TO_MBPS(mmqos->comm_port_bw_rec->peak_bw[comm_id][port_id][i]),
		   ICC_TO_MBPS(mmqos->comm_port_bw_rec->l_avg_bw[comm_id][port_id][i]),
		   ICC_TO_MBPS(mmqos->comm_port_bw_rec->l_peak_bw[comm_id][port_id][i]));
}

static void chn_bw_dump(struct seq_file *file, u32 comm_id, u32 chnn_id,
			u32 i, struct mtk_mmqos *mmqos)
{
	u64 rem_nsec;
	u64 ts;

	ts = mmqos->chn_bw_rec->time[comm_id][chnn_id][i];
	rem_nsec = do_div(ts, 1000000000);

	seq_printf(file, "[%5llu.%06llu] comm%d_%d %8d %8d %8d %8d\n",
		   (u64)ts, rem_nsec / 1000, comm_id, chnn_id,
		   ICC_TO_MBPS(mmqos->chn_bw_rec->srt_r_bw[comm_id][chnn_id][i]),
		   ICC_TO_MBPS(mmqos->chn_bw_rec->srt_w_bw[comm_id][chnn_id][i]),
		   ICC_TO_MBPS(mmqos->chn_bw_rec->hrt_r_bw[comm_id][chnn_id][i]),
		   ICC_TO_MBPS(mmqos->chn_bw_rec->hrt_w_bw[comm_id][chnn_id][i]));
}

static void hrt_bw_dump(struct seq_file *file, u32 i, struct mtk_mmqos *mmqos)
{
	u64 rem_nsec;
	u64 ts;

	ts = mmqos->hrt->hrt_rec.time[i];
	rem_nsec = do_div(ts, 1000000000);

	seq_printf(file, "[%5llu.%06llu]     %8d\n", (u64)ts, rem_nsec / 1000,
		   mmqos->hrt->hrt_rec.avail_hrt[i]);
}

static void larb_port_ostdl_full_dump(struct seq_file *file,
				      u32 larb_id, struct mtk_mmqos *mmqos)
{
	u32 i, start;

	start = mmqos->larb_port_bw_rec->idx[larb_id];

	for (i = start; i < RECORD_NUM; i++)
		larb_port_ostdl_dump(file, larb_id, i, mmqos);

	for (i = 0; i < start; i++)
		larb_port_ostdl_dump(file, larb_id, i, mmqos);
}

static void comm_port_bw_full_dump(struct seq_file *file,
				   u32 comm_id, u32 port_id, struct mtk_mmqos *mmqos)
{
	u32 i, start;

	start = mmqos->comm_port_bw_rec->idx[comm_id][port_id];

	for (i = start; i < RECORD_NUM; i++)
		comm_port_bw_dump(file, comm_id, port_id, i, mmqos);

	for (i = 0; i < start; i++)
		comm_port_bw_dump(file, comm_id, port_id, i, mmqos);
}

static void chn_bw_full_dump(struct seq_file *file, u32 comm_id,
			     u32 chnn_id, struct mtk_mmqos *mmqos)
{
	u32 i, start;

	start = mmqos->chn_bw_rec->idx[comm_id][chnn_id];

	for (i = start; i < RECORD_NUM; i++)
		chn_bw_dump(file, comm_id, chnn_id, i, mmqos);

	for (i = 0; i < start; i++)
		chn_bw_dump(file, comm_id, chnn_id, i, mmqos);
}

static void hrt_bw_full_dump(struct seq_file *file, struct mtk_mmqos *mmqos)
{
	struct hrt_record *rec = &mmqos->hrt->hrt_rec;
	u32 i, start;

	start = rec->idx;

	for (i = start; i < RECORD_NUM; i++)
		hrt_bw_dump(file, i, mmqos);

	for (i = 0; i < start; i++)
		hrt_bw_dump(file, i, mmqos);
}

static int mmqos_last_dump(struct seq_file *file, void *data)
{
	struct mtk_mmqos *mmqos = NULL;
	u64 ts, ns;

	mmqos = mtk_mmqos_get_drv_data();
	if (IS_ERR_OR_NULL(mmqos)) {
		mmqos_dbg(mmqos->dev, LOG_BW, "not ready");
		return 0;
	}

	seq_puts(file, "last larb port:\n");

	ts = mmqos->last_rec->larb_port_update_time;
	ns = do_div(ts, 1000000000);

	seq_printf(file, "[%5llu.%06llu] %#x %u %u\n", (u64)ts, ns / 1000,
		   mmqos->last_rec->larb_port_node_id, mmqos->last_rec->larb_port_avg_bw,
		   mmqos->last_rec->larb_port_peak_bw);

	seq_puts(file, "last larb:\n");

	ts = mmqos->last_rec->larb_update_time;
	ns = do_div(ts, 1000000000);

	seq_printf(file, "[%5llu.%06llu] %#x %u %u", (u64)ts, ns / 1000,
		   mmqos->last_rec->larb_node_id, mmqos->last_rec->larb_avg_bw,
		   mmqos->last_rec->larb_peak_bw);

	return 0;
}

static int mmqos_last_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, mmqos_last_dump, inode->i_private);
}

static const struct proc_ops mmqos_last_debug_fops = {
	.proc_open = mmqos_last_debug_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

static void mmpc_subsys_hw_mode_full_dump(struct seq_file *file, int sid, struct mtk_mmqos *mmqos)
{
	u32 mmpc_hw_bw_hrt = mmqos->mmpc_hw_bw_hrt;
	u32 mmpc_hw_bw = mmqos->mmpc_hw_bw;

	seq_printf(file, "\nsid:%d, Total HRT: %u, SRT: %u\n",
		   sid, read_register(mmpc_hw_bw_hrt + SUBSYS_HW_BW_HRT(sid), mmqos),
		   read_register(mmpc_hw_bw_hrt + SUBSYS_HW_BW_SRT(sid), mmqos));

	for (int i = 0; i < MAX_REG_VALUE_NUM; i++) {
		seq_printf(file, "i:%d, offset:%#x, value:%#x\n",
			   i, mmpc_hw_bw + SUBSYS_HW_BW_OFFSET(sid, i),
			   read_register(mmpc_hw_bw + SUBSYS_HW_BW_OFFSET(sid, i), mmqos));
	}
}

static void mmpc_total_bw_full_dump(struct seq_file *file, struct mtk_mmqos *mmqos)
{
	u32 mmpc_total_mmqos_bw = mmqos->mmpc_total_mmqos_bw;
	u32 mmpc_total_pmqos_bw = mmqos->mmpc_total_pmqos_bw;

	seq_printf(file, "\nTotal HRT: %u, SRT: %u\n",
		   read_register(mmpc_total_pmqos_bw, mmqos),
		   read_register(mmpc_total_pmqos_bw + TOTAL_SRT_BW, mmqos));

	for (int i = 0; i < MAX_BW_VALUE_NUM; i++) {
		seq_printf(file, "i:%d, offset:%#x, value:%u\n",
			   i, mmpc_total_mmqos_bw + TOTAL_BW(i),
			   read_register(mmpc_total_mmqos_bw + TOTAL_BW(i), mmqos));
	}
}

static void mmpc_dvfsrc_full_dump(struct seq_file *file, struct mtk_mmqos *mmqos)
{
	for (int sid = 0; sid < MAX_SUBSYS_NUM; sid++)
		mmpc_subsys_hw_mode_full_dump(file, sid, mmqos);

	mmpc_total_bw_full_dump(file, mmqos);
}

static int mmqos_bw_dump(struct seq_file *file, void *data)
{
	u32 comm_id = 0, chnn_id = 0, port_id = 0, larb_id = 0;
	struct mtk_mmqos *mmqos = NULL;
	u32 mmqos_state;

	mmqos = mtk_mmqos_get_drv_data();
	if (IS_ERR_OR_NULL(mmqos)) {
		mmqos_dbg(mmqos->dev, LOG_BW, "not ready");
		return 0;
	}

	mmqos_state = mmqos->mmqos_state;
	seq_printf(file, "MMQoS HRT BW Dump: %8s\n",
		   "avail");

	hrt_bw_full_dump(file, mmqos);

	seq_printf(file, "MMQoS Channel BW Dump: %8s %8s %8s %8s\n", "s_r", "s_w", "h_r", "h_w");

	for (comm_id = 0; comm_id < MAX_RECORD_COMM_NUM; comm_id++) {
		for (chnn_id = 0; chnn_id < MMQOS_COMM_CHANNEL_NUM; chnn_id++)
			chn_bw_full_dump(file, comm_id, chnn_id, mmqos);
	}

	seq_printf(file, "MMQoS Common Port BW Dump:        %8s %8s %8s %8s\n",
		   "avg", "peak", "l_avg", "l_peak");

	for (comm_id = 0; comm_id < MAX_RECORD_COMM_NUM; comm_id++) {
		for (port_id = 0; port_id < MAX_RECORD_PORT_NUM; port_id++)
			comm_port_bw_full_dump(file, comm_id, port_id, mmqos);
	}

	seq_printf(file, "MMQoS OSTDL Dump r:%2d w:%2d   %8s %8s %8s %8s\n",
		   mmqos->r_hrt_ostdl, mmqos->w_hrt_ostdl, "avg_bw", "peak_bw", "mix_bw", "ostdl");

	for (larb_id = 0; larb_id < MAX_RECORD_LARB_NUM; larb_id++)
		larb_port_ostdl_full_dump(file, larb_id, mmqos);

	if (mmqos_state & MMPC_ENABLE)
		mmpc_dvfsrc_full_dump(file, mmqos);

	return 0;
}

static int mmqos_debug_opp_open(struct inode *inode, struct file *file)
{
	return single_open(file, mmqos_bw_dump, inode->i_private);
}

static const struct proc_ops mmqos_debug_fops = {
	.proc_open = mmqos_debug_opp_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};

int mtk_mmqos_probe(struct platform_device *pdev)
{
	struct device *larb_imu_dev[MTK_LARB_NR_MAX];
	struct proc_dir_entry *dir, *proc, *last_proc;
	const struct mtk_mmqos_desc *mmqos_desc;
	struct common_port_node *comm_port_node;
	struct larb_port_node *larb_port_node;
	const struct mtk_node_desc *node_desc;
	int i, j, id, num_larbs = 0, ret;
#if IS_ENABLED(CONFIG_MTK_DVFSRC)
	struct platform_device *comm_pdev;
#endif
	struct platform_device *larb_pdev;
	struct mmqos_base_node *base_node;
	struct of_phandle_iterator it;
	struct icc_onecell_data *data;
	struct common_node *comm_node;
	struct icc_node *node, *temp;
	struct larb_node *larb_node;
	struct device *dev_root;
	struct device *larb_dev;
	struct mtk_mmqos *mmqos;
	struct device_node *np;

	mmqos = devm_kzalloc(&pdev->dev, sizeof(*mmqos), GFP_KERNEL);
	if (!mmqos) {
		dev_err(&pdev->dev, "alloc mmqos(%zu) fail.\n", sizeof(*mmqos));
		return -ENOMEM;
	}

	mmqos->pdev = pdev;
	mmqos->dev = &pdev->dev;

	mmqos->last_rec = kzalloc(sizeof(struct last_record), GFP_KERNEL);
	if (!mmqos->last_rec)
		return -ENOMEM;

	mmqos->chn_bw_rec = devm_kzalloc(&pdev->dev, sizeof(struct chn_bw_record), GFP_KERNEL);
	if (!mmqos->chn_bw_rec)
		return -ENOMEM;

	mmqos->comm_port_bw_rec = devm_kzalloc(&pdev->dev,
					       sizeof(struct comm_port_bw_record), GFP_KERNEL);
	if (!mmqos->comm_port_bw_rec)
		return -ENOMEM;

	mmqos->larb_port_bw_rec = devm_kzalloc(&pdev->dev,
					       sizeof(struct larb_port_bw_record), GFP_KERNEL);
	if (!mmqos->larb_port_bw_rec)
		return -ENOMEM;

	mmqos->disp_bw = devm_kzalloc(&pdev->dev, sizeof(struct disp_bw), GFP_KERNEL);
	if (!mmqos->disp_bw)
		return -ENOMEM;

	mmqos->chn_bw = devm_kzalloc(&pdev->dev, sizeof(struct chn_bw), GFP_KERNEL);
	if (!mmqos->chn_bw)
		return -ENOMEM;

	of_for_each_phandle(&it, ret, pdev->dev.of_node, "mediatek,larbs-supply", NULL, 0) {
		np = of_node_get(it.node);
		if (!of_device_is_available(np))
			continue;

		larb_pdev = of_find_device_by_node(np);

		dev_root = bus_get_dev_root(pdev->dev.bus);
		if (!larb_pdev && dev_root) {
			larb_pdev = of_platform_device_create(np, NULL, dev_root);
			put_device(dev_root);
			if (!larb_pdev || !larb_pdev->dev.driver) {
				of_node_put(np);
				return -EPROBE_DEFER;
			}
		}

		if (of_property_read_u32(np, "mediatek,larb-id", &id))
			id = num_larbs;

		larb_imu_dev[id] = &larb_pdev->dev;
		num_larbs += 1;
	}

	INIT_LIST_HEAD(&mmqos->comm_list);
	INIT_LIST_HEAD(&mmqos->prov.nodes);
	mmqos->prov.set = mtk_mmqos_set;
	mmqos->prov.aggregate = mtk_mmqos_aggregate;
	mmqos->prov.path_is_write = mtk_mmqos_path_is_write;
	mmqos->prov.xlate = mtk_mmqos_xlate;
	mmqos->prov.dev = &pdev->dev;

	ret = mtk_icc_provider_add(&mmqos->prov);
	if (ret) {
		mmqos_err(&pdev->dev, "mtk_icc_provider_add failed:%d\n", ret);
		ret = -EINVAL;
		goto err;
	}

	mmqos_desc = (struct mtk_mmqos_desc *)of_device_get_match_data(&pdev->dev);
	if (!mmqos_desc) {
		mmqos_err(&pdev->dev, "match mmqos_desc fail.\n");
		ret = -EINVAL;
		goto err;
	}

	freq_mode = mmqos_desc->freq_mode ? mmqos_desc->freq_mode : freq_mode;
	mmqos_dbg(mmqos->dev, LOG_BW, "freq_mode:%d", freq_mode);

	data = devm_kzalloc(&pdev->dev, sizeof(*data) + mmqos_desc->num_nodes * sizeof(node),
			    GFP_KERNEL);
	if (!data) {
		mmqos_err(&pdev->dev, "alloc data(%zu) fail.\n",
			  sizeof(*data) + mmqos_desc->num_nodes * sizeof(node));
		ret = -ENOMEM;
		goto err;
	}

	for (i = 0; i < mmqos_desc->num_nodes; i++) {
		node_desc = &mmqos_desc->nodes[i];
		node = mtk_icc_node_create(node_desc->id);
		if (IS_ERR(node)) {
			mmqos_err(&pdev->dev, "mtk_icc_node_create failed\n");
			ret = PTR_ERR(node);
			goto err;
		}

		mtk_icc_node_add(node, &mmqos->prov);
		if (node_desc->link != MMQOS_NO_LINK) {
			ret = mtk_icc_link_create(node, node_desc->link);
			if (ret) {
				mmqos_err(&pdev->dev, "mtk_icc_link_create failed:%d\n", ret);
				goto err;
			}
		}

		node->name = node_desc->name;
		base_node = devm_kzalloc(&pdev->dev, sizeof(*base_node), GFP_KERNEL);
		if (!base_node) {
			mmqos_err(&pdev->dev, "alloc base_node(%zu) fail.\n", sizeof(*base_node));
			ret = -ENOMEM;
			goto err;
		}

		base_node->icc_node = node;
		switch (NODE_TYPE(node->id)) {
		case MTK_MMQOS_NODE_COMMON:
			comm_node = devm_kzalloc(&pdev->dev, sizeof(*comm_node), GFP_KERNEL);
			if (!comm_node) {
				mmqos_err(&pdev->dev, "alloc comm_node(%zu) fail.\n",
					  sizeof(*comm_node));
				ret = -ENOMEM;
				goto err;
			}

			comm_node->clk = devm_clk_get(&pdev->dev,
						      mmqos_desc->comm_muxes[MASK_8(node->id)]);
			if (IS_ERR(comm_node->clk)) {
				mmqos_err(&pdev->dev, "get clk fail:%s\n",
					  mmqos_desc->comm_muxes[MASK_8(node->id)]);
				ret = -EINVAL;
				goto err;
			}

			comm_node->freq = clk_get_rate(comm_node->clk) / CLK_MHZ;
			INIT_LIST_HEAD(&comm_node->list);
			list_add_tail(&comm_node->list, &mmqos->comm_list);
			INIT_LIST_HEAD(&comm_node->comm_port_list);
#if IS_ENABLED(CONFIG_MTK_DVFSRC)
			comm_node->icc_path =
				of_icc_get(&pdev->dev,
					   mmqos_desc->comm_icc_path_names[MASK_8(node->id)]);
			if (IS_ERR_OR_NULL(comm_node->icc_path)) {
				mmqos_err(&pdev->dev, "get icc_path fail:%s\n",
					  mmqos_desc->comm_icc_path_names[MASK_8(node->id)]);
				ret = -EINVAL;
				goto err;
			}

			comm_node->icc_hrt_path =
				of_icc_get(&pdev->dev,
					   mmqos_desc->comm_icc_hrt_path_names[MASK_8(node->id)]);
			if (IS_ERR_OR_NULL(comm_node->icc_hrt_path)) {
				mmqos_err(&pdev->dev, "get icc_hrt_path fail:%s\n",
					  mmqos_desc->comm_icc_hrt_path_names[MASK_8(node->id)]);
				ret = -EINVAL;
				goto err;
			}

			np = of_parse_phandle(pdev->dev.of_node, "mediatek,commons-supply",
					      MASK_8(node->id));
			if (!of_device_is_available(np)) {
				mmqos_err(&pdev->dev, "get common(%lu) dev fail\n",
					  MASK_8(node->id));
				break;
			}

			comm_pdev = of_find_device_by_node(np);
			if (comm_pdev)
				comm_node->comm_dev = &comm_pdev->dev;
			else
				mmqos_err(&pdev->dev, "comm(%lu) pdev is null\n", MASK_8(node->id));

			comm_node->comm_reg =
				devm_regulator_get_optional(comm_node->comm_dev,
							    "mmdvfs-dvfsrc-vcore");
			if (freq_mode == BY_REGULATOR)
				if (IS_ERR_OR_NULL(comm_node->comm_reg))
					dev_notice(&pdev->dev, "get common(%lu) reg fail\n",
						   MASK_8(node->id));

			dev_pm_opp_of_add_table(comm_node->comm_dev);
			comm_node->base = base_node;
			node->data = (void *)comm_node;
#endif
			break;
		case MTK_MMQOS_NODE_COMMON_PORT:
			comm_port_node = devm_kzalloc(&pdev->dev, sizeof(*comm_port_node),
						      GFP_KERNEL);
			if (!comm_port_node) {
				mmqos_err(&pdev->dev, "alloc comm_port_node(%zu) fail.\n",
					  sizeof(*comm_port_node));
				ret = -ENOMEM;
				goto err;
			}

			comm_port_node->channel =
				mmqos_desc->comm_port_channels[
				MASK_8((node->id >> 8))][MASK_8(node->id)];
			comm_port_node->channel_v2 = node_desc->channel;
			comm_port_node->hrt_type =
				mmqos_desc->comm_port_hrt_types[
				MASK_8((node->id >> 8))][MASK_8(node->id)];
			mutex_init(&comm_port_node->bw_lock);
			comm_port_node->common = node->links[0]->data;
			INIT_LIST_HEAD(&comm_port_node->list);
			list_add_tail(&comm_port_node->list,
				      &comm_port_node->common->comm_port_list);
			comm_port_node->base = base_node;
			node->data = (void *)comm_port_node;
			break;
		case MTK_MMQOS_NODE_LARB:
			larb_node = devm_kzalloc(&pdev->dev, sizeof(*larb_node), GFP_KERNEL);
			if (!larb_node) {
				mmqos_err(&pdev->dev, "alloc larb_node(%zu) fail.\n",
					  sizeof(*larb_node));
				ret = -ENOMEM;
				goto err;
			}

			comm_port_node = node->links[0]->data;
			larb_dev = larb_imu_dev[node->id & (MTK_LARB_NR_MAX - 1)];
			if (larb_dev) {
				comm_port_node->larb_dev = larb_dev;
				larb_node->larb_dev = larb_dev;
			}

			larb_node->channel = node_desc->channel;
			larb_node->channel_v2 = comm_port_node->channel_v2;
			larb_node->is_write = node_desc->is_write;
			larb_node->bw_ratio = node_desc->bw_ratio;

			/* init disable dualpipe */
			mmqos->dual_pipe_enable = false;

			for (j = 0; j < MMQOS_MAX_REPORT_LARB_NUM; j++) {
				if (node->id == mmqos_desc->report_bw_larbs[j]) {
					larb_node->is_report_bw_larbs = true;
					id = mmqos_desc->report_bw_real_larbs[j];
					larb_dev = larb_imu_dev[id & (MTK_LARB_NR_MAX - 1)];
					larb_node->larb_dev = larb_dev;
				}
			}

			larb_node->base = base_node;
			node->data = (void *)larb_node;
			break;
		case MTK_MMQOS_NODE_LARB_PORT:
			larb_port_node = devm_kzalloc(&pdev->dev, sizeof(*larb_port_node),
						      GFP_KERNEL);
			if (!larb_port_node) {
				mmqos_err(&pdev->dev, "alloc larb_port_node(%zu) fail.\n",
					  sizeof(*larb_port_node));
				ret = -ENOMEM;
				goto err;
			}

			larb_port_node->channel = node_desc->channel;
			larb_port_node->is_write = node_desc->is_write;
			larb_port_node->bw_ratio = node_desc->bw_ratio;
			larb_port_node->base = base_node;
			node->data = (void *)larb_port_node;
			break;
		default:
			mmqos_err(&pdev->dev, "invalid node id:%#x\n", node->id);
			ret = -EINVAL;
			goto err;
		}

		data->nodes[i] = node;
	}

	data->num_nodes = mmqos_desc->num_nodes;
	mmqos->prov.data = data;
	mmqos->max_ratio = mmqos_desc->max_ratio;

	if (mmqos_desc->max_disp_ostdl == 0)
		mmqos->max_disp_ostdl = mmqos_desc->max_ratio;
	else
		mmqos->max_disp_ostdl = mmqos_desc->max_disp_ostdl;

	mmqos->mmqos_state = mmqos_desc->mmqos_state;
	mmqos_dbg(mmqos->dev, LOG_BW, "probe state: %#x", mmqos->mmqos_state);

	mmqos->log_level = mmqos_desc->mmqos_log_level;
	log_level = mmqos->log_level;
	mmqos_dbg(mmqos->dev, LOG_BW, "log level: %#x", mmqos->log_level);

	mmqos->r_hrt_ostdl = HRT_OSTDL_1_5;
	mmqos->w_hrt_ostdl = HRT_OSTDL_1_5;
	of_property_read_u32(pdev->dev.of_node, "r-hrt-ostdl", &mmqos->r_hrt_ostdl);
	of_property_read_u32(pdev->dev.of_node, "w-hrt-ostdl", &mmqos->w_hrt_ostdl);

	mmqos_dbg(mmqos->dev, LOG_BW, "hrt ostdl policy, read:%d, write:%d",
		  mmqos->r_hrt_ostdl, mmqos->w_hrt_ostdl);

	for (i = 0 ; i < MMQOS_MAX_DISP_VIRT_LARB_NUM ; i++)
		mmqos->disp_virt_larbs[i] = mmqos_desc->disp_virt_larbs[i];

	mmqos->hrt = devm_kzalloc(&pdev->dev, sizeof(struct mmqos_hrt), GFP_KERNEL);
	if (!mmqos->hrt) {
		mmqos_err(&pdev->dev, "alloc mmqos_hrt(%zu) fail.\n", sizeof(struct mmqos_hrt));
		ret = -ENOMEM;
		goto err;
	}

	memcpy(mmqos->hrt, &mmqos_desc->hrt, sizeof(mmqos_desc->hrt));
	mmqos_dbg(mmqos->dev, LOG_BW, "hrt_total_bw: %d", mmqos->hrt->hrt_total_bw);

	/* init mutex_lock for hrt */
	mutex_init(&mmqos->hrt->hrt_rec.lock);
	mutex_init(&mmqos->bw_lock);

	mmqos->nb.notifier_call = update_mm_clk;
	platform_set_drvdata(pdev, mmqos);

	/* create proc file */
	dir = proc_mkdir("mmqos", NULL);
	if (IS_ERR_OR_NULL(dir))
		mmqos_dbg(mmqos->dev, LOG_BW, "proc_mkdir failed:%ld", PTR_ERR(dir));

	proc = proc_create("mmqos_bw", 0444, dir, &mmqos_debug_fops);
	if (IS_ERR_OR_NULL(proc))
		mmqos_dbg(mmqos->dev, LOG_BW, "proc_create failed:%ld", PTR_ERR(proc));
	else
		mmqos->proc = proc;

	last_proc = proc_create("last_mmqos", 0444, dir, &mmqos_last_debug_fops);
	if (IS_ERR_OR_NULL(last_proc))
		mmqos_dbg(mmqos->dev, LOG_BW, "last proc_create failed:%ld", PTR_ERR(last_proc));
	else
		mmqos->last_proc = last_proc;

	dev_info(&pdev->dev, "mmqos probe done.\n");

	return 0;

err:
	mmqos_err(&pdev->dev, "mmqos probe fail:%d.\n", ret);
	list_for_each_entry_safe(node, temp, &mmqos->prov.nodes, node_list) {
		mtk_icc_node_del(node);
		mtk_icc_node_destroy(node->id);
	}
	mtk_icc_provider_del(&mmqos->prov);
	return ret;
}
EXPORT_SYMBOL_GPL(mtk_mmqos_probe);

int mtk_mmqos_v2_probe(struct platform_device *pdev)
{
	const struct mtk_mmqos_desc *mmqos_desc = NULL;
	struct mtk_mmqos *mmqos = NULL;
#if IS_ENABLED(CONFIG_MTK_MMQOS_VCP)
	struct task_struct *kthr_vcp;
#endif
	struct resource *res;
	int probe_ret;
	u32 mmqos_state;

	probe_ret = mtk_mmqos_probe(pdev);
	mmqos = mtk_mmqos_get_drv_data();
	if (IS_ERR_OR_NULL(mmqos)) {
		mmqos_err(&pdev->dev, "not ready");
		return -EPROBE_DEFER;
	}

	mmqos_state = mmqos->mmqos_state;

	if(freq_mode != BY_VMMRC) {
		dev_info(mmqos->dev, "[mmqos dbg]freq_mode no vmmrc.\n");
		mmqos->vmmrc_base = NULL;
		return -probe_ret;
	}

	mmqos_desc = (struct mtk_mmqos_desc *)of_device_get_match_data(&pdev->dev);
	if (!mmqos_desc) {
		mmqos_err(mmqos->dev, "match mmqos_desc fail.\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	    if (!res)
		return -EINVAL;

	mmqos->vmmrc_base = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (IS_ERR(mmqos->vmmrc_base)) {
		dev_info(mmqos->dev, "[mmqos dbg]vmmrc_base ioremap not success.\n");
		mmqos->vmmrc_base = NULL;
	} else {
		mmqos->apmcu_mask_offset = mmqos_desc->vmmrc_setting.vmmrc_mask;
		mmqos->apmcu_on_bw_offset = mmqos_desc->vmmrc_setting.vmmrc_on_table;

		if (mmqos_state & VMMRC_ENABLE) {
			mmqos->apmcu_mask_bit = mmqos_desc->vmmrc_setting.vmmrc_mask_bit;
			mmqos->apmcu_off_bw_offset = mmqos_desc->vmmrc_setting.vmmrc_off_table;

			res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
			if (!res)
				return -ENOMEM;

			mmqos->mminfra_base = devm_platform_ioremap_resource(pdev, 0);
			if (IS_ERR(mmqos->mminfra_base)) {
				mmqos_err(&pdev->dev, "mminfra_base ioremap fail.\n");
				return PTR_ERR(mmqos->mminfra_base);
			}

			mmqos_dbg(mmqos->dev, LOG_BW,
				  "vmmrc m=%#x, b=%#x, on=%#x, off=%#x",
				  mmqos->apmcu_mask_offset,  mmqos->apmcu_mask_bit,
				  mmqos->apmcu_on_bw_offset, mmqos->apmcu_off_bw_offset);

		} else if (mmqos_state & MMPC_ENABLE) {
			mmqos->mmpc_sw_en_all_on = mmqos_desc->mmpc_setting.mmpc_sw_en_all_on;
			mmqos->mmpc_hw_bw = mmqos_desc->mmpc_setting.mmpc_hw_bw;
			mmqos->mmpc_hw_bw_hrt = mmqos_desc->mmpc_setting.mmpc_hw_bw_hrt;
			mmqos->mmpc_total_mmqos_bw = mmqos_desc->mmpc_setting.mmpc_total_mmqos_bw;
			mmqos->mmpc_total_pmqos_bw = mmqos_desc->mmpc_setting.mmpc_total_pmqos_bw;
			mmqos->vmmrc_level_hex = mmqos_desc->vmmrc_setting.vmmrc_level_hex;

			res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
			if (!res)
				return -ENOMEM;

			mmqos->mminfra_base = devm_platform_ioremap_resource(pdev, 0);
			if (IS_ERR(mmqos->mminfra_base)) {
				mmqos_err(&pdev->dev, "mminfra_base ioremap fail.\n");
				return PTR_ERR(mmqos->mminfra_base);
			}

			mmqos_dbg(mmqos->dev, LOG_BW,
				  "vmmrc mask=%#x, en=%#x, tbl=%#x",
				  mmqos->apmcu_mask_offset,
				  mmqos->mmpc_sw_en_all_on,
				  mmqos->apmcu_on_bw_offset);
			mmqos_dbg(mmqos->dev, LOG_BW,
				  "mmpc bw:%#x, hrt:%#x, mqosbw:%#x, pqosbw:%#x, level:%#x",
				  mmqos->mmpc_hw_bw, mmqos->mmpc_hw_bw_hrt,
				  mmqos->mmpc_total_mmqos_bw, mmqos->mmpc_total_pmqos_bw,
				  mmqos->vmmrc_level_hex);
		} else {
			dev_info(mmqos->dev, "no VMMRC_ENABLE & MMPC_ENABLE");
		}
	}
#if IS_ENABLED(CONFIG_MTK_MMQOS_VCP)
	if (mmqos_state & VCP_ENABLE)
		kthr_vcp = kthread_run(mmqos_vcp_init_thread, pdev, "mmqos-vcp");
	else
		mmqos_err(mmqos->dev, "VCP not enable");
#endif

	return probe_ret;
}
EXPORT_SYMBOL_GPL(mtk_mmqos_v2_probe);

int mtk_mmqos_remove(struct platform_device *pdev)
{
	struct mtk_mmqos *mmqos = platform_get_drvdata(pdev);
	struct icc_node *node, *temp;

	list_for_each_entry_safe(node, temp, &mmqos->prov.nodes, node_list) {
		mtk_icc_node_del(node);
		mtk_icc_node_destroy(node->id);
	}

	mtk_icc_provider_del(&mmqos->prov);


	return 0;
}
EXPORT_SYMBOL_GPL(mtk_mmqos_remove);


module_param(log_level, uint, 0644);
MODULE_PARM_DESC(log_level, "mmqos log level");

static int mmqos_set_state(const char *val, const struct kernel_param *kp)
{
	struct mtk_mmqos *mmqos = NULL;
	u32 mmqos_state;
	u32 state = 0;
	int ret;

	mmqos = mtk_mmqos_get_drv_data();
	if (IS_ERR_OR_NULL(mmqos)) {
		mmqos_dbg(mmqos->dev, LOG_BW, "not ready");
		return -EINVAL;
	}

	mmqos_state = mmqos->mmqos_state;
	ret = kstrtou32(val, 0, &state);
	if (ret) {
		mmqos_err(mmqos->dev, "failed:%d state:%#x", ret, state);
		return ret;
	}
	mmqos_dbg(mmqos->dev, LOG_BW, "sync mmqos_state: %d -> %d", mmqos_state, state);
	mmqos->mmqos_state = state;
#if IS_ENABLED(CONFIG_MTK_MMQOS_VCP)
	mmqos_set_state_to_vcp();
#endif

	return ret;
}

static const struct kernel_param_ops mmqos_state_ops = {
	.set = mmqos_set_state,
	.get = param_get_uint,
};
module_param_cb(mmqos_state, &mmqos_state_ops, &mmqos_state, 0644);
MODULE_PARM_DESC(mmqos_state, "mmqos_state");

module_param(freq_mode, uint, 0644);
MODULE_PARM_DESC(freq_mode, "mminfra change frequency mode");

u32 larb_port_ostdl;

struct device *get_larb_dev(int larb_id)
{
	struct mtk_mmqos *mmqos = mtk_mmqos_get_drv_data();
	struct icc_onecell_data *icc_data;
	struct larb_node *larb_node;
	struct icc_node *node;
	int i;

	if (larb_id < 0)
		return NULL;

	if (IS_ERR_OR_NULL(mmqos)) {
		mmqos_dbg(mmqos->dev, LOG_BW, "not ready");
		return NULL;
	}

	icc_data = (struct icc_onecell_data *)mmqos->prov.data;

	for (i = 0; i < icc_data->num_nodes; i++)
		if (icc_data->nodes[i]->id == SLAVE_LARB(larb_id)) {
			node = icc_data->nodes[i];
			larb_node = (struct larb_node *)node->data;
			mmqos_dbg(mmqos->dev, LOG_BW, "found larb id:%d", larb_id);
			return larb_node->larb_dev;
		}

	mmqos_dbg(mmqos->dev, LOG_BW, "cannot find larb id:%d", larb_id);

	return NULL;
}

int mmqos_set_larb_port_ostdl(const char *val, const struct kernel_param *kp)
{
	struct mtk_mmqos *mmqos = NULL;
	int larb_id, port_id, ostdl;
	struct device *larb_dev;
	int result;

	mmqos = mtk_mmqos_get_drv_data();
	if (IS_ERR_OR_NULL(mmqos)) {
		mmqos_err(mmqos->dev, "not ready");
		return -EINVAL;
	}

	result = sscanf(val, "%d %d %d", &larb_id, &port_id, &ostdl);
	if (ostdl <= 0) {
		mmqos_err(mmqos->dev, "ostdl should not <= 0");
		return -EINVAL;
	}

	mmqos_dbg(mmqos->dev, LOG_BW, "larb:%d port:%d ostdl:%d", larb_id, port_id, ostdl);

	larb_dev = get_larb_dev(larb_id);
	if (!larb_dev) {
		mmqos_err(mmqos->dev, "wrong larb_id");
		return -EINVAL;
	}

	mtk_smi_larb_bw_set(larb_dev, port_id, ostdl);

	return 0;
}

static const struct kernel_param_ops larb_port_ostdl_ops = {
	.set = mmqos_set_larb_port_ostdl,
	.get = param_get_uint,
};
module_param_cb(larb_port_ostdl, &larb_port_ostdl_ops, &larb_port_ostdl, 0644);
MODULE_PARM_DESC(larb_port_ostdl, "force set ostdl to larb port");

MODULE_IMPORT_NS(MTK_SMI);
MODULE_LICENSE("GPL");
#endif /* MMQOS_MTK_C */
