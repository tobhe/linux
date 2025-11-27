// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2024 MediaTek Inc.
 */

#include <drm/display/drm_dp.h>
#include <drm/display/drm_dp_aux_bus.h>
#include <drm/display/drm_dp_helper.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/display/drm_dsc.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_modes.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

#include <linux/arm-smccc.h>
#include <linux/atomic.h>
#include <linux/bitfield.h>
#include <linux/bpf.h>
#include <linux/capability.h>
#include <linux/clk.h>
#include <linux/compat.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/extcon.h>
#include <linux/if_vlan.h>
#include <linux/io.h>
#include <linux/kallsyms.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/linkage.h>
#include <linux/mfd/syscon.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_graph.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/pm_domain.h>
#include <linux/regmap.h>
#include <linux/refcount.h>
#include <linux/sched.h>
#include <linux/set_memory.h>
#include <linux/skbuff.h>
#include <linux/sockptr.h>
#include <linux/soc/mediatek/mtk-mmsys.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>

#include <sound/hdmi-codec.h>
#include <uapi/drm/mediatek_drm.h>
#include <video/videomode.h>

#include "mtk_dp_v2.h"
#include "mtk_dp_reg_v2.h"
#include "mtk_dp_hdcp1x.h"
#include "mtk_dp_hdcp2.h"
#include "mtk_dp_mst.h"

#define AUX_CMD_I2C_R				0x05
#define AUX_CMD_I2C_R_MOT0			0x01
#define AUX_CMD_NATIVE_R			0x09
#define AUX_CMD_NATIVE_W			0x08
#define AUX_NO_REPLY_WAIT_TIME		3200
#define AUX_WRITE_READ_WAIT_TIME	20 /* us */
#define AUX_WAIT_REPLY_LP_CNT_NUM	20000
#define ENABLE_DP_EF_MODE			0x1
#define DP_AUX_SET_ENAHNCED_FRAME	0x80
#define DP_CHECK_SINK_CAP_TIMEOUT_COUNT	0x3
#define DP_PHY_REG_COUNT			6
#define DP_TBC_BUF_READ_START_ADR_THRD	0x08
#define ENABLE_DP_FIX_TPS2			0x0
#define ENABLE_DP_SSC_FORCEON		0
#define ENCODER_1_IRQ_MSK			BIT(3)
#define ENCODER_IRQ_MSK				BIT(0)
#define IEC_CH_STATUS_LEN			5
#define MTK_DP_SIP_CONTROL_AARCH32	MTK_SIP_SMC_CMD(0x523)
#define TRANS_IRQ_MSK				BIT(1)
#define DP_CTS_RETRAIN_TIMES_14		12
#define DP_CTS_RETRAIN_TIMES_DEFAULT	6
#define DP_LT_RETRY_LIMIT			20
#define DPTX_TRAIN_MAX_ITERATION		5
#define HPD_DEBOUNCE				100
#define ACK_ESI_RETRY_TIMES			3
#define MAX_MAC_REG_RANG			0x8000
#define MAX_PHYD_REG_RANG			0x1500
#define MST_HPD_EVENT_HANDLE_TIMES		200
#define MTK_DP_SIP_ATF_VIDEO_UNMUTE		BIT(5)
#define AUDIO_SRAM_RESET_TIMEOUT_MS		1000
#define DP_CHECK_PHY_POLLING_DELAY_TIME		0
#define DP_CHECK_PHY_POLLING_TIMEOUT		10

#define GET_DP_ENCODER_ID(dev, dp_dev) \
	((dev) == (dp_dev) ? DP_ENCODER_ID_0 : DP_ENCODER_ID_1)

enum aux_reply_cmd {
	AUX_REPLY_ACK = 0x00,
	AUX_DPCD_NACK = BIT(0),
	AUX_DPCD_DEFER = BIT(1),
	AUX_EDID_NACK = BIT(2),
	AUX_EDID_DEFER = BIT(3),
	AUX_HW_FAILED = BIT(4),
	AUX_INVALID_CMD = BIT(5),
};

enum dp_disp_state {
	DP_DISP_STATE_NONE,
	DP_DISP_STATE_RESUME,
	DP_DISP_STATE_SUSPEND,
	DP_DISP_STATE_SUSPENDING,
};

enum dp_fec_error_count_type {
	FEC_ERROR_COUNT_DISABLE,
	FEC_UNCORRECTED_BLOCK_ERROR_COUNT,
	FEC_CORRECTED_BLOCK_ERROR_COUNT,
	FEC_BIT_ERROR_COUNT,
	FEC_PARITY_BLOCK_ERROR_COUNT,
	FEC_PARITY_BIT_ERROR_COUNT,
};

enum dp_notify_state {
	DP_NOTIFY_STATE_NO_DEVICE,
	DP_NOTIFY_STATE_ACTIVE,
};

enum dp_pg_location {
	DP_PG_LOCATION_NONE,
	DP_PG_LOCATION_ALL,
	DP_PG_LOCATION_TOP,
	DP_PG_LOCATION_BOTTOM,
	DP_PG_LOCATION_LEFT_OF_TOP,
	DP_PG_LOCATION_LEFT_OF_BOTTOM,
	DP_PG_LOCATION_LEFT,
	DP_PG_LOCATION_RIGHT,
	DP_PG_LOCATION_LEFT_OF_LEFT,
	DP_PG_LOCATION_RIGHT_OF_LEFT,
	DP_PG_LOCATION_LEFT_OF_RIGHT,
	DP_PG_LOCATION_RIGHT_OF_RIGHT,
	DP_PG_LOCATION_MAX,
};

enum dp_pg_pixel_mask {
	DP_PG_PIXEL_MASK_NONE,
	DP_PG_PIXEL_ODD_MASK,
	DP_PG_PIXEL_EVEN_MASK,
	DP_PG_PIXEL_MASK_MAX,
};

enum dp_pg_purecolor {
	DP_PG_PURECOLOR_NONE,
	DP_PG_PURECOLOR_BLUE,
	DP_PG_PURECOLOR_GREEN,
	DP_PG_PURECOLOR_RED,
	DP_PG_PURECOLOR_MAX,
};

enum dp_pg_sel {
	DP_PG_20BIT,
	DP_PG_80BIT,
	DP_PG_11BIT,
	DP_PG_8BIT,
	DP_PG_PRBS7,
};

enum dp_power_status_type {
	DP_POWER_STATUS_NONE,
	DP_POWER_STATUS_AC_ON,
	DP_POWER_STATUS_DC_ON,
	DP_POWER_STATUS_PS_ON,
	DP_POWER_STATUS_DC_OFF,
	DP_POWER_STATUS_POWER_SAVING,
};

enum dp_sdp_asp_hb3_auch {
	DP_SDP_ASP_HB3_AU02CH = 0x01,
	DP_SDP_ASP_HB3_AU08CH = 0x07,
};

enum dp_sdp_hb1_pkg_type {
	DP_SDP_HB1_PKG_RESERVE = 0x00,
	DP_SDP_HB1_PKG_AUDIO_TS = 0x01,
	DP_SDP_HB1_PKG_AUDIO = 0x02,
	DP_SDP_HB1_PKG_EXT = 0x04,
	DP_SDP_HB1_PKG_ACM = 0x05,
	DP_SDP_HB1_PKG_ISRC = 0x06,
	DP_SDP_HB1_PKG_VSC = 0x07,
	DP_SDP_HB1_PKG_CAMERA = 0x08,
	DP_SDP_HB1_PKG_PPS = 0x10,
	DP_SDP_HB1_PKG_EXT_VESA = 0x20,
	DP_SDP_HB1_PKG_EXT_CEA = 0x21,
	DP_SDP_HB1_PKG_NON_AINFO = 0x80,
	DP_SDP_HB1_PKG_VS_INFO = 0x81,
	DP_SDP_HB1_PKG_AVI_INFO = 0x82,
	DP_SDP_HB1_PKG_SPD_INFO = 0x83,
	DP_SDP_HB1_PKG_AINFO = 0x84,
	DP_SDP_HB1_PKG_MPG_INFO = 0x85,
	DP_SDP_HB1_PKG_NTSC_INFO = 0x86,
	DP_SDP_HB1_PKG_DRM_INFO = 0x87,
	DP_SDP_HB1_PKG_MAX_NUM
};

enum dp_sdp_pkg_type {
	DP_SDP_PKG_NONE,
	DP_SDP_PKG_ACM,
	DP_SDP_PKG_ISRC,
	DP_SDP_PKG_AVI,
	DP_SDP_PKG_AUI,
	DP_SDP_PKG_SPD,
	DP_SDP_PKG_MPEG,
	DP_SDP_PKG_NTSC,
	DP_SDP_PKG_VSP,
	DP_SDP_PKG_VSC,
	DP_SDP_PKG_EXT,
	DP_SDP_PKG_PPS0,
	DP_SDP_PKG_PPS1,
	DP_SDP_PKG_PPS2,
	DP_SDP_PKG_PPS3,
	DP_SDP_PKG_RESERVED,
	DP_SDP_PKG_DRM,
	DP_SDP_PKG_ADS,
	DP_SDP_PKG_MAX_NUM
};

enum dp_train_stage {
	DP_LT_NONE		= 0x0000,
	DP_LT_CR_L0_FAIL	= 0x0008,
	DP_LT_CR_L1_FAIL	= 0x0009,
	DP_LT_CR_L2_FAIL	= 0x000a,
	DP_LT_EQ_L0_FAIL	= 0x0080,
	DP_LT_EQ_L1_FAIL	= 0x0090,
	DP_LT_EQ_L2_FAIL	= 0x00a0,
	DP_LT_PASS		= 0x7777,
};

enum dp_usb_pin_assign_type {
	DP_USB_PIN_ASSIGNMENT_C = 4,
	DP_USB_PIN_ASSIGNMENT_D = 8,
	DP_USB_PIN_ASSIGNMENT_E = 16,
	DP_USB_PIN_ASSIGNMENT_F = 32,
	DP_USB_PIN_ASSIGNMENT_MAX_NUM,
};

enum dp_version {
	DP_VER_11 = 0x11,
	DP_VER_12 = 0x12,
	DP_VER_14 = 0x14,
	DP_VER_12_14 = 0x16,
	DP_VER_14_14 = 0x17,
	DP_VER_MAX,
};

enum dp_video_mute {
	DP_VIDEO_UNMUTE = 1,
	DP_VIDEO_MUTE = 2,
};

union dp_rx_audio_chsts {
	struct{
		u8 rev : 1;
		u8 is_lpcm : 1;
		u8 copy_right : 1;
		u8 addition_format_info : 3;
		u8 channel_status_mode : 2;
		u8 category_code;
		u8 source_number : 4;
		u8 channel_number : 4;
		u8 sampling_freq : 4;
		u8 clock_accuary : 2;
		u8 rev2 : 2;
		u8 word_len : 4;
		u8 original_sampling_freq : 4;
	} audio_chsts;

	u8 audio_chsts_raw[IEC_CH_STATUS_LEN];
};

struct drm_display_limit_mode {
	int hdisplay;
	int vdisplay;
	int vrefresh;
	int clock;
	int valid;
};

static const u32 mt8196_input_fmts[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_YUV8_1X24,
	MEDIA_BUS_FMT_YUYV8_1X16,
};

static const u32 mt8196_output_fmts[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_YUV8_1X24,
	MEDIA_BUS_FMT_YUYV8_1X16,
};

static bool mtk_dp_mst_link_status(struct mtk_dp *mtk_dp);
static bool mtk_dp_link_status_v2(struct mtk_dp *mtk_dp);
static int mtk_dp_training_handle_v2(struct mtk_dp *mtk_dp);

static unsigned long mtk_dp_atf_call_v2(struct mtk_dp *mtk_dp, unsigned int cmd, unsigned int para)
{
	struct arm_smccc_res res;
	u32 x3 = (cmd << 16) | para;

	arm_smccc_smc(MTK_DP_SIP_CONTROL_AARCH32, cmd, para,
		      x3, 0xfefd, 0, 0, 0, &res);

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] %s, cmd:0x%x, p1:0x%x, ret:0x%lx-0x%lx",
		    __func__, cmd, para, res.a0, res.a1);
	return res.a1;
}

u32 mtk_dp_read_v2(struct mtk_dp *mtk_dp, u32 offset)
{
	u32 read_val = 0;

	if (offset > MAX_MAC_REG_RANG) {
		dev_err(mtk_dp->dev, "[DPTX] %s, error reg:0x%p, offset:0x%x\n",
			__func__, mtk_dp->regs, offset);
		return 0;
	}

	read_val = readl(mtk_dp->regs + offset - (offset % 4))
			 >> ((offset % 4) * 8);

	return read_val;
}

static void mtk_dp_write_v2(struct mtk_dp *mtk_dp, u32 offset, u32 val)
{
	if ((offset % 4 != 0) || offset > MAX_MAC_REG_RANG) {
		dev_err(mtk_dp->dev, "[DPTX] %s, error reg:0x%p, offset:0x%x, value:0x%x\n",
			__func__, mtk_dp->regs, offset, val);
		return;
	}

	writel(val, mtk_dp->regs + offset);
}

void mtk_dp_mask_v2(struct mtk_dp *mtk_dp, u32 offset, u32 val, u32 mask)
{
	void __iomem *reg = mtk_dp->regs + offset;
	u32 tmp;

	if ((offset % 4 != 0) || offset > MAX_MAC_REG_RANG) {
		dev_err(mtk_dp->dev, "[DPTX] %s, error reg:0x%p, offset:0x%x, value:0x%x\n",
			__func__, mtk_dp->regs, offset, val);
		return;
	}

	tmp = readl(reg);
	tmp = (tmp & ~mask) | (val & mask);
	writel(tmp, reg);
}

void mtk_dp_write_byte_v2(struct mtk_dp *mtk_dp,
			  u32 addr, u8 val, u32 mask)
{
	if (addr % 2)
		mtk_dp_mask_v2(mtk_dp, addr - 1, (u32)(val << 8), (mask << 8));
	else
		mtk_dp_mask_v2(mtk_dp, addr, (u32)val, mask);
}

int mtk_dp_update_bits_v2(struct mtk_dp *mtk_dp, u32 offset,
			  u32 val, u32 mask)
{
	mtk_dp_mask_v2(mtk_dp, offset, val, mask);

	return 0;
}

static u32 mtk_dp_phy_read_v2(struct mtk_dp *mtk_dp, u32 offset)
{
	u32 read_val = 0;

	if (offset > MAX_PHYD_REG_RANG) {
		dev_err(mtk_dp->dev, "[DPTX] %s, error offset:0x%x\n",
			__func__, offset);
		return 0;
	}

	read_val = readl(mtk_dp->phyd_regs + offset - (offset % 4))
			 >> ((offset % 4) * 8);

	return read_val;
}

static void mtk_dp_phy_write_v2(struct mtk_dp *mtk_dp, u32 offset, u32 val)
{
	if ((offset % 4 != 0) || offset > MAX_PHYD_REG_RANG) {
		dev_err(mtk_dp->dev, "[DPTX] %s, error offset:0x%x, value:0x%x\n",
			__func__, offset, val);
		return;
	}

	writel(val, mtk_dp->phyd_regs + offset);
}

static void mtk_dp_phy_mask_v2(struct mtk_dp *mtk_dp, u32 offset, u32 val, u32 mask)
{
	void __iomem *reg = mtk_dp->phyd_regs + offset;
	u32 tmp;

	if ((offset % 4 != 0) || offset > MAX_PHYD_REG_RANG) {
		dev_err(mtk_dp->dev, "[DPTX] %s, error reg:0x%p, offset:0x%x, value:0x%x\n",
			__func__, mtk_dp->phyd_regs, offset, val);
		return;
	}

	tmp = readl(reg);
	tmp = (tmp & ~mask) | (val & mask);
	writel(tmp, reg);
}

static void mtk_dp_phy_write_byte_v2(struct mtk_dp *mtk_dp,
			   	     u32 addr, u8 val, u32 mask)
{
	if (addr % 2)
		mtk_dp_phy_mask_v2(mtk_dp, addr - 1, (u32)(val << 8), (mask << 8));
	else
		mtk_dp_phy_mask_v2(mtk_dp, addr, (u32)val, mask);
}

static u8 dp_aux_read_bytes_v2(struct mtk_dp *mtk_dp, u8 cmd,
			       u64  dpcd_addr, size_t length, u8 *rx_buf)
{
	bool vaild_cmd = false;
	u8 phy_status = 0x00;
	u8 reply_cmd = 0xff;
	u8 rd_count = 0x0;
	u8 aux_irq_status = 0;
	u8 ret = AUX_HW_FAILED;
	unsigned int wait_reply_count = AUX_WAIT_REPLY_LP_CNT_NUM;

	WRITE_BYTE(mtk_dp, REG_3640_AUX_TX_P0, 0x7f);
	usleep_range(AUX_WRITE_READ_WAIT_TIME, AUX_WRITE_READ_WAIT_TIME + 1);

	if (length > 16 ||
	    (cmd == AUX_CMD_NATIVE_R && length == 0x0))
		return AUX_INVALID_CMD;

	WRITE_BYTE(mtk_dp, REG_3650_AUX_TX_P0 + 1, 0x01);
	WRITE_BYTE(mtk_dp, REG_3644_AUX_TX_P0, cmd);
	WRITE_2BYTE(mtk_dp, REG_3648_AUX_TX_P0, dpcd_addr & GENMASK(15,0));
	WRITE_BYTE_MASK(mtk_dp, REG_364C_AUX_TX_P0,
			dpcd_addr >> 16,
			MCU_REQUEST_ADDRESS_MSB_AUX_TX_P0_FLDMASK);

	if (length > 0) {
		WRITE_2BYTE_MASK(mtk_dp, REG_3650_AUX_TX_P0,
				 (length - 1) << MCU_REQUEST_DATA_NUM_AUX_TX_P0_FLDMASK_POS,
				 MCU_REQUEST_DATA_NUM_AUX_TX_P0_FLDMASK);
		WRITE_BYTE(mtk_dp, REG_362C_AUX_TX_P0, 0x00);
	}

	if (cmd == AUX_CMD_I2C_R || cmd == AUX_CMD_I2C_R_MOT0)
		if (length == 0x0)
			WRITE_2BYTE_MASK(mtk_dp, REG_362C_AUX_TX_P0,
					 0x01 << AUX_NO_LENGTH_AUX_TX_P0_FLDMASK_POS,
					 AUX_NO_LENGTH_AUX_TX_P0_FLDMASK);

	WRITE_2BYTE_MASK(mtk_dp, REG_3630_AUX_TX_P0,
			 0x01 << AUX_TX_REQUEST_READY_AUX_TX_P0_FLDMASK_POS,
			 AUX_TX_REQUEST_READY_AUX_TX_P0_FLDMASK);

	while (--wait_reply_count) {
		aux_irq_status = READ_BYTE(mtk_dp, REG_3640_AUX_TX_P0);

		if (aux_irq_status & AUX_RX_AUX_RECV_COMPLETE_IRQ_AUX_TX_P0_FLDMASK) {
			dev_dbg(mtk_dp->dev, "[AUX] Read Complete irq\n");
			vaild_cmd = true;
			break;
		}

		if (aux_irq_status & AUX_RX_EDID_RECV_COMPLETE_IRQ_AUX_TX_P0_FLDMASK) {
			vaild_cmd = true;
			break;
		}

		if (aux_irq_status & AUX_400US_TIMEOUT_IRQ_AUX_TX_P0_FLDMASK) {
			/* for no reply should wait at least 3200 us */
			usleep_range(AUX_NO_REPLY_WAIT_TIME, AUX_NO_REPLY_WAIT_TIME + 1);
			drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] (AUX Read)HW Timeout 400us irq");
			break;
		}
		udelay(1);
	}

	if (wait_reply_count == 0x0) {
		phy_status = READ_BYTE(mtk_dp, REG_3628_AUX_TX_P0);
		if (phy_status != 0x01)
			dev_err(mtk_dp->dev, "[DPTX] Aux R:Aux hang, need SW reset\n");

		WRITE_2BYTE_MASK(mtk_dp, REG_3650_AUX_TX_P0,
				 0x01 << MCU_ACK_TRANSACTION_COMPLETE_AUX_TX_P0_FLDMASK_POS,
				 MCU_ACK_TRANSACTION_COMPLETE_AUX_TX_P0_FLDMASK);
		WRITE_BYTE(mtk_dp, REG_3640_AUX_TX_P0, 0x7F);

		drm_dbg_kms(mtk_dp->drm_dev,
			    "[DPTX] wait_reply_count:%x, TimeOut", wait_reply_count);
		return AUX_HW_FAILED;
	}

	reply_cmd = READ_BYTE(mtk_dp, REG_3624_AUX_TX_P0) & GENMASK(3,0);
	if (reply_cmd)
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] reply_cmd:%x, NACK or Defer\n", reply_cmd);

	if (length == 0)
		WRITE_BYTE(mtk_dp, REG_362C_AUX_TX_P0, 0x00);

	if (reply_cmd == AUX_REPLY_ACK) {
		WRITE_2BYTE_MASK(mtk_dp, REG_3620_AUX_TX_P0,
				 0x0 << AUX_RD_MODE_AUX_TX_P0_FLDMASK_POS,
				 AUX_RD_MODE_AUX_TX_P0_FLDMASK);

		for (rd_count = 0x0; rd_count < length;
				rd_count++) {
			WRITE_2BYTE_MASK(mtk_dp, REG_3620_AUX_TX_P0,
					 0x01 << AUX_RX_FIFO_READ_PULSE_AUX_TX_P0_FLDMASK_POS,
					 AUX_RX_FIFO_READ_PULSE_AUX_TX_P0_FLDMASK);

			*(rx_buf + rd_count) = READ_BYTE(mtk_dp, REG_3620_AUX_TX_P0);
		}
	}

	WRITE_2BYTE_MASK(mtk_dp, REG_3650_AUX_TX_P0,
			 0x01 << MCU_ACK_TRANSACTION_COMPLETE_AUX_TX_P0_FLDMASK_POS,
			 MCU_ACK_TRANSACTION_COMPLETE_AUX_TX_P0_FLDMASK);
	WRITE_BYTE(mtk_dp, REG_3640_AUX_TX_P0, 0x7F);

	if (vaild_cmd) {
		dev_dbg(mtk_dp->dev, "[AUX] Read reply_cmd:%d\n", reply_cmd);
		ret = reply_cmd;
	} else {
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] [AUX] Timeout Read reply_cmd:%d\n", reply_cmd);
		ret = AUX_HW_FAILED;
	}

	return ret;
}

static u8 dp_aux_write_bytes_v2(struct mtk_dp *mtk_dp,
		      u8 cmd, u64  dpcd_addr, size_t length, u8 *data)
{
	bool vaild_cmd = false;
	u8 reply_cmd = 0x0;
	u8 aux_irq_status;
	u8 phy_status = 0x00;
	u8 i, ret = AUX_HW_FAILED;
	u16 wait_reply_count = AUX_WAIT_REPLY_LP_CNT_NUM;
	u8 reg_index;

	if (length > 16 || (cmd == AUX_CMD_NATIVE_W && length == 0x0))
		return AUX_INVALID_CMD;

	WRITE_BYTE_MASK(mtk_dp, REG_3704_AUX_TX_P0,
			1 << AUX_TX_FIFO_NEW_MODE_EN_AUX_TX_P0_FLDMASK_POS,
			AUX_TX_FIFO_NEW_MODE_EN_AUX_TX_P0_FLDMASK);
	WRITE_BYTE(mtk_dp, REG_3650_AUX_TX_P0 + 1, 0x01);
	WRITE_BYTE(mtk_dp, REG_3640_AUX_TX_P0, 0x7F);
	usleep_range(AUX_WRITE_READ_WAIT_TIME, AUX_WRITE_READ_WAIT_TIME + 1);

	WRITE_BYTE(mtk_dp, REG_3650_AUX_TX_P0 + 1, 0x01);
	WRITE_BYTE(mtk_dp, REG_3644_AUX_TX_P0, cmd);
	WRITE_BYTE(mtk_dp, REG_3648_AUX_TX_P0, dpcd_addr & GENMASK(7,0));
	WRITE_BYTE(mtk_dp, REG_3648_AUX_TX_P0 + 1, (dpcd_addr >> 8) & GENMASK(7,0));
	WRITE_BYTE_MASK(mtk_dp, REG_364C_AUX_TX_P0,
			dpcd_addr >> 16,
			MCU_REQUEST_ADDRESS_MSB_AUX_TX_P0_FLDMASK);

	if (length > 0) {
		WRITE_BYTE(mtk_dp, REG_362C_AUX_TX_P0, 0x00);
		for (i = 0x0; i < (length + 1) / 2; i++)
			for (reg_index = 0; reg_index < 2; reg_index++)
				if ((i * 2 + reg_index) < length)
					WRITE_BYTE(mtk_dp, REG_3708_AUX_TX_P0 + i * 4 + reg_index,
						   data[i * 2 + reg_index]);
		WRITE_BYTE(mtk_dp, REG_3650_AUX_TX_P0 + 1, ((length - 1) & GENMASK(3,0)) << 4);
	} else {
		WRITE_BYTE(mtk_dp, REG_362C_AUX_TX_P0, 0x01);
	}

	WRITE_BYTE_MASK(mtk_dp, REG_3704_AUX_TX_P0,
			AUX_TX_FIFO_WRITE_DATA_NEW_MODE_TOGGLE_AUX_TX_P0_FLDMASK,
			AUX_TX_FIFO_WRITE_DATA_NEW_MODE_TOGGLE_AUX_TX_P0_FLDMASK);
	WRITE_BYTE(mtk_dp, REG_3630_AUX_TX_P0, 0x08);

	while (--wait_reply_count) {
		aux_irq_status = READ_BYTE(mtk_dp, REG_3640_AUX_TX_P0);

		if (aux_irq_status & AUX_RX_AUX_RECV_COMPLETE_IRQ_AUX_TX_P0_FLDMASK) {
			dev_dbg(mtk_dp->dev, "[AUX] Write Complete irq\n");
			vaild_cmd = true;
			break;
		}

		if (aux_irq_status & AUX_400US_TIMEOUT_IRQ_AUX_TX_P0_FLDMASK) {
			/* for no reply should wait at least 3200 us */
			usleep_range(AUX_NO_REPLY_WAIT_TIME, AUX_NO_REPLY_WAIT_TIME + 1);
			drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] (AUX write)HW Timeout 400us irq");
			break;
		}
		udelay(1);
	}

	if (wait_reply_count == 0x0) {
		phy_status = READ_BYTE(mtk_dp, REG_3628_AUX_TX_P0);
		if (phy_status != 0x01)
			dev_err(mtk_dp->dev, "[DPTX] Aux Write:Aux hang, need SW reset!\n");

		WRITE_BYTE(mtk_dp, REG_3650_AUX_TX_P0 + 1, 0x01);
		WRITE_BYTE(mtk_dp, REG_3640_AUX_TX_P0, 0x7F);

		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] reply_cmd:0x%x, wait_reply_count:%d\n",
			    reply_cmd, wait_reply_count);
		return AUX_HW_FAILED;
	}

	reply_cmd = READ_BYTE(mtk_dp, REG_3624_AUX_TX_P0) & GENMASK(3,0);
	if (reply_cmd)
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] reply_cmd:%x, NACK or Defer\n", reply_cmd);

	WRITE_BYTE(mtk_dp, REG_3650_AUX_TX_P0 + 1, 0x01);

	if (length == 0)
		WRITE_BYTE(mtk_dp, REG_362C_AUX_TX_P0, 0x00);

	WRITE_BYTE(mtk_dp, REG_3640_AUX_TX_P0, 0x7F);

	if (vaild_cmd) {
		dev_dbg(mtk_dp->dev, "[AUX] Write reply_cmd:%d\n", reply_cmd);
		ret = reply_cmd;
	} else {
		drm_dbg_kms(mtk_dp->drm_dev,
			    "[DPTX] [AUX] Timeout, Write reply_cmd:%d\n", reply_cmd);
		ret = AUX_HW_FAILED;
	}

	return ret;
}

static bool mtk_dp_aux_write_bytes_v2(struct mtk_dp *mtk_dp, u8 cmd,
				      u32 dpcd_addr, size_t length, u8 *data)
{
	u8 reply_status, retry_limit = 8;

	if (!mtk_dp->training_info.cable_plug_in)
		return false;

	while (--retry_limit) {
		reply_status = dp_aux_write_bytes_v2(mtk_dp, cmd, dpcd_addr,
						     length, data);
		if (reply_status == AUX_REPLY_ACK)
			return true;

		usleep_range(50, 51);
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] Remaining retries: %u\n", retry_limit);
	}

	dev_err(mtk_dp->dev, "[DPTX] Aux Write Fail, cmd:%d, addr:0x%x, len:%zu\n",
		cmd, dpcd_addr, length);

	return false;
}

static bool mtk_dp_aux_write_dpcd_v2(struct mtk_dp *mtk_dp, u8 cmd,
				     u32 dpcd_addr, size_t length, u8 *data)
{
	bool ret = true;
	size_t i;

	for (i = 0; i + DP_AUX_MAX_PAYLOAD_BYTES <= length; i += DP_AUX_MAX_PAYLOAD_BYTES) {
		ret &= mtk_dp_aux_write_bytes_v2(mtk_dp, cmd, dpcd_addr + i,
						DP_AUX_MAX_PAYLOAD_BYTES,
						data + i);
	}

	if (length % DP_AUX_MAX_PAYLOAD_BYTES || length == 0) {
		ret &= mtk_dp_aux_write_bytes_v2(mtk_dp, cmd, dpcd_addr + i,
						length % DP_AUX_MAX_PAYLOAD_BYTES,
						data + i);
	}

	dev_dbg(mtk_dp->dev, "Aux write cmd:%d, addr:0x%x, len:%zu, %s\n",
		cmd, dpcd_addr, length, ret ? "Success" : "Fail");

	for (i = 0; i < length; i++)
		dev_dbg(mtk_dp->dev, "DPCD%zx:0x%x", dpcd_addr + i, data[i]);

	return ret;
}

static bool mtk_dp_aux_read_bytes_v2(struct mtk_dp *mtk_dp, u8 cmd,
				     u32 dpcd_addr, size_t length, u8 *data)
{
	u8 reply_status, retry_limit = 8;

	if (!mtk_dp->training_info.cable_plug_in)
		return false;

	while (--retry_limit) {
		reply_status = dp_aux_read_bytes_v2(mtk_dp, cmd, dpcd_addr,
						    length, data);
		if (reply_status == AUX_REPLY_ACK)
			return true;

		usleep_range(50, 51);
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] Remaining retries: %u\n", retry_limit);
	}

	dev_err(mtk_dp->dev, "[DPTX] Aux Read Fail, cmd:%d, addr:0x%x, len:%zu\n",
		cmd, dpcd_addr, length);

	return false;
}

static bool mtk_dp_aux_read_dpcd_v2(struct mtk_dp *mtk_dp, u8 cmd,
				    u32 dpcd_addr, size_t length, u8 *data)
{
	bool ret = true;
	size_t i;

	memset(data, 0, length);

	for (i = 0; i + DP_AUX_MAX_PAYLOAD_BYTES <= length; i += DP_AUX_MAX_PAYLOAD_BYTES) {
		ret &= mtk_dp_aux_read_bytes_v2(mtk_dp, cmd, dpcd_addr + i,
						DP_AUX_MAX_PAYLOAD_BYTES,
						data + i);
	}

	if (length % DP_AUX_MAX_PAYLOAD_BYTES || length == 0) {
		ret &= mtk_dp_aux_read_bytes_v2(mtk_dp, cmd, dpcd_addr + i,
						length % DP_AUX_MAX_PAYLOAD_BYTES,
						data + i);
	}

	dev_dbg(mtk_dp->dev, "Aux Read cmd:%d, addr:0x%x, len:%zu, %s\n",
		cmd, dpcd_addr, length, ret ? "Success" : "Fail");

	for (i = 0; i < length; i++)
		dev_dbg(mtk_dp->dev, "DPCD%zx:0x%x", dpcd_addr + i, data[i]);

	return ret;
}

static ssize_t mtk_dp_aux_transfer_v2(struct drm_dp_aux *mtk_aux,
				      struct drm_dp_aux_msg *msg)
{
	u8 cmd;
	void *data;
	size_t length, ret = 0;
	u32 addr;
	bool ack = false;
	struct mtk_dp *mtk_dp;

	mtk_dp = container_of(mtk_aux, struct mtk_dp, aux);
	cmd = msg->request;
	addr = msg->address;
	length = msg->size;
	data = msg->buffer;

	if (mtk_dp->disp_state == DP_DISP_STATE_SUSPENDING ||
	    mtk_dp->disp_state == DP_DISP_STATE_SUSPEND ||
	    !mtk_dp->training_info.cable_plug_in) {
		msg->reply = DP_AUX_NATIVE_REPLY_NACK | DP_AUX_I2C_REPLY_NACK;
		return -EIO;
	}

	switch (cmd) {
	case DP_AUX_I2C_MOT:
	case DP_AUX_I2C_WRITE:
	case DP_AUX_NATIVE_WRITE:
	case DP_AUX_I2C_WRITE_STATUS_UPDATE:
	case DP_AUX_I2C_WRITE_STATUS_UPDATE | DP_AUX_I2C_MOT:
		cmd &= ~DP_AUX_I2C_WRITE_STATUS_UPDATE;
		ack = mtk_dp_aux_write_dpcd_v2(mtk_dp, cmd,
					       addr, length, data);
		break;

	case DP_AUX_I2C_READ:
	case DP_AUX_NATIVE_READ:
	case DP_AUX_I2C_READ | DP_AUX_I2C_MOT:
		ack = mtk_dp_aux_read_dpcd_v2(mtk_dp, cmd,
					      addr, length, data);
		break;

	default:
		dev_err(mtk_dp->dev, "[DPTX] invalid aux cmd:%d\n", cmd);
		ret = -EINVAL;
		break;
	}

	if (ack) {
		msg->reply = DP_AUX_NATIVE_REPLY_ACK | DP_AUX_I2C_REPLY_ACK;
		ret = length;
	} else {
		msg->reply = DP_AUX_NATIVE_REPLY_NACK | DP_AUX_I2C_REPLY_NACK;
		ret = -EAGAIN;
	}

	return ret;
}

static void mtk_dp_aux_swap_enable_v2(struct mtk_dp *mtk_dp)
{
	WRITE_2BYTE_MASK(mtk_dp, REG_360C_AUX_TX_P0,
			 1 << AUX_SWAP_AUX_TX_P0_FLDMASK_POS,
			 AUX_SWAP_AUX_TX_P0_FLDMASK);
	WRITE_BYTE_MASK(mtk_dp, REG_3680_AUX_TX_P0,
			1 << AUX_SWAP_TX_AUX_TX_P0_FLDMASK_POS,
			AUX_SWAP_TX_AUX_TX_P0_FLDMASK);
}

static void mtk_dp_aux_swap_disable_v2(struct mtk_dp *mtk_dp)
{
	WRITE_2BYTE_MASK(mtk_dp, REG_360C_AUX_TX_P0, 0,
			 AUX_SWAP_AUX_TX_P0_FLDMASK);
	WRITE_2BYTE_MASK(mtk_dp, REG_3680_AUX_TX_P0, 0,
			 AUX_SWAP_TX_AUX_TX_P0_FLDMASK);
}

static void mtk_dp_aux_setting_v2(struct mtk_dp *mtk_dp)
{
	if (mtk_dp->swap_enable)
		mtk_dp_aux_swap_enable_v2(mtk_dp);
	else
		mtk_dp_aux_swap_disable_v2(mtk_dp);

	/* modify timeout threshold = 1595 [12 : 8] */
	WRITE_2BYTE_MASK(mtk_dp, REG_360C_AUX_TX_P0, 0x1D0C, AUX_TIMEOUT_THR_AUX_TX_P0_FLDMASK);
	WRITE_BYTE_MASK(mtk_dp, REG_3658_AUX_TX_P0, 0, BIT(0));

	WRITE_2BYTE(mtk_dp, REG_36A0_AUX_TX_P0, 0xfffc);

	/* 26M */
	WRITE_2BYTE_MASK(mtk_dp, REG_3634_AUX_TX_P0,
			 0x19 << AUX_TX_OVER_SAMPLE_RATE_AUX_TX_P0_FLDMASK_POS,
			 AUX_TX_OVER_SAMPLE_RATE_AUX_TX_P0_FLDMASK);
	WRITE_BYTE_MASK(mtk_dp, REG_3614_AUX_TX_P0,
			0x0D << AUX_RX_UI_CNT_THR_AUX_TX_P0_FLDMASK_POS,
			AUX_RX_UI_CNT_THR_AUX_TX_P0_FLDMASK);

	WRITE_4BYTE_MASK(mtk_dp, REG_37C8_AUX_TX_P0, MTK_ATOP_EN_AUX_TX_P0_FLDMASK,
			 MTK_ATOP_EN_AUX_TX_P0_FLDMASK);
	/* disable aux sync_stop detect function */
	WRITE_4BYTE_MASK(mtk_dp, REG_3690_AUX_TX_P0,
			 0x1 << RX_REPLY_COMPLETE_MODE_AUX_TX_P0_FLDMASK_POS,
			 RX_REPLY_COMPLETE_MODE_AUX_TX_P0_FLDMASK);

	/* Con Thd = 1.5ms+Vx0.1ms */
	WRITE_4BYTE_MASK(mtk_dp, REG_367C_AUX_TX_P0,
			 5 << HPD_CONN_THD_AUX_TX_P0_FLDMASK_POS,
			 HPD_CONN_THD_AUX_TX_P0_FLDMASK);
	/* DisCon Thd = 1.5ms+Vx0.1ms */
	WRITE_4BYTE_MASK(mtk_dp, REG_37A0_AUX_TX_P0,
			 5 << HPD_DISC_THD_AUX_TX_P0_FLDMASK_POS,
			 HPD_DISC_THD_AUX_TX_P0_FLDMASK);

	WRITE_4BYTE_MASK(mtk_dp, REG_3690_AUX_TX_P0,
			 RX_REPLY_COMPLETE_MODE_AUX_TX_P0_FLDMASK,
			 RX_REPLY_COMPLETE_MODE_AUX_TX_P0_FLDMASK);
}

static void mtk_dp_aux_init_v2(struct mtk_dp *mtk_dp)
{
	drm_dp_aux_init(&mtk_dp->aux);

	mtk_dp->aux.name = kasprintf(GFP_KERNEL, "DPDDC-MTK");
	mtk_dp->aux.transfer = mtk_dp_aux_transfer_v2;
}

static void mtk_dp_fec_init_setting_v2(struct mtk_dp *mtk_dp)
{
	WRITE_4BYTE_MASK(mtk_dp, REG_3540_DP_TRANS_P0,
			 1 << FEC_CLOCK_EN_MODE_DP_TRANS_P0_FLDMASK_POS,
			 FEC_CLOCK_EN_MODE_DP_TRANS_P0_FLDMASK);
	WRITE_4BYTE_MASK(mtk_dp, REG_3540_DP_TRANS_P0,
			 2 << FEC_FIFO_UNDER_POINT_DP_TRANS_P0_FLDMASK_POS,
			 FEC_FIFO_UNDER_POINT_DP_TRANS_P0_FLDMASK);
}

void mtk_dp_fec_enable_v2(struct mtk_dp *mtk_dp)
{
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] FEC enable\n");

	WRITE_BYTE_MASK(mtk_dp, REG_3540_DP_TRANS_P0, BIT(0), BIT(0));
}

static void mtk_dp_fec_disable_v2(struct mtk_dp *mtk_dp)
{
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] FEC disable\n");

	WRITE_BYTE_MASK(mtk_dp, REG_3540_DP_TRANS_P0, 0, BIT(0));
}

static void mtk_dp_fec_ready_v2(struct mtk_dp *mtk_dp)
{
	u8 *fec_cap = &mtk_dp->mtk_con[DP_FIRST_CON]->fec_cap;

	if (drm_dp_dpcd_readb(&mtk_dp->aux, DP_FEC_CAPABILITY, fec_cap) < 0) {
		dev_err(mtk_dp->dev, "[DPTX] fail to read FEC DPCD\n");
		return;
	}
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] FEC cap:0x%x\n", *fec_cap);

	if (drm_dp_sink_supports_fec(*fec_cap)) {
		if (drm_dp_dpcd_writeb(&mtk_dp->aux, DP_FEC_CONFIGURATION, DP_FEC_READY) <= 0)
			dev_err(mtk_dp->dev, "[DPTX] Failed to set FEC_READY\n");

		if (drm_dp_dpcd_writeb(&mtk_dp->aux, DP_FEC_STATUS,
			       DP_FEC_DECODE_EN_DETECTED | DP_FEC_DECODE_DIS_DETECTED) <= 0)
			dev_err(mtk_dp->dev, "[DPTX] Failed to clear FEC detected flags\n");
	}
}

static void mtk_dp_msa_enable_bypass_v2(struct mtk_dp *mtk_dp,
			      const enum dp_encoder_id encoder_id, bool enable)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	WRITE_2BYTE_MASK(mtk_dp, REG_3030_DP_ENCODER0_P0 + reg_offset, enable ? 0 : 0x3ff, GENMASK(10,0));
}

static void mtk_dp_msa_set_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);
	struct dp_timing_parameter *dp_timing = &mtk_dp->info[encoder_id].dp_output_timing;

	WRITE_2BYTE(mtk_dp, REG_3010_DP_ENCODER0_P0 + reg_offset,
		    dp_timing->htt);
	WRITE_2BYTE(mtk_dp, REG_3018_DP_ENCODER0_P0 + reg_offset,
		    dp_timing->hsw + dp_timing->hbp);
	WRITE_2BYTE_MASK(mtk_dp, REG_3028_DP_ENCODER0_P0 + reg_offset,
			 dp_timing->hsw << HSW_SW_DP_ENCODER0_P0_FLDMASK_POS,
			 HSW_SW_DP_ENCODER0_P0_FLDMASK);
	WRITE_2BYTE_MASK(mtk_dp, REG_3028_DP_ENCODER0_P0 + reg_offset,
			 dp_timing->hsp << HSP_SW_DP_ENCODER0_P0_FLDMASK_POS,
			 HSP_SW_DP_ENCODER0_P0_FLDMASK);
	WRITE_2BYTE(mtk_dp, REG_3020_DP_ENCODER0_P0 + reg_offset,
		    dp_timing->hde);
	WRITE_2BYTE(mtk_dp, REG_3014_DP_ENCODER0_P0 + reg_offset,
		    dp_timing->vtt);
	WRITE_2BYTE(mtk_dp, REG_301C_DP_ENCODER0_P0 + reg_offset,
		    dp_timing->vsw + dp_timing->vbp);
	WRITE_2BYTE_MASK(mtk_dp, REG_302C_DP_ENCODER0_P0 + reg_offset,
			 dp_timing->vsw << VSW_SW_DP_ENCODER0_P0_FLDMASK_POS,
			 VSW_SW_DP_ENCODER0_P0_FLDMASK);
	WRITE_2BYTE_MASK(mtk_dp, REG_302C_DP_ENCODER0_P0 + reg_offset,
			 dp_timing->vsp << VSP_SW_DP_ENCODER0_P0_FLDMASK_POS,
			 VSP_SW_DP_ENCODER0_P0_FLDMASK);
	WRITE_2BYTE(mtk_dp, REG_3024_DP_ENCODER0_P0 + reg_offset,
		    dp_timing->vde);
	if (!mtk_dp->dsc_enable[encoder_id])
		WRITE_2BYTE(mtk_dp, REG_3064_DP_ENCODER0_P0 + reg_offset,
			    dp_timing->hde);
	WRITE_2BYTE(mtk_dp, REG_3154_DP_ENCODER0_P0 + reg_offset,
		    (dp_timing->htt));
	WRITE_2BYTE(mtk_dp, REG_3158_DP_ENCODER0_P0 + reg_offset,
		    (dp_timing->hfp));
	WRITE_2BYTE(mtk_dp, REG_315C_DP_ENCODER0_P0 + reg_offset,
		    (dp_timing->hsw));
	WRITE_2BYTE(mtk_dp, REG_3160_DP_ENCODER0_P0 + reg_offset,
		    dp_timing->hbp + dp_timing->hsw);
	WRITE_2BYTE(mtk_dp, REG_3164_DP_ENCODER0_P0 + reg_offset,
		    (dp_timing->hde));
	WRITE_2BYTE(mtk_dp, REG_3168_DP_ENCODER0_P0 + reg_offset,
		    dp_timing->vtt);
	WRITE_2BYTE(mtk_dp, REG_316C_DP_ENCODER0_P0 + reg_offset,
		    dp_timing->vfp);
	WRITE_2BYTE(mtk_dp, REG_3170_DP_ENCODER0_P0 + reg_offset,
		    dp_timing->vsw);
	WRITE_2BYTE(mtk_dp, REG_3174_DP_ENCODER0_P0 + reg_offset,
		    dp_timing->vbp + dp_timing->vsw);
	WRITE_2BYTE(mtk_dp, REG_3178_DP_ENCODER0_P0 + reg_offset,
		    dp_timing->vde);

	drm_dbg_kms(mtk_dp->drm_dev,
		    "[DPTX] [%d] set MSA, Htt:%d, Vtt:%d, Hact:%d, Vact:%d, fps:%d\n",
		    encoder_id,
		    dp_timing->htt, dp_timing->vtt,
		    dp_timing->hde, dp_timing->vde, dp_timing->frame_rate);
}

static bool mtk_dp_mn_overwrite_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id,
			 bool enable, u64 video_m, u64 video_n)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	if (enable) {
		/* Turn-on overwrite MN */
		WRITE_2BYTE(mtk_dp, REG_3008_DP_ENCODER0_P0 + reg_offset,
			    video_m & GENMASK(15,0));
		WRITE_BYTE(mtk_dp, REG_300C_DP_ENCODER0_P0 + reg_offset,
			   ((video_m >> 16) & GENMASK(7,0)));
		WRITE_2BYTE(mtk_dp, REG_3044_DP_ENCODER0_P0 + reg_offset,
			    video_n & GENMASK(15,0));
		WRITE_BYTE(mtk_dp, REG_3048_DP_ENCODER0_P0 + reg_offset,
			   (video_n >> 16) & GENMASK(7,0));

		WRITE_2BYTE(mtk_dp, REG_3050_DP_ENCODER0_P0 + reg_offset,
			    video_n & GENMASK(15,0));
		WRITE_BYTE(mtk_dp, REG_3054_DP_ENCODER0_P0 + reg_offset,
			   (video_n >> 16) & GENMASK(7,0));
		WRITE_BYTE_MASK(mtk_dp, REG_3004_DP_ENCODER0_P0 + 1 + reg_offset,
				BIT(0), BIT(0));
	} else {
		/* Turn-off overwrite MN */
		WRITE_BYTE_MASK(mtk_dp, REG_3004_DP_ENCODER0_P0 + 1 + reg_offset, 0, BIT(0));
	}

	return true;
}

static void mtk_dp_mn_calculate_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id)
{
	u8 frame_rate = 60;
	u32 pix_clk = 148500000;
	u32 pll_rate = (0x00d8 << 2) * 10; /*Base = 1Khz*/
	u32 val = 0, m_vid = 0;
	u32 n_vid = 0x8000;

	if (mtk_dp->info[encoder_id].dp_output_timing.frame_rate > 0) {
		frame_rate = mtk_dp->info[encoder_id].dp_output_timing.frame_rate;
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] Frame Rate = %d\n",
			    mtk_dp->info[encoder_id].dp_output_timing.frame_rate);

		pix_clk = (u32)mtk_dp->info[encoder_id].dp_output_timing.htt *
			  (u32)mtk_dp->info[encoder_id].dp_output_timing.vtt *
			  (u32)frame_rate;
	} else if (mtk_dp->info[encoder_id].dp_output_timing.pixcel_rate > 0) {
		frame_rate = 60;
		pix_clk = mtk_dp->info[encoder_id].dp_output_timing.pixcel_rate * 1000;
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] Pix Clk (kHz) = %llu\n",
			    mtk_dp->info[encoder_id].dp_output_timing.pixcel_rate);
	} else {
		frame_rate = 60;
		pix_clk = (u32)mtk_dp->info[encoder_id].dp_output_timing.htt *
			  (u32)mtk_dp->info[encoder_id].dp_output_timing.vtt *
			  (u32)frame_rate;

		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] both frame_rate & pix_clk = 0\n");
	}

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] pix_clk = 0x%x\r\n", pix_clk);

	val = pix_clk / (100000);

	if (val > 0) {
		m_vid = (val * n_vid) / pll_rate;

		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] Cal PR = %d x(1/10) Mhz\n", val);
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] m_vid 0x%x\n", m_vid);
		mtk_dp->info[encoder_id].dp_output_timing.pixcel_rate = pix_clk / 1000;
		mtk_dp->info[encoder_id].video_m = m_vid;
		mtk_dp->info[encoder_id].video_n = n_vid;
	}

	if (mtk_dp->training_info.link_rate >= DP_LINK_RATE_UHBR10) {
		mtk_dp->info[encoder_id].video_m = pix_clk >> 24;
		mtk_dp->info[encoder_id].video_n = pix_clk & GENMASK(23,0);
		mtk_dp_mn_overwrite_v2(mtk_dp, encoder_id, true, m_vid, n_vid);
	} else if (mtk_dp->training_info.link_rate >= DP_LINK_RATE_RBR && val > 0) {
		m_vid = (val * n_vid) / (mtk_dp->training_info.link_rate * 270);
		mtk_dp->info[encoder_id].video_m = m_vid;
		mtk_dp->info[encoder_id].video_n = n_vid;
	} else {
		drm_dbg_kms(mtk_dp->drm_dev,
			    "[DPTX] Set video MN fail, due to invalid link rate\n");
	}
}

u8 mtk_dp_color_get_bpp_v2(u8 color_format, u8 color_depth)
{
	u8 color_bpp;

	switch (color_depth) {
	case DP_COLOR_DEPTH_6BIT:
		if (color_format == DP_PIXELFORMAT_YUV422)
			color_bpp = 16;
		else if (color_format == DP_PIXELFORMAT_YUV420)
			color_bpp = 12;
		else
			color_bpp = 18;
		break;
	case DP_COLOR_DEPTH_8BIT:
		if (color_format == DP_PIXELFORMAT_YUV422)
			color_bpp = 16;
		else if (color_format == DP_PIXELFORMAT_YUV420)
			color_bpp = 12;
		else
			color_bpp = 24;
		break;
	case DP_COLOR_DEPTH_10BIT:
		if (color_format == DP_PIXELFORMAT_YUV422)
			color_bpp = 20;
		else if (color_format == DP_PIXELFORMAT_YUV420)
			color_bpp = 15;
		else
			color_bpp = 30;
		break;
	case DP_COLOR_DEPTH_12BIT:
		if (color_format == DP_PIXELFORMAT_YUV422)
			color_bpp = 24;
		else if (color_format == DP_PIXELFORMAT_YUV420)
			color_bpp = 18;
		else
			color_bpp = 36;
		break;
	case DP_COLOR_DEPTH_16BIT:
		if (color_format == DP_PIXELFORMAT_YUV422)
			color_bpp = 32;
		else if (color_format == DP_PIXELFORMAT_YUV420)
			color_bpp = 24;
		else
			color_bpp = 48;
		break;
	default:
		color_bpp = 24;
		break;
	}

	return color_bpp;
}

static void mtk_dp_color_set_format_v2(struct mtk_dp *mtk_dp,
				       const enum dp_encoder_id encoder_id, u8 color_format)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] Set Color Format:0x%x\n", color_format);

	if (color_format == DP_PIXELFORMAT_RGB ||
	    color_format == DP_PIXELFORMAT_YUV444)
		WRITE_BYTE_MASK(mtk_dp, REG_303C_DP_ENCODER0_P0 + 1 + reg_offset,
				(0), GENMASK(6, 4));
	else if (color_format == DP_PIXELFORMAT_YUV422)
		WRITE_BYTE_MASK(mtk_dp, REG_303C_DP_ENCODER0_P0 + 1 + reg_offset,
				(BIT(4)), GENMASK(6, 4));
	else if (color_format == DP_PIXELFORMAT_YUV420)
		WRITE_BYTE_MASK(mtk_dp, REG_303C_DP_ENCODER0_P0 + 1 + reg_offset,
				(BIT(5)), GENMASK(6, 4));
}

static void mtk_dp_color_set_depth_v2(struct mtk_dp *mtk_dp,
				      const enum dp_encoder_id encoder_id, u8 color_depth)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	drm_dbg_kms(mtk_dp->drm_dev,
		    "[DPTX] Set Color Depth:%d (0~4=6/8/10/12/16 bpp)\n", color_depth);

	switch (color_depth) {
	case DP_COLOR_DEPTH_6BIT:
		WRITE_BYTE_MASK(mtk_dp, REG_303C_DP_ENCODER0_P0 + 1 + reg_offset, 4, 0x07);
		break;
	case DP_COLOR_DEPTH_8BIT:
		WRITE_BYTE_MASK(mtk_dp, REG_303C_DP_ENCODER0_P0 + 1 + reg_offset, 3, 0x07);
		break;
	case DP_COLOR_DEPTH_10BIT:
		WRITE_BYTE_MASK(mtk_dp, REG_303C_DP_ENCODER0_P0 + 1 + reg_offset, 2, 0x07);
		break;
	case DP_COLOR_DEPTH_12BIT:
		WRITE_BYTE_MASK(mtk_dp, REG_303C_DP_ENCODER0_P0 + 1 + reg_offset, 1, 0x07);
		break;
	case DP_COLOR_DEPTH_16BIT:
		WRITE_BYTE_MASK(mtk_dp, REG_303C_DP_ENCODER0_P0 + 1 + reg_offset, 0, 0x07);
		break;
	default:
		break;
	}
}

static void mtk_dp_pg_enable_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id, bool enable)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	if (enable)
		WRITE_BYTE_MASK(mtk_dp,
				REG_3038_DP_ENCODER0_P0 + 1 + reg_offset, BIT(3), BIT(3));
	else
		WRITE_BYTE_MASK(mtk_dp,
				REG_3038_DP_ENCODER0_P0 + 1 + reg_offset, 0, BIT(3));
}

static void mtk_dp_pg_pure_color_v2(struct mtk_dp *mtk_dp,
				    const enum dp_encoder_id encoder_id, u8 rgb, u32 color_depth)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	/* video select hw or pattern gen 0:HW 1:PG */
	WRITE_BYTE_MASK(mtk_dp, REG_3038_DP_ENCODER0_P0 + 1 + reg_offset, BIT(3), BIT(3));
	/* reg_pattern_sel */
	WRITE_BYTE_MASK(mtk_dp, REG_31B0_DP_ENCODER0_P0 + reg_offset, 0, GENMASK(6, 4));

	switch (rgb) {
	case DP_PG_PURECOLOR_BLUE:
		WRITE_2BYTE_MASK(mtk_dp, REG_317C_DP_ENCODER0_P0 + reg_offset,
				 0, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_3180_DP_ENCODER0_P0 + reg_offset,
				 0, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_3184_DP_ENCODER0_P0 + reg_offset,
				 color_depth, GENMASK(11, 0));
		break;
	case DP_PG_PURECOLOR_GREEN:
		WRITE_2BYTE_MASK(mtk_dp, REG_317C_DP_ENCODER0_P0 + reg_offset,
				 0, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_3180_DP_ENCODER0_P0 + reg_offset,
				 color_depth, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_3184_DP_ENCODER0_P0 + reg_offset,
				 0, GENMASK(11, 0));
		break;
	case DP_PG_PURECOLOR_RED:
		WRITE_2BYTE_MASK(mtk_dp, REG_317C_DP_ENCODER0_P0 + reg_offset,
				 color_depth, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_3180_DP_ENCODER0_P0 + reg_offset,
				 0, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_3184_DP_ENCODER0_P0 + reg_offset,
				 0, GENMASK(11, 0));
		break;
	default:
		WRITE_2BYTE_MASK(mtk_dp, REG_317C_DP_ENCODER0_P0 + reg_offset,
				 color_depth, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_3180_DP_ENCODER0_P0 + reg_offset,
				 0, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_3184_DP_ENCODER0_P0 + reg_offset,
				 0, GENMASK(11, 0));
		break;
	}
}

static void mtk_dp_pg_vertical_ramping_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id,
					  u8 rgb, u32 color_depth, u8 location)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	WRITE_BYTE_MASK(mtk_dp, REG_3038_DP_ENCODER0_P0 + 1 + reg_offset, BIT(3), BIT(3));
	WRITE_BYTE_MASK(mtk_dp, REG_31B0_DP_ENCODER0_P0 + reg_offset,
			1 << PGEN_PATTERN_SEL_DP_ENCODER0_P0_FLDMASK_POS,
			PGEN_PATTERN_SEL_DP_ENCODER0_P0_FLDMASK);
	WRITE_BYTE_MASK(mtk_dp, REG_31B0_DP_ENCODER0_P0 + reg_offset,
			1 << PGEN_PAT_DIRECTION_DP_ENCODER0_P0_FLDMASK_POS,
			PGEN_PAT_DIRECTION_DP_ENCODER0_P0_FLDMASK);
	WRITE_BYTE_MASK(mtk_dp, REG_31B0_DP_ENCODER0_P0 + reg_offset,
			rgb << PGEN_PAT_RGB_ENABLE_DP_ENCODER0_P0_FLDMASK_POS,
			PGEN_PAT_RGB_ENABLE_DP_ENCODER0_P0_FLDMASK);

	switch (location) {
	case DP_PG_LOCATION_ALL:
		WRITE_2BYTE_MASK(mtk_dp, REG_317C_DP_ENCODER0_P0 + reg_offset,
				 0, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_3180_DP_ENCODER0_P0 + reg_offset,
				 0, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_3184_DP_ENCODER0_P0 + reg_offset,
				 color_depth, GENMASK(11, 0));

		WRITE_2BYTE(mtk_dp, REG_31A0_DP_ENCODER0_P0 + reg_offset,
			    0x3FFF);
		break;
	case DP_PG_LOCATION_TOP:
		WRITE_2BYTE_MASK(mtk_dp, REG_317C_DP_ENCODER0_P0 + reg_offset,
				 0, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_3180_DP_ENCODER0_P0 + reg_offset,
				 color_depth, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_3184_DP_ENCODER0_P0 + reg_offset,
				 0, GENMASK(11, 0));
		WRITE_2BYTE(mtk_dp, REG_31A0_DP_ENCODER0_P0 + reg_offset, 0x40);
		break;

	case DP_PG_LOCATION_BOTTOM:
		WRITE_2BYTE_MASK(mtk_dp, REG_317C_DP_ENCODER0_P0 + reg_offset,
				 color_depth, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_3180_DP_ENCODER0_P0 + reg_offset,
				 0, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_3184_DP_ENCODER0_P0 + reg_offset,
				 0, GENMASK(11, 0));
		WRITE_2BYTE(mtk_dp, REG_31A0_DP_ENCODER0_P0 + reg_offset, 0x2fff);
		break;
	default:
		WRITE_2BYTE_MASK(mtk_dp, REG_317C_DP_ENCODER0_P0 + reg_offset,
				 0, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_3180_DP_ENCODER0_P0 + reg_offset,
				 0, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_3184_DP_ENCODER0_P0 + reg_offset,
				 color_depth, GENMASK(11, 0));
		WRITE_2BYTE(mtk_dp, REG_31A0_DP_ENCODER0_P0 + reg_offset, 0x3fff);
		break;
	}
}

static void mtk_dp_pg_horizontal_ramping_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id,
					    u8 rgb, u32 color_depth, u8 location)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);
	u64 ramp = 0x3FFF;

	/* video select hw or pattern gen 0:HW 1:PG */
	WRITE_BYTE_MASK(mtk_dp, REG_3038_DP_ENCODER0_P0 + 1 + reg_offset, BIT(3), BIT(3));
	WRITE_BYTE_MASK(mtk_dp, REG_31B0_DP_ENCODER0_P0 + reg_offset,
			2 << PGEN_PATTERN_SEL_DP_ENCODER0_P0_FLDMASK_POS,
			PGEN_PATTERN_SEL_DP_ENCODER0_P0_FLDMASK);
	WRITE_BYTE_MASK(mtk_dp, REG_31B0_DP_ENCODER0_P0 + reg_offset,
			1 << PGEN_PAT_DIRECTION_DP_ENCODER0_P0_FLDMASK_POS,
			PGEN_PAT_DIRECTION_DP_ENCODER0_P0_FLDMASK);
	WRITE_BYTE_MASK(mtk_dp, REG_31B0_DP_ENCODER0_P0 + reg_offset,
			rgb << PGEN_PAT_RGB_ENABLE_DP_ENCODER0_P0_FLDMASK_POS,
			PGEN_PAT_RGB_ENABLE_DP_ENCODER0_P0_FLDMASK);

	WRITE_2BYTE(mtk_dp, REG_31A0_DP_ENCODER0_P0 + reg_offset, ramp);

	switch (location) {
	case DP_PG_LOCATION_ALL:
		WRITE_2BYTE_MASK(mtk_dp, REG_317C_DP_ENCODER0_P0 + reg_offset,
				 0, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_3180_DP_ENCODER0_P0 + reg_offset,
				 0, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_3184_DP_ENCODER0_P0 + reg_offset,
				 0, GENMASK(11, 0));
		break;
	case DP_PG_LOCATION_LEFT_OF_TOP:
		WRITE_2BYTE_MASK(mtk_dp, REG_317C_DP_ENCODER0_P0 + reg_offset,
				 0, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_3180_DP_ENCODER0_P0 + reg_offset,
				 0, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_3184_DP_ENCODER0_P0 + reg_offset,
				 0, GENMASK(11, 0));
		WRITE_2BYTE(mtk_dp, REG_31A0_DP_ENCODER0_P0 + reg_offset, 0x3fff);
		break;
	case DP_PG_LOCATION_LEFT_OF_BOTTOM:
		WRITE_2BYTE_MASK(mtk_dp, REG_317C_DP_ENCODER0_P0 + reg_offset,
				 0, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_3180_DP_ENCODER0_P0 + reg_offset,
				 0, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_3184_DP_ENCODER0_P0 + reg_offset,
				 0, GENMASK(11, 0));
		WRITE_2BYTE(mtk_dp, REG_31A0_DP_ENCODER0_P0 + reg_offset, 0x3fff);
		break;
	default:
		break;
	}
}

static void mtk_dp_pg_vertical_color_bar_v2(struct mtk_dp *mtk_dp,
					    const enum dp_encoder_id encoder_id, u8 location)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	WRITE_BYTE_MASK(mtk_dp, REG_3038_DP_ENCODER0_P0 + 1 + reg_offset, BIT(3), BIT(3));
	WRITE_BYTE_MASK(mtk_dp, REG_31B0_DP_ENCODER0_P0 + reg_offset,
			3 << PGEN_PATTERN_SEL_DP_ENCODER0_P0_FLDMASK_POS,
			PGEN_PATTERN_SEL_DP_ENCODER0_P0_FLDMASK);

	switch (location) {
	case DP_PG_LOCATION_ALL:
		WRITE_BYTE_MASK(mtk_dp, REG_3190_DP_ENCODER0_P0 + reg_offset,
				0, GENMASK(5, 4));
		WRITE_BYTE_MASK(mtk_dp, REG_3190_DP_ENCODER0_P0 + reg_offset,
				0, GENMASK(2, 0));
		break;
	case DP_PG_LOCATION_LEFT:
		WRITE_BYTE_MASK(mtk_dp, REG_3190_DP_ENCODER0_P0 + reg_offset,
				BIT(4), GENMASK(5, 4));
		WRITE_BYTE_MASK(mtk_dp, REG_3190_DP_ENCODER0_P0 + reg_offset,
				0, GENMASK(2, 0));
		break;
	case DP_PG_LOCATION_RIGHT:
		WRITE_BYTE_MASK(mtk_dp, REG_3190_DP_ENCODER0_P0 + reg_offset,
				BIT(4), GENMASK(5, 4));
		WRITE_BYTE_MASK(mtk_dp, REG_3190_DP_ENCODER0_P0 + reg_offset,
				BIT(2), GENMASK(2, 0));
		break;
	case DP_PG_LOCATION_LEFT_OF_LEFT:
		WRITE_BYTE_MASK(mtk_dp, REG_3190_DP_ENCODER0_P0 + reg_offset,
				BIT(5) | BIT(4), GENMASK(5, 4));
		WRITE_BYTE_MASK(mtk_dp, REG_3190_DP_ENCODER0_P0 + reg_offset,
				0, GENMASK(2, 0));
		break;
	case DP_PG_LOCATION_RIGHT_OF_LEFT:
		WRITE_BYTE_MASK(mtk_dp, REG_3190_DP_ENCODER0_P0 + reg_offset,
				BIT(5) | BIT(4), GENMASK(5, 4));
		WRITE_BYTE_MASK(mtk_dp, REG_3190_DP_ENCODER0_P0 + reg_offset,
				BIT(1), GENMASK(2, 0));
		break;
	case DP_PG_LOCATION_LEFT_OF_RIGHT:
		WRITE_BYTE_MASK(mtk_dp, REG_3190_DP_ENCODER0_P0 + reg_offset,
				BIT(5) | BIT(4), GENMASK(5, 4));
		WRITE_BYTE_MASK(mtk_dp, REG_3190_DP_ENCODER0_P0 + reg_offset,
				BIT(2), GENMASK(2, 0));
		break;
	case DP_PG_LOCATION_RIGHT_OF_RIGHT:
		WRITE_BYTE_MASK(mtk_dp, REG_3190_DP_ENCODER0_P0 + reg_offset,
				BIT(5) | BIT(4), GENMASK(5, 4));
		WRITE_BYTE_MASK(mtk_dp, REG_3190_DP_ENCODER0_P0 + reg_offset,
				BIT(2) | BIT(1), GENMASK(2, 0));
		break;
	default:
		break;
	}
}

static void mtk_dp_pg_horizontal_color_bar_v2(struct mtk_dp *mtk_dp,
					      const enum dp_encoder_id encoder_id, u8 location)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	/* video select hw or pattern gen 0:HW 1:PG */
	WRITE_BYTE_MASK(mtk_dp, REG_3038_DP_ENCODER0_P0 + 1 + reg_offset, BIT(3), BIT(3));
	WRITE_BYTE_MASK(mtk_dp, REG_31B0_DP_ENCODER0_P0 + reg_offset,
			4 << PGEN_PATTERN_SEL_DP_ENCODER0_P0_FLDMASK_POS,
			PGEN_PATTERN_SEL_DP_ENCODER0_P0_FLDMASK);
	switch (location) {
	case DP_PG_LOCATION_ALL:
		WRITE_BYTE_MASK(mtk_dp, REG_3190_DP_ENCODER0_P0 + reg_offset,
				0, GENMASK(5, 4));
		WRITE_BYTE_MASK(mtk_dp, REG_3190_DP_ENCODER0_P0 + reg_offset,
				0, GENMASK(2, 0));
		break;
	case DP_PG_LOCATION_TOP:
		WRITE_BYTE_MASK(mtk_dp, REG_3190_DP_ENCODER0_P0 + reg_offset,
				BIT(4), GENMASK(5, 4));
		WRITE_BYTE_MASK(mtk_dp, REG_3190_DP_ENCODER0_P0 + reg_offset,
				0, GENMASK(2, 0));
		break;
	case DP_PG_LOCATION_BOTTOM:
		WRITE_BYTE_MASK(mtk_dp, REG_3190_DP_ENCODER0_P0 + reg_offset,
				BIT(4), GENMASK(5, 4));
		WRITE_BYTE_MASK(mtk_dp, REG_3190_DP_ENCODER0_P0 + reg_offset,
				BIT(2), GENMASK(2, 0));
		break;
	default:
		break;
	}
}

static void mtk_dp_pg_chessboard_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id, u8 location,
				    u16 hde, u16 vde)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	WRITE_BYTE_MASK(mtk_dp, REG_3038_DP_ENCODER0_P0 + 1 + reg_offset, BIT(3), BIT(3));
	WRITE_BYTE_MASK(mtk_dp, REG_31B0_DP_ENCODER0_P0 + reg_offset, BIT(6) | BIT(4),
			GENMASK(6, 4));

	if (location == DP_PG_LOCATION_ALL) {
		WRITE_2BYTE_MASK(mtk_dp, REG_317C_DP_ENCODER0_P0 + reg_offset,
				 0xfff, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_3180_DP_ENCODER0_P0 + reg_offset,
				 0xfff, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_3184_DP_ENCODER0_P0 + reg_offset,
				 0xfff, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_3194_DP_ENCODER0_P0 + reg_offset,
				 0, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_3198_DP_ENCODER0_P0 + reg_offset,
				 0, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_319C_DP_ENCODER0_P0 + reg_offset,
				 0, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_31A8_DP_ENCODER0_P0 + reg_offset,
				 (hde / 8), GENMASK(13, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_31AC_DP_ENCODER0_P0 + reg_offset,
				 (vde / 8), GENMASK(13, 0));
	}
}

static void mtk_dp_pg_sub_pixel_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id, u8 location)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	WRITE_BYTE_MASK(mtk_dp, REG_3038_DP_ENCODER0_P0 + 1 + reg_offset, BIT(3), BIT(3));
	WRITE_BYTE_MASK(mtk_dp, REG_31B0_DP_ENCODER0_P0 + reg_offset,
			BIT(6) | BIT(5), GENMASK(6, 4));

	switch (location) {
	case DP_PG_PIXEL_ODD_MASK:
		WRITE_BYTE_MASK(mtk_dp, REG_31B0_DP_ENCODER0_P0 + 1 + reg_offset, 0, BIT(5));
		break;
	case DP_PG_PIXEL_EVEN_MASK:
		WRITE_BYTE_MASK(mtk_dp, REG_31B0_DP_ENCODER0_P0 + 1 + reg_offset,
				BIT(5), BIT(5));
		break;
	default:
		break;
	}
}

static void mtk_dp_pg_frame_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id, u8 location,
			       u16 hde, u16 vde)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	WRITE_BYTE_MASK(mtk_dp, REG_3038_DP_ENCODER0_P0 + 1 + reg_offset, BIT(3), BIT(3));
	WRITE_BYTE_MASK(mtk_dp, REG_31B0_DP_ENCODER0_P0 + reg_offset, BIT(6) | BIT(5) | BIT(4),
			GENMASK(6, 4));

	if (location == DP_PG_PIXEL_ODD_MASK) {
		WRITE_BYTE_MASK(mtk_dp, REG_31B0_DP_ENCODER0_P0 + 1 + reg_offset,
				0, BIT(5));
		WRITE_2BYTE_MASK(mtk_dp, REG_317C_DP_ENCODER0_P0 + reg_offset,
				 0xfff, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_3180_DP_ENCODER0_P0 + reg_offset,
				 0xfff, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_3184_DP_ENCODER0_P0 + reg_offset,
				 0xfff, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_3194_DP_ENCODER0_P0 + reg_offset,
				 0xfff, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_3198_DP_ENCODER0_P0 + reg_offset,
				 0xfff, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_319C_DP_ENCODER0_P0 + reg_offset,
				 0, GENMASK(11, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_31A8_DP_ENCODER0_P0 + reg_offset,
				 ((hde / 8) - 12), GENMASK(13, 0));
		WRITE_2BYTE_MASK(mtk_dp, REG_31AC_DP_ENCODER0_P0 + reg_offset,
				 ((vde / 8) - 12), GENMASK(13, 0));
		WRITE_BYTE_MASK(mtk_dp, REG_31B4_DP_ENCODER0_P0 + reg_offset,
				0x0b, GENMASK(3, 0));
	}
}

static void mtk_dp_pg_type_sel_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id,
				  int pattern_type, u8 bgr, u32 color_depth, u8 location)
{
	u16 hde, vde;

	hde = mtk_dp->info[encoder_id].dp_output_timing.hde;
	vde = mtk_dp->info[encoder_id].dp_output_timing.vde;

	switch (pattern_type) {
	case DP_PG_PURE_COLOR:
		mtk_dp_pg_pure_color_v2(mtk_dp, encoder_id, bgr, color_depth);
		break;

	case DP_PG_VERTICAL_RAMPING:
		mtk_dp_pg_vertical_ramping_v2(mtk_dp, encoder_id, bgr, color_depth, location);
		break;

	case DP_PG_HORIZONTAL_RAMPING:
		mtk_dp_pg_horizontal_ramping_v2(mtk_dp, encoder_id, bgr,
					        color_depth, location);
		break;

	case DP_PG_VERTICAL_COLOR_BAR:
		mtk_dp_pg_vertical_color_bar_v2(mtk_dp, encoder_id, location);
		break;

	case DP_PG_HORIZONTAL_COLOR_BAR:
		mtk_dp_pg_horizontal_color_bar_v2(mtk_dp, encoder_id, location);
		break;

	case DP_PG_CHESSBOARD_PATTERN:
		mtk_dp_pg_chessboard_v2(mtk_dp, encoder_id, location, hde, vde);
		break;

	case DP_PG_SUB_PIXEL_PATTERN:
		mtk_dp_pg_sub_pixel_v2(mtk_dp, encoder_id, location);
		break;

	case DP_PG_FRAME_PATTERN:
		mtk_dp_pg_frame_v2(mtk_dp, encoder_id, location, hde, vde);
		break;

	default:
		break;
	}
}

static void mtk_dp_spkg_asp_hb32_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id,
				    u8 enable, u8 HB3, u8 HB2)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	WRITE_2BYTE_MASK(mtk_dp, REG_30BC_DP_ENCODER0_P0 + reg_offset,
			 (enable ? 0x01 : 0x00) << ASP_HB23_SEL_DP_ENCODER0_P0_FLDMASK_POS,
			 ASP_HB23_SEL_DP_ENCODER0_P0_FLDMASK);
	WRITE_2BYTE_MASK(mtk_dp, REG_312C_DP_ENCODER0_P0 + reg_offset,
			 HB2 << ASP_HB2_DP_ENCODER0_P0_FLDMASK_POS,
			 ASP_HB2_DP_ENCODER0_P0_FLDMASK);
	WRITE_2BYTE_MASK(mtk_dp, REG_312C_DP_ENCODER0_P0 + reg_offset,
			 HB3 << ASP_HB3_DP_ENCODER0_P0_FLDMASK_POS,
			 ASP_HB3_DP_ENCODER0_P0_FLDMASK);
}

static void mtk_dp_spkg_sdp_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id,
			       bool enable,
			       u8 sdp_type,
			       u8 *hb,
			       u8 *db)
{
	u8  offset;
	u16 st_offset;
	u8  hb_offset;
	u8  reg_index;
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	if (enable) {
		for (offset = 0; offset < 0x10; offset++)
			for (reg_index = 0; reg_index < 2; reg_index++) {
				u32 addr = REG_3200_DP_ENCODER1_P0
					   + offset * 4 + reg_index + reg_offset;

				WRITE_BYTE(mtk_dp, addr, (db[offset * 2 + reg_index]));
				drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] SDP address:%u, data:%d\n",
				            addr, db[offset * 2 + reg_index]);
			}

		if (sdp_type == DP_SDP_PKG_DRM) {
			for (hb_offset = 0; hb_offset < 4 / 2; hb_offset++)
				for (reg_index = 0; reg_index < 2; reg_index++) {
					u32 addr = REG_3138_DP_ENCODER0_P0
					+ hb_offset * 4 + reg_index + reg_offset;
					u8 offset = hb_offset * 2 + reg_index;

					WRITE_BYTE(mtk_dp, addr, (hb[offset]));
					drm_dbg_kms(mtk_dp->drm_dev,
						    "[DPTX] W Reg addr:%x, index:%d\n", addr, offset);
				}
		} else if (sdp_type >= DP_SDP_PKG_PPS0 &&
			   sdp_type <= DP_SDP_PKG_PPS3) {
			for (hb_offset = 0; hb_offset < (4 / 2); hb_offset++)
				for (reg_index = 0; reg_index < 2; reg_index++) {
					u32 addr = REG_3130_DP_ENCODER0_P0
					+ hb_offset * 4 + reg_index + reg_offset;
					u8 offset = hb_offset * 2 + reg_index;

					WRITE_BYTE(mtk_dp, addr, hb[offset]);
					drm_dbg_kms(mtk_dp->drm_dev,
						    "[DPTX] W H1 Reg addr:%x, index:%d\n", addr, offset);
				}
		} else if (sdp_type == DP_SDP_PKG_ADS) {
			for (hb_offset = 0; hb_offset < (4 >> 1); hb_offset++)
				for (reg_index = 0; reg_index < 2; reg_index++) {
					u32 addr = REG_31F0_DP_ENCODER0_P0 + reg_offset
									+ hb_offset * 8 + reg_index;
					u8 offset = hb_offset * 2 + reg_index;

					WRITE_BYTE(mtk_dp, addr, hb[offset]);
				}
		} else {
			st_offset = (sdp_type - DP_SDP_PKG_ACM) * 8;

			for (hb_offset = 0; hb_offset < 4 / 2; hb_offset++)
				for (reg_index = 0; reg_index < 2; reg_index++) {
					u32 addr = REG_30D8_DP_ENCODER0_P0
					+ st_offset
					+ hb_offset * 4 + reg_index + reg_offset;
					u8 offset = hb_offset * 2 + reg_index;

					WRITE_BYTE(mtk_dp, addr, hb[offset]);
					drm_dbg_kms(mtk_dp->drm_dev,
						    "[DPTX] W H2 Reg addr:%x, index:%d\n", addr, offset);
				}
		}
	}

	switch (sdp_type) {
	case DP_SDP_PKG_NONE:
		break;

	case DP_SDP_PKG_ACM:
		WRITE_BYTE(mtk_dp, REG_30B4_DP_ENCODER0_P0 + reg_offset, 0x00);

		if (enable) {
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					DP_SDP_PKG_ACM,
					GENMASK(4, 0));
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					BIT(5), BIT(5));
			WRITE_BYTE(mtk_dp, REG_30B4_DP_ENCODER0_P0 + reg_offset, 0x05);
			drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] SENT SDP TYPE ACM\n");
		}

		break;

	case DP_SDP_PKG_ISRC:
		WRITE_BYTE(mtk_dp, (REG_30B4_DP_ENCODER0_P0 + 1 + reg_offset), 0x00);

		if (enable) {
			WRITE_BYTE(mtk_dp, (REG_31EC_DP_ENCODER0_P0 + 1 + reg_offset), 0x1C);
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					DP_SDP_PKG_ISRC,
					GENMASK(4, 0));

			if (hb[3] & BIT(2))
				WRITE_BYTE_MASK(mtk_dp, REG_30BC_DP_ENCODER0_P0 + reg_offset,
						BIT(0), BIT(0));
			else
				WRITE_BYTE_MASK(mtk_dp, REG_30BC_DP_ENCODER0_P0 + reg_offset,
						0, BIT(0));

			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					BIT(5), BIT(5));
			WRITE_BYTE(mtk_dp, (REG_30B4_DP_ENCODER0_P0 + 1 + reg_offset), 0x05);
			drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] SENT SDP TYPE ISRC\n");
		}

		break;

	case DP_SDP_PKG_AVI:
		WRITE_BYTE(mtk_dp, (REG_30A4_DP_ENCODER0_P0 + 1 + reg_offset), 0x00);

		if (enable) {
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					DP_SDP_PKG_AVI,
					GENMASK(4, 0));
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					BIT(5), BIT(5));
			WRITE_BYTE(mtk_dp, (REG_30A4_DP_ENCODER0_P0 + 1 + reg_offset), 0x05);
			drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] SENT SDP TYPE AVI\n");
		}

		break;

	case DP_SDP_PKG_AUI:
		WRITE_BYTE(mtk_dp, REG_30A8_DP_ENCODER0_P0 + reg_offset, 0x00);

		if (enable) {
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					DP_SDP_PKG_AUI,
					GENMASK(4, 0));
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					BIT(5), BIT(5));
			WRITE_BYTE(mtk_dp, REG_30A8_DP_ENCODER0_P0 + reg_offset, 0x05);
			drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] SENT SDP TYPE AUI\n");
		}

		break;

	case DP_SDP_PKG_SPD:
		WRITE_BYTE(mtk_dp, (REG_30A8_DP_ENCODER0_P0 + 1 + reg_offset), 0x00);

		if (enable) {
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					DP_SDP_PKG_SPD,
					GENMASK(4, 0));
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					BIT(5), BIT(5));
			WRITE_BYTE(mtk_dp, REG_30A8_DP_ENCODER0_P0 + 1 + reg_offset, 0x05);
			drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] SENT SDP TYPE SPD\n");
		}

		break;

	case DP_SDP_PKG_MPEG:
		WRITE_BYTE(mtk_dp, REG_30AC_DP_ENCODER0_P0 + reg_offset, 0x00);

		if (enable) {
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					DP_SDP_PKG_MPEG,
					GENMASK(4, 0));
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					BIT(5), BIT(5));
			WRITE_BYTE(mtk_dp, REG_30AC_DP_ENCODER0_P0 + reg_offset, 0x05);
			drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] SENT SDP TYPE MPEG\n");
		}

		break;

	case DP_SDP_PKG_NTSC:
		WRITE_BYTE(mtk_dp, (REG_30AC_DP_ENCODER0_P0 + 1 + reg_offset), 0x00);

		if (enable) {
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					DP_SDP_PKG_NTSC,
					GENMASK(4, 0));
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					BIT(5), BIT(5));
			WRITE_BYTE(mtk_dp, (REG_30AC_DP_ENCODER0_P0 + 1 + reg_offset), 0x05);
			drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] SENT SDP TYPE NTSC\n");
		}

		break;

	case DP_SDP_PKG_VSP:
		WRITE_BYTE(mtk_dp, REG_30B0_DP_ENCODER0_P0 + reg_offset, 0x00);

		if (enable) {
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					DP_SDP_PKG_VSP,
					GENMASK(4, 0));
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					BIT(5), BIT(5));
			WRITE_BYTE(mtk_dp, REG_30B0_DP_ENCODER0_P0 + reg_offset, 0x05);
			drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] SENT SDP TYPE VSP\n");
		}

		break;

	case DP_SDP_PKG_VSC:
		WRITE_BYTE(mtk_dp, REG_30B8_DP_ENCODER0_P0 + reg_offset, 0x00);

		if (enable) {
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					DP_SDP_PKG_VSC,
					GENMASK(4, 0));
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					BIT(5), BIT(5));
			WRITE_BYTE(mtk_dp, REG_30B8_DP_ENCODER0_P0 + reg_offset, 0x05);
			drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] SENT SDP TYPE VSC\n");
		}

		break;

	case DP_SDP_PKG_EXT:
		WRITE_BYTE(mtk_dp, (REG_30B0_DP_ENCODER0_P0 + 1 + reg_offset), 0x00);

		if (enable) {
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					DP_SDP_PKG_EXT,
					GENMASK(4, 0));
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					BIT(5), BIT(5));
			WRITE_BYTE(mtk_dp, REG_30B0_DP_ENCODER0_P0 + 1 + reg_offset, 0x05);
			drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] SENT SDP TYPE EXT\n");
		}

		break;

	case DP_SDP_PKG_PPS0:
		WRITE_BYTE(mtk_dp, REG_31E8_DP_ENCODER0_P0 + reg_offset, 0x00);

		if (enable) {
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					DP_SDP_PKG_PPS0,
					GENMASK(4, 0));
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					BIT(5), BIT(5));
			WRITE_BYTE(mtk_dp, REG_31E8_DP_ENCODER0_P0 + reg_offset, 0x05);
			drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] SENT SDP TYPE PPS0\n");
		}

		break;

	case DP_SDP_PKG_PPS1:
		WRITE_BYTE(mtk_dp, REG_31E8_DP_ENCODER0_P0 + reg_offset, 0x00);

		if (enable) {
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					DP_SDP_PKG_PPS1,
					GENMASK(4, 0));
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					BIT(5), BIT(5));
			WRITE_BYTE(mtk_dp, REG_31E8_DP_ENCODER0_P0 + reg_offset, 0x05);
			drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] SENT SDP TYPE PPS1\n");
		}

		break;

	case DP_SDP_PKG_PPS2:
		WRITE_BYTE(mtk_dp, REG_31E8_DP_ENCODER0_P0 + reg_offset, 0x00);

		if (enable) {
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					DP_SDP_PKG_PPS2,
					GENMASK(4, 0));
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					BIT(5), BIT(5));
			WRITE_BYTE(mtk_dp, REG_31E8_DP_ENCODER0_P0 + reg_offset, 0x05);
			drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] SENT SDP TYPE PPS2\n");
		}

		break;

	case DP_SDP_PKG_PPS3:
		WRITE_BYTE(mtk_dp, REG_31E8_DP_ENCODER0_P0 + reg_offset, 0x00);

		if (enable) {
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					DP_SDP_PKG_PPS3,
					GENMASK(4, 0));
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					BIT(5), BIT(5));
			WRITE_BYTE(mtk_dp, REG_31E8_DP_ENCODER0_P0 + reg_offset, 0x05);
			drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] SENT SDP TYPE PPS3\n");
		}

		break;

	case DP_SDP_PKG_DRM:
		WRITE_BYTE(mtk_dp, REG_31DC_DP_ENCODER0_P0 + reg_offset, 0x00);

		if (enable) {
			WRITE_BYTE(mtk_dp, REG_3138_DP_ENCODER0_P0 + reg_offset, hb[0]);
			WRITE_BYTE(mtk_dp, (REG_3138_DP_ENCODER0_P0 + 1 + reg_offset), hb[1]);
			WRITE_BYTE(mtk_dp, REG_313C_DP_ENCODER0_P0 + reg_offset, hb[2]);
			WRITE_BYTE(mtk_dp, REG_313C_DP_ENCODER0_P0 + 1 + reg_offset, hb[3]);
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					DP_SDP_PKG_DRM,
					GENMASK(4, 0));
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					BIT(5), BIT(5));
			WRITE_BYTE(mtk_dp, REG_31DC_DP_ENCODER0_P0 + reg_offset, 0x05);
			drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] SENT SDP TYPE DRM\n");
		}

		break;

	case DP_SDP_PKG_ADS:
		/* adaptive sync SDP transmit disable */
		WRITE_BYTE_MASK(mtk_dp, REG_31EC_DP_ENCODER0_P0 + reg_offset, 0,
				ADS_CFG_DP_ENCODER0_P0_FLDMASK);
		if (enable) {
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					DP_SDP_PKG_ADS,
					SDP_PACKET_TYPE_DP_ENCODER1_P0_FLDMASK);
			/* write sdp data trigger */
			WRITE_BYTE_MASK(mtk_dp, REG_3280_DP_ENCODER1_P0 + reg_offset,
					1 << SDP_PACKET_W_DP_ENCODER1_P0_FLDMASK_POS,
					SDP_PACKET_W_DP_ENCODER1_P0_FLDMASK);
			/* adaptive sync SDP transmit enable */
			WRITE_BYTE_MASK(mtk_dp, REG_31EC_DP_ENCODER0_P0 + reg_offset,
					1 << ADS_CFG_DP_ENCODER0_P0_FLDMASK_POS,
					ADS_CFG_DP_ENCODER0_P0_FLDMASK);
			drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] SENT SDP TYPE ADS\n");
		}

		break;

	default:
		break;
	}
}

static void mtk_dp_spkg_vsc_ext_vesa_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id,
					bool enable,
					u8 hdr_num,
					u8 *db)
{
	u8  vsc_hb1 = 0x20;	/* VESA : 0x20; CEA : 0x21 */
	u8  vsc_hb2;
	u8  pkg_cnt;
	u8  loop;
	u8  offset;
	u8  reg_index;
	u16 sdp_offset;
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	if (!enable) {
		WRITE_BYTE_MASK(mtk_dp, (REG_30A0_DP_ENCODER0_P0 + 1 + reg_offset), 0, BIT(0));
		WRITE_BYTE_MASK(mtk_dp, REG_328C_DP_ENCODER1_P0 + reg_offset, 0, BIT(7));
		return;
	}

	vsc_hb2 = (hdr_num > 0) ? BIT(6) : 0x00;

	WRITE_BYTE(mtk_dp, REG_31C8_DP_ENCODER0_P0 + reg_offset, 0x00);
	WRITE_BYTE(mtk_dp, (REG_31C8_DP_ENCODER0_P0 + 1 + reg_offset), vsc_hb1);
	WRITE_BYTE(mtk_dp, REG_31CC_DP_ENCODER0_P0 + reg_offset, vsc_hb2);
	WRITE_BYTE(mtk_dp, (REG_31CC_DP_ENCODER0_P0 + 1 + reg_offset), 0x00);
	WRITE_BYTE(mtk_dp, REG_31D8_DP_ENCODER0_P0 + reg_offset, hdr_num);

	WRITE_BYTE_MASK(mtk_dp, REG_328C_DP_ENCODER1_P0 + reg_offset, BIT(0), BIT(0));
	WRITE_BYTE_MASK(mtk_dp, REG_328C_DP_ENCODER1_P0 + reg_offset, BIT(2), BIT(2));

	usleep_range(50, 51);
	WRITE_BYTE_MASK(mtk_dp, REG_328C_DP_ENCODER1_P0 + reg_offset, 0, BIT(2));
	usleep_range(50, 51);

	for (pkg_cnt = 0; pkg_cnt < (hdr_num + 1); pkg_cnt++) {
		sdp_offset = 0;

		for (loop = 0; loop < 4; loop++) {
			for (offset = 0; offset < 8 / 2; offset++) {
				for (reg_index = 0; reg_index < 2; reg_index++) {
					u32 addr = REG_3290_DP_ENCODER1_P0
					+ offset * 4 + reg_index + reg_offset;
					u8 tmp = sdp_offset
							+ offset * 2 + reg_index;

					WRITE_BYTE(mtk_dp, addr, db[tmp]);
				}
			}

			WRITE_BYTE_MASK(mtk_dp, REG_328C_DP_ENCODER1_P0 + reg_offset,
					BIT(6), BIT(6));
			sdp_offset += 8;
		}
	}

	WRITE_BYTE_MASK(mtk_dp, (REG_30A0_DP_ENCODER0_P0 + 1 + reg_offset), BIT(0), BIT(0));
	WRITE_BYTE_MASK(mtk_dp, REG_328C_DP_ENCODER1_P0 + reg_offset, BIT(7), BIT(7));
}

void mtk_dp_audio_mute_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id, bool enable)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] [%d] enable:%d\n", encoder_id, enable);

	mtk_dp->info[encoder_id].audio_mute = enable;

	if (enable) {
		WRITE_2BYTE_MASK(mtk_dp, REG_3030_DP_ENCODER0_P0 + reg_offset,
				 VBID_AUDIO_MUTE_FLAG_SW_DP_ENCODER0_P0_FLDMASK,
				 VBID_AUDIO_MUTE_FLAG_SW_DP_ENCODER0_P0_FLDMASK);

		WRITE_2BYTE_MASK(mtk_dp, REG_3030_DP_ENCODER0_P0 + reg_offset,
				 VBID_AUDIO_MUTE_FLAG_SEL_DP_ENCODER0_P0_FLDMASK,
				 VBID_AUDIO_MUTE_FLAG_SEL_DP_ENCODER0_P0_FLDMASK);

		WRITE_BYTE_MASK(mtk_dp, REG_3088_DP_ENCODER0_P0 + reg_offset,
				0x0, AU_EN_DP_ENCODER0_P0_FLDMASK);
		WRITE_BYTE(mtk_dp, REG_30A4_DP_ENCODER0_P0 + reg_offset, 0x00);

		WRITE_2BYTE_MASK(mtk_dp, REG_33F0_DP_ENCODER1_P0 + reg_offset, BIT(9), BIT(9));
		WRITE_2BYTE_MASK(mtk_dp, REG_33F0_DP_ENCODER1_P0 + reg_offset, 0x0, BIT(9));
	} else {
		WRITE_2BYTE_MASK(mtk_dp, REG_3030_DP_ENCODER0_P0 + reg_offset, (0x00),
				 VBID_AUDIO_MUTE_FLAG_SEL_DP_ENCODER0_P0_FLDMASK);

		WRITE_BYTE_MASK(mtk_dp, REG_3088_DP_ENCODER0_P0 + reg_offset,
				AU_EN_DP_ENCODER0_P0_FLDMASK,
				AU_EN_DP_ENCODER0_P0_FLDMASK);

		WRITE_BYTE(mtk_dp, REG_30A4_DP_ENCODER0_P0 + reg_offset,
			   (1 << AUDIO_TS_SEND_EN_DP_ENCODER0_P0_FLDMASK_POS) |
			   (AUDIO_TS_SEND_FREQ_TYPE_ONE_IN_EVERY_N_PLUS_1_FRAMES
			   << AUDIO_TS_SEND_FREQ_TYPE_DP_ENCODER0_P0_FLDMASK_POS));
	}
}

static void mtk_dp_audio_sample_arrange_v2(struct mtk_dp *mtk_dp,
				 const enum dp_encoder_id encoder_id, u8 enable)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);
	u32 value = 0;
	u64 tmp = 0;

	tmp = mtk_dp->info[encoder_id].dp_output_timing.htt -
				mtk_dp->info[encoder_id].dp_output_timing.hde;

	value = div_u64(tmp * mtk_dp->training_info.link_rate * 27 * 200,
			mtk_dp->info[encoder_id].dp_output_timing.pixcel_rate);

	if (enable) {
		WRITE_4BYTE_MASK(mtk_dp, REG_3374_DP_ENCODER1_P0 + reg_offset,
				 BIT(12), BIT(12));
		WRITE_4BYTE_MASK(mtk_dp, REG_3374_DP_ENCODER1_P0 + reg_offset,
				 (u16)value, GENMASK(11, 0));
	} else {
		WRITE_4BYTE_MASK(mtk_dp, REG_3374_DP_ENCODER1_P0 + reg_offset, 0, BIT(12));
		WRITE_4BYTE_MASK(mtk_dp, REG_3374_DP_ENCODER1_P0 + reg_offset,
				 0, GENMASK(11, 0));
	}

	drm_dbg_kms(mtk_dp->drm_dev,
		    "[DPTX] Encoder %d, Audio arrange patch enable = %d, value = 0x%x\n",
		    encoder_id, enable, value);
}

static void mtk_dp_audio_sdp_setting_v2(struct mtk_dp *mtk_dp,
			      const enum dp_encoder_id encoder_id, u8 channel)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	WRITE_BYTE_MASK(mtk_dp, REG_312C_DP_ENCODER0_P0 + reg_offset, 0x00, GENMASK(7, 0));

	if (channel == 8)
		WRITE_2BYTE_MASK(mtk_dp, REG_312C_DP_ENCODER0_P0 + reg_offset, 0x0700, GENMASK(15, 8));
	else
		WRITE_2BYTE_MASK(mtk_dp, REG_312C_DP_ENCODER0_P0 + reg_offset, 0x0100, GENMASK(15, 8));
}

static void mtk_dp_audio_set_mdiv_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id, u8 div)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	WRITE_2BYTE_MASK(mtk_dp, REG_30BC_DP_ENCODER0_P0 + reg_offset,
			 (div << AUDIO_M_CODE_MULT_DIV_SEL_DP_ENCODER0_P0_FLDMASK_POS),
			 AUDIO_M_CODE_MULT_DIV_SEL_DP_ENCODER0_P0_FLDMASK);
}

void mtk_dp_audio_pg_enable_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id,
			       int channel, int fs, u8 enable)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	WRITE_2BYTE_MASK(mtk_dp, REG_3320_DP_ENCODER1_P0 + reg_offset,
			 0x3F << AUDIO_PATTERN_GEN_DSTB_CNT_THRD_DP_ENCODER1_P0_FLDMASK_POS,
	AUDIO_PATTERN_GEN_DSTB_CNT_THRD_DP_ENCODER1_P0_FLDMASK);

	WRITE_2BYTE_MASK(mtk_dp, REG_307C_DP_ENCODER0_P0 + reg_offset, 0,
			 HBLANK_SPACE_FOR_SDP_HW_EN_DP_ENCODER0_P0_FLDMASK);

	WRITE_BYTE_MASK(mtk_dp, REG_33F4_DP_ENCODER1_P0 + reg_offset, BIT(4), BIT(4));

	if (enable) {
		WRITE_2BYTE_MASK(mtk_dp, REG_3324_DP_ENCODER1_P0 + reg_offset,
				 0x3 << AUDIO_SOURCE_MUX_DP_ENCODER1_P0_FLDMASK_POS,
				 AUDIO_SOURCE_MUX_DP_ENCODER1_P0_FLDMASK);
		WRITE_2BYTE_MASK(mtk_dp, REG_331C_DP_ENCODER1_P0 + reg_offset,
				 0x0, TDM_AUDIO_DATA_EN_DP_ENCODER1_P0_FLDMASK);
		WRITE_BYTE_MASK(mtk_dp, REG_33F4_DP_ENCODER1_P0 + reg_offset, 0, BIT(0));
	} else {
		WRITE_2BYTE_MASK(mtk_dp, REG_3324_DP_ENCODER1_P0 + reg_offset,
				 0x4 << AUDIO_SOURCE_MUX_DP_ENCODER1_P0_FLDMASK_POS,
				 AUDIO_SOURCE_MUX_DP_ENCODER1_P0_FLDMASK);

		WRITE_2BYTE_MASK(mtk_dp, REG_331C_DP_ENCODER1_P0 + reg_offset,
				 TDM_AUDIO_DATA_EN_DP_ENCODER1_P0_FLDMASK,
				 TDM_AUDIO_DATA_EN_DP_ENCODER1_P0_FLDMASK);

		WRITE_2BYTE_MASK(mtk_dp, REG_331C_DP_ENCODER1_P0 + reg_offset,
				 (0x1f << TDM_AUDIO_DATA_BIT_DP_ENCODER1_P0_FLDMASK_POS),
				 TDM_AUDIO_DATA_BIT_DP_ENCODER1_P0_FLDMASK);
		WRITE_BYTE_MASK(mtk_dp, REG_33F4_DP_ENCODER1_P0 + reg_offset, BIT(0), BIT(0));
	}

	drm_dbg_kms(mtk_dp->drm_dev,
		    "[DPTX] encoder_id = %d, fs = %d, ch = %d\n", encoder_id, fs, channel);

	WRITE_BYTE_MASK(mtk_dp, REG_33F0_DP_ENCODER1_P0 + 1 + reg_offset, BIT(1), BIT(1));

	WRITE_2BYTE_MASK(mtk_dp, REG_3304_DP_ENCODER1_P0 + reg_offset,
			 AU_PRTY_REGEN_DP_ENCODER1_P0_FLDMASK,
			 AU_PRTY_REGEN_DP_ENCODER1_P0_FLDMASK);
	WRITE_2BYTE_MASK(mtk_dp, REG_3304_DP_ENCODER1_P0 + reg_offset,
			 AU_CH_STS_REGEN_DP_ENCODER1_P0_FLDMASK,
			 AU_CH_STS_REGEN_DP_ENCODER1_P0_FLDMASK);

	WRITE_2BYTE_MASK(mtk_dp, REG_3304_DP_ENCODER1_P0 + reg_offset,
			 0x1000, 0x1000);

	WRITE_2BYTE_MASK(mtk_dp, REG_3088_DP_ENCODER0_P0 + reg_offset,
			 AUDIO_2CH_SEL_DP_ENCODER0_P0_FLDMASK,
			 AUDIO_2CH_SEL_DP_ENCODER0_P0_FLDMASK);
	WRITE_2BYTE_MASK(mtk_dp, REG_3088_DP_ENCODER0_P0 + reg_offset,
			 AUDIO_MN_GEN_EN_DP_ENCODER0_P0_FLDMASK,
			 AUDIO_MN_GEN_EN_DP_ENCODER0_P0_FLDMASK);
	WRITE_2BYTE_MASK(mtk_dp, REG_3088_DP_ENCODER0_P0 + reg_offset,
			 AUDIO_8CH_SEL_DP_ENCODER0_P0_FLDMASK,
			 AUDIO_8CH_SEL_DP_ENCODER0_P0_FLDMASK);
	WRITE_2BYTE_MASK(mtk_dp, REG_3040_DP_ENCODER0_P0 + reg_offset,
			 AUDIO_16CH_SEL_DP_ENCODER0_P0_FLDMASK,
			 AUDIO_16CH_SEL_DP_ENCODER0_P0_FLDMASK);
	WRITE_2BYTE_MASK(mtk_dp, REG_3040_DP_ENCODER0_P0 + reg_offset,
			 AUDIO_32CH_SEL_DP_ENCODER0_P0_FLDMASK,
			 AUDIO_32CH_SEL_DP_ENCODER0_P0_FLDMASK);

	switch (fs) {
	case 44100:
	default:
		WRITE_2BYTE_MASK(mtk_dp, REG_3324_DP_ENCODER1_P0 + reg_offset,
				 (0x0 << AUDIO_PATTERN_GEN_FS_SEL_DP_ENCODER1_P0_FLDMASK_POS),
				 AUDIO_PATTERN_GEN_FS_SEL_DP_ENCODER1_P0_FLDMASK);
		break;

	case 48000:
		WRITE_2BYTE_MASK(mtk_dp, REG_3324_DP_ENCODER1_P0 + reg_offset,
				 (0x1 << AUDIO_PATTERN_GEN_FS_SEL_DP_ENCODER1_P0_FLDMASK_POS),
				 AUDIO_PATTERN_GEN_FS_SEL_DP_ENCODER1_P0_FLDMASK);
		break;

	case 192000:
		WRITE_2BYTE_MASK(mtk_dp, REG_3324_DP_ENCODER1_P0 + reg_offset,
				 (0x2 << AUDIO_PATTERN_GEN_FS_SEL_DP_ENCODER1_P0_FLDMASK_POS),
				 AUDIO_PATTERN_GEN_FS_SEL_DP_ENCODER1_P0_FLDMASK);
		break;
	}

	WRITE_2BYTE_MASK(mtk_dp, REG_3088_DP_ENCODER0_P0 + reg_offset, 0,
			 AUDIO_2CH_EN_DP_ENCODER0_P0_FLDMASK);
	WRITE_2BYTE_MASK(mtk_dp, REG_3088_DP_ENCODER0_P0 + reg_offset, 0,
			 AUDIO_8CH_EN_DP_ENCODER0_P0_FLDMASK);
	WRITE_2BYTE_MASK(mtk_dp, REG_3040_DP_ENCODER0_P0 + reg_offset, 0,
			 AUDIO_16CH_EN_DP_ENCODER0_P0_FLDMASK);
	WRITE_2BYTE_MASK(mtk_dp, REG_3040_DP_ENCODER0_P0 + reg_offset, 0,
			 AUDIO_32CH_EN_DP_ENCODER0_P0_FLDMASK);

	switch (channel) {
	case 2:
	default:
		WRITE_2BYTE_MASK(mtk_dp, REG_3324_DP_ENCODER1_P0 + reg_offset,
				 (0x0 << AUDIO_PATTERN_GEN_CH_NUM_DP_ENCODER1_P0_FLDMASK_POS),
				 AUDIO_PATTERN_GEN_CH_NUM_DP_ENCODER1_P0_FLDMASK);
		WRITE_2BYTE_MASK(mtk_dp, REG_3088_DP_ENCODER0_P0 + reg_offset,
				 (0x1 << AUDIO_2CH_EN_DP_ENCODER0_P0_FLDMASK_POS),
				 AUDIO_2CH_EN_DP_ENCODER0_P0_FLDMASK);

		WRITE_2BYTE_MASK(mtk_dp, REG_331C_DP_ENCODER1_P0 + reg_offset,
				 (0x1 << TDM_AUDIO_DATA_CH_NUM_DP_ENCODER1_P0_FLDMASK_POS),
				 TDM_AUDIO_DATA_CH_NUM_DP_ENCODER1_P0_FLDMASK);
		mtk_dp_spkg_asp_hb32_v2(mtk_dp, encoder_id, TRUE, DP_SDP_ASP_HB3_AU02CH, 0x0);
		break;

	case 8:
		WRITE_2BYTE_MASK(mtk_dp, REG_3324_DP_ENCODER1_P0 + reg_offset,
				 (0x1 << AUDIO_PATTERN_GEN_CH_NUM_DP_ENCODER1_P0_FLDMASK_POS),
				 AUDIO_PATTERN_GEN_CH_NUM_DP_ENCODER1_P0_FLDMASK);
		WRITE_2BYTE_MASK(mtk_dp, REG_3088_DP_ENCODER0_P0 + reg_offset,
				 (0x1 << AUDIO_8CH_EN_DP_ENCODER0_P0_FLDMASK_POS),
				 AUDIO_8CH_EN_DP_ENCODER0_P0_FLDMASK);

		WRITE_2BYTE_MASK(mtk_dp, REG_331C_DP_ENCODER1_P0 + reg_offset,
				 (0x7 << TDM_AUDIO_DATA_CH_NUM_DP_ENCODER1_P0_FLDMASK_POS),
				 TDM_AUDIO_DATA_CH_NUM_DP_ENCODER1_P0_FLDMASK);
		mtk_dp_spkg_asp_hb32_v2(mtk_dp, encoder_id, TRUE, DP_SDP_ASP_HB3_AU08CH, 0x0);
		break;

	case 16:
		WRITE_2BYTE_MASK(mtk_dp, REG_3324_DP_ENCODER1_P0 + reg_offset,
				 (0x2 << AUDIO_PATTERN_GEN_CH_NUM_DP_ENCODER1_P0_FLDMASK_POS),
				 AUDIO_PATTERN_GEN_CH_NUM_DP_ENCODER1_P0_FLDMASK);
		WRITE_2BYTE_MASK(mtk_dp, REG_3040_DP_ENCODER0_P0 + reg_offset,
				 (0x1 << AUDIO_16CH_EN_DP_ENCODER0_P0_FLDMASK_POS),
				 AUDIO_16CH_EN_DP_ENCODER0_P0_FLDMASK);
		break;

	case 32:
		WRITE_2BYTE_MASK(mtk_dp, REG_3324_DP_ENCODER1_P0 + reg_offset,
				 (0x3 << AUDIO_PATTERN_GEN_CH_NUM_DP_ENCODER1_P0_FLDMASK_POS),
				 AUDIO_PATTERN_GEN_CH_NUM_DP_ENCODER1_P0_FLDMASK);
		WRITE_2BYTE_MASK(mtk_dp, REG_3040_DP_ENCODER0_P0 + reg_offset,
				 (0x1 << AUDIO_32CH_EN_DP_ENCODER0_P0_FLDMASK_POS),
				 AUDIO_32CH_EN_DP_ENCODER0_P0_FLDMASK);
		break;
	}

	/* TDM to DPTX reset [1] */
	WRITE_BYTE_MASK(mtk_dp, REG_331C_DP_ENCODER1_P0 + reg_offset,
			TDM_AUDIO_RST_DP_ENCODER1_P0_FLDMASK,
			TDM_AUDIO_RST_DP_ENCODER1_P0_FLDMASK);
	udelay(5);
	WRITE_BYTE_MASK(mtk_dp, REG_331C_DP_ENCODER1_P0 + reg_offset,
			0x0, TDM_AUDIO_RST_DP_ENCODER1_P0_FLDMASK);
	/* audio channel count change reset */
	WRITE_BYTE_MASK(mtk_dp, REG_33F0_DP_ENCODER1_P0 + 1 + reg_offset, 0, BIT(1));
}

static void mtk_dp_audio_ch_status_set_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id,
					  int channel, int fs, int wordlength)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);
	union dp_rx_audio_chsts dp_audio;

	memset(&dp_audio, 0, sizeof(dp_audio));

	switch (channel) {
	case 2:
	default:
		dp_audio.audio_chsts.channel_number = 2;
		break;

	case 8:
		dp_audio.audio_chsts.channel_number = 8;
		break;
	}

	switch (fs) {
	case 32000:
		dp_audio.audio_chsts.sampling_freq = 3;
		dp_audio.audio_chsts.original_sampling_freq = 0xc;
		break;

	case 44100:
		dp_audio.audio_chsts.sampling_freq = 0;
		dp_audio.audio_chsts.original_sampling_freq = 0xf;
		break;

	case 48000:
	default:
		dp_audio.audio_chsts.sampling_freq = 2;
		dp_audio.audio_chsts.original_sampling_freq = 0xd;
		break;

	case 88200:
		dp_audio.audio_chsts.sampling_freq = 8;
		dp_audio.audio_chsts.original_sampling_freq = 7;
		break;

	case 96000:
		dp_audio.audio_chsts.sampling_freq = 0xa;
		dp_audio.audio_chsts.original_sampling_freq = 5;
		break;

	case 192000:
		dp_audio.audio_chsts.sampling_freq = 0xe;
		dp_audio.audio_chsts.original_sampling_freq = 1;
		break;
	}

	switch (wordlength) {
	case DP_BITWIDTH_16:
		dp_audio.audio_chsts.word_len = 0b0010;
		break;

	case DP_BITWIDTH_20:
		dp_audio.audio_chsts.word_len = 0b0011;
		break;

	case DP_BITWIDTH_24:
	default:
		dp_audio.audio_chsts.word_len = 0b1011;
		break;
	}

	WRITE_2BYTE(mtk_dp, REG_308C_DP_ENCODER0_P0 + reg_offset,
		    ((dp_audio.audio_chsts_raw[1] << 8) | dp_audio.audio_chsts_raw[0]));
	WRITE_2BYTE(mtk_dp, REG_3090_DP_ENCODER0_P0 + reg_offset,
		    ((dp_audio.audio_chsts_raw[3] << 8) | dp_audio.audio_chsts_raw[2]));
	WRITE_BYTE(mtk_dp, REG_3094_DP_ENCODER0_P0 + reg_offset, dp_audio.audio_chsts_raw[4]);

	mdelay(1);
}

static void mtk_dp_audio_sdp_config_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id,
				       int ch, int fs, int len)
{
	u8 SDP_DB[32] = {0};
	u8 SDP_HB[4] = {0, DP_SDP_HB1_PKG_AINFO, 0x1b, 0x48};

	if (ch > 0)
		SDP_DB[0x0] = 0x10 | (ch - 1);
	else
		SDP_DB[0x0] = 0x10;

	switch (fs) {
	case 32000:
		SDP_DB[0x1] = 0x1 << 2 ;
		break;

	case 44100:
		SDP_DB[0x1] = 0x2 << 2 ;
		break;

	case 48000:
		SDP_DB[0x1] = 0x3 << 2 ;
		break;

	case 88200:
		SDP_DB[0x1] = 0x4 << 2 ;
		break;

	case 96000:
		SDP_DB[0x1] = 0x5 << 2 ;
		break;

	case 192000:
		SDP_DB[0x1] = 0x6 << 2 ;
		break;
	default:
		SDP_DB[0x1] = 0x0;
		break;
	}

	switch (len) {
	case DP_BITWIDTH_16:
		SDP_DB[0x1] |= 0x1;
		break;

	case DP_BITWIDTH_20:
		SDP_DB[0x1] |= 0x2;
		break;

	case DP_BITWIDTH_24:
	default:
		SDP_DB[0x1] |= 0x3;
		break;
	}

	SDP_DB[0x2] = 0x0;

	if (ch == 8)
		SDP_DB[0x3] = 0x13;
	else
		SDP_DB[0x3] = 0x00;

	mtk_dp_audio_sdp_setting_v2(mtk_dp, encoder_id, ch);
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] [%d] I2S Set Audio Channel = %d\n", encoder_id, ch);
	mtk_dp_spkg_sdp_v2(mtk_dp, encoder_id, true, DP_SDP_PKG_AUI, SDP_HB, SDP_DB);
}

void mtk_dp_audio_config_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id)
{
	int ch, fs, len;
	u8 table[8][5] = {"X1", "X2", "X4", "X8",
			  "/2", "/4", "X1", "/8"};

	ch = mtk_dp->info[encoder_id].audio_cur_cfg.channels;
	fs = mtk_dp->info[encoder_id].audio_cur_cfg.sample_rate;
	len = mtk_dp->info[encoder_id].audio_cur_cfg.word_length_bits;

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] [%d] ch:%d, fs:%d, len:%d\n", encoder_id, ch, fs, len);

	mtk_dp_audio_sdp_config_v2(mtk_dp, encoder_id, ch, fs, len);

	mtk_dp_audio_ch_status_set_v2(mtk_dp, encoder_id, ch, fs, len);

	mtk_dp_audio_pg_enable_v2(mtk_dp, encoder_id, ch, fs, false);

	drm_dbg_kms(mtk_dp->drm_dev,
		    "[DPTX] [%d] Set audio M div %s\n", encoder_id, table[DP_AUDIO_M_DIV_D2]);
	mtk_dp_audio_set_mdiv_v2(mtk_dp, encoder_id, DP_AUDIO_M_DIV_D2);
}

bool mtk_dp_parse_audio_cap_v2(struct mtk_dp *mtk_dp, struct mtk_dp_audio_cfg *cfg)
{
	if (!mtk_dp->data->audio_supported)
		return false;

	if (cfg->sad_count <= 0) {
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] The SADs is NULL\n");
		return false;
	}

	return true;
}

void mtk_dp_audio_update_plugged_status_v2(struct mtk_dp *mtk_dp)
{
	bool plugged = false;
	u8 encoder_id, con_id;

	mutex_lock(&mtk_dp->update_plugged_status_lock);
	if (!mtk_dp->mst_enable) {
		if (mtk_dp->mtk_con[DP_FIRST_CON]) {
			plugged = mtk_dp->mtk_con[DP_FIRST_CON]->video_enable &
				  mtk_dp->info[DP_ENCODER_ID_0].audio_cur_cfg.detect_monitor;
			drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] SST, video enable:%d, detect monitor:%d\n",
				    mtk_dp->mtk_con[DP_FIRST_CON]->video_enable,
				    mtk_dp->info[DP_ENCODER_ID_0].audio_cur_cfg.detect_monitor);
			if (mtk_dp->plugged_cb && mtk_dp->codec_dev[DP_ENCODER_ID_0])
				mtk_dp->plugged_cb(mtk_dp->codec_dev[DP_ENCODER_ID_0], plugged);
		}
	} else {
		for (con_id = 0; con_id < ARRAY_SIZE(mtk_dp->mtk_con); con_id++) {
			if (mst_con_with_encoder(mtk_dp->mtk_con[con_id])) {
				encoder_id = mtk_dp->mtk_con[con_id]->encoder_id;
				plugged = mtk_dp->mtk_con[con_id]->video_enable &&
					  mtk_dp->info[encoder_id].audio_cur_cfg.detect_monitor;
				drm_dbg_kms(mtk_dp->drm_dev,
					    "[DPTX] MST, enc[%d] con[%d], video enable:%d, detect monitor:%d\n",
					    encoder_id, con_id, mtk_dp->mtk_con[con_id]->video_enable,
					    mtk_dp->info[encoder_id].audio_cur_cfg.detect_monitor);

				if (mtk_dp->plugged_cb && mtk_dp->codec_dev[encoder_id]) {
					drm_dbg_kms(mtk_dp->drm_dev,
						    "[DPTX] audio supported:%d, audio enable:%d, plugged:%d\n",
						    mtk_dp->data->audio_supported, mtk_dp->audio_enable, plugged);
					mtk_dp->plugged_cb(mtk_dp->codec_dev[encoder_id], plugged);
				}
			}
		}
	}
	mutex_unlock(&mtk_dp->update_plugged_status_lock);
}

static void mtk_dp_spkg_vsc_ext_cea_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id,
				       bool enable,
				       u8 hdr_num,
				       u8 *db)
{
	u8  vsc_hb1 = 0x21;
	u8  vsc_hb2;
	u8  pkg_cnt;
	u8  loop;
	u8  offset;
	u8  reg_index;
	u16 sdp_offset;
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	if (!enable) {
		WRITE_BYTE_MASK(mtk_dp, (REG_30A0_DP_ENCODER0_P0 + 1  + reg_offset), 0, BIT(4));
		WRITE_BYTE_MASK(mtk_dp, REG_32A0_DP_ENCODER1_P0  + reg_offset, 0, BIT(7));
		return;
	}

	vsc_hb2 = (hdr_num > 0) ? 0x40 : 0x00;

	WRITE_BYTE(mtk_dp, REG_31D0_DP_ENCODER0_P0  + reg_offset, 0x00);
	WRITE_BYTE(mtk_dp, (REG_31D0_DP_ENCODER0_P0 + 1  + reg_offset), vsc_hb1);
	WRITE_BYTE(mtk_dp, REG_31D4_DP_ENCODER0_P0  + reg_offset, vsc_hb2);
	WRITE_BYTE(mtk_dp, (REG_31D4_DP_ENCODER0_P0 + 1  + reg_offset), 0x00);
	WRITE_BYTE(mtk_dp, (REG_31D8_DP_ENCODER0_P0 + 1  + reg_offset), hdr_num);

	WRITE_BYTE_MASK(mtk_dp, REG_32A0_DP_ENCODER1_P0  + reg_offset, BIT(0), BIT(0));
	WRITE_BYTE_MASK(mtk_dp, REG_32A0_DP_ENCODER1_P0  + reg_offset, BIT(2), BIT(2));
	usleep_range(50, 51);

	WRITE_BYTE_MASK(mtk_dp, REG_32A0_DP_ENCODER1_P0  + reg_offset, 0, BIT(2));

	for (pkg_cnt = 0; pkg_cnt < (hdr_num + 1); pkg_cnt++) {
		sdp_offset = 0;

		for (loop = 0; loop < 4; loop++) {
			for (offset = 0; offset < 4; offset++) {
				for (reg_index = 0; reg_index < 2; reg_index++) {
					u32 addr = REG_32A4_DP_ENCODER1_P0
					+ offset * 4 + reg_index  + reg_offset;
					u8 tmp = sdp_offset
						 + offset * 2 + reg_index;

					WRITE_BYTE(mtk_dp, addr, db[tmp]);
				}
			}

			WRITE_BYTE_MASK(mtk_dp, REG_32A0_DP_ENCODER1_P0  + reg_offset,
					BIT(6), BIT(6));
			sdp_offset += 8;
		}
	}

	WRITE_BYTE_MASK(mtk_dp, (REG_30A0_DP_ENCODER0_P0 + 1  + reg_offset), BIT(4), BIT(4));
	WRITE_BYTE_MASK(mtk_dp, REG_32A0_DP_ENCODER1_P0  + reg_offset, BIT(7), BIT(7));
}

static void mtk_dsc_read_dsc_dpcd_v2(struct mtk_dp *mtk_dp, struct drm_dp_aux *aux,
				u8 dsc_dpcd[DP_DSC_RECEIVER_CAP_SIZE])
{
	if (drm_dp_dpcd_read(aux, DP_DSC_SUPPORT, dsc_dpcd, DP_DSC_RECEIVER_CAP_SIZE) < 0)
		dev_err(mtk_dp->dev, "[DPTX] Failed to read DPCD register 0x%x\n", DP_DSC_SUPPORT);
}

void mtk_dp_dsc_support_v2(struct mtk_dp *mtk_dp)
{
	mtk_dsc_read_dsc_dpcd_v2(mtk_dp, &mtk_dp->aux, mtk_dp->mtk_con[DP_FIRST_CON]->dsc_dpcd);

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] sink dsc capable:%d\n",
		    drm_dp_sink_supports_dsc(mtk_dp->mtk_con[DP_FIRST_CON]->dsc_dpcd));
}

void mtk_dp_dsc_enable_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] DSC enable\n");

	WRITE_2BYTE_MASK(mtk_dp, REG_31C4_DP_ENCODER0_P0 + reg_offset,
			 0,
			 PPS_HW_BYPASS_MASK_DP_ENCODER0_P0_FLDMASK);

	/* [0] : DSC Enable */
	WRITE_BYTE_MASK(mtk_dp,
			REG_336C_DP_ENCODER1_P0 + reg_offset, BIT(0), BIT(0));
	/* 300C [9] : VB-ID[6] DSC enable */
	WRITE_BYTE_MASK(mtk_dp,
			REG_300C_DP_ENCODER0_P0 + 1 + reg_offset, BIT(1), BIT(1));
	/* 303C[10 : 8] : DSC color depth */
	WRITE_BYTE_MASK(mtk_dp,
			REG_303C_DP_ENCODER0_P0 + 1 + reg_offset,
			0x7, GENMASK(2, 0));
	/* 303C[14 : 12] : DSC color format */
	WRITE_BYTE_MASK(mtk_dp,
			REG_303C_DP_ENCODER0_P0 + 1 + reg_offset,
			0x7 << 4, GENMASK(6, 4));
	/* 31FC[12] : HDE last num control */
	WRITE_2BYTE_MASK(mtk_dp, REG_31FC_DP_ENCODER0_P0 + reg_offset,
			 0x2 << DE_LAST_NUM_SW_DP_ENCODER0_P0_FLDMASK_POS,
			 DE_LAST_NUM_SW_DP_ENCODER0_P0_FLDMASK);
}

void mtk_dp_dsc_disable_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] DSC disable\n");

	WRITE_2BYTE_MASK(mtk_dp, REG_31C4_DP_ENCODER0_P0 + reg_offset,
			 0,
			 PPS_HW_BYPASS_MASK_DP_ENCODER0_P0_FLDMASK);

	/* DSC Disable */
	WRITE_BYTE_MASK(mtk_dp,
			REG_336C_DP_ENCODER1_P0 + reg_offset, 0, BIT(0));
	WRITE_BYTE_MASK(mtk_dp,
			REG_300C_DP_ENCODER0_P0 + 1 + reg_offset, 0, BIT(1));
	/* default 8bit */
	WRITE_BYTE_MASK(mtk_dp,
			REG_303C_DP_ENCODER0_P0 + 1 + reg_offset,
			0x3, GENMASK(2, 0));
	/* default RGB */
	WRITE_BYTE_MASK(mtk_dp,
			REG_303C_DP_ENCODER0_P0 + 1 + reg_offset,
			0x0, GENMASK(6, 4));

	/* 31FC[12] : HDE last num control */
	/* 31FC[12] : HDE last num control */
	WRITE_2BYTE_MASK(mtk_dp, REG_31FC_DP_ENCODER0_P0 + reg_offset,
			 0, DE_LAST_NUM_SW_DP_ENCODER0_P0_FLDMASK);
}

void mtk_dp_set_chunk_size_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id,
		u8 slice_num, u16 chunk_num, u8 remainder,
		u8 lane_count, u32 hde_last_num, u8 hde_num_even)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	WRITE_BYTE_MASK(mtk_dp,
			REG_336C_DP_ENCODER1_P0 + reg_offset,
			slice_num << 4, GENMASK(7, 4));
	WRITE_BYTE_MASK(mtk_dp,
			REG_336C_DP_ENCODER1_P0 + 1 + reg_offset,
			remainder, GENMASK(3, 0));
	WRITE_2BYTE(mtk_dp,
		    REG_3370_DP_ENCODER1_P0 + reg_offset, chunk_num - 1); /* set chunk_num */

	/* 0x31FC replaced by 0x3064 */
	WRITE_2BYTE(mtk_dp, REG_3064_DP_ENCODER0_P0 + reg_offset, hde_last_num);
}

void mtk_dp_dsc_set_param_v2(struct mtk_dp *mtk_dp,
		const enum dp_encoder_id encoder_id, union dp_pps *pps)
{
	u16 chunk_num = pps->pps_raw[14] << 8 | pps->pps_raw[15];
	u8 slice_num = (pps->pps_raw[8] << 8 | pps->pps_raw[9]) /
					(pps->pps_raw[12] << 8 | pps->pps_raw[13]);
	u32 hde_last_num = 0;
	u32 hde_num_even = 0;
	u8 lane_count = mtk_dp->training_info.link_lane_count;

	hde_last_num = (chunk_num % lane_count);
	hde_num_even = chunk_num + (hde_last_num ? (lane_count - hde_last_num) : 0);
	hde_last_num = ((hde_num_even + lane_count) * slice_num);
	hde_last_num = DIV_ROUND_UP(hde_last_num,3);

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] encoder_id = %d\n", encoder_id);
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] slice_num = %d\n", slice_num);
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] chunk_size = %d\n", chunk_num);
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] lane count = %d\n", lane_count);
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] hde_last_num = %d\n", hde_last_num);
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] hde_num_even = %d\n", hde_num_even);

	mtk_dp_set_chunk_size_v2(mtk_dp, encoder_id, slice_num - 1, chunk_num, chunk_num % 12,
				 lane_count, hde_last_num, hde_num_even);
}

void mtk_dp_dsc_set_pps_v2(struct mtk_dp *mtk_dp,
		const enum dp_encoder_id encoder_id, union dp_pps *pps, bool enable)
{
	u8 hb[4] = {0x0, 0x10, 0x7F, 0x0};
	int i;

	for (i = 0; i < 128; i += 8)
		drm_dbg_kms(mtk_dp->drm_dev,
			    "[DPTX] 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x\n",
			    pps->pps_raw[i + 0], pps->pps_raw[i + 1],
			    pps->pps_raw[i + 2], pps->pps_raw[i + 3],
			    pps->pps_raw[i + 4], pps->pps_raw[i + 5],
			    pps->pps_raw[i + 6], pps->pps_raw[i + 7]);

	mtk_dp_spkg_sdp_v2(mtk_dp, encoder_id, enable, DP_SDP_PKG_PPS0, hb, pps->pps_raw +  0);
	mtk_dp_spkg_sdp_v2(mtk_dp, encoder_id, enable, DP_SDP_PKG_PPS1, hb, pps->pps_raw + 32);
	mtk_dp_spkg_sdp_v2(mtk_dp, encoder_id, enable, DP_SDP_PKG_PPS2, hb, pps->pps_raw + 64);
	mtk_dp_spkg_sdp_v2(mtk_dp, encoder_id, enable, DP_SDP_PKG_PPS3, hb, pps->pps_raw + 96);
}

void mtk_dp_dsc_pps_send_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id)
{
	mtk_dp_dsc_set_pps_v2(mtk_dp, encoder_id, &mtk_dp->info[encoder_id].pps, true);
	mtk_dp_dsc_set_param_v2(mtk_dp, encoder_id, &mtk_dp->info[encoder_id].pps);
}

void mtk_dp_dsc_parse_pps_param_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id, u8 *pps)
{
	union dp_pps pps_struct;
	u8 bpp;

	pps_struct.pps.major = (pps[0] >> 4) & 0xF;
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] major:%d\n", pps_struct.pps.major);
	pps_struct.pps.minor = pps[0] & 0xF;
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] minor:%d\n", pps_struct.pps.minor);

	pps_struct.pps.color_depth = (pps[3] >> 4) & 0xF;
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] color_depth:%d\n", pps_struct.pps.color_depth);
	pps_struct.pps.buffer_depth = pps[3] & 0xF;
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] buffer_depth:%d\n", pps_struct.pps.buffer_depth);

	pps_struct.pps.bp_enable = (pps[4] >> 5) & 0x1;
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] bp_enable:%d\n", pps_struct.pps.bp_enable);
	pps_struct.pps.convert_rgb = (pps[4] >> 4) & 0x1;
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] convert_rgb:%d\n", pps_struct.pps.convert_rgb);
	pps_struct.pps.simple_422 = (pps[4] >> 3) & 0x1;
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] simple_422:%d\n", pps_struct.pps.simple_422);
	pps_struct.pps.vbr_enable = (pps[4] >> 1) & 0x1;
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] vbr_enable:%d\n", pps_struct.pps.vbr_enable);

	pps_struct.pps.bit_per_pixel = ((pps[4] & 0x3) << 4) |
						(pps[5]  >> 4);
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] bit_per_pixel:%d\n", pps_struct.pps.bit_per_pixel);

	pps_struct.pps.pic_height = (pps[6] << 8) | pps[7];
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] pic_height:%d\n", pps_struct.pps.pic_height);
	pps_struct.pps.pic_width = (pps[8] << 8) | pps[9];
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] pic_width:%d\n", pps_struct.pps.pic_width);
	pps_struct.pps.slice_height = (pps[10] << 8) | pps[11];
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] slice_height:%d\n", pps_struct.pps.slice_height);
	pps_struct.pps.slice_width = (pps[12] << 8) | pps[13];
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] slice_width:%d\n", pps_struct.pps.slice_width);
	pps_struct.pps.chunk_size = (pps[14] << 8) | pps[15];
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] chunk_size:%d\n", pps_struct.pps.chunk_size);

	pps_struct.pps.native_420 = (pps[88] >> 1) & 0x1;
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] native_420:%d\n", pps_struct.pps.native_420);
	pps_struct.pps.native_422 = pps[88] & 0x1;
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] native_422:%d\n", pps_struct.pps.native_422);

	bpp = ((pps[4] & 0x3) << 4) | (pps[5]  >> 4);
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] bpp:%d\n", bpp);

	memcpy(&mtk_dp->info[encoder_id].pps.pps_raw, pps, 128);
}

static int mtk_dsc_compute_params_v2(struct mtk_dp *mtk_dp,
			const enum dp_encoder_id encoder_id,
			struct drm_dsc_config *vdsc_cfg,
			u16 width, u16 hight,
			u8 slice_count, u16 bpc, u16 compressed_bpp)
{
	int ret = 0;
	int con_id;

	con_id = encoder_id_to_con_id(mtk_dp, encoder_id,
				      mtk_dp->mst_enable ? DRM_DP_MST : DRM_DP_SST);
	if (con_id < 0)
		return con_id;

	vdsc_cfg->rc_model_size = DSC_RC_MODEL_SIZE_CONST;

	vdsc_cfg->pic_width = width;
	vdsc_cfg->pic_height = hight;

	vdsc_cfg->slice_width = DIV_ROUND_UP(vdsc_cfg->pic_width, slice_count);

	if (vdsc_cfg->pic_height % 8 == 0)
		vdsc_cfg->slice_height = 8;
	else if (vdsc_cfg->pic_height % 4 == 0)
		vdsc_cfg->slice_height = 4;
	else
		vdsc_cfg->slice_height = 2;

	vdsc_cfg->convert_rgb = mtk_dp->info[encoder_id].format == DP_PIXELFORMAT_RGB;
	vdsc_cfg->native_420 = false;
	vdsc_cfg->native_422 = false;
	vdsc_cfg->simple_422 = false;
	vdsc_cfg->vbr_enable = false;

	vdsc_cfg->bits_per_pixel = compressed_bpp << 4;
	vdsc_cfg->bits_per_component = bpc;

	drm_dsc_set_rc_buf_thresh(vdsc_cfg);

	drm_dsc_set_const_params(vdsc_cfg);

	ret = drm_dsc_setup_rc_params(vdsc_cfg, DRM_DSC_1_2_444);

	if (ret < 0)
		dev_err(mtk_dp->dev, "[DPTX] drm_dsc_setup_rc_params ret %d\n", ret);

	/* InitialScaleValue is a 6 bit value with 3 fractional bits (U3.3) */
	vdsc_cfg->initial_scale_value = (vdsc_cfg->rc_model_size << 3) /
		(vdsc_cfg->rc_model_size - vdsc_cfg->initial_offset);

	vdsc_cfg->dsc_version_major =
		(mtk_dp->mtk_con[con_id]->dsc_dpcd[DP_DSC_REV - DP_DSC_SUPPORT] &
		DP_DSC_MAJOR_MASK) >> DP_DSC_MAJOR_SHIFT;
	vdsc_cfg->dsc_version_minor =
		min(2, (mtk_dp->mtk_con[con_id]->dsc_dpcd[DP_DSC_REV - DP_DSC_SUPPORT] &
		DP_DSC_MINOR_MASK) >> DP_DSC_MINOR_SHIFT);
	if (vdsc_cfg->convert_rgb)
		vdsc_cfg->convert_rgb =
			mtk_dp->mtk_con[con_id]->dsc_dpcd[DP_DSC_DEC_COLOR_FORMAT_CAP -
			DP_DSC_SUPPORT] & DP_DSC_RGB;

	vdsc_cfg->line_buf_depth = bpc + 1;

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] line_buf_depth:%d\n", vdsc_cfg->line_buf_depth);

	vdsc_cfg->block_pred_enable =
		mtk_dp->mtk_con[con_id]->dsc_dpcd[DP_DSC_BLK_PREDICTION_SUPPORT - DP_DSC_SUPPORT] &
		DP_DSC_BLK_PREDICTION_IS_SUPPORTED;

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] block_pred_enable:%d\n", vdsc_cfg->block_pred_enable);

	drm_dsc_compute_rc_parameters(vdsc_cfg);

	return 0;
}

void mtk_dp_dsc_check_prepare_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id)
{
	struct drm_dsc_picture_parameter_set pps;
	struct drm_dsc_config mtk_dsc_cfg;

	memset(&pps, 0x0, sizeof(pps));
	memset(&mtk_dsc_cfg, 0x0, sizeof(mtk_dsc_cfg));

	mtk_dsc_compute_params_v2(mtk_dp, encoder_id, &mtk_dsc_cfg,
				  mtk_dp->mode[encoder_id].hdisplay,
				  mtk_dp->mode[encoder_id].vdisplay, 2, 8, 8);
	drm_dsc_pps_payload_pack(&pps, &mtk_dsc_cfg);
	mtk_dp_dsc_parse_pps_param_v2(mtk_dp, encoder_id, (u8 *)&pps);

	memcpy(mtk_dp->prop_dsc_cfg[encoder_id]->values,
	       &mtk_dsc_cfg, sizeof(struct drm_dsc_config));
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] dsc version:%d", mtk_dsc_cfg.dsc_version_minor);
}

u32 mtk_dp_dsc_cal_clock_v2(struct drm_display_mode *mode)
{
	u16 hblank;
	u16 hactive;
	u64 htotal;
	u64 mode_htotal;
	u32 pixel_clock;

	hblank = mode->htotal - mode->hdisplay;
	hactive = ((mode->hdisplay * DP_DSC_BPP + (12 * 8 - 1)) / (12 * 8)) * 4;
	htotal = hblank + hactive;
	mode_htotal =  mode->htotal;
	pixel_clock = div_u64(mode->clock * htotal, mode_htotal);

	return pixel_clock;
}

static int set_dsc_decompression_flag_v2(struct drm_dp_aux *aux, u8 flag, bool set)
{
	int err;
	u8 val;

	err = drm_dp_dpcd_readb(aux, DP_DSC_ENABLE, &val);
	if (err < 0)
		return err;

	if (set)
		val |= flag;
	else
		val &= ~flag;

	return drm_dp_dpcd_writeb(aux, DP_DSC_ENABLE, val);
}

static void mtk_dp_phy_set_rate_param_v2(struct mtk_dp *mtk_dp, enum dp_link_rate val)
{
	u32 phyd_dig_glb_offset = mtk_dp->data->phyd_dig_glb_offset;

	switch (val) {
	case DP_LINK_RATE_RBR:
		/* Set gear : 0x0 : RBR, 0x1 : HBR, 0x2 : HBR2, 0x3 : HBR3 */
		PHY_WRITE_4BYTE(mtk_dp, phyd_dig_glb_offset + DP_PHY_DIG_BIT_RATE, 0x0);
		break;

	case DP_LINK_RATE_HBR:
		/* Set gear : 0x0 : RBR, 0x1 : HBR, 0x2 : HBR2, 0x3 : HBR3 */
		PHY_WRITE_4BYTE(mtk_dp, phyd_dig_glb_offset + DP_PHY_DIG_BIT_RATE, 0x1);
		break;

	case DP_LINK_RATE_HBR2:
		/* Set gear : 0x0 : RBR, 0x1 : HBR, 0x2 : HBR2, 0x3 : HBR3 */
		PHY_WRITE_4BYTE(mtk_dp, phyd_dig_glb_offset + DP_PHY_DIG_BIT_RATE, 0x2);
		break;

	case DP_LINK_RATE_HBR3:
		/* Set gear : 0x0 : RBR, 0x1 : HBR, 0x2 : HBR2, 0x3 : HBR3 */
		PHY_WRITE_4BYTE(mtk_dp, phyd_dig_glb_offset + DP_PHY_DIG_BIT_RATE, 0x3);
		break;
	default:
		break;
	}
}

static void mtk_dp_phy_set_link_rate_v2(struct mtk_dp *mtk_dp, enum dp_link_rate val)
{
	mtk_dp_phy_set_rate_param_v2(mtk_dp, val);
}

static void mtk_dp_phy_set_lane_pwr_v2(struct mtk_dp *mtk_dp, enum dp_lane_count lane_count)
{
	int power_indx = lane_count - 1;
	u8 power_bmp = BIT(power_indx);
	u32 phyd_dig_glb_offset = mtk_dp->data->phyd_dig_glb_offset;

	do {
		power_bmp |= BIT(power_indx);
		PHY_WRITE_BYTE_MASK(mtk_dp, phyd_dig_glb_offset + DP_PHY_DIG_TX_CTL_0,
				    power_bmp << TX_LN_EN_FLDMASK_POS,
				    TX_LN_EN_FLDMASK);

		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] set lane pwr %x\n",
			    FIELD_GET(TX_LN_EN_FLDMASK,
				      PHY_READ_BYTE(mtk_dp, phyd_dig_glb_offset
						    + DP_PHY_DIG_TX_CTL_0)));
	} while (--power_indx >= 0);
}

static void mtk_dp_phy_clear_lane_pwr_v2(struct mtk_dp *mtk_dp)
{
	u32 phyd_dig_glb_offset = mtk_dp->data->phyd_dig_glb_offset;

	u8 power_bmp = FIELD_GET(TX_LN_EN_FLDMASK,
				 PHY_READ_BYTE(mtk_dp, phyd_dig_glb_offset + DP_PHY_DIG_TX_CTL_0));
	do {
		power_bmp >>= 1;
		PHY_WRITE_BYTE_MASK(mtk_dp, phyd_dig_glb_offset + DP_PHY_DIG_TX_CTL_0,
				    power_bmp << TX_LN_EN_FLDMASK_POS,
				    TX_LN_EN_FLDMASK);
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] clear lane pwr %x\n",
			    FIELD_GET(TX_LN_EN_FLDMASK,
				      PHY_READ_BYTE(mtk_dp,
				      phyd_dig_glb_offset + DP_PHY_DIG_TX_CTL_0)));
	} while (power_bmp > 0);
}

static void mtk_dp_phy_power_on_v2(struct mtk_dp *mtk_dp)
{
	u8 rdy_status;
	u8 rdy_bmp = RGS_BIAS_READY_FLDMASK;
	void *reg = mtk_dp->phyd_regs
		    + mtk_dp->data->phyd_dig_glb_offset + DP_PHY_DIG_GLB_STATUS_02;

	PHY_WRITE_BYTE_MASK(mtk_dp, mtk_dp->data->phyd_dig_glb_offset + DP_PHY_DIG_PLL_CTL_0,
			    0x1 << FORCE_PWR_STATE_EN_FLDMASK_POS, FORCE_PWR_STATE_EN_FLDMASK);
	PHY_WRITE_BYTE_MASK(mtk_dp, mtk_dp->data->phyd_dig_glb_offset + DP_PHY_DIG_PLL_CTL_0,
			    0x3 << FORCE_PWR_STATE_VAL_FLDMASK_POS, FORCE_PWR_STATE_VAL_FLDMASK);

	if (readl_poll_timeout_atomic(reg, rdy_status, rdy_status & rdy_bmp,
		DP_CHECK_PHY_POLLING_DELAY_TIME, DP_CHECK_PHY_POLLING_TIMEOUT)) {
		dev_err(mtk_dp->dev, "[DPTX] Polling BIAS status fail %x\n", rdy_status);
		return;
	}
	drm_dbg_kms(mtk_dp->drm_dev,
		    "[DPTX] DP PHYD power on and BIAS status %x\n", rdy_status);
}

static void mtk_dp_phy_power_down_v2(struct mtk_dp *mtk_dp)
{
	mtk_dp_phy_clear_lane_pwr_v2(mtk_dp);

	PHY_WRITE_BYTE_MASK(mtk_dp, mtk_dp->data->phyd_dig_glb_offset + DP_PHY_DIG_PLL_CTL_0,
			    0x1 << FORCE_PWR_STATE_EN_FLDMASK_POS, FORCE_PWR_STATE_EN_FLDMASK);
	/* power off TPLL and Lane */
	PHY_WRITE_BYTE_MASK(mtk_dp, mtk_dp->data->phyd_dig_glb_offset + DP_PHY_DIG_PLL_CTL_0,
			    0x1 << FORCE_PWR_STATE_VAL_FLDMASK_POS, FORCE_PWR_STATE_VAL_FLDMASK);

	PHY_WRITE_BYTE_MASK(mtk_dp, mtk_dp->data->phyd_dig_glb_offset + DP_PHY_DIG_SW_RST,
			    0, BIT(1) | BIT(3));
	PHY_WRITE_BYTE_MASK(mtk_dp, mtk_dp->data->phyd_dig_glb_offset + DP_PHY_DIG_SW_RST,
			    BIT(1) | BIT(3), BIT(1) | BIT(3));

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] DP PHYD power down\n");
}

static void mtk_dp_phy_check_ready_v2(struct mtk_dp *mtk_dp, enum dp_lane_count lane_count)
{
	u8 rdy_status;
	u8 rdy_bmp = (RGS_BIAS_READY_FLDMASK | RGS_TX_LN0_READY_FLDMASK);
	void *reg = mtk_dp->phyd_regs
		    + mtk_dp->data->phyd_dig_glb_offset + DP_PHY_DIG_GLB_STATUS_02;

	switch (lane_count) {
	case DP_4LANE:
		rdy_bmp |= (RGS_TX_LN3_READY_FLDMASK |
			    RGS_TX_LN2_READY_FLDMASK |
			    RGS_TX_LN1_READY_FLDMASK);
		break;
	case DP_2LANE:
		rdy_bmp |= RGS_TX_LN1_READY_FLDMASK;
		break;
	default:
		break;
	}

	if (readl_poll_timeout_atomic(reg, rdy_status, rdy_status & rdy_bmp,
		DP_CHECK_PHY_POLLING_DELAY_TIME, DP_CHECK_PHY_POLLING_TIMEOUT)) {
		dev_err(mtk_dp->dev, "[DPTX] Polling tx_ln status fail %x\n", rdy_status);
		return;
	}
	drm_dbg_kms(mtk_dp->drm_dev,
		    "[DPTX] Polling tx_ln status pass and tx_ln status %x\n", rdy_status);
}

static void mtk_dp_phy_reset_swing_pre_v2(struct mtk_dp *mtk_dp)
{
	u8 i;

	for (i = 0; i < 4; i++) {
		PHY_WRITE_4BYTE_MASK(mtk_dp,
				     mtk_dp->data->phyd_dig_lan_offset[i] + DRIVING_FORCE,
				     0x1 << DP_TX_FORCE_VOLT_SWING_EN_FLDMASK_POS,
				     DP_TX_FORCE_VOLT_SWING_EN_FLDMASK);
		PHY_WRITE_4BYTE_MASK(mtk_dp,
				     mtk_dp->data->phyd_dig_lan_offset[i] + DRIVING_FORCE,
				     0x1 << DP_TX_FORCE_PRE_EMPH_EN_FLDMASK_POS,
				     DP_TX_FORCE_PRE_EMPH_EN_FLDMASK);
		PHY_WRITE_4BYTE_MASK(mtk_dp,
				     mtk_dp->data->phyd_dig_lan_offset[i] + DRIVING_FORCE,
				     0x0 << DP_TX_FORCE_VOLT_SWING_VAL_FLDMASK_POS,
				     DP_TX_FORCE_VOLT_SWING_VAL_FLDMASK);
		PHY_WRITE_4BYTE_MASK(mtk_dp,
				     mtk_dp->data->phyd_dig_lan_offset[i] + DRIVING_FORCE,
				     0x0  << DP_TX_FORCE_PRE_EMPH_VAL_FLDMASK_POS,
				     DP_TX_FORCE_PRE_EMPH_VAL_FLDMASK);
	}
}

static void mtk_dp_phy_ssc_enable_v2(struct mtk_dp *mtk_dp, bool enable)
{
	u32 phyd_dig_glb_offset = mtk_dp->data->phyd_dig_glb_offset;
	if (enable)
		PHY_WRITE_BYTE_MASK(mtk_dp, phyd_dig_glb_offset + DP_PHY_DIG_PLL_CTL_1,
				    0x1 << TPLL_SSC_EN_FLDMASK_POS, TPLL_SSC_EN_FLDMASK);
	else
		PHY_WRITE_BYTE_MASK(mtk_dp, phyd_dig_glb_offset + DP_PHY_DIG_PLL_CTL_1,
				    0x0, TPLL_SSC_EN_FLDMASK);

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] Phy SSC enable = %d\n", enable);
}

static void mtk_dp_phy_param_init_v2(struct mtk_dp *mtk_dp, u32 *buffer, u32 size)
{
	u32 i = 0;
	u8  mask = GENMASK(5,0);

	if (!buffer || size != DP_PHY_REG_COUNT) {
		dev_err(mtk_dp->dev, "[DPTX] invalid param\n");
		return;
	}

	for (i = 0; i < DP_PHY_LEVEL_COUNT; i++) {
		mtk_dp->phy_params[i].c0 = (buffer[i / 4] >> (8 * (i % 4))) & mask;
		mtk_dp->phy_params[i].cp1 = (buffer[i / 4 + 3] >> (8 * (i % 4))) & mask;
	}
}

static void mtk_dp_phy_4lane_enable_v2(struct mtk_dp *mtk_dp)
{
	u8 i;
	u8 lane_count;
	u16 value;
	u32 tmp;

	lane_count = 4;
	value = (BIT(12) | BIT(13));

	tmp = readl(mtk_dp->phy_mux_regs);
	tmp |= mtk_dp->data->phy_4lane_ctrl_bit;
	writel(tmp, mtk_dp->phy_mux_regs);

	if (mtk_dp->data->need_phy_lane_enable_set)
		for (i = 1; i <= lane_count; i++)
			PHY_WRITE_2BYTE_MASK(mtk_dp, 0x0100 * i, value, (BIT(12) | BIT(13)));
}

static void mtk_dp_phy_4lane_disable_v2(struct mtk_dp *mtk_dp)
{
	u8 i;
	u8 lane_count;
	u16 value;
	u32 tmp;

	lane_count = 2;
	value = BIT(12);

	tmp = readl(mtk_dp->phy_mux_regs);
	tmp &= ~mtk_dp->data->phy_4lane_ctrl_bit;
	writel(tmp, mtk_dp->phy_mux_regs);

	if (mtk_dp->data->need_phy_lane_enable_set)
		for (i = 1; i <= lane_count; i++)
			PHY_WRITE_2BYTE_MASK(mtk_dp, 0x0100 * i, value, (BIT(12) | BIT(13)));
}

void mtk_dp_phy_flip_enable_v2(struct mtk_dp *mtk_dp)
{
	u32 tmp;

	if (mtk_dp->data->need_phy_flip_set) {
		PHY_WRITE_BYTE(mtk_dp, 0x01a0, 0x47);
		PHY_WRITE_BYTE(mtk_dp, 0x02a0, 0x47);
		PHY_WRITE_BYTE(mtk_dp, 0x03a0, 0x46);
		PHY_WRITE_BYTE(mtk_dp, 0x04a0, 0x46);
	}

	tmp = readl(mtk_dp->phy_mux_regs);
	tmp |= mtk_dp->data->phy_flip_ctrl_bit;
	writel(tmp, mtk_dp->phy_mux_regs);

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] Phy flip enable\n");
}

void mtk_dp_phy_flip_disable_v2(struct mtk_dp *mtk_dp)
{
	u32 tmp;

	if (mtk_dp->data->need_phy_flip_set) {
		PHY_WRITE_BYTE(mtk_dp, 0x01a0, 0x46);
		PHY_WRITE_BYTE(mtk_dp, 0x02a0, 0x46);
		PHY_WRITE_BYTE(mtk_dp, 0x03a0, 0x47);
		PHY_WRITE_BYTE(mtk_dp, 0x04a0, 0x47);
	}

	tmp = readl(mtk_dp->phy_mux_regs);
	tmp &= ~mtk_dp->data->phy_flip_ctrl_bit;
	writel(tmp, mtk_dp->phy_mux_regs);

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] Phy flip disable\n");
}

static void mtk_mt8196_dp_phy_set_param_v2(struct mtk_dp *mtk_dp)
{
	u8 i;

	/* 4-1. PLL */
	PHY_WRITE_BYTE_MASK(mtk_dp, 0x0614, BIT(0), BIT(0));
	/* 4-2. Unused AUX TX High-Z */
	PHY_WRITE_4BYTE_MASK(mtk_dp, 0x0700, 0x0, BIT(20));

	/* 4-4. Swing and Pre-emphasis Optimization */
	for (i = 0; i < 4; i++) {
		PHY_WRITE_4BYTE(mtk_dp, mtk_dp->data->phyd_dig_lan_offset[i] + DRIVING_PARAM_3,
				0x110e0c0a);
		PHY_WRITE_4BYTE(mtk_dp, mtk_dp->data->phyd_dig_lan_offset[i] + DRIVING_PARAM_4,
				0x1212110e);
		PHY_WRITE_4BYTE(mtk_dp, mtk_dp->data->phyd_dig_lan_offset[i] + DRIVING_PARAM_5,
				0x1815);
		PHY_WRITE_4BYTE(mtk_dp, mtk_dp->data->phyd_dig_lan_offset[i] + DRIVING_PARAM_6,
				0x7040200);
		PHY_WRITE_4BYTE(mtk_dp, mtk_dp->data->phyd_dig_lan_offset[i] + DRIVING_PARAM_7,
				0x60300);
		PHY_WRITE_4BYTE(mtk_dp, mtk_dp->data->phyd_dig_lan_offset[i] + DRIVING_PARAM_8,
				0x3);
	}
}

void mtk_mt8189_dp_phy_set_param_v2(struct mtk_dp *mtk_dp)
{
	int i;

	PHY_WRITE_4BYTE(mtk_dp,
			mtk_dp->data->phyd_ana_glb_offset
			+ DP_PHY_GLB_FORCE_CTRL_00,
			mtk_dp->data->patch_phy_settings[0]);

	for (i = 0; i < 4; i++) {
		PHY_WRITE_4BYTE(mtk_dp,
				mtk_dp->data->phyd_ana_lan_offset[i]
				+ DP_PHY_LANE_TX_2,
				mtk_dp->data->patch_phy_settings[1]);
	}

	PHY_WRITE_4BYTE(mtk_dp,
			mtk_dp->data->phyd_dig_glb_offset
			+ DP_PHY_DIG_GLB_DA_REG_00,
			mtk_dp->data->patch_phy_settings[2]);
	PHY_WRITE_4BYTE(mtk_dp,
			mtk_dp->data->phyd_dig_glb_offset
			+ DP_PHY_DIG_GLB_DA_REG_01,
			mtk_dp->data->patch_phy_settings[3]);
	PHY_WRITE_4BYTE(mtk_dp,
			mtk_dp->data->phyd_dig_glb_offset
			+ DP_PHY_DIG_GLB_DA_REG_02,
			mtk_dp->data->patch_phy_settings[4]);
	PHY_WRITE_4BYTE(mtk_dp,
			mtk_dp->data->phyd_dig_glb_offset
			+ DP_PHY_DIG_GLB_DA_REG_03,
			mtk_dp->data->patch_phy_settings[5]);

	for (i = 0; i < 4; i++) {
		PHY_WRITE_4BYTE(mtk_dp,
				mtk_dp->data->phyd_dig_lan_offset[i] + DRIVING_PARAM_3,
				mtk_dp->data->txfir_settings[0]);
		PHY_WRITE_4BYTE(mtk_dp,
				mtk_dp->data->phyd_dig_lan_offset[i] + DRIVING_PARAM_4,
				mtk_dp->data->txfir_settings[1]);
		PHY_WRITE_4BYTE(mtk_dp,
				mtk_dp->data->phyd_dig_lan_offset[i] + DRIVING_PARAM_5,
				mtk_dp->data->txfir_settings[2]);
		PHY_WRITE_4BYTE(mtk_dp,
				mtk_dp->data->phyd_dig_lan_offset[i] + DRIVING_PARAM_6,
				mtk_dp->data->txfir_settings[3]);
		PHY_WRITE_4BYTE(mtk_dp,
				mtk_dp->data->phyd_dig_lan_offset[i] + DRIVING_PARAM_7,
				mtk_dp->data->txfir_settings[4]);
		PHY_WRITE_4BYTE(mtk_dp,
				mtk_dp->data->phyd_dig_lan_offset[i] + DRIVING_PARAM_8,
				mtk_dp->data->txfir_settings[5]);
	}
}

static void mtk_dp_phy_setting_v2(struct mtk_dp *mtk_dp)
{
	/* step1: phy init */
	if (mtk_dp->training_info.max_link_lane_count == DP_4LANE)
		mtk_dp_phy_4lane_enable_v2(mtk_dp);
	else
		mtk_dp_phy_4lane_disable_v2(mtk_dp);

	if (mtk_dp->phy_flip_enable)
		mtk_dp_phy_flip_enable_v2(mtk_dp);
	else
		mtk_dp_phy_flip_disable_v2(mtk_dp);

	if (mtk_dp->data->set_param)
		mtk_dp->data->set_param(mtk_dp);

	/* step2: phy power ON */
	mtk_dp_phy_power_on_v2(mtk_dp);
}

static void mtk_dp_phy_training_config_v2(struct mtk_dp *mtk_dp, const u8 link_rate,
				const u8 lane_count, bool ssc_enable)
{
	mtk_dp_phy_reset_swing_pre_v2(mtk_dp);
	mtk_dp_phy_ssc_enable_v2(mtk_dp, ssc_enable);

	/* step1: phy-d power down */
	mtk_dp_phy_power_down_v2(mtk_dp);

	/* step2: phy-d set link rate */
	mtk_dp_phy_set_link_rate_v2(mtk_dp, link_rate);
	mtk_dp_phy_power_on_v2(mtk_dp);

	/* step3: phy-d enable lane */
	mtk_dp_phy_set_lane_pwr_v2(mtk_dp, lane_count);
	mtk_dp_phy_check_ready_v2(mtk_dp, lane_count);
}

static void mtk_dp_phy_set_idle_pattern_v2(struct mtk_dp *mtk_dp, bool enable)
{
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] Idle pattern enable:%d\n", enable);
	WRITE_BYTE_MASK(mtk_dp, REG_3580_DP_TRANS_P0 + 1, enable ? 0x0f : 0x00, 0x0f);
}

static bool mtk_dp_ssc_check_v2(struct mtk_dp *mtk_dp, bool *p_enable)
{
	u8 status = 0;
	int ret = 0;

	if (mtk_dp->data->ssc_support)
		*p_enable = mtk_dp->training_info.sink_ssc_en;
	else
		*p_enable = false;

	/* write DPCD_00107 = BIT4 when SSC enable */
	status = *p_enable ? BIT(4) : 0;
	ret = drm_dp_dpcd_write(&mtk_dp->aux, DPCD_00107, &status, 0x1);

	if (ret < 0) {
		*p_enable = false;
		dev_err(mtk_dp->dev, "[DPTX] Write DPCD_00107 Fail!!!\n");
		return false;
	}

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] %s SSC via DPCD_00107\n",
		    (*p_enable) ? "Enable" : "Disable ");

	return true;
}

static void mtk_dp_video_mute_sw_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id, bool enable)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] encoder:%d, enable:%d\n", encoder_id, enable);

	WRITE_BYTE_MASK(mtk_dp, REG_304C_DP_ENCODER0_P0 + reg_offset, enable ? BIT(2) : 0, BIT(2));
}

void mtk_dp_video_mute_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id, bool enable)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] encoder:%d, enable:%d\n", encoder_id, enable);

	mtk_dp->info[encoder_id].video_mute = enable;

	mtk_dp_video_mute_sw_v2(mtk_dp, encoder_id, enable);

	if (enable) {
		WRITE_BYTE_MASK(mtk_dp,
				REG_3000_DP_ENCODER0_P0 + reg_offset,
				BIT(3) | BIT(2),
				GENMASK(3,2));
		/* Video mute enable */
		mtk_dp_atf_call_v2(mtk_dp, MTK_DP_SIP_ATF_VIDEO_UNMUTE, 1);
	} else {
		WRITE_BYTE_MASK(mtk_dp,
				REG_3000_DP_ENCODER0_P0 + reg_offset,
				BIT(3),
				GENMASK(3,2));
		/* [3] Sw ov Mode [2] mute value */
		mtk_dp_atf_call_v2(mtk_dp, MTK_DP_SIP_ATF_VIDEO_UNMUTE, 0);
	}

	WRITE_BYTE_MASK(mtk_dp, 0x402c, 0, BIT(4));
	WRITE_BYTE_MASK(mtk_dp, 0x402c, 1, BIT(4));

	msleep(100);
}

static void mtk_dp_sdp_set_asp_count_init_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id)
{
	u16 down_asp = 0x0000;
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	mtk_dp->info[encoder_id].dp_output_timing.hbk =
		mtk_dp->info[encoder_id].dp_output_timing.htt -
		mtk_dp->info[encoder_id].dp_output_timing.hde;

	if (mtk_dp->info[encoder_id].dp_output_timing.pixcel_rate > 0) {
		if (mtk_dp->training_info.link_rate <= DP_LINK_RATE_HBR3)
			down_asp =
				div_u64((u64)mtk_dp->info[encoder_id].dp_output_timing.hbk *
						mtk_dp->training_info.link_rate * 27 * 250 * 4,
					mtk_dp->info[encoder_id].dp_output_timing.pixcel_rate * 5);
		else
			down_asp =
				div_u64((u64)mtk_dp->info[encoder_id].dp_output_timing.hbk *
						mtk_dp->training_info.link_rate * 10 * 1000 * 250 * 4,
					mtk_dp->info[encoder_id].dp_output_timing.pixcel_rate) * 32 * 5;
	}

	/* [11 : 0] reg_sdp_down_asp_cnt_init */
	WRITE_2BYTE_MASK(mtk_dp, REG_3374_DP_ENCODER1_P0 + reg_offset,
			 down_asp << SDP_DOWN_ASP_CNT_INIT_DP_ENCODER1_P0_FLDMASK_POS,
			 SDP_DOWN_ASP_CNT_INIT_DP_ENCODER1_P0_FLDMASK);
}

static u32 mtk_dp_calculate_sdp_down_cnt_v2(struct mtk_dp *mtk_dp, u32 sdp_down_cnt)

{
	u32 down_cnt = 0;

	switch (mtk_dp->training_info.link_lane_count) {
	case DP_1LANE:
		down_cnt = max(sdp_down_cnt, 0x1e);
		break;

	case DP_2LANE:
		down_cnt = max(sdp_down_cnt, 0x14);
		break;

	case DP_4LANE:
		down_cnt = max(sdp_down_cnt, 0x08);
		break;

	default:
		down_cnt = max(sdp_down_cnt, 0x08);
		break;
	}

	return down_cnt;
}

static void mtk_dp_sdp_set_down_cnt_init_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id,
				  u16 sram_read_start)
{
	u32 sdp_down_cnt = 0;
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	/* sram_read_start * lane_cnt * 2(pixelperaddr) * link_rate / pixel_clock * 0.8(margin) */
	sdp_down_cnt = div_u64((u64)sram_read_start * mtk_dp->training_info.link_lane_count * 2 *
					mtk_dp->training_info.link_rate * 2700 * 8,
				mtk_dp->info[encoder_id].dp_output_timing.pixcel_rate);

	if (mtk_dp->info[encoder_id].format == DP_PIXELFORMAT_YUV420)
		sdp_down_cnt = sdp_down_cnt / 2;

	sdp_down_cnt = mtk_dp_calculate_sdp_down_cnt_v2(mtk_dp, sdp_down_cnt);

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] pixcel_rate:%llu sdp_down_cnt:%x\n",
		    mtk_dp->info[encoder_id].dp_output_timing.pixcel_rate, sdp_down_cnt);

	/* [11 : 0]REG_sdp_down_cnt_init */
	WRITE_2BYTE_MASK(mtk_dp, REG_3040_DP_ENCODER0_P0 + reg_offset, sdp_down_cnt, GENMASK(11,0));
}

static void mtk_dp_sdp_set_down_cnt_init_in_hblanking_v2(struct mtk_dp *mtk_dp,
					       const enum dp_encoder_id encoder_id)
{
	u32 sdp_down_cnt;
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	/* hblank * link_rate / pixel_clock * 0.8(margin) / 4(1T4B) */
	sdp_down_cnt = div_u64((u64)(mtk_dp->info[encoder_id].dp_output_timing.htt -
					mtk_dp->info[encoder_id].dp_output_timing.hde) *
					mtk_dp->training_info.link_rate * 2700 * 2,
				mtk_dp->info[encoder_id].dp_output_timing.pixcel_rate);

	drm_dbg_kms(mtk_dp->drm_dev,
		    "[DPTX] [%d] htt:%d, hde:%d, link_rate:%d, pixcel_rate:%llu, color_format:0x%x\n",
		    encoder_id,
		    mtk_dp->info[encoder_id].dp_output_timing.htt,
		    mtk_dp->info[encoder_id].dp_output_timing.hde,
		    mtk_dp->training_info.link_rate,
		    mtk_dp->info[encoder_id].dp_output_timing.pixcel_rate,
		    mtk_dp->info[encoder_id].format);

	if (mtk_dp->info[encoder_id].format == DP_PIXELFORMAT_YUV420)
		sdp_down_cnt = sdp_down_cnt / 2;

	sdp_down_cnt = mtk_dp_calculate_sdp_down_cnt_v2(mtk_dp, sdp_down_cnt);

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] sdp_down_cnt_blank:%x\n", sdp_down_cnt);

	/* [11 : 0]REG_sdp_down_cnt_init_in_hblank */
	WRITE_2BYTE_MASK(mtk_dp, REG_3364_DP_ENCODER1_P0 + reg_offset, sdp_down_cnt, GENMASK(11,0));
}

static void mtk_dp_sdp_path_reset_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	WRITE_2BYTE_MASK(mtk_dp, (REG_3004_DP_ENCODER0_P0 + reg_offset),
			 (0x1 << SDP_RESET_SW_DP_ENCODER0_P0_FLDMASK_POS),
			 SDP_RESET_SW_DP_ENCODER0_P0_FLDMASK);
	udelay(5);

	WRITE_2BYTE_MASK(mtk_dp, (REG_3004_DP_ENCODER0_P0 + reg_offset),
			 (0x0 << SDP_RESET_SW_DP_ENCODER0_P0_FLDMASK_POS),
			 SDP_RESET_SW_DP_ENCODER0_P0_FLDMASK);
}

static void mtk_dp_tu_set_sram_rd_start_v2(struct mtk_dp *mtk_dp,
				 const enum dp_encoder_id encoder_id, u16 val)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	/* [5:0]video sram start address */
	WRITE_BYTE_MASK(mtk_dp, REG_303C_DP_ENCODER0_P0 + reg_offset, val, GENMASK(5,0));
}

static void mtk_dp_tu_set_encoder_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	WRITE_BYTE_MASK(mtk_dp, REG_303C_DP_ENCODER0_P0 + 1 + reg_offset, BIT(7), BIT(7));
	WRITE_2BYTE(mtk_dp, REG_3040_DP_ENCODER0_P0 + reg_offset, 0x2020);
	WRITE_2BYTE_MASK(mtk_dp, REG_3364_DP_ENCODER1_P0 + reg_offset, 0x2020, GENMASK(11,0));
	WRITE_BYTE_MASK(mtk_dp, REG_3300_DP_ENCODER1_P0 + 1 + reg_offset, 0x02, GENMASK(1,0));
	WRITE_BYTE_MASK(mtk_dp, REG_3364_DP_ENCODER1_P0 + 1 + reg_offset, 0x40, GENMASK(6,4));
}

static void mtk_dp_tu_set_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id)
{
	int tu_size = 0;
	int n_value = 0;
	int f_value = 0;
	int pixcel_rate = 0;
	u8 color_bpp;
	u16 sram_read_start = 0;

	color_bpp = mtk_dp_color_get_bpp_v2(mtk_dp->info[encoder_id].format,
					 mtk_dp->info[encoder_id].depth);
	pixcel_rate = div_u64(mtk_dp->info[encoder_id].dp_output_timing.pixcel_rate, 1000);
	tu_size = (640 * (pixcel_rate) * color_bpp) /
			(mtk_dp->training_info.link_rate * 27 *
			mtk_dp->training_info.link_lane_count * 8);

	n_value = tu_size / 10;
	f_value = tu_size % 10;

	drm_dbg_kms(mtk_dp->drm_dev,
		    "[DPTX] tu_size:%d n_value:%d f_value:%d\n", tu_size, n_value, f_value);
	if (mtk_dp->training_info.link_lane_count > 0) {
		sram_read_start = mtk_dp->info[encoder_id].dp_output_timing.hde /
			(mtk_dp->training_info.link_lane_count * 4 * 2 * 2);
		sram_read_start =
			(sram_read_start < DP_TBC_BUF_READ_START_ADR_THRD) ?
			sram_read_start : DP_TBC_BUF_READ_START_ADR_THRD;
		mtk_dp_tu_set_sram_rd_start_v2(mtk_dp, encoder_id, sram_read_start);
	}

	mtk_dp_tu_set_encoder_v2(mtk_dp, encoder_id);
	mtk_dp_audio_sample_arrange_v2(mtk_dp, encoder_id, true);
	mtk_dp_sdp_set_down_cnt_init_in_hblanking_v2(mtk_dp, encoder_id);
	mtk_dp_sdp_set_down_cnt_init_v2(mtk_dp, encoder_id, sram_read_start);
	mtk_dp_sdp_set_asp_count_init_v2(mtk_dp, encoder_id);
}

static bool mtk_dp_swingt_set_pre_emphasis_v2(struct mtk_dp *mtk_dp,
					      enum dp_lane_num lane_num,
					      enum dp_swing_num swing_level,
					      enum dp_preemphasis_num pre_emphasis_level)
{
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] lane:%d, set Swing:0x%x, Emp:0x%x\n",
	        lane_num, swing_level, pre_emphasis_level);

	if (lane_num < DP_LANE0 || lane_num > DP_LANE3) {
		dev_err(mtk_dp->dev, "[DPTX] lane number is error\n");
		return false;
	}

	PHY_WRITE_4BYTE_MASK(mtk_dp, mtk_dp->data->phyd_dig_lan_offset[lane_num] + DRIVING_FORCE,
			     (swing_level << DP_TX_FORCE_VOLT_SWING_VAL_FLDMASK_POS),
			     DP_TX_FORCE_VOLT_SWING_VAL_FLDMASK);
	PHY_WRITE_4BYTE_MASK(mtk_dp, mtk_dp->data->phyd_dig_lan_offset[lane_num] + DRIVING_FORCE,
			     (pre_emphasis_level << DP_TX_FORCE_PRE_EMPH_VAL_FLDMASK_POS),
			     DP_TX_FORCE_PRE_EMPH_VAL_FLDMASK);

	return true;
}

static void mtk_dp_enable_video_interlance_v2(struct mtk_dp *mtk_dp,
					      const enum dp_encoder_id encoder_id)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	WRITE_BYTE_MASK(mtk_dp, REG_3030_DP_ENCODER0_P0 + 1 + reg_offset,
			BIT(6) | BIT(5), GENMASK(6,5));
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] DP imode force-ov\n");
}

static void mtk_dp_disable_video_interlance_v2(struct mtk_dp *mtk_dp,
					       const enum dp_encoder_id encoder_id)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	WRITE_BYTE_MASK(mtk_dp, REG_3030_DP_ENCODER0_P0 + 1 + reg_offset,
			BIT(6), GENMASK(6,5));
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] DP pmode force-ov\n");
}

static void mtk_dp_set_lane_count_v2(struct mtk_dp *mtk_dp, const enum dp_lane_count lane_count)
{
	const u8 value = lane_count >> 1;
	enum dp_encoder_id encoder_id;
	u32 reg_offset;

	if (value == 0) {
		WRITE_BYTE_MASK(mtk_dp, REG_35F0_DP_TRANS_P0, 0, GENMASK(3,2));
	} else if (value < mtk_dp->training_info.max_link_lane_count) {
		WRITE_BYTE_MASK(mtk_dp, REG_35F0_DP_TRANS_P0, BIT(3), GENMASK(3,2));
	} else {
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] Un-expected lane count:%d\n", lane_count);
		return;
	}

	for (encoder_id = 0; encoder_id < mtk_dp->data->encoder_num; encoder_id++) {
		reg_offset = DP_REG_OFFSET(encoder_id);

		WRITE_BYTE_MASK(mtk_dp, REG_3000_DP_ENCODER0_P0 + reg_offset,
				value << LANE_NUM_DP_ENCODER0_P0_FLDMASK_POS,
				LANE_NUM_DP_ENCODER0_P0_FLDMASK);
	}

	WRITE_BYTE_MASK(mtk_dp, REG_34A4_DP_TRANS_P0,
			value << LANE_NUM_DP_TRANS_P0_FLDMASK_POS,
			LANE_NUM_DP_TRANS_P0_FLDMASK);
}

static void mtk_dp_set_training_pattern_v2(struct mtk_dp *mtk_dp, int value)
{
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] Set Train Pattern:0x%x\n", value);

	if (value <= DP_TPS4) {
		if (value == DP_TPS1) /* if Set TPS1 */
			mtk_dp_phy_set_idle_pattern_v2(mtk_dp, false);

		WRITE_BYTE_MASK(mtk_dp, (REG_3400_DP_TRANS_P0 + 1), value, GENMASK(7, 4));
	}
	mdelay(20);
}

static void mtk_dp_set_enhanced_frame_mode_v2(struct mtk_dp *mtk_dp, bool enable)
{
	enum dp_encoder_id encoder_id;
	u32 reg_offset;

	for (encoder_id = 0; encoder_id < mtk_dp->data->encoder_num; encoder_id++) {
		reg_offset = DP_REG_OFFSET(encoder_id);

		if (enable)
			/* [4] enhanced_frame_mode [1 : 0] lane_num */
			WRITE_BYTE_MASK(mtk_dp, REG_3000_DP_ENCODER0_P0 + reg_offset,
					BIT(4), BIT(4));
		else
			/* [4] enhanced_frame_mode [1 : 0] lane_num */
			WRITE_BYTE_MASK(mtk_dp, REG_3000_DP_ENCODER0_P0 + reg_offset, 0, BIT(4));
	}
}

static void mtk_dp_set_scramble_v2(struct mtk_dp *mtk_dp, bool  enable)
{
	WRITE_BYTE_MASK(mtk_dp, REG_3404_DP_TRANS_P0, enable ? BIT(0) : 0, BIT(0));
}

static void mtk_dp_set_misc_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id)
{
	u8 format, depth;
	union dp_misc DP_MISC;
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	format = mtk_dp->info[encoder_id].format;
	depth = mtk_dp->info[encoder_id].depth;

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] format:0x%x, depth:0x%x\n", format, depth);

	/* MISC0[7:5] color depth */
	DP_MISC.misc.color_depth = depth;

	/* MISC0[3]: 0->RGB, 1->YUV */
	/* MISC0[2:1]: 01b->4:2:2, 10b->4:4:4 */
	switch (format) {
	case DP_PIXELFORMAT_YUV444:
		DP_MISC.misc.color_format = 0x2;
		DP_MISC.misc.spec_def1 = 0x1;
		break;

	case DP_PIXELFORMAT_YUV422:
		DP_MISC.misc.color_format = 0x1;
		DP_MISC.misc.spec_def1 = 0x1;
		break;

	case DP_PIXELFORMAT_YUV420:
		/* not support */
		break;

	case DP_PIXELFORMAT_RAW:
		DP_MISC.misc.color_format = 0x1;
		DP_MISC.misc.spec_def2 = 0x1;
		break;
	case DP_PIXELFORMAT_Y_ONLY:
		DP_MISC.misc.color_format = 0x0;
		DP_MISC.misc.spec_def2 = 0x1;
		break;

	case DP_PIXELFORMAT_RGB:
	default:
		DP_MISC.misc.color_format = 0x0;
		DP_MISC.misc.spec_def2 = 0x0;
		break;
	}

	WRITE_BYTE_MASK(mtk_dp, REG_3034_DP_ENCODER0_P0 + reg_offset, DP_MISC.misc_raw[0], GENMASK(7,1));
	WRITE_BYTE_MASK(mtk_dp, REG_3034_DP_ENCODER0_P0 + 1 + reg_offset,
			DP_MISC.misc_raw[1], GENMASK(7, 0));
}

static void mtk_dp_set_output_timing_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id)
{
	if (mtk_dp->info[encoder_id].dp_output_timing.video_ip_mode == DP_VIDEO_INTERLACE)
		mtk_dp_enable_video_interlance_v2(mtk_dp, encoder_id);
	else
		mtk_dp_disable_video_interlance_v2(mtk_dp, encoder_id);

	mtk_dp_msa_set_v2(mtk_dp, encoder_id);
}

static void mtk_dp_video_config_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id);
static void mtk_dp_set_dp_out_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id)
{
	mtk_dp_video_config_v2(mtk_dp, encoder_id);
	mtk_dp_msa_enable_bypass_v2(mtk_dp, encoder_id, false);
	mtk_dp_set_output_timing_v2(mtk_dp, encoder_id);
	mtk_dp_mn_calculate_v2(mtk_dp, encoder_id);

	mtk_dp_pg_enable_v2(mtk_dp, encoder_id, false);
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] Set dpintf output\n");

	mtk_dp_tu_set_v2(mtk_dp, encoder_id);
}

static void mtk_dp_verify_clock_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id)
{
	u64 m, n, ls_clk, pix_clk, fs;
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	n = 0x8000;
	ls_clk = mtk_dp->training_info.link_rate * 27;

	m = READ_4BYTE(mtk_dp, REG_33C8_DP_ENCODER1_P0 + reg_offset);
	pix_clk = div_u64(m * ls_clk * 1000 * 1000, n);
	drm_dbg_kms(mtk_dp->drm_dev,
		    "[DPTX] [%d] video M:0x%llx, DP calc pixel clock:%llu Hz, dp_intf clock:%llu Hz\n",
	            encoder_id, m, pix_clk, pix_clk / 4);

	m = READ_4BYTE(mtk_dp, REG_33D0_DP_ENCODER1_P0);
	fs = div_u64( m * ls_clk * 1000 * 1000, n * 512);
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] [%d] audio M:0x%llx, fs:%llu\n", encoder_id, m, fs);
}

void mtk_dp_video_enable_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id)
{
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] Output Video[%d] enable\n", encoder_id);

	mtk_dp_set_dp_out_v2(mtk_dp, encoder_id);
	mtk_dp_verify_clock_v2(mtk_dp, encoder_id);
}

static void mtk_dp_stop_sent_sdp_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id);
void mtk_dp_video_disable_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id)
{
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] Output Video[%d] disable\n", encoder_id);

	mtk_dp_stop_sent_sdp_v2(mtk_dp, encoder_id);
	mtk_dp_sdp_path_reset_v2(mtk_dp, encoder_id);

	mtk_dp_dsc_disable_v2(mtk_dp, encoder_id);
}

static void mtk_dp_video_config_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id)
{
	struct dp_timing_parameter *dp_timing = &mtk_dp->info[encoder_id].dp_output_timing;
	struct videomode vm = {0};
	int con_id;
	u8 data;

	if (!mtk_dp->dp_ready) {
		dev_err(mtk_dp->dev, "[DPTX] %s, DP is not ready\n", __func__);
		return;
	}

	data = mtk_dp->dsc_enable[encoder_id];

	mtk_dp_mn_overwrite_v2(mtk_dp, encoder_id, false, 0x0, 0x8000);

	vm.hactive = mtk_dp->mode[encoder_id].hdisplay;
	vm.hfront_porch = mtk_dp->mode[encoder_id].hsync_start - mtk_dp->mode[encoder_id].hdisplay;
	vm.hsync_len = mtk_dp->mode[encoder_id].hsync_end - mtk_dp->mode[encoder_id].hsync_start;
	vm.hback_porch = mtk_dp->mode[encoder_id].htotal - mtk_dp->mode[encoder_id].hsync_end;
	vm.vactive = mtk_dp->mode[encoder_id].vdisplay;
	vm.vfront_porch = mtk_dp->mode[encoder_id].vsync_start - mtk_dp->mode[encoder_id].vdisplay;
	vm.vsync_len = mtk_dp->mode[encoder_id].vsync_end - mtk_dp->mode[encoder_id].vsync_start;
	vm.vback_porch = mtk_dp->mode[encoder_id].vtotal - mtk_dp->mode[encoder_id].vsync_end;
	vm.pixelclock = mtk_dp->mode[encoder_id].clock * 1000;

	dp_timing->frame_rate = mtk_dp->mode[encoder_id].clock * 1000 /
				mtk_dp->mode[encoder_id].htotal / mtk_dp->mode[encoder_id].vtotal;
	dp_timing->htt = mtk_dp->mode[encoder_id].htotal;
	dp_timing->hbp = vm.hback_porch;
	dp_timing->hsw = vm.hsync_len;
	dp_timing->hsp = mtk_dp->mode[encoder_id].flags & DRM_MODE_FLAG_PHSYNC ? 0 : 1;
	dp_timing->hfp = vm.hfront_porch;
	dp_timing->hde = vm.hactive;
	dp_timing->vtt = mtk_dp->mode[encoder_id].vtotal;
	dp_timing->vbp = vm.vback_porch;
	dp_timing->vsw = vm.vsync_len;
	dp_timing->vsp = mtk_dp->mode[encoder_id].flags & DRM_MODE_FLAG_PVSYNC ? 0 : 1;
	dp_timing->vfp = vm.vfront_porch;
	dp_timing->vde = vm.vactive;

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] frame_rate:%d\n", dp_timing->frame_rate);
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] htt:%d\n", dp_timing->htt);
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] hbp:%d\n", dp_timing->hbp);
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] hsw:%d\n", dp_timing->hsw);
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] hsp:%d\n", dp_timing->hsp);
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] hfp:%d\n", dp_timing->hfp);
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] hde:%d\n", dp_timing->hde);
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] vtt:%d\n", dp_timing->vtt);
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] vbp:%d\n", dp_timing->vbp);
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] vsw:%d\n", dp_timing->vsw);
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] vsp:%d\n", dp_timing->vsp);
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] vfp:%d\n", dp_timing->vfp);
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] vde:%d\n", dp_timing->vde);

	mtk_dp_mn_calculate_v2(mtk_dp, encoder_id);

	if (mtk_dp->dsc_enable[encoder_id])
		mtk_dp_mn_overwrite_v2(mtk_dp, encoder_id, true,
				       mtk_dp->info[encoder_id].video_m,
				       mtk_dp->info[encoder_id].video_n);

	/* interlace not support */
	dp_timing->video_ip_mode = DP_VIDEO_PROGRESSIVE;
	mtk_dp_msa_set_v2(mtk_dp, encoder_id);

	mtk_dp_set_misc_v2(mtk_dp, encoder_id);
	if (mtk_dp->info[encoder_id].pattern_gen)
		mtk_dp_pg_type_sel_v2(mtk_dp, encoder_id,
				      DP_PG_VERTICAL_COLOR_BAR,
				      DP_PG_PURECOLOR_BLUE,
				      0xfff,
				      DP_PG_LOCATION_ALL);

	if (!mtk_dp->dsc_enable[encoder_id]) {
		mtk_dp_color_set_depth_v2(mtk_dp, encoder_id, mtk_dp->info[encoder_id].depth);
		mtk_dp_color_set_format_v2(mtk_dp, encoder_id, mtk_dp->info[encoder_id].format);
	} else {
		mtk_dp_dsc_pps_send_v2(mtk_dp, encoder_id);
		mtk_dp_dsc_enable_v2(mtk_dp, encoder_id);
	}

	if (mtk_dp->mst_enable) {
		con_id = encoder_id_to_con_id(mtk_dp, encoder_id, DRM_DP_MST);
		if (con_id < 0)
			return;

		if (drm_dp_sink_supports_dsc(mtk_dp->mtk_con[con_id]->dsc_dpcd))
			set_dsc_decompression_flag_v2(mtk_dp->mtk_con[con_id]->dsc_aux,
						      DP_DECOMPRESSION_EN,
						      mtk_dp->dsc_enable[encoder_id]);
	} else {
		if (drm_dp_sink_supports_dsc(mtk_dp->mtk_con[DP_FIRST_CON]->dsc_dpcd))
			set_dsc_decompression_flag_v2(&mtk_dp->aux,
						      DP_DECOMPRESSION_EN,
						      mtk_dp->dsc_enable[encoder_id]);
	}
}

static void mtk_dp_stop_sent_sdp_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id)
{
	u8 pkg_type;

	for (pkg_type = DP_SDP_PKG_ACM; pkg_type < DP_SDP_PKG_MAX_NUM; pkg_type++)
		mtk_dp_spkg_sdp_v2(mtk_dp, encoder_id, false, pkg_type, NULL, NULL);

	mtk_dp_spkg_vsc_ext_vesa_v2(mtk_dp, encoder_id, false, 0x00, NULL);
	mtk_dp_spkg_vsc_ext_cea_v2(mtk_dp, encoder_id, false, 0x00, NULL);
}

static u8 mtk_dp_get_sink_count_v2(struct mtk_dp *mtk_dp)
{
	u8 tmp = 0;
	int ret;

	ret = drm_dp_dpcd_read(&mtk_dp->aux, DPCD_00200, &tmp, 0x1);

	if (ret < 0) {
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] Failed to read DPCD: %d\n", ret);
		return 0;
	}

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] sink count:%d\n", DP_GET_SINK_COUNT(tmp));
	return DP_GET_SINK_COUNT(tmp);
}

static bool mtk_dp_hpd_get_pin_level_v2(struct mtk_dp *mtk_dp)
{
	bool ret = ((READ_2BYTE(mtk_dp, REG_364C_AUX_TX_P0) &
		    HPD_STATUS_AUX_TX_P0_FLDMASK) >>
		    HPD_STATUS_AUX_TX_P0_FLDMASK_POS);

	return ret;
}

static bool mtk_dp_check_sink_cap_v2(struct mtk_dp *mtk_dp)
{
	u8 tmp[0x10];
	int ret;

	if (!mtk_dp_hpd_get_pin_level_v2(mtk_dp))
		return false;

	memset(tmp, 0x0, sizeof(tmp));

	ret = drm_dp_dpcd_read(&mtk_dp->aux, DPCD_00000, tmp, 0x10);
	if (ret < 0)
		return false;

	mtk_dp->training_info.sink_ext_cap_en = (tmp[0x0E] & BIT(7)) ?
		true : false;
	if (mtk_dp->training_info.sink_ext_cap_en) {
		ret = drm_dp_dpcd_read(&mtk_dp->aux, DPCD_02200, tmp, 0x10);
		if (ret < 0)
			return false;
	}

	mtk_dp->training_info.dpcd_rev = tmp[0x0];
	drm_dbg_kms(mtk_dp->drm_dev,
		    "[DPTX] SINK DPCD version:0x%x\n", mtk_dp->training_info.dpcd_rev);

	if (drm_dp_read_dpcd_caps(&mtk_dp->aux, mtk_dp->rx_cap) != 0)
		return false;

	if (mtk_dp->training_info.dpcd_rev >= 0x14) {
		mtk_dp_fec_ready_v2(mtk_dp);
		mtk_dp_dsc_support_v2(mtk_dp);
	}

	mtk_dp->training_info.tps3_support = (tmp[0x2] & BIT(6)) >> 0x6;
	mtk_dp->training_info.tps4_support = (tmp[0x3] & BIT(7)) >> 0x7;

	mtk_dp->training_info.dwn_strm_port_present =
			(tmp[0x5] & BIT(0));

	if ((tmp[0x3] & BIT(0)) == 0x1) {
		mtk_dp->training_info.sink_ssc_en = true;
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] SINK SUPPORT SSC\n");
	} else {
		mtk_dp->training_info.sink_ssc_en = false;
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] SINK NOT SUPPORT SSC\n");
	}

	// 4.2.2.7, Read 80 when DOWN_STREAM_PORT were detected
	// DPCD 00005 or 02205: DOWN_STREAM_PORT_PRESENT
	// DPCD 00007 or 02207: DFP_COUNT
	// For pass DP LL CTS even if no need the information
	if ((tmp[0x5] & BIT(0)) && ((tmp[0x7] & 0x0f) > 0x0))
		drm_dp_dpcd_read(&mtk_dp->aux, DP_DOWNSTREAM_PORT_0, tmp, 0x10);

	ret = drm_dp_dpcd_read(&mtk_dp->aux, DPCD_00021, tmp, 0x1);
	if (ret < 0)
		return false;

	mtk_dp->training_info.dp_mst_cap = (tmp[0x0] & BIT(0));
	mtk_dp->training_info.dp_mst_branch = false;

	if (mtk_dp->training_info.dp_mst_cap == BIT(0)) {
		if (mtk_dp->training_info.dwn_strm_port_present == 0x1)
			mtk_dp->training_info.dp_mst_branch = true;

		ret = drm_dp_dpcd_read(&mtk_dp->aux, DPCD_02003, tmp, 0x1);
		if (ret < 0)
			return false;

		if (tmp[0x0] != 0x0) {
			ret = drm_dp_dpcd_write(&mtk_dp->aux, DPCD_02003,
						tmp, 0x1);
			if (ret < 0)
				return false;
		}

		ret = drm_dp_dpcd_read(&mtk_dp->aux, DP_MSTM_CTRL, tmp, 0x1);
		if (ret < 0)
			return false;
		mtk_dp->training_info.dp_mst_en = (tmp[0x0] & BIT(0));
	}

	drm_dbg_kms(mtk_dp->drm_dev,
		    "[DPTX] mst_cap:%d, mst_en:%d, mst_branch:%d, port_present:%d\n",
		    mtk_dp->training_info.dp_mst_cap,
		    mtk_dp->training_info.dp_mst_en,
		    mtk_dp->training_info.dp_mst_branch,
		    mtk_dp->training_info.dwn_strm_port_present);

	mtk_dp->training_info.sink_count = mtk_dp_get_sink_count_v2(mtk_dp);

	if (!mtk_dp->training_info.dp_mst_branch) {
		u8 dpcd_201 = 0;

		ret = drm_dp_dpcd_read(&mtk_dp->aux, DPCD_00201, &dpcd_201, 1);
		if (ret < 0)
			return false;
	}

	return true;
}

static void mtk_dp_hotplug_uevent_v2(struct mtk_dp *mtk_dp)
{
	if (mtk_dp->drm_dev) {
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] notify drm framework hotplug event\n");
		drm_helper_hpd_irq_event(mtk_dp->drm_dev);
	} else {
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] there is no drm dev\n");
	}
}

static u16 mtk_dp_hpd_get_irq_status_v2(struct mtk_dp *mtk_dp)
{
	return READ_2BYTE(mtk_dp, REG_3608_AUX_TX_P0);
}

static void mtk_dp_hpd_interrupt_clr_v2(struct mtk_dp *mtk_dp, u16 status)
{
	WRITE_2BYTE_MASK(mtk_dp, REG_3668_AUX_TX_P0, status, status);
	WRITE_2BYTE_MASK(mtk_dp, REG_3668_AUX_TX_P0, 0, status);

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] HPD ISR status:0x%x\n", mtk_dp_hpd_get_irq_status_v2(mtk_dp));
}

static void mtk_dp_hpd_interrupt_enable_v2(struct mtk_dp *mtk_dp, bool enable)
{
	WRITE_4BYTE_MASK(mtk_dp, DP_TX_TOP_IRQ_MASK,
			 TRANS_IRQ_MSK | ENCODER_IRQ_MSK,
			 TRANS_IRQ_MSK | ENCODER_IRQ_MSK);

	/* [7]:int[6]:Con[5]DisCon[4]No-Use:UnMASK HPD Port */
	if (enable)
		WRITE_2BYTE_MASK(mtk_dp, REG_3660_AUX_TX_P0, 0x0,
				 HPD_DISCONNECT | HPD_CONNECT | HPD_INT_EVNET);
	else
		WRITE_2BYTE_MASK(mtk_dp, REG_3660_AUX_TX_P0,
				 DP_TX_INT_MASK_AUX_TX_P0_FLDMASK,
				 DP_TX_INT_MASK_AUX_TX_P0_FLDMASK);
}

static void mtk_dp_hpd_detect_setting_v2(struct mtk_dp *mtk_dp)
{
	/* Crystal frequency value for 1us timing normalization */
	/* [7:2]: Integer value */
	/* [1:0]: Fractional value */
	/* 0x30: 12.0us, 0x68: 26us */
	WRITE_2BYTE_MASK(mtk_dp, REG_366C_AUX_TX_P0,
			 0x68 << XTAL_FREQ_AUX_TX_P0_FLDMASK_POS,
			 XTAL_FREQ_AUX_TX_P0_FLDMASK);

	/* Adjust Tx reg_hpd_disc_thd to 2ms, it is because of the spec. "HPD pulse" description */
	/* Low Bound: 3'b010 ~ 500us */
	/* Up Bound: 3'b110 ~1.9ms */
	WRITE_2BYTE_MASK(mtk_dp, REG_364C_AUX_TX_P0,
			 (0x32 << HPD_INT_THD_AUX_TX_P0_FLDMASK_POS),
			 HPD_INT_THD_AUX_TX_P0_FLDMASK);
}

static void mtk_dp_init_variable_v2(struct mtk_dp *mtk_dp)
{
	enum dp_encoder_id encoder_id;

	mtk_dp->training_info.dp_version = DP_VER_14;
	mtk_dp->training_info.max_link_rate = mtk_dp->data->max_link_rate;
	mtk_dp->training_info.max_link_lane_count = mtk_dp->data->max_lane_count;
	mtk_dp->training_info.sink_ext_cap_en = false;
	mtk_dp->training_info.sink_ssc_en = false;
	mtk_dp->training_info.tps3_support = true;
	mtk_dp->training_info.tps4_support = true;
	mtk_dp->training_info.phy_status = HPD_INITIAL_STATE;
	mtk_dp->training_info.cable_plug_in = false;
	for (encoder_id = 0; encoder_id < mtk_dp->data->encoder_num; encoder_id++) {
		mtk_dp->info[encoder_id].depth = DP_COLOR_DEPTH_8BIT;
		memset(&mtk_dp->info[encoder_id].dp_output_timing, 0,
		       sizeof(struct dp_timing_parameter));
		mtk_dp->info[encoder_id].dp_output_timing.frame_rate = 60;

		mtk_dp->dsc_enable[encoder_id] = false;
	}
	mtk_dp->dp_ready = false;
	mtk_dp->need_debounce = false;
	mtk_dp->audio_enable = false;
}

static void mtk_dp_initial_setting_v2(struct mtk_dp *mtk_dp)
{
	enum dp_encoder_id encoder_id;
	u32 reg_offset;

	WRITE_4BYTE_MASK(mtk_dp, DP_TX_TOP_PWR_STATE,
			 (0x3 << DP_PWR_STATE_FLDMASK_POS), DP_PWR_STATE_FLDMASK);

	WRITE_BYTE(mtk_dp, REG_342C_DP_TRANS_P0, 0x68); /* 26M xtal clock */

	mtk_dp_fec_init_setting_v2(mtk_dp);

	for (encoder_id = 0; encoder_id < mtk_dp->data->encoder_num; encoder_id++) {
		reg_offset = DP_REG_OFFSET(encoder_id);
		WRITE_4BYTE_MASK(mtk_dp, REG_31EC_DP_ENCODER0_P0  + reg_offset, BIT(4), BIT(4));
		WRITE_4BYTE_MASK(mtk_dp, REG_304C_DP_ENCODER0_P0  + reg_offset, 0, BIT(8));
		WRITE_4BYTE_MASK(mtk_dp, REG_304C_DP_ENCODER0_P0  + reg_offset, BIT(3), BIT(3));
	}

	/* 31C4[13] : DSC bypass [11]pps bypass */
	WRITE_2BYTE_MASK(mtk_dp, REG_31C4_DP_ENCODER0_P0 + reg_offset,
			 0,
			 PPS_HW_BYPASS_MASK_DP_ENCODER0_P0_FLDMASK);

	WRITE_2BYTE_MASK(mtk_dp, REG_31C4_DP_ENCODER0_P0 + reg_offset,
			 0,
			 DSC_BYPASS_EN_DP_ENCODER0_P0_FLDMASK);

	WRITE_2BYTE_MASK(mtk_dp, REG_336C_DP_ENCODER1_P0 + reg_offset,
			 0,
			 DSC_BYTE_SWAP_DP_ENCODER1_P0_FLDMASK);
}

static void mtk_dp_encoder_reset_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	/* dp tx encoder reset all sw */
	WRITE_2BYTE_MASK(mtk_dp, (REG_3004_DP_ENCODER0_P0 + reg_offset),
			 1 << DP_TX_ENCODER_4P_RESET_SW_DP_ENCODER0_P0_FLDMASK_POS,
			 DP_TX_ENCODER_4P_RESET_SW_DP_ENCODER0_P0_FLDMASK);
	mdelay(1);

	/* dp tx encoder reset all sw */
	WRITE_2BYTE_MASK(mtk_dp, (REG_3004_DP_ENCODER0_P0 + reg_offset),
			 0,
			 DP_TX_ENCODER_4P_RESET_SW_DP_ENCODER0_P0_FLDMASK);
}

static void mtk_dp_digital_setting_v2(struct mtk_dp *mtk_dp, const enum dp_encoder_id encoder_id)
{
	u32 reg_offset = DP_REG_OFFSET(encoder_id);

	mtk_dp_spkg_asp_hb32_v2(mtk_dp, encoder_id, false, DP_SDP_ASP_HB3_AU02CH, 0x0);
	/* Mengkun suggest: disable reg_sdp_down_cnt_new_mode */
	WRITE_BYTE_MASK(mtk_dp, REG_304C_DP_ENCODER0_P0 + reg_offset, 0,
			SDP_DOWN_CNT_NEW_MODE_DP_ENCODER0_P0_FLDMASK);
	/* reg_sdp_asp_insert_in_hblank: default = 1 */
	WRITE_2BYTE_MASK(mtk_dp, REG_3374_DP_ENCODER1_P0 + reg_offset,
			 0x1 << SDP_ASP_INSERT_IN_HBLANK_DP_ENCODER1_P0_FLDMASK_POS,
			 SDP_ASP_INSERT_IN_HBLANK_DP_ENCODER1_P0_FLDMASK);

	WRITE_BYTE_MASK(mtk_dp, REG_304C_DP_ENCODER0_P0 + reg_offset, 0,
			VBID_VIDEO_MUTE_DP_ENCODER0_P0_FLDMASK);
	/* MISC0 */
	mtk_dp_color_set_format_v2(mtk_dp, encoder_id, mtk_dp->info[encoder_id].format);

	/* [13 : 12] : = 2b'01 VDE check BS2BS & set min value */
	mtk_dp_color_set_depth_v2(mtk_dp, encoder_id, mtk_dp->info[encoder_id].depth);
	WRITE_4BYTE(mtk_dp, REG_3368_DP_ENCODER1_P0 + reg_offset,
		    (0x1 << 15) |
		    (0x4 << BS2BS_MODE_DP_ENCODER1_P0_FLDMASK_POS) |
		    (0x1 << SDP_DP13_EN_DP_ENCODER1_P0_FLDMASK_POS) |
		    (0x1 << VIDEO_STABLE_CNT_THRD_DP_ENCODER1_P0_FLDMASK_POS) |
		    (0x1 << VIDEO_SRAM_FIFO_CNT_RESET_SEL_DP_ENCODER1_P0_FLDMASK_POS));

	mtk_dp_encoder_reset_v2(mtk_dp, encoder_id);
}

static void mtk_dp_digital_sw_reset_v2(struct mtk_dp *mtk_dp)
{
	WRITE_BYTE_MASK(mtk_dp, REG_340C_DP_TRANS_P0 + 1, BIT(5), BIT(5));
	mdelay(1);
	WRITE_BYTE_MASK(mtk_dp, REG_340C_DP_TRANS_P0 + 1, 0, BIT(5));
}

static void mtk_dp_analog_power_on_v2(struct mtk_dp *mtk_dp)
{
	WRITE_BYTE_MASK(mtk_dp, DP_TX_TOP_RESET_AND_PROBE, 0, BIT(4));
	udelay(10);
	WRITE_BYTE_MASK(mtk_dp, DP_TX_TOP_RESET_AND_PROBE, BIT(4), BIT(4));
	WRITE_2BYTE(mtk_dp, TOP_OFFSET, 0x0);
}

static void mtk_dp_analog_power_off_v2(struct mtk_dp *mtk_dp)
{
	udelay(10);
	PHY_WRITE_2BYTE(mtk_dp, 0x0034, 0x4aa);
	PHY_WRITE_2BYTE(mtk_dp, 0x1040, 0x0);
	PHY_WRITE_2BYTE(mtk_dp, 0x0038, 0x555);
}

static void mtk_dp_init_port_v2(struct mtk_dp *mtk_dp)
{
	enum dp_encoder_id encoder_id;

	mtk_dp_phy_set_idle_pattern_v2(mtk_dp, true);
	mtk_dp_init_variable_v2(mtk_dp);

	mtk_dp_fec_disable_v2(mtk_dp);
	for (encoder_id = 0; encoder_id < mtk_dp->data->encoder_num; encoder_id++)
		mtk_dp_dsc_disable_v2(mtk_dp, encoder_id);

	mtk_dp_initial_setting_v2(mtk_dp);
	mtk_dp_aux_setting_v2(mtk_dp);
	for (encoder_id = 0; encoder_id < mtk_dp->data->encoder_num; encoder_id++)
		mtk_dp_digital_setting_v2(mtk_dp, encoder_id);

	mtk_dp_analog_power_on_v2(mtk_dp);
	mtk_dp_phy_setting_v2(mtk_dp);
	mtk_dp_hpd_detect_setting_v2(mtk_dp);

	mtk_dp_digital_sw_reset_v2(mtk_dp);
}

static void mtk_dp_disconnect_release_v2(struct mtk_dp *mtk_dp)
{
	int i;

	if (mtk_dp->mst_enable) {
		mtk_dp_mst_drv_unprepare(mtk_dp);
	} else {
		mtk_dp_video_mute_v2(mtk_dp, DP_SST_ENCODER_PORT, true);
		mtk_dp_audio_mute_v2(mtk_dp, DP_SST_ENCODER_PORT, true);
		mtk_dp_video_disable_v2(mtk_dp, DP_SST_ENCODER_PORT);
		mtk_dp->mtk_con[DP_FIRST_CON]->video_enable = false;
	}

	memset(mtk_dp->mtk_con[DP_FIRST_CON]->dsc_dpcd, 0,
	       sizeof(mtk_dp->mtk_con[DP_FIRST_CON]->dsc_dpcd));
	mtk_dp->mtk_con[DP_FIRST_CON]->fec_cap = 0;
	kfree(mtk_dp->mtk_con[DP_FIRST_CON]->edid);
	mtk_dp->mtk_con[DP_FIRST_CON]->edid = NULL;

	mtk_dp_hdcp_disable(mtk_dp);

	mtk_dp_audio_update_plugged_status_v2(mtk_dp);

	mtk_dp_phy_set_idle_pattern_v2(mtk_dp, true);
	mtk_dp_fec_disable_v2(mtk_dp);

	for (i = 0; i < mtk_dp->data->encoder_num; i++)
		mtk_dp_dsc_disable_v2(mtk_dp, i);

	mtk_dp_analog_power_off_v2(mtk_dp);

	mtk_dp_init_variable_v2(mtk_dp);
}

void mtk_dp_hdcp_update_value(struct mtk_dp *mtk_dp)
{
	schedule_work(&mtk_dp->prop_work);
}

static void mtk_dp_hdcp_get_info(struct mtk_dp *mtk_dp)
{
	mtk_dp_hdcp2x_get_info(&mtk_dp->hdcp_info);
	mtk_dp_hdcp1x_get_info(&mtk_dp->hdcp_info);
}

static void mtk_dp_hdcp_disable_handle(struct work_struct *data)
{
	struct mtk_dp *mtk_dp = container_of(data, struct mtk_dp, hdcp_disable_work);

	mutex_lock(&mtk_dp->hdcp_mutex);

	if (mtk_dp->hdcp_info.auth_status == AUTH_ZERO)
		goto end;

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] [HDCP] disable HDCP\n");

	if (mtk_dp->hdcp_info.auth_version == HDCP_VERSION_2X)
		mtk_dp_hdcp2x_disable(&mtk_dp->hdcp_info);
	else if (mtk_dp->hdcp_info.auth_version == HDCP_VERSION_1X)
		mtk_dp_hdcp1x_disable(&mtk_dp->hdcp_info);

	mtk_dp_hdcp_update_value(mtk_dp);

end:
	mutex_unlock(&mtk_dp->hdcp_mutex);

	cancel_delayed_work_sync(&mtk_dp->check_work);
}

void mtk_dp_hdcp_disable(struct mtk_dp *mtk_dp)
{
	queue_work(mtk_dp->hdcp_workqueue, &mtk_dp->hdcp_disable_work);
}

static void mtk_dp_hdcp_check_work(struct work_struct *work)
{
	struct mtk_dp *mtk_dp = container_of(to_delayed_work(work),
		struct mtk_dp, check_work);

	if (mtk_dp->hdcp_info.auth_version == HDCP_VERSION_2X &&
	    (!mtk_dp_hdcp2x_check_link(&mtk_dp->hdcp_info))) {
		schedule_delayed_work(&mtk_dp->check_work, DRM_HDCP2_CHECK_PERIOD_MS);
	} else if (mtk_dp->hdcp_info.auth_version == HDCP_VERSION_1X &&
		(!mtk_dp_hdcp1x_check_link(&mtk_dp->hdcp_info))) {
		schedule_delayed_work(&mtk_dp->check_work, DRM_HDCP_CHECK_PERIOD_MS);
	}
}

bool mtk_dp_hdcp_need_hdcp(struct mtk_dp *mtk_dp)
{
	bool need_hdcp = false;
	u8 i;

	if (!mtk_dp->mst_enable) {
		if (mtk_dp->mtk_con[DP_FIRST_CON]->video_enable)
			need_hdcp = mtk_dp->con_state[DP_SST_ENCODER_PORT].content_protection !=
				DRM_MODE_CONTENT_PROTECTION_UNDESIRED;
	} else {
		for (i = 0; i < ARRAY_SIZE(mtk_dp->mtk_con); i++) {
			if (mst_con_with_encoder(mtk_dp->mtk_con[i]) &&
				mtk_dp->mtk_con[i]->video_enable &&
				mtk_dp->con_state[mtk_dp->mtk_con[i]->encoder_id].content_protection !=
				DRM_MODE_CONTENT_PROTECTION_UNDESIRED) {
				need_hdcp = true;
				break;
			}
		}
	}

	return need_hdcp;
}

static bool mtk_dp_hdcp_need_content_type1(struct mtk_dp *mtk_dp)
{
	bool need_type1 = false;
	u8 i;

	if (!mtk_dp->mst_enable) {
		if (mtk_dp->mtk_con[DP_FIRST_CON]->video_enable)
			need_type1 = mtk_dp->con_state[DP_SST_ENCODER_PORT].hdcp_content_type ==
				DRM_MODE_HDCP_CONTENT_TYPE1;
	} else {
		for (i = 0; i < ARRAY_SIZE(mtk_dp->mtk_con); i++) {
			if (mst_con_with_encoder(mtk_dp->mtk_con[i]) &&
				mtk_dp->mtk_con[i]->video_enable &&
				mtk_dp->con_state[mtk_dp->mtk_con[i]->encoder_id].hdcp_content_type ==
				DRM_MODE_HDCP_CONTENT_TYPE1) {
				need_type1 = true;
				break;
			}
		}
	}

	return need_type1;
}

static void mtk_dp_hdcp_enable_handle(struct work_struct *data)
{
	struct mtk_dp *mtk_dp = container_of(data, struct mtk_dp, hdcp_enable_work);
	unsigned long check_link_interval = DRM_HDCP_CHECK_PERIOD_MS;
	bool need_type1;
	int ret = -EINVAL;

	mutex_lock(&mtk_dp->hdcp_mutex);

	need_type1 = mtk_dp_hdcp_need_content_type1(mtk_dp);

	if (mtk_dp->hdcp_info.auth_status == AUTH_PASS &&
		need_type1 && mtk_dp->hdcp_info.auth_version == HDCP_VERSION_1X) {
		mtk_dp_hdcp1x_disable(&mtk_dp->hdcp_info);
	}

	if (mtk_dp->hdcp_info.auth_status == AUTH_PASS)
		goto end;

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] [HDCP] start HDCP work\n");

	mtk_dp_hdcp_get_info(mtk_dp);

	if (mtk_dp->hdcp_info.hdcp2_info.capable) {
		ret = mtk_dp_hdcp2x_enable(&mtk_dp->hdcp_info);
		if (!ret)
			check_link_interval = DRM_HDCP2_CHECK_PERIOD_MS;
	}

	if (!need_type1 && ret && mtk_dp->hdcp_info.hdcp1x_info.capable)
		ret = mtk_dp_hdcp1x_enable(&mtk_dp->hdcp_info);

	if (ret)
		goto end;

	if (mtk_dp->mst_enable)
		mtk_dp_mst_drv_set_hdcp_setting(mtk_dp);

	schedule_delayed_work(&mtk_dp->check_work, check_link_interval);
	mtk_dp_hdcp_update_value(mtk_dp);
end:
	mutex_unlock(&mtk_dp->hdcp_mutex);
}

void mtk_dp_hdcp_enable(struct mtk_dp *mtk_dp)
{
	if (!mtk_dp->training_info.cable_plug_in || !mtk_dp->dp_ready) {
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] [HDCP] cable_plug_in:%d, dp_ready:%d",
			    mtk_dp->training_info.cable_plug_in, mtk_dp->dp_ready);
		return;
	}

	queue_work(mtk_dp->hdcp_workqueue, &mtk_dp->hdcp_enable_work);
}

static void mtk_dp_hdcp_update_content_protection(struct mtk_dp *mtk_dp, u8 con_id, u8 encoder_id)
{
	struct drm_device *drm_dev;
	u8 update_cp;

	update_cp = mtk_dp->hdcp_info.auth_status == AUTH_PASS ?
				DRM_MODE_CONTENT_PROTECTION_ENABLED :
				DRM_MODE_CONTENT_PROTECTION_DESIRED;

	drm_dev = mtk_dp->mtk_con[con_id]->connector.dev;

	drm_modeset_lock(&drm_dev->mode_config.connection_mutex, NULL);

	/* only update between ENABLED/DESIRED */
	if (mtk_dp->con_state[encoder_id].content_protection !=
		DRM_MODE_CONTENT_PROTECTION_UNDESIRED) {
		drm_dbg_kms(mtk_dp->drm_dev,
			    "[DPTX] [HDCP][%d] con:%d, cp:%d, ct:%d, status:%d, version:%d\n",
			    encoder_id, con_id,
			    mtk_dp->con_state[encoder_id].content_protection,
			    mtk_dp->con_state[encoder_id].content_type,
			    mtk_dp->hdcp_info.auth_status, mtk_dp->hdcp_info.auth_version);

		drm_hdcp_update_content_protection(&mtk_dp->mtk_con[con_id]->connector, update_cp);
	}

	drm_modeset_unlock(&drm_dev->mode_config.connection_mutex);
}

static void mtk_dp_hdcp_prop_work(struct work_struct *work)
{
	struct mtk_dp *mtk_dp;
	u8 i;

	mtk_dp = container_of(work, struct mtk_dp, prop_work);

	if (!mtk_dp->mst_enable) {
		mtk_dp_hdcp_update_content_protection(mtk_dp, DP_FIRST_CON, DP_SST_ENCODER_PORT);
		return;
	}

	for (i = 0; i < ARRAY_SIZE(mtk_dp->mtk_con); i++)
		if (mst_con_with_encoder(mtk_dp->mtk_con[i]))
			mtk_dp_hdcp_update_content_protection(mtk_dp, i, mtk_dp->mtk_con[i]->encoder_id);
}

void mtk_dp_hdcp_atomic_check(struct mtk_dp *mtk_dp, enum dp_encoder_id id,
			      struct drm_connector_state *state)
{
	bool need_hdcp;

	mutex_lock(&mtk_dp->hdcp_mutex);

	need_hdcp = mtk_dp_hdcp_need_hdcp(mtk_dp);

	if (!need_hdcp)
		mtk_dp_hdcp_disable(mtk_dp);

	if (need_hdcp)
		mtk_dp_hdcp_enable(mtk_dp);

	mutex_unlock(&mtk_dp->hdcp_mutex);
}

static enum drm_mode_status mtk_dp_check_mode_v2(struct mtk_dp *mtk_dp,
						 struct drm_display_mode *mode, int bpp, bool *dsc)
{
	u32 rate;
	int pixel_clock;
	u8 color_bpp;
	enum drm_mode_status ret = MODE_CLOCK_HIGH;
	bool exceeds_max_fps, is_max_hdisplay, is_max_vdisplay;

	if ((mtk_dp->data->max_hdisplay != 0 && mode->hdisplay > mtk_dp->data->max_hdisplay) ||
	    (mtk_dp->data->max_vdisplay != 0 && mode->vdisplay > mtk_dp->data->max_vdisplay) ||
	    (mode->htotal - mode->hdisplay < mtk_dp->data->min_hblanking))
		return MODE_BAD;

	is_max_hdisplay = (mtk_dp->data->max_hdisplay != 0 &&
			   mode->hdisplay == mtk_dp->data->max_hdisplay);
	is_max_vdisplay = (mtk_dp->data->max_vdisplay != 0 &&
			  mode->vdisplay == mtk_dp->data->max_vdisplay);
	exceeds_max_fps = (mtk_dp->data->max_fps_at_max_resolution != 0 &&
			   drm_mode_vrefresh(mode) > mtk_dp->data->max_fps_at_max_resolution);
	if (is_max_hdisplay && is_max_vdisplay && exceeds_max_fps)
		return MODE_BAD;

	/* This is for temporarily removing the timing. */
	if (mode->hdisplay < mtk_dp->data->min_hdisplay ||
	    mode->vdisplay < mtk_dp->data->min_vdisplay)
		return MODE_BAD;

	if(!mtk_dp_timing_alignment_valid(mtk_dp, mode))
		return MODE_BAD;

	*dsc = false;

	rate = drm_dp_bw_code_to_link_rate(mtk_dp->training_info.link_rate) *
		mtk_dp->training_info.link_lane_count;
	rate = rate * 97 / 100;

	if ((mode->clock * bpp / 8) < rate) {
		*dsc = false;
		ret = MODE_OK;
		goto end;
	}

	if (mtk_dp->data->dsc_support && mtk_dp->mtk_con[DP_FIRST_CON] &&
		drm_dp_sink_supports_dsc(mtk_dp->mtk_con[DP_FIRST_CON]->dsc_dpcd) &&
		drm_dp_sink_supports_fec(mtk_dp->mtk_con[DP_FIRST_CON]->fec_cap)) {
		pixel_clock = mtk_dp_dsc_cal_clock_v2(mode);
		color_bpp = mtk_dp_color_get_bpp_v2(DP_PIXELFORMAT_RGB, DP_COLOR_DEPTH_8BIT);
		if ((pixel_clock * color_bpp / 8) < rate) {
			*dsc = true;
			ret = MODE_OK;
		}
	}

end:
	return ret;
}

static int mtk_dp_bridge_atomic_check_v2(struct drm_bridge *bridge,
					 struct drm_bridge_state *bridge_state,
					 struct drm_crtc_state *crtc_state,
					 struct drm_connector_state *conn_state)
{
	struct mtk_dp_bridge *mtk_bridge = container_of(bridge, struct mtk_dp_bridge, bridge);
	enum dp_encoder_id id = mtk_bridge->encoder_id;
	struct mtk_dp *mtk_dp = mtk_bridge->mtk_dp;
	bool dsc = false;
	u8 bpp;

	if (!conn_state->crtc) {
		dev_err(mtk_dp->dev,
			"[DPTX] Can't enable bridge as connector state doesn't have a crtc\n");
		return -EINVAL;
	}

	if (bridge_state->input_bus_cfg.format == MEDIA_BUS_FMT_YUYV8_1X16)
		mtk_dp->info[id].format = DP_PIXELFORMAT_YUV422;
	else
		mtk_dp->info[id].format = DP_PIXELFORMAT_RGB;

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] bridge[%d] atomic check, [cp ct]:old[%d %d], new[%d %d]\n",
		    id, mtk_dp->con_state[id].content_protection,
		    mtk_dp->con_state[id].hdcp_content_type,
		    conn_state->content_protection, conn_state->hdcp_content_type);

	memcpy(&mtk_dp->con_state[id], conn_state, sizeof(struct drm_connector_state));

	mtk_dp_hdcp_atomic_check(mtk_dp, id, conn_state);

	if (mtk_dp->mst_enable)
		return mtk_dp_mst_atomic_check(mtk_dp, id, bridge_state, crtc_state, conn_state);

	drm_mode_copy(&mtk_dp->mode[id], &crtc_state->adjusted_mode);

	bpp = mtk_dp_color_get_bpp_v2(mtk_dp->info[id].format,
				      mtk_dp->info[id].depth);
	mtk_dp_check_mode_v2(mtk_dp, &mtk_dp->mode[id], bpp, &dsc);
	mtk_dp->dsc_enable[id] = dsc;

	if (mtk_dp->dsc_enable[id]) {
		mtk_dp->info[id].format = DP_PIXELFORMAT_RGB;
		mtk_dp->info[id].depth = DP_COLOR_DEPTH_8BIT;
		mtk_dp_dsc_check_prepare_v2(mtk_dp, id);
		mtk_dp->prop_dsc_enable[id]->values[0] = 1;
	}

	if (mtk_dp->prop_dsc_enable[id])
		mtk_dp->prop_dsc_enable[id]->values[0] = mtk_dp->dsc_enable[id] ? 1 : 0;

	drm_dbg_kms(mtk_dp->drm_dev,
		    "[DPTX] bridge[%d] SST atomic check, dsc:%d, color:in[0x%04x] out[0x%04x], tt:%d %d, act:%d %d, fps:%d, clk:%d\n",
		    id, dsc, bridge_state->input_bus_cfg.format, bridge_state->output_bus_cfg.format,
		    mtk_dp->mode[id].htotal, mtk_dp->mode[id].vtotal,
		    mtk_dp->mode[id].hdisplay, mtk_dp->mode[id].vdisplay,
		    drm_mode_vrefresh(&mtk_dp->mode[id]), mtk_dp->mode[id].clock);

	return 0;
}

static void mtk_dp_bridge_atomic_disable_v2(struct drm_bridge *bridge,
					    struct drm_bridge_state *old_state)
{
	struct mtk_dp_bridge *mtk_bridge = container_of(bridge, struct mtk_dp_bridge, bridge);
	enum dp_encoder_id id = mtk_bridge->encoder_id;
	struct mtk_dp *mtk_dp = mtk_bridge->mtk_dp;
	struct drm_atomic_state *state = old_state->base.state;
	int con_id;

	if (mtk_dp->mst_enable) {
		mtk_dp_mst_atomic_disable(mtk_dp, id, state);
		return;
	}

	con_id = encoder_id_to_con_id(mtk_dp, id, DRM_DP_SST);
	if (con_id < 0)
		return;

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] bridge[%d] SST disable", id);

	mtk_dp_video_mute_v2(mtk_dp, id, true);
	mtk_dp_audio_mute_v2(mtk_dp, id, true);
	mtk_dp_video_disable_v2(mtk_dp, id);

	mtk_dp->mtk_con[con_id]->video_enable = false;

	mtk_dp_hdcp_disable(mtk_dp);

	mtk_dp_audio_update_plugged_status_v2(mtk_dp);
}

static void mtk_dp_bridge_atomic_enable_v2(struct drm_bridge *bridge,
					   struct drm_bridge_state *old_state)
{
	struct mtk_dp_bridge *mtk_bridge = container_of(bridge, struct mtk_dp_bridge, bridge);
	enum dp_encoder_id id = mtk_bridge->encoder_id;
	struct mtk_dp *mtk_dp = mtk_bridge->mtk_dp;
	struct drm_atomic_state *state = old_state->base.state;
	int con_id;
	bool link_ok = true;

	if (mtk_dp->mst_enable) {
		if (!mtk_dp_mst_link_status(mtk_dp))
			link_ok = false;
	} else {
		if (!mtk_dp_link_status_v2(mtk_dp))
			link_ok = false;
	}

	if (!link_ok) {
		dev_err(mtk_dp->dev, "[DPTX] link fail, training again\n");
		mtk_dp->dp_ready = false;
		mtk_dp_training_handle_v2(mtk_dp);
	}

	if (mtk_dp->mst_enable) {
		mtk_dp_mst_atomic_enable(mtk_dp, id, state);
		return;
	}

	con_id = encoder_id_to_con_id(mtk_dp, id, DRM_DP_SST);
	if (con_id < 0)
		return;

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] bridge[%d] SST enable", id);

	if (!mtk_dp->training_info.cable_plug_in || !mtk_dp->dp_ready) {
		dev_err(mtk_dp->dev, "[DPTX] bridge[%d] SST cable_plug_in:%d, dp_ready:%d",
			id, mtk_dp->training_info.cable_plug_in, mtk_dp->dp_ready);
		return;
	}

	if (drm_dp_sink_supports_fec(mtk_dp->mtk_con[con_id]->fec_cap))
		mtk_dp_fec_enable_v2(mtk_dp);

	mtk_dp_video_mute_v2(mtk_dp, id, true);
	mtk_dp_video_enable_v2(mtk_dp, id);
	mtk_dp_video_mute_v2(mtk_dp, id, false);

	mtk_dp->mtk_con[con_id]->video_enable = true;

	/* audio */
	mtk_dp->audio_enable =
		mtk_dp_parse_audio_cap_v2(mtk_dp,
					  &mtk_dp->info[id].audio_cur_cfg);

	if (mtk_dp->audio_enable) {
		mtk_dp_audio_mute_v2(mtk_dp, id, true);
		mtk_dp_audio_config_v2(mtk_dp, id);
		mtk_dp_audio_mute_v2(mtk_dp, id, false);
	} else {
		memset(&mtk_dp->info[id].audio_cur_cfg, 0,
		       sizeof(mtk_dp->info[id].audio_cur_cfg));
	}

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] bridge[%d] SST pattern_gen:%d, audio_enable:%d\n",
		    id, mtk_dp->info[id].pattern_gen, mtk_dp->audio_enable);

	mtk_dp_audio_update_plugged_status_v2(mtk_dp);

	/* HDCP */
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] bridge[%d] SST hdcp_content_type:%d, content protection: %d",
		    id, mtk_dp->con_state[id].hdcp_content_type, mtk_dp->con_state[id].content_protection);
	if (mtk_dp->con_state[id].content_protection == DRM_MODE_CONTENT_PROTECTION_DESIRED)
		mtk_dp_hdcp_enable(mtk_dp);
}

static enum drm_connector_status mtk_dp_con_detect_v2
	(struct drm_connector *connector, bool force)
{
	struct mtk_dp *mtk_dp;
	struct mtk_dp_con *mtk_con;
	enum drm_connector_status ret = connector_status_disconnected;
	enum drm_dp_mst_mode dp_mode;

	mtk_con = container_of(connector, struct mtk_dp_con, connector);
	mtk_dp = mtk_con->mtk_dp;

	if (!mtk_dp->training_info.cable_plug_in ||
	    !mtk_dp->dp_ready)
		goto end;

	if (mtk_dp->data->mst_support) {
		dp_mode = drm_dp_read_mst_cap(&mtk_dp->aux, mtk_dp->rx_cap);
		if (dp_mode == DRM_DP_MST) {
			drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] support MST\n");
			mtk_dp->mst_enable = true;
			mtk_dp_mst_drv_prepare(mtk_dp);
			return connector_status_disconnected;
		}

		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] support SST\n");
		mtk_dp_mst_drv_unprepare(mtk_dp);
		mtk_dp->mst_enable = false;
	}

	if (mtk_dp->training_info.sink_count)
		ret = connector_status_connected;

end:
	drm_dbg_kms(mtk_dp->drm_dev,
		    "[DPTX] connector[%d], plug in:%d, sink count:%d, mst:%d, detect:%d",
		    mtk_dp_con_id(mtk_dp, mtk_con),
		    mtk_dp->training_info.cable_plug_in,
		    mtk_dp->training_info.sink_count,
		    mtk_dp->mst_enable, ret);
	return ret;
}

static void mtk_dp_con_destroy_v2(struct drm_connector *connector)
{
	struct mtk_dp_con *mtk_con = container_of(connector, struct mtk_dp_con, connector);
	struct mtk_dp *mtk_dp = mtk_con->mtk_dp;
	int id =  mtk_dp_con_id(mtk_dp, mtk_con);

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] connector[%d] destroy\n", id);

	if (id < 0)
		return;

	drm_connector_cleanup(connector);

	kfree(mtk_con);
	mtk_dp->mtk_con[id] = NULL;
}

static int mtk_dp_con_late_register_v2(struct drm_connector *connector)
{
	struct mtk_dp *mtk_dp;
	struct mtk_dp_con *mtk_con;

	mtk_con = container_of(connector, struct mtk_dp_con, connector);
	mtk_dp = mtk_con->mtk_dp;

	if (connector->connector_type == DRM_MODE_CONNECTOR_DisplayPort)
		mtk_dp->aux.dev = connector->kdev;

	return 0;
}

static void mtk_dp_con_early_unregister_v2(struct drm_connector *connector)
{
	struct mtk_dp_con *mtk_con;
	struct mtk_dp *mtk_dp;

	mtk_con = container_of(connector, struct mtk_dp_con, connector);
	mtk_dp = mtk_con->mtk_dp;
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] connector[%d] early unregister\n", mtk_dp_con_id(mtk_dp, mtk_con));
}

static const struct drm_connector_funcs mtk_dp_con_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = mtk_dp_con_detect_v2,
	.destroy = mtk_dp_con_destroy_v2,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
	.late_register = mtk_dp_con_late_register_v2,
	.early_unregister = mtk_dp_con_early_unregister_v2,
};

static enum drm_mode_status mtk_dp_con_mode_valid_v2(struct drm_connector *connector,
						     struct drm_display_mode *mode)
{
	struct mtk_dp_con *mtk_con;
	struct mtk_dp *mtk_dp;
	enum drm_mode_status mode_status;
	bool dsc = false;
	u8 bpp = 24;

	mtk_con = container_of(connector, struct mtk_dp_con, connector);
	mtk_dp = mtk_con->mtk_dp;

	bpp = connector->display_info.color_formats & DRM_COLOR_FORMAT_YCBCR422 ? 16 : 24;
	mode_status = mtk_dp_check_mode_v2(mtk_dp, mode, bpp, &dsc);

	drm_dbg_kms(mtk_dp->drm_dev,
		    "[DPTX] connector[%d] SST mode valid status:%d, dsc:%d, Htt:%d, Vtt:%d, Hact:%d, Vact:%d, fps:%d, clk:%d, YCBCR422:%d\n",
		    mtk_dp_con_id(mtk_dp, mtk_con), mode_status, dsc,
		    mode->htotal, mode->vtotal,
		    mode->hdisplay, mode->vdisplay,
		    drm_mode_vrefresh(mode), mode->clock,
		    connector->display_info.color_formats & DRM_COLOR_FORMAT_YCBCR422);

	return mode_status;
}

static int mtk_dp_con_get_modes_v2(struct drm_connector *connector)
{
	struct mtk_dp *mtk_dp;
	struct mtk_dp_con *mtk_con;
	int ret, num_modes = 0;
	struct mtk_dp_audio_cfg *audio_caps;
	struct cea_sad *sads;
	int encoder_id;
	unsigned long timeout = 0;

	mtk_con = container_of(connector, struct mtk_dp_con, connector);
	mtk_dp = mtk_con->mtk_dp;

	if (!mtk_con->edid) {
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] get edid\n");
		timeout = jiffies + msecs_to_jiffies(2000);
		while (time_before(jiffies, timeout)) {
			mtk_con->edid = drm_get_edid(connector, &mtk_dp->aux.ddc);
			if (mtk_con->edid)
				break;

			msleep(100);
		}

		if (!mtk_con->edid) {
			dev_err(mtk_dp->dev, "[DPTX] Failed to read EDID\n");
			goto fail;
		}
	}

	/* audio caps */
	encoder_id = mtk_con->encoder_id;
	audio_caps = &mtk_dp->info[encoder_id].audio_cur_cfg;
	audio_caps->sad_count = drm_edid_to_sad(mtk_con->edid, &sads);
	kfree(sads);
	audio_caps->detect_monitor = drm_detect_monitor_audio(mtk_con->edid);

	ret = drm_connector_update_edid_property(&mtk_con->connector, mtk_con->edid);
	if (ret) {
		dev_err(mtk_dp->dev, "[DPTX] Failed to update EDID property: %d\n", ret);
		goto fail;
	}

	num_modes = drm_add_edid_modes(&mtk_con->connector, mtk_con->edid);

fail:
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] connector[%d] SST modes:%d\n", mtk_dp_con_id(mtk_dp, mtk_con), num_modes);
	return num_modes;
}

static const struct drm_connector_helper_funcs mtk_dp_con_helper_funcs = {
	.get_modes = mtk_dp_con_get_modes_v2,
	.mode_valid = mtk_dp_con_mode_valid_v2,
};

static struct mtk_dp_con *mtk_dp_create_connector_v2(struct mtk_dp *mtk_dp)
{
	struct mtk_dp_con *mtk_con;
	struct drm_bridge *bridge;
	int ret;

	bridge = devm_drm_of_get_bridge(mtk_dp->dev, mtk_dp->dev->of_node,
					DP_SST_ENCODER_PORT, DP_ENCODER_ENDPOINT);
	if (IS_ERR(bridge)) {
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] create con, can not find bridge[%d, %d]",
			    DP_SST_ENCODER_PORT, DP_ENCODER_ENDPOINT);
		return NULL;
	}
	if (!bridge->encoder) {
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] create con, bridge have no encoder[%d, %d]",
			    DP_SST_ENCODER_PORT, DP_ENCODER_ENDPOINT);
		return NULL;
	}
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] create con, found dp_intf[%d] bridge node:%pOF\n",
		    DP_SST_ENCODER_PORT, bridge->of_node);

	mtk_con = kzalloc(sizeof(*mtk_con), GFP_KERNEL);
	if (!mtk_con)
		return NULL;

	mtk_con->mtk_dp = mtk_dp;

	ret = drm_connector_init(mtk_dp->drm_dev, &mtk_con->connector,
				 &mtk_dp_con_funcs, DRM_MODE_CONNECTOR_DisplayPort);
	if (ret) {
		drm_dbg_kms(mtk_dp->drm_dev,
			    "[DPTX] create con, failed to init connector:%d\n", ret);
		kfree(mtk_con);
		return NULL;
	}

	drm_display_info_set_bus_formats(&mtk_con->connector.display_info,
					 mt8196_output_fmts,
					 ARRAY_SIZE(mt8196_output_fmts));

	drm_connector_helper_add(&mtk_con->connector,
				 &mtk_dp_con_helper_funcs);
	mtk_con->connector.polled = DRM_CONNECTOR_POLL_HPD;

	ret = drm_connector_attach_encoder(&mtk_con->connector, bridge->encoder);
	if (ret) {
		drm_dbg_kms(mtk_dp->drm_dev,
			    "[DPTX] create con, failed to attach encoder:%d\n", ret);
		kfree(mtk_con);
		return NULL;
	}

	mtk_con->encoder = bridge->encoder;
	mtk_con->encoder_id = DP_SST_ENCODER_PORT;
	mtk_con->dp_mode = DRM_DP_SST;

	if (mtk_con->connector.funcs->reset)
		mtk_con->connector.funcs->reset(&mtk_con->connector);

	drm_connector_attach_content_protection_property(&mtk_con->connector, true);

	ret = drm_connector_register(&mtk_con->connector);
	if (ret) {
		drm_dbg_kms(mtk_dp->drm_dev,
			    "[DPTX] create con, failed to register connector:%d\n", ret);
		kfree(mtk_con);
		return NULL;
	}

	mtk_dp->mtk_con[DP_FIRST_CON] = mtk_con;
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] create con, create mtk connector[%d]\n", DP_FIRST_CON);

	return mtk_dp->mtk_con[DP_FIRST_CON];
}

static int mtk_dp_bridge_attach_v2(struct drm_bridge *bridge,
				   enum drm_bridge_attach_flags flags)
{
	struct mtk_dp_bridge *mtk_bridge = container_of(bridge, struct mtk_dp_bridge, bridge);
	struct mtk_dp *mtk_dp = mtk_bridge->mtk_dp;
	int ret;

	drm_dbg_kms(bridge->dev, "[DPTX] bridge[%d] attach, refcount:%u", mtk_bridge->encoder_id, atomic_read(&mtk_dp->refcount));

	/*
	 * In a scenario that supports MST, DP will have more
	 * than one encoder, with each encoder corresponding
	 * to a bridge. DP initialization is performed only
	 * after all bridges are attached.
	 */
	if (atomic_inc_return(&mtk_dp->refcount) != mtk_dp->data->encoder_num)
		return 0;

	mtk_dp->drm_dev = bridge->dev;

	mtk_dp->aux.drm_dev = bridge->dev;
	ret = drm_dp_aux_register(&mtk_dp->aux);
	if (ret) {
		dev_err(mtk_dp->dev, "[DPTX] failed to register DP AUX channel:%d\n", ret);
		return ret;
	}

	if (mtk_dp->data->mst_support)
		mtk_dp_mst_drv_init(mtk_dp);
	mtk_dp_create_connector_v2(mtk_dp);

	mtk_dp_init_port_v2(mtk_dp);

	enable_irq(mtk_dp->irq);
	mtk_dp_hpd_interrupt_enable_v2(mtk_dp, true);

	return 0;
}

static void mtk_dp_resouce_free(struct mtk_dp *mtk_dp)
{
	mtk_dp_hpd_interrupt_enable_v2(mtk_dp, false);
	disable_irq(mtk_dp->irq);

	mtk_dp_disconnect_release_v2(mtk_dp);

	if (mtk_dp->data->mst_support)
		mtk_dp_mst_drv_deinit(mtk_dp);

	mtk_dp->drm_dev = NULL;
	drm_dp_aux_unregister(&mtk_dp->aux);
}

static void mtk_dp_bridge_detach_v2(struct drm_bridge *bridge)
{
	struct mtk_dp_bridge *mtk_bridge = container_of(bridge, struct mtk_dp_bridge, bridge);
	struct mtk_dp *mtk_dp = mtk_bridge->mtk_dp;

	if (atomic_dec_and_test(&mtk_dp->refcount))
		mtk_dp_resouce_free(mtk_dp);
}

static u32 *mtk_dp_bridge_atomic_get_output_bus_fmts_v2(struct drm_bridge *bridge,
							struct drm_bridge_state *bridge_state,
							struct drm_crtc_state *crtc_state,
							struct drm_connector_state *conn_state,
							unsigned int *num_output_fmts)
{
	u32 *output_fmts;

	*num_output_fmts = 0;
	output_fmts = kmalloc(sizeof(*output_fmts), GFP_KERNEL);
	if (!output_fmts)
		return NULL;
	*num_output_fmts = 1;
	output_fmts[0] = MEDIA_BUS_FMT_FIXED;
	return output_fmts;
}

static u32 *mtk_dp_bridge_atomic_get_input_bus_fmts_v2(struct drm_bridge *bridge,
						       struct drm_bridge_state *bridge_state,
						       struct drm_crtc_state *crtc_state,
						       struct drm_connector_state *conn_state,
						       u32 output_fmt,
						       unsigned int *num_input_fmts)
{
	u32 *input_fmts;
	bool dsc = false, use_platform_format = false;
	u8 bpp;
	bool support_422 = false;
	struct mtk_dp_bridge *mtk_bridge = container_of(bridge, struct mtk_dp_bridge, bridge);
	struct mtk_dp *mtk_dp = mtk_bridge->mtk_dp;
	struct drm_display_mode *mode = &crtc_state->adjusted_mode;
	u32 lane_count_min = mtk_dp->training_info.link_lane_count;
	u32 rate = drm_dp_bw_code_to_link_rate(mtk_dp->training_info.link_rate) * lane_count_min;
	int fmt = MEDIA_BUS_FMT_RGB888_1X24; /* default format: RGB888 */

	if (mtk_dp->mst_enable)
		goto set_fmts;

	support_422 = conn_state->connector->display_info.color_formats &
		DRM_COLOR_FORMAT_YCBCR422 ? true : false;
	bpp = support_422 ? 16 : 24;
	mtk_dp_check_mode_v2(mtk_dp, mode, bpp, &dsc);
	if (dsc)
		goto set_fmts;

	/* SSC + FEC would occupy 3% */
	rate = rate * 97 / 100;
	if (rate > (mode->clock * 24 / 8))
		use_platform_format = true;
	else if (support_422 && rate > (mode->clock * 16 / 8))
		fmt = MEDIA_BUS_FMT_YUYV8_1X16;
	else
		goto end;

set_fmts:
	*num_input_fmts = use_platform_format ? ARRAY_SIZE(mt8196_input_fmts) : 1;
	input_fmts = kcalloc(*num_input_fmts, sizeof(*input_fmts), GFP_KERNEL);
	if (!input_fmts)
		return NULL;

	if (use_platform_format)
		memcpy(input_fmts, mt8196_input_fmts, sizeof(mt8196_input_fmts));
	else
		input_fmts[0] = fmt;

	drm_dbg_kms(mtk_dp->drm_dev,
		    "[DPTX] bridge[%d] input fmts, fmt:%s, dsc:%d, Htt:%d, Vtt:%d, Hact:%d, Vact:%d, fps:%d, clk:%d",
		    mtk_bridge->encoder_id,
		    (!use_platform_format) ?
		    (input_fmts[0] == MEDIA_BUS_FMT_YUYV8_1X16) ?
			"MEDIA_BUS_FMT_YUYV8_1X16" : "MEDIA_BUS_FMT_RGB888_1X24" :
		    "mt8196_input_fmts",
		    dsc, mode->htotal, mode->vtotal,
		    mode->hdisplay, mode->vdisplay,
		    drm_mode_vrefresh(mode), mode->clock);
	return input_fmts;

end:
	*num_input_fmts = 0;
	dev_err(mtk_dp->dev, "[DPTX] bridge[%d] get_input_bus_fmts, return NULL", mtk_bridge->encoder_id);
	return NULL;
}

static const struct drm_bridge_funcs mtk_dp_bridge_funcs = {
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_get_output_bus_fmts = mtk_dp_bridge_atomic_get_output_bus_fmts_v2,
	.atomic_get_input_bus_fmts = mtk_dp_bridge_atomic_get_input_bus_fmts_v2,
	.atomic_reset = drm_atomic_helper_bridge_reset,
	.attach = mtk_dp_bridge_attach_v2,
	.detach = mtk_dp_bridge_detach_v2,
	.atomic_enable = mtk_dp_bridge_atomic_enable_v2,
	.atomic_disable = mtk_dp_bridge_atomic_disable_v2,
	.atomic_check = mtk_dp_bridge_atomic_check_v2,
};

static void mtk_dp_training_check_swing_pre_v2(struct mtk_dp *mtk_dp,
					       u8 lane_count,
					       u8 *dpcd_202,
					       u8 *dpcd_buffer,
					       u8 is_adjustable_swing_pre,
					       u8 is_lttpr)
{
	u8 swing, emhasis;
	u8 lane01_adjust_offset, lane23_adjust_offset;

	if (is_lttpr) {
		lane01_adjust_offset = 3; /* F0033h-F0030h */
		lane23_adjust_offset = 4; /* F0034h-F0030h */
	} else {
		lane01_adjust_offset = 4; /* 206h-202h */
		lane23_adjust_offset = 5; /* 207h-202h */
	}

	if (lane_count >= 0x1) { /* lane0 */
		swing = (dpcd_202[lane01_adjust_offset] & 0x3);
		emhasis = ((dpcd_202[lane01_adjust_offset] & 0x0c) >> 2);

		/* Adjust the swing and pre-emphasis */
		if (is_adjustable_swing_pre)
			mtk_dp_swingt_set_pre_emphasis_v2(mtk_dp, DP_LANE0, swing, emhasis);
		/* Adjust the swing and pre-emphasis done, notify Sink Side */
		dpcd_buffer[0x0] = swing | (emhasis << 3);

		/* MAX_SWING_REACHED */
		if (swing == DP_SWING3)
			dpcd_buffer[0x0] |= BIT(2);
		/* MAX_PRE-EMPHASIS_REACHED */
		if (emhasis == DP_PREEMPHASIS3)
			dpcd_buffer[0x0] |= BIT(5);
	}

	if (lane_count >= 0x2) { /* lane1 */
		swing = (dpcd_202[lane01_adjust_offset] & 0x30) >> 4;
		emhasis = ((dpcd_202[lane01_adjust_offset] & 0xc0) >> 6);

		/* Adjust the swing and pre-emphasis */
		if (is_adjustable_swing_pre)
			mtk_dp_swingt_set_pre_emphasis_v2(mtk_dp, DP_LANE1, swing, emhasis);
		/* Adjust the swing and pre-emphasis done, notify Sink Side */
		dpcd_buffer[0x1] = swing | (emhasis << 3);

		/* MAX_SWING_REACHED */
		if (swing == DP_SWING3)
			dpcd_buffer[0x1] |= BIT(2);
		/* MAX_PRE-EMPHASIS_REACHED */
		if (emhasis == DP_PREEMPHASIS3)
			dpcd_buffer[0x1] |= BIT(5);
	}

	if (lane_count == 0x4) { /* lane 2,3 */
		swing = (dpcd_202[lane23_adjust_offset] & 0x3);
		emhasis = ((dpcd_202[lane23_adjust_offset] & 0x0c) >> 2);

		/* Adjust the swing and pre-emphasis */
		if (is_adjustable_swing_pre)
			mtk_dp_swingt_set_pre_emphasis_v2(mtk_dp, DP_LANE2, swing, emhasis);
		/* Adjust the swing and pre-emphasis done, notify Sink Side */
		dpcd_buffer[0x2] = swing | (emhasis << 3);

		/* MAX_SWING_REACHED */
		if (swing == DP_SWING3)
			dpcd_buffer[0x2] |= BIT(2);
		/* MAX_PRE-EMPHASIS_REACHED */
		if (emhasis == DP_PREEMPHASIS3)
			dpcd_buffer[0x2] |= BIT(5);

		swing = (dpcd_202[lane23_adjust_offset] & 0x30) >> 4;
		emhasis = ((dpcd_202[lane23_adjust_offset] & 0xc0) >> 6);

		/* Adjust the swing and pre-emphasis */
		if (is_adjustable_swing_pre)
			mtk_dp_swingt_set_pre_emphasis_v2(mtk_dp, DP_LANE3, swing, emhasis);
		/* Adjust the swing and pre-emphasis done, notify Sink Side */
		dpcd_buffer[0x3] = swing | (emhasis << 3);

		/* MAX_SWING_REACHED */
		if (swing == DP_SWING3)
			dpcd_buffer[0x3] |= BIT(2);
		/* MAX_PRE-EMPHASIS_REACHED */
		if (emhasis == DP_PREEMPHASIS3)
			dpcd_buffer[0x3] |= BIT(5);
	}

	/* Wait signal stable enough */
	mdelay(1);
}

static enum dp_train_stage mtk_dp_check_training_res_v2(struct mtk_dp *mtk_dp, u8 dpcd_202)
{
	enum dp_train_stage res = DP_LT_PASS;

	if (mtk_dp->training_info.cr_done == 0x0) {
		if ((dpcd_202 & 0x01) != 0x01)
			res = DP_LT_CR_L0_FAIL;
		else if ((dpcd_202 & 0x11) != 0x11)
			res = DP_LT_CR_L1_FAIL;
		else
			res = DP_LT_CR_L2_FAIL;
	} else if (mtk_dp->training_info.eq_done == 0x0) {
		if ((dpcd_202 & 0x07) != 0x07)
			res = DP_LT_EQ_L0_FAIL;
		else if ((dpcd_202 & 0x77) != 0x77)
			res = DP_LT_EQ_L1_FAIL;
		else
			res = DP_LT_EQ_L2_FAIL;
	}

	return res;
}

static enum dp_train_stage mtk_dp_training_flow_v2(struct mtk_dp *mtk_dp, u8 link_rate, u8 lane_count)
{
	u8 dpcd_buffer[0x4], dpcd_202[0x6], temp[0x6];
	u8 dpcd_206 = 0xff;
	u8 retry_times = 0;
	u8 control = 0;
	u8 loop = 0;
	bool ssc_enable = false;
	enum dp_train_stage res = DP_LT_NONE;

	memset(temp, 0x0, sizeof(temp));
	memset(dpcd_buffer, 0x0, sizeof(dpcd_buffer));
	memset(dpcd_202, 0x0, sizeof(dpcd_202));

	temp[0] = link_rate;
	temp[1] = (lane_count | DP_AUX_SET_ENAHNCED_FRAME);
	drm_dp_dpcd_write(&mtk_dp->aux, DPCD_00100, temp, 0x2);

	mtk_dp_ssc_check_v2(mtk_dp, &ssc_enable);
	mtk_dp_phy_training_config_v2(mtk_dp, link_rate, lane_count, ssc_enable);
	mtk_dp_set_lane_count_v2(mtk_dp, lane_count);
	mdelay(5);

	do {
		retry_times++;

		if (!mtk_dp->training_info.cable_plug_in) {
			drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] Training Abort, HPD is low\n");
			return DP_LT_NONE;
		}

		if (mtk_dp->training_info.cr_done == 0x0) {
			drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] CR Training START\n");
			mtk_dp_set_scramble_v2(mtk_dp, false);

			if (control == 0x0)	{
				mtk_dp_set_training_pattern_v2(mtk_dp, BIT(4));
				control = 0x1;
				temp[0] = 0x21;
				drm_dp_dpcd_write(&mtk_dp->aux, DPCD_00102, temp, 0x1);
				loop++;

				/* force use SWING = 0 & PRE = 0 to start 1st link training */
				mtk_dp_training_check_swing_pre_v2(mtk_dp, lane_count, temp,
								   dpcd_buffer, true, false);
			}

			drm_dp_dpcd_write(&mtk_dp->aux, DPCD_00103, dpcd_buffer, lane_count);
			drm_dp_link_train_clock_recovery_delay(&mtk_dp->aux, mtk_dp->rx_cap);
			drm_dp_dpcd_read(&mtk_dp->aux, DPCD_00202, dpcd_202, 0x6);

			if (drm_dp_clock_recovery_ok(dpcd_202, lane_count)) {
				drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] CR Training Success\n");

				mtk_dp->training_info.cr_done = true;

				retry_times = 0x0;
				loop = 0x1;
			} else {
				/* request swing & emp is the same with last time */
				if (dpcd_206 == dpcd_202[0x4]) {
					if ((dpcd_206 & 0x3) == 0x3) /* lane0 match max swing */
						loop = DPTX_TRAIN_MAX_ITERATION;
					else
						loop++;
				} else {
					dpcd_206 = dpcd_202[0x4];
				}

				drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] CR Training Fail\n");
			}
		} else if (mtk_dp->training_info.eq_done == 0x0) {
			drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] EQ Training START\n");

			if (control == 0x1) {
				if (mtk_dp->training_info.tps4_support) {
					mtk_dp_set_training_pattern_v2(mtk_dp, DP_TPS4);
					temp[0] = 0x07;
				} else if (mtk_dp->training_info.tps3_support) {
					mtk_dp_set_training_pattern_v2(mtk_dp, DP_TPS3);
					temp[0] = 0x23;
				} else {
					mtk_dp_set_training_pattern_v2(mtk_dp, DP_TPS2);
					temp[0] = 0x22;
				}
				drm_dp_dpcd_write(&mtk_dp->aux, DPCD_00102, temp, 0x1);

				control = 0x2;
				drm_dp_dpcd_read(&mtk_dp->aux, DPCD_00206, (dpcd_202 + 4), 0x2);

				loop++;
				mtk_dp_training_check_swing_pre_v2(mtk_dp, lane_count, dpcd_202,
								dpcd_buffer, true, false);
			}

			drm_dp_dpcd_write(&mtk_dp->aux, DPCD_00103, dpcd_buffer, lane_count);
			drm_dp_link_train_channel_eq_delay(&mtk_dp->aux, mtk_dp->rx_cap);

			drm_dp_dpcd_read(&mtk_dp->aux, DPCD_00202, dpcd_202, 0x6);

			if (!drm_dp_clock_recovery_ok(dpcd_202, lane_count)) {
				mtk_dp->training_info.cr_done = false;
				mtk_dp->training_info.eq_done = false;
				break;
			}

			if (drm_dp_channel_eq_ok(dpcd_202, lane_count)) {
				drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] EQ Training Success\n");
				if (dpcd_202[2] & 0x1) {
					mtk_dp->training_info.eq_done = true;
					drm_dbg_kms(mtk_dp->drm_dev,
						    "[DPTX] Inter-lane skew Success\n");
					break;
				}
			}

			drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] EQ Training Fail\n");

			if (dpcd_206 == dpcd_202[0x4])
				loop++;
			else
				dpcd_206 = dpcd_202[0x4];
		}

		mtk_dp_training_check_swing_pre_v2(mtk_dp, lane_count, dpcd_202,
						   dpcd_buffer, true, false);
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] retry_times:%d, loop:%d\n", retry_times, loop);
	} while ((retry_times < DP_LT_RETRY_LIMIT) &&
		 (loop < DPTX_TRAIN_MAX_ITERATION));

	temp[0] = 0x0;
	drm_dp_dpcd_write(&mtk_dp->aux, DPCD_00102, temp, 0x1);
	mtk_dp_set_training_pattern_v2(mtk_dp, DP_0);

	if (mtk_dp->training_info.eq_done) {
		mtk_dp->training_info.link_rate = link_rate;
		mtk_dp->training_info.link_lane_count = lane_count;

		mtk_dp_set_scramble_v2(mtk_dp, true);
		mtk_dp_set_enhanced_frame_mode_v2(mtk_dp, ENABLE_DP_EF_MODE);

		drm_dbg_kms(mtk_dp->drm_dev,
			    "[DPTX] Training PASS, link rate:0x%x, lane count:%d\n",
			    mtk_dp->training_info.link_rate,
			    mtk_dp->training_info.link_lane_count);
		return DP_LT_PASS;
	}

	dev_err(mtk_dp->dev, "[DPTX] Training Fail\n");

	res = mtk_dp_check_training_res_v2(mtk_dp, dpcd_202[0]);

	return res;
}

static void mtk_dp_phy_param(struct mtk_dp *mtk_dp, enum dp_link_rate link_rate)
{
	int i;
	u32 *phy_settings_param;

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] Set phy by link rate\n");

	if ((link_rate == DP_LINK_RATE_RBR) || (link_rate == DP_LINK_RATE_HBR)) {
		phy_settings_param = mtk_dp->phy_settings;
	} else if (link_rate == DP_LINK_RATE_HBR2) {
		phy_settings_param = mtk_dp->phy_settings_hbr2;
	}

	for (i = 0; i < 4; i++) {
		PHY_WRITE_4BYTE(mtk_dp,
				mtk_dp->data->phyd_dig_lan_offset[i] + DRIVING_PARAM_3,
				phy_settings_param[0]);
		PHY_WRITE_4BYTE(mtk_dp,
				mtk_dp->data->phyd_dig_lan_offset[i] + DRIVING_PARAM_4,
				phy_settings_param[1]);
		PHY_WRITE_4BYTE(mtk_dp,
				mtk_dp->data->phyd_dig_lan_offset[i] + DRIVING_PARAM_5,
				phy_settings_param[2]);
		PHY_WRITE_4BYTE(mtk_dp,
				mtk_dp->data->phyd_dig_lan_offset[i] + DRIVING_PARAM_6,
				phy_settings_param[3]);
		PHY_WRITE_4BYTE(mtk_dp,
				mtk_dp->data->phyd_dig_lan_offset[i] + DRIVING_PARAM_7,
				phy_settings_param[4]);
		PHY_WRITE_4BYTE(mtk_dp,
				mtk_dp->data->phyd_dig_lan_offset[i] + DRIVING_PARAM_8,
				phy_settings_param[5]);
	}
}

static int mtk_dp_set_training_start_v2(struct mtk_dp *mtk_dp)
{
	enum dp_link_rate max_link_rate = mtk_dp->training_info.max_link_rate;
	enum dp_lane_count max_lane_count = mtk_dp->training_info.max_link_lane_count;
	enum dp_link_rate link_rate;
	enum dp_lane_count lane_count;
	u32 loop;

	if (mtk_dp->training_info.dp_version == DP_VER_14)
		loop = DP_CTS_RETRAIN_TIMES_14;
	else
		loop = DP_CTS_RETRAIN_TIMES_DEFAULT;

	link_rate = mtk_dp->rx_cap[1];
	lane_count = mtk_dp->rx_cap[2] & GENMASK(4, 0);
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] RX support link rate:0x%x, lane count:%d",
		link_rate, lane_count);

	link_rate = (link_rate >= max_link_rate) ?
		max_link_rate : link_rate;
	lane_count = (lane_count >= max_lane_count) ?
		max_lane_count : lane_count;

	switch (link_rate) {
	case DP_LINK_RATE_RBR:
	case DP_LINK_RATE_HBR:
	case DP_LINK_RATE_HBR2:
	case DP_LINK_RATE_HBR25:
	case DP_LINK_RATE_HBR3:
		break;

	default:
		if (link_rate > DP_LINK_RATE_HBR3)
			link_rate = DP_LINK_RATE_HBR3;
		else if (link_rate > DP_LINK_RATE_HBR2)
			link_rate = DP_LINK_RATE_HBR2;
		else if (link_rate > DP_LINK_RATE_HBR)
			link_rate = DP_LINK_RATE_HBR;
		else
			link_rate = DP_LINK_RATE_RBR;
		break;
	};

	max_link_rate = link_rate;

	do {
		if (!mtk_dp->training_info.cable_plug_in) {
			drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] plug out, stop training");
			return DP_RET_PLUG_OUT;
		}

		mtk_dp->training_info.cr_done = false;
		mtk_dp->training_info.eq_done = false;

		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] training with link rate:0x%x, lane count:%d",
			link_rate, lane_count);

		if(mtk_dp->phy_by_link_rate)
			mtk_dp_phy_param(mtk_dp, link_rate);

		mtk_dp_training_flow_v2(mtk_dp, link_rate, lane_count);

		if (!mtk_dp->training_info.cr_done) {
			switch (link_rate) {
			case DP_LINK_RATE_RBR:
				lane_count = lane_count / 2;
				link_rate = max_link_rate;

				if (lane_count == 0x0)
					return DP_RET_TRANING_FAIL;

				break;

			case DP_LINK_RATE_HBR:
				link_rate = DP_LINK_RATE_RBR;
				break;

			case DP_LINK_RATE_HBR2:
				link_rate = DP_LINK_RATE_HBR;
				break;

			case DP_LINK_RATE_HBR3:
				link_rate = DP_LINK_RATE_HBR2;
				break;

			default:
				return DP_RET_TRANING_FAIL;
			};

			loop--;
		} else if (!mtk_dp->training_info.eq_done) {
			if (lane_count == DP_4LANE)
				lane_count = DP_2LANE;
			else if (lane_count >= DP_1LANE)
				lane_count = DP_1LANE;
			else
				return DP_RET_TRANING_FAIL;

			loop--;
		} else {
			return DP_RET_NOERR;
		}
	} while (loop > 0);

	return DP_RET_TRANING_FAIL;
}

static int mtk_dp_training_handle_v2(struct mtk_dp *mtk_dp)
{
	int ret = DP_RET_NOERR;

	if (!mtk_dp->training_info.cable_plug_in || mtk_dp->dp_ready)
		return DP_RET_PLUG_OUT;

	mtk_dp_fec_disable_v2(mtk_dp);

	ret = mtk_dp_set_training_start_v2(mtk_dp);
	if (ret == DP_RET_NOERR)
		mtk_dp->dp_ready = true;
	else
		dev_err(mtk_dp->dev, "[DPTX] Handle Training Fail 6 times\n");

	return ret;
}

static int mtk_dp_audio_hw_params_v2(struct device *dev, void *data,
				  struct hdmi_codec_daifmt *daifmt,
				  struct hdmi_codec_params *params)
{
	struct mtk_dp *mtk_dp = data;
	enum dp_encoder_id encoder_id = GET_DP_ENCODER_ID(dev, mtk_dp->dev);

	mtk_dp->info[encoder_id].audio_cur_cfg.channels = params->cea.channels;
	mtk_dp->info[encoder_id].audio_cur_cfg.word_length_bits = params->sample_width;
	mtk_dp->info[encoder_id].audio_cur_cfg.sample_rate = params->sample_rate;
	mtk_dp->audio_codec_on[encoder_id] = true;
	mtk_dp_audio_config_v2(mtk_dp, encoder_id);

	return 0;
}

static void mtk_dp_audio_shutdown_v2(struct device *dev, void *data)
{
	struct mtk_dp *mtk_dp = data;
	enum dp_encoder_id encoder_id = GET_DP_ENCODER_ID(dev, mtk_dp->dev);

	mtk_dp->audio_codec_on[encoder_id] = false;
	mtk_dp_audio_mute_v2(mtk_dp, encoder_id, true);
}

static int mtk_dp_audio_get_eld_v2(struct device *dev, void *data, uint8_t *buf,
				size_t len)
{
	struct mtk_dp *mtk_dp = data;
	int con_id = -ENODEV;
	u8 i, encoder_id;
	enum dp_encoder_id target_encoder_id = GET_DP_ENCODER_ID(dev, mtk_dp->dev);

	if (!mtk_dp->mst_enable) {
		con_id = DP_FIRST_CON;
	} else {
		for (i = 0; i < ARRAY_SIZE(mtk_dp->mtk_con); i++) {
			if (mst_con_with_encoder(mtk_dp->mtk_con[i])) {
				encoder_id = mtk_dp->mtk_con[i]->encoder_id;
				drm_dbg_kms(mtk_dp->drm_dev,
					    "[DPTX] MST, enc[%d] con[%d], video enable:%d, detect monitor:%d\n",
					    encoder_id, i, mtk_dp->mtk_con[i]->video_enable,
					    mtk_dp->info[encoder_id].audio_cur_cfg.detect_monitor);

				if (mtk_dp->mtk_con[i]->video_enable &&
				    mtk_dp->info[encoder_id].audio_cur_cfg.detect_monitor &&
				    encoder_id == target_encoder_id) {
					con_id = i;
					break;
				}
			}
		}
	}

	if (con_id < 0 || !mtk_dp->mtk_con[con_id]) {
		dev_info(mtk_dp->dev, "audio eld not found!\n");
		memset(buf, 0, len);
	} else {
		memcpy(buf, mtk_dp->mtk_con[con_id]->connector.eld, len);
	}

	return 0;
}

static int mtk_dp_audio_hook_plugged_cb_v2(struct device *dev, void *data,
					   hdmi_codec_plugged_cb fn,
					   struct device *codec_dev)
{
	struct mtk_dp *mtk_dp = data;
	enum dp_encoder_id encoder_id = GET_DP_ENCODER_ID(dev, mtk_dp->dev);

	mutex_lock(&mtk_dp->update_plugged_status_lock);
	mtk_dp->plugged_cb = fn;
	mtk_dp->codec_dev[encoder_id] = codec_dev;
	mutex_unlock(&mtk_dp->update_plugged_status_lock);

	mtk_dp_audio_update_plugged_status_v2(mtk_dp);

	return 0;
}

static void mtk_dp_audio_trigger_work(struct work_struct *work)
{
	struct mtk_dp *mtk_dp = container_of(work, struct mtk_dp, audio_work);
	enum dp_encoder_id encoder_id;
	u32 reg_offset;
	u32 audio_sram;
	bool full, empty;
	u32 write_0_1;
	u32 write_2_3;
	unsigned long timeout;

	for (encoder_id = 0; encoder_id < mtk_dp->data->encoder_num; encoder_id++) {
		if (!mtk_dp->audio_codec_on[encoder_id])
			continue;

		reg_offset = DP_REG_OFFSET(encoder_id);

		mtk_dp_audio_mute_v2(mtk_dp, encoder_id, true);

		timeout = jiffies + msecs_to_jiffies(AUDIO_SRAM_RESET_TIMEOUT_MS);
		while (time_before(jiffies, timeout)) {
			WRITE_BYTE_MASK(mtk_dp, REG_3088_DP_ENCODER0_P0 + reg_offset,
					AU_EN_DP_ENCODER0_P0_FLDMASK, AU_EN_DP_ENCODER0_P0_FLDMASK);
			udelay(50);
			WRITE_BYTE_MASK(mtk_dp, REG_3088_DP_ENCODER0_P0 + reg_offset,
					0x0, AU_EN_DP_ENCODER0_P0_FLDMASK);

			WRITE_2BYTE_MASK(mtk_dp, REG_33EC_DP_ENCODER1_P0 + reg_offset,
					 AU_SRAM_FULL_CLR_DP_ENCODER1_P0_FLDMASK,
					 AU_SRAM_FULL_CLR_DP_ENCODER1_P0_FLDMASK);
			udelay(20);
			WRITE_2BYTE_MASK(mtk_dp, REG_33EC_DP_ENCODER1_P0 + reg_offset,
					 0, AU_SRAM_FULL_CLR_DP_ENCODER1_P0_FLDMASK);

			WRITE_2BYTE_MASK(mtk_dp, REG_33F0_DP_ENCODER1_P0 + reg_offset,
					 AU_SRAM_CONTROL_RESET_DP_ENCODER1_P0_FLDMASK,
					 AU_SRAM_CONTROL_RESET_DP_ENCODER1_P0_FLDMASK);
			udelay(20);
			WRITE_2BYTE_MASK(mtk_dp, REG_33F0_DP_ENCODER1_P0 + reg_offset,
					 0x0, AU_SRAM_CONTROL_RESET_DP_ENCODER1_P0_FLDMASK);

			udelay(200);

			audio_sram = READ_2BYTE(mtk_dp, REG_33EC_DP_ENCODER1_P0 + reg_offset);
			write_0_1 = READ_2BYTE(mtk_dp, REG_32E8_DP_ENCODER1_P0 + reg_offset) &
					       AU_SRAM_WRITE_ADDR_0_DP_ENCODER1_P0_FLDMASK;
			write_2_3 = READ_2BYTE(mtk_dp, REG_32EC_DP_ENCODER1_P0 + reg_offset) &
					       AU_SRAM_WRITE_ADDR_2_DP_ENCODER1_P0_FLDMASK;

			full = !!(audio_sram & AU_SRAM_FULL_DP_ENCODER1_P0_FLDMASK);
			empty = !!(audio_sram & AU_SRAM_EMPTY_DP_ENCODER1_P0_FLDMASK);

			drm_dbg_kms(mtk_dp->drm_dev,
				    "[DPTX] Enc[%d] audio SRAM, full:%d, empty:%d, write:0x%x, 0x%x\n",
				    encoder_id, full, empty, write_0_1, write_2_3);

			if (!full && empty && !write_0_1 && !write_2_3)
				break;

			usleep_range(1000, 1100);
		}

		mtk_dp_audio_mute_v2(mtk_dp, encoder_id, false);
	}
}

static int mtk_dp_audio_trigger(struct device *dev, int event)
{
	struct mtk_dp *mtk_dp = dev_get_drvdata(dev);

	mtk_dp = mtk_dp ? mtk_dp : dev_get_drvdata(dev->parent);

	/*
	 * The audio module will send data to the DP only when
	 * the trigger callback function returns.
	 * We hope that the audio module can send data to the
	 * DP module as soon as possible without causing delay
	 * due to DP reset audio SRAM.
	 * Therefore, we adopt asynchronous work.
	 */
	if (event)
		schedule_work(&mtk_dp->audio_work);

	return 0;
}

static const struct hdmi_codec_ops mtk_dp_audio_codec_ops = {
	.hw_params = mtk_dp_audio_hw_params_v2,
	.audio_shutdown = mtk_dp_audio_shutdown_v2,
	.get_eld = mtk_dp_audio_get_eld_v2,
	.hook_plugged_cb = mtk_dp_audio_hook_plugged_cb_v2,
	.trigger = mtk_dp_audio_trigger,
	.no_capture_mute = 1,
};

static int mtk_dp_register_audio_driver_v2(struct device *dev)
{
	struct mtk_dp *mtk_dp = dev_get_drvdata(dev);
	struct hdmi_codec_pdata codec_data = {
		.ops = &mtk_dp_audio_codec_ops,
		.max_i2s_channels = 8,
		.i2s = 1,
		.data = mtk_dp,
	};

	mtk_dp->audio_pdev[DP_ENCODER_ID_0] =
		platform_device_register_data(dev,
					      HDMI_CODEC_DRV_NAME,
					      PLATFORM_DEVID_AUTO,
					      &codec_data,
					      sizeof(codec_data));
	if (IS_ERR(mtk_dp->audio_pdev[DP_ENCODER_ID_0])) {
		dev_err(dev, "failed to register audio device\n");
		return PTR_ERR(mtk_dp->audio_pdev[DP_ENCODER_ID_0]);
	}

	if (mtk_dp->data->encoder_num == 2) {
		int ret;

		mtk_dp->pseudo_audio_pdev =
			platform_device_alloc("pseudo-audio", PLATFORM_DEVID_NONE);
		if (!mtk_dp->pseudo_audio_pdev) {
			dev_err(dev, "failed to allocate pseudo-audio device\n");
			return -ENOMEM;
		}

		mtk_dp->pseudo_audio_pdev->dev.parent = dev;
		ret = platform_device_add(mtk_dp->pseudo_audio_pdev);
		if (ret) {
			dev_err(dev, "failed to add pseudo-audio device\n");
			platform_device_put(mtk_dp->pseudo_audio_pdev);
			return ret;
		}

		mtk_dp->audio_pdev[DP_ENCODER_ID_1] =
			platform_device_register_data(&mtk_dp->pseudo_audio_pdev->dev,
						      HDMI_CODEC_DRV_NAME,
						      PLATFORM_DEVID_NONE,
						      &codec_data,
						      sizeof(codec_data));
		return PTR_ERR_OR_ZERO(mtk_dp->audio_pdev[DP_ENCODER_ID_1]);
	}

	return 0;
}

static bool mtk_dp_link_ok_v2(struct mtk_dp *mtk_dp,
			      u8 link_status[DP_LINK_STATUS_SIZE])
{
	bool uhbr = mtk_dp->training_info.link_rate >= DP_LINK_RATE_UHBR10;
	bool ok;

	if (uhbr)
		ok = drm_dp_128b132b_lane_channel_eq_done(link_status,
							  mtk_dp->training_info.link_lane_count);
	else
		ok = drm_dp_channel_eq_ok(link_status, mtk_dp->training_info.link_lane_count);

	if (ok)
		return true;

	drm_dbg_kms(mtk_dp->drm_dev,
		    "[DPTX] ln0_1:0x%x ln2_3:0x%x align:0x%x sink:0x%x adj_req0_1:0x%x adj_req2_3:0x%x\n",
		    link_status[0], link_status[1], link_status[2],
		    link_status[3], link_status[4], link_status[5]);
	drm_dbg_kms(mtk_dp->drm_dev,
		    "[DPTX] %s link not ok, retraining\n", uhbr ? "128b/132b" : "8b/10b");

	return false;
}

static bool mtk_dp_mst_link_status(struct mtk_dp *mtk_dp)
{
	u8 link_status[DP_LINK_STATUS_SIZE] = {};
	const size_t esi_link_status_size = DP_LINK_STATUS_SIZE - 2;

	if (drm_dp_dpcd_read(&mtk_dp->aux, DP_LANE0_1_STATUS_ESI, link_status,
			     esi_link_status_size) != esi_link_status_size) {
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] fail to read link status\n");
		return false;
	}

	return mtk_dp_link_ok_v2(mtk_dp, link_status);
}

static bool mtk_dp_mst_hpd_event_handler(struct mtk_dp *mtk_dp)
{
	bool link_ok = true;
	bool handled = true;
	int retry;
	int rc;
	int i;

	for (i = 0; i < MST_HPD_EVENT_HANDLE_TIMES; i++) {
		u8 esi[4] = {};
		u8 ack[4] = {};

		rc = drm_dp_dpcd_read(&mtk_dp->aux, DP_SINK_COUNT_ESI, esi, 4);
		if (rc != 4) {
			dev_err(mtk_dp->dev, "[DPTX] fail to get ESI\n");
			link_ok = false;
			break;
		}
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] DPRX ESI: %4ph\n", esi);

		/* check link fail */
		if (link_ok && esi[3] & LINK_STATUS_CHANGED) {
			if (!mtk_dp_mst_link_status(mtk_dp))
				link_ok = false;
			ack[3] |= LINK_STATUS_CHANGED;
		}

		drm_dp_mst_hpd_irq_handle_event(&mtk_dp->mgr, esi, ack, &handled);

		if (esi[1] & DP_CP_IRQ) {
			drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] DP CP IRQ\n");
			ack[1] |= DP_CP_IRQ;
		}

		if (!memchr_inv(ack, 0, sizeof(ack)))
			break;

		for (retry = 0; retry < ACK_ESI_RETRY_TIMES; retry++) {
			rc = drm_dp_dpcd_write(&mtk_dp->aux, DP_SINK_COUNT_ESI + 1, &ack[1], 3);
			if (rc == 3)
				break;
		}
		if (retry == ACK_ESI_RETRY_TIMES)
			dev_err(mtk_dp->dev, "[DPTX] fail to ack ESI\n");

		if (ack[1] & (DP_DOWN_REP_MSG_RDY | DP_UP_REQ_MSG_RDY))
			drm_dp_mst_hpd_irq_send_new_request(&mtk_dp->mgr);

		usleep_range(1000, 1100);
	}

	if (!link_ok) {
		dev_err(mtk_dp->dev, "[DPTX] mst link fail, training again\n");
		mtk_dp->dp_ready = false;
		mtk_dp_training_handle_v2(mtk_dp);
		return false;
	}

	return true;
}

static void mtk_dp_check_device_service_irq_v2(struct mtk_dp *mtk_dp)
{
	u8 val;

	if (mtk_dp->rx_cap[DP_DPCD_REV] < 0x11)
		return;

	if (drm_dp_dpcd_readb(&mtk_dp->aux,
			      DP_DEVICE_SERVICE_IRQ_VECTOR, &val) != 1 || !val)
		return;

	drm_dp_dpcd_writeb(&mtk_dp->aux, DP_DEVICE_SERVICE_IRQ_VECTOR, val);

	if (val & DP_AUTOMATED_TEST_REQUEST)
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] DP_AUTOMATED_TEST_REQUEST\n");

	if (val & DP_CP_IRQ)
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] DP_CP_IRQ\n");

	if (val & DP_SINK_SPECIFIC_IRQ)
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] Sink specific irq unhandled\n");
}

static bool mtk_dp_check_link_service_irq_v2(struct mtk_dp *mtk_dp)
{
	bool reprobe_needed = false;
	u8 val;

	if (mtk_dp->rx_cap[DP_DPCD_REV] < 0x11)
		return false;

	if (drm_dp_dpcd_readb(&mtk_dp->aux,
			      DP_LINK_SERVICE_IRQ_VECTOR_ESI0, &val) != 1 || !val)
		return false;

	if (drm_dp_dpcd_writeb(&mtk_dp->aux,
			       DP_LINK_SERVICE_IRQ_VECTOR_ESI0, val) != 1)
		return reprobe_needed;

	if (val & HDMI_LINK_STATUS_CHANGED)
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] link status changed!\n");

	return reprobe_needed;
}

static bool mtk_dp_link_status_v2(struct mtk_dp *mtk_dp)
{
	u8 link_status[DP_LINK_STATUS_SIZE];

	if (!mtk_dp->dp_ready)
		return false;

	if (drm_dp_dpcd_read_phy_link_status(&mtk_dp->aux, DP_PHY_DPRX,
					     link_status) < 0)
		return false;

	/* Retrain if link not ok */
	return mtk_dp_link_ok_v2(mtk_dp, link_status);
}

static bool mtk_dp_hpd_event_handler_v2(struct mtk_dp *mtk_dp)
{
	u8 sink_cnt = 0;
	bool link_ok = true;
	bool reprobe_needed = false;

	sink_cnt = mtk_dp_get_sink_count_v2(mtk_dp);

	if (sink_cnt != mtk_dp->training_info.sink_count) {
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] sink count change:%d\n", sink_cnt);

		/* Connection Map:
		 * DPTX -> (*1) Branch (*2) -> RX (dprx/hdmirx/others)
		 *
		 * After sink count changed (0 -> 1),
		 * (downstream branch device plug-in cable *2),
		 * some branch devices seems to output nothing (RX no signal),
		 * we try to do link training again (*1),
		 * and then branch device display fine (*2).
		 */
		if (sink_cnt) {
			mtk_dp->dp_ready = false;
			mtk_dp_check_sink_cap_v2(mtk_dp);
			mtk_dp_training_handle_v2(mtk_dp);
		}

		/* we need to notify user the sink count changed */
		mtk_dp->training_info.sink_count = sink_cnt;
		mtk_dp_hotplug_uevent_v2(mtk_dp);
	}

	mtk_dp_check_device_service_irq_v2(mtk_dp);
	reprobe_needed = mtk_dp_check_link_service_irq_v2(mtk_dp);

	if (!mtk_dp_link_status_v2(mtk_dp))
		link_ok = false;

	if (!link_ok) {
		dev_err(mtk_dp->dev, "[DPTX] link fail, training again\n");
		mtk_dp->dp_ready = false;
		mtk_dp_training_handle_v2(mtk_dp);
		return false;
	}

	return !reprobe_needed;
}

static struct drm_property *mtk_dp_find_property_v2(struct drm_encoder *encoder, u8 *name, int size)
{
	struct drm_property *property, *pt;

	list_for_each_entry_safe(property, pt, &encoder->dev->mode_config.property_list, head) {
		if (property && !strncmp(property->name, name, size))
			return property;
	}

	return NULL;
}

static void mtk_dp_init_property_v2(struct mtk_dp *mtk_dp)
{
	struct drm_bridge *bridge;
	struct drm_property *prop;
	char dsc_enable[] = "dp_dsc_enable";
	char dsc_cfg[] = "dp_dsc_cfg";
	char result[20];
	u8 i;

	if (mtk_dp->init_property)
		return;

	for (i = 0; i < mtk_dp->data->encoder_num; i++) {
		bridge = devm_drm_of_get_bridge(mtk_dp->dev,
						mtk_dp->dev->of_node, i, DP_ENCODER_ENDPOINT);
		if (IS_ERR(bridge)) {
			drm_dbg_kms(mtk_dp->drm_dev,
				    "[DPTX] find encoder, can not find bridge[%d, %d]", i, 0);
			continue;
		}
		if (!bridge->encoder) {
			drm_dbg_kms(mtk_dp->drm_dev,
				    "[DPTX] find encoder, bridge have no encoder[%d, %d]", i, 0);
			continue;
		}
		drm_dbg_kms(mtk_dp->drm_dev,
			    "[DPTX] find encoder, found dp_intf[%d] bridge node:%pOF\n", i, bridge->of_node);

		snprintf(result, sizeof(result), "%s%d", dsc_enable, i);
		prop = mtk_dp_find_property_v2(bridge->encoder,
					       result, strlen(result));
		if (prop)
			mtk_dp->prop_dsc_enable[i] = prop;
		else
			dev_err(mtk_dp->dev, "[DPTX] [%d] fail to find property dp_dsc_enable", i);

		snprintf(result, sizeof(result), "%s%d", dsc_cfg, i);
		prop = mtk_dp_find_property_v2(bridge->encoder,
					       result, strlen(result));
		if (prop)
			mtk_dp->prop_dsc_cfg[i] = prop;
		else
			dev_err(mtk_dp->dev, "[DPTX] [%d] fail to find property dp_dsc_cfg", i);
	}

	mtk_dp->init_property = true;
}

static irqreturn_t mtk_dp_hpd_event_thread_v2(int hpd, void *dev)
{
	struct mtk_dp *mtk_dp = dev;
	bool cable_state_change;
	unsigned long flags;
	u16 phy_status;
	int i, ret;
	bool wait_done[DP_ENCODER_NUM] = {false};
	u8 wait_done_count = 0;
	unsigned long timeout = 0;

	if (mtk_dp->need_debounce && mtk_dp->training_info.cable_plug_in) {
		msleep(HPD_DEBOUNCE);
		mtk_dp->need_debounce = false;
	}

	spin_lock_irqsave(&mtk_dp->irq_thread_lock, flags);
	cable_state_change = mtk_dp->training_info.cable_state_change;
	phy_status = mtk_dp->training_info.phy_status;
	spin_unlock_irqrestore(&mtk_dp->irq_thread_lock, flags);

	mtk_dp->training_info.cable_state_change = false;
	if (mtk_dp->training_info.phy_status & HPD_INT_EVNET)
		mtk_dp->training_info.phy_status &= ~HPD_INT_EVNET;

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] cable_state_change:0x%x, phy_status:0x%x\n",
		    cable_state_change, phy_status);

	if (cable_state_change) {
		if (!mtk_dp->training_info.cable_plug_in) {
			drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] HPD_DISCON\n");
			mtk_dp_hotplug_uevent_v2(mtk_dp);
			mtk_dp_disconnect_release_v2(mtk_dp);

			/* Wait until crtc of current encoder is disabled */
			timeout = jiffies + msecs_to_jiffies(2000);
			while (time_before(jiffies, timeout)) {
				for (i = 0; i < mtk_dp->data->encoder_num; i++) {
					if (wait_done[i])
						continue;

					if (mtk_dp->mtk_con[i] && mtk_dp->mtk_con[i]->encoder &&
					    mtk_dp->mtk_con[i]->encoder->crtc &&
					    mtk_dp->mtk_con[i]->encoder->crtc->state->enable)
						continue;

					wait_done[i] = true;
					wait_done_count++;
					drm_dbg_kms(mtk_dp->drm_dev,
						    "[DPTX] con[%d] crtc is disabled in HPD low\n",
						    i);
				}

				if (wait_done_count == mtk_dp->data->encoder_num)
					break;

				msleep(100);
			}

			for (i = 0; i < mtk_dp->data->encoder_num; i++) {
				if (wait_done[i])
					continue;

				dev_err(mtk_dp->dev,
					"[DPTX] connector[%d] crtc is not disabled in HPD low\n", i);
			}
		} else {
			drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] HPD_CON\n");

			if (mtk_dp->phy_flip_pin)
				mtk_dp->phy_flip_enable = gpiod_get_value_cansleep(mtk_dp->phy_flip_pin);

			mtk_dp_init_property_v2(mtk_dp);
			mtk_dp_initial_setting_v2(mtk_dp);
			mtk_dp_analog_power_on_v2(mtk_dp);
			mtk_dp_phy_setting_v2(mtk_dp);

			for (i = 0; i < DP_CHECK_SINK_CAP_TIMEOUT_COUNT; i++) {
				if (mtk_dp_check_sink_cap_v2(mtk_dp))
					break;

				if (!mtk_dp->training_info.cable_plug_in)
					goto end;

				msleep(100);
			}

			ret = drm_dp_link_power_up(&mtk_dp->aux, mtk_dp->training_info.dpcd_rev);
			if (ret != 0)
				return IRQ_NONE;

			mtk_dp_training_handle_v2(mtk_dp);

			mtk_dp_hotplug_uevent_v2(mtk_dp);
		}
	}

	if (phy_status & HPD_INT_EVNET) {
		/*
		 * when the link is unsuccessful (removed or unstable),
		 * returning IRQ_NONE will prevent further processing
		 */
		if (mtk_dp->mst_start) {
			if (!mtk_dp_mst_hpd_event_handler(mtk_dp))
				return IRQ_NONE;
		} else if (!mtk_dp_hpd_event_handler_v2(mtk_dp))
			return IRQ_NONE;
	}

end:
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] event thread done\n");
	return IRQ_HANDLED;
}

static void mtk_dp_hpd_handle_in_isr_v2(struct mtk_dp *mtk_dp)
{
	bool current_hpd = mtk_dp_hpd_get_pin_level_v2(mtk_dp);

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] current_hpd:0x%x, phy_status:0x%x\n",
		    current_hpd, mtk_dp->training_info.phy_status);

	if (mtk_dp->training_info.phy_status == HPD_INITIAL_STATE)
		return;

	if ((mtk_dp->training_info.phy_status & (HPD_CONNECT | HPD_DISCONNECT))
		== (HPD_CONNECT | HPD_DISCONNECT)) {
		if (current_hpd)
			mtk_dp->training_info.phy_status &= ~HPD_DISCONNECT;
		else
			mtk_dp->training_info.phy_status &= ~HPD_CONNECT;
	}

	if ((mtk_dp->training_info.phy_status & (HPD_INT_EVNET | HPD_DISCONNECT))
		== (HPD_INT_EVNET | HPD_DISCONNECT)) {
		if (current_hpd)
			mtk_dp->training_info.phy_status &= ~HPD_DISCONNECT;
	}

	/* ignore plug-in --> plug-in event */
	if (mtk_dp->training_info.cable_plug_in)
		mtk_dp->training_info.phy_status &= ~HPD_CONNECT;
	else
		mtk_dp->training_info.phy_status &= ~HPD_DISCONNECT;

	if (mtk_dp->training_info.phy_status & HPD_CONNECT) {
		mtk_dp->training_info.phy_status &= ~HPD_CONNECT;
		mtk_dp->training_info.cable_plug_in = true;
		mtk_dp->training_info.cable_state_change = true;
		mtk_dp->need_debounce = true;

		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] HPD_CON_ISR\n");
	}

	if (mtk_dp->training_info.phy_status & HPD_DISCONNECT) {
		mtk_dp->training_info.phy_status &= ~HPD_DISCONNECT;

		mtk_dp->training_info.cable_plug_in = false;
		mtk_dp->training_info.cable_state_change = true;
		mtk_dp->need_debounce = true;

		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] HPD_DISCON_ISR\n");
	}

	/* handle IRQ in thread */
	if (mtk_dp->training_info.phy_status & HPD_INT_EVNET)
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] ****** HPD_INT ******\n");
}

static irqreturn_t mtk_dp_hpd_event_v2(int hpd, void *dev)
{
	struct mtk_dp *mtk_dp = dev;
	u32 int_status;
	u16 hw_status;
	unsigned long flags;

	int_status = READ_4BYTE(mtk_dp, DP_TX_TOP_IRQ_STATUS);

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] int_status = 0x%x\n", int_status);

	if (int_status & BIT(3))
		WRITE_4BYTE_MASK(mtk_dp, DP_TX_TOP_IRQ_MASK, ENCODER_1_IRQ_MSK, ENCODER_1_IRQ_MSK);

	if (int_status & BIT(0))
		WRITE_4BYTE_MASK(mtk_dp, DP_TX_TOP_IRQ_MASK, ENCODER_IRQ_MSK, ENCODER_IRQ_MSK);

	if ((int_status & BIT(2)) || (int_status & BIT(1))) {
		if (int_status & BIT(1))
			WRITE_4BYTE_MASK(mtk_dp, DP_TX_TOP_IRQ_MASK, TRANS_IRQ_MSK, TRANS_IRQ_MSK);

		spin_lock_irqsave(&mtk_dp->irq_thread_lock, flags);

		hw_status = mtk_dp_hpd_get_irq_status_v2(mtk_dp);
		if (hw_status != 0)
			drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] hw status:0x%x\n", hw_status);

		mtk_dp->training_info.phy_status |= hw_status;

		mtk_dp_hpd_handle_in_isr_v2(mtk_dp);

		if (mtk_dp->training_info.cable_state_change)
			drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] cable_state_change:0x%x, hw_status:%x\n",
				    mtk_dp->training_info.cable_state_change, hw_status);

		if (hw_status)
			mtk_dp_hpd_interrupt_clr_v2(mtk_dp, hw_status);

		spin_unlock_irqrestore(&mtk_dp->irq_thread_lock, flags);
	}

	return IRQ_WAKE_THREAD;
}

static int mtk_dp_dt_parse_pdata_v2(struct mtk_dp *mtk_dp,
				 struct platform_device *pdev)
{
	struct resource regs;
	struct device *dev = &pdev->dev;
	struct platform_device *vdisp_ao_pdev;
	struct device_node *vdisp_ao_node;
	int ret = 0;
	u32 phy_params_int[DP_PHY_REG_COUNT] = {
		0x20181410, 0x20241e18, 0x00003028,
		0x10080400, 0x000c0600, 0x00000008
	};
	u32 phy_params_dts[DP_PHY_REG_COUNT];

	if (of_address_to_resource(dev->of_node, 0, &regs) != 0)
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] Missing reg[0] in %s node\n",
			    dev->of_node->full_name);

	if (of_address_to_resource(dev->of_node, 1, &regs) != 0)
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] Missing reg[1] in %s node\n",
			    dev->of_node->full_name);

	mtk_dp->regs = of_iomap(dev->of_node, 0);
	mtk_dp->phyd_regs = of_iomap(dev->of_node, 1);
	mtk_dp->phy_mux_regs = of_iomap(dev->of_node, 2);
	mtk_dp->mac_power_regs = of_iomap(dev->of_node, 3);

	mtk_dp_phy_param_init_v2(mtk_dp, phy_params_int, ARRAY_SIZE(phy_params_int));

	mtk_dp->phy_by_link_rate = of_property_read_bool(dev->of_node, "phy-by-link-rate");

	if (mtk_dp->phy_by_link_rate) {
		ret = of_property_read_u32_array(dev->of_node, "phy-params",
						 phy_params_dts, ARRAY_SIZE(phy_params_dts));
		if (ret) {
			memcpy(mtk_dp->phy_settings, phy_params_int, sizeof(phy_params_int));
			drm_dbg_kms(mtk_dp->drm_dev,
				    "[DPTX] get phy-params fail, use default val, ret:%d\n", ret);
		} else {
			memcpy(mtk_dp->phy_settings, phy_params_dts, sizeof(phy_params_dts));
		}

		ret = of_property_read_u32_array(dev->of_node, "phy-params-hbr2",
						 phy_params_dts, ARRAY_SIZE(phy_params_dts));
		if (ret) {
			memcpy(mtk_dp->phy_settings_hbr2, phy_params_int, sizeof(phy_params_int));
			drm_dbg_kms(mtk_dp->drm_dev,
			    "[DPTX] get phy-params-hbr2 fail, use default val, ret:%d\n", ret);
		} else {
			memcpy(mtk_dp->phy_settings_hbr2, phy_params_dts, sizeof(phy_params_dts));
		}
	}

	vdisp_ao_node = of_parse_phandle(dev->of_node, "mediatek,vdisp-ao", 0);
	if (!vdisp_ao_node) {
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] mediatek,vdisp-ao property not found\n");
		return 0;
	}

	vdisp_ao_pdev = of_find_device_by_node(vdisp_ao_node);
	of_node_put(vdisp_ao_node);
	if (WARN_ON(!vdisp_ao_pdev)) {
		dev_err(dev, "[DPTX] vdisp-ao find pdev failed\n");
		return -ENODEV;
	}
	mtk_dp->vdisp_ao_dev =  &vdisp_ao_pdev->dev;
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] vdiso-ao get dev success\n");

	return 0;
}

static void mtk_dp_enable_mac_power_v2(struct mtk_dp *mtk_dp)
{
	writel((readl(mtk_dp->mac_power_regs) & ~BIT(0)), mtk_dp->mac_power_regs);
	writel((readl(mtk_dp->mac_power_regs) | BIT(4)), mtk_dp->mac_power_regs);

	writel((readl(mtk_dp->mac_power_regs) | BIT(0)), mtk_dp->mac_power_regs);
	writel((readl(mtk_dp->mac_power_regs) & ~BIT(4)), mtk_dp->mac_power_regs);
}

static void mtk_dp_disable_mac_power_v2(struct mtk_dp *mtk_dp)
{
	writel((readl(mtk_dp->mac_power_regs) & ~BIT(0)), mtk_dp->mac_power_regs);
	writel((readl(mtk_dp->mac_power_regs) | BIT(4)), mtk_dp->mac_power_regs);
}

#ifdef CONFIG_PM_SLEEP
static int mtk_dp_suspend_v2(struct device *dev)
{
	struct mtk_dp *mtk_dp = dev_get_drvdata(dev);

	if (!mtk_dp) {
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] suspend, dp not initial\n");
		return 0;
	}

	if (mtk_dp->disp_state == DP_DISP_STATE_SUSPENDING ||
	    mtk_dp->disp_state == DP_DISP_STATE_SUSPEND) {
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] have suspended and do nothing\n");
		return 0;
	}

	drm_dp_link_power_down(&mtk_dp->aux, mtk_dp->training_info.dpcd_rev);

	mtk_dp->disp_state = DP_DISP_STATE_SUSPENDING;

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] suspend +\n");

	mtk_dp_hpd_interrupt_enable_v2(mtk_dp, false);

	mtk_dp_disconnect_release_v2(mtk_dp);

	mtk_dp_disable_mac_power_v2(mtk_dp);

	clk_disable_unprepare(mtk_dp->pclk);

	mtk_dp->disp_state = DP_DISP_STATE_SUSPEND;
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] pm_runtime_put_sync\n");
	pm_runtime_put_sync(mtk_dp->dev);

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] suspend -\n");

	return 0;
}

static int mtk_dp_resume_v2(struct device *dev)
{
	struct mtk_dp *mtk_dp = dev_get_drvdata(dev);
	int ret = 0;

	if (!mtk_dp) {
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] resume, dp not initial\n");
		return 0;
	}

	if (mtk_dp->disp_state == DP_DISP_STATE_RESUME) {
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] have resumed and do nothing\n");
		return 0;
	}

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] pm_runtime_get_sync\n");
	pm_runtime_get_sync(dev);
	mtk_dp->disp_state = DP_DISP_STATE_RESUME;

	ret = clk_prepare_enable(mtk_dp->pclk);
	if (ret) {
		dev_err(mtk_dp->dev, "[DPTX] Failed to prepare and enable clock\n");
		return ret;
	}

	mtk_dp_enable_mac_power_v2(mtk_dp);

	mtk_dp_init_port_v2(mtk_dp);

	/* Set vdisp_ao default config to get merge irq normally */
	if (mtk_dp->vdisp_ao_dev)
		mtk_mmsys_default_config(mtk_dp->vdisp_ao_dev);

	mtk_dp_hpd_interrupt_enable_v2(mtk_dp, true);

	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] resume done\n");

	return 0;
}
#endif

static int mtk_drm_dp_notifier_v2(struct notifier_block *notifier,
			       unsigned long pm_event, void *unused)
{
	struct mtk_dp *mtk_dp = container_of(notifier, struct mtk_dp, notifier);
	struct device *dev = mtk_dp->dev;

	drm_dbg_kms(mtk_dp->drm_dev,
		    "[DPTX] %s pm_event %lu dev %s usage_count %d nb priority %d\n",
		    __func__, pm_event, dev_name(dev), atomic_read(&dev->power.usage_count),
		    notifier->priority);

	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		mtk_dp_suspend_v2(dev);
		return NOTIFY_OK;
	case PM_POST_SUSPEND:
		mtk_dp_resume_v2(dev);
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static int mtk_dp_pm_init_v2(struct mtk_dp *mtk_dp, struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;

	mtk_dp->genpd_dp_tx = dev_pm_domain_attach_by_name(dev, "pd_dp_tx");
	if (IS_ERR_OR_NULL(mtk_dp->genpd_dp_tx)) {
		ret = PTR_ERR(mtk_dp->genpd_dp_tx) ? : -ENODATA;
		drm_dbg_kms(mtk_dp->drm_dev,
			    "[DPTX] failed to attach pd_dp_tx pm-domain: %d\n", ret);
		return -ENODEV;
	}

	mtk_dp->genpd_dp_phy = dev_pm_domain_attach_by_name(dev, "pd_dp_phy");
	if (IS_ERR_OR_NULL(mtk_dp->genpd_dp_phy)) {
		ret = PTR_ERR(mtk_dp->genpd_dp_phy) ? : -ENODATA;
		drm_dbg_kms(mtk_dp->drm_dev,
			    "[DPTX] failed to attach pd_dp_phy pm-domain: %d\n", ret);
		return -ENODEV;
	}

	mtk_dp->genpd_dl_dp_tx = device_link_add(dev, mtk_dp->genpd_dp_tx,
						 DL_FLAG_PM_RUNTIME |
						 DL_FLAG_STATELESS);
	if (!mtk_dp->genpd_dl_dp_tx) {
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] failed to add dp tx link\n");
		return -ENODEV;
	}

	mtk_dp->genpd_dl_dp_phy = device_link_add(dev, mtk_dp->genpd_dp_phy,
						  DL_FLAG_PM_RUNTIME |
						  DL_FLAG_STATELESS);
	if (!mtk_dp->genpd_dl_dp_phy) {
		drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] failed to add dp phy link\n");
		return -ENODEV;
	}

	return 0;
}

static int mtk_drm_dp_probe_v2(struct platform_device *pdev)
{
	struct mtk_dp *mtk_dp;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int ret;
	struct mtk_drm_private *mtk_priv = dev_get_drvdata(dev);
	int i = 0;

	mtk_dp = devm_kzalloc(dev, sizeof(*mtk_dp), GFP_KERNEL);
	if (!mtk_dp)
		return -ENOMEM;

	memset(mtk_dp, 0, sizeof(struct mtk_dp));
	mtk_dp->id = 0x0;
	mtk_dp->dev = dev;
	mtk_dp->priv = mtk_priv;
	mtk_dp->disp_state = DP_DISP_STATE_NONE;

	mtk_dp->data = (struct mtk_dp_data *)of_device_get_match_data(dev);

	ret = mtk_dp_pm_init_v2(mtk_dp, pdev);
	if (ret)
		return ret;

	pm_runtime_enable(mtk_dp->dev);
	drm_dbg_kms(mtk_dp->drm_dev, "[DPTX] pm_runtime_get_sync\n");
	pm_runtime_get_sync(mtk_dp->dev);

	mtk_dp->irq = platform_get_irq(pdev, 0);
	if (mtk_dp->irq < 0) {
		dev_err(mtk_dp->dev, "[DPTX] failed to request dp irq resource\n");
		return -EPROBE_DEFER;
	}

	ret = mtk_dp_dt_parse_pdata_v2(mtk_dp, pdev);
	if (ret)
		return ret;

	mtk_dp_aux_init_v2(mtk_dp);

	platform_set_drvdata(pdev, mtk_dp);

	if (mtk_dp->data->audio_supported) {
		mutex_init(&mtk_dp->update_plugged_status_lock);

		ret = mtk_dp_register_audio_driver_v2(dev);
		if (ret) {
			dev_err(mtk_dp->dev, "[DPTX] Failed to register audio driver: %d\n", ret);
			return ret;
		}
	}

	dev_dbg(mtk_dp->dev, "irq:%d\n", mtk_dp->irq);
	spin_lock_init(&mtk_dp->irq_thread_lock);

	ret = devm_request_threaded_irq(&pdev->dev, mtk_dp->irq, mtk_dp_hpd_event_v2,
					mtk_dp_hpd_event_thread_v2,
					IRQF_NO_AUTOEN, dev_name(&pdev->dev),
					mtk_dp);
	if (ret) {
		dev_err(mtk_dp->dev, "[DPTX] failed to request mediatek dp irq\n");
		return -EPROBE_DEFER;
	}

	mutex_init(&mtk_dp->hdcp_mutex);
	mutex_init(&mtk_dp->mst_mutex);
	init_waitqueue_head(&mtk_dp->hdcp_info.hdcp2_info.cp_irq_queue);
	INIT_WORK(&mtk_dp->hdcp_enable_work, mtk_dp_hdcp_enable_handle);
	INIT_WORK(&mtk_dp->hdcp_disable_work, mtk_dp_hdcp_disable_handle);
	INIT_WORK(&mtk_dp->prop_work, mtk_dp_hdcp_prop_work);
	INIT_WORK(&mtk_dp->audio_work, mtk_dp_audio_trigger_work);
	INIT_DELAYED_WORK(&mtk_dp->check_work, mtk_dp_hdcp_check_work);
	mtk_dp->hdcp_workqueue = create_workqueue("mtk_dp_hdcp_work");
	if (!mtk_dp->hdcp_workqueue) {
		dev_err(mtk_dp->dev, "[DPTX] failed to create hdcp work queue");
		return -ENOMEM;
	}

	mtk_dp_enable_mac_power_v2(mtk_dp);

	mtk_dp->pclk = devm_clk_get_enabled(mtk_dp->dev, "mux_dp");
	if (IS_ERR(mtk_dp->pclk))
		return dev_err_probe(dev, PTR_ERR(mtk_dp->pclk),
				     "[DPTX] Failed to get pixel clock\n");
	mtk_dp->pclk_src = devm_clk_get(mtk_dp->dev, "ck_26m");
	if (IS_ERR(mtk_dp->pclk_src))
		return dev_err_probe(dev, PTR_ERR(mtk_dp->pclk_src),
				     "[DPTX] Failed to get pixel source clock\n");

	ret = clk_set_parent(mtk_dp->pclk, mtk_dp->pclk_src);
	if (ret < 0)
		dev_err(mtk_dp->dev, "[DPTX] Failed to clk_set_parent:%d\n", ret);

	dev_dbg(mtk_dp->dev, "pclk:%ld\n", clk_get_rate(mtk_dp->pclk));

	mtk_dp->notifier.notifier_call = mtk_drm_dp_notifier_v2;
	ret = register_pm_notifier(&mtk_dp->notifier);
	if (ret)
		dev_err(mtk_dp->dev, "[DPTX] register pm notifier failed %d", ret);

	for_each_of_graph_port(np, port) {
		if (i >= mtk_dp->data->encoder_num)
			break;

		dev_dbg(dev, "[DPTX] port:%pOF\n", port);

		mtk_dp->mtk_bridge[i] = devm_kzalloc(dev, sizeof(struct mtk_dp_bridge), GFP_KERNEL);
		if (!mtk_dp->mtk_bridge[i])
			return -ENOMEM;

		mtk_dp->mtk_bridge[i]->bridge.funcs = &mtk_dp_bridge_funcs;
		mtk_dp->mtk_bridge[i]->bridge.of_node = port;
		mtk_dp->mtk_bridge[i]->bridge.type = mtk_dp->data->bridge_type;
		drm_bridge_add(&mtk_dp->mtk_bridge[i]->bridge);

		mtk_dp->mtk_bridge[i]->mtk_dp = mtk_dp;
		mtk_dp->mtk_bridge[i]->encoder_id = i;

		i++;
	}

	atomic_set(&mtk_dp->refcount, 0);

	mtk_dp->phy_flip_pin = devm_gpiod_get_optional(dev, "mediatek,phy-flip", GPIOD_ASIS);
	if (IS_ERR(mtk_dp->phy_flip_pin))
		return PTR_ERR(mtk_dp->phy_flip_pin);

	return ret;
}

static int mtk_drm_dp_remove_v2(struct platform_device *pdev)
{
	struct mtk_dp *mtk_dp = platform_get_drvdata(pdev);
	int ret = 0;
	int i;

	for (i = 0; i < mtk_dp->data->encoder_num; i++)
		if (mtk_dp->mtk_bridge[i])
			drm_bridge_remove(&mtk_dp->mtk_bridge[i]->bridge);

	/* unregister pm notifier */
	ret = unregister_pm_notifier(&mtk_dp->notifier);
	if (ret)
		dev_err(mtk_dp->dev, "[DPTX] unregister_pm_notifier failed %d", ret);

	clk_disable_unprepare(mtk_dp->pclk);
	mtk_dp_disable_mac_power_v2(mtk_dp);

	for (i = 0; i < DP_ENCODER_NUM; i++) {
		if (mtk_dp->audio_pdev[i])
			platform_device_unregister(mtk_dp->audio_pdev[i]);
	}

	if (mtk_dp->pseudo_audio_pdev)
		platform_device_unregister(mtk_dp->pseudo_audio_pdev);

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return ret;
}

static SIMPLE_DEV_PM_OPS(mtk_dp_pm_ops,
		mtk_dp_suspend_v2, mtk_dp_resume_v2);

static const struct mtk_dp_efuse_fmt mt8196_dp_efuse_fmt[5] = {0};

static const struct mtk_dp_data mt8196_dp_data = {
	.bridge_type = DRM_MODE_CONNECTOR_DisplayPort,
	.smc_cmd = BIT(5),
	.efuse_fmt = mt8196_dp_efuse_fmt,
	.audio_supported = true,
	.audio_m_div2_bit = 0,
	.dsc_support = true,
	.mst_support = true,
	.max_hdisplay = 3840,
	.max_vdisplay = 2160,
	.min_hblanking = 80,
	.min_hdisplay = 800,
	.min_vdisplay = 600,
	.set_param = mtk_mt8196_dp_phy_set_param_v2,
	.phyd_dig_glb_offset = 0x1000,
	.phyd_dig_lan_offset = {0x1100, 0x1200, 0x1300, 0x1400},
	.max_link_rate = DP_LINK_RATE_HBR3,
	.max_lane_count = DP_2LANE,
	.encoder_num = 2,
	.need_phy_lane_enable_set = true,
	.need_phy_flip_set = true,
	.phy_4lane_ctrl_bit = BIT(19),
	.phy_flip_ctrl_bit = BIT(18),
};

static const struct mtk_dp_data mt8189_dp_data = {
	.bridge_type = DRM_MODE_CONNECTOR_DisplayPort,
	.smc_cmd = BIT(5),
	.efuse_fmt = mt8196_dp_efuse_fmt,
	.audio_supported = true,
	.audio_m_div2_bit = 0,
	.max_hdisplay = 3840,
	.max_vdisplay = 2160,
	.min_hblanking = 80,
	.max_fps_at_max_resolution = 30,
	.min_hdisplay = 800,
	.min_vdisplay = 600,
	.patch_phy_settings = {0x21, 0x1000e, 0x20fff00, 0x20202, 0xa5231a, 0xdde70305},
	.txfir_settings = {0x20181410, 0x20241e18, 0x3028, 0x10080400, 0xc0600, 0x8},
	.set_param = mtk_mt8189_dp_phy_set_param_v2,
	.phyd_dig_glb_offset = 0x1400,
	.phyd_dig_lan_offset = {0x1000, 0x1100, 0x1200, 0x1300},
	.phyd_ana_glb_offset = 0x400,
	.phyd_ana_lan_offset = {0x0, 0x100, 0x200, 0x300},
	.max_link_rate = DP_LINK_RATE_HBR2,
	.max_lane_count = DP_2LANE,
	.encoder_num = 1,
	.phy_4lane_ctrl_bit = BIT(8),
	.phy_flip_ctrl_bit = BIT(7),
	.ssc_support = true,
};

static const struct of_device_id mtk_dp_of_match_v2[] = {
	{ .compatible = "mediatek,mt8196-dp-tx",
		.data = &mt8196_dp_data,
	},
	{ .compatible = "mediatek,mt8189-dp-tx",
		.data = &mt8189_dp_data,
	},
	{ },
};

MODULE_DEVICE_TABLE(of, mtk_dp_of_match_v2);

static struct platform_driver mtk_dp_tx_driver = {
	.probe = mtk_drm_dp_probe_v2,
	.remove = mtk_drm_dp_remove_v2,
	.driver = {
		.name = "mediatek-drm-dp-v2",
		.of_match_table = mtk_dp_of_match_v2,
		.pm = &mtk_dp_pm_ops,
	},
};

module_platform_driver(mtk_dp_tx_driver);

MODULE_AUTHOR("Jitao Shi <jitao.shi@mediatek.com>");
MODULE_AUTHOR("Markus Schneider-Pargmann <msp@baylibre.com>");
MODULE_AUTHOR("Bo-Chen Chen <rex-bc.chen@mediatek.com>");
MODULE_DESCRIPTION("MediaTek DisplayPort Driver");
MODULE_LICENSE("GPL");
