// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015 MediaTek Inc.
 */

#include <linux/arm-smccc.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/mailbox_controller.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/soc/mediatek/mtk-dpc.h>
#include <linux/soc/mediatek/mtk-cmdq.h>
#include <linux/soc/mediatek/mtk-mmsys.h>
#include <linux/soc/mediatek/mtk-mutex.h>
#include <linux/soc/mediatek/mtk_sip_svc.h>

#include <asm/barrier.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>
#include <drm/drm_vblank_work.h>

#include "mtk_crtc.h"
#include "mtk_ddp_comp.h"
#include "mtk_disp_pmqos.h"
#include "mtk_drm_drv.h"
#include "mtk_gem.h"
#include "mtk_plane.h"

struct mtk_crtc_qos_ctx {
	unsigned int last_hrt_req;
	unsigned int last_channel_hrt_req[BW_CHANNEL_NR];
	unsigned int last_plane_hrt_req[MAX_PLANE];
	unsigned int plane_hrt_req[MAX_PLANE];
	unsigned int last_plane_srt_req[MAX_PLANE];
	unsigned int plane_srt_req[MAX_PLANE];
	unsigned int plane_channel_id[MAX_PLANE];
};

/*
 * struct mtk_crtc - MediaTek specific crtc structure.
 * @base: crtc object.
 * @enabled: records whether crtc_enable succeeded
 * @planes: array of 4 drm_plane structures, one for each overlay plane
 * @pending_planes: whether any plane has pending changes to be applied
 * @mmsys_dev: pointer to the mmsys device for configuration registers
 * @mutex: handle to one of the ten disp_mutex streams
 * @ddp_comp_nr: number of components in ddp_comp
 * @ddp_comp: array of pointers the mtk_ddp_comp structures used by this crtc
 *
 * TODO: Needs update: this header is missing a bunch of member descriptions.
 */
struct mtk_crtc {
	struct drm_crtc			base;
	bool				enabled;

	bool				pending_needs_vblank;
	struct drm_pending_vblank_event	*event;

	struct drm_plane		*planes;
	unsigned int			layer_nr;
	bool				pending_planes;
	bool				pending_async_planes;

#if IS_REACHABLE(CONFIG_MTK_CMDQ)
	struct cmdq_client		cmdq_client;
	struct cmdq_pkt			cmdq_handle;
	u32				cmdq_event;
	u32				cmdq_vblank_cnt;
	wait_queue_head_t		cb_blocking_queue;
	struct task_struct		*cmdq_done_task;
	atomic_t			cmdq_done;

	struct cmdq_client		sec_cmdq_client;
	bool				sec_cmdq_working;
	wait_queue_head_t		sec_cb_blocking_queue;
#endif

	struct device			*mmsys_dev[MAX_MMSYS];
	struct device			*dma_dev;
	struct device			*vdisp_ao_dev;
	struct device			*dpc_dev;
	struct device			*pmqos_dev;
	struct mtk_mutex		*mutex[MAX_MMSYS];
	unsigned int			ddp_comp_nr;
	struct mtk_ddp_comp		**ddp_comp;
	enum mtk_drm_mmsys		*ddp_comp_sys;
	bool				exist[MAX_MMSYS];
	unsigned int			num_conn_routes;
	const struct mtk_drm_route	*conn_routes;
	enum mtk_drm_mmsys		conn_routes_sys;

	/* lock for display hardware access */
	struct mutex			hw_lock;
	bool				config_updating;
	/* lock for config_updating to cmd buffer */
	spinlock_t			config_lock;

	/* support crc */
	struct mtk_ddp_comp		*crc_provider;
	struct drm_vblank_work		crc_work;

	bool				sec_on;
	/* QoS setting*/
	struct mtk_crtc_qos_ctx		*qos_ctx;
};

struct mtk_crtc_state {
	struct drm_crtc_state		base;
	struct dsc_info			dsc;

	bool				pending_config;
	unsigned int			pending_width;
	unsigned int			pending_height;
	unsigned int			pending_vrefresh;
	unsigned int			pending_hrt_bw;
	unsigned int			pending_channel_bw[BW_CHANNEL_NR];
};

struct mtk_crtc_comp_info {
	enum mtk_drm_mmsys sys;
	unsigned int comp_id;
};

static inline struct mtk_crtc *to_mtk_crtc(struct drm_crtc *c)
{
	return container_of(c, struct mtk_crtc, base);
}

static inline struct mtk_crtc_state *to_mtk_crtc_state(struct drm_crtc_state *s)
{
	return container_of(s, struct mtk_crtc_state, base);
}

static void mtk_crtc_finish_page_flip(struct mtk_crtc *mtk_crtc)
{
	struct drm_crtc *crtc = &mtk_crtc->base;
	unsigned long flags;

	if (mtk_crtc->event) {
		spin_lock_irqsave(&crtc->dev->event_lock, flags);
		drm_crtc_send_vblank_event(crtc, mtk_crtc->event);
		drm_crtc_vblank_put(crtc);
		mtk_crtc->event = NULL;
		spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
	}
}

static void mtk_drm_finish_page_flip(struct mtk_crtc *mtk_crtc)
{
	unsigned long flags;

	drm_crtc_handle_vblank(&mtk_crtc->base);

#if IS_REACHABLE(CONFIG_MTK_CMDQ)
	if (mtk_crtc->cmdq_client.chan)
		return;
#endif

	spin_lock_irqsave(&mtk_crtc->config_lock, flags);
	if (!mtk_crtc->config_updating && mtk_crtc->pending_needs_vblank) {
		mtk_crtc_finish_page_flip(mtk_crtc);
		mtk_crtc->pending_needs_vblank = false;
	}
	spin_unlock_irqrestore(&mtk_crtc->config_lock, flags);
}

#if IS_REACHABLE(CONFIG_MTK_CMDQ)
static int mtk_drm_cmdq_pkt_create(struct cmdq_client *client, struct cmdq_pkt *pkt,
				   size_t size)
{
	struct device *dev;
	dma_addr_t dma_addr;

	pkt->va_base = kzalloc(size, GFP_KERNEL);
	if (!pkt->va_base)
		return -ENOMEM;

	pkt->buf_size = size;
	pkt->cl = (void *)client;

	dev = client->chan->mbox->dev;
	dma_addr = dma_map_single(dev, pkt->va_base, pkt->buf_size,
				  DMA_TO_DEVICE);
	if (dma_mapping_error(dev, dma_addr)) {
		dev_err(dev, "dma map failed, size=%u\n", (u32)(u64)size);
		kfree(pkt->va_base);
		return -ENOMEM;
	}

	pkt->pa_base = dma_addr;

	return 0;
}

static void mtk_drm_cmdq_pkt_destroy(struct cmdq_pkt *pkt)
{
	struct cmdq_client *client = (struct cmdq_client *)pkt->cl;

	dma_unmap_single(client->chan->mbox->dev, pkt->pa_base, pkt->buf_size,
			 DMA_TO_DEVICE);
	kfree(pkt->va_base);
	cmdq_sec_pkt_free_sec_data(pkt);
}
#endif

void mtk_crtc_disable_secure_state(struct drm_crtc *crtc)
{
#if IS_REACHABLE(CONFIG_MTK_CMDQ)
	int i;
	struct mtk_ddp_comp *ddp_first_comp;
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct cmdq_pkt *cmdq_handle;
	struct cmdq_sec_data *sec_data;

	if (!mtk_crtc->sec_cmdq_client.chan) {
		dev_err(crtc->dev->dev,
			"crtc-%d secure mbox channel is NULL\n", drm_crtc_index(crtc));
		return;
	}

	if (!mtk_crtc->sec_on) {
		DRM_DEV_DEBUG_DRIVER(crtc->dev->dev,
				     "crtc-%d is already disabled!\n", drm_crtc_index(crtc));
		return;
	}

	/* This will be freed in ddp_cmdq_cb(), it doesn't need to be freed in this function */
	cmdq_handle = kzalloc(sizeof(*cmdq_handle), GFP_KERNEL);
	if (!cmdq_handle){
		dev_err(crtc->dev->dev, "mtk_crtc %d failed to kzalloc secure cmdq packet\n",
			  drm_crtc_index(&mtk_crtc->base));
		return;
	}

	if (mtk_drm_cmdq_pkt_create(&mtk_crtc->sec_cmdq_client, cmdq_handle, PAGE_SIZE) < 0) {
		dev_err(crtc->dev->dev, "mtk_crtc %d failed to create secure cmdq packet\n",
			  drm_crtc_index(&mtk_crtc->base));
		kfree(cmdq_handle);
		return;
	}

	if (cmdq_sec_pkt_alloc_sec_data(cmdq_handle) < 0) {
		dev_err(crtc->dev->dev, "mtk_crtc %d failed to create secure cmdq packet data\n",
			  drm_crtc_index(&mtk_crtc->base));
		mtk_drm_cmdq_pkt_destroy(cmdq_handle);
		kfree(cmdq_handle);
		return;
	}
	/* make sure this disable secure layer command won't be dropped */
	sec_data = (struct cmdq_sec_data *)cmdq_handle->sec_data;
	sec_data->needs_vblank = true;

	/*
	 * Secure path only support DL mode, so we just wait
	 * the first path frame done here
	 */
	cmdq_pkt_clear_event(cmdq_handle, mtk_crtc->cmdq_event);
	cmdq_pkt_wfe(cmdq_handle, mtk_crtc->cmdq_event, false);

	ddp_first_comp = mtk_crtc->ddp_comp[0];
	for (i = 0; i < mtk_crtc->layer_nr; i++) {
		struct drm_plane *plane = &mtk_crtc->planes[i];
		struct mtk_plane_state *plane_state = to_mtk_plane_state(plane->state);

		/* make sure secure layer off before switching secure state */
		if (plane_state->pending.is_secure) {
			plane_state->pending.enable = false;
			mtk_ddp_comp_layer_config(ddp_first_comp, i, plane_state,
						  cmdq_handle);
		}
	}

	cmdq_sec_insert_backup_cookie(cmdq_handle);
	cmdq_pkt_finalize(cmdq_handle);
	dma_sync_single_for_device(mtk_crtc->sec_cmdq_client.chan->mbox->dev,
				   cmdq_handle->pa_base,
				   cmdq_handle->cmd_buf_size,
				   DMA_TO_DEVICE);

	mtk_crtc->sec_cmdq_working = true;
	mbox_send_message(mtk_crtc->sec_cmdq_client.chan, cmdq_handle);
	mbox_client_txdone(mtk_crtc->sec_cmdq_client.chan, 0);

	/* Wait for sec state to be disabled by cmdq */
	wait_event_timeout(mtk_crtc->sec_cb_blocking_queue,
			   !mtk_crtc->sec_cmdq_working,
			   msecs_to_jiffies(500));

	mutex_lock(&mtk_crtc->hw_lock);
	mtk_crtc->sec_on = false;
	dev_dbg(crtc->dev->dev, "crtc-%d disable secure plane!\n", drm_crtc_index(crtc));
	mutex_unlock(&mtk_crtc->hw_lock);
#endif
}

static void mtk_crtc_plane_switch_sec_state(struct drm_crtc *crtc,
					    struct drm_atomic_state *state)
{
#if IS_REACHABLE(CONFIG_MTK_CMDQ)
	bool sec_on = false, cursor_update = false;
	int i, plane_num = 0;
	struct drm_crtc_state *new_crtc_state = drm_atomic_get_new_crtc_state(state, crtc);
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct drm_plane *plane;
	struct drm_plane_state *new_plane_state;

	if (!mtk_crtc->sec_cmdq_client.chan){
		dev_err(crtc->dev->dev, "%s %d: crtc-%d, no sec_cmdq_client!\n",
			__func__, __LINE__, drm_crtc_index(crtc));
		return;
	}

	/* check updating plane state */
	for_each_new_plane_in_state(state, plane, new_plane_state, i) {
		if (!plane || !plane->state || !plane->state->crtc ||
		    !new_plane_state || !new_plane_state->crtc)
			continue;

		if(plane->type == DRM_PLANE_TYPE_CURSOR)
			cursor_update = true;

		if (new_plane_state->fb && mtk_plane_fb_is_secure(new_plane_state->fb))
			sec_on = true;

		plane_num++;
	}

	DRM_DEV_DEBUG_DRIVER(crtc->dev->dev, "plane_num=%d, sec_on=%d, cursor_only=%d, crtc-%d\n",
			     plane_num, sec_on, (plane_num == 1 && cursor_update),
			     drm_crtc_index(crtc));

	/* If no plane changed, not switching secure state */
	if (plane_num == 0)
		return;

	/* If only the cursor is updated, not switching secure state */
	if (plane_num == 1 && cursor_update)
		return;

	if (!sec_on) {
		mtk_crtc_disable_secure_state(crtc);
		return;
	}

	/* If crtc is going to be disabled, not switching to secure state */
	if (new_crtc_state && !new_crtc_state->active)
		return;

	mutex_lock(&mtk_crtc->hw_lock);
	mtk_crtc->sec_on = true;
	mutex_unlock(&mtk_crtc->hw_lock);
#endif
}

static void mtk_crtc_destroy(struct drm_crtc *crtc)
{
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	int i;

	priv = priv->all_drm_private[drm_crtc_index(crtc)];

	for (i = 0; i < MAX_MMSYS; i++)
		if (mtk_crtc->mutex[i])
			mtk_mutex_put(mtk_crtc->mutex[i]);

#if IS_REACHABLE(CONFIG_MTK_CMDQ)
	mtk_drm_cmdq_pkt_destroy(&mtk_crtc->cmdq_handle);

	if (mtk_crtc->cmdq_client.chan) {
		mbox_free_channel(mtk_crtc->cmdq_client.chan);
		mtk_crtc->cmdq_client.chan = NULL;
	}

	if (mtk_crtc->sec_cmdq_client.chan) {
		device_link_remove(priv->dev, mtk_crtc->sec_cmdq_client.chan->mbox->dev);
		mbox_free_channel(mtk_crtc->sec_cmdq_client.chan);
		mtk_crtc->sec_cmdq_client.chan = NULL;
	}
#endif

	for (i = 0; i < mtk_crtc->ddp_comp_nr; i++) {
		struct mtk_ddp_comp *comp;

		comp = mtk_crtc->ddp_comp[i];
		mtk_ddp_comp_unregister_vblank_cb(comp);
	}

	drm_crtc_cleanup(crtc);
}

static void mtk_crtc_reset(struct drm_crtc *crtc)
{
	struct mtk_crtc_state *state;

	if (crtc->state)
		__drm_atomic_helper_crtc_destroy_state(crtc->state);

	kfree(to_mtk_crtc_state(crtc->state));
	crtc->state = NULL;

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (state)
		__drm_atomic_helper_crtc_reset(crtc, &state->base);
}

static struct drm_crtc_state *mtk_crtc_duplicate_state(struct drm_crtc *crtc)
{
	struct mtk_crtc_state *state;
	int i;

	state = kmalloc(sizeof(*state), GFP_KERNEL);
	if (!state)
		return NULL;

	__drm_atomic_helper_crtc_duplicate_state(crtc, &state->base);

	WARN_ON(state->base.crtc != crtc);
	state->base.crtc = crtc;
	state->pending_config = false;
	state->pending_hrt_bw = NO_PENDING_HRT;
	for (i = 0; i < BW_CHANNEL_NR; i++)
		state->pending_channel_bw[i] = NO_PENDING_HRT;

	return &state->base;
}

static void mtk_crtc_destroy_state(struct drm_crtc *crtc,
				   struct drm_crtc_state *state)
{
	__drm_atomic_helper_crtc_destroy_state(state);
	kfree(to_mtk_crtc_state(state));
}

static enum drm_mode_status
mtk_crtc_mode_valid(struct drm_crtc *crtc, const struct drm_display_mode *mode)
{
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	enum drm_mode_status status = MODE_OK;
	int i;

	for (i = 0; i < mtk_crtc->ddp_comp_nr; i++) {
		status = mtk_ddp_comp_mode_valid(mtk_crtc->ddp_comp[i], mode);
		if (status != MODE_OK)
			break;
	}
	return status;
}

static bool mtk_crtc_mode_fixup(struct drm_crtc *crtc,
				const struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	/* Nothing to do here, but this callback is mandatory. */
	return true;
}

static void mtk_crtc_mode_set_nofb(struct drm_crtc *crtc)
{
	struct mtk_crtc_state *state = to_mtk_crtc_state(crtc->state);

	state->pending_width = crtc->mode.hdisplay;
	state->pending_height = crtc->mode.vdisplay;
	state->pending_vrefresh = drm_mode_vrefresh(&crtc->mode);
	wmb();	/* Make sure the above parameters are set before update */
	state->pending_config = true;
}

static int mtk_crtc_ddp_clk_enable(struct mtk_crtc *mtk_crtc)
{
	int ret;
	int i;

	for (i = 0; i < mtk_crtc->ddp_comp_nr; i++) {
		enum mtk_drm_mmsys mmsys;

		ret = mtk_ddp_comp_clk_enable(mtk_crtc->ddp_comp[i]);
		if (mtk_ddp_comp_get_type(mtk_crtc->ddp_comp[i]->id) == MTK_DISP_VIRTUAL) {
			mmsys = mtk_crtc->ddp_comp_sys[i];
			ret = mtk_mmsys_ddp_clk_enable(mtk_crtc->mmsys_dev[mmsys],
						       mtk_crtc->ddp_comp[i]->id);
		}
		if (ret) {
			DRM_ERROR("Failed to enable clock %d: %d\n", i, ret);
			goto err;
		}
	}

	return 0;
err:
	while (--i >= 0) {
		mtk_ddp_comp_clk_disable(mtk_crtc->ddp_comp[i]);
		if (mtk_ddp_comp_get_type(mtk_crtc->ddp_comp[i]->id) == MTK_DISP_VIRTUAL)
			mtk_mmsys_ddp_clk_disable(mtk_crtc->mmsys_dev[mtk_crtc->ddp_comp_sys[i]],
						  mtk_crtc->ddp_comp[i]->id);
	}
	return ret;
}

static void mtk_crtc_ddp_clk_disable(struct mtk_crtc *mtk_crtc)
{
	int i;
	enum mtk_drm_mmsys mmsys;

	for (i = 0; i < mtk_crtc->ddp_comp_nr; i++) {
		mtk_ddp_comp_clk_disable(mtk_crtc->ddp_comp[i]);
		if (mtk_ddp_comp_get_type(mtk_crtc->ddp_comp[i]->id) == MTK_DISP_VIRTUAL) {
			mmsys = mtk_crtc->ddp_comp_sys[i];
			mtk_mmsys_ddp_clk_disable(mtk_crtc->mmsys_dev[mmsys],
						  mtk_crtc->ddp_comp[i]->id);
		}
	}
}

static
struct mtk_ddp_comp *mtk_ddp_comp_for_plane(struct drm_crtc *crtc,
					    struct drm_plane *plane,
					    unsigned int *local_layer)
{
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp;
	int i, count = 0;
	unsigned int local_index = plane - mtk_crtc->planes;

	for (i = 0; i < mtk_crtc->ddp_comp_nr; i++) {
		comp = mtk_crtc->ddp_comp[i];
		if (local_index < (count + mtk_ddp_comp_layer_nr(comp))) {
			*local_layer = local_index - count;
			return comp;
		}
		count += mtk_ddp_comp_layer_nr(comp);
	}

	WARN(1, "Failed to find component for plane %d\n", plane->index);
	return NULL;
}

static unsigned int mtk_crtc_hrt_frame_bw(struct drm_crtc *crtc)
{
	unsigned int bw = 0;
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *output_comp = mtk_crtc->ddp_comp[mtk_crtc->ddp_comp_nr - 1];

	mtk_ddp_comp_hrt_bw_get(output_comp, &bw);
	return bw;
}

static void mtk_crtc_pre_update_hrt_state(struct drm_crtc *crtc)
{
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	unsigned int crtc_id = drm_crtc_index(crtc);
	struct mtk_crtc_state *state;
	unsigned int bw = 0;
	unsigned int count = 0;
	int i, j;

	state = to_mtk_crtc_state(mtk_crtc->base.state);
	bw = mtk_crtc_hrt_frame_bw(crtc);

	for (i = 0; i < mtk_crtc->layer_nr; i++) {
		struct drm_plane *plane = &mtk_crtc->planes[i];
		struct mtk_plane_state *plane_state;

		plane_state = to_mtk_plane_state(plane->state);
		if (plane_state->pending.enable) {
			const struct drm_format_info *info =
					drm_format_info(plane_state->pending.format);
			int bpp;

			count++;
			bpp = drm_format_info_bpp(info, 0);
			mtk_crtc->qos_ctx->plane_hrt_req[i] = bw * bpp / 8 / 4;
		} else {
			mtk_crtc->qos_ctx->plane_hrt_req[i] = 0;
		}
	}

	bw = bw * count;
	if (bw > mtk_crtc->qos_ctx->last_hrt_req) {
		mtk_disp_pmqos_set_hrt_bw(mtk_crtc->pmqos_dev, crtc_id, bw);
		mtk_crtc->qos_ctx->last_hrt_req = bw;
	} else if (bw < mtk_crtc->qos_ctx->last_hrt_req) {
		state->pending_hrt_bw = bw;
	}

	for (i = 0; i < BW_CHANNEL_NR; i++) {
		unsigned int channel_bw = 0;

		for (j = 0; j < mtk_crtc->layer_nr; j++)
			if (mtk_crtc->qos_ctx->plane_channel_id[j] == i)
				channel_bw += mtk_crtc->qos_ctx->plane_hrt_req[j];

		if (channel_bw > mtk_crtc->qos_ctx->last_channel_hrt_req[i]) {
			mtk_disp_pmqos_set_channel_hrt_bw(mtk_crtc->pmqos_dev, crtc_id,
							  channel_bw, i);
			mtk_crtc->qos_ctx->last_channel_hrt_req[i] = channel_bw;
		} else if (channel_bw < mtk_crtc->qos_ctx->last_channel_hrt_req[i]) {
			state->pending_channel_bw[i] = channel_bw;
		}
	}

	for (i = 0; i < mtk_crtc->layer_nr; i++) {
		struct drm_plane *plane = &mtk_crtc->planes[i];
		struct mtk_plane_state *plane_state = to_mtk_plane_state(plane->state);
		struct mtk_ddp_comp *comp = mtk_crtc->ddp_comp[0];
		unsigned int plane_hrt_bw = mtk_crtc->qos_ctx->plane_hrt_req[i];

		if (plane_hrt_bw > mtk_crtc->qos_ctx->last_plane_hrt_req[i]) {
			mtk_ddp_comp_hrt_bw_set(comp, i, plane_hrt_bw);
			mtk_crtc->qos_ctx->last_plane_hrt_req[i] = plane_hrt_bw;
		} else if (plane_hrt_bw < mtk_crtc->qos_ctx->last_plane_hrt_req[i]) {
			plane_state->pending.hrt_bw = plane_hrt_bw;
		}
	}
}

static void mtk_crtc_post_update_hrt_state(struct mtk_crtc *mtk_crtc)
{
	struct mtk_crtc_state *state = to_mtk_crtc_state(mtk_crtc->base.state);
	int i, crtc_id = drm_crtc_index(&mtk_crtc->base);

	for (i = 0; i < mtk_crtc->layer_nr; i++) {
		struct drm_plane *plane = &mtk_crtc->planes[i];
		struct mtk_plane_state *plane_state = to_mtk_plane_state(plane->state);
		struct mtk_ddp_comp *comp = mtk_crtc->ddp_comp[0];

		if (plane_state->pending.hrt_bw == NO_PENDING_HRT)
			continue;

		mtk_ddp_comp_hrt_bw_set(comp, i, plane_state->pending.hrt_bw);
		mtk_crtc->qos_ctx->last_plane_hrt_req[i] = plane_state->pending.hrt_bw;
		plane_state->pending.hrt_bw = NO_PENDING_HRT;
	}

	for (i = 0; i < BW_CHANNEL_NR; i++) {
		if (state->pending_channel_bw[i] == NO_PENDING_HRT)
			continue;

		mtk_disp_pmqos_set_channel_hrt_bw(mtk_crtc->pmqos_dev, crtc_id,
						  state->pending_channel_bw[i], i);
		mtk_crtc->qos_ctx->last_channel_hrt_req[i] = state->pending_channel_bw[i];
		state->pending_channel_bw[i] = NO_PENDING_HRT;
	}

	if (state->pending_hrt_bw == NO_PENDING_HRT)
		return;

	mtk_disp_pmqos_set_hrt_bw(mtk_crtc->pmqos_dev, drm_crtc_index(&mtk_crtc->base),
				  state->pending_hrt_bw);
	mtk_crtc->qos_ctx->last_hrt_req = state->pending_hrt_bw;
	state->pending_hrt_bw = NO_PENDING_HRT;
}

static void mtk_crtc_update_srt_state(struct drm_crtc *crtc)
{
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_crtc_state *state;
	unsigned int total_srt = 0;
	unsigned int vdisplay, vtotal, vrefresh;
	int i, j;

	state = to_mtk_crtc_state(mtk_crtc->base.state);

	vdisplay = crtc->state->adjusted_mode.vdisplay;
	vtotal = crtc->state->adjusted_mode.vtotal;
	vrefresh = drm_mode_vrefresh(&crtc->state->adjusted_mode);

	for (i = 0; i < mtk_crtc->layer_nr; i++) {
		struct drm_plane *plane = &mtk_crtc->planes[i];
		struct mtk_plane_state *plane_state;
		struct mtk_ddp_comp *comp = mtk_crtc->ddp_comp[0];

		plane_state = to_mtk_plane_state(plane->state);
		if (plane_state->pending.enable) {
			const struct drm_format_info *info =
				drm_format_info(plane_state->pending.format);
			int bpp;
			uint64_t bw;

			/* SRT BW = w * h * bpp * vrefresh * blanking_ratio */
			bpp = drm_format_info_bpp(info, 0);
			bw = plane_state->pending.width * plane_state->pending.height * bpp / 8;
			do_div(bw, 1000);
			bw *= vtotal / vdisplay * vrefresh;
			do_div(bw, 1000);
			mtk_crtc->qos_ctx->plane_srt_req[i] = bw;
		} else {
			mtk_crtc->qos_ctx->plane_srt_req[i] = 0;
		}

		if (mtk_crtc->qos_ctx->plane_srt_req[i] != mtk_crtc->qos_ctx->last_plane_srt_req[i])
			mtk_ddp_comp_srt_bw_set(comp, i, mtk_crtc->qos_ctx->plane_srt_req[i]);

		mtk_crtc->qos_ctx->last_plane_srt_req[i] = mtk_crtc->qos_ctx->plane_srt_req[i];
		total_srt += mtk_crtc->qos_ctx->plane_srt_req[i];
	}

	mtk_disp_pmqos_clear_channel_srt_bw(mtk_crtc->pmqos_dev, drm_crtc_index(crtc));
	mtk_disp_pmqos_set_srt_bw(mtk_crtc->pmqos_dev, drm_crtc_index(crtc), total_srt);
	for (i = 0; i < BW_CHANNEL_NR; i++) {
		unsigned int channel_bw = 0;

		for (j = 0; j < mtk_crtc->layer_nr; j++)
			if (mtk_crtc->qos_ctx->plane_channel_id[j] == i)
				channel_bw += mtk_crtc->qos_ctx->plane_srt_req[j];

		mtk_disp_pmqos_set_channel_srt_bw(mtk_crtc->pmqos_dev, drm_crtc_index(crtc),
						  channel_bw, i);
	}
}

#if IS_REACHABLE(CONFIG_MTK_CMDQ)
static int ddp_cmdq_done_kthread(void *data)
{
	struct mtk_crtc *mtk_crtc = (struct mtk_crtc *)data;

	while (!kthread_should_stop()) {
		wait_event_interruptible(mtk_crtc->cb_blocking_queue,
					 atomic_read(&mtk_crtc->cmdq_done));
		atomic_set(&mtk_crtc->cmdq_done, 0);

		mutex_lock(&mtk_crtc->hw_lock);
		mtk_crtc_post_update_hrt_state(mtk_crtc);
		mutex_unlock(&mtk_crtc->hw_lock);
	}
	return 0;
}

static void ddp_cmdq_cb(struct mbox_client *cl, void *mssg)
{
	struct cmdq_cb_data *data = mssg;
	struct cmdq_client *cmdq_cl = container_of(cl, struct cmdq_client, client);
	struct mtk_crtc *mtk_crtc;
	struct mtk_crtc_state *state;
	unsigned int i;
	unsigned long flags;
	bool is_secure = (data->pkt && data->pkt->sec_data);

	if (data->sta < 0) {
		if (is_secure) {
			mtk_drm_cmdq_pkt_destroy(data->pkt);
			kfree(data->pkt);
		}
		return;
	}

	if (is_secure)
		mtk_crtc = container_of(cmdq_cl, struct mtk_crtc, sec_cmdq_client);
	else
		mtk_crtc = container_of(cmdq_cl, struct mtk_crtc, cmdq_client);

	state = to_mtk_crtc_state(mtk_crtc->base.state);

	spin_lock_irqsave(&mtk_crtc->config_lock, flags);
	if (mtk_crtc->config_updating)
		goto ddp_cmdq_cb_out;

	state->pending_config = false;

	if (mtk_crtc->pending_planes) {
		for (i = 0; i < mtk_crtc->layer_nr; i++) {
			struct drm_plane *plane = &mtk_crtc->planes[i];
			struct mtk_plane_state *plane_state;

			plane_state = to_mtk_plane_state(plane->state);

			plane_state->pending.config = false;
		}
		mtk_crtc->pending_planes = false;
	}

	if (mtk_crtc->pending_async_planes) {
		for (i = 0; i < mtk_crtc->layer_nr; i++) {
			struct drm_plane *plane = &mtk_crtc->planes[i];
			struct mtk_plane_state *plane_state;

			plane_state = to_mtk_plane_state(plane->state);

			plane_state->pending.async_config = false;
		}
		mtk_crtc->pending_async_planes = false;
	}

ddp_cmdq_cb_out:

	if (mtk_crtc->pending_needs_vblank) {
		mtk_crtc_finish_page_flip(mtk_crtc);
		mtk_crtc->pending_needs_vblank = false;
	}

	spin_unlock_irqrestore(&mtk_crtc->config_lock, flags);

	if (is_secure) {
		mtk_drm_cmdq_pkt_destroy(data->pkt);
		kfree(data->pkt);
	}

	mtk_crtc->cmdq_vblank_cnt = 0;
	wake_up(&mtk_crtc->cb_blocking_queue);

	/*
	 * 1.Make sure sec_cb_blocking queue is waked later than cb_blocking_queue.
	 * 2.The new mtk_crtc_update_config() will do the pre update HRT earlier than
	 * the ddp_cmdq_cb() called from mtk_crtc_disable_secure_state(), that may
	 * cause the underflow issue if mtk_crtc_post_update_hrt_state() scales down
	 * the HRT for the non-disabled layer in mtk_crtc_disable_secure_state().
	 * So skip triggering the post update HRT to avoid this timing issue.
	 */
	if (mtk_crtc->sec_cmdq_working) {
		mtk_crtc->sec_cmdq_working = false;
		wake_up(&mtk_crtc->sec_cb_blocking_queue);
		return;
	}

	/* For post update HRT */
	atomic_set(&mtk_crtc->cmdq_done, 1);
	wake_up_interruptible(&mtk_crtc->cb_blocking_queue);
}
#endif

static int mtk_crtc_ddp_hw_init(struct mtk_crtc *mtk_crtc)
{
	struct drm_crtc *crtc = &mtk_crtc->base;
	struct drm_connector *connector;
	struct drm_encoder *encoder;
	struct drm_connector_list_iter conn_iter;
	unsigned int width, height, vrefresh, bpc = MTK_DEFAULT_MAX_BPC;
	int ret;
	int i, j;
	enum mtk_drm_mmsys mmsys;
	struct mtk_ddp_comp *output_comp;

	if (WARN_ON(!crtc->state))
		return -EINVAL;

	width = crtc->state->adjusted_mode.hdisplay;
	height = crtc->state->adjusted_mode.vdisplay;
	vrefresh = drm_mode_vrefresh(&crtc->state->adjusted_mode);

	drm_for_each_encoder(encoder, crtc->dev) {
		if (encoder->crtc != crtc)
			continue;

		drm_connector_list_iter_begin(crtc->dev, &conn_iter);
		drm_for_each_connector_iter(connector, &conn_iter) {
			if (connector->encoder != encoder)
				continue;
			if (connector->display_info.bpc != 0 &&
			    bpc > connector->display_info.bpc)
				bpc = connector->display_info.bpc;
		}
		drm_connector_list_iter_end(&conn_iter);
	}

	ret = pm_runtime_resume_and_get(crtc->dev->dev);
	if (ret < 0) {
		DRM_ERROR("Failed to enable power domain: %d\n", ret);
		return ret;
	}

	mtk_disp_pmqos_set_mmclk_by_pixclk(mtk_crtc->pmqos_dev, drm_crtc_index(crtc),
					   crtc->state->adjusted_mode.crtc_clock / 1000, __func__);
	DRM_DEBUG_DRIVER("crtc%d (%dx%d-%dx%d) mtk_disp_pmqos_set_mmclk_by_pixclk: %d Mpixels/s\n",
			 drm_crtc_index(crtc), crtc->state->adjusted_mode.crtc_hdisplay,
			 crtc->state->adjusted_mode.crtc_vdisplay,
			 crtc->state->adjusted_mode.crtc_htotal,
			 crtc->state->adjusted_mode.crtc_vtotal,
			 crtc->state->adjusted_mode.crtc_clock / 1000);

	for (i = 0; i < MAX_MMSYS; i++)
		if (mtk_crtc->exist[i])
			mtk_mmsys_top_clk_enable(mtk_crtc->mmsys_dev[i]);

	for (i = 0; i < MAX_MMSYS; i++) {
		if (!mtk_crtc->mutex[i] || !mtk_crtc->exist[i])
			continue;
		ret = mtk_mutex_prepare(mtk_crtc->mutex[i]);
		if (ret < 0) {
			DRM_ERROR("Failed to enable mmsys%d mutex clock: %d\n", i, ret);
			goto err_pm_runtime_put;
		}
	}

	ret = mtk_crtc_ddp_clk_enable(mtk_crtc);
	if (ret < 0) {
		DRM_ERROR("Failed to enable component clocks: %d\n", ret);
		goto err_mutex_unprepare;
	}

	if (mtk_crtc->vdisp_ao_dev)
		mtk_mmsys_default_config(mtk_crtc->vdisp_ao_dev);

	for (i = 0; i < MAX_MMSYS; i++)
		if (mtk_crtc->exist[i])
			mtk_mmsys_default_config(mtk_crtc->mmsys_dev[i]);

	output_comp = mtk_crtc->ddp_comp[mtk_crtc->ddp_comp_nr - 1];
	if (output_comp) {
		mmsys = mtk_crtc->ddp_comp_sys[0];
		mtk_ddp_comp_fifo_sel(mtk_crtc->ddp_comp[0], mtk_crtc->mmsys_dev[mmsys],
				      output_comp->id);
	}

	for (i = 0; i < mtk_crtc->ddp_comp_nr - 1; i++) {
		mmsys = mtk_crtc->ddp_comp_sys[i];
		if (!mtk_ddp_comp_connect(mtk_crtc->ddp_comp[i], mtk_crtc->mmsys_dev[mmsys],
					  mtk_crtc->ddp_comp[i + 1]->id))
			mtk_mmsys_ddp_connect(mtk_crtc->mmsys_dev[mmsys],
					      mtk_crtc->ddp_comp[i]->id,
					      mtk_crtc->ddp_comp[i + 1]->id);
		if (!mtk_ddp_comp_add(mtk_crtc->ddp_comp[i], mtk_crtc->mutex[mmsys]))
			mtk_mutex_add_comp(mtk_crtc->mutex[mmsys],
					   mtk_crtc->ddp_comp[i]->id);
	}
	mmsys = mtk_crtc->ddp_comp_sys[i];
	if (!mtk_ddp_comp_add(mtk_crtc->ddp_comp[i], mtk_crtc->mutex[mmsys]))
		mtk_mutex_add_comp(mtk_crtc->mutex[mmsys], mtk_crtc->ddp_comp[i]->id);

	/* Need to set sof source for all mmsys mutexes in this crtc */
	for (j = 0; j < MAX_MMSYS; j++)
		if (mtk_crtc->exist[j] && mtk_crtc->mutex[j])
			mtk_mutex_write_comp_sof(mtk_crtc->mutex[j], mtk_crtc->ddp_comp[i]->id);

	for (i = 0; i < MAX_MMSYS; i++)
		if (mtk_crtc->exist[i] && mtk_crtc->mutex[i])
			mtk_mutex_enable(mtk_crtc->mutex[i]);

	for (i = 0; i < mtk_crtc->ddp_comp_nr; i++) {
		struct mtk_ddp_comp *comp = mtk_crtc->ddp_comp[i];

		if (i == 1)
			mtk_ddp_comp_bgclr_in_on(comp);

		if (mtk_ddp_comp_get_type(comp->id) == MTK_DISP_VIRTUAL)
			mtk_mmsys_ddp_config(mtk_crtc->mmsys_dev[mtk_crtc->ddp_comp_sys[i]],
					     comp->id, width, height, NULL);
		else
			mtk_ddp_comp_config(comp, width, height, vrefresh, bpc, NULL);
		mtk_ddp_comp_start(comp);
	}

	/* Initially configure all planes */
	for (i = 0; i < mtk_crtc->layer_nr; i++) {
		struct drm_plane *plane = &mtk_crtc->planes[i];
		struct mtk_plane_state *plane_state;
		struct mtk_ddp_comp *comp;
		unsigned int local_layer;

		plane_state = to_mtk_plane_state(plane->state);

		/* should not enable layer before crtc enabled */
		plane_state->pending.enable = false;
		comp = mtk_ddp_comp_for_plane(crtc, plane, &local_layer);
		if (comp)
			mtk_ddp_comp_layer_config(comp, local_layer,
						  plane_state, NULL);
	}

	return 0;

err_mutex_unprepare:
	while(--i >= 0)
		if (mtk_crtc->exist[i] && mtk_crtc->mutex[i])
			mtk_mutex_unprepare(mtk_crtc->mutex[i]);

err_pm_runtime_put:
	pm_runtime_put(crtc->dev->dev);
	return ret;
}

static void mtk_crtc_ddp_hw_fini(struct mtk_crtc *mtk_crtc)
{
	struct drm_device *drm = mtk_crtc->base.dev;
	struct drm_crtc *crtc = &mtk_crtc->base;
	unsigned long flags;
	int i;
	enum mtk_drm_mmsys mmsys;

	for (i = 0; i < mtk_crtc->ddp_comp_nr; i++) {
		mtk_ddp_comp_stop(mtk_crtc->ddp_comp[i]);
		if (i == 1)
			mtk_ddp_comp_bgclr_in_off(mtk_crtc->ddp_comp[i]);
	}

	for (i = 0; i < mtk_crtc->ddp_comp_nr; i++) {
		mmsys = mtk_crtc->ddp_comp_sys[i];
		if (!mtk_ddp_comp_remove(mtk_crtc->ddp_comp[i], mtk_crtc->mutex[mmsys]))
			mtk_mutex_remove_comp(mtk_crtc->mutex[mtk_crtc->ddp_comp_sys[i]],
					      mtk_crtc->ddp_comp[i]->id);
	}
	for (i = 0; i < MAX_MMSYS; i++)
		if (mtk_crtc->exist[i] && mtk_crtc->mutex[i])
			mtk_mutex_disable(mtk_crtc->mutex[i]);

	for (i = 0; i < mtk_crtc->ddp_comp_nr - 1; i++) {
		struct mtk_ddp_comp *comp;
		unsigned int curr, next;

		comp = mtk_crtc->ddp_comp[i];
		curr = mtk_crtc->ddp_comp[i]->id;
		next = mtk_crtc->ddp_comp[i + 1]->id;
		mmsys = mtk_crtc->ddp_comp_sys[i];
		if (!mtk_ddp_comp_disconnect(comp, mtk_crtc->mmsys_dev[mmsys], next))
			mtk_mmsys_ddp_disconnect(mtk_crtc->mmsys_dev[mmsys], curr, next);
		if (!mtk_ddp_comp_remove(comp, mtk_crtc->mutex[mmsys]))
			mtk_mutex_remove_comp(mtk_crtc->mutex[mtk_crtc->ddp_comp_sys[i]],
					      mtk_crtc->ddp_comp[i]->id);
	}

	mmsys = mtk_crtc->ddp_comp_sys[i];
	if (!mtk_ddp_comp_remove(mtk_crtc->ddp_comp[i], mtk_crtc->mutex[mmsys]))
		mtk_mutex_remove_comp(mtk_crtc->mutex[mmsys], mtk_crtc->ddp_comp[i]->id);

	mtk_crtc_ddp_clk_disable(mtk_crtc);

	for (i = 0; i < MAX_MMSYS; i++)
		if (mtk_crtc->exist[i] && mtk_crtc->mutex[i])
			mtk_mutex_unprepare(mtk_crtc->mutex[i]);

	for (i = 0; i < MAX_MMSYS; i++)
		if (mtk_crtc->exist[i])
			mtk_mmsys_top_clk_disable(mtk_crtc->mmsys_dev[i]);

	mtk_disp_pmqos_set_mmclk_by_pixclk(mtk_crtc->pmqos_dev, drm_crtc_index(crtc), 0, __func__);
	pm_runtime_put_sync(drm->dev);

	if (crtc->state->event && !crtc->state->active) {
		spin_lock_irqsave(&crtc->dev->event_lock, flags);
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
		spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
	}
}

static void mtk_crtc_ddp_config(struct drm_crtc *crtc,
				struct cmdq_pkt *cmdq_handle)
{
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_crtc_state *state = to_mtk_crtc_state(mtk_crtc->base.state);
	struct mtk_ddp_comp *comp = mtk_crtc->ddp_comp[0];
	unsigned int i;
	unsigned int local_layer;

	/*
	 * TODO: instead of updating the registers here, we should prepare
	 * working registers in atomic_commit and let the hardware command
	 * queue update module registers on vblank.
	 */
	if (state->pending_config) {
		mtk_ddp_comp_config(comp, state->pending_width,
				    state->pending_height,
				    state->pending_vrefresh, 0,
				    cmdq_handle);

		if (!cmdq_handle)
			state->pending_config = false;
	}

	if (mtk_crtc->pending_planes) {
		for (i = 0; i < mtk_crtc->layer_nr; i++) {
			struct drm_plane *plane = &mtk_crtc->planes[i];
			struct mtk_plane_state *plane_state;

			plane_state = to_mtk_plane_state(plane->state);

			if (!plane_state->pending.config)
				continue;

			comp = mtk_ddp_comp_for_plane(crtc, plane, &local_layer);

			if (comp)
				mtk_ddp_comp_layer_config(comp, local_layer,
							  plane_state,
							  cmdq_handle);
			if (!cmdq_handle)
				plane_state->pending.config = false;
		}

		if (!cmdq_handle)
			mtk_crtc->pending_planes = false;
	}

	if (mtk_crtc->pending_async_planes) {
		for (i = 0; i < mtk_crtc->layer_nr; i++) {
			struct drm_plane *plane = &mtk_crtc->planes[i];
			struct mtk_plane_state *plane_state;

			plane_state = to_mtk_plane_state(plane->state);

			if (!plane_state->pending.async_config)
				continue;

			comp = mtk_ddp_comp_for_plane(crtc, plane, &local_layer);

			if (comp)
				mtk_ddp_comp_layer_config(comp, local_layer,
							  plane_state,
							  cmdq_handle);
			if (!cmdq_handle)
				plane_state->pending.async_config = false;
		}

		if (!cmdq_handle)
			mtk_crtc->pending_async_planes = false;
	}
}

static void mtk_crtc_update_config(struct mtk_crtc *mtk_crtc, bool needs_vblank)
{
#if IS_REACHABLE(CONFIG_MTK_CMDQ)
	struct cmdq_client cmdq_client;
	struct cmdq_pkt *cmdq_handle;
#endif
	struct drm_crtc *crtc = &mtk_crtc->base;
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	unsigned int pending_planes = 0, pending_async_planes = 0;
	int i;
	unsigned long flags;

	mutex_lock(&mtk_crtc->hw_lock);

	spin_lock_irqsave(&mtk_crtc->config_lock, flags);
	mtk_crtc->config_updating = true;
	spin_unlock_irqrestore(&mtk_crtc->config_lock, flags);

	mtk_crtc_pre_update_hrt_state(crtc);
	mtk_crtc_update_srt_state(crtc);
	if (needs_vblank)
		mtk_crtc->pending_needs_vblank = true;

	for (i = 0; i < mtk_crtc->layer_nr; i++) {
		struct drm_plane *plane = &mtk_crtc->planes[i];
		struct mtk_plane_state *plane_state;

		plane_state = to_mtk_plane_state(plane->state);
		if (plane_state->pending.dirty) {
			plane_state->pending.config = true;
			plane_state->pending.dirty = false;
			pending_planes |= BIT(i);
		} else if (plane_state->pending.async_dirty) {
			plane_state->pending.async_config = true;
			plane_state->pending.async_dirty = false;
			pending_async_planes |= BIT(i);
		}
	}
	if (pending_planes)
		mtk_crtc->pending_planes = true;
	if (pending_async_planes)
		mtk_crtc->pending_async_planes = true;

	if (priv->data->shadow_register) {
		for (i = 0; i < MAX_MMSYS; i++)
			if (mtk_crtc->exist[i] && mtk_crtc->mutex[i])
				mtk_mutex_acquire(mtk_crtc->mutex[i]);

		mtk_crtc_ddp_config(crtc, NULL);

		for (i = 0; i < MAX_MMSYS; i++)
			if (mtk_crtc->exist[i] && mtk_crtc->mutex[i])
				mtk_mutex_release(mtk_crtc->mutex[i]);
	}
#if IS_REACHABLE(CONFIG_MTK_CMDQ)
	if (mtk_crtc->sec_on) {
		cmdq_handle = kzalloc(sizeof(*cmdq_handle), GFP_KERNEL);
		if (!cmdq_handle) {
			DRM_ERROR("mtk_crtc %d failed to kzalloc secure cmdq packet\n",
				  drm_crtc_index(&mtk_crtc->base));
			goto update_config_err;
		}
		if (mtk_drm_cmdq_pkt_create(&mtk_crtc->sec_cmdq_client,
					    cmdq_handle, PAGE_SIZE) < 0) {
			DRM_ERROR("mtk_crtc %d failed to create secure cmdq packet\n",
				  drm_crtc_index(&mtk_crtc->base));
			goto update_config_err;
		}
		if (cmdq_sec_pkt_alloc_sec_data(cmdq_handle) < 0) {
			DRM_ERROR("mtk_crtc %d failed to create secure cmdq packet data\n",
				  drm_crtc_index(&mtk_crtc->base));
			goto update_config_err;
		}

		cmdq_client = mtk_crtc->sec_cmdq_client;
	} else if (mtk_crtc->cmdq_client.chan) {
		mbox_flush(mtk_crtc->cmdq_client.chan, 2000);
		mtk_crtc->cmdq_handle.cmd_buf_size = 0;

		cmdq_client =  mtk_crtc->cmdq_client;
		cmdq_handle = &mtk_crtc->cmdq_handle;
	} else {
		cmdq_client.chan = NULL;
		cmdq_handle = NULL;
	}

	if (cmdq_client.chan && cmdq_handle) {
		cmdq_pkt_clear_event(cmdq_handle, mtk_crtc->cmdq_event);
		cmdq_pkt_wfe(cmdq_handle, mtk_crtc->cmdq_event, false);
		mtk_crtc_ddp_config(crtc, cmdq_handle);
		if (mtk_crtc->sec_on)
			cmdq_sec_insert_backup_cookie(cmdq_handle);
		cmdq_pkt_finalize(cmdq_handle);
		dma_sync_single_for_device(cmdq_client.chan->mbox->dev,
					   cmdq_handle->pa_base,
					   cmdq_handle->cmd_buf_size,
					   DMA_TO_DEVICE);
		/*
		 * CMDQ command should execute in next 3 vblank.
		 * One vblank interrupt before send message (occasionally)
		 * and one vblank interrupt after cmdq done,
		 * so it's timeout after 3 vblank interrupt.
		 * If it fail to execute in next 3 vblank, timeout happen.
		 */
		mtk_crtc->cmdq_vblank_cnt = 3;

		spin_lock_irqsave(&mtk_crtc->config_lock, flags);
		mtk_crtc->config_updating = false;
		spin_unlock_irqrestore(&mtk_crtc->config_lock, flags);

		mbox_send_message(cmdq_client.chan, cmdq_handle);
		mbox_client_txdone(cmdq_client.chan, 0);
		goto update_config_out;
	}

update_config_err:
#endif
	spin_lock_irqsave(&mtk_crtc->config_lock, flags);
	mtk_crtc->config_updating = false;
	spin_unlock_irqrestore(&mtk_crtc->config_lock, flags);

#if IS_REACHABLE(CONFIG_MTK_CMDQ)
update_config_out:
#endif

	mutex_unlock(&mtk_crtc->hw_lock);
}

static void mtk_crtc_ddp_irq(void *data)
{
	struct drm_crtc *crtc = data;
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = crtc->dev->dev_private;

#if IS_REACHABLE(CONFIG_MTK_CMDQ)
	if (!priv->data->shadow_register && !mtk_crtc->cmdq_client.chan)
		mtk_crtc_ddp_config(crtc, NULL);
	else if (mtk_crtc->cmdq_vblank_cnt > 0 && --mtk_crtc->cmdq_vblank_cnt == 0)
		DRM_ERROR("mtk_crtc %d CMDQ execute command timeout!\n",
			  drm_crtc_index(&mtk_crtc->base));
#else
	if (!priv->data->shadow_register)
		mtk_crtc_ddp_config(crtc, NULL);
#endif
	mtk_drm_finish_page_flip(mtk_crtc);
}

static int mtk_crtc_enable_vblank(struct drm_crtc *crtc)
{
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp = mtk_crtc->ddp_comp[0];

	mtk_ddp_comp_enable_vblank(comp);

	return 0;
}

static void mtk_crtc_disable_vblank(struct drm_crtc *crtc)
{
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp = mtk_crtc->ddp_comp[0];

	mtk_ddp_comp_disable_vblank(comp);
}

static void mtk_crtc_update_output(struct drm_crtc *crtc,
				   struct drm_atomic_state *state)
{
	int crtc_index = drm_crtc_index(crtc);
	int i;
	unsigned int mmsys;
	struct device *dev;
	struct drm_crtc_state *crtc_state = state->crtcs[crtc_index].new_state;
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv;
	unsigned int encoder_mask = crtc_state->encoder_mask;

	if (!crtc_state->connectors_changed)
		return;

	if (!mtk_crtc->num_conn_routes)
		return;

	mmsys = mtk_crtc->conn_routes_sys;
	priv = ((struct mtk_drm_private *)crtc->dev->dev_private)->all_drm_private[mmsys];
	dev = priv->dev;

	dev_dbg(dev, "connector change:%d, encoder mask:0x%x for crtc:%d\n",
		crtc_state->connectors_changed, encoder_mask, crtc_index);

	for (i = 0; i < mtk_crtc->num_conn_routes; i++) {
		unsigned int comp_id = mtk_crtc->conn_routes[i].route_ddp;
		struct mtk_ddp_comp *comp = &priv->ddp_comp[comp_id];

		if (comp->encoder_index >= 0 &&
		    (encoder_mask & BIT(comp->encoder_index))) {
			mtk_crtc->ddp_comp[mtk_crtc->ddp_comp_nr - 1] = comp;
			mtk_crtc->ddp_comp_sys[mtk_crtc->ddp_comp_nr - 1] = mmsys;
			mtk_crtc->exist[mmsys] = true;
			dev_dbg(dev, "Add comp_id: %d at path index %d\n",
				comp->id, mtk_crtc->ddp_comp_nr - 1);
			break;
		}
	}
}

static void mtk_crtc_crc_work(struct kthread_work *base)
{
	struct drm_vblank_work *work = to_drm_vblank_work(base);
	struct mtk_crtc *mtk_crtc = container_of(work, typeof(*mtk_crtc), crc_work);
	struct mtk_ddp_comp *comp = mtk_crtc->crc_provider;
	u64 vblank = drm_crtc_vblank_count(&mtk_crtc->base);

	spin_lock_irq(&mtk_crtc->base.crc.lock);
	if (!mtk_crtc->base.crc.opened) {
		spin_unlock_irq(&mtk_crtc->base.crc.lock);
		return;
	}
	spin_unlock_irq(&mtk_crtc->base.crc.lock);

	comp->funcs->crc_read(comp->dev);

	/* could take more than 50ms to finish */
	drm_crtc_add_crc_entry(&mtk_crtc->base, true, vblank,
				comp->funcs->crc_entry(comp->dev));

	drm_vblank_work_schedule(&mtk_crtc->crc_work, vblank + 1, true);
}

static int mtk_crtc_set_crc_source(struct drm_crtc *crtc, const char *src)
{
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);

	if (!src)
		return -EINVAL;

	if (strcmp(src, "auto") != 0) {
		DRM_ERROR("%s(crtc-%d): undnown source '%s'\n",
			  __func__, drm_crtc_index(crtc), src);
		return -EINVAL;
	}

	/*
	 * skip the first crc because the first frame (vblank+1) is configured
	 * by mtk_crtc_ddp_hw_init() when atomic enable
	 */
	drm_vblank_work_schedule(&mtk_crtc->crc_work,
				 drm_crtc_vblank_count(crtc) + 2, false);
	return 0;
}

static int mtk_crtc_verify_crc_source(struct drm_crtc *crtc, const char *src,
				      size_t *cnt)
{
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp = mtk_crtc->crc_provider;

	if (!comp) {
		DRM_ERROR("%s(crtc-%d): no crc provider\n",
			  __func__, drm_crtc_index(crtc));
		return -ENOENT;
	}

	if (src && strcmp(src, "auto") != 0) {
		DRM_ERROR("%s(crtc-%d): unknown source '%s'\n",
			  __func__, drm_crtc_index(crtc), src);
		return -EINVAL;
	}

	*cnt = comp->funcs->crc_cnt(comp->dev);

	return 0;
}

int mtk_crtc_plane_check(struct drm_crtc *crtc, struct drm_plane *plane,
			 struct mtk_plane_state *state)
{
	unsigned int local_layer;
	struct mtk_ddp_comp *comp;

	comp = mtk_ddp_comp_for_plane(crtc, plane, &local_layer);
	if (comp)
		return mtk_ddp_comp_layer_check(comp, local_layer, state);
	return 0;
}

void mtk_crtc_plane_disable(struct drm_crtc *crtc, struct drm_plane *plane)
{
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_plane_state *plane_state = to_mtk_plane_state(plane->state);
	int i;

	if (!mtk_crtc->enabled)
		return;

	/* set pending plane state to disabled */
	for (i = 0; i < mtk_crtc->layer_nr; i++) {
		struct drm_plane *mtk_plane = &mtk_crtc->planes[i];
		struct mtk_plane_state *mtk_plane_state = to_mtk_plane_state(mtk_plane->state);

		if (mtk_plane->index == plane->index) {
			memcpy(mtk_plane_state, plane_state, sizeof(*plane_state));
			break;
		}
	}
	mtk_crtc_update_config(mtk_crtc, false);

#if IS_REACHABLE(CONFIG_MTK_CMDQ)
	/* wait for planes to be disabled by cmdq */
	if (mtk_crtc->cmdq_client.chan)
		wait_event_timeout(mtk_crtc->cb_blocking_queue,
				   mtk_crtc->cmdq_vblank_cnt == 0,
				   msecs_to_jiffies(500));
#endif
}

void mtk_crtc_async_update(struct drm_crtc *crtc, struct drm_plane *plane,
			   struct drm_atomic_state *state)
{
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);

	if (!mtk_crtc->enabled)
		return;

	mtk_crtc_update_config(mtk_crtc, false);
}

static void mtk_crtc_atomic_enable(struct drm_crtc *crtc,
				   struct drm_atomic_state *state)
{
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct drm_crtc_state *crtc_state = state->crtcs[drm_crtc_index(crtc)].new_state;
	struct mtk_crtc_state *mtk_crtc_state = to_mtk_crtc_state(crtc_state);
	struct mtk_ddp_comp *comp = mtk_crtc->ddp_comp[0];
	int ret;
	int i, j;
	int mmsys_cnt = 0;
	struct mtk_drm_private *priv;
	struct arm_smccc_res res;

	DRM_DEBUG_DRIVER("%s %d\n", __func__, crtc->base.id);

	for (i = 0; i < MAX_MMSYS; i++)
		if (mtk_crtc->exist[i])
			mmsys_cnt++;

	if (mmsys_cnt == 1) {
		ret = mtk_ddp_comp_power_on(comp);
		if (ret < 0) {
			DRM_DEV_ERROR(comp->dev, "Failed to enable power domain: %d\n", ret);
			return;
		}
	} else {
		for (i = 0; i < MAX_MMSYS; i++) {
			if (!mtk_crtc->exist[i])
				continue;
			ret = pm_runtime_resume_and_get(mtk_crtc->mmsys_dev[i]);
			if (ret < 0) {
				DRM_DEV_ERROR(mtk_crtc->mmsys_dev[i],
					      "Failed to enable power domain: %d\n", ret);
				for (j = i - 1; j >= 0; j--)
					if (mtk_crtc->exist[i])
						pm_runtime_put(mtk_crtc->mmsys_dev[j]);
				return;
			}
		}
	}

	priv = crtc->dev->dev_private;
	/* We should disable the secure state before use the display in some SOC,such as,mt8189 */
	if (priv->data->default_sec_mode) {
		arm_smccc_smc(MTK_SIP_KERNEL_DISP_CONTROL, DISP_ATF_CMD_CONFIG_DISP_CONFIG,
			      0, 0, 0, 0, 0, 0, &res);
		if (res.a0 != 0) {
			DRM_DEV_ERROR(comp->dev, "Disp disable security fail, ret %ld\n", res.a0);
			return;
		}
	}

#if IS_REACHABLE(CONFIG_MTK_DPC)
	if (mtk_crtc->dpc_dev) {
		ret = pm_runtime_resume_and_get(mtk_crtc->dpc_dev);
		if (ret < 0) {
			DRM_DEV_ERROR(mtk_crtc->dpc_dev,
				      "Failed to enable power domain: %d\n", ret);
			return;
		}
		dpc_enable(mtk_crtc->dpc_dev, DPC_SUBSYS_DISP);
	}
#endif
	mtk_crtc_update_output(crtc, state);

	/* Get dsc_info from output comp */
	comp = mtk_crtc->ddp_comp[mtk_crtc->ddp_comp_nr - 1];
	i = mtk_crtc->conn_routes_sys;
	priv = ((struct mtk_drm_private *)crtc->dev->dev_private)->all_drm_private[i];
	dev_dbg(priv->dev, "Updated DSC info path index %d\n",
		mtk_crtc->ddp_comp_nr - 1);
	mtk_ddp_comp_get_dsc_info(comp, &mtk_crtc_state->dsc);

	/* Set dsc_info in current crtc */
	for (i = 0; i < mtk_crtc->ddp_comp_nr; i++) {
		comp = mtk_crtc->ddp_comp[i];
		j = mtk_crtc->conn_routes_sys;
		priv = ((struct mtk_drm_private *)crtc->dev->dev_private)->all_drm_private[j];

		if (mtk_ddp_comp_get_type(comp->id) == MTK_DISP_DSC) {
			dev_dbg(priv->dev, "Updated DSC info path index %d\n",
				mtk_crtc->ddp_comp_nr - 1);
			mtk_ddp_comp_set_dsc_info(comp, &mtk_crtc_state->dsc);
		}
	}

	ret = mtk_crtc_ddp_hw_init(mtk_crtc);
	if (ret) {
		mtk_ddp_comp_power_off(comp);
		return;
	}

	drm_crtc_vblank_on(crtc);
	mtk_crtc->enabled = true;

	drm_vblank_work_init(&mtk_crtc->crc_work, crtc, mtk_crtc_crc_work);
}

static void mtk_crtc_atomic_disable(struct drm_crtc *crtc,
				    struct drm_atomic_state *state)
{
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp = mtk_crtc->ddp_comp[0];
	int i, ret;
	int mmsys_cnt = 0;

	DRM_DEBUG_DRIVER("%s %d\n", __func__, crtc->base.id);
	if (!mtk_crtc->enabled)
		return;

	mtk_crtc_disable_secure_state(crtc);

	for (i = 0; i < MAX_MMSYS; i++)
		if (mtk_crtc->exist[i])
			mmsys_cnt++;

	/* Set all pending plane state to disabled */
	for (i = 0; i < mtk_crtc->layer_nr; i++) {
		struct drm_plane *plane = &mtk_crtc->planes[i];
		struct mtk_plane_state *plane_state;

		plane_state = to_mtk_plane_state(plane->state);
		plane_state->pending.enable = false;
		plane_state->pending.config = true;
	}
	mtk_crtc->pending_planes = true;

	mtk_crtc_update_config(mtk_crtc, false);
#if IS_REACHABLE(CONFIG_MTK_CMDQ)
	/* Wait for planes to be disabled by cmdq */
	if (mtk_crtc->cmdq_client.chan)
		wait_event_timeout(mtk_crtc->cb_blocking_queue,
				   mtk_crtc->cmdq_vblank_cnt == 0,
				   msecs_to_jiffies(500));
#endif
	/* Wait for planes to be disabled */
	drm_crtc_wait_one_vblank(crtc);

	drm_crtc_vblank_off(crtc);
	mtk_crtc_ddp_hw_fini(mtk_crtc);

#if IS_REACHABLE(CONFIG_MTK_DPC)
	if (mtk_crtc->dpc_dev) {
		dpc_disable(mtk_crtc->dpc_dev, DPC_SUBSYS_DISP);
		ret = pm_runtime_put(mtk_crtc->dpc_dev);
		if (ret < 0)
			DRM_DEV_ERROR(mtk_crtc->dpc_dev,
				      "Failed to disable power domain: %d\n", ret);
	}
#endif

	if (mmsys_cnt == 1) {
		ret = pm_runtime_put(comp->dev);
		if (ret < 0)
			DRM_DEV_ERROR(comp->dev, "Failed to disable power domain: %d\n", ret);
	} else {
		for (i = 0; i < MAX_MMSYS; i++) {
			if (mtk_crtc->exist[i]) {
				ret = pm_runtime_put(mtk_crtc->mmsys_dev[i]);
				if (ret < 0)
					DRM_DEV_ERROR(mtk_crtc->mmsys_dev[i],
						      "Failed to disable power domain: %d\n", ret);
			}
		}
	}

	mtk_crtc->enabled = false;
}

static void mtk_crtc_atomic_begin(struct drm_crtc *crtc,
				  struct drm_atomic_state *state)
{
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state,
									  crtc);
	struct mtk_crtc_state *mtk_crtc_state = to_mtk_crtc_state(crtc_state);
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	unsigned long flags;

	if (priv->data->has_secure)
		mtk_crtc_plane_switch_sec_state(crtc, state);

	if (mtk_crtc->event && mtk_crtc_state->base.event)
		DRM_ERROR("new event while there is still a pending event\n");

	if (mtk_crtc_state->base.event) {
		mtk_crtc_state->base.event->pipe = drm_crtc_index(crtc);
		WARN_ON(drm_crtc_vblank_get(crtc) != 0);

		spin_lock_irqsave(&crtc->dev->event_lock, flags);
		mtk_crtc->event = mtk_crtc_state->base.event;
		spin_unlock_irqrestore(&crtc->dev->event_lock, flags);

		mtk_crtc_state->base.event = NULL;
	}
}

static void mtk_crtc_atomic_flush(struct drm_crtc *crtc,
				  struct drm_atomic_state *state)
{
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	int i;

	if (crtc->state->color_mgmt_changed)
		for (i = 0; i < mtk_crtc->ddp_comp_nr; i++) {
			mtk_ddp_gamma_set(mtk_crtc->ddp_comp[i], crtc->state);
			mtk_ddp_ctm_set(mtk_crtc->ddp_comp[i], crtc->state);
		}
	mtk_crtc_update_config(mtk_crtc, !!mtk_crtc->event);
}

static const struct drm_crtc_funcs mtk_crtc_funcs = {
	.set_config		= drm_atomic_helper_set_config,
	.page_flip		= drm_atomic_helper_page_flip,
	.destroy		= mtk_crtc_destroy,
	.reset			= mtk_crtc_reset,
	.atomic_duplicate_state	= mtk_crtc_duplicate_state,
	.atomic_destroy_state	= mtk_crtc_destroy_state,
	.enable_vblank		= mtk_crtc_enable_vblank,
	.disable_vblank		= mtk_crtc_disable_vblank,
	.set_crc_source		= mtk_crtc_set_crc_source,
	.verify_crc_source	= mtk_crtc_verify_crc_source,
};

static const struct drm_crtc_helper_funcs mtk_crtc_helper_funcs = {
	.mode_fixup	= mtk_crtc_mode_fixup,
	.mode_set_nofb	= mtk_crtc_mode_set_nofb,
	.mode_valid	= mtk_crtc_mode_valid,
	.atomic_begin	= mtk_crtc_atomic_begin,
	.atomic_flush	= mtk_crtc_atomic_flush,
	.atomic_enable	= mtk_crtc_atomic_enable,
	.atomic_disable	= mtk_crtc_atomic_disable,
};

static int mtk_crtc_init(struct drm_device *drm, struct mtk_crtc *mtk_crtc,
			 unsigned int pipe)
{
	struct drm_plane *primary = NULL;
	struct drm_plane *cursor = NULL;
	int i, ret;

	for (i = 0; i < mtk_crtc->layer_nr; i++) {
		if (mtk_crtc->planes[i].type == DRM_PLANE_TYPE_PRIMARY)
			primary = &mtk_crtc->planes[i];
		else if (mtk_crtc->planes[i].type == DRM_PLANE_TYPE_CURSOR)
			cursor = &mtk_crtc->planes[i];
	}

	ret = drm_crtc_init_with_planes(drm, &mtk_crtc->base, primary, cursor,
					&mtk_crtc_funcs, NULL);
	if (ret)
		goto err_cleanup_crtc;

	drm_crtc_helper_add(&mtk_crtc->base, &mtk_crtc_helper_funcs);

	return 0;

err_cleanup_crtc:
	drm_crtc_cleanup(&mtk_crtc->base);
	return ret;
}

static int mtk_crtc_num_comp_planes(struct mtk_crtc *mtk_crtc, int comp_idx)
{
	struct mtk_ddp_comp *comp;

	if (comp_idx > 1)
		return 0;

	comp = mtk_crtc->ddp_comp[comp_idx];
	if (!comp->funcs)
		return 0;

	if (comp_idx == 1 && !comp->funcs->bgclr_in_on)
		return 0;

	return mtk_ddp_comp_layer_nr(comp);
}

static inline
enum drm_plane_type mtk_crtc_plane_type(unsigned int plane_idx,
					unsigned int num_planes)
{
	if (plane_idx == 0)
		return DRM_PLANE_TYPE_PRIMARY;
	else if (plane_idx == (num_planes - 1))
		return DRM_PLANE_TYPE_CURSOR;
	else
		return DRM_PLANE_TYPE_OVERLAY;

}

static int mtk_crtc_init_comp_planes(struct drm_device *drm_dev,
				     struct mtk_crtc *mtk_crtc,
				     int comp_idx, int pipe)
{
	int num_planes = mtk_crtc_num_comp_planes(mtk_crtc, comp_idx);
	struct mtk_ddp_comp *comp = mtk_crtc->ddp_comp[comp_idx];
	int i, ret;

	for (i = 0; i < num_planes; i++) {
		ret = mtk_plane_init(drm_dev,
				&mtk_crtc->planes[mtk_crtc->layer_nr],
				BIT(pipe),
				mtk_crtc_plane_type(mtk_crtc->layer_nr, num_planes),
				mtk_ddp_comp_supported_rotations(comp),
				mtk_ddp_comp_get_blend_modes(comp),
				mtk_ddp_comp_get_formats(comp),
				mtk_ddp_comp_get_num_formats(comp), i);
		if (ret)
			return ret;

		mtk_crtc->layer_nr++;
	}
	return 0;
}

struct device *mtk_crtc_dma_dev_get(struct drm_crtc *crtc)
{
	struct mtk_crtc *mtk_crtc = NULL;

	if (!crtc)
		return NULL;

	mtk_crtc = to_mtk_crtc(crtc);
	if (!mtk_crtc)
		return NULL;

	return mtk_crtc->dma_dev;
}

int mtk_crtc_create(struct drm_device *drm_dev, enum mtk_crtc_path path_sel)
{
	struct mtk_drm_private *priv = drm_dev->dev_private;
	struct device *dev = drm_dev->dev;
	struct mtk_crtc *mtk_crtc;
	unsigned int num_comp_planes = 0;
	int ret;
	int i, j, k;
	bool has_ctm = false;
	uint gamma_lut_size = 0;
	struct drm_crtc *tmp;
	int crtc_i = 0;
	struct mtk_drm_private *subsys_priv;
	struct mtk_crtc_comp_info path[DDP_COMPONENT_ID_MAX];
	unsigned int path_len = 0;
	const struct mtk_drm_route *conn_routes = NULL;
	unsigned int num_conn_routes = 0;
	enum mtk_drm_mmsys conn_mmsys;

	drm_for_each_crtc(tmp, drm_dev)
		crtc_i++;

	for (j = 0; j < priv->data->mmsys_dev_num; j++) {
		for (k = 0; k < MAX_MMSYS; k++) {
			const unsigned int *subsys_path;
			unsigned int subsys_path_len = 0;
			unsigned int order = 0;

			subsys_priv = priv->all_drm_private[k];
			if (!subsys_priv)
				continue;

			if (path_sel == CRTC_MAIN) {
				subsys_path = subsys_priv->data->main_path;
				subsys_path_len = subsys_priv->data->main_len;
				order = subsys_priv->data->main_order;
			} else if (path_sel == CRTC_EXT) {
				subsys_path = subsys_priv->data->ext_path;
				subsys_path_len = subsys_priv->data->ext_len;
				order = subsys_priv->data->ext_order;
			} else if (path_sel == CRTC_THIRD) {
				subsys_path = subsys_priv->data->third_path;
				subsys_path_len = subsys_priv->data->third_len;
				order = subsys_priv->data->third_order;
			}

			if (subsys_priv->data->num_conn_routes) {
				conn_routes = subsys_priv->data->conn_routes;
				num_conn_routes = subsys_priv->data->num_conn_routes;
				conn_mmsys = subsys_priv->data->mmsys_id;
			}

			if (j != order)
				continue;
			if (!subsys_path_len)
				continue;

			for (i = 0; i < subsys_path_len; i++) {
				path[path_len].sys = subsys_priv->data->mmsys_id;
				path[path_len].comp_id = subsys_path[i];
				path_len++;
			}
		}
	}

	if (!path_len)
		return 0;

	if (num_conn_routes) {
		for (i = 0; i < num_conn_routes; i++)
			if (conn_routes->crtc_id == crtc_i)
				break;
		if (i == num_conn_routes) {
			num_conn_routes = 0;
			conn_routes = NULL;
		}
	}

	for (i = 0; i < path_len; i++) {
		enum mtk_ddp_comp_id comp_id = path[i].comp_id;
		struct device_node *node;
		struct mtk_ddp_comp *comp;

		priv = priv->all_drm_private[path[i].sys];
		node = priv->comp_node[comp_id];
		comp = &priv->ddp_comp[comp_id];

		/* Not all drm components have a DTS device node, such as ovl_adaptor,
		 * which is the drm bring up sub driver
		 */
		if (!node && comp_id != DDP_COMPONENT_DRM_OVL_ADAPTOR &&
		    comp_id != DDP_COMPONENT_DRM_OVLSYS_ADAPTOR0 &&
		    comp_id != DDP_COMPONENT_DRM_OVLSYS_ADAPTOR1 &&
		    comp_id != DDP_COMPONENT_DRM_OVLSYS_ADAPTOR2 &&
		    mtk_ddp_comp_get_type(comp_id) != MTK_DISP_VIRTUAL) {
			dev_info(dev,
				"Not creating crtc %d because component %d is disabled or missing\n",
				crtc_i, comp_id);
			return 0;
		}

		if (!comp->dev && mtk_ddp_comp_get_type(comp_id) != MTK_DISP_VIRTUAL) {
			dev_err(dev, "Component %pOF not initialized\n", node);
			return -ENODEV;
		}
	}

	mtk_crtc = devm_kzalloc(dev, sizeof(*mtk_crtc), GFP_KERNEL);
	if (!mtk_crtc)
		return -ENOMEM;

	mtk_crtc->qos_ctx = devm_kzalloc(dev, sizeof(struct mtk_crtc_qos_ctx), GFP_KERNEL);
	if (!mtk_crtc->qos_ctx)
		return -ENOMEM;

	for (i = 0; i < MAX_MMSYS; i++)
		if (priv->all_drm_private[i])
			mtk_crtc->mmsys_dev[i] = priv->all_drm_private[i]->mmsys_dev;
	mtk_crtc->ddp_comp_nr = path_len;
	mtk_crtc->ddp_comp = devm_kcalloc(dev,
					  mtk_crtc->ddp_comp_nr + (conn_routes ? 1 : 0),
					  sizeof(*mtk_crtc->ddp_comp),
					  GFP_KERNEL);
	if (!mtk_crtc->ddp_comp)
		return -ENOMEM;

	mtk_crtc->ddp_comp_sys = devm_kmalloc_array(dev, mtk_crtc->ddp_comp_nr +
						    (conn_routes ? 1 : 0),
						    sizeof(*mtk_crtc->ddp_comp_sys), GFP_KERNEL);
	if (!mtk_crtc->ddp_comp_sys)
		return -ENOMEM;

	for (i = 0; i < MAX_MMSYS; i++) {
		if (!priv->all_drm_private[i])
			continue;

		priv = priv->all_drm_private[i];
		mtk_crtc->mutex[i] = mtk_mutex_get(priv->mutex_dev);
		if (IS_ERR(mtk_crtc->mutex[i])) {
			ret = PTR_ERR(mtk_crtc->mutex[i]);
			dev_err(dev, "Failed to get mutex: %d\n", ret);
			return ret;
		}
	}

	for (i = 0; i < mtk_crtc->ddp_comp_nr; i++) {
		unsigned int comp_id = path[i].comp_id;
		struct mtk_ddp_comp *comp;

		priv = priv->all_drm_private[path[i].sys];
		comp = &priv->ddp_comp[comp_id];
		if (mtk_ddp_comp_get_type(comp_id) == MTK_DISP_VIRTUAL)
			comp->id = comp_id;
		mtk_crtc->ddp_comp[i] = comp;
		mtk_crtc->ddp_comp_sys[i] = path[i].sys;
		mtk_crtc->exist[path[i].sys] = true;

		if (comp->funcs) {
			if (comp->funcs->gamma_set && comp->funcs->gamma_get_lut_size) {
				unsigned int lut_sz = mtk_ddp_gamma_get_lut_size(comp);

				if (lut_sz)
					gamma_lut_size = lut_sz;
			}

			if (comp->funcs->ctm_set)
				has_ctm = true;

			if (comp->funcs->crc_cnt &&
			    comp->funcs->crc_entry &&
			    comp->funcs->crc_read)
				mtk_crtc->crc_provider = comp;

		}

		mtk_ddp_comp_register_vblank_cb(comp, mtk_crtc_ddp_irq,
						&mtk_crtc->base);
	}

	for (i = 0; i < mtk_crtc->ddp_comp_nr; i++)
		num_comp_planes += mtk_crtc_num_comp_planes(mtk_crtc, i);

	mtk_crtc->planes = devm_kcalloc(dev, num_comp_planes,
					sizeof(struct drm_plane), GFP_KERNEL);
	if (!mtk_crtc->planes)
		return -ENOMEM;

	for (i = 0; i < mtk_crtc->ddp_comp_nr; i++) {
		ret = mtk_crtc_init_comp_planes(drm_dev, mtk_crtc, i, crtc_i);
		if (ret)
			return ret;
	}

	/*
	 * Default to use the first component as the dma dev.
	 * In the case of ovl_adaptor sub driver, it needs to use the
	 * dma_dev_get function to get representative dma dev.
	 */
	priv = priv->all_drm_private[path[0].sys];
	mtk_crtc->dma_dev = mtk_ddp_comp_dma_dev_get(&priv->ddp_comp[path[0].comp_id]);

	mtk_crtc->vdisp_ao_dev = priv->vdisp_ao_dev;
	mtk_crtc->dpc_dev = priv->dpc_dev;
	mtk_crtc->pmqos_dev = priv->pmqos_dev;
	ret = mtk_crtc_init(drm_dev, mtk_crtc, crtc_i);
	if (ret < 0)
		return ret;

	if (gamma_lut_size)
		drm_mode_crtc_set_gamma_size(&mtk_crtc->base, gamma_lut_size);
	drm_crtc_enable_color_mgmt(&mtk_crtc->base, 0, has_ctm, gamma_lut_size);
	mutex_init(&mtk_crtc->hw_lock);
	spin_lock_init(&mtk_crtc->config_lock);

	if (mtk_crtc->layer_nr) {
		for (i = 0; i < mtk_crtc->layer_nr; i++)
			mtk_ddp_comp_channel_id_get(&priv->ddp_comp[path[0].comp_id], i,
						    &mtk_crtc->qos_ctx->plane_channel_id[i]);
	}

#if IS_REACHABLE(CONFIG_MTK_CMDQ)
	mtk_crtc->cmdq_client.client.dev = mtk_crtc->mmsys_dev[priv->data->mmsys_id];
	mtk_crtc->cmdq_client.client.tx_block = false;
	mtk_crtc->cmdq_client.client.knows_txdone = true;
	mtk_crtc->cmdq_client.client.rx_callback = ddp_cmdq_cb;
	mtk_crtc->cmdq_client.chan =
			mbox_request_channel(&mtk_crtc->cmdq_client.client, priv->mbox_index);
	if (IS_ERR(mtk_crtc->cmdq_client.chan)) {
		dev_dbg(dev, "mtk_crtc %d failed to create mailbox client, writing register by CPU now\n",
			drm_crtc_index(&mtk_crtc->base));
		mtk_crtc->cmdq_client.chan = NULL;
	}

	if (mtk_crtc->cmdq_client.chan) {
		char name[30];

		ret = of_property_read_u32_index(priv->mutex_node,
						 "mediatek,gce-events",
						 priv->mbox_index,
						 &mtk_crtc->cmdq_event);
		if (ret) {
			dev_dbg(dev, "mtk_crtc %d failed to get mediatek,gce-events property\n",
				drm_crtc_index(&mtk_crtc->base));
			goto cmdq_err;
		} else {
			ret = mtk_drm_cmdq_pkt_create(&mtk_crtc->cmdq_client,
						      &mtk_crtc->cmdq_handle,
						      PAGE_SIZE);
			if (ret) {
				dev_dbg(dev, "mtk_crtc %d failed to create cmdq packet\n",
					drm_crtc_index(&mtk_crtc->base));
				goto cmdq_err;
			}
		}

		/* for sending blocking cmd in crtc disable */
		init_waitqueue_head(&mtk_crtc->cb_blocking_queue);
		priv->mbox_index++;

		/* for update hrt bw in non irq context */
		snprintf(name, 30, "crtc%d_cmdq_done_thread", crtc_i);
		mtk_crtc->cmdq_done_task = kthread_create(ddp_cmdq_done_kthread, mtk_crtc, name);
		atomic_set(&mtk_crtc->cmdq_done, 0);
		wake_up_process(mtk_crtc->cmdq_done_task);
	}

	if (priv->data->has_secure) {
		if (priv->sec_mbox_index == 0)
			priv->sec_mbox_index = priv->data->sec_mbox_index;
		mtk_crtc->sec_cmdq_client.client.dev = mtk_crtc->mmsys_dev[priv->data->mmsys_id];
		mtk_crtc->sec_cmdq_client.client.tx_block = false;
		mtk_crtc->sec_cmdq_client.client.knows_txdone = true;
		mtk_crtc->sec_cmdq_client.client.rx_callback = ddp_cmdq_cb;
		mtk_crtc->sec_cmdq_client.chan =
			mbox_request_channel(&mtk_crtc->sec_cmdq_client.client, priv->sec_mbox_index);
		if (IS_ERR(mtk_crtc->sec_cmdq_client.chan)) {
			dev_err(dev, "mtk_crtc %d failed to create sec mailbox client\n",
				drm_crtc_index(&mtk_crtc->base));
			mtk_crtc->sec_cmdq_client.chan = NULL;
		}

		if (mtk_crtc->sec_cmdq_client.chan) {
			/* for sending blocking cmd in crtc disable */
			init_waitqueue_head(&mtk_crtc->sec_cb_blocking_queue);
			priv->sec_mbox_index++;
		}
	}

cmdq_err:
	if (ret) {
		if (mtk_crtc->cmdq_client.chan) {
			mbox_free_channel(mtk_crtc->cmdq_client.chan);
			mtk_crtc->cmdq_client.chan = NULL;
		}
		if (mtk_crtc->sec_cmdq_client.chan) {
			mbox_free_channel(mtk_crtc->sec_cmdq_client.chan);
			mtk_crtc->sec_cmdq_client.chan = NULL;
		}
	}
#endif

	if (conn_routes) {
		priv = priv->all_drm_private[conn_mmsys];
		for (i = 0; i < num_conn_routes; i++) {
			unsigned int comp_id = conn_routes[i].route_ddp;
			struct device_node *node = priv->comp_node[comp_id];
			struct mtk_ddp_comp *comp = &priv->ddp_comp[comp_id];

			if (!comp->dev) {
				dev_dbg(dev, "comp_id:%d, Component %pOF not initialized\n",
					comp_id, node);
				/* mark encoder_index to -1, if route comp device is not enabled */
				comp->encoder_index = -1;
				continue;
			}

			mtk_ddp_comp_encoder_index_set(&priv->ddp_comp[comp_id]);
		}

		mtk_crtc->conn_routes_sys = conn_mmsys;
		mtk_crtc->num_conn_routes = num_conn_routes;
		mtk_crtc->conn_routes = conn_routes;

		/* increase ddp_comp_nr at the end of mtk_crtc_create */
		mtk_crtc->ddp_comp_nr++;
	}

	for (i = 0; i < MAX_MMSYS; i++)
		if (mtk_crtc->exist[i])
			device_link_add(mtk_crtc->base.dev->dev,
					priv->all_drm_private[i]->mutex_dev, 0);

	return 0;
}

#if IS_REACHABLE(CONFIG_MTK_CMDQ)
void mtk_crtc_destroy_crc_cmdq(struct mtk_crtc_crc *crc)
{
	if (!crc->cnt)
		return;

	if (crc->pa) {
		dma_unmap_single(crc->cmdq_client.chan->mbox->dev,
				 crc->pa, crc->cnt * sizeof(*crc->va),
				 DMA_TO_DEVICE);
		crc->pa = 0;
	}

	if (crc->cmdq_client.chan) {
		cmdq_pkt_destroy(&crc->cmdq_client, &crc->cmdq_handle);
		mbox_free_channel(crc->cmdq_client.chan);
		crc->cmdq_client.chan = NULL;
	}
}

/**
 * mtk_crtc_create_crc_cmdq - Create a CMDQ thread for syncing the CRCs
 * @dev: Kernel device node of the CRC provider
 * @crc: Pointer of the CRC to init
 *
 * This function will create a looping thread on GCE (Global Command Engine) to
 * keep the CRC up to date by monitoring the assigned event (usually the frame
 * done event) of the CRC provider, and read the CRCs from the registers to a
 * shared memory for the workqueue to read. To start/stop the looping thread,
 * please call 'mtk_crtc_start_crc_cmdq()' and 'mtk_crtc_stop_crc_cmdq()'
 * defined below
 *
 * The reaseon why we don't update the CRCs with CPU is that the front porch of
 * 4k60 timing in CEA-861 is less than 60us, and register read/write speed is
 * relatively unreliable comparing to GCE due to the bus design.
 *
 * We must create a new thread instead of using the original one for plane
 * update is because:
 * 1. We cannot add another wait-for-event command at the end of cmdq packet, or
 *    the cmdq callback will delay for too long
 * 2. Will get the CRC of the previous frame if using the existed wait-for-event
 *    command which is at the beginning of the packet
 */
void mtk_crtc_create_crc_cmdq(struct device *dev, struct mtk_crtc_crc *crc)
{
	int i;
	dma_addr_t crc_pa;

	if (!crc->cnt) {
		dev_warn(dev, "%s: not support\n", __func__);
		return;
	}

	if (!crc->va) {
		dev_warn(dev, "%s: no memory\n", __func__);
		return;
	}

	crc->cmdq_client.client.dev = dev;
	crc->cmdq_client.client.tx_block = false;
	crc->cmdq_client.client.knows_txdone = true;
	crc->cmdq_client.client.rx_callback = NULL;
	crc->cmdq_client.chan = mbox_request_channel(&crc->cmdq_client.client, 0);
	if (IS_ERR(crc->cmdq_client.chan)) {
		dev_warn(dev, "%s: failed to create mailbox client\n", __func__);
		crc->cmdq_client.chan = NULL;
		goto cleanup;
	}

	if (cmdq_pkt_create(&crc->cmdq_client, &crc->cmdq_handle, PAGE_SIZE)) {
		dev_warn(dev, "%s: failed to create cmdq packet\n", __func__);
		goto cleanup;
	}

	/* map the entry to get a dma address for cmdq to store the crc */
	crc->pa = dma_map_single(crc->cmdq_client.chan->mbox->dev,
				 crc->va, crc->cnt * sizeof(*crc->va),
				 DMA_FROM_DEVICE);

	if (dma_mapping_error(crc->cmdq_client.chan->mbox->dev, crc->pa)) {
		dev_err(dev, "%s: failed to map dma\n", __func__);
		goto cleanup;
	}

	if (crc->cmdq_event)
		cmdq_pkt_wfe(&crc->cmdq_handle, crc->cmdq_event, true);

	for (i = 0; i < crc->cnt; i++) {
		crc_pa = crc->pa + i * sizeof(*crc->va);

		if (cmdq_addr_need_offset(crc->cmdq_client.chan, crc_pa))
			crc_pa += cmdq_get_offset_pa(crc->cmdq_client.chan);

		/* put crc to spr1 register */
		if (crc->to_mminfra_out) {
			cmdq_pkt_assign(&crc->cmdq_handle, CMDQ_THR_SPR_IDX0,
				CMDQ_ADDR_HIGH(crc->crc_out));
			cmdq_pkt_read_s(&crc->cmdq_handle, CMDQ_THR_SPR_IDX0,
				CMDQ_ADDR_LOW(crc->ofs[i]),
				CMDQ_THR_SPR_IDX1);
		} else {
			cmdq_pkt_assign(&crc->cmdq_handle, CMDQ_THR_SPR_IDX0,
				CMDQ_ADDR_HIGH(crc->cmdq_reg->pa_base));
			cmdq_pkt_read_s(&crc->cmdq_handle, CMDQ_THR_SPR_IDX0,
				CMDQ_ADDR_LOW(crc->cmdq_reg->offset + crc->ofs[i]),
				CMDQ_THR_SPR_IDX1);
		}

		/* copy spr1 register to physical address of the crc */
		cmdq_pkt_assign(&crc->cmdq_handle, CMDQ_THR_SPR_IDX2,
				CMDQ_ADDR_HIGH(crc_pa));
		cmdq_pkt_write_s(&crc->cmdq_handle, CMDQ_THR_SPR_IDX2,
				 CMDQ_ADDR_LOW(crc_pa),
				 CMDQ_THR_SPR_IDX1);
	}

	/* reset crc */
	mtk_ddp_write_mask(&crc->cmdq_handle, ~0, crc->cmdq_reg, 0,
			   crc->rst_ofs, crc->rst_msk);

	/* clear reset bit */
	mtk_ddp_write_mask(&crc->cmdq_handle, 0, crc->cmdq_reg, 0,
			   crc->rst_ofs, crc->rst_msk);

	/* jump to head of the cmdq packet */
	cmdq_pkt_jump_abs(&crc->cmdq_handle, crc->cmdq_handle.pa_base,
			  cmdq_get_shift_pa(crc->cmdq_client.chan));

	return;

cleanup:
	mtk_crtc_destroy_crc_cmdq(crc);
}

/**
 * mtk_crtc_start_crc_cmdq - Start the GCE looping thread for CRC update
 * @crc: Pointer of the CRC information
 */
void mtk_crtc_start_crc_cmdq(struct mtk_crtc_crc *crc)
{
	if (!crc->cmdq_client.chan)
		return;

	dma_sync_single_for_device(crc->cmdq_client.chan->mbox->dev,
				   crc->cmdq_handle.pa_base,
				   crc->cmdq_handle.cmd_buf_size,
				   DMA_TO_DEVICE);
	mbox_send_message(crc->cmdq_client.chan, &crc->cmdq_handle);
	mbox_client_txdone(crc->cmdq_client.chan, 0);
}

/**
 * mtk_crtc_stop_crc_cmdq - Stop the GCE looping thread for CRC update
 * @crc: Pointer of the CRC information
 */
void mtk_crtc_stop_crc_cmdq(struct mtk_crtc_crc *crc)
{
	if (!crc->cmdq_client.chan)
		return;

	/* remove all the commands from the cmdq packet */
	mbox_flush(crc->cmdq_client.chan, 2000);
}

void mtk_crtc_read_crc(struct mtk_crtc_crc *crc)
{
	if (!crc->cmdq_client.chan)
		return;

	/* sync to see the most up-to-date copy of the DMA buffer */
	dma_sync_single_for_cpu(crc->cmdq_client.chan->mbox->dev,
				crc->pa, crc->cnt * sizeof(*crc->va),
				DMA_FROM_DEVICE);
}
#endif
