// SPDX-License-Identifier: GPL-2.0 or MIT
/* Copyright 2025 ARM Limited. All rights reserved. */

#include <linux/nvmem-consumer.h>

#include "panthor_device.h"
#include "panthor_hw.h"
#include "panthor_regs.h"

#define GPU_PROD_ID_MAKE(arch_major, prod_major) \
	(((arch_major) << 24) | (prod_major))

static char *get_gpu_model_name(struct panthor_device *ptdev)
{
	const u32 gpu_id = ptdev->gpu_info.gpu_id;
	const u32 product_id = GPU_PROD_ID_MAKE(GPU_ARCH_MAJOR(gpu_id),
						GPU_PROD_MAJOR(gpu_id));
	const bool ray_intersection = !!(ptdev->gpu_info.gpu_features &
					 GPU_FEATURES_RAY_INTERSECTION);
	const u8 shader_core_count = hweight64(ptdev->gpu_info.shader_present);

	switch (product_id) {
	case GPU_PROD_ID_MAKE(10, 2):
		return "Mali-G710";
	case GPU_PROD_ID_MAKE(10, 3):
		return "Mali-G510";
	case GPU_PROD_ID_MAKE(10, 4):
		return "Mali-G310";
	case GPU_PROD_ID_MAKE(10, 7):
		return "Mali-G610";
	case GPU_PROD_ID_MAKE(11, 2):
		if (shader_core_count > 10 && ray_intersection)
			return "Mali-G715-Immortalis";
		else if (shader_core_count >= 7)
			return "Mali-G715";

		fallthrough;
	case GPU_PROD_ID_MAKE(11, 3):
		return "Mali-G615";
	case GPU_PROD_ID_MAKE(12, 0):
		if (shader_core_count >= 10 && ray_intersection)
			return "Mali-G720-Immortalis";
		else if (shader_core_count >= 6)
			return "Mali-G720";

		fallthrough;
	case GPU_PROD_ID_MAKE(12, 1):
		return "Mali-G620";
	case GPU_PROD_ID_MAKE(13, 0):
		if (shader_core_count >= 10 && ray_intersection)
			return "Mali-G925-Immortalis";
		else if (shader_core_count >= 6)
			return "Mali-G725";

		fallthrough;
	case GPU_PROD_ID_MAKE(13, 1):
		return "Mali-G625";
	}

	return "(Unknown Mali GPU)";
}

static int panthor_gpu_get_shader_present(struct panthor_device *ptdev, __u64 *reg)
{
	struct device *dev =ptdev->base.dev;
	struct nvmem_cell *cell = nvmem_cell_get(dev, "shader-present");
	ssize_t len;
	void *buf;
	int ret;

	if (IS_ERR(cell)) {
		/* On platforms without this cell, use the Mali register */
		if (PTR_ERR(cell) == -ENOENT) {
			*reg = gpu_read64(ptdev, GPU_SHADER_PRESENT);
			return 0;
		}

		return dev_err_probe(dev, PTR_ERR(cell),
				     "Failed to get shader-present nvmem cell\n");
	}

	buf = nvmem_cell_read(cell, &len);
	if (IS_ERR(buf)) {
		ret = dev_err_probe(dev, PTR_ERR(buf),
				    "Failed to read shader-present nvmem cell\n");
		goto err_put_cell;
	}

	if (!len || len > 8) {
		ret = dev_err_probe(dev, -EINVAL, "shader-present cell can't be length %ld\n",
				    len);
		goto err_free;
	}

	*reg = 0;
	memcpy(reg, buf, len);

	kfree(buf);
	nvmem_cell_put(cell);

	return 0;

err_free:
	kfree(buf);
err_put_cell:
	nvmem_cell_put(cell);

	return ret;
}

static int panthor_gpu_info_init(struct panthor_device *ptdev)
{
	unsigned int i;
	int ret;

	ptdev->gpu_info.gpu_id = gpu_read(ptdev, GPU_ID);
	ptdev->gpu_info.csf_id = gpu_read(ptdev, GPU_CSF_ID);
	ptdev->gpu_info.gpu_rev = gpu_read(ptdev, GPU_REVID);
	ptdev->gpu_info.core_features = gpu_read(ptdev, GPU_CORE_FEATURES);
	ptdev->gpu_info.l2_features = gpu_read(ptdev, GPU_L2_FEATURES);
	ptdev->gpu_info.tiler_features = gpu_read(ptdev, GPU_TILER_FEATURES);
	ptdev->gpu_info.mem_features = gpu_read(ptdev, GPU_MEM_FEATURES);
	ptdev->gpu_info.mmu_features = gpu_read(ptdev, GPU_MMU_FEATURES);
	ptdev->gpu_info.thread_features = gpu_read(ptdev, GPU_THREAD_FEATURES);
	ptdev->gpu_info.max_threads = gpu_read(ptdev, GPU_THREAD_MAX_THREADS);
	ptdev->gpu_info.thread_max_workgroup_size = gpu_read(ptdev, GPU_THREAD_MAX_WORKGROUP_SIZE);
	ptdev->gpu_info.thread_max_barrier_size = gpu_read(ptdev, GPU_THREAD_MAX_BARRIER_SIZE);
	ptdev->gpu_info.coherency_features = gpu_read(ptdev, GPU_COHERENCY_FEATURES);
	for (i = 0; i < 4; i++)
		ptdev->gpu_info.texture_features[i] = gpu_read(ptdev, GPU_TEXTURE_FEATURES(i));

	ptdev->gpu_info.as_present = gpu_read(ptdev, GPU_AS_PRESENT);

	ret = panthor_gpu_get_shader_present(ptdev, &ptdev->gpu_info.shader_present);
	if (ret)
		return ret;

	ptdev->gpu_info.tiler_present = gpu_read64(ptdev, GPU_TILER_PRESENT);
	ptdev->gpu_info.l2_present = gpu_read64(ptdev, GPU_L2_PRESENT);

	/* Introduced in arch 11.x */
	ptdev->gpu_info.gpu_features = gpu_read64(ptdev, GPU_FEATURES);

	return 0;
}

static int panthor_hw_info_init(struct panthor_device *ptdev)
{
	u32 major, minor, status;
	int ret;

	ret = panthor_gpu_info_init(ptdev);
	if (ret)
		return ret;

	major = GPU_VER_MAJOR(ptdev->gpu_info.gpu_id);
	minor = GPU_VER_MINOR(ptdev->gpu_info.gpu_id);
	status = GPU_VER_STATUS(ptdev->gpu_info.gpu_id);

	drm_info(&ptdev->base,
		 "%s id 0x%x major 0x%x minor 0x%x status 0x%x",
		 get_gpu_model_name(ptdev), ptdev->gpu_info.gpu_id >> 16,
		 major, minor, status);

	drm_info(&ptdev->base,
		 "Features: L2:%#x Tiler:%#x Mem:%#x MMU:%#x AS:%#x",
		 ptdev->gpu_info.l2_features,
		 ptdev->gpu_info.tiler_features,
		 ptdev->gpu_info.mem_features,
		 ptdev->gpu_info.mmu_features,
		 ptdev->gpu_info.as_present);

	drm_info(&ptdev->base,
		 "shader_present=0x%0llx l2_present=0x%0llx tiler_present=0x%0llx",
		 ptdev->gpu_info.shader_present, ptdev->gpu_info.l2_present,
		 ptdev->gpu_info.tiler_present);

	return 0;
}

int panthor_hw_init(struct panthor_device *ptdev)
{
	return panthor_hw_info_init(ptdev);
}
