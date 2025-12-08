# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_DMABUF_HEAPS_CMA)		+= cma_heap.o
obj-$(CONFIG_DMABUF_HEAPS_RESTRICTED)	+= restricted_heap.o
obj-$(CONFIG_DMABUF_HEAPS_RESTRICTED_MTK)	+= restricted_heap_mtk.o
obj-$(CONFIG_DMABUF_HEAPS_SYSTEM)	+= system_heap.o

mtk_heaps-objs := \
	mtk_system_heap.o \
	mtk_page_pool.o \
	mtk_cache_pool.o \
	mtk_dmabuf_monitor.o \
	mtk-deferred-free-helper.o \
	mtk_heap_refill.o

obj-$(CONFIG_DMABUF_HEAPS_MTK_SYSTEM)	+= mtk_heaps.o

