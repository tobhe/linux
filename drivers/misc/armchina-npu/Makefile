# SPDX-License-Identifier: GPL-2.0

ccflags-y += -I$(src) -I$(src)/include -I$(src)/zhouyi
ccflags-y += -DKMD_VERSION=\"5.11.0\"
ccflags-$(CONFIG_ENABLE_DEVFREQ) += -DCONFIG_ENABLE_DEVFREQ

obj-$(CONFIG_ARMCHINA_NPU) += armchina_npu.o

armchina_npu-y := \
aipu.o \
aipu_common.o \
aipu_io.o \
aipu_irq.o \
aipu_job_manager.o \
aipu_mm.o \
aipu_dma_buf.o \
aipu_priv.o \
aipu_tcb.o \
zhouyi/zhouyi.o

armchina_npu-$(CONFIG_ARMCHINA_NPU_ARCH_V1) += \
zhouyi/v1.o \
zhouyi/v1v2_priv.o

armchina_npu-$(CONFIG_ARMCHINA_NPU_ARCH_V2) += \
zhouyi/v2.o \
zhouyi/v1v2_priv.o

armchina_npu-$(CONFIG_ARMCHINA_NPU_ARCH_V3) += \
zhouyi/v3.o \
zhouyi/v3_priv.o

armchina_npu-$(CONFIG_ARMCHINA_NPU_ARCH_V3_1) += \
zhouyi/v3_1.o \
zhouyi/v3_1_priv.o

armchina_npu-$(CONFIG_ARMCHINA_NPU_SOC_DEFAULT) += default/default.o
armchina_npu-$(CONFIG_ARMCHINA_NPU_SOC_R329)    += r329/r329.o
armchina_npu-$(CONFIG_ARMCHINA_NPU_SOC_SKY1)            += sky1/sky1.o
