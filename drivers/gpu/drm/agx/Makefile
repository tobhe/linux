# SPDX-License-Identifier: GPL-2.0-only OR MIT

agxdrm-y := agx_drv.o

obj-$(CONFIG_DRM_AGX) += agxdrm.o

# header test

# exclude some broken headers from the test coverage
no-header-test := \

always-y += \
	$(patsubst %.h,%.hdrtest, $(filter-out $(no-header-test), \
		$(shell cd $(srctree)/$(src) && find * -name '*.h')))

quiet_cmd_hdrtest = HDRTEST $(patsubst %.hdrtest,%.h,$@)
      cmd_hdrtest = $(CC) $(filter-out $(CFLAGS_GCOV), $(c_flags)) -S -o /dev/null -x c /dev/null -include $<; touch $@

$(obj)/%.hdrtest: $(src)/%.h FORCE
	$(call if_changed_dep,hdrtest)
