human_arch	= ARMv8
build_arch	= arm64
header_arch	= $(build_arch)
defconfig	= defconfig
flavours	= asahi-edge
build_image	= Image.gz
kernel_file	= arch/$(build_arch)/boot/Image.gz
install_file	= vmlinuz
no_dumpfile	= true

do_linux_tools		= false
do_tools_usbip		= false
do_tools_cpupower	= false
do_tools_perf		= false
do_tools_bpftool	= false

do_common_headers_indep	= false

do_dtbs			= true
do_libc_dev_package	= false
do_doc_package		= false
do_source_package	= false
do_extras_package	= false
do_zfs			= false

skipabi		= true
skipmodule	= true
skipretpoline	= true

gcc=clang
