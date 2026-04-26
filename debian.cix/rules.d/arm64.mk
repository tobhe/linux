build_arch	= arm64
defconfig	= defconfig
flavours	= cix
build_image	= vmlinuz.efi
kernel_file	= arch/$(build_arch)/boot/vmlinuz.efi
install_file	= vmlinuz
no_dumpfile	= true
uefi_signed     = true

vdso		= vdso_install

do_tools_usbip  = true
do_tools_cpupower = true
do_tools_perf   = true
do_tools_perf_jvmti = false
do_tools_perf_python = false
do_tools_bpftool = false
do_tools_rtla = false

do_dtbs		= true
