#!/usr/bin/env bash
set -ex

if [ -z "$KERNEL_ARCH" ]; then
  exit 1
fi

# preparation
mkdir -p kernels modules dtbs
rm -f .config defconfig kernels/* modules/* dtbs/*

LOCALVERSION=$(git branch --show-current)
export LOCALVERSION
export MAKEFLAGS="-j${FDO_CI_CONCURRENT:-4}"

GIT_TAG=$(git describe --tags --always)

if [ "${KERNEL_ARCH}" == "x86_64" ]; then
  DEFCONFIG="arch/x86/configs/x86_64_defconfig"
  KERNEL_IMAGE_NAME=( bzImage )
elif [ "${KERNEL_ARCH}" == "arm" ]; then
  CROSS_COMPILE=arm-linux-gnueabihf-
  DEFCONFIG="arch/arm/configs/multi_v7_defconfig"
  KERNEL_IMAGE_NAME=( zImage )
  DT_PATH="arch/arm/boot/dts"
elif [ "${KERNEL_ARCH}" == "arm64" ]; then
  CROSS_COMPILE=aarch64-linux-gnu-
  DEFCONFIG="arch/arm64/configs/defconfig"
  KERNEL_IMAGE_NAME=( Image )
  DT_PATH="arch/arm64/boot/dts"
else
  exit 1
fi

make() {
    command make ARCH="${KERNEL_ARCH}" CROSS_COMPILE="${CROSS_COMPILE}" "$@"
}
export ARCH="${KERNEL_ARCH}"
export CROSS_COMPILE="${CROSS_COMPILE}"

# defconfig
./scripts/kconfig/merge_config.sh ${DEFCONFIG} kernel/configs/mesa3d-ci_"${KERNEL_ARCH}".config

#build
make "${KERNEL_IMAGE_NAME[@]}"
for image in "${KERNEL_IMAGE_NAME[@]}"; do
  cp -v "arch/${KERNEL_ARCH}/boot/${image}" kernels/
done

if [ -n "${DT_PATH:-}" ]; then
  make dtbs
  find "${DT_PATH}" -type f -name '*.dtb' -exec cp --update=none-fail -v {} dtbs/ \;
fi

# workarounds and specific stuff
if [[ ${KERNEL_ARCH} = "arm64" ]]; then
  { # Google's Cheza
    make Image.lzma  # Google's Cheza
    mkimage \
        -f auto \
        -A arm \
        -O linux \
        -d arch/arm64/boot/Image.lzma \
        -C lzma\
        -b arch/arm64/boot/dts/qcom/sdm845-cheza-r3.dtb \
        kernels/cheza-kernel
    KERNEL_IMAGE_NAME+=( cheza-kernel )
  }
  { # db410c
    gzip -k kernels/Image
    KERNEL_IMAGE_NAME+=( Image.gz )
  }
fi

# modules
make modules
make INSTALL_MOD_PATH=modules modules_install
rm modules/lib/modules/*/build
tar --zstd -cvf modules.tar.zst -C modules .
rm modules -rf

# defconfig template
make savedefconfig

if [ -n "$S3_HOST" ] && [ -n "$CI_PROJECT_PATH" ] && [ -n "$DEBIAN_ARCH" ] && [ -n "$S3_JWT_FILE" ]; then
  # FIXME: drop DEBIAN_ARCH
  S3_BUCKET="mesa-rootfs"
  S3_PATH="${S3_HOST}/${S3_BUCKET}/${CI_PROJECT_PATH}/${GIT_TAG}/${DEBIAN_ARCH}"

  # upload
  FILES_TO_UPLOAD=( modules.tar.zst kernels/* )
  if [ "${KERNEL_ARCH}" != "x86_64" ]; then
    FILES_TO_UPLOAD+=( dtbs/* )
  fi

  for f in "${FILES_TO_UPLOAD[@]}"; do
    ci-fairy s3cp --token-file "${S3_JWT_FILE}" "$f" "https://${S3_PATH}/$(basename -a "$f")"
  done
else
  echo "Skipping upload as one of the these vars is not set: S3_HOST, CI_PROJECT_PATH, DEBIAN_ARCH, S3_JWT_FILE"
fi

git clean --quiet -fdx -e 'ccache/' -e '.config' -e 'defconfig' -e 'modules.tar.zst' -e 'kernels/' -e 'dtbs/'

echo "GIT_TAG: ${GIT_TAG}"
