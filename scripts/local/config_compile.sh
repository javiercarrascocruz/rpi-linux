#!/bin/bash
set -x #echo on

#64-bit kernel config for Raspberry Pi Zero 2 W
KERNEL=kernel8
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- bcm2711_defconfig
#directories to mount the SD partitions
mkdir -p mnt/{fat32,ext4}
#cross-compile
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- Image modules dtbs -j12
