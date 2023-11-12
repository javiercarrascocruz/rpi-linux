#!/bin/bash
set -x #echo on

#64-bit kernel config for Raspberry Pi Zero 2 W
KERNEL=kernel8
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- bcm2711_defconfig
#cross-compile
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- Image modules dtbs -j12
# mount SD partitions
sudo mount /dev/sda1 mnt/fat32
sudo mount /dev/sda2 mnt/ext4
#install kernel modules
sudo env PATH=$PATH make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- INSTALL_MOD_PATH=mnt/ext4 modules_install
#copy custom kernel keeping the original one as backup (add kernel=kernel-myconfig.img to config.txt)
sudo cp arch/arm64/boot/Image mnt/fat32/kernel-myconfig.img
#install overlays
sudo cp arch/arm64/boot/dts/broadcom/*.dtb mnt/fat32/
sudo cp arch/arm64/boot/dts/overlays/*.dtb* mnt/fat32/overlays/
sudo cp arch/arm64/boot/dts/overlays/README mnt/fat32/overlays/
#unmount SD partitions
sudo umount mnt/fat32
sudo umount mnt/ext4
