#!/bin/bash
set -x #echo on

#cross-compile
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- Image modules dtbs -j12

