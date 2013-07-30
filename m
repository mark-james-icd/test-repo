#!/bin/sh
PWD=$(pwd)
echo $PWD
export CROSS_COMPILE=arm-linux-gnueabihf-
export ARCH=arm
export TEGRA_KERNEL_OUT=$PWD/../../_out
mkdir -p $TEGRA_KERNEL_OUT
make O=$TEGRA_KERNEL_OUT $@
