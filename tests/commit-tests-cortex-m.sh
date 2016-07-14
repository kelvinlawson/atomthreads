#!/bin/bash 

# Quit on any error
set -e

# Start at top-level directory
cd `dirname $0`/..

# Build qemu-arm-cortex-m and run tests
cd ports/cortex-m
# git submodule?
make clean
make
make BOARD=qemu QEMU=/usr/local/bin/qemu-system-gnuarmeclipse qemutests
