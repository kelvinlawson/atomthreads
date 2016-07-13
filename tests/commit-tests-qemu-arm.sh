#!/bin/bash 

# Quit on any error
set -e

# Start at top-level directory
cd `dirname $0`/..

# Build qemu-arm and run tests
cd ports/arm/platforms/qemu_integratorcp
make clean
make
make qemutests
