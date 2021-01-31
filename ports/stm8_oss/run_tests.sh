#!/bin/bash 

# change to current working directory
cd `dirname $0`

# SDCC test
echo ""
echo "run SDCC " $1
stm8flash -c stlink -p stm8s105c6 -w build-sdcc/$1.ihx
miniterm /dev/ttyUSB0 9600

# Cosmic test
echo ""
echo "run Cosmic" $1
stm8flash -c stlink -p stm8s105c6 -w build-cosmic/$1.s19
miniterm /dev/ttyUSB0 9600

# IAR test
echo ""
echo "run IAR" $1
stm8flash -c stlink -p stm8s105c6 -w build-iar/$1.s19
miniterm /dev/ttyUSB0 9600
