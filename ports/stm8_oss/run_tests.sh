#!/bin/bash 

# change to current working directory
cd `dirname $0`

# SDCC test
echo ""
echo "run SDCC " $1
stm8flash -c stlink -p stm8s105c6 -w build-sdcc/$1.ihx		# STM8S Discovery
#stm8flash -c stlinkv2 -p stm8l152c6 -w build-sdcc/$1.ihx	# STM8L Discovery
#stm8gal -p /dev/ttyUSB0 -V 0 -R 0 -w build-sdcc/$1.ihx		# Sduino / muBoard
miniterm /dev/ttyUSB0 9600
exit

# Cosmic test
echo ""
echo "run Cosmic" $1
stm8flash -c stlink -p stm8s105c6 -w build-cosmic/$1.s19	# STM8S Discovery
#stm8flash -c stlinkv2 -p stm8l152c6 -w build-cosmic/$1.ihx	# STM8L Discovery
#stm8gal -p /dev/ttyUSB0 -V 0 -w build-cosmic/$1.ihx		# Sduino / muBoard
miniterm /dev/ttyUSB0 9600

# IAR test
echo ""
echo "run IAR" $1
stm8flash -c stlink -p stm8s105c6 -w build-iar/$1.s19		# STM8S Discovery
#stm8flash -c stlinkv2 -p stm8l152c6 -w build-cosmic/$1.ihx	# STM8L Discovery
#stm8gal -p /dev/ttyUSB0 -V 0 -w build-cosmic/$1.ihx		# Sduino / muBoard
miniterm /dev/ttyUSB0 9600
