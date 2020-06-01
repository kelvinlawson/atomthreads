#!/bin/bash

# Quit on any error
set -e
cd ../ports/x86

BIN_DIR=build/run/iso/boot
GRUB_DIR=$BIN_DIR/grub
GRUB_CFG=$GRUB_DIR/grub.cfg
ISO=all_tests_multiboot.iso

make clean

mkdir -p $GRUB_DIR

make 

cp build/*.bin $BIN_DIR

for fn in $BIN_DIR/*.bin; do
	echo "menuentry \"$(basename $fn)\" { multiboot /boot/$(basename $fn) }" >>  $GRUB_CFG
done

if [ $(ls $BIN_DIR/*.bin | wc -l) -gt 0   ];then 
	grub-mkrescue -o  $ISO build/run/iso
	qemu-system-i386 -cdrom $ISO  -monitor stdio
fi

