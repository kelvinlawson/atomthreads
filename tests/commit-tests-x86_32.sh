#!/bin/bash

# Quit on any error
set -e
cd ../ports/x86

BIN_DIR=build/run/iso/boot
GRUB_CFG=$BIN_DIR/grub/grub.cfg
ISO=all_tests_multiboot.iso

if [ $GRUB_CFG]; then 
	rm $GRUB_CFG
fi

make clean
make 

cp build/*.bin build/run/iso/boot/

for fn in $BIN_DIR/*.bin; do
	echo "menuentry \"$(basename $fn)\" { multiboot /boot/$(basename $fn) }" >>  $GRUB_CFG
done

if [ $(ls $BIN_DIR/*.bin | wc -l) -gt 0   ];then 
	grub-mkrescue -o  $ISO build/run/iso
	qemu-system-i386 -cdrom $ISO  -monitor stdio -enable-kvm
fi

