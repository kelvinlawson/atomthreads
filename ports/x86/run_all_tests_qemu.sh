#!/bin/bash

BIN_DIR=build/run/iso/boot
GRUB_CFG=$BIN_DIR/grub/grub.cfg
ISO=all_tests_multiboot.iso

rm  $BIN_DIR/*.bin
rm $GRUB_CFG

make 

cp build/*.bin build/run/iso/boot/

for fn in $BIN_DIR/*.bin; do
	echo "menuentry \"$(basename $fn)\" { multiboot /boot/$(basename $fn) }" >>  $GRUB_CFG
done

if [ $(ls $BIN_DIR/*.bin | wc -l) -gt 0   ];then 
	grub-mkrescue -o  $ISO build/run/iso
	qemu-system-i386 -cdrom $ISO  -monitor stdio #-enable-kvm
fi

