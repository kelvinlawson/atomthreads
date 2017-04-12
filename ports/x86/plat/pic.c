/*
 * AtomThreads project - Copyright (c) 2010, Kelvin Lawson. All rights reserved.
 * 
 * Please refer to the README file for further details and License information.
 *
 * x86 port by: ido@mm.st
 */

#include "x86.h"

#define PIC_MASTER_REG 0x20
#define PIC_MASTER_IMR 0x21
#define PIC_SLAVE_REG 0xA0
#define PIC_SLAVE_IMR 0xA1

#define PIC_ICW_1 0x11

void pic_init(void){
    // ICW #1
    out8(PIC_MASTER_REG, PIC_ICW_1);
    out8(PIC_SLAVE_REG, PIC_ICW_1);

    // ICW #2 
    out8(PIC_MASTER_IMR, 0x20); // remapping IRQ0-IRQ7 to start from 0x20, i.e 32'th ISR
    out8(PIC_SLAVE_IMR, 0x28); // remapping IRQ8-IRQ15 to start from 0x28, i.e 32'th ISR
    
    // ICW #3 - MASTER/SLAVE coordination
    out8(PIC_MASTER_IMR, 0x4);
    out8(PIC_SLAVE_IMR, 0x2);

    // ICW #4 - set 80x86 mode
    out8(PIC_MASTER_IMR, 0x01);
    out8(PIC_SLAVE_IMR, 0x01);

    out8(PIC_MASTER_IMR, 0x0);
    out8(PIC_SLAVE_IMR, 0x0);
}

void pic_disable(void){
    out8(PIC_MASTER_IMR, 0xff);
    out8(PIC_SLAVE_IMR, 0xff);
}

void pic_send_EOI(uint32_t irq){
    if (40 <= irq)
    	out8(PIC_SLAVE_REG, 0x20);

    out8(PIC_MASTER_REG, 0x20);
}
