/*
 * AtomThreads project - Copyright (c) 2010, Kelvin Lawson. All rights reserved.
 * 
 * Please refer to the README file for further details and License information.
 *
 * x86 port by: ido@mm.st
 */

#include "atom.h"
#include "plat.h"
#include "print.h"

static void *irq_routines[16] = {0};

void irq_register_handler(int irq, void (*handler)(x86_iframe_t*)){
    if((NULL == handler) || (irq<0) || (irq>15)) return;
    irq_routines[irq] = handler;
}

void irq_unregister_handler(int irq){
    irq_routines[irq] = 0;
}

void handle_platform_irq(x86_iframe_t* frame){
    void (*handler)(x86_iframe_t* frame);
    uint32_t irq = frame->vector -32;
   
    handler = irq_routines[irq];

    if (handler){
        handler(frame);
	if(irq==IRQ_PIT) return;
    }

    pic_send_EOI(irq);
}

void sys_tick_handler(x86_iframe_t* frame){
    atomIntEnter();
    atomTimerTick();
    pic_send_EOI(IRQ_PIT);
    atomIntExit(TRUE);
}

void sys_key_handler(x86_iframe_t* frame){
	uint8_t scan_code = in8(0x60);
	if(0x90 == scan_code) // Q - pressed
		plat_reboot();
	else if(0x81 <= scan_code  || 0xd3 <= scan_code)
  		print("click @%d\n" , atomTimeGet());
}
