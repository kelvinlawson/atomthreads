/*
 * AtomThreads project - Copyright (c) 2010, Kelvin Lawson. All rights reserved.
 * 
 * Please refer to the README file for further details and License information.
 *
 * x86 port by: ido@mm.st
 */

#ifndef PLAT_H
#define PLAT_H

#define IRQ_PIT 0
#define IRQ_PS2 1

#include "x86.h"

void pic_init(void);
void pic_send_EOI(uint32_t);
void pic_disable(void);

void pit_init(uint32_t frequency);
void irq_install_handler(int irq, void (*handler)(void));

#define plat_reboot() out8(0x64, 0xFE)
#define plat_hide_cursor() out16(0x3D4,0x200A)

void irq_register_handler(int irq, void (*handler)(x86_iframe_t*));
void irq_unregister_handler(int irq);

void handle_platform_irq(x86_iframe_t* frame);
void sys_key_handler(x86_iframe_t* frame);
void sys_tick_handler(x86_iframe_t* frame);


extern void terminal_init();
extern void terminal_putchar(char c);
extern void terminal_putchar_color(char c, uint8_t text_color, uint8_t background_color);

#endif
