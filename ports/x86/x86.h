/*
 * AtomThreads project - Copyright (c) 2010, Kelvin Lawson. All rights reserved.
 * 
 * Please refer to the README file for further details and License information.
 *
 * x86 port by: ido@mm.st
 */

#ifndef X86_H
#define X86_H

#include <stddef.h>
#include <stdint.h>

typedef struct x86_32_iframe {
    uint32_t di, si, bp, sp, bx, dx, cx, ax; 
    uint32_t ds, es, fs, gs;                        
    uint32_t vector;                                 
    uint32_t err_code;                                
    uint32_t ip, cs, flags;                            
    uint32_t user_sp, user_ss;                          
} x86_iframe_t ;

void gdt_install_flat(void);    
void setup_idt(void);

static inline void out8(uint16_t port, uint8_t value)
{
    __asm__ volatile("outb %[value], %[port]" :: [port] "d"(port), [value] "a"(value));
}

static inline void out16(uint16_t port, uint16_t value)
{
    __asm__ volatile("outw %[value], %[port]" :: [port] "d"(port), [value] "a"(value));
}

static inline uint8_t in8(uint16_t port)
{
    uint8_t value;
    __asm__ volatile("inb %[port], %[value]" : [value] "=a"(value) : [port] "d" (port));
    return value;
}

static inline uint32_t x86_get_eflags(void){
    
    uint32_t flags;

    __asm__ volatile(
        "pushfl;"
        "popl %0"
        : "=rm" (flags)
        :: "memory");

    return flags;
}

static inline void x86_set_eflags(uint32_t flags){
    __asm__ volatile(
        "pushl %0;"
        "popfl"
        :: "g" (flags)
        : "memory", "cc");
}

static inline void x86_enable_int(void){
    __asm__ volatile("sti");
}

static inline void x86_disable_int(void){
    __asm__ volatile("cli");
}

static inline void x86_halt(void){
    __asm__ volatile(
		    "cli;"
		    "hlt");
}
#endif
