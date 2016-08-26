/*
 * AtomThreads project - Copyright (c) 2010, Kelvin Lawson. All rights reserved.
 * 
 * Please refer to the README file for further details and License information.
 *
 * x86 port by: ido@mm.st
 */

#include "plat.h"
#include "print.h"

char *exception_messages[] =
{
    "Division By Zero",
    "Debug",
    "Non Maskable Interrupt",
    "Breakpoint",
    "Into Detected Overflow",
    "Out of Bounds",
    "Invalid Opcode",
    "No Coprocessor",

    "Double Fault",
    "Coprocessor Segment Overrun",
    "Bad TSS",
    "Segment Not Present",
    "Stack Fault",
    "General Protection Fault",
    "Page Fault",
    "Unknown Interrupt",

    "Coprocessor Fault",
    "Alignment Check",
    "Machine Check",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",

    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved"
};

void handle_exception(uint32_t vector)
{
    print("Exception raised: ");
    print(exception_messages[vector]);

    x86_halt();
}

void handle_unknown(void){
	handle_exception(16);
}

void  x86_exception_handler(x86_iframe_t* iframe){

    uint32_t vector = iframe->vector;

    if(31 >= vector)
	handle_exception(vector);
    else if(47 >= vector)
	handle_platform_irq(iframe);
    else 
	handle_unknown();
}
