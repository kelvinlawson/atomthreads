/*
 * Copyright (c) 2013, Kelvin Lawson. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. No personal names or organizations' names associated with the
 *    Atomthreads project may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE ATOMTHREADS PROJECT AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE PROJECT OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdio.h>
#include <stdarg.h>
#include "atomport.h"
#include "atomport-private.h"
#include "atom.h"
#include "atomport.h"
#include "dm36x-io.h"
#include "uart.h"
 

/** Imports required by C startup code */
extern unsigned long _start_vectors, _end_vectors, _end_text, _start_data, _end_data, _start_bss, _end_bss;
extern int main(void);


/** Timer input clock speed: 24MHz */
#define TIMER_CLK       24000000


/** Register access macros */
#define TIMER0_REG(offset)      *(volatile uint32_t *)(DM36X_TIMER0_BASE + offset)
#define INTC_REG(offset)        *(volatile uint32_t *)(DM36X_INTC_BASE + offset)


/**
 * \b _mainCRTStartup
 *
 * C startup code for environments without a suitable built-in one.
 * May be provided by the compiler toolchain in some cases.
 *
 */
extern void _mainCRTStartup (void) __attribute__((weak));
void _mainCRTStartup(void)
{
    unsigned long *src;
    unsigned long *dst;

    // Copy vector table from SRAM to IRAM0 (ARM vector table must be at 0x00000000)
    src = &_start_vectors;
    dst = (unsigned long *)0x00000000;
    while(src < &_end_vectors)
        *(dst++) = *(src++);

#ifdef ROM
    // Running from ROM: copy data section to RAM
    src = &_end_text;
    dst = &_start_data;
    while(dst < &_end_data)
        *(dst++) = *(src++);
#endif

    // Clear BSS
    src = &_start_bss;
    while(src < &_end_bss)
        *(src++) = 0;

    // Jump to main application entry point
    main();
}


/**
 * \b low_level_init
 *
 * Initializes the PIC and starts the system timer tick interrupt.
 *
 */
int
low_level_init (void)
{
    /* Initialise TIMER0 registers for interrupt 100 times per second */

    /* Reset & disable all TIMER0 timers */
    TIMER0_REG(DM36X_TIMER_INTCTL_STAT) = 0;    /* Disable interrupts */
    TIMER0_REG(DM36X_TIMER_TCR) = 0;            /* Disable all TIMER0 timers */
    TIMER0_REG(DM36X_TIMER_TGCR) = 0;           /* Put all TIMER0 timers in reset */
    TIMER0_REG(DM36X_TIMER_TIM12) = 0;          /* Clear Timer 1:2 */

    /* Set up Timer 1:2 in 32-bit unchained mode */
    TIMER0_REG(DM36X_TIMER_TGCR) = (1 << 2);    /* Select 32-bit unchained mode (TIMMODE) */
    TIMER0_REG(DM36X_TIMER_TGCR) |= (1 << 0);   /* Remove Timer 1:2 from reset (TIM12RS) */
    TIMER0_REG(DM36X_TIMER_PRD12) = (TIMER_CLK / SYSTEM_TICKS_PER_SEC) - 1;     /* Set period to 100 ticks per second (PRD12) */
    TIMER0_REG(DM36X_TIMER_TCR) |= (0 << 8);    /* Select external clock source for Timer 1:2 (CLKSRC12) */

    /* Enable interrupts */
    TIMER0_REG(DM36X_TIMER_INTCTL_STAT) = (1 << 1) | (1 << 0);  /* Enable/ack Compare/Match interrupt for Timer 1:2 */

    /* Enable timer */
    TIMER0_REG(DM36X_TIMER_TCR) |= (2 << 6);    /* Enable Timer 1:2 continuous (ENAMODE12) */

    /* Initialise INTC interrupt controller */
    INTC_REG(DM36X_INTC_INTCTL) = 0;
    INTC_REG(DM36X_INTC_EABASE) = 0;

    /* Ack TINT0 IRQ in INTC interrupt controller */
    INTC_REG(DM36X_INTC_IRQ1) = (1 << (DM36X_INTC_VEC_TINT0 - 32));

    /* Enable TINT0 IRQ in INTC interrupt controller */
    INTC_REG(DM36X_INTC_EINT1) |= (1 << (DM36X_INTC_VEC_TINT0 - 32));

    return 0 ;
}


/**
 * \b __interrupt_dispatcher
 *
 * Interrupt dispatcher: determines the source of the IRQ and calls
 * the appropriate ISR.
 *
 * Currently only the OS system tick ISR is implemented.
 *
 * Note that any ISRs which call Atomthreads OS routines that can
 * cause rescheduling of threads must be surrounded by calls to
 * atomIntEnter() and atomIntExit().
 *
 */
void
__interrupt_dispatcher (void) 
{
    uint32_t vector;

    /* Read IRQENTRY register to determine the source of the interrupt */
    vector = (INTC_REG(DM36X_INTC_IRQENTRY) / 4) - 1;

    /* TIMER0:12 tick interrupt (call Atomthreads timer tick ISR) */
    if (vector == DM36X_INTC_VEC_TINT0)
    {
        /* Reload timer and enable interupts */
        TIMER0_REG(DM36X_TIMER_PRD12) = (TIMER_CLK / SYSTEM_TICKS_PER_SEC) - 1;     /* Set period to 100 ticks per second (PRD12) */
        TIMER0_REG(DM36X_TIMER_INTCTL_STAT) = (1 << 1) | (1 << 0);  /* Enable/ack Compare/Match interrupt for Timer 1:2 */

        /*
         * Let the Atomthreads kernel know we're about to enter an OS-aware
         * interrupt handler which could cause scheduling of threads.
         */
        atomIntEnter();

        /* Call the OS system tick handler */
        atomTimerTick();

        /* Call the interrupt exit routine */
        atomIntExit(TRUE);

        /* Ack the interrupt */
        INTC_REG((vector >= 32) ? DM36X_INTC_IRQ1 : DM36X_INTC_IRQ0) = (1 << (vector >= 32) ? (vector - 32) : vector);
    }

}


/**
 * \b null_handler
 *
 * Handler to catch interrupts at uninitialised vectors.
 *
 */
void null_handler (void) 
{
    uart_write_halt ("Unhandled interrupt\n");
}

