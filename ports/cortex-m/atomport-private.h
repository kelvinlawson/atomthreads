/*
 * Copyright (c) 2015, Tido Klaassen. All rights reserved.
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

#ifndef __ATOMPORT_PRIVATE_H_
#define __ATOMPORT_PRIVATE_H_

#include "atomport.h"
#include "atom.h"

/**
 * context saved automagically by exception entry
 */
struct isr_stack {
        uint32_t r0;
        uint32_t r1;
        uint32_t r2;
        uint32_t r3;
        uint32_t r12;
        uint32_t lr;
        uint32_t pc;
        uint32_t psr;
} __attribute__((packed));

struct isr_fpu_stack {
        uint32_t s0;
        uint32_t s1;
        uint32_t s2;
        uint32_t s3;
        uint32_t s4;
        uint32_t s5;
        uint32_t s6;
        uint32_t s7;
        uint32_t s8;
        uint32_t s9;
        uint32_t s10;
        uint32_t s11;
        uint32_t s12;
        uint32_t s13;
        uint32_t s14;
        uint32_t s15;
        uint32_t fpscr;
} __attribute__((packed));

/**
 *  remaining context saved by task switch ISR
 */
struct task_stack {
        uint32_t r4;
        uint32_t r5;
        uint32_t r6;
        uint32_t r7;
        uint32_t r8;
        uint32_t r9;
        uint32_t r10;
        uint32_t r11;
        uint32_t exc_ret;
} __attribute__((packed));

struct task_fpu_stack {
        uint32_t s16;
        uint32_t s17;
        uint32_t s18;
        uint32_t s19;
        uint32_t s20;
        uint32_t s21;
        uint32_t s22;
        uint32_t s23;
        uint32_t s24;
        uint32_t s25;
        uint32_t s26;
        uint32_t s27;
        uint32_t s28;
        uint32_t s29;
        uint32_t s30;
        uint32_t s31;
} __attribute__((packed));

/**
 * Info needed by pend_sv_handler used for delayed task switching.
 * Running_tcb is a pointer to the TCB currently running (gosh, really?!)
 * next_tcb is a pointer to a TCB that should be running.
 * archContextSwitch() will update next_tcb and trigger a pend_sv. The
 * pend_sv_handler will be called as soon as all other ISRs have returned,
 * do the real context switch and update running_tcb.
 */
struct task_switch_info {
    volatile struct atom_tcb *running_tcb;
    volatile struct atom_tcb *next_tcb;
#if defined(__NEWLIB__)
    struct _reent *reent;
#endif
} __attribute__((packed));

#endif /* __ATOMPORT_PRIVATE_H_ */
