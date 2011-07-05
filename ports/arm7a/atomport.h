/*
 * Copyright (c) 2011, Anup Patel. All rights reserved.
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

#ifndef __ATOM_PORT_H
#define __ATOM_PORT_H

/* Required number of system ticks per second (normally 100 for 10ms tick) */
#define SYSTEM_TICKS_PER_SEC            100

typedef signed int int32_t;
typedef signed short int16_t;
typedef signed char int8_t;
typedef unsigned int uint32_t;
typedef unsigned short uint16_t;
typedef unsigned char uint8_t;
typedef long long int64_t;
typedef unsigned long size_t;

typedef unsigned int irq_flags_t;
typedef unsigned int virtual_addr_t;
typedef unsigned int virtual_size_t;
typedef unsigned int physical_addr_t;
typedef unsigned int physical_size_t;
typedef unsigned int clock_freq_t;
typedef unsigned long long jiffies_t;

#define UINT32 uint32_t
#define STACK_ALIGN_SIZE sizeof(uint32_t)
#define NULL ((void *)(0))

/**
 * Architecture-specific types.
 * Most of these are available from stdint.h on this platform, which is
 * included above.
 */
#define POINTER void *

struct pt_regs {
	uint32_t cpsr;	// Current Program Status
	uint32_t gpr[13];	// R0 - R12
	uint32_t sp;
	uint32_t lr;
	uint32_t pc;
} __attribute ((packed)) ;
typedef struct pt_regs pt_regs_t;

#include <printk.h>
#include <arm_irq.h>

/* Critical region protection */
#define CRITICAL_STORE		irq_flags_t status_flags
#define CRITICAL_START()	status_flags = arm_irq_save();
#define CRITICAL_END()		arm_irq_restore(status_flags);

/* Uncomment to enable stack-checking */
/* #define ATOM_STACK_CHECKING */

#endif /* __ATOM_PORT_H */
