/*
 * Copyright (c) 2012, Natie van Rooyen. All rights reserved.
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

/* Portable uint8_t and friends available from stdint.h on this platform */
#include <stdint.h>

/* Definition of NULL is available from stddef.h on this platform */
#include <stddef.h>

/* Reentrancy structure */
#include <sys/reent.h>

/* Required number of system ticks per second (normally 100 for 10ms tick) */
#define SYSTEM_TICKS_PER_SEC            100

/* Size of each stack entry / stack alignment size (32 bits on this platform) */
#define STACK_ALIGN_SIZE                sizeof(uint32_t)

/**
 * Architecture-specific types.
 * Most of these are available from stdint.h on this platform, which is
 * included above.
 */
#define POINTER void *

/**
 * Architecture-specific definition of atom event size.
 * It is best selected as the size of the architecture, but can be
 * reduced or increased depending on requirements.
 */
typedef uint32_t ATOM_EVENTS;

/**
 * Hardware timer functions (optional, not available on all ports)
 */
extern void archUsleep (int32_t microsecs);
extern int32_t archUsleepStart (void);
extern int archUsleepCheckExpired (int32_t start_time, int32_t delay_usecs);
extern int32_t archUsecStart (void);
extern int32_t archUsecDiff (int32_t start_time);

/**
 * ISR handler registration (optional, not available on all ports)
 */
typedef void (*ISR_FUNC)(int int_vector);
extern int archIntInstallISR (int int_vector, ISR_FUNC isr_func);
extern int archIntEnable (int int_vector, int enable);

/**
 *
 * Functions defined in atomport_arm.asm
 *
 */
extern uint32_t  contextEnterCritical (void) ;
extern void      contextExitCritical (uint32_t posture) ;

/**
 * Critical region protection: this should disable interrupts
 * to protect OS data structures during modification. It must
 * allow nested calls, which means that interrupts should only
 * be re-enabled when the outer CRITICAL_END() is reached.
 */
#define CRITICAL_STORE          uint32_t __atom_critical
#define CRITICAL_START()        __atom_critical = contextEnterCritical()
#define CRITICAL_END()          contextExitCritical(__atom_critical)

/**
 * When using newlib, define port private field in atom_tcb to be a
 * struct _reent.
 */
struct arm_port_priv {
    struct _reent reent;
};
#define THREAD_PORT_PRIV    struct arm_port_priv port_priv

/* Uncomment to enable stack-checking */
/* #define ATOM_STACK_CHECKING */


#endif /* __ATOM_PORT_H */
