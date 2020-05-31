/*
 * Copyright (c) 2010, Kelvin Lawson. All rights reserved.
 * Copyright (c) 2016, Dr. Philipp Klaus Krause.
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


#include "stm8s_type.h"

#if defined(__IAR_SYSTEMS_ICC__)
#include "intrinsics.h"
#elif defined (__RCSTM8__)
#include <intrins.h>
#endif

/* Definition of NULL is available from stddef.h on this platform */
#include <stddef.h>

/* Required number of system ticks per second (normally 100 for 10ms tick) */
#define SYSTEM_TICKS_PER_SEC            100

/**
 * Architecture-specific types.
 */
#if defined(__CSMC__) || defined (__RCSTM8__) /* Cosmic and Raisonance do not have the C99 stdint.h header*/
#define int8_t   s8
#define int16_t  s16
#define int32_t  s32
#define uint8_t  u8
#define uint16_t u16
#define uint32_t u32
#else
#include <stdint.h>
#endif
#define POINTER  void *

/* Size of each stack entry / stack alignment size (8 bits on STM8) */
#define STACK_ALIGN_SIZE                sizeof(uint8_t)

/**
 * Critical region protection: this should disable interrupts
 * to protect OS data structures during modification. It must
 * allow nested calls, which means that interrupts should only
 * be re-enabled when the outer CRITICAL_END() is reached.
 */

/* COSMIC: Use inline assembler */
#if defined(__CSMC__)
#define CRITICAL_STORE      uint8_t ccr
#define CRITICAL_START()    _asm ("push CC\npop a\nld (X),A\nsim", &ccr)
#define CRITICAL_END()      _asm ("ld A,(X)\npush A\npop CC", &ccr)

/* IAR: Use intrinsics */
#elif defined(__IAR_SYSTEMS_ICC__)
#define CRITICAL_STORE      __istate_t _istate
#define CRITICAL_START()    _istate = __get_interrupt_state(); __disable_interrupt()
#define CRITICAL_END()      __set_interrupt_state(_istate)

/* Raisonance: Use intrinsics */
#elif defined(__RCSTM8__)
#define CRITICAL_STORE      unsigned char ccr
#define CRITICAL_START()    ccr = _getCC_(); _sim_()
#define CRITICAL_END()      _setCC_(ccr)

/* SDCC: Use custom function */
#elif defined(__SDCC_stm8)
uint8_t get_cc(void);
void set_cc(uint8_t);
#define CRITICAL_STORE      uint8_t ccr
#define CRITICAL_START()    ccr = get_cc(); __asm__("sim")
#define CRITICAL_END()      set_cc(ccr)
#endif

/* Uncomment to enable stack-checking */
/* #define ATOM_STACK_CHECKING */


#endif /* __ATOM_PORT_H */
