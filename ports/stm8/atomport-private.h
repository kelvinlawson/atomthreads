/*
 * Copyright (c) 2010, Kelvin Lawson. All rights reserved.
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

#ifndef __ATOM_PORT_PRIVATE_H
#define __ATOM_PORT_PRIVATE_H


/**
 * Compiler-specific modifier to prevent some functions from saving
 * and restoring registers on entry and exit, if the function is
 * known to never complete (e.g. thread entry points).
 * Reduces stack usage on supporting compilers.
 */
#ifdef __IAR_SYSTEMS_ICC__
#define NO_REG_SAVE __task
#else
#define NO_REG_SAVE
#endif


/**
 * Compiler-specific modifiers for interrupt handler functions.
 *
 * COSMIC: Uses @interrupt modifier for interrupt handlers. We
 * also force all interrupts to save c_lreg, a separate memory
 * area which Cosmic uses for longs and floats. This memory
 * area must be saved by interrupt handlers for context
 * switch purposes, and to avoid making it impossible to use
 * longs in any OS kernel code accessed by interrupt handlers.
 *
 * IAR: Uses __interrupt modifier for interrupt handlers.
 *
 * RAISONANCE: Uses no prefix modifier, but specifies
 * interrupt vector after (see TIM1_SystemTickISR() for an
 * example).
 */
#if defined(__CSMC__)
#define INTERRUPT @far @interrupt @svlreg
#elif defined (__IAR_SYSTEMS_ICC__)
#define INTERRUPT __interrupt
#elif defined(__RCSTM8__) || defined(__SDCC_stm8)
#define INTERRUPT
#endif


/* Function prototypes */
void archInitSystemTickTimer (void);
#ifndef __SDCC_stm8
INTERRUPT void TIM1_SystemTickISR (void);
#else
void TIM1_SystemTickISR (void) __interrupt(11);
#endif

#endif /* __ATOM_PORT_PRIVATE_H */

