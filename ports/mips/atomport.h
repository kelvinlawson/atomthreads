/*
 * Copyright (c) 2011, Himanshu Chauhan. All rights reserved.
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

/**
 * Definition of NULL. stddef.h not available on this platform.
 */
#define NULL ((void *)(0))

/* Size of each stack entry / stack alignment size (32 bits on MIPS) */
#define STACK_ALIGN_SIZE                sizeof(uint32_t)

/**
 * Architecture-specific types.
 * Provide stdint.h style types.
 */
#define uint8_t   unsigned char
#define uint16_t  unsigned short
#define uint32_t  unsigned long
#define uint64_t  unsigned long long
#define int8_t    char
#define int16_t   short
#define int32_t   long
#define int64_t   long long
#define size_t    unsigned long
#define POINTER   void *
#define UINT32    uint32_t


/**
 * Critical region protection: this should disable interrupts
 * to protect OS data structures during modification. It must
 * allow nested calls, which means that interrupts should only
 * be re-enabled when the outer CRITICAL_END() is reached.
 */
extern uint32_t at_preempt_count;
#define CRITICAL_STORE	    uint32_t status_reg
#define CRITICAL_START()					\
	do {							\
		__asm__ __volatile__("di %0\t\n"		\
				     "ehb\t\n"			\
				     :"=r"(status_reg));	\
	}while(0);

#define CRITICAL_END()							\
	do {								\
		__asm__ __volatile__("mtc0 %0, $12\t\n"			\
				     "nop\t\n"				\
				     "ehb\t\n"				\
				     ::"r"(status_reg));		\
	}while(0);

/* Uncomment to enable stack-checking */
/* #define ATOM_STACK_CHECKING */


#endif /* __ATOM_PORT_H */
