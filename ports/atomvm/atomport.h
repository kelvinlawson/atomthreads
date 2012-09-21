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

#ifndef __ATOM_PORT_H__
#define __ATOM_PORT_H__

#include "atomvm.h"

#define SYSTEM_MEMALIGN		sizeof (unsigned int)
#define SYSTEM_TICKS_PER_SEC            100


typedef unsigned int		uintptr_t ;
typedef int					intptr_t ;
typedef unsigned int		uint32_t ;
typedef unsigned short		uint16_t ;
typedef unsigned char		uint8_t ;
typedef int					int32_t ;
typedef short				int16_t ;
typedef char				int8_t ;


/**
 * Architecture-specific types.
 * Most of these are available from stdint.h on this platform, which is
 * included above.
 */
#define POINTER				void *

 
/* Critical region protection */
extern  unsigned int		__enter_critical() ;
extern void					__exit_critical(unsigned int) ;

#define CRITICAL_STORE		unsigned int __atom   
#define CRITICAL_START()	 __atom = __enter_critical()
#define CRITICAL_END()		 __exit_critical(__atom)

#define ATOM_TLS			HATOMVM_CONTEXT	context ;

/* Function prototypes */
extern void					atomvmRun () ;
extern void					archTimerTickIrqHandler () ;

/* The instance of the atomvm for this port */
extern HATOMVM				the_atomvm  ;

#endif /* __ATOM_PORT_H */
