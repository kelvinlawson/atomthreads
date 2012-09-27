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

#ifndef __ATOM_TIMER_H
#define __ATOM_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "atomport.h"


/* Callback function prototype */
typedef void ( * TIMER_CB_FUNC ) ( POINTER cb_data ) ;

/* Data structures */

/* Timer descriptor */
typedef struct atom_timer
{
    TIMER_CB_FUNC   cb_func;    /* Callback function */
    POINTER	        cb_data;    /* Pointer to callback parameter/data */
    uint32_t	    cb_ticks;   /* Ticks until callback */

	/* Internal data */
    struct atom_timer *next_timer;		/* Next timer in doubly-linked list */

} ATOM_TIMER;

/* Function prototypes */

extern uint8_t atomTimerRegister (ATOM_TIMER *timer_ptr);
extern uint8_t atomTimerCancel (ATOM_TIMER *timer_ptr);
extern uint8_t atomTimerDelay (uint32_t ticks);
extern uint32_t atomTimeGet (void);
extern void atomTimeSet (uint32_t new_time);

#ifdef __cplusplus
}
#endif

#endif /* __ATOM_TIMER_H */
