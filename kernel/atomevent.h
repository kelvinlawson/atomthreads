/*
 * Copyright (c) 2017, jinsong yu, ramaxel, All rights reserved.
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

#ifndef __ATOM_EVENT_H
#define __ATOM_EVENT_H


#define ATOM_NO_WAIT          0
#define ATOM_WAIT_FOREVER     0xFFFFFFFFUL
#define ATOM_AND              2
#define ATOM_AND_CLEAR        3
#define ATOM_OR               0
#define ATOM_OR_CLEAR         1



#define ATOM_EVENT_AND_MASK                 0x2
#define ATOM_EVENT_CLEAR_MASK               0x1



#define ATOM_NO_EVENTS                    0x07 //located it here temp.. place it in atom.h finnally.

/*ATOM_EVENT_FLAGS_GROUP * _ATOM_event_flags_created_ptr;*/


#ifdef __cplusplus
extern "C" {
#endif


/* Define the event flags group structure utilized by the application.  */

typedef struct atom_event
{
	uint32_t	atom_event_current;
    ATOM_TCB *  suspQ;  /* Queue of threads suspended on this event */

} ATOM_EVENT;



extern uint8_t  atomEventCreate(ATOM_EVENT *event_ptr, char *name_ptr);
extern uint8_t atomEventDelete (ATOM_EVENT *event_ptr);
extern uint8_t  atomEventGet(ATOM_EVENT *event_ptr, uint32_t requested_flags,\
        uint8_t get_option, uint32_t *actual_flags_ptr, uint32_t timeout);
extern uint8_t  atomEventSet(ATOM_EVENT *event_ptr, uint32_t flags_to_set, uint8_t set_option);

#ifdef __cplusplus
}
#endif



#endif /* __ATOM_EVENT_H */
