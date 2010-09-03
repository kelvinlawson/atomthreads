/*
 * Copyright (c) 2010, Natie van Rooyen. All rights reserved.
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
#ifndef __ATOMVM_H__
#define __ATOMVM_H__

#include <crtdbg.h>
#include "atomuser.h"


#ifdef _DEBUG
#define ATOMVM_ASSERT(x, msg)			_ASSERT(x)
#else
#define ATOMVM_ASSERT(x, msg)
#endif


/* Forward declarations */

/* This is an opaque handle to an instance of an atomvm created
   by a call to atomvmCtrlInit() */
typedef struct ATOMVM*					HATOMVM ; 

/* This is an opaque handle to an atomvm context created
   by a call to atomvmContextCreate() */
typedef struct ATOMVM_CONTEXT*			HATOMVM_CONTEXT ; 


/* Function prototypes used for controlling the atom virtual machine */
extern uint32_t			atomvmCtrlInit (HATOMVM* atomvm) ;
extern void				atomvmCtrlRun (HATOMVM atomvm, uint32_t flags) ;
extern void				atomvmCtrlIntRequest (HATOMVM atomvm, uintptr_t isr) ;
extern void				atomvmCtrlClose (HATOMVM atomvm) ;

/* Function prototypes for use by the atom virtual machine */
extern int32_t			atomvmExitCritical (HATOMVM atomvm) ;
extern int32_t			atomvmEnterCritical (HATOMVM atomvm) ;
extern uint32_t			atomvmContextCreate (HATOMVM atomvm, HATOMVM_CONTEXT* context, uint32_t stack, uint32_t entry) ;
extern uint32_t			atomvmContextSwitch (HATOMVM atomvm, HATOMVM_CONTEXT new_context) ;
extern void				atomvmContextDesrtroy (HATOMVM atomvm, HATOMVM_CONTEXT context) ;

/* Function prototypes to be implemted in the atom virtual machine */
extern  void			__atomvmReset (void) ;
extern  void			__atomvmClose (void) ;

 
#endif /* __ATOMVM_H__ */
