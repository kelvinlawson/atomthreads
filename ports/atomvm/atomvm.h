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



/** \mainpage \ref atomvm
*   \defgroup atomvm Atomvm API
*
* @authors Natie van Rooyen
*
* @section intro Introduction
* Atomvm is a tiny virtual machine that runs on Windows and can be debugged 
* from an IDE like Microsoft Visual C++ Express. The primary purpose of this 
* virtual machine is for the evaluation of Real Time Operating Systems (like 
* Atomthreads) and the development and testing of modules for this Real Time 
* Operating System on a Windows machine.
* 
* Atomvm makes use of the Windows API functions GetThreadContext() and 
* SetThreadContext() to create multiple virtual contexts or threads inside a 
* single Windows thread. Atomvm also simulates interrupts with an interrupt 
* mask accessible from the Atomvm threads. External events can be queued as 
* interrupts to Atomvm, for example a timer loop generating system timer tick 
* interrupts for a Real Time Operating System ported to Atomvm.
* 
* */

#ifndef __ATOMVM_H__
#define __ATOMVM_H__

#include <crtdbg.h>
#include "types.h"


#if defined _DEBUG || defined DEBUG
#define ATOMVM_ASSERT(x, msg)           _ASSERT((x))
#else
#define ATOMVM_ASSERT(x, msg)
#endif


#define ATOMVM_MAX_VM           8

/* Forward declarations */

/* This is an opaque handle to an instance of an atomvm created
   by a call to atomvmCtrlCreate() */
typedef struct ATOMVM*                  HATOMVM ; 

/* This is an opaque handle to an atomvm context created
   by a call to atomvmContextCreate() */
typedef struct ATOMVM_CONTEXT*          HATOMVM_CONTEXT ; 


/* Function prototypes used for controlling the atom virtual machine */
extern uint32_t         atomvmCtrlCreate (HATOMVM* atomvm) ;
extern void             atomvmCtrlRun (HATOMVM atomvm, uint32_t flags) ;
extern void             atomvmCtrlIntRequest (HATOMVM atomvm, uintptr_t isr) ;
extern void             atomvmCtrlClose (HATOMVM atomvm) ;

/* Function prototypes for use by the atom virtual machine from within the
   call to __atomvmReset(). */
extern int32_t          atomvmInterruptMask (uint32_t mask) ;
extern uint32_t         atomvmContextCreate (HATOMVM_CONTEXT* context, uint32_t stack, uint32_t entry) ;
extern uint32_t         atomvmContextSwitch (HATOMVM_CONTEXT old_context, HATOMVM_CONTEXT new_context) ;
extern void             atomvmContextDesrtroy (HATOMVM_CONTEXT context) ;
extern void             atomvmWriteThreadId (uint32_t thread_id) ;
extern uint32_t         atomvmReadThreadId (void) ;
extern void             atomvmInterruptWait (void) ;
extern uint32_t         atomvmGetVmId (void) ;


/**
* \ingroup atomvm
* \b __atomvmReset
*
* Function prototype to be implemted as entry point for the atom virtual machine.
*
* @return void.
*/
extern  void            __atomvmReset (void) ;

/**
* \ingroup atomvm
* \b __atomvmClose
*
* Function prototype to be implemted in the atom virtual machine
*
* @return void.
*/
extern  void            __atomvmClose (void) ;

#endif /* __ATOMVM_H__ */
