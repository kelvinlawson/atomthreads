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
* Atomvm is a tiny virtual machine that can run on Windows inside an IDE with a 
* debugger like Microsoft Visual C++ Express. The primary purpose of this virtual 
* machine is for the evaluation of Real Time Operating Systems (like atomthreads) 
* and the development and testing of modules for this Real Time Operating System
* in a user friendly environment.:
* 
* @section build Building The Source
* To test this project, just add all the files from the "atomthreads/kernel" 
* directory and the "atomthreads/ports/atomvm" directory as well as the test 
* program main.c to your project. Add both the 
* before mentioned directories to the include paths of your project and compile. \n
* Atomvm was designed for multi core systems but also runs fine on any single 
* core system.
*
* @section test Running The Test
* The test, main.c, is intentioned to stress the virtual  machine as opposed to 
* testing the Real Time Operating System. However, this test can also run the 
* unit tests of atomthreads by using the preprocessor directive "UNIT_TESTS" and 
* linking in the desired unit test into the project.
* */

#ifndef __ATOMVM_H__
#define __ATOMVM_H__

#include <crtdbg.h>
#include "atomuser.h"


#if defined _DEBUG || defined DEBUG
#define ATOMVM_ASSERT(x, msg)           _ASSERT(x)
#else
#define ATOMVM_ASSERT(x, msg)
#endif


#define ATOMVM_MAX_VM           8

/* Forward declarations */

/* This is an opaque handle to an instance of an atomvm created
   by a call to atomvmCtrlInit() */
typedef struct ATOMVM*                  HATOMVM ; 

/* This is an opaque handle to an atomvm context created
   by a call to atomvmContextCreate() */
typedef struct ATOMVM_CONTEXT*          HATOMVM_CONTEXT ; 


/* Function prototypes used for controlling the atom virtual machine */
extern uint32_t         atomvmCtrlInit (HATOMVM* atomvm) ;
extern void             atomvmCtrlRun (HATOMVM atomvm, uint32_t flags) ;
extern void             atomvmCtrlIntRequest (HATOMVM atomvm, uintptr_t isr) ;
extern void             atomvmCtrlClose (HATOMVM atomvm) ;

/* Function prototypes for use by the atom virtual machine */
extern int32_t          atomvmExitCritical () ;
extern int32_t          atomvmEnterCritical () ;
extern int32_t          atomvmCriticalCount () ;
extern uint32_t         atomvmContextCreate (HATOMVM_CONTEXT* context, uint32_t stack, uint32_t entry) ;
extern uint32_t         atomvmContextSwitch (HATOMVM_CONTEXT old_context, HATOMVM_CONTEXT new_context) ;
extern void             atomvmContextDesrtroy (HATOMVM_CONTEXT context) ;
extern void             atomvmWriteThreadId (uint32_t thread_id) ;
extern uint32_t         atomvmReadThreadId () ;
/* Function prototypes for use by the atom virtual machine 
    for synchronization with other running atom virtual machines */
extern uint32_t         atomvmGetVmId () ;
extern void             atomvmInterruptWait () ;
extern void             atomvmEventWait () ;
extern void             atomvmEventSend () ;
extern uint32_t         atomvmScheduleIpi (uint32_t target, uintptr_t isr) ;




/**
* \ingroup atomvm
* \b __atomvmReset
*
* Function prototype to be implemted in the atom virtual machine
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
