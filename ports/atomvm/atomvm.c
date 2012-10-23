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

/**
 * \file
 * Atom Virtual Machine.
 *
 *
 * This module implements the virtual machine.
 *
 *
 * \b Functions contained in this module:\n
 *
 * \b Function prototypes used for controlling the atom virtual machine: \n
 *
 * \li atomvmCtrlCreate(): .
 * \li atomvmCtrlRun(): .
 * \li atomvmCtrlIntRequest(): .
 * \li atomvmCtrlClose(): .
 *
 * \b Function prototypes for use by the atom virtual machine: \n
 *
 * \li atomvmInterruptMask(): .
 * \li atomvmContextCreate(): .
 * \li atomvmContextSwitch(): .
 * \li atomvmContextDesrtroy(): .
 * \li atomvmWriteThreadId(): .
 * \li atomvmReadThreadId(): .
 * \li atomvmInterruptWait(): .
 * \li atomvmGetVmId(): .
 *
 * \b Function prototypes to be implemted in the atom virtual machine: \n
 *
 * \li __atomvmReset(): .
 * \li __atomvmClose(): .
 *
 */

#include "atomvm.h"
#include <string.h>
#include <windows.h>



#define CONTEXT_VM                                          (CONTEXT_INTEGER | CONTEXT_CONTROL | CONTEXT_SEGMENTS)

/* Data types */

/* Forward declarations */
typedef struct ATOMVM_S *                   PATOMVM ;
typedef struct ATOMVM_CALLBACK_S *          PATOMVM_CALLBACK ;
typedef struct ATOMVM_CONTEXT_S *           PATOMVM_CONTEXT ;

typedef  uint32_t (*ATOMVM_CALLBACK_F) (PATOMVM, PATOMVM_CALLBACK) ;

typedef struct ATOMVM_CALLBACK_S {

    /* Address of callback function */
    volatile ATOMVM_CALLBACK_F      callback ;

    /* Synchronization lock, the virtual machine will be suspended during
    the callback. Regular WIN32 synchronization methods cant be used
    because SuspendThread() is used on the vm thread. */
    volatile uint32_t               lock ;

    /* Result of the call */
    volatile uint32_t               result ;

} ATOMVM_CALLBACK, *PATOMVM_CALLBACK ;


/* ATOMVM_CALLBACK_CONTEXT is the parameter for a ATOMVM_CALLBACK_F call
that take as parameter a pointer to a ATOMVM_CONTEXT to operate on */
typedef struct ATOMVM_CALLBACK_CONTEXT_S {

    ATOMVM_CALLBACK                 callback ;

    /* Context the callback function will operate on */
    volatile PATOMVM_CONTEXT        pcontext ;

} ATOMVM_CALLBACK_CONTEXT, *PATOMVM_CALLBACK_CONTEXT ;


/* ATOMVM_CALLBACK_CONTEXT_SWITCH is the parameter for a ATOMVM_CALLBACK_F call
that take as parameter a pointer to a ATOMVM_CONTEXT to operate on */
typedef struct ATOMVM_CALLBACK_CONTEXT_SWITCH_S {

    ATOMVM_CALLBACK                 callback ;

    /* Context the callback function will operate on */
    volatile PATOMVM_CONTEXT        p_old_context ;
    volatile PATOMVM_CONTEXT        p_new_context ;

} ATOMVM_CALLBACK_CONTEXT_SWITCH, *PATOMVM_CALLBACK_CONTEXT_SWITCH ;

/* ATOMVM_CALLBACK_INT_REQUEST is the parameter for a ATOMVM_CALLBACK_F call
that take as parameter a pointer to to the function that will be called in
an interrupt context */
typedef struct ATOMVM_CALLBACK_INT_REQUEST_S {

    ATOMVM_CALLBACK                 callback ;

    /* Function pointer the callback will call */
    void (*isr) (void) ;

} ATOMVM_CALLBACK_INT_REQUEST, *PATOMVM_CALLBACK_INT_REQUEST ;

/* ATOMVM_CONTEXT saves the state of a context created by
atomvmContextCreate() and sheduled by atomvmContextSwitch(). */
typedef struct ATOMVM_CONTEXT_S {

    /* A virtual machine thread context. These are saved and restored
    during context initialization and context switches */
    CONTEXT                         context ;

    /* When entering a critical section the interrupt_mask is
    set for the context. Interrupts will only occur while
    the interrupt_mask is zero. */
    volatile uint32_t               interrupt_mask ;
    uint32_t                        thread_id ;

} ATOMVM_CONTEXT, *PATOMVM_CONTEXT ;

/* ATOMVM defines the state of an instance to an atomvm. It is created
by a call to atomvmCtrlCreate(). */
typedef struct ATOMVM_S {

    uint32_t                        atomvm_id ;

    /* Thread the virtual machine will run in */
    HANDLE                          vm_thread ;

    /* Handles to events and mutexes used for synchronization */
    HANDLE                          atomvm_call ;
    HANDLE                          atomvm_int ;
    HANDLE                          atomvm_int_complete ;
    HANDLE                          atomvm_close ;

    /* next ISR */
    volatile void                   (*isr)(void) ;
    /* True if in an ISR */
    volatile uint32_t               status_isr ;

    /* The current context that was scheduled by a call
    to atomvmContextSwitch() */
    PATOMVM_CONTEXT                 current_context ;

    /* Service call address, synchronization lock, parameters
    and, return value for the current service call */
    PATOMVM_CALLBACK                service_call ;

    /* Context for startup, before any context was scheduled */
    ATOMVM_CONTEXT                  atom_init_context ;

} ATOMVM, *PATOMVM ;


/* Global declarations */
volatile uint32_t                   g_atomvm_id = 0 ;
volatile DWORD                      g_atomvm_tls_idx ;


/* Forward declaration for the atom virtual machine thread */
static DWORD WINAPI                 vm_thread (LPVOID lpParameter) ;


/**
* \ingroup atomvm
* \b atomvmCtrlCreate
*
* This is an atomvm controll function used by a controlling thread.
*
* Initializes the virtual machine.
*
* @param[out] atomvm Handle to the virtual machine to create.
*
* @return Zero on failure.
*/
uint32_t
atomvmCtrlCreate (HATOMVM *atomvm)
{
    PATOMVM patomvm = 0 ;

    patomvm = (PATOMVM) malloc (sizeof(struct ATOMVM_S)) ;

    if (patomvm) {

        memset (patomvm, 0, sizeof(struct ATOMVM_S)) ;

        patomvm->atomvm_id = InterlockedIncrement(&g_atomvm_id) - 1 ;

        if (patomvm->atomvm_id == 0) {
            g_atomvm_tls_idx = TlsAlloc () ;
        }

        patomvm->atomvm_call = CreateEvent (NULL, TRUE, FALSE, 0) ;
        patomvm->atomvm_int = CreateEvent (NULL, TRUE, FALSE, 0) ;
        patomvm->atomvm_int_complete = CreateEvent (NULL, FALSE, TRUE, 0) ;
        patomvm->atomvm_close = CreateEvent (NULL, TRUE, FALSE, 0) ;

        ATOMVM_ASSERT(patomvm->atomvm_call && patomvm->atomvm_int && patomvm->atomvm_int_complete && 
                patomvm->atomvm_close, _T("ResumeThread failed")) ;

        patomvm->vm_thread = CreateThread (NULL, 0, vm_thread, (void*)patomvm, CREATE_SUSPENDED, NULL) ;

        ATOMVM_ASSERT(patomvm->vm_thread, _T("CreateThread failed")) ;

        patomvm->atom_init_context.interrupt_mask = 1 ;
        patomvm->current_context = &patomvm->atom_init_context ;

        *atomvm = (HATOMVM)patomvm ;

    }

    return patomvm != 0 ;
}


/**
* \ingroup atomvm
* \b atomvmCtrlRun
*
* After a call to atomvmCtrlCreate this function start the atom virtual machine.
* The calling thread will be used to manage interrupts and service calls in
* the virtual machine. This function will not return untill atomvmCtrlClose
* is called.
*
* @param[in] atomvm Handle to the virtual machine created by atomvmCtrlCreate.
* @param[in] flags not used.
*
* @return None
*/
void
atomvmCtrlRun (HATOMVM atomvm, uint32_t flags)
{
    PATOMVM             patomvm = (PATOMVM) atomvm ;
    HANDLE              wait[3] ;
    uint32_t            res ;
    uint32_t            wait_object ;
    PATOMVM_CALLBACK    service_call ;
#if defined DEBUG || defined _DEBUG
    BOOL                tls_res = 
#endif
    TlsSetValue (g_atomvm_tls_idx, (void*) atomvm) ;
    ATOMVM_ASSERT(tls_res, _T("TlsSetValue failed")) ;

    ResumeThread (patomvm->vm_thread) ;

    wait[0] = patomvm->atomvm_call ;
    wait[1] = patomvm->atomvm_int ;
    wait[2] = patomvm->atomvm_close ;

    for(;;) {

        wait_object = WaitForMultipleObjects (3, wait,FALSE,INFINITE) ;

        if (wait_object == WAIT_OBJECT_0) {

            service_call = patomvm->service_call ;
            while (!service_call->lock) {
                SwitchToThread () ;
            }

            while ((res = SuspendThread (patomvm->vm_thread)) == (DWORD)-1) ;
            ATOMVM_ASSERT(res == 0 , _T("SuspendThread failed")) ;
#if (_WIN32_WINNT >= 0x0600)
            /*
                This is used for multi processor machines to ensure the thread
                is stopped before executing the next instruction. Set
                _WIN32_WINNT < 0x0600 if you are running Windows XP */
            FlushProcessWriteBuffers ();
#endif
            InterlockedExchange (&service_call->result, service_call->callback (patomvm, service_call)) ;
            InterlockedExchange (&service_call->lock, 0) ;
            ResetEvent (patomvm->atomvm_call) ;
            res = ResumeThread (patomvm->vm_thread) ;
            ATOMVM_ASSERT(res == 1 , _T("ResumeThread failed")) ;

        }

        else if (wait_object == WAIT_OBJECT_0 + 1) {

            if (patomvm->current_context->interrupt_mask == 0) {

                while ((res = SuspendThread (patomvm->vm_thread)) == (DWORD)-1) ;
                ATOMVM_ASSERT(res == 0 , _T("SuspendThread failed")) ;
#if (_WIN32_WINNT >= 0x0600)
                /*
                    This is used for multi processor machines to ensure the thread
                    is stopped before executing the next instruction. Set
                    _WIN32_WINNT < 0x0600 if you are running Windows XP */
                FlushProcessWriteBuffers ();
#endif
                if (patomvm->current_context->interrupt_mask == 0) {

                    patomvm->status_isr++ ;
                    patomvm->isr () ;
                    patomvm->status_isr-- ;

                    res = ResumeThread (patomvm->vm_thread) ;
                    ATOMVM_ASSERT(res == 1 , _T("ResumeThread failed")) ;
                    
                    ResetEvent (patomvm->atomvm_int) ;
                    InterlockedExchange ((volatile uint32_t*)&patomvm->isr, 0) ;
                    SetEvent (patomvm->atomvm_int_complete) ;

                } else {

                    res = ResumeThread (patomvm->vm_thread) ;
                    ATOMVM_ASSERT(res == 1 , _T("ResumeThread failed")) ;
                    SwitchToThread () ;

                }

            } else {

                SwitchToThread () ;

            }

        } else if (wait_object == WAIT_OBJECT_0 + 2) {

            break ;

        } else {

            ATOMVM_ASSERT(res == 1 , _T("WaitForMultipleObjects failed")) ;

        }

    }

}


/**
* \ingroup atomvm
* \b atomvmCtrlClose
*
* This is an atomvm controll function used by a controlling thread
* and must not be called from the atom virtual machine.
*
* Closes the virtual machine and release all memory and handles created
* in atomvmCtrlCreate.
*
* ToDo: more testing.
*
* @param[in] atomvm Handle to the virtual machine created by atomvmCtrlCreate.
*
* @return None
*/
void
atomvmCtrlClose (HATOMVM atomvm)
{
    PATOMVM     patomvm = (PATOMVM) atomvm ;
    DWORD       code ;

    __atomvmClose () ;

    SetEvent (patomvm->atomvm_close) ;
    do {
        SwitchToThread () ;
        GetExitCodeThread (patomvm->vm_thread, &code) ;
    }	while (code == STILL_ACTIVE) ;

    CloseHandle (patomvm->atomvm_call) ;
    CloseHandle (patomvm->atomvm_int) ;
    CloseHandle (patomvm->atomvm_int_complete) ;
    CloseHandle (patomvm->atomvm_close) ;
    CloseHandle (patomvm->vm_thread) ;

    // TlsFree (g_atomvm_tls_idx) ;

    free (atomvm) ;
}


/**
* \b invokeCallback
*
* Invokes callback functions in the context of the controll thread as
* requested from the virtual machine. In case this callback came from inside,
* an isr it is already in the conrtext of the controll thread and the callback
* routine is called directly.
*
* The atom virtual machine thread is suspended during the callback.
*
* @param[in] patomvm Pointer to the virtual machine created by atomvmCtrlCreate.
* @param[in] callback Callback function.
* @param[in/out] context Context the function will operate on.
*
* @return Zero on failure, try to call GetLastError().
*/
uint32_t
invokeCallback (PATOMVM patomvm, ATOMVM_CALLBACK_F callback, PATOMVM_CALLBACK service_call)
{
    uint32_t res ;

    if (patomvm->status_isr == 0) {

        service_call->lock = 0 ;
        service_call->callback = callback ;
        patomvm->service_call = service_call ;

        SetEvent (patomvm->atomvm_call) ;
        InterlockedIncrement (&service_call->lock) ;
        while (service_call->lock != 0) ;
        res = service_call->result ;

    } else {

        res = callback (patomvm, service_call) ;

    }

    return res ;
}


/*
* \b getAtomvm
*
* Get the atomvm instance for the calling thread
*
* @return atomvm instance
*/
__inline PATOMVM
getAtomvm (void)
{
    PATOMVM patomvm = (PATOMVM) TlsGetValue (g_atomvm_tls_idx) ;
    
    ATOMVM_ASSERT(patomvm , _T("TlsGetValue failed")) ;

    return patomvm ;
}

/**
* \ingroup atomvm
* \b atomvmInterruptMask
*
* This function is to be used by the atom virtual machine.
*
* This function will mask interrupts for the current atomvm context.
*
* @param[in] mask zero enables interrupts any other value masks interrupts.
*
* @return Interrupt mask before the function call.
*/
int32_t
atomvmInterruptMask (uint32_t mask)
{
    PATOMVM         patomvm = getAtomvm () ;
    int32_t         interrupts = 0;

    if (patomvm->status_isr == 0) {
        interrupts = InterlockedExchange (&patomvm->current_context->interrupt_mask, mask) ;
    }

    return interrupts ;
}


/**
* \ingroup atomvm
* \b atomvmCtrlIntRequest
*
* This is an atomvm controll function used by external threads
* and must not be called from the atom virtual machine.
*
* This function requests an interrupt service routine to be called in the
* context of the atom virtual machine.
*
* The call will return immediately after the interrupt was scheduled.
* The call will block while a previously scheduled interrupt is in progress.
*
* @param[in] atomvm Handle to the virtual machine created by atomvmCtrlCreate.
* @param[in] isr The address of the interrupt service routine. 
*
* @return None
*/
void
atomvmCtrlIntRequest (HATOMVM atomvm, void (*isr) (void))
{
    PATOMVM         patomvm = (PATOMVM) atomvm ;

    WaitForSingleObject (patomvm->atomvm_int_complete, INFINITE) ;
    while (InterlockedCompareExchange ((volatile uint32_t *)&patomvm->isr, (uint32_t)isr, 0) != 0) {
		SwitchToThread() ;
	}
    SetEvent (patomvm->atomvm_int) ;
}


/**
* \b callbackContextCreate
*
* This function is invoked from the controll thread after a call to atomvmContextCreate.
*
* The atom virtual machine is suspended while this function is called.
*
* @param[in] patomvm Pointer to the virtual machine created by atomvmCtrlCreate.
* @param[out] context Context to be initialized.
*
* @return Zero on failure, try to call GetLastError().
*/
uint32_t
callbackContextCreate (PATOMVM patomvm, PATOMVM_CALLBACK callback)
{
    PATOMVM_CALLBACK_CONTEXT    context_switch = (PATOMVM_CALLBACK_CONTEXT)callback;
    CONTEXT *                   pcontext = &context_switch->pcontext->context ;

    pcontext->ContextFlags = CONTEXT_VM ;

    return GetThreadContext (patomvm->vm_thread, pcontext) ;
}


/**
* \ingroup atomvm
* \b atomvmContextCreate
*
* This function is to be used by the atom virtual machine.
*
* This function creates a atomvm thread context that can be scheduled
* by atomvmContextSwitch.
*
* @param[out] context Handle to the context of the thread that are allocated
* by the caller.
* @param[in] stack Stack top.
* @param[in] entry Entry point using the default caling convention of the compiler.
*
* @return Zero on failure, try to call GetLastError().
*/
uint32_t
atomvmContextCreate (HATOMVM_CONTEXT* atomvm_context, uint32_t stack, uint32_t entry)
{
    uint32_t            res ;
    PATOMVM             patomvm = getAtomvm () ;
    PATOMVM_CONTEXT     new_context = (PATOMVM_CONTEXT)malloc (sizeof(ATOMVM_CONTEXT)) ;
    CONTEXT*            pcontext = &new_context->context ;
    ATOMVM_CALLBACK_CONTEXT     context_init ;

    context_init.pcontext = new_context ;
    new_context->interrupt_mask = 1 ;

    res = invokeCallback (patomvm, callbackContextCreate, (PATOMVM_CALLBACK)&context_init) ;

    if (res) {
        pcontext->Ebp = stack ;
        pcontext->Esp = stack ;
        pcontext->Eip = entry ;
        *atomvm_context = (HATOMVM_CONTEXT)new_context ;
    }

    return res ;
}


/**
* \b callbackContextSwitch
*
* This function is invoked from the controll thread after a call to atomvmContextSwitch.
*
* The atom virtual machine is suspended while this function is called.
*
* @param[in] patomvm Pointer to the virtual machine created by atomvmCtrlCreate.
* @param[out] context Context to be scheduled.
*
* @return Zero on failure, try to call GetLastError().
*/
uint32_t
callbackContextSwitch (PATOMVM patomvm, PATOMVM_CALLBACK callback)
{
    uint32_t                    res1 = 1 ;
    uint32_t                    res2 ;
    PATOMVM_CALLBACK_CONTEXT_SWITCH    context_switch = (PATOMVM_CALLBACK_CONTEXT_SWITCH)callback ;

    if (context_switch->p_old_context) {
        res1 = GetThreadContext (patomvm->vm_thread, &context_switch->p_old_context->context) ;
        ATOMVM_ASSERT(res1 , _T("GetThreadContext failed")) ;
    }

    patomvm->current_context = context_switch->p_new_context ;
    res2 = SetThreadContext (patomvm->vm_thread, &context_switch->p_new_context->context) ;
    ATOMVM_ASSERT(res2 , _T("SetThreadContext failed")) ;

    return res1 & res2 ;
}


/**
* \ingroup atomvm
* \b atomvmContextSwitch
*
* This function is to be used by the atom virtual machine.
*
* This function schedules a thread for the context created by atomvmContextCreate.
*
* @param[in] new_context The context to schedule.
*
* @return Zero on failure, try to call GetLastError().
*/
uint32_t
atomvmContextSwitch  (HATOMVM_CONTEXT old_context, HATOMVM_CONTEXT new_context)
{
    PATOMVM                     patomvm = getAtomvm () ;
    ATOMVM_CALLBACK_CONTEXT_SWITCH     context_switch ;

    context_switch.p_old_context = (PATOMVM_CONTEXT) old_context ;
    context_switch.p_new_context = (PATOMVM_CONTEXT) new_context ;

    return invokeCallback (patomvm, callbackContextSwitch, (PATOMVM_CALLBACK)&context_switch) ;
}

/**
* \ingroup atomvm
* \b atomvmContextDesrtroy
*
* This function is to be used by the atom virtual machine.
*
* This functiondestroyes a atomvm context created by atomvmContextCreate.
*
* @param[in] context The context to destroy.
*
* @return None
*/
void
atomvmContextDesrtroy  (HATOMVM_CONTEXT context)
{
    PATOMVM     patomvm = getAtomvm () ;

    ATOMVM_ASSERT(patomvm->current_context !=  (PATOMVM_CONTEXT)context,
    			_T("atomvmContextDesrtroy failed")) ;

    free((void*)context) ;
}

/**
* \ingroup atomvm
* \b atomvmWriteThreadId
*
* Write a thread ID.
*
* Write a thread ID for the current context.
*
* @param[in] thread_id thread_id.
*
* @return None
*/
void
atomvmWriteThreadId  (uint32_t thread_id)
{
    PATOMVM     patomvm = getAtomvm () ;

    patomvm->current_context->thread_id = thread_id ;
}

/**
* \ingroup atomvm
* \b atomvmReadThreadId
*
* Write a thread ID.
*
* Read a thread ID for the current context.
*
* @return thread_id
*/
uint32_t
atomvmReadThreadId  (void)
{
    PATOMVM     patomvm = getAtomvm () ;

    return patomvm->current_context->thread_id ;
}


/**
* \ingroup atomvm
* \b atomvmGetVmId
*
* Returns an identifier for the virtual machine. This is zero for the first 
* virtual machine created with atomvmCtrlCreate(), 1 for the second and so on.
*
* @return The atom vm ID
*/
uint32_t
atomvmGetVmId (void)
{
    PATOMVM patomvm =  getAtomvm () ;

    return patomvm->atomvm_id ;
}


/**
* \b callbackInterruptWait
*
* This function is invoked from the controll thread after a call to atomvmInterruptWait().
*
* The atom virtual machine is suspended while this function is called.
*
* @param[in] patomvm Pointer to the virtual machine created by atomvmCtrlCreate.
* @param[out] callback Callback parameter.
*
* @return Zero on failure, try to call GetLastError().
*/
uint32_t
callbackIntWait (PATOMVM patomvm, PATOMVM_CALLBACK callback)
{
    WaitForSingleObject (patomvm->atomvm_int_complete, INFINITE) ;
    return WaitForSingleObject (patomvm->atomvm_int, INFINITE) == WAIT_OBJECT_0 ;
}

/**
* \ingroup atomvm
* \b atomvmInterruptWait
*
* This function is to be used by the atom virtual machine.
*
* This function if for synchronization between multiple
* atom vms.
*
*
* @return void.
*/
void
atomvmIntWait  (void)
{
    PATOMVM                     patomvm = getAtomvm () ;
    ATOMVM_CALLBACK             callback ;

    invokeCallback (patomvm, callbackIntWait, (PATOMVM_CALLBACK)&callback) ;
}

/**
* \b callbackIntRequest
*
* This function is invoked from the controll thread after a call to atomvmIntRequest().
*
* The atom virtual machine is suspended while this function is called.
*
* @param[in] patomvm Pointer to the virtual machine created by atomvmCtrlCreate.
* @param[in] callback Callback parameter.
*
* @return Zero on failure, try to call GetLastError().
*/
uint32_t
callbackIntRequest (PATOMVM patomvm, PATOMVM_CALLBACK callback)
{
    PATOMVM_CALLBACK_INT_REQUEST    int_request = (PATOMVM_CALLBACK_INT_REQUEST)callback ;

    int_request->isr () ;
    return 1 ;
}

/**
* \ingroup atomvm
* \b atomvmIntRequest
*
* This function is to be used by the atom virtual machine.
*
* @param[in] isr Function that will be called from the controll thread.
*
* @return void.
*/
void
atomvmIntRequest  (void (*isr) (void))
{
    PATOMVM                         patomvm = getAtomvm () ;
    ATOMVM_CALLBACK_INT_REQUEST     callback ;
    
    callback.isr = isr ;
    invokeCallback (patomvm, callbackIntRequest, (PATOMVM_CALLBACK)&callback) ;
}


/**
* \b vm_thread
*
* Windows thread in which the atom virtual machine will execute.
*
* __atomvmReset() runs the virtual machie and should only return after
* __atomvmClose() was called.
*
* @return None.
*/
DWORD WINAPI
vm_thread (LPVOID lpParameter)
{
    BOOL res = TlsSetValue (g_atomvm_tls_idx, lpParameter) ;

    ATOMVM_ASSERT(res, _T("TlsSetValue failed")) ;
    __atomvmReset () ;
    return 0 ;
}
