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

#include "atomvm.h"
#include <string.h>

#define CONTEXT_VM											(CONTEXT_INTEGER | CONTEXT_CONTROL | CONTEXT_SEGMENTS)

#define ATOMVM_ATOMVM_PERF_COUNTER(patomvm, counter)		(InterlockedIncrement(&patomvm->perf_counters.counter))

/* Data types */

/* Forward declarations */
typedef struct ATOMVM_S *					PATOMVM ;
typedef struct ATOMVM_CALLBACK_S *			PATOMVM_CALLBACK ;
typedef struct ATOMVM_CONTEXT_S *			PATOMVM_CONTEXT ;

typedef  uint32_t (*ATOMVM_CALLBACK_F) (PATOMVM, PATOMVM_CALLBACK) ;

typedef struct ATOMVM_CALLBACK_S {

    /* Address of callback function */
    volatile ATOMVM_CALLBACK_F		callback ;

    /* Synchronization lock, the virtual machine will be suspended during
    the callback. Regular WIN32 synchronization methods cant be used
    because SuspendThread() is used on the vm thread. */
    volatile uint32_t 				lock ;

    /* Result of the call */
    volatile uint32_t 				result ;

} ATOMVM_CALLBACK, *PATOMVM_CALLBACK ;


/* ATOMVM_CALLBACK_CONTEXT is the parameter for a ATOMVM_CALLBACK_F call
that take as parameter a pointer to a ATOMVM_CONTEXT to operate on */
typedef struct ATOMVM_CALLBACK_CONTEXT_S {

    ATOMVM_CALLBACK					callback ;

    /* Context the callback function will operate on */
    volatile PATOMVM_CONTEXT		pcontext ;

} ATOMVM_CALLBACK_CONTEXT, *PATOMVM_CALLBACK_CONTEXT ;

typedef struct ATOMVM_PERF_COUNTERS_S {

    /* Number of interrupt requests */
    uint32_t						int_request ;

    /* Number of service calls (context switches and
    context init from atom virtual machime) */
    uint32_t						service_call ;

    /* Total number of context switches */
    uint32_t						context_switch ;

} ATOMVM_PERF_COUNTERS, *PATOMVM_PERF_COUNTERS ;

/* ATOMVM_CONTEXT saves the state of a context created by
atomvmContextCreate() and sheduled by atomvmContextSwitch(). */
typedef struct ATOMVM_CONTEXT_S {

    /* A virtual machine thread context. These are saved and restored
    during context initialization and context switches */
    CONTEXT							context ;

    /* When entering a critical section the critical_count is
    incremented for the context. Interrupts will only occur while
    the critical_count is zero. The functions atomvmExitCritical()
    and atomvmEnterCritical() will respectively decrement and
    increment the critical count  */
    volatile uint32_t				critical_count ;

} ATOMVM_CONTEXT, *PATOMVM_CONTEXT ;

/* ATOMVM defines the state of an instance to an atomvm. It is created
by a call to atomvmCtrlInit(). */
typedef struct ATOMVM_S {

    /* Thread the virtual machine will run in */
    HANDLE							vm_thread ;

    /* Handles to events and mutexes used for synchronization */
    HANDLE							atomvm_call ;
    HANDLE							atomvm_int ;
    HANDLE							atomvm_int_complete ;
    HANDLE							atomvm_close ;

    /* next ISR */
    volatile void					(*isr)(void) ;
    /* True if in an ISR */
    volatile uint32_t				status_isr ;

    /* The current context that was scheduled by a call
    to atomvmContextSwitch() */
    PATOMVM_CONTEXT					current_context ;

    /* Service call address, synchronization lock, parameters
    and, return value for the current service call */
    PATOMVM_CALLBACK				service_call ;

    /* Context for startup, before any context was scheduled
    (workaround to not check everytime if the first context
    was already started) */
    ATOMVM_CONTEXT					atom_init_context ;

    /* Performance counters */
    volatile ATOMVM_PERF_COUNTERS	perf_counters ;

} ATOMVM, *PATOMVM ;


/* Forward declaration for the atom virtual machine thread */
static DWORD WINAPI		vm_thread (LPVOID lpParameter) ;


/**
* \b atomvmCtrlInit
*
* This is an atomvm controll function used by a controlling thread
* and must not be called from the atom virtual machine.
*
* Initializes the virtual machine.
*
* ToDo: More Error checking.
*
* @param[out] atomvm Handle to the virtual machine created.
*
* @return Zero on failure.
*/
uint32_t
atomvmCtrlInit (HATOMVM *atomvm)
{
    PATOMVM patomvm = (PATOMVM) malloc (sizeof(struct ATOMVM_S)) ;

    if (patomvm) {

        memset (patomvm, 0, sizeof(struct ATOMVM_S)) ;

        patomvm->atomvm_call = CreateEvent (NULL, TRUE, FALSE, 0) ;
        patomvm->atomvm_int = CreateEvent (NULL, TRUE, FALSE, 0) ;
        patomvm->atomvm_int_complete = CreateEvent (NULL, FALSE, TRUE, 0) ;
        patomvm->atomvm_close = CreateEvent (NULL, TRUE, FALSE, 0) ;

        patomvm->vm_thread = CreateThread (NULL, 0, vm_thread, 0, CREATE_SUSPENDED, NULL) ;

        patomvm->atom_init_context.critical_count = 1 ;
        patomvm->current_context = &patomvm->atom_init_context ;

        *atomvm = (HATOMVM)patomvm ;

    }

    return patomvm != 0 ;
}


/**
* \b atomvmCtrlRun
*
* This is an atomvm controll function used by a controlling thread
* and must not be called from the atom virtual machine.
*
* After a call to atomvmCtrlInit this function start the atom virtual machine.
* The calling thread will be used to manage interrupts and service calls in
* the virtual machine. This function will not return untill atomvmCtrlClose
* is called.
*
* @param[in] atomvm Handle to the virtual machine created by atomvmCtrlInit.
* @param[in] flags Reserved for later use.
*
* @return None
*/
void
atomvmCtrlRun (HATOMVM atomvm, uint32_t flags)
{
    PATOMVM				patomvm = (PATOMVM) atomvm ;
    HANDLE				wait[3] ;
    uint32_t			res ;
    uint32_t			wait_object ;
    PATOMVM_CALLBACK	pservice_call ;

    ResumeThread (patomvm->vm_thread) ;

    wait[0] = patomvm->atomvm_call ;
    wait[1] = patomvm->atomvm_int ;
    wait[2] = patomvm->atomvm_close ;

    for(;;) {

        wait_object = WaitForMultipleObjects (3, wait,FALSE,INFINITE) ;

        if (wait_object == WAIT_OBJECT_0) {

            ATOMVM_ATOMVM_PERF_COUNTER(patomvm, service_call) ;

            //InterlockedExchange ((volatile uint32_t*)&pservice_call, (uint32_t)patomvm->service_call) ;
            pservice_call = patomvm->service_call ;
            while (!pservice_call->lock) {
                SwitchToThread () ;
            }

            while ((res = SuspendThread (patomvm->vm_thread)) == (DWORD)-1) ;
            ATOMVM_ASSERT(res == 0 , _T("SuspendThread failed")) ;
#if (_WIN32_WINNT >= 0x0600)
            /*
                This is used for multi processor machines to ensure the thread
                is stopped before executing the next instruction. */
            FlushProcessWriteBuffers ();
#endif
            InterlockedExchange (&pservice_call->result, pservice_call->callback (patomvm, pservice_call)) ;
            InterlockedExchange (&pservice_call->lock, 0) ;
            ResetEvent (patomvm->atomvm_call) ;
            res = ResumeThread (patomvm->vm_thread) ;
            ATOMVM_ASSERT(res == 1 , _T("ResumeThread failed")) ;

        }

        else if (wait_object == WAIT_OBJECT_0 + 1) {

            if (patomvm->current_context->critical_count == 0) {

                while ((res = SuspendThread (patomvm->vm_thread)) == (DWORD)-1) ;
                ATOMVM_ASSERT(res == 0 , _T("SuspendThread failed")) ;
#if (_WIN32_WINNT >= 0x0600)
                /*
                    This is used for multi processor machines to ensure the thread
                    is stopped before executing the next instruction. */
                FlushProcessWriteBuffers ();
#endif
                if (patomvm->current_context->critical_count == 0) {

                    ATOMVM_ATOMVM_PERF_COUNTER(patomvm, int_request) ;

                    patomvm->status_isr++ ;
                    patomvm->isr () ;
                    patomvm->status_isr-- ;

                    res = ResumeThread (patomvm->vm_thread) ;
                    ATOMVM_ASSERT(res == 1 , _T("ResumeThread failed")) ;
                    InterlockedExchange ((volatile uint32_t*)&patomvm->isr, 0) ;
                    ResetEvent (patomvm->atomvm_int) ;	
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

        }

    }

}


/**
* \b atomvmCtrlClose
*
* This is an atomvm controll function used by a controlling thread
* and must not be called from the atom virtual machine.
*
* Closes the virtual machine and release all memory and handles created
* in atomvmCtrlInit.
*
* ToDo: this function was never tested.
*
* @param[in] atomvm Handle to the virtual machine created by atomvmCtrlInit.
*
* @return None
*/
void
atomvmCtrlClose (HATOMVM atomvm)
{
    PATOMVM		patomvm = (PATOMVM) atomvm ;
    DWORD		code ;

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
* The atom virtual machine is suspended during the callback.
*
* @param[in] patomvm Pointer to the virtual machine created by atomvmCtrlInit.
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


/**
* \b atomvmExitCritical
*
* This function is to be used by the atom virtual machine.
*
* This function will decrement the critical count for the current atomvm context.
* When the critical count reaches zero, interrupts will be enabled again. Calling
* this function from inside an isr has no effect.
*
* @param[in] atomvm Handle to the virtual machine created by atomvmCtrlInit.
*
* @return Critical count before this function was called.
*/
int32_t
atomvmExitCritical (HATOMVM atomvm)
{
    PATOMVM	patomvm = (PATOMVM) atomvm ;
    int32_t count = 0;

    if (patomvm->status_isr == 0) {
        count = InterlockedDecrement (&patomvm->current_context->critical_count) ;
    }

    return count ;
}


/**
* \b atomvmEnterCritical
*
* This function is to be used by the atom virtual machine.
*
* This function will increment the critical count for the current atomvm
* context. Interrupts will be disabled when the critical count is not zero.
* Calling this function from inside an isr has no effect.
*
* All threads are created with a critical count of 1.
*
* @param[in] atomvm Handle to the virtual machine created by atomvmCtrlInit.
*
* @return Critical count before this function was called.
*/
int32_t
atomvmEnterCritical (HATOMVM atomvm)
{
    PATOMVM	patomvm = (PATOMVM) atomvm ;
    int32_t count = 0 ;

    if (patomvm->status_isr == 0) {
        count = InterlockedIncrement (&patomvm->current_context->critical_count) ;
    }

    return count ;
}


/**
* \b atomvmCtrlIntRequest
*
* This is an atomvm controll function used by a controlling thread(s)
* and must not be called from the atom virtual machine.
*
* This function requests an interrupt service routine to be called in the
* context of the atom virtual machine.
*
* The call will block untill after the interrupt completes.
*
* @param[in] atomvm Handle to the virtual machine created by atomvmCtrlInit.
* @param[in] isr The address of the interrupt service routine. The routine must use
* the default calling convention of the compiler.
*
* @return None
*/
void
atomvmCtrlIntRequest (HATOMVM atomvm, uint32_t isr)
{
    PATOMVM	patomvm = (PATOMVM) atomvm ;

    while (InterlockedCompareExchange ((volatile uint32_t *)&patomvm->isr, isr, 0) == 0) {
		SwitchToThread() ;
	}
    SignalObjectAndWait(patomvm->atomvm_int, patomvm->atomvm_int_complete, INFINITE, FALSE) ;

}


/**
* \b callbackContextCreate
*
* This function is invoked from the controll thread after a call to atomvmContextCreate.
*
* The atom virtual machine is suspended while this function is called.
*
* @param[in] patomvm Pointer to the virtual machine created by atomvmCtrlInit.
* @param[out] context Context to be initialized.
*
* @return Zero on failure, try to call GetLastError().
*/
uint32_t
callbackContextCreate (PATOMVM patomvm, PATOMVM_CALLBACK callback)
{
    PATOMVM_CALLBACK_CONTEXT context_switch = (PATOMVM_CALLBACK_CONTEXT)callback;
    CONTEXT * pcontext = &context_switch->pcontext->context ;

    pcontext->ContextFlags = CONTEXT_VM ;

    return GetThreadContext (patomvm->vm_thread, pcontext) ;
}


/**
* \b atomvmContextCreate
*
* This function is to be used by the atom virtual machine.
*
* This function creates a atomvm thread context that can be scheduled
* by atomvmContextSwitch.
*
* @param[in] atomvm Handle to the virtual machine created by atomvmCtrlInit.
* @param[out] context Handle to the context of the thread that are allocated
* by the caller.
* @param[in] stack Stack top.
* @param[in] entry Entry point using the default caling convention of the compiler.
*
* @return Zero on failure, try to call GetLastError().
*/
uint32_t
atomvmContextCreate (HATOMVM atomvm, HATOMVM_CONTEXT* atomvm_context, uint32_t stack, uint32_t entry)
{
    uint32_t res ;
    PATOMVM	patomvm = (PATOMVM) atomvm ;
    PATOMVM_CONTEXT new_context = (PATOMVM_CONTEXT)malloc (sizeof(ATOMVM_CONTEXT)) ;
    CONTEXT* pcontext = &new_context->context ;
    ATOMVM_CALLBACK_CONTEXT context_init ;

    context_init.pcontext = new_context ;

    new_context->critical_count = 1 ;

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
* @param[in] patomvm Pointer to the virtual machine created by atomvmCtrlInit.
* @param[out] context Context to be scheduled.
*
* @return Zero on failure, try to call GetLastError().
*/
uint32_t
callbackContextSwitch (PATOMVM patomvm, PATOMVM_CALLBACK callback)
{
    uint32_t res1 ;
    uint32_t res2 ;
    PATOMVM_CALLBACK_CONTEXT context_switch = (PATOMVM_CALLBACK_CONTEXT)callback ;
    CONTEXT* pnew_context = &context_switch->pcontext->context ;

    ATOMVM_ATOMVM_PERF_COUNTER(patomvm, context_switch) ;

    res1 = GetThreadContext (patomvm->vm_thread, &patomvm->current_context->context) ;
    ATOMVM_ASSERT(res1 , _T("GetThreadContext failed")) ;

    InterlockedExchange ((volatile uint32_t*)&patomvm->current_context, (uint32_t)pnew_context) ;
    res2 = SetThreadContext (patomvm->vm_thread, &patomvm->current_context->context) ;
    ATOMVM_ASSERT(res2 , _T("SetThreadContext failed")) ;

    return res1 & res2 ;
}


/**
* \b atomvmContextSwitch
*
* This function is to be used by the atom virtual machine.
*
* This function schedules a thread for the context created by atomvmContextCreate.
*
* @param[in] atomvm Handle to the virtual machine created by atomvmCtrlInit.
* @param[in] new_context The context to schedule.
*
* @return Zero on failure, try to call GetLastError().
*/
uint32_t
atomvmContextSwitch  (HATOMVM atomvm, HATOMVM_CONTEXT new_context)
{
    PATOMVM	patomvm = (PATOMVM) atomvm ;
    ATOMVM_CALLBACK_CONTEXT context_switch ;

    context_switch.pcontext = (PATOMVM_CONTEXT) new_context ;

    return invokeCallback (patomvm, callbackContextSwitch, (PATOMVM_CALLBACK)&context_switch) ;
}

/**
* \b atomvmContextDesrtroy
*
* This function is to be used by the atom virtual machine.
*
* This functiondestroyes a atomvm context created by atomvmContextCreate.
*
* @param[in] atomvm Handle to the virtual machine created by atomvmCtrlInit.
* @param[in] context The context to destroy.
*
* @return None
*/
void
atomvmContextDesrtroy  (HATOMVM atomvm, HATOMVM_CONTEXT context)
{
    PATOMVM	patomvm = (PATOMVM) atomvm ;

    ATOMVM_ASSERT(patomvm->current_context !=  (PATOMVM_CONTEXT)context,
    			_T("atomvmContextDesrtroy failed")) ;

    free((void*)context) ;
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
    __atomvmReset () ;
    return 0 ;
}
