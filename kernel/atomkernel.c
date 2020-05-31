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


/**
 * \file
 * Kernel library.
 *
 *
 * This module implements the core kernel functionality of managing threads,
 * context-switching and interrupt handlers. It also contains functions for
 * managing queues of TCBs (task control blocks) which are used not only for
 * the queue of ready threads, but also by other OS primitives (such as
 * semaphores) for generically managing lists of TCBs.
 *
 * Core kernel functionality such as managing the queue of ready threads and
 * how context-switch decisions are made is described within the code. However
 * a quick summary is as follows:
 *
 * There is a ready queue of threads. There must always be at least one thread
 * ready-to-run. If no application threads are ready, the internal kernel idle
 * thread will be run. This ensures that there is a thread to run at all
 * times.
 *
 * Application code creates threads using atomThreadCreate(). These threads
 * are added to the ready queue and eventually run when it is their turn
 * (based on priority). When threads are currently-running they are taken off
 * the ready queue. Threads continue to run until:
 * \li They schedule themselves out by calling an OS primitive which blocks,
 *     such as a timer delay or blocking on a semaphore. At this point they
 *     are placed on the queue of the OS primitive in which they are blocking
 *     (for example a timer delay or semaphore).
 * \li They are preempted by a higher priority thread. This could happen at
 *     any time if a kernel call from the currently-running thread or from an
 *     interrupt handler makes a higher priority thread ready-to-run.
 *     Generally this will occur immediately, and while the previously-running
 *     thread is still considered ready-to-run, it is no longer the
 *     currently-running thread so goes back on to the ready queue.
 * \li They are scheduled out after a timeslice when another thread of the
 *     same priority is also ready. This happens on a timer tick, and ensures
 *     that threads of the same priority share timeslices. In this case the
 *     previously-running thread is still considered ready-to-run so is placed
 *     back on to the ready queue.
 *
 * Thread scheduling decisions are made by atomSched(). This is called at
 * several times, but should never be called by application code directly:
 * \li After interrupt handlers: The scheduler is called after every
 *     interrupt handler has completed. This allows for any threads which
 *     have been made ready-to-run by the interrupt handler to be scheduled
 *     in. For example if an interrupt handler posts a semaphore which wakes
 *     up a thread of higher priority than the currently-running thread, then
 *     the end of interrupt handler reschedule will schedule that thread in.
 * \li On timer ticks: The timer tick is implemented as an interrupt handler
 *     so the end of interrupt call to the scheduler is made as normal, except
 *     that in this case round-robin rescheduling is allowed (where threads
 *     of the same priority are given a timeslice each in round-robin
 *     fashion). This must only occur on timer ticks when the system tick
 *     count is incremented.
 * \li After any OS call changes ready states: Any OS primitives which change
 *     the running state of a thread will call the scheduler to ensure that
 *     the change of thread state is noted. For example if a new thread is
 *     created using atomThreadCreate(), it will internally call the scheduler
 *     in case the newly-created thread is higher priority than the
 *     currently-running thread. Similarly OS primitives such as semaphores
 *     often make changes to a thread's running state. If a thread is going to
 *     sleep blocking on a semaphore then the scheduler will be run to ensure
 *     that some other thread is scheduled in in its place. If a thread is
 *     woken by a semaphore post, the scheduler will also be called in case
 *     that thread should now be scheduled in (note that when semaphores are
 *     posted from an interrupt handler this is deferred to the end of
 *     interrupt scheduler call).
 *
 * When a thread reschedule needs to take place, the scheduler calls out to
 * the architecture-specific port to perform the context-switch, using
 * archContextSwitch() which must be provided by each architecture port. This
 * function carries out the low-level saving and restoring of registers
 * appropriate for the architecture. The thread being switched out must have
 * a set of CPU registers saved, and the thread being scheduled in has a set
 * of CPU registers restored (which were previously saved). In this fashion
 * threads are rescheduled with the CPU registers in exactly the same state as
 * when the thread was scheduled out.

 * New threads which have never been scheduled in have a pre-formatted stack
 * area containing a set of CPU register values ready for restoring that
 * appears exactly as if the thread had been previously scheduled out. In
 * other words, the scheduler need not know when it restores registers to
 * switch a thread in whether it has previously run or if it has never been
 * run since the thread was created. The context-save area is formatted in
 * exactly the same manner.
 *
 *
 * \b Functions contained in this module:\n
 *
 * \b Application-callable initialisation functions: \n
 *
 * \li atomOSInit(): Initialises the operating system.
 * \li atomOSStart(): Starts the OS running (with the highest priority thread).
 *
 * \b Application-callable general functions: \n
 *
 * \li atomThreadCreate(): Thread creation API.
 * \li atomCurrentContext(): Used by kernel and application code to check
 *     whether the thread is currently running at thread or interrupt context.
 *     This is very useful for implementing safety checks and preventing
 *     interrupt handlers from making kernel calls that would block.
 * \li atomIntEnter() / atomIntExit(): Must be called by any interrupt handlers.
 *
 * \b Internal kernel functions: \n
 *
 * \li atomSched(): Core scheduler.
 * \li atomThreadSwitch(): Context-switch routine.
 * \li atomIdleThread(): Simple thread to be run when no other threads ready.
 * \li tcbEnqueuePriority(): Enqueues TCBs (task control blocks) on lists.
 * \li tcbDequeueHead(): Dequeues the head of a TCB list.
 * \li tcbDequeueEntry(): Dequeues a particular entry from a TCB list.
 * \li tcbDequeuePriority(): Dequeues an entry from a TCB list using priority.
 *
 */


#include "atom.h"


/* Global data */

/**
 * This is the head of the queue of threads that are ready to run. It is
 * ordered by priority, with the higher priority threads coming first. Where
 * there are multiple threads of the same priority, the TCB (task control
 * block) pointers are FIFO-ordered.
 *
 * Dequeuing the head is a fast operation because the list is ordered.
 * Enqueuing may have to walk up to the end of the list. This means that
 * context-switch times depend on the number of threads on the ready queue,
 * but efficient use is made of available RAM on tiny systems by avoiding
 * priority tables etc. This scheme can be easily swapped out for other
 * scheduler schemes by replacing the TCB enqueue and dequeue functions.
 *
 * Once a thread is scheduled in, it is not present on the ready queue or any
 * other kernel queue while it is running. When scheduled out it will be
 * either placed back on the ready queue (if still ready), or will be suspended
 * on some OS primitive if no longer ready (e.g. on the suspended TCB queue
 * for a semaphore, or in the timer list if suspended on a timer delay).
 */
ATOM_TCB *tcbReadyQ = NULL;

/** Set to TRUE when OS is started and running threads */
uint8_t atomOSStarted = FALSE;


/* Local data */

/** This is a pointer to the TCB for the currently-running thread */
static ATOM_TCB *curr_tcb = NULL;

/** Storage for the idle thread's TCB */
static ATOM_TCB idle_tcb;

/* Number of nested interrupts */
static int atomIntCnt = 0;


/* Constants */

/** Bytecode to fill thread stacks with for stack-checking purposes */
#define STACK_CHECK_BYTE    0x5A


/* Forward declarations */
static void atomThreadSwitch(ATOM_TCB *old_tcb, ATOM_TCB *new_tcb);
static void atomIdleThread (uint32_t data);


/**
 * \b atomSched
 *
 * This is an internal function not for use by application code.
 *
 * This is the main scheduler routine. It is called by the various OS
 * library routines to check if any threads should be scheduled in now.
 * If so, the context will be switched from the current thread to the
 * new one.
 *
 * The scheduler is priority-based with round-robin performed on threads
 * with the same priority. Round-robin is only performed on timer ticks
 * however. During reschedules caused by an OS operation (e.g. after
 * giving or taking a semaphore) we only allow the scheduling in of
 * threads with higher priority than current priority. On timer ticks we
 * also allow the scheduling of same-priority threads - in that case we
 * schedule in the head of the ready list for that priority and put the
 * current thread at the tail.
 *
 * @param[in] timer_tick Should be TRUE when called from the system tick
 *
 * @return None
 */
void atomSched (uint8_t timer_tick)
{
    CRITICAL_STORE;
    ATOM_TCB *new_tcb = NULL;
    int16_t lowest_pri;

    /**
     * Check the OS has actually started. As long as the proper initialisation
     * sequence is followed there should be no calls here until the OS is
     * started, but we check to handle badly-behaved ports.
     */
    if (atomOSStarted == FALSE)
    {
        /* Don't schedule anything in until the OS is started */
        return;
    }

    /* Enter critical section */
    CRITICAL_START ();

    /**
     * If the current thread is going into suspension or is being
     * terminated (run to completion), then unconditionally dequeue
     * the next thread for execution.
     */
    if ((curr_tcb->suspended == TRUE) || (curr_tcb->terminated == TRUE))
    {
        /**
         * Dequeue the next ready to run thread. There will always be
         * at least the idle thread waiting. Note that this could
         * actually be the suspending thread if it was unsuspended
         * before the scheduler was called.
         */
        new_tcb = tcbDequeueHead (&tcbReadyQ);

        /**
         * Don't need to add the current thread to any queue because
         * it was suspended by another OS mechanism and will be
         * sitting on a suspend queue or similar within one of the OS
         * primitive libraries (e.g. semaphore).
         */

        /* Switch to the new thread */
        atomThreadSwitch (curr_tcb, new_tcb);
    }

    /**
     * Otherwise the current thread is still ready, but check
     * if any other threads are ready.
     */
    else
    {
        /* Calculate which priority is allowed to be scheduled in */
        if (timer_tick == TRUE)
        {
            /* Same priority or higher threads can preempt */
            lowest_pri = (int16_t)curr_tcb->priority;
        }
        else if (curr_tcb->priority > 0)
        {
            /* Only higher priority threads can preempt, invalid for 0 (highest) */
            lowest_pri = (int16_t)(curr_tcb->priority - 1);
        }
        else
        {
            /**
             * Current priority is already highest (0), don't allow preempt by
             * threads of any priority because this is not a time-slice.
             */
            lowest_pri = -1;
        }

        /* Check if a reschedule is allowed */
        if (lowest_pri >= 0)
        {
            /* Check for a thread at the given minimum priority level or higher */
            new_tcb = tcbDequeuePriority (&tcbReadyQ, (uint8_t)lowest_pri);

            /* If a thread was found, schedule it in */
            if (new_tcb)
            {
                /* Add the current thread to the ready queue */
                (void)tcbEnqueuePriority (&tcbReadyQ, curr_tcb);

                /* Switch to the new thread */
                atomThreadSwitch (curr_tcb, new_tcb);
            }
        }
    }

    /* Exit critical section */
    CRITICAL_END ();
}


/**
 * \b atomThreadSwitch
 *
 * This is an internal function not for use by application code.
 *
 * The function is called by the scheduler to perform a context switch.
 * Execution will switch to the new thread's context, therefore the
 * function doesn't actually return until the old thread is scheduled
 * back in.
 *
 * @param[in] old_tcb Pointer to TCB for thread being scheduled out
 * @param[in] new_tcb Pointer to TCB for thread being scheduled in
 *
 * @return None
 */
static void atomThreadSwitch(ATOM_TCB *old_tcb, ATOM_TCB *new_tcb)
{
    /**
     * The context switch will shift execution to a different thread. The
     * new thread is now ready to run so clear its suspend status in
     * preparation for it waking up.
     */
    new_tcb->suspended = FALSE;

    /**
     * Check if the new thread is actually the current one, in which
     * case we don't need to do any context switch. This can happen
     * if a thread goes into suspend but is unsuspended again before
     * it is fully scheduled out.
     */
    if (old_tcb != new_tcb)
    {
        /* Set the new currently-running thread pointer */
        curr_tcb = new_tcb;

        /* Call the architecture-specific context switch */
        archContextSwitch (old_tcb, new_tcb);
    }
}


/**
 * \b atomThreadCreate
 *
 * Creates and starts a new thread.
 *
 * Callers provide the ATOM_TCB structure storage, these are not obtained
 * from an internal TCB free list.
 *
 * The function puts the new thread on the ready queue and calls the
 * scheduler. If the priority is higher than the current priority, then the
 * new thread may be scheduled in before the function returns.
 *
 * Optionally prefills the thread stack with a known value to enable stack
 * usage checking (if the ATOM_STACK_CHECKING macro is defined and
 * stack_check parameter is set to TRUE).
 *
 * @param[in] tcb_ptr Pointer to the thread's TCB storage
 * @param[in] priority Priority of the thread (0 to 255)
 * @param[in] entry_point Thread entry point
 * @param[in] entry_param Parameter passed to thread entry point
 * @param[in] stack_bottom Bottom of the stack area
 * @param[in] stack_size Size of the stack area in bytes
 * @param[in] stack_check TRUE to enable stack checking for this thread
 *
 * @retval ATOM_OK Success
 * @retval ATOM_ERR_PARAM Bad parameters
 * @retval ATOM_ERR_QUEUE Error putting the thread on the ready queue
 */
uint8_t atomThreadCreate (ATOM_TCB *tcb_ptr, uint8_t priority, void (*entry_point)(uint32_t), uint32_t entry_param, void *stack_bottom, uint32_t stack_size, uint8_t stack_check)
{
    CRITICAL_STORE;
    uint8_t status;
    uint8_t *stack_top;
#ifdef ATOM_STACK_CHECKING
	int32_t count;
#endif

    if ((tcb_ptr == NULL) || (entry_point == NULL) || (stack_bottom == NULL)
        || (stack_size == 0))
    {
        /* Bad parameters */
        status = ATOM_ERR_PARAM;
    }
    else
    {

        /* Set up the TCB initial values */
        tcb_ptr->suspended = FALSE;
        tcb_ptr->terminated = FALSE;
        tcb_ptr->priority = priority;
        tcb_ptr->prev_tcb = NULL;
        tcb_ptr->next_tcb = NULL;
        tcb_ptr->suspend_timo_cb = NULL;

        /**
         * Store the thread entry point and parameter in the TCB. This may
         * not be necessary for all architecture ports if they put all of
         * this information in the initial thread stack.
         */
        tcb_ptr->entry_point = entry_point;
        tcb_ptr->entry_param = entry_param;

        /**
         * Calculate a pointer to the topmost stack entry, suitably aligned
         * for the architecture. This may discard the top few bytes if the
         * stack size is not a multiple of the stack entry/alignment size.
         */
        stack_top = (uint8_t *)stack_bottom + (stack_size & ~(STACK_ALIGN_SIZE - 1)) - STACK_ALIGN_SIZE;

        /**
         * Additional processing only required if stack-checking is
         * enabled. Incurs a slight overhead on each thread creation
         * and uses some additional storage in the TCB, but can be
         * compiled out if not desired.
         */
#ifdef ATOM_STACK_CHECKING
        /* Set up stack-checking if enabled for this thread */
        if (stack_check)
        {
            /* Store the stack details for use by the stack-check function */
            tcb_ptr->stack_bottom = stack_bottom;
            tcb_ptr->stack_size = stack_size;

            /**
             * Prefill the stack with a known value. This is used later in
             * calls to atomThreadStackCheck() to get an indication of how
             * much stack has been used during runtime.
             */
		    count = (int32_t)stack_size;
            while (count > 0)
            {
                /* Initialise all stack bytes from top down to 0x5A */
                *((uint8_t *)stack_bottom + (count - 1)) = STACK_CHECK_BYTE;
                count--;
            }
        }
#else
        /* Avoid compiler warning due to unused parameter */
        stack_check = stack_check;
#endif

        /**
         * Call the arch-specific routine to set up the stack. This routine
         * is responsible for creating the context save area necessary for
         * allowing atomThreadSwitch() to schedule it in. The initial
         * archContextSwitch() call when this thread gets scheduled in the
         * first time will then restore the program counter to the thread
         * entry point, and any other necessary register values ready for
         * it to start running.
         */
        archThreadContextInit (tcb_ptr, stack_top, entry_point, entry_param);

        /* Protect access to the OS queue */
        CRITICAL_START ();

        /* Put this thread on the ready queue */
        if (tcbEnqueuePriority (&tcbReadyQ, tcb_ptr) != ATOM_OK)
        {
            /* Exit critical region */
            CRITICAL_END ();

            /* Queue-related error */
            status = ATOM_ERR_QUEUE;
        }
        else
        {
            /* Exit critical region */
            CRITICAL_END ();

            /**
             * If the OS is started and we're in thread context, check if we
             * should be scheduled in now.
             */
            if ((atomOSStarted == TRUE) && atomCurrentContext())
                atomSched (FALSE);

            /* Success */
            status = ATOM_OK;
        }
    }

    return (status);
}


#ifdef ATOM_STACK_CHECKING
/**
 * \b atomThreadStackCheck
 *
 * Check the stack usage of a thread.
 *
 * If the ATOM_STACK_CHECKING macro is defined, thread stacks are filled
 * with a known value before the thread is started. This function can be
 * called at runtime to examine the stack and find the high water mark
 * (the furthest modified byte from the start of the stack).
 *
 * This gives an indication of how much stack the thread has used. It is
 * useful but not absolutely precise because the thread may legitimately
 * have the known value on its stack. The thread's stack pointer may also
 * have strayed outside of the allowable stack area while leaving some of
 * the known-value bytes unmodified. This simple method cannot trap stack
 * usage outside of the thread's allocated stack, for which you could use
 * additional guard areas (still limited in scope) or compiler/CPU/MMU
 * features.
 *
 * The function takes a thread's TCB and returns both the number of stack
 * bytes used, and the free stack bytes.
 *
 * @param[in] tcb_ptr Pointer to the TCB of the thread to stack-check
 * @param[in,out] used_bytes Pointer into which the used byte count is copied
 * @param[in,out] free_bytes Pointer into which the free byte count is copied
 *
 * @retval ATOM_OK Success
 * @retval ATOM_ERR_PARAM Bad parameters
 * @retval ATOM_ERR_QUEUE Error putting the thread on the ready queue
 */
uint8_t atomThreadStackCheck (ATOM_TCB *tcb_ptr, uint32_t *used_bytes, uint32_t *free_bytes)
{
    uint8_t status;
    uint8_t *stack_ptr;
    int i;

    if ((tcb_ptr == NULL) || (used_bytes == NULL) || (free_bytes == NULL))
    {
        /* Bad parameters */
        status = ATOM_ERR_PARAM;
    }
    else
    {
        /**
         * Starting at the bottom end, count the unmodified areas until a
         * modified byte is found.
         */
        stack_ptr = (uint8_t *)tcb_ptr->stack_bottom;
        for (i = 0; i < tcb_ptr->stack_size; i++)
        {
            /* Loop until a modified byte is found */
            if (*stack_ptr++ != STACK_CHECK_BYTE)
            {
                /* Found a modified byte */
                break;
            }
        }

        /* We quit the loop above on the count of the free bytes */
        *free_bytes = (uint32_t)i;

        /* Calculate used bytes using our knowledge of the stack size */
        *used_bytes = tcb_ptr->stack_size - *free_bytes;

        /* No error */
        status = ATOM_OK;

    }

    return (status);

}
#endif /* ATOM_STACK_CHECKING */


/**
 * \b atomIntEnter
 *
 * Interrupt handler entry routine.
 *
 * Must be called at the start of any interrupt handlers that may
 * call an OS primitive and make a thread ready.
 *
 * @return None
 */
void atomIntEnter (void)
{
    /* Increment the interrupt count */
    atomIntCnt++;
}


/**
 * \b atomIntExit
 *
 * Interrupt handler exit routine.
 *
 * Must be called at the end of any interrupt handlers that may
 * call an OS primitive and make a thread ready.
 *
 * This is responsible for calling the scheduler at the end of
 * interrupt handlers to determine whether a new thread has now
 * been made ready and should be scheduled in.
 *
 * @param timer_tick TRUE if this is a timer tick
 *
 * @return None
 */
void atomIntExit (uint8_t timer_tick)
{
    /* Decrement the interrupt count */
    atomIntCnt--;

    /* Call the scheduler */
    atomSched (timer_tick);
}


/**
 * \b atomCurrentContext
 *
 * Get the current thread context.
 *
 * Returns a pointer to the current thread's TCB, or NULL if not in
 * thread-context (in interrupt context).
 *
 * @retval Pointer to current thread's TCB, NULL if in interrupt context
 */
ATOM_TCB *atomCurrentContext (void)
{
    /* Return the current thread's TCB or NULL if in interrupt context */
    if (atomIntCnt == 0)
        return (curr_tcb);
    else
        return (NULL);
}


/**
 * \b atomOSInit
 *
 * Initialise the atomthreads OS.
 *
 * Must be called before any application code uses the atomthreads APIs. No
 * threads are actually started until the application calls atomOSStart().
 *
 * Callers must provide a pointer to some storage for the idle thread stack.
 * The caller is responsible for calculating the appropriate space required
 * for their particular architecture.
 *
 * Applications should use the following initialisation sequence:
 *
 * \li Call atomOSInit() before calling any atomthreads APIs
 * \li Arrange for a timer to call atomTimerTick() periodically
 * \li Create one or more application threads using atomThreadCreate()
 * \li Start the OS using atomOSStart(). At this point the highest
 *     priority application thread created will be started.
 *
 * Interrupts should be disabled until the first thread restore is complete,
 * to avoid any complications due to interrupts occurring while crucial
 * operating system facilities are being initialised. They are normally
 * enabled by the archFirstThreadRestore() routine in the architecture port.
 *
 * @param[in] idle_thread_stack_bottom Ptr to bottom of stack for idle thread
 * @param[in] idle_thread_stack_size Size of idle thread stack in bytes
 * @param[in] idle_thread_stack_check TRUE if stack checking required on idle thread
 *
 * @retval ATOM_OK Success
 * @retval ATOM_ERROR Initialisation error
 */
uint8_t atomOSInit (void *idle_thread_stack_bottom, uint32_t idle_thread_stack_size, uint8_t idle_thread_stack_check)
{
    uint8_t status;

    /* Initialise data */
    curr_tcb = NULL;
    tcbReadyQ = NULL;
    atomOSStarted = FALSE;

    /* Create the idle thread */
    status = atomThreadCreate(&idle_tcb,
                 IDLE_THREAD_PRIORITY,
                 atomIdleThread,
                 0,
                 idle_thread_stack_bottom,
                 idle_thread_stack_size,
				 idle_thread_stack_check);

    /* Return status */
    return (status);

}
/**
 * \b atomOSStart
 *
 * Start the highest priority thread running.
 *
 * This function must be called after all OS initialisation is complete, and
 * at least one application thread has been created. It will start executing
 * the highest priority thread created (or first created if multiple threads
 * share the highest priority).
 *
 * Interrupts must still be disabled at this point. They must only be enabled
 * when the first thread is restored and started by the architecture port's
 * archFirstThreadRestore() routine.
 *
 * @return None
 */
void atomOSStart (void)
{
    ATOM_TCB *new_tcb;

    /**
     * Enable the OS started flag. This stops routines like atomThreadCreate()
     * attempting to schedule in a newly-created thread until the scheduler is
     * up and running.
     */
    atomOSStarted = TRUE;

    /**
     * Application calls to atomThreadCreate() should have added at least one
     * thread to the ready queue. Take the highest priority one off and
     * schedule it in. If no threads were created, the OS will simply start
     * the idle thread (the lowest priority allowed to be scheduled is the
     * idle thread's priority, 255).
     */
    new_tcb = tcbDequeuePriority (&tcbReadyQ, 255);
    if (new_tcb)
    {
        /* Set the new currently-running thread pointer */
        curr_tcb = new_tcb;

        /* Restore and run the first thread */
        archFirstThreadRestore (new_tcb);

        /* Never returns to here, execution shifts to new thread context */
    }
    else
    {
        /* No ready threads were found. atomOSInit() probably was not called */
    }

}


/**
 * \b atomIdleThread
 *
 * Entry point for idle thread.
 *
 * This thread must always be present, and will be the thread executed when
 * no other threads are ready to run. It must not call any library routines
 * which would cause it to block.
 *
 * @param[in] param Unused (optional thread entry parameter)
 *
 * @return None
 */
static void atomIdleThread (uint32_t param)
{
    /* Compiler warning  */
    param = param;

    /* Loop forever */
    while (1)
    {
        /** \todo Provide user idle hooks*/
    }
}


/**
 * \b tcbEnqueuePriority
 *
 * This is an internal function not for use by application code.
 *
 * Enqueues the TCB \c tcb_ptr on the TCB queue pointed to by \c tcb_queue_ptr.
 * TCBs are placed on the queue in priority order. If there are existing TCBs
 * at the same priority as the TCB to be enqueued, the enqueued TCB will be
 * placed at the end of the same-priority TCBs. Calls to tcbDequeuePriority()
 * will dequeue same-priority TCBs in FIFO order.
 *
 * \c tcb_queue_ptr may be modified by the routine if the enqueued TCB becomes
 * the new list head. It is valid for tcb_queue_ptr to point to a NULL pointer,
 * which is the case if the queue is currently empty.
 *
 * \b NOTE: Assumes that the caller is already in a critical section.
 *
 * @param[in,out] tcb_queue_ptr Pointer to TCB queue head pointer
 * @param[in] tcb_ptr Pointer to TCB to enqueue
 *
 * @retval ATOM_OK Success
 * @retval ATOM_ERR_PARAM Bad parameters
 */
uint8_t tcbEnqueuePriority (ATOM_TCB **tcb_queue_ptr, ATOM_TCB *tcb_ptr)
{
    uint8_t status;
    ATOM_TCB *prev_ptr, *next_ptr;

    /* Parameter check */
    if ((tcb_queue_ptr == NULL) || (tcb_ptr == NULL))
    {
        /* Return error */
        status = ATOM_ERR_PARAM;
    }
    else
    {
        /* Walk the list and enqueue at the end of the TCBs at this priority */
        prev_ptr = next_ptr = *tcb_queue_ptr;
        do
        {
            /* Insert if:
             *   next_ptr = NULL (we're at the head of an empty queue or at the tail)
             *   the next TCB in the list is lower priority than the one we're enqueuing.
             */
            if ((next_ptr == NULL) || (next_ptr->priority > tcb_ptr->priority))
            {
                /* Make this TCB the new listhead */
                if (next_ptr == *tcb_queue_ptr)
                {
                    *tcb_queue_ptr = tcb_ptr;
                    tcb_ptr->prev_tcb = NULL;
                    tcb_ptr->next_tcb = next_ptr;
                    if (next_ptr)
                        next_ptr->prev_tcb = tcb_ptr;
                }
                /* Insert between two TCBs or at the tail */
                else
                {
                    tcb_ptr->prev_tcb = prev_ptr;
                    tcb_ptr->next_tcb = next_ptr;
                    prev_ptr->next_tcb = tcb_ptr;
                    if (next_ptr)
                        next_ptr->prev_tcb = tcb_ptr;
                }

                /* Quit the loop, we've finished inserting */
                break;
            }
            else
            {
                /* Not inserting here, try the next one */
                prev_ptr = next_ptr;
                next_ptr = next_ptr->next_tcb;
            }

        }
        while (prev_ptr != NULL);

        /* Successful */
        status = ATOM_OK;
    }

    return (status);
}


/**
 * \b tcbDequeueHead
 *
 * This is an internal function not for use by application code.
 *
 * Dequeues the highest priority TCB on the queue pointed to by
 * \c tcb_queue_ptr.
 *
 * The TCB will be removed from the queue. Same priority TCBs are dequeued in
 * FIFO order.
 *
 * \c tcb_queue_ptr will be modified by the routine if a TCB is dequeued,
 * as this will be the list head. It is valid for tcb_queue_ptr to point to a
 * NULL pointer, which is the case if the queue is currently empty. In this
 * case the function returns NULL.
 *
 * \b NOTE: Assumes that the caller is already in a critical section.
 *
 * @param[in,out] tcb_queue_ptr Pointer to TCB queue head pointer
 *
 * @return Pointer to highest priority TCB on queue, or NULL if queue empty
 */
ATOM_TCB *tcbDequeueHead (ATOM_TCB **tcb_queue_ptr)
{
    ATOM_TCB *ret_ptr;

    /* Parameter check */
    if (tcb_queue_ptr == NULL)
    {
        /* Return NULL */
        ret_ptr = NULL;
    }
    /* Check for an empty queue */
    else if (*tcb_queue_ptr == NULL)
    {
        /* Return NULL */
        ret_ptr = NULL;
    }
    /* Remove and return the listhead */
    else
    {
        ret_ptr = *tcb_queue_ptr;
        *tcb_queue_ptr = ret_ptr->next_tcb;
        if (*tcb_queue_ptr)
            (*tcb_queue_ptr)->prev_tcb = NULL;
        ret_ptr->next_tcb = ret_ptr->prev_tcb = NULL;
    }

    return (ret_ptr);
}


/**
 * \b tcbDequeueEntry
 *
 * This is an internal function not for use by application code.
 *
 * Dequeues a particular TCB from the queue pointed to by \c tcb_queue_ptr.
 *
 * The TCB will be removed from the queue.
 *
 * \c tcb_queue_ptr may be modified by the routine if the dequeued TCB was
 * the list head. It is valid for tcb_queue_ptr to point to a NULL pointer,
 * which is the case if the queue is currently empty. In this case the
 * function returns NULL.
 *
 * \b NOTE: Assumes that the caller is already in a critical section.
 *
 * @param[in,out] tcb_queue_ptr Pointer to TCB queue head pointer
 * @param[in] tcb_ptr Pointer to TCB to dequeue
 *
 * @return Pointer to the dequeued TCB, or NULL if entry wasn't found
 */
ATOM_TCB *tcbDequeueEntry (ATOM_TCB **tcb_queue_ptr, ATOM_TCB *tcb_ptr)
{
    ATOM_TCB *ret_ptr, *prev_ptr, *next_ptr;

    /* Parameter check */
    if (tcb_queue_ptr == NULL)
    {
        /* Return NULL */
        ret_ptr = NULL;
    }
    /* Check for an empty queue */
    else if (*tcb_queue_ptr == NULL)
    {
        /* Return NULL */
        ret_ptr = NULL;
    }
    /* Find and remove/return the specified entry */
    else
    {
        ret_ptr = NULL;
        prev_ptr = next_ptr = *tcb_queue_ptr;
        while (next_ptr)
        {
            /* Is this entry the one we're looking for? */
            if (next_ptr == tcb_ptr)
            {
                if (next_ptr == *tcb_queue_ptr)
                {
                    /* We're removing the list head */
                    *tcb_queue_ptr = next_ptr->next_tcb;
                    if (*tcb_queue_ptr)
                        (*tcb_queue_ptr)->prev_tcb = NULL;
                }
                else
                {
                    /* We're removing a mid or tail TCB */
                    prev_ptr->next_tcb = next_ptr->next_tcb;
                    if (next_ptr->next_tcb)
                        next_ptr->next_tcb->prev_tcb = prev_ptr;
                }
                ret_ptr = next_ptr;
                ret_ptr->prev_tcb = ret_ptr->next_tcb = NULL;
                break;
            }

            /* Move on to the next in the list */
            prev_ptr = next_ptr;
            next_ptr = next_ptr->next_tcb;
        }
    }

    return (ret_ptr);
}


/**
 * \b tcbDequeuePriority
 *
 * This is an internal function not for use by application code.
 *
 * Dequeues the first TCB of the given priority or higher, from the queue
 * pointed to by \c tcb_queue_ptr. Because the queue is ordered high priority
 * first, we only ever dequeue the list head, if any. If the list head is
 * lower priority than we wish to dequeue, then all following ones will also
 * be lower priority and hence are not parsed.
 *
 * The TCB will be removed from the queue. Same priority TCBs will be dequeued
 * in FIFO order.
 *
 * \c tcb_queue_ptr may be modified by the routine if the dequeued TCB was
 * the list head. It is valid for tcb_queue_ptr to point to a NULL pointer,
 * which is the case if the queue is currently empty. In this case the
 * function returns NULL.
 *
 * \b NOTE: Assumes that the caller is already in a critical section.
 *
 * @param[in,out] tcb_queue_ptr Pointer to TCB queue head pointer
 * @param[in] priority Minimum priority to qualify for dequeue
 *
 * @return Pointer to the dequeued TCB, or NULL if none found within priority
 */
ATOM_TCB *tcbDequeuePriority (ATOM_TCB **tcb_queue_ptr, uint8_t priority)
{
    ATOM_TCB *ret_ptr;

    /* Parameter check */
    if (tcb_queue_ptr == NULL)
    {
        /* Return NULL */
        ret_ptr = NULL;
    }
    /* Check for an empty queue */
    else if (*tcb_queue_ptr == NULL)
    {
        /* Return NULL */
        ret_ptr = NULL;
    }
    /* Check if the list head priority is within our range */
    else if ((*tcb_queue_ptr)->priority <= priority)
    {
       /* Remove the list head */
        ret_ptr = *tcb_queue_ptr;
        *tcb_queue_ptr = (*tcb_queue_ptr)->next_tcb;
        if (*tcb_queue_ptr)
        {
            (*tcb_queue_ptr)->prev_tcb = NULL;
            ret_ptr->next_tcb = NULL;
        }
    }
    else
    {
        /* No higher priority ready threads found */
        ret_ptr = NULL;
    }

    return (ret_ptr);
}
