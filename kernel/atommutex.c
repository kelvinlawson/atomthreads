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
 * Mutex library.
 *
 *
 * This module implements a mutual exclusion library with the following
 * features:
 *
 * \par Flexible blocking APIs
 * Threads which wish to take a mutex lock can choose whether to block,
 * block with timeout, or not block if the mutex is already locked by a
 * different thread.
 *
 * \par Interrupt-safe calls
 * Some APIs can be called from interrupt context, but because a mutex
 * must be owned by a particular thread the lock/unlock calls must be
 * performed by threads only (i.e. it does not make sense to allow
 * calls from interrupt context). All APIs are documented with their
 * capability of being called from interrupt context. Any attempt to
 * make a call which cannot be made from interrupt context will be
 * automatically and safely prevented.
 *
 * \par Priority-based queueing
 * Where multiple threads are blocking on a mutex, they are woken in
 * order of the threads' priorities. Where multiple threads of the same
 * priority are blocking, they are woken in FIFO order.
 *
 * \par Recursive locks
 * A mutex can be locked recursively by the same thread up to a
 * maximum recursion level of 255. An internal count of locks is
 * maintained and the mutex is only released when the count reaches
 * zero (when the thread has been unlocked the same number of times
 * as it was locked). This makes a mutex more suitable for use as
 * mutual exclusions than a semaphore with initial count of 1.
 *
 * \par Thread ownership
 * Once a thread has locked a mutex, only that thread may release the
 * lock. This is another feature which makes the mutex more suitable
 * for mutual exclusion than a semaphore with initial count 1. It
 * prevents programming errors whereby the wrong thread is used to
 * perform the unlock. This cannot be done for semaphores which do not
 * have a concept of ownership (because it must be possible to use them
 * to signal between threads). 
 *
 * \par Smart mutex deletion
 * Where a mutex is deleted while threads are blocking on it, all blocking
 * threads are woken and returned a status code to indicate the reason for
 * being woken.
 *
 *
 * \n <b> Usage instructions: </b> \n
 *
 * All mutex objects must be initialised before use by calling
 * atomMutexCreate(). Once initialised atomMutexGet() and atomMutexPut()
 * are used to lock and unlock the mutex respectively. A mutex may be
 * locked recursively by the same thread, allowing for simplified code
 * structure.
 *
 * While a thread owns the lock on a mutex, no other thread can take the lock.
 * These other threads will block until the mutex is released by the current
 * owner (unless the calling parameters request no blocking, in which case the
 * lock request will return with an error). If a mutex is released while
 * threads are blocking on it, the highest priority thread is woken. Where
 * multiple threads of the same priority are blocking, they are woken in the
 * order in which the threads started blocking. 
 *
 * A mutex which is no longer required can be deleted using atomMutexDelete().
 * This function automatically wakes up any threads which are waiting on the
 * deleted mutex.
 *
 */


#include <stdio.h>
#include "atom.h"
#include "atommutex.h"
#include "atomtimer.h"


/* Local data types */

typedef struct mutex_timer
{
    ATOM_TCB *tcb_ptr;      /* Thread which is suspended with timeout */
    ATOM_MUTEX *mutex_ptr;  /* Mutex the thread is suspended on */
} MUTEX_TIMER;


/* Forward declarations */

static void atomMutexTimerCallback (POINTER cb_data);


/**
 * \b atomMutexCreate
 *
 * Initialises a mutex object.
 *
 * Must be called before calling any other mutex library routines on a
 * mutex. Objects can be deleted later using atomMutexDelete().
 *
 * Does not set the owner of a mutex. atomMutexGet() must be called after
 * creation in order to actually take ownership.
 *
 * Does not allocate storage, the caller provides the mutex object.
 *
 * This function can be called from interrupt context.
 *
 * @param[in] mutex Pointer to mutex object
 *
 * @retval ATOM_OK Success
 * @retval ATOM_ERR_PARAM Bad parameters
 */
uint8_t atomMutexCreate (ATOM_MUTEX *mutex)
{
    uint8_t status;

    /* Parameter check */
    if (mutex == NULL)
    {
        /* Bad mutex pointer */
        status = ATOM_ERR_PARAM;
    }
    else
    {
        /* Start with no owner (unlocked) */
        mutex->owner = NULL;

        /* Reset the initial lock count */
        mutex->count = 0;

        /* Initialise the suspended threads queue */
        mutex->suspQ = NULL;

        /* Successful */
        status = ATOM_OK;
    }

    return (status);
}


/**
 * \b atomMutexDelete
 *
 * Deletes a mutex object.
 *
 * Any threads currently suspended on the mutex will be woken up with
 * return status ATOM_ERR_DELETED. If called at thread context then the
 * scheduler will be called during this function which may schedule in one
 * of the woken threads depending on relative priorities.
 *
 * This function can be called from interrupt context, but loops internally
 * waking up all threads blocking on the mutex, so the potential
 * execution cycles cannot be determined in advance.
 *
 * @param[in] mutex Pointer to mutex object
 *
 * @retval ATOM_OK Success
 * @retval ATOM_ERR_QUEUE Problem putting a woken thread on the ready queue
 * @retval ATOM_ERR_TIMER Problem cancelling a timeout on a woken thread
 */
uint8_t atomMutexDelete (ATOM_MUTEX *mutex)
{
    uint8_t status;
    CRITICAL_STORE;
    ATOM_TCB *tcb_ptr;
    uint8_t woken_threads = FALSE;

    /* Parameter check */
    if (mutex == NULL)
    {
        /* Bad mutex pointer */
        status = ATOM_ERR_PARAM;
    }
    else
    {
        /* Default to success status unless errors occur during wakeup */
        status = ATOM_OK;

        /* Wake up all suspended tasks */
        while (1)
        {
            /* Enter critical region */
            CRITICAL_START ();

            /* Check if any threads are suspended */
            tcb_ptr = tcbDequeueHead (&mutex->suspQ);

            /* A thread is suspended on the mutex */
            if (tcb_ptr)
            {
                /* Return error status to the waiting thread */
                tcb_ptr->suspend_wake_status = ATOM_ERR_DELETED;

                /* Put the thread on the ready queue */
                if (tcbEnqueuePriority (&tcbReadyQ, tcb_ptr) != ATOM_OK)
                {
                    /* Exit critical region */
                    CRITICAL_END ();

                    /* Quit the loop, returning error */
                    status = ATOM_ERR_QUEUE;
                    break;
                }

                /* If there's a timeout on this suspension, cancel it */
                if (tcb_ptr->suspend_timo_cb)
                {
                    /* Cancel the callback */
                    if (atomTimerCancel (tcb_ptr->suspend_timo_cb) != ATOM_OK)
                    {
                        /* Exit critical region */
                        CRITICAL_END ();

                        /* Quit the loop, returning error */
                        status = ATOM_ERR_TIMER;
                        break;
                    }

                    /* Flag as no timeout registered */
                    tcb_ptr->suspend_timo_cb = NULL;

                }

                /* Exit critical region */
                CRITICAL_END ();

                /* Request a reschedule */
                woken_threads = TRUE;
            }

            /* No more suspended threads */
            else
            {
                /* Exit critical region and quit the loop */
                CRITICAL_END ();
                break;
            }
        }

        /* Call scheduler if any threads were woken up */
        if (woken_threads == TRUE)
        {
            /**
             * Only call the scheduler if we are in thread context, otherwise
             * it will be called on exiting the ISR by atomIntExit().
             */
            if (atomCurrentContext())
                atomSched (FALSE);
        }
    }

    return (status);
}


/**
 * \b atomMutexGet
 *
 * Take the lock on a mutex.
 *
 * This takes ownership of a mutex if it is not currently owned. Ownership
 * is held by this thread until a corresponding call to atomMutexPut() by
 * the same thread.
 *
 * Can be called recursively by the original locking thread (owner).
 * Recursive calls are counted, and ownership is not relinquished until
 * the number of unlock (atomMutexPut()) calls by the owner matches the
 * number of lock (atomMutexGet()) calls.
 *
 * No thread other than the owner can lock or unlock the mutex while it is
 * locked by another thread.
 *
 * Depending on the \c timeout value specified the call will do one of
 * the following if the mutex is already locked by another thread:
 *
 * \c timeout == 0 : Call will block until the mutex is available \n
 * \c timeout > 0 : Call will block until available up to the specified timeout \n
 * \c timeout == -1 : Return immediately if mutex is locked by another thread \n
*
 * If the call needs to block and \c timeout is zero, it will block
 * indefinitely until the owning thread calls atomMutexPut() or
 * atomMutexDelete() is called on the mutex.
 *
 * If the call needs to block and \c timeout is non-zero, the call will only
 * block for the specified number of system ticks after which time, if the
 * thread was not already woken, the call will return with \c ATOM_TIMEOUT.
 *
 * If the call would normally block and \c timeout is -1, the call will
 * return immediately with \c ATOM_WOULDBLOCK.
 *
 * This function can only be called from thread context. A mutex has the
 * concept of an owner thread, so it is never valid to make a mutex call
 * from interrupt context when there is no thread to associate with.
 *
 * @param[in] mutex Pointer to mutex object
 * @param[in] timeout Max system ticks to block (0 = forever)
 *
 * @retval ATOM_OK Success
 * @retval ATOM_TIMEOUT Mutex timed out before being woken
 * @retval ATOM_WOULDBLOCK Called with timeout == -1 but count is zero
 * @retval ATOM_ERR_DELETED Mutex was deleted while suspended
 * @retval ATOM_ERR_CONTEXT Not called in thread context and attempted to block
 * @retval ATOM_ERR_PARAM Bad parameter
 * @retval ATOM_ERR_QUEUE Problem putting the thread on the suspend queue
 * @retval ATOM_ERR_TIMER Problem registering the timeout
 * @retval ATOM_ERR_OVF The recursive lock count would have overflowed (>255)
 */
uint8_t atomMutexGet (ATOM_MUTEX *mutex, int32_t timeout)
{
    CRITICAL_STORE;
    uint8_t status;
    MUTEX_TIMER timer_data;
    ATOM_TIMER timer_cb;
    ATOM_TCB *curr_tcb_ptr;

    /* Check parameters */
    if (mutex == NULL)
    {
        /* Bad mutex pointer */
        status = ATOM_ERR_PARAM;
    }
    else
    {
        /* Get the current TCB */
        curr_tcb_ptr = atomCurrentContext();

        /* Protect access to the mutex object and OS queues */
        CRITICAL_START ();

        /**
         * Check we are at thread context. Because mutexes have the concept of
         * owner threads, it is never valid to call here from an ISR,
         * regardless of whether we will block.
         */
        if (curr_tcb_ptr == NULL)
        {
            /* Exit critical region */
            CRITICAL_END ();

            /* Not currently in thread context, can't suspend */
            status = ATOM_ERR_CONTEXT;
        }

        /* Otherwise if mutex is owned by another thread, block the calling thread */
        else if ((mutex->owner != NULL) && (mutex->owner != curr_tcb_ptr))
        {
            /* If called with timeout >= 0, we should block */
            if (timeout >= 0)
            {
                /* Add current thread to the suspend list on this mutex */
                if (tcbEnqueuePriority (&mutex->suspQ, curr_tcb_ptr) != ATOM_OK)
                {
                    /* Exit critical region */
                    CRITICAL_END ();

                    /* There was an error putting this thread on the suspend list */
                    status = ATOM_ERR_QUEUE;
                }
                else
                {
                    /* Set suspended status for the current thread */
                    curr_tcb_ptr->suspended = TRUE;

                    /* Track errors */
                    status = ATOM_OK;

                    /* Register a timer callback if requested */
                    if (timeout)
                    {
                        /* Fill out the data needed by the callback to wake us up */
                        timer_data.tcb_ptr = curr_tcb_ptr;
                        timer_data.mutex_ptr = mutex;

                        /* Fill out the timer callback request structure */
                        timer_cb.cb_func = atomMutexTimerCallback;
                        timer_cb.cb_data = (POINTER)&timer_data;
                        timer_cb.cb_ticks = timeout;

                        /**
                         * Store the timer details in the TCB so that we can
                         * cancel the timer callback if the mutex is put
                         * before the timeout occurs.
                         */
                        curr_tcb_ptr->suspend_timo_cb = &timer_cb;

                        /* Register a callback on timeout */
                        if (atomTimerRegister (&timer_cb) != ATOM_OK)
                        {
                            /* Timer registration failed */
                            status = ATOM_ERR_TIMER;

                            /* Clean up and return to the caller */
                            (void)tcbDequeueEntry (&mutex->suspQ, curr_tcb_ptr);
                            curr_tcb_ptr->suspended = FALSE;
                            curr_tcb_ptr->suspend_timo_cb = NULL;
                        }
                    }

                    /* Set no timeout requested */
                    else
                    {
                        /* No need to cancel timeouts on this one */
                        curr_tcb_ptr->suspend_timo_cb = NULL;
                    }

                    /* Exit critical region */
                    CRITICAL_END ();

                    /* Check no errors have occurred */
                    if (status == ATOM_OK)
                    {
                        /**
                         * Current thread now blocking, schedule in a new
                         * one. We already know we are in thread context
                         * so can call the scheduler from here.
                         */
                        atomSched (FALSE);

                        /**
                         * Normal atomMutexPut() wakeups will set ATOM_OK status,
                         * while timeouts will set ATOM_TIMEOUT and mutex
                         * deletions will set ATOM_ERR_DELETED. */
                        status = curr_tcb_ptr->suspend_wake_status;

                        /**
                         * If we were woken up by another thread relinquishing
                         * the mutex and handing this thread ownership, then
                         * the relinquishing thread will set status to ATOM_OK
                         * and will make this thread the owner. Setting the
                         * owner before waking the thread ensures that no other
                         * thread can preempt and take ownership of the mutex
                         * between this thread being made ready to run, and
                         * actually being scheduled back in here.
                         */
                        if (status == ATOM_OK)
                        {
                            /**
                             * Since this thread has just gained ownership, the
                             * lock count is zero and should be incremented
                             * once for this call.
                             */
                            mutex->count++;
                        }
                    }
                }
            }
            else
            {
                /* timeout == -1, requested not to block and mutex is owned by another thread */
                CRITICAL_END();
                status = ATOM_WOULDBLOCK;
            }
        }
        else
        {
            /* Thread is not owned or is owned by us, we can claim ownership */

            /* Increment the lock count, checking for count overflow */
            if (mutex->count == 255)
            {
                /* Don't increment, just return error status */
                status = ATOM_ERR_OVF;
            }
            else
            {
                /* Increment the count and return to the calling thread */
                mutex->count++;

                /* If the mutex is not locked, mark the calling thread as the new owner */
                if (mutex->owner == NULL)
                {
                    mutex->owner = curr_tcb_ptr;
                }

                /* Successful */
                status = ATOM_OK;
            }

            /* Exit critical region */
            CRITICAL_END ();
        }
    }

    return (status);
}


/**
 * \b atomMutexPut
 *
 * Give back the lock on a mutex.
 *
 * This checks that the mutex is owned by the calling thread, and decrements
 * the recursive lock count. Once the lock count reaches zero, the lock is
 * considered relinquished and no longer owned by this thread.
 *
 * If the lock is relinquished and there are threads blocking on the mutex, the
 * call will wake up the highest priority thread suspended. Only one thread is
 * woken per call to atomMutexPut(). If multiple threads of the same priority
 * are suspended, they are woken in order of suspension (FIFO).
 *
 * This function can only be called from thread context. A mutex has the
 * concept of an owner thread, so it is never valid to make a mutex call
 * from interrupt context when there is no thread to associate with.
 *
 * @param[in] mutex Pointer to mutex object
 *
 * @retval ATOM_OK Success
 * @retval ATOM_ERR_PARAM Bad parameter
 * @retval ATOM_ERR_QUEUE Problem putting a woken thread on the ready queue
 * @retval ATOM_ERR_TIMER Problem cancelling a timeout for a woken thread
 * @retval ATOM_ERR_OWNERSHIP Attempt to unlock mutex not owned by this thread
 */
uint8_t atomMutexPut (ATOM_MUTEX * mutex)
{
    uint8_t status;
    CRITICAL_STORE;
    ATOM_TCB *tcb_ptr, *curr_tcb_ptr;

    /* Check parameters */
    if (mutex == NULL)
    {
        /* Bad mutex pointer */
        status = ATOM_ERR_PARAM;
    }
    else
    {
        /* Get the current TCB */
        curr_tcb_ptr = atomCurrentContext();

        /* Protect access to the mutex object and OS queues */
        CRITICAL_START ();

        /* Check if the calling thread owns this mutex */
        if (mutex->owner != curr_tcb_ptr)
        {
            /* Exit critical region */
            CRITICAL_END ();

            /* Attempt to unlock by non-owning thread */
            status = ATOM_ERR_OWNERSHIP;
        }
        else
        {
            /* Lock is owned by this thread, decrement the recursive lock count */
            mutex->count--;

            /* Once recursive lock count reaches zero, we relinquish ownership */
            if (mutex->count == 0)
            {
                /* Relinquish ownership */
                mutex->owner = NULL;

                /* If any threads are blocking on this mutex, wake them now */
                if (mutex->suspQ)
                {
                    /**
                     * Threads are woken up in priority order, with a FIFO system
                     * used on same priority threads. We always take the head,
                     * ordering is taken care of by an ordered list enqueue.
                     */
                    tcb_ptr = tcbDequeueHead (&mutex->suspQ);
                    if (tcbEnqueuePriority (&tcbReadyQ, tcb_ptr) != ATOM_OK)
                    {
                        /* Exit critical region */
                        CRITICAL_END ();

                        /* There was a problem putting the thread on the ready queue */
                        status = ATOM_ERR_QUEUE;
                    }
                    else
                    {
                        /* Set OK status to be returned to the waiting thread */
                        tcb_ptr->suspend_wake_status = ATOM_OK;

                        /* Set this thread as the new owner of the mutex */
                        mutex->owner = tcb_ptr;

                        /* If there's a timeout on this suspension, cancel it */
                        if ((tcb_ptr->suspend_timo_cb != NULL)
                            && (atomTimerCancel (tcb_ptr->suspend_timo_cb) != ATOM_OK))
                        {
                            /* There was a problem cancelling a timeout on this mutex */
                            status = ATOM_ERR_TIMER;
                        }
                        else
                        {
                            /* Flag as no timeout registered */
                            tcb_ptr->suspend_timo_cb = NULL;

                            /* Successful */
                            status = ATOM_OK;
                        }

                        /* Exit critical region */
                        CRITICAL_END ();

                        /**
                         * The scheduler may now make a policy decision to
                         * thread switch. We already know we are in thread
                         * context so can call the scheduler from here.
                         */
                        atomSched (FALSE);
                    }
                }
                else
                {
                    /**
                     * Relinquished ownership and no threads waiting.
                     * Nothing to do.
                     */

                    /* Exit critical region */
                    CRITICAL_END ();

                    /* Successful */
                    status = ATOM_OK;
                }
            }
            else
            {
                /**
                 * Decremented lock but still retain ownership due to
                 * recursion. Nothing to do.
                 */

                /* Exit critical region */
                CRITICAL_END ();

                /* Successful */
                status = ATOM_OK;
            }
        }
    }

    return (status);
}


/**
 * \b atomMutexTimerCallback
 *
 * This is an internal function not for use by application code.
 *
 * Timeouts on suspended threads are notified by the timer system through
 * this generic callback. The timer system calls us back with a pointer to
 * the relevant \c MUTEX_TIMER object which is used to retrieve the
 * mutex details.
 *
 * @param[in] cb_data Pointer to a MUTEX_TIMER object
 */
static void atomMutexTimerCallback (POINTER cb_data)
{
    MUTEX_TIMER *timer_data_ptr;
    CRITICAL_STORE;

    /* Get the MUTEX_TIMER structure pointer */
    timer_data_ptr = (MUTEX_TIMER *)cb_data;

    /* Check parameter is valid */
    if (timer_data_ptr)
    {
        /* Enter critical region */
        CRITICAL_START ();

        /* Set status to indicate to the waiting thread that it timed out */
        timer_data_ptr->tcb_ptr->suspend_wake_status = ATOM_TIMEOUT;

        /* Flag as no timeout registered */
        timer_data_ptr->tcb_ptr->suspend_timo_cb = NULL;

        /* Remove this thread from the mutex's suspend list */
        (void)tcbDequeueEntry (&timer_data_ptr->mutex_ptr->suspQ, timer_data_ptr->tcb_ptr);

        /* Put the thread on the ready queue */
        (void)tcbEnqueuePriority (&tcbReadyQ, timer_data_ptr->tcb_ptr);

        /* Exit critical region */
        CRITICAL_END ();

        /**
         * Note that we don't call the scheduler now as it will be called
         * when we exit the ISR by atomIntExit().
         */
    }
}
