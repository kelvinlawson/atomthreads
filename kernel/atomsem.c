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
 * Semaphore library.
 *
 *
 * This module implements a counting semaphore library with the following
 * features:
 *
 * \par Flexible blocking APIs
 * Threads which wish to decrement a semaphore can choose whether to block,
 * block with timeout, or not block if the semaphore has reached zero.
 *
 * \par Interrupt-safe calls
 * All APIs can be called from interrupt context. Any calls which could
 * potentially block have optional parameters to prevent blocking if you
 * wish to call them from interrupt context. Any attempt to make a call
 * which would block from interrupt context will be automatically and
 * safely prevented.
 *
 * \par Priority-based queueing
 * Where multiple threads are blocking on a semaphore, they are woken in
 * order of the threads' priorities. Where multiple threads of the same
 * priority are blocking, they are woken in FIFO order.
 *
 * \par Count up to 255
 * Semaphore counts can be initialised and incremented up to a maximum of 255.
 *
 * \par Smart semaphore deletion
 * Where a semaphore is deleted while threads are blocking on it, all blocking
 * threads are woken and returned a status code to indicate the reason for
 * being woken.
 *
 *
 * \n <b> Usage instructions: </b> \n
 *
 * All semaphore objects must be initialised before use by calling
 * atomSemCreate(). Once initialised atomSemGet() and atomSemPut() are used to
 * decrement and increment the semaphore count respectively.
 *
 * If a semaphore count reaches zero, further calls to atomSemGet() will block
 * the calling thread (unless the calling parameters request no blocking). If
 * a call is made to atomSemPut() while threads are blocking on a zero-count
 * semaphore, the highest priority thread is woken. Where multiple threads of
 * the same priority are blocking, they are woken in the order in which the
 * threads started blocking. 
 *
 * A semaphore which is no longer required can be deleted using
 * atomSemDelete(). This function automatically wakes up any threads which are
 * waiting on the deleted semaphore.
 *
 *
 * \n <b> Notes: </b> \n
 *
 * Note that those considering using a semaphore initialised to 1 for mutual
 * exclusion purposes may wish to investigate the mutex library available in
 * Atomthreads.
 *
 */


#include <stdio.h>
#include "atom.h"
#include "atomsem.h"
#include "atomtimer.h"


/* Local data types */

typedef struct sem_timer
{
    ATOM_TCB *tcb_ptr;  /* Thread which is suspended with timeout */
    ATOM_SEM *sem_ptr;  /* Semaphore the thread is suspended on */
} SEM_TIMER;


/* Forward declarations */

static void atomSemTimerCallback (POINTER cb_data);


/**
 * \b atomSemCreate
 *
 * Initialises a semaphore object.
 *
 * Must be called before calling any other semaphore library routines on a
 * semaphore. Objects can be deleted later using atomSemDelete().
 *
 * Does not allocate storage, the caller provides the semaphore object.
 *
 * This function can be called from interrupt context.
 *
 * @param[in] sem Pointer to semaphore object
 * @param[in] initial_count Initial count value
 *
 * @retval ATOM_OK Success
 * @retval ATOM_ERR_PARAM Bad parameters
 */
uint8_t atomSemCreate (ATOM_SEM *sem, uint8_t initial_count)
{
    uint8_t status;

    /* Parameter check */
    if (sem == NULL)
    {
        /* Bad semaphore pointer */
        status = ATOM_ERR_PARAM;
    }
    else
    {
        /* Set the initial count */
        sem->count = initial_count;

        /* Initialise the suspended threads queue */
        sem->suspQ = NULL;

        /* Successful */
        status = ATOM_OK;
    }

    return (status);
}


/**
 * \b atomSemDelete
 *
 * Deletes a semaphore object.
 *
 * Any threads currently suspended on the semaphore will be woken up with
 * return status ATOM_ERR_DELETED. If called at thread context then the
 * scheduler will be called during this function which may schedule in one
 * of the woken threads depending on relative priorities.
 *
 * This function can be called from interrupt context, but loops internally
 * waking up all threads blocking on the semaphore, so the potential
 * execution cycles cannot be determined in advance.
 *
 * @param[in] sem Pointer to semaphore object
 *
 * @retval ATOM_OK Success
 * @retval ATOM_ERR_QUEUE Problem putting a woken thread on the ready queue
 * @retval ATOM_ERR_TIMER Problem cancelling a timeout on a woken thread
 */
uint8_t atomSemDelete (ATOM_SEM *sem)
{
    uint8_t status;
    CRITICAL_STORE;
    ATOM_TCB *tcb_ptr;
    uint8_t woken_threads = FALSE;

    /* Parameter check */
    if (sem == NULL)
    {
        /* Bad semaphore pointer */
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
            tcb_ptr = tcbDequeueHead (&sem->suspQ);

            /* A thread is suspended on the semaphore */
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
 * \b atomSemGet
 *
 * Perform a get operation on a semaphore.
 *
 * This decrements the current count value for the semaphore and returns.
 * If the count value is already zero then the call will block until the
 * count is incremented by another thread, or until the specified \c timeout
 * is reached. Blocking threads will also be woken if the semaphore is
 * deleted by another thread while blocking.
 *
 * Depending on the \c timeout value specified the call will do one of
 * the following if the count value is zero:
 *
 * \c timeout == 0 : Call will block until the count is non-zero \n
 * \c timeout > 0 : Call will block until non-zero up to the specified timeout \n
 * \c timeout == -1 : Return immediately if the count is zero \n
 *
 * If the call needs to block and \c timeout is zero, it will block
 * indefinitely until atomSemPut() or atomSemDelete() is called on the
 * semaphore.
 *
 * If the call needs to block and \c timeout is non-zero, the call will only
 * block for the specified number of system ticks after which time, if the
 * thread was not already woken, the call will return with \c ATOM_TIMEOUT.
 *
 * If the call would normally block and \c timeout is -1, the call will
 * return immediately with \c ATOM_WOULDBLOCK.
 *
 * This function can only be called from interrupt context if the \c timeout
 * parameter is -1 (in which case it does not block).
 *
 * @param[in] sem Pointer to semaphore object
 * @param[in] timeout Max system ticks to block (0 = forever)
 *
 * @retval ATOM_OK Success
 * @retval ATOM_TIMEOUT Semaphore timed out before being woken
 * @retval ATOM_WOULDBLOCK Called with timeout == -1 but count is zero
 * @retval ATOM_ERR_DELETED Semaphore was deleted while suspended
 * @retval ATOM_ERR_CONTEXT Not called in thread context and attempted to block
 * @retval ATOM_ERR_PARAM Bad parameter
 * @retval ATOM_ERR_QUEUE Problem putting the thread on the suspend queue
 * @retval ATOM_ERR_TIMER Problem registering the timeout
 */
uint8_t atomSemGet (ATOM_SEM *sem, int32_t timeout)
{
    CRITICAL_STORE;
    uint8_t status;
    SEM_TIMER timer_data;
    ATOM_TIMER timer_cb;
    ATOM_TCB *curr_tcb_ptr;

    /* Check parameters */
    if (sem == NULL)
    {
        /* Bad semaphore pointer */
        status = ATOM_ERR_PARAM;
    }
    else
    {
        /* Protect access to the semaphore object and OS queues */
        CRITICAL_START ();

        /* If count is zero, block the calling thread */
        if (sem->count == 0)
        {
            /* If called with timeout >= 0, we should block */
            if (timeout >= 0)
            {
                /* Count is zero, block the calling thread */

                /* Get the current TCB */
                curr_tcb_ptr = atomCurrentContext();

                /* Check we are actually in thread context */
                if (curr_tcb_ptr)
                {
                    /* Add current thread to the suspend list on this semaphore */
                    if (tcbEnqueuePriority (&sem->suspQ, curr_tcb_ptr) != ATOM_OK)
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
                            timer_data.sem_ptr = sem;

                            /* Fill out the timer callback request structure */
                            timer_cb.cb_func = atomSemTimerCallback;
                            timer_cb.cb_data = (POINTER)&timer_data;
                            timer_cb.cb_ticks = timeout;

                            /**
                             * Store the timer details in the TCB so that we can
                             * cancel the timer callback if the semaphore is put
                             * before the timeout occurs.
                             */
                            curr_tcb_ptr->suspend_timo_cb = &timer_cb;

                            /* Register a callback on timeout */
                            if (atomTimerRegister (&timer_cb) != ATOM_OK)
                            {
                                /* Timer registration failed */
                                status = ATOM_ERR_TIMER;

                                /* Clean up and return to the caller */
                                (void)tcbDequeueEntry (&sem->suspQ, curr_tcb_ptr);
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
                             * Normal atomSemPut() wakeups will set ATOM_OK status,
                             * while timeouts will set ATOM_TIMEOUT and semaphore
                             * deletions will set ATOM_ERR_DELETED.
                             */
                            status = curr_tcb_ptr->suspend_wake_status;

                            /**
                             * If we have been woken up with ATOM_OK then
                             * another thread incremented the semaphore and
                             * handed control to this thread. In theory the
                             * the posting thread increments the counter and
                             * as soon as this thread wakes up we decrement
                             * the counter here, but to prevent another
                             * thread preempting this thread and decrementing
                             * the semaphore before this section was
                             * scheduled back in, we emulate the increment
                             * and decrement by not incrementing in the
                             * atomSemPut() and not decrementing here. The
                             * count remains zero throughout preventing other
                             * threads preempting before we decrement the
                             * count again.
                             */

                        }
                    }
                }
                else
                {
                    /* Exit critical region */
                    CRITICAL_END ();

                    /* Not currently in thread context, can't suspend */
                    status = ATOM_ERR_CONTEXT;
                }
            }
            else
            {
                /* timeout == -1, requested not to block and count is zero */
                CRITICAL_END();
                status = ATOM_WOULDBLOCK;
            }
        }
        else
        {
            /* Count is non-zero, just decrement it and return to calling thread */
            sem->count--;

            /* Exit critical region */
            CRITICAL_END ();

            /* Successful */
            status = ATOM_OK;
        }
    }

    return (status);
}


/**
 * \b atomSemPut
 *
 * Perform a put operation on a semaphore.
 *
 * This increments the current count value for the semaphore and returns.
 *
 * If the count value was previously zero and there are threads blocking on the
 * semaphore, the call will wake up the highest priority thread suspended. Only
 * one thread is woken per call to atomSemPut(). If multiple threads of the
 * same priority are suspended, they are woken in order of suspension (FIFO).
 *
 * This function can be called from interrupt context.
 *
 * @param[in] sem Pointer to semaphore object
 *
 * @retval ATOM_OK Success
 * @retval ATOM_ERR_OVF The semaphore count would have overflowed (>255)
 * @retval ATOM_ERR_PARAM Bad parameter
 * @retval ATOM_ERR_QUEUE Problem putting a woken thread on the ready queue
 * @retval ATOM_ERR_TIMER Problem cancelling a timeout for a woken thread
 */
uint8_t atomSemPut (ATOM_SEM * sem)
{
    uint8_t status;
    CRITICAL_STORE;
    ATOM_TCB *tcb_ptr;

    /* Check parameters */
    if (sem == NULL)
    {
        /* Bad semaphore pointer */
        status = ATOM_ERR_PARAM;
    }
    else
    {
        /* Protect access to the semaphore object and OS queues */
        CRITICAL_START ();

        /* If any threads are blocking on the semaphore, wake up one */
        if (sem->suspQ)
        {
            /**
             * Threads are woken up in priority order, with a FIFO system
             * used on same priority threads. We always take the head,
             * ordering is taken care of by an ordered list enqueue.
             */
            tcb_ptr = tcbDequeueHead (&sem->suspQ);
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

                /* If there's a timeout on this suspension, cancel it */
                if ((tcb_ptr->suspend_timo_cb != NULL)
                    && (atomTimerCancel (tcb_ptr->suspend_timo_cb) != ATOM_OK))
                {
                    /* There was a problem cancelling a timeout on this semaphore */
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
                 * The scheduler may now make a policy decision to thread
                 * switch if we are currently in thread context. If we are
                 * in interrupt context it will be handled by atomIntExit().
                 */
                if (atomCurrentContext())
                    atomSched (FALSE);
            }
        }

        /* If no threads waiting, just increment the count and return */
        else
        {
            /* Check for count overflow */
            if (sem->count == 255)
            {
                /* Don't increment, just return error status */
                status = ATOM_ERR_OVF;
            }
            else
            {
                /* Increment the count and return success */
                sem->count++;
                status = ATOM_OK;
            }

            /* Exit critical region */
            CRITICAL_END ();
        }
    }

    return (status);
}


/**
 * \b atomSemResetCount
 *
 * Set a new count value on a semaphore.
 *
 * Care must be taken when using this function, as there may be threads
 * suspended on the semaphore. In general it should only be used once a
 * semaphore is out of use.
 *
 * This function can be called from interrupt context.
 *
 * @param[in] sem Pointer to semaphore object
 * @param[in] count New count value
 *
 * @retval ATOM_OK Success
 * @retval ATOM_ERR_PARAM Bad parameter
 */
uint8_t atomSemResetCount (ATOM_SEM *sem, uint8_t count)
{
    uint8_t status;

    /* Parameter check */
    if (sem == NULL)
    {
        /* Bad semaphore pointer */
        status = ATOM_ERR_PARAM;
    }
    else
    {
        /* Set the count */
        sem->count = count;

        /* Successful */
        status = ATOM_OK;
    }

    return (status);

}


/**
 * \b atomSemTimerCallback
 *
 * This is an internal function not for use by application code.
 *
 * Timeouts on suspended threads are notified by the timer system through
 * this generic callback. The timer system calls us back with a pointer to
 * the relevant \c SEM_TIMER object which is used to retrieve the
 * semaphore details.
 *
 * @param[in] cb_data Pointer to a SEM_TIMER object
 */
static void atomSemTimerCallback (POINTER cb_data)
{
    SEM_TIMER *timer_data_ptr;
    CRITICAL_STORE;

    /* Get the SEM_TIMER structure pointer */
    timer_data_ptr = (SEM_TIMER *)cb_data;

    /* Check parameter is valid */
    if (timer_data_ptr)
    {
        /* Enter critical region */
        CRITICAL_START ();

        /* Set status to indicate to the waiting thread that it timed out */
        timer_data_ptr->tcb_ptr->suspend_wake_status = ATOM_TIMEOUT;

        /* Flag as no timeout registered */
        timer_data_ptr->tcb_ptr->suspend_timo_cb = NULL;

        /* Remove this thread from the semaphore's suspend list */
        (void)tcbDequeueEntry (&timer_data_ptr->sem_ptr->suspQ, timer_data_ptr->tcb_ptr);

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
