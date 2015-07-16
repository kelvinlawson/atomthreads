/*
 * Copyright (c) 2015, Stefan Petersen. All rights reserved.
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
 * Event library.
 *
 *
 * This module implements event handling, which is a very simple way of
 * waiting for events, for instance from a timer or an interrupt.
 *
 * \par Zero size cost
 * The event register is stored in the TCB of every thread in atomThreads.
 * Disadvantage is that every TCB will contain the event register whether
 * it is used or not. There is also no queue to the event, therefore the
 * signaller must know the TCB of the thread to call.
 *
 * \par No initialization before use
 * Having the event register int the TCB means you don't have to allocate
 * and initialize anything before using event.
 *
 * \par Interrupt-safe calls
 * atomEventWait() for events generally blocks while waiting for
 * atomEventSignal() to signal, but this can be controlled at call.
 * atomEventSignal() to does not block at all. If the thread is not waiting
 * for an event, the events signalled is stored in the event register of the
 * TCB.
 *
 * \par Thread to thread signalling
 * Signalling can only go to one task at a time, so several tasks can not lock
 * on a particual event.
 *
 * \par Signalling events can not lock
 * When event(s) are signalled with atomEventSignal() two things can happen.
 * -# If the thread is not waiting for an event, the event mask will be stored
 * in the threads event register
 * -# If the thread is waiting for an event the thread will be put back on the
 * ready queue and will be scheduled in as soon as its priority allows.
 * Please note that when atomEventWait() returns it will clear the event
 * register in the TCB.
 *
 * \par Events
 * Events is a bitfields with implementation defined size and every
 * bit in that word can signify an event. That makes it possible to wait for
 * several events at the same time.
 * Signalling of events is per thread, so the signaller must know the TCB of the
 * thread to signal before hand (not usually a problem).
 *
 * \n <b> Usage instructions: </b> \n
 *
 * The size of this bitfield is implementation dependent and is defined in
 * atomport.h. Since the only data that is needed is stored in the TCB it
 * is not necessary to allocate any memory nor special initalizing.
 *
 * The TCB of a waiting process is removed from the ready queue, but then not
 * stored in any new queue while waiting for the event. The only way to get
 * the TCB back into the ready queue when signalling is to actually send the
 * TCB in the signalling call.
 *
 */

#include "atomevent.h"

/* Forward declarations */

static void atomEventTimerCallback (POINTER cb_data);

/**
 * \b atomEventSignal
 *
 * Signal events to a particual thread.
 *
 * Signals an event to a thread given by the TCB pointer waiting in
 * an atomEventWait() with the same bits set in the event mask.
 * If the thread is not waiting, then the events signalled will be stored
 * int the event register of the TCB until the atomEventWait() is called.
 *
 * @param[in] curr_tcb_ptr Pointer to TCB which is to be signalled
 * @param[in] events Bits to signal to thread.
 *
 * @retval ATOM_OK Success
 * @retval ATOM_ERR_PARAM Bad parameter
 * @retval ATOM_ERR_QUEUE Problem putting the thread back on the ready queue
 */
uint8_t atomEventSignal (ATOM_TCB *curr_tcb_ptr, ATOM_EVENTS events)
{
    uint8_t status;
    CRITICAL_STORE;

   /* Check parameters */
    if (curr_tcb_ptr == NULL)
    {
        /* Bad TCB pointer */
        status = ATOM_ERR_PARAM;
    }
    else
    {
        /* Protect access to the tcb object and OS queues */
        CRITICAL_START ();

        if (curr_tcb_ptr->suspended)
        {
            if (curr_tcb_ptr->events & events)
            {
                /* Set the events to be signalled when waiting thread is to be
                 * scheduled in again. */
                curr_tcb_ptr->events = curr_tcb_ptr->events & events;
                /* Put this thread on the ready queue */
                if (tcbEnqueuePriority (&tcbReadyQ, curr_tcb_ptr) != ATOM_OK)
                {
                    /* Queue-related error */
                    status = ATOM_ERR_QUEUE;
                }
                else
                {
                    curr_tcb_ptr->suspended = FALSE;
                    status = curr_tcb_ptr->suspend_wake_status; /* FIX Check this */

                    /* If there's a timeout on this suspension, cancel it */
                    if (curr_tcb_ptr->suspend_timo_cb)
                    {
                        /* Cancel the callback */
                        if (atomTimerCancel (curr_tcb_ptr->suspend_timo_cb) != ATOM_OK)
                        {
                            /* Return timer error */
                            status = ATOM_ERR_TIMER;
                        }
                        /* Flag has no timeout registered */
                        curr_tcb_ptr->suspend_timo_cb = NULL;
                    }
                }
            }
            else
            {
                /* No requested bits signalled which probably should be
                   considered an error in parameters. */
                status = ATOM_ERR_PARAM;
            }
            /* Exit critical region */
            CRITICAL_END ();
            if (status == ATOM_OK)
            {
                if (atomCurrentContext())
                {
                    atomSched(FALSE);
                }
            }
        }
        else
        {
            /* TCB is not waiting for events, we mask in signalled events */
            curr_tcb_ptr->events = events;
            /* Exit critical region */
            CRITICAL_END ();
            status = ATOM_OK;
        }

    }
    return status;
}


/**
 * \b atomEventWait
 *
 * Wait for events to be signalled.
 *
 * Depending on the \c timeout value specified the call will do one of
 * the following:
 *
 * \c timeout == 0 : Call will block until it is signalled \n
 * \c timeout > 0 : Call will block until available up to the specified timeout \n
 *
 * If the call needs to block and \c timeout is zero, it will block
 * indefinitely until the someone will call atomEventSignal() with one
 * or several of the event mask bits set.
 *
 * If the call needs to block and \c timeout is non-zero, the call will o1nly
 * block for the specified number of system ticks after which time, if the
 * thread was not already woken, the call will return with \c 0.
 *
 * WaitEvent clears the event memory after an event has occurred.
 * WaitSingleEvent does not clear event memory.
 *
 * @param[in] event_mask Bits to signal to thread.
 * @param[out] events Pointer to bits in the event that is signalled or
 *                    zero if timed out.
 * @param[in] timeout Max system ticks to block (0 = forever / -1 = non-blocking)
 *
 * @retval ATOM_OK Success
 * @retval ATOM_TIMEOUT Event timed out before being woken
 * @retval ATOM_WOULDBLOCK Called with timeout == -1 but no event awaits
 * @retval ATOM_ERR_CONTEXT Not called in thread context and attempted to block
 * @retval ATOM_ERR_PARAM Bad parameter
 * @retval ATOM_ERR_QUEUE Problem putting the thread on the suspend queue
 * @retval ATOM_ERR_TIMER Problem registering the timeout
 */
uint8_t atomEventWait (ATOM_EVENTS event_mask, ATOM_EVENTS *events, int32_t timeout)
{
    uint8_t status;
    ATOM_TCB *curr_tcb_ptr;
    ATOM_TIMER timer_cb;
    CRITICAL_STORE;

    /* Get the current TCB */
    curr_tcb_ptr = atomCurrentContext();

    if (event_mask == 0L)
    {
        status = ATOM_ERR_PARAM;
    }
    else
    {
        CRITICAL_START ();
        /* If already event bits are set in the TCBs event register */
        if (event_mask & curr_tcb_ptr->events)
        {
            *events = curr_tcb_ptr->events & event_mask;
            CRITICAL_END ();
            status = ATOM_OK;
        }
        else
        {
            if (timeout < 0)
            {
                CRITICAL_END ();
                *events = 0;
                status = ATOM_WOULDBLOCK;
            }
            else
            {
                /* Remember the event mask we are waiting for */
                curr_tcb_ptr->events = event_mask;
                /* Suspend ourselves */
                curr_tcb_ptr->suspended = TRUE;
                curr_tcb_ptr->suspend_wake_status = ATOM_OK;
                if (timeout)
                {
                    /* Fill out the timer callback request structure */
                    timer_cb.cb_func = atomEventTimerCallback;
                    timer_cb.cb_data = (POINTER)curr_tcb_ptr;
                    timer_cb.cb_ticks = timeout;

                    /**
                     * Store the timer details in the TCB so that we can
                     * cancel the timer callback if a event is signalled
                     * before the timeout occurs.
                     */
                    curr_tcb_ptr->suspend_timo_cb = &timer_cb;

                    /* Register a callback on timeout */
                    if (atomTimerRegister (&timer_cb) != ATOM_OK)
                    {
                        /* Timer registration failed */
                        status = ATOM_ERR_TIMER;

                        /* Clean up and return to the caller */
                        curr_tcb_ptr->suspended = FALSE;
                        curr_tcb_ptr->suspend_timo_cb = NULL;
                    }
                    else
                    {
                        status = ATOM_OK;
                    }
                }

                /* Set no timeout requested */
                else
                {
                    /* No need to cancel timeouts on this one */
                    curr_tcb_ptr->suspend_timo_cb = NULL;
                    status = ATOM_OK;
                }
                /**
                 * Only call the scheduler if we are in thread context,
                 * otherwise it will be called on exiting the ISR by
                 * atomIntExit().
                 */
                CRITICAL_END ();
                if ((status == ATOM_OK) && atomCurrentContext())
                {
                    atomSched (FALSE);
                    /* Restore returned events, clears event memory and update
                       status after context switch */
                    *events = curr_tcb_ptr->events;
                    curr_tcb_ptr->events = 0;
                    status = curr_tcb_ptr->suspend_wake_status;
                }
                else
                {
                    status = ATOM_ERR_CONTEXT;
                }
            }
        }
    }
    return status;
}


/**
 * \b atomEventTimerCallback
 *
 * This is an internal function not for use by application code.
 *
 * Timeouts on suspended threads are notified by the timer system through
 * this generic callback. The timer system calls us back with a pointer to
 * the relevant \c ATOM_TCB object which is used to retrieve the
 * event details.
 *
 * @param[in] cb_data Pointer to a ATOM_TCB object
 */
static void atomEventTimerCallback (POINTER cb_data)
{
    ATOM_TCB *curr_tcb_ptr;
    CRITICAL_STORE;

    /* Get the  pointer */
    curr_tcb_ptr = (ATOM_TCB *)cb_data;

    /* Check parameter is valid */
    if (curr_tcb_ptr)
    {
        /* Enter critical region */
        CRITICAL_START ();

        /* Set status to indicate to the waiting thread that it timed out */
        curr_tcb_ptr->suspend_wake_status = ATOM_TIMEOUT;

        /* Flag as no timeout registered */
        curr_tcb_ptr->suspend_timo_cb = NULL;

        /* Put the thread on the ready queue */
        (void)tcbEnqueuePriority (&tcbReadyQ, curr_tcb_ptr);

        /* Exit critical region */
        CRITICAL_END ();

        /**
         * Note that we don't call the scheduler now as it will be called
         * when we exit the ISR by atomIntExit().
         */
    }
}
