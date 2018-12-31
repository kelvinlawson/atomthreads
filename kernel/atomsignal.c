/*
 * Copyright (c) 2018, Mike Yu. All rights reserved.
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

#include "atomsignal.h"

/* Forward declarations */

static void atomSignalTimerCallback(POINTER cb_date);

/**
 * \b atomSignalWait
 *
 * Wait for one or more Signal Flags to become signaled for the current \b RUNNING thread.
 *
 * @param[in] signals       wait until all specified signal flags set or 0 for any single signal flag
 * @param[in] timeout       Max system ticks to block (0 = forever / -1 = non-blocking)
 *
 * @retval ATOM_SIGNAL
 */
ATOM_SIGNAL atomSignalWait(uint16_t signals, int32_t timeout)
{
    ATOM_SIGNAL ret;
    ATOM_TCB *curr_tcb_ptr;
    ATOM_TIMER timer_cb;
    CRITICAL_STORE;

    /* Get the current TCB */
    curr_tcb_ptr = atomCurrentContext();
    if (curr_tcb_ptr == NULL)
    {
        ret.status = ATOM_ERR_CONTEXT;
        return ret;
    }

    if (signals & (0xFFFF << atomFeature_Signals))
    {
        ret.status = ATOM_ERR_PARAM;
        return ret;
    }
    else
    {
        CRITICAL_START ();

        if (signals != 0)
        {
            if ((curr_tcb_ptr->events & signals) == signals)
            {
                curr_tcb_ptr->events &= ~signals;
                CRITICAL_END ();
                ret.status = ATOM_OK;
                ret.value = signals;
                return ret;
            }
            curr_tcb_ptr->waits = signals;
        }
        else
        {
            if (curr_tcb_ptr->events)
            {
                ret.value = curr_tcb_ptr->events;
                curr_tcb_ptr->events = 0;
                CRITICAL_END ();
                ret.status = ATOM_OK;
                return ret;
            }
            curr_tcb_ptr->waits = 0xFFFF;
        }

        if (timeout < 0)
        {
            CRITICAL_END ();
            ret.status = ATOM_TIMEOUT;
            return ret;
        }
        else
        {
            /* Suspend ourselves */
            curr_tcb_ptr->suspended = TRUE;
            curr_tcb_ptr->suspend_wake_status = ATOM_OK;
            if (timeout)
            {
                /* Fill out the timer callback request structure */
                timer_cb.cb_func = atomSignalTimerCallback;
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
                    ret.status = ATOM_ERR_TIMER;

                    /* Clean up and return to the caller */
                    curr_tcb_ptr->suspended = FALSE;
                    curr_tcb_ptr->suspend_timo_cb = NULL;
                }
                else
                {
                    ret.status = ATOM_OK;
                }
            }
            else
            {
                /* No need to cancel timeouts on this one */
                curr_tcb_ptr->suspend_timo_cb = NULL;
                ret.status = ATOM_OK;
            }
            /**
             * Only call the scheduler if we are in thread context,
             * otherwise it will be called on exiting the ISR by
             * atomIntExit().
             */
            CRITICAL_END ();
            if ((ret.status == ATOM_OK))
            {
                atomSched (FALSE);
                /* Restore returned events, clears event memory and update
                   status after context switch */
                ret.status = curr_tcb_ptr->suspend_wake_status;
                if (ret.status == ATOM_OK)
                {
                    if (signals != 0)
                    {
                        curr_tcb_ptr->events &= ~signals;
                        ret.value = signals;
                    }
                    else
                    {
                        ret.value = curr_tcb_ptr->events;
                        curr_tcb_ptr->events = 0;
                    }
                }
            }
        }
    }
    return ret;
}

/**
 * \b atomSignalSet
 *
 * Set the specified Signal Flags of an active thread.
 *
 * Set the specified Signal Flags to a thread given by the TCB pointer waiting in
 * an atomSignalWait() with the same bits set in the Event Flags.
 * If the thread is not waiting, then the signals signalled will be stored
 * in the event register of the TCB until the atomSignalWait() is called.
 *
 * @param[in] curr_tcb_ptr Pointer to TCB which is to be signalled
 * @param[in] signals signal flags
 *
 * @retval ATOM_OK Success
 * @retval ATOM_ERR_PARAM Bad parameter
 * @retval ATOM_ERR_QUEUE Problem putting the thread back on the ready queue
 */
uint8_t atomSignalSet (ATOM_TCB *curr_tcb_ptr, uint16_t signals)
{
    uint8_t status;
    CRITICAL_STORE;

    /* Check parameters */
    if (curr_tcb_ptr == NULL)
    {
        /* Bad TCB pointer */
        return ATOM_ERR_PARAM;
    }

    if (signals & (0xFFFF << atomFeature_Signals))
    {
        return ATOM_ERR_PARAM;
    }

    /* Protect access to the tcb object and OS queues */
    CRITICAL_START ();

    if (curr_tcb_ptr->suspended)
    {
        if (curr_tcb_ptr->waits & signals)
        {
            curr_tcb_ptr->events = curr_tcb_ptr->events | signals;
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
        /* TCB is not waiting for events*/
        curr_tcb_ptr->events |= signals;
        /* Exit critical region */
        CRITICAL_END ();
        status = ATOM_OK;
    }
    return status;
}

/**
 * \b atomSignalTimerCallback
 *
 * Clear the specified event flags of an active thread.
 *
 * @param[in] curr_tcb_ptr Pointer to TCB whose event flags is to be cleared.
 * @param[in] signals event flags to be cleared
 *
 * @retval ATOM_OK Success
 * @retval ATOM_ERR_PARAM Bad parameter
 */
uint8_t atomSignalClear(ATOM_TCB *curr_tcb_ptr, uint16_t signals)
{
    if (curr_tcb_ptr == NULL)
    {
        /* Bad TCB pointer */
        return ATOM_ERR_PARAM;
    }

    if (signals & (0xFFFF << atomFeature_Signals))
    {
        return ATOM_ERR_PARAM;
    }

    curr_tcb_ptr->events &= (uint8_t)~signals;

    return ATOM_OK;
}

/**
 * \b atomSignalTimerCallback
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
static void atomSignalTimerCallback (POINTER cb_data)
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

