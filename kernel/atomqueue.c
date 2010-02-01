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
 * Queue library.
 *
 *
 * This module implements a queue / message-passing library with the following
 * features:
 *
 * \par Flexible blocking APIs
 * Threads which wish to make a call which may block can choose whether to
 * block, block with timeout, or not block and return a relevent status
 * code.
 *
 * \par Interrupt-safe calls
 * All APIs can be called from interrupt context. Any calls which could
 * potentially block have optional parameters to prevent blocking if you
 * wish to call them from interrupt context. Any attempt to make a call
 * which would block from interrupt context will be automatically and
 * safely prevented.
 *
 * \par Priority-based queueing
 * Where multiple threads are blocking on a queue, they are woken in order of
 * the threads' priorities. Where multiple threads of the same priority are
 * blocking, they are woken in FIFO order.
 *
 * \par Configurable queue sizes
 * Queues can be created with any sized message, and any number of stored
 * messages.
 *
 * \par Smart queue deletion
 * Where a queue is deleted while threads are blocking on it, all blocking
 * threads are woken and returned a status code to indicate the reason for
 * being woken.
 *
 *
 * \n <b> Usage instructions: </b> \n
 *
 * All queue objects must be initialised before use by calling
 * atomQueueCreate(). Once initialised atomQueueGet() and atomQueuePut() are
 * used to send and receive messages via the queue respectively.
 *
 * Messages can be added to a queue by calling atomQueuePut(). If the queue is
 * full the caller will block until space becomes available (by a message
 * being removed from the queue). Optionally a non-blocking call can be made
 * in which case the call will return with a status code indicating that the
 * queue is full. This allows messages to be posted from interrupt handlers
 * or threads which you do not wish to block, providing it is not fatal that
 * the call could fail if the queue was full.
 *
 * Messages can be received from the queue by calling atomQueueGet(). This
 * will return the first message available in the queue in FIFO order. If
 * the queue is empty then the call will block. Optionally, a non-blocking
 * call can be made in which case the call will return with a status code
 * indicating that the queue is full. This allows messages to be received
 * by interrupt handlers or threads which you do not wish to block.
 * 
 * A queue which is no longer required can be deleted using atomQueueDelete().
 * This function automatically wakes up any threads which are waiting on the
 * deleted queue.
 *
 */
 

#include <stdio.h>
#include <string.h>

#include "atom.h"
#include "atomqueue.h"
#include "atomtimer.h"


/* Local data types */

typedef struct queue_timer
{
    ATOM_TCB   *tcb_ptr;    /* Thread which is suspended with timeout */
    ATOM_QUEUE *queue_ptr;  /* Queue the thread is interested in */
    ATOM_TCB   **suspQ;     /* TCB queue which thread is suspended on */
} QUEUE_TIMER;


/* Forward declarations */

static uint8_t queue_remove (ATOM_QUEUE *qptr, uint8_t* msgptr);
static uint8_t queue_insert (ATOM_QUEUE *qptr, uint8_t* msgptr);
static void atomQueueTimerCallback (POINTER cb_data);


/**
 * \b atomQueueCreate
 *
 * Initialises a queue object.
 *
 * Must be called before calling any other queue library routines on a
 * queue. Objects can be deleted later using atomQueueDelete().
 *
 * Does not allocate storage, the caller provides the queue object.
 *
 * Callers pass in their own buffer area for storing the queue messages while
 * in transit between threads. The provided storage must be large enough to
 * store (\c unit_size * \c max_num_mgs) bytes. i.e. the storage area will be
 * used for up to \c max_num_msgs messages each of size \c unit_size.
 *
 * Queues use a fixed-size message.
 *
 * This function can be called from interrupt context.
 *
 * @param[in] qptr Pointer to queue object
 * @param[in] buff_ptr Pointer to buffer storage area
 * @param[in] unit_size Size in bytes of each queue message
 * @param[in] max_num_msgs Maximum number of messages in the queue
 *
 * @retval ATOM_OK Success
 * @retval ATOM_ERR_PARAM Bad parameters
 */
uint8_t atomQueueCreate (ATOM_QUEUE *qptr, uint8_t *buff_ptr, uint32_t unit_size, uint32_t max_num_msgs)
{
    uint8_t status;

    /* Parameter check */
    if ((qptr == NULL) || (buff_ptr == NULL))
    {
        /* Bad pointers */
        status = ATOM_ERR_PARAM;
    }
    else if ((unit_size == 0) || (max_num_msgs == 0))
    {
        /* Bad values */
        status = ATOM_ERR_PARAM;
    }
    else
    {
       /* Store the queue details */
        qptr->buff_ptr = buff_ptr;
        qptr->unit_size = unit_size;
        qptr->max_num_msgs = max_num_msgs;

        /* Initialise the suspended threads queues */
        qptr->putSuspQ = NULL;
        qptr->getSuspQ = NULL;

        /* Initialise the insert/remove pointers */
        qptr->insert_index = 0;
        qptr->remove_index = 0;
        qptr->num_msgs_stored = 0;

        /* Successful */
        status = ATOM_OK;
    }

    return (status);
}


/**
 * \b atomQueueDelete
 *
 * Deletes a queue object.
 *
 * Any threads currently suspended on the queue will be woken up with
 * return status ATOM_ERR_DELETED. If called at thread context then the
 * scheduler will be called during this function which may schedule in one
 * of the woken threads depending on relative priorities.
 *
 * This function can be called from interrupt context, but loops internally
 * waking up all threads blocking on the queue, so the potential
 * execution cycles cannot be determined in advance.
 *
 * @param[in] qptr Pointer to queue object
 *
 * @retval ATOM_OK Success
 * @retval ATOM_ERR_QUEUE Problem putting a woken thread on the ready queue
 * @retval ATOM_ERR_TIMER Problem cancelling a timeout on a woken thread
 */
uint8_t atomQueueDelete (ATOM_QUEUE *qptr)
{
    uint8_t status;
    CRITICAL_STORE;
    ATOM_TCB *tcb_ptr;
    uint8_t woken_threads = FALSE;

    /* Parameter check */
    if (qptr == NULL)
    {
        /* Bad pointer */
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
            if (((tcb_ptr = tcbDequeueHead (&qptr->getSuspQ)) != NULL)
                || ((tcb_ptr = tcbDequeueHead (&qptr->putSuspQ)) != NULL))
            {
                /* A thread is waiting on a suspend queue */

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
 * \b atomQueueGet
 *
 * Attempt to retrieve a message from a queue.
 *
 * Retrieves one message at a time. Messages are copied into the passed
 * \c msgptr storage area which should be large enough to contain one
 * message of \c unit_size bytes. Where multiple messages are in the queue,
 * messages are retrieved in FIFO order.
 *
 * If the queue is currently empty, the call will do one of the following
 * depending on the \c timeout value specified:
 *
 * \c timeout == 0 : Call will block until a message is available \n
 * \c timeout > 0 : Call will block until a message or the specified timeout \n
 * \c timeout == -1 : Return immediately if no message is on the queue \n
 *
 * If a maximum timeout value is specified (\c timeout > 0), and no message
 * is present on the queue for the specified number of system ticks, the
 * call will return with \c ATOM_TIMEOUT.
 *
 * This function can only be called from interrupt context if the \c timeout
 * parameter is -1 (in which case it does not block).
 *
 * @param[in] qptr Pointer to queue object
 * @param[in] timeout Max system ticks to block (0 = forever, -1 =  no block)
 * @param[out] msgptr Pointer to which the received message will be copied
 *
 * @retval ATOM_OK Success
 * @retval ATOM_TIMEOUT Queue wait timed out before being woken
 * @retval ATOM_WOULDBLOCK Called with timeout == -1 but queue was empty
 * @retval ATOM_ERR_DELETED Queue was deleted while suspended
 * @retval ATOM_ERR_CONTEXT Not called in thread context and attempted to block
 * @retval ATOM_ERR_PARAM Bad parameter
 * @retval ATOM_ERR_QUEUE Problem putting the thread on the suspend queue
 * @retval ATOM_ERR_TIMER Problem registering the timeout
 */
uint8_t atomQueueGet (ATOM_QUEUE *qptr, int32_t timeout, uint8_t *msgptr)
{
    CRITICAL_STORE;
    uint8_t status;
    QUEUE_TIMER timer_data;
    ATOM_TIMER timer_cb;
    ATOM_TCB *curr_tcb_ptr;

    /* Check parameters */
    if ((qptr == NULL) || (msgptr == NULL))
    {
        /* Bad pointer */
        status = ATOM_ERR_PARAM;
    }
    else
    {
        /* Protect access to the queue object and OS queues */
        CRITICAL_START ();

        /* If no messages on the queue, block the calling thread */
        if (qptr->num_msgs_stored == 0)
        {
            /* If called with timeout >= 0, we should block */
            if (timeout >= 0)
            {
                /* Queue is empty, block the calling thread */

                /* Get the current TCB */
                curr_tcb_ptr = atomCurrentContext();

                /* Check we are actually in thread context */
                if (curr_tcb_ptr)
                {
                    /* Add current thread to the list suspended on receives */
                    if (tcbEnqueuePriority (&qptr->getSuspQ, curr_tcb_ptr) == ATOM_OK)
                    {
                        /* Set suspended status for the current thread */
                        curr_tcb_ptr->suspended = TRUE;

                        /* Track errors */
                        status = ATOM_OK;

                        /* Register a timer callback if requested */
                        if (timeout)
                        {
                            /**
                             * Fill out the data needed by the callback to
                             * wake us up.
                             */
                            timer_data.tcb_ptr = curr_tcb_ptr;
                            timer_data.queue_ptr = qptr;
                            timer_data.suspQ = &qptr->getSuspQ;

                            /* Fill out the timer callback request structure */
                            timer_cb.cb_func = atomQueueTimerCallback;
                            timer_cb.cb_data = (POINTER)&timer_data;
                            timer_cb.cb_ticks = timeout;

                            /**
                             * Store the timer details in the TCB so that we
                             * can cancel the timer callback if the queue is
                             * put before the timeout occurs.
                             */
                            curr_tcb_ptr->suspend_timo_cb = &timer_cb;

                            /* Register a callback on timeout */
                            if (atomTimerRegister (&timer_cb) != ATOM_OK)
                            {
                                /* Timer registration failed */
                                status = ATOM_ERR_TIMER;

                                /* Clean up and return to the caller */
                                (void)tcbDequeueEntry (&qptr->getSuspQ, curr_tcb_ptr);
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

                        /* Check no errors occurred */
                        if (status == ATOM_OK)
                        {
                            /**
                             * Current thread now blocking, schedule in a new
                             * one. We already know we are in thread context
                             * so can call the scheduler from here.
                             */
                            atomSched (FALSE);

                            /**
                             * Normal atomQueuePut() wakeups will set ATOM_OK
                             * status, while timeouts will set ATOM_TIMEOUT
                             * and queue deletions will set ATOM_ERR_DELETED.
                             */
                            status = curr_tcb_ptr->suspend_wake_status;

                            /**
                             * Check suspend_wake_status. If it is ATOM_OK
                             * then we were woken because a message has been
                             * put on the queue and we can now copy it out.
                             * Otherwise we were woken because we timed out
                             * waiting for a message, or the queue was
                             * deleted, so we should just quit.
                             */
                            if (status == ATOM_OK)
                            {
                                /* Enter critical region */
                                CRITICAL_START();

                                /* Copy the message out of the queue */
                                status = queue_remove (qptr, msgptr);

                                /* Exit critical region */
                                CRITICAL_END();
                            }
                        }
                    }
                    else
                    {
                        /* There was an error putting this thread on the suspend list */
                        CRITICAL_END ();
                        status = ATOM_ERR_QUEUE;
                    }
                }
                else
                {
                    /* Not currently in thread context, can't suspend */
                    CRITICAL_END ();
                    status = ATOM_ERR_CONTEXT;
                }
            }
            else
            {
                /* timeout == -1, requested not to block and queue is empty */
                CRITICAL_END();
                status = ATOM_WOULDBLOCK;
            }
        }
        else
        {
            /* No need to block, there is a message to copy out of the queue */
            status = queue_remove (qptr, msgptr);

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

    return (status);
}


/**
 * \b atomQueuePut
 *
 * Attempt to put a message onto a queue.
 *
 * Sends one message at a time. Messages are copied from the passed
 * \c msgptr storage area which should contain a message of \c unit_size
 * bytes.
 *
 * If the queue is currently full, the call will do one of the following
 * depending on the \c timeout value specified:
 *
 * \c timeout == 0 : Call will block until space is available \n
 * \c timeout > 0 : Call will block until space or the specified timeout \n
 * \c timeout == -1 : Return immediately if the queue is full \n
 *
 * If a maximum timeout value is specified (\c timeout > 0), and no space
 * is available on the queue for the specified number of system ticks, the
 * call will return with \c ATOM_TIMEOUT.
 *
 * This function can only be called from interrupt context if the \c timeout
 * parameter is -1 (in which case it does not block and may fail to post a
 * message if the queue is full).
 *
 * @param[in] qptr Pointer to queue object
 * @param[in] timeout Max system ticks to block (0 = forever, -1 =  no block)
 * @param[out] msgptr Pointer from which the message should be copied out
 *
 * @retval ATOM_OK Success
 * @retval ATOM_WOULDBLOCK Called with timeout == -1 but queue was full
 * @retval ATOM_TIMEOUT Queue wait timed out before being woken
 * @retval ATOM_ERR_DELETED Queue was deleted while suspended
 * @retval ATOM_ERR_CONTEXT Not called in thread context and attempted to block
 * @retval ATOM_ERR_PARAM Bad parameter
 * @retval ATOM_ERR_QUEUE Problem putting the thread on the suspend queue
 * @retval ATOM_ERR_TIMER Problem registering the timeout
 */
uint8_t atomQueuePut (ATOM_QUEUE *qptr, int32_t timeout, uint8_t *msgptr)
{
    CRITICAL_STORE;
    uint8_t status;
    QUEUE_TIMER timer_data;
    ATOM_TIMER timer_cb;
    ATOM_TCB *curr_tcb_ptr;

    /* Check parameters */
    if ((qptr == NULL) || (msgptr == NULL))
    {
        /* Bad pointer */
        status = ATOM_ERR_PARAM;
    }
    else
    {
        /* Protect access to the queue object and OS queues */
        CRITICAL_START ();

        /* If queue is full, block the calling thread */
        if (qptr->num_msgs_stored == qptr->max_num_msgs)
        {
            /* If called with timeout >= 0, we should block */
            if (timeout >= 0)
            {
                /* Queue is full, block the calling thread */

                /* Get the current TCB */
                curr_tcb_ptr = atomCurrentContext();

                /* Check we are actually in thread context */
                if (curr_tcb_ptr)
                {
                    /* Add current thread to the suspend list on sends */
                    if (tcbEnqueuePriority (&qptr->putSuspQ, curr_tcb_ptr) == ATOM_OK)
                    {
                        /* Set suspended status for the current thread */
                        curr_tcb_ptr->suspended = TRUE;

                        /* Track errors */
                        status = ATOM_OK;

                        /* Register a timer callback if requested */
                        if (timeout)
                        {
                            /**
                             * Fill out the data needed by the callback to
                             * wake us up.
                             */
                            timer_data.tcb_ptr = curr_tcb_ptr;
                            timer_data.queue_ptr = qptr;
                            timer_data.suspQ = &qptr->putSuspQ;


                            /* Fill out the timer callback request structure */
                            timer_cb.cb_func = atomQueueTimerCallback;
                            timer_cb.cb_data = (POINTER)&timer_data;
                            timer_cb.cb_ticks = timeout;

                            /**
                             * Store the timer details in the TCB so that we
                             * can cancel the timer callback if a message is
                             * removed from the queue before the timeout
                             * occurs.
                             */
                            curr_tcb_ptr->suspend_timo_cb = &timer_cb;

                            /* Register a callback on timeout */
                            if (atomTimerRegister (&timer_cb) != ATOM_OK)
                            {
                                /* Timer registration failed */
                                status = ATOM_ERR_TIMER;

                                /* Clean up and return to the caller */
                                (void)tcbDequeueEntry (&qptr->putSuspQ, curr_tcb_ptr);
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

                        /* Check timer registration was successful */
                        if (status == ATOM_OK)
                        {
                            /**
                             * Current thread now blocking, schedule in a new
                             * one. We already know we are in thread context
                             * so can call the scheduler from here.
                             */
                            atomSched (FALSE);

                            /**
                             * Normal atomQueueGet() wakeups will set ATOM_OK
                             * status, while timeouts will set ATOM_TIMEOUT
                             * and queue deletions will set ATOM_ERR_DELETED.
                             */
                            status = curr_tcb_ptr->suspend_wake_status;

                            /**
                             * Check suspend_wake_status. If it is ATOM_OK
                             * then we were woken because a message has been
                             * removed from the queue and we can now add ours.
                             * Otherwise we were woken because we timed out
                             * waiting for a message, or the queue was
                             * deleted, so we should just quit.
                             */
                            if (status == ATOM_OK)
                            {
                                /* Enter critical region */
                                CRITICAL_START();

                                /* Copy the message into the queue */
                                status = queue_insert (qptr, msgptr);

                                /* Exit critical region */
                                CRITICAL_END();
                            }
                        }
                    }
                    else
                    {
                        /* There was an error putting this thread on the suspend list */
                        CRITICAL_END ();
                        status = ATOM_ERR_QUEUE;
                    }
                }
                else
                {
                    /* Not currently in thread context, can't suspend */
                    CRITICAL_END ();
                    status = ATOM_ERR_CONTEXT;
                }
            }
            else
            {
                /* timeout == -1, cannot block. Just return queue is full */
                CRITICAL_END();
                status = ATOM_WOULDBLOCK;
            }
        }
        else
        {
            /* No need to block, there is space to copy into the queue */
            status = queue_insert (qptr, msgptr);

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

    return (status);
}


/**
 * \b atomQueueTimerCallback
 *
 * This is an internal function not for use by application code.
 *
 * Timeouts on suspended threads are notified by the timer system through
 * this generic callback. The timer system calls us back with a pointer to
 * the relevant \c QUEUE_TIMER object which is used to retrieve the
 * queue details.
 *
 * @param[in] cb_data Pointer to a QUEUE_TIMER object
 */
static void atomQueueTimerCallback (POINTER cb_data)
{
    QUEUE_TIMER *timer_data_ptr;
    CRITICAL_STORE;

    /* Get the QUEUE_TIMER structure pointer */
    timer_data_ptr = (QUEUE_TIMER *)cb_data;

    /* Check parameter is valid */
    if (timer_data_ptr)
    {
        /* Enter critical region */
        CRITICAL_START ();

        /* Set status to indicate to the waiting thread that it timed out */
        timer_data_ptr->tcb_ptr->suspend_wake_status = ATOM_TIMEOUT;

        /* Flag as no timeout registered */
        timer_data_ptr->tcb_ptr->suspend_timo_cb = NULL;

        /**
         * Remove this thread from the queue's suspend list. Handles threads
         * suspended on the receive list as well as the send list.
         */
        (void)tcbDequeueEntry (timer_data_ptr->suspQ, timer_data_ptr->tcb_ptr);

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


/**
 * \b queue_remove
 *
 * This is an internal function not for use by application code.
 *
 * Removes a message from a queue. Assumes that there is a message present,
 * which is already checked by the calling functions with interrupts locked
 * out.
 *
 * Also wakes up a suspended thread if there are any waiting to send on the
 * queue.
 *
 * Assumes interrupts are already locked out.
 *
 * @param[in] qptr Pointer to an ATOM_QUEUE object
 * @param[in] msgptr Destination pointer for the message to be copied into
 *
 * @retval ATOM_OK Success
 * @retval ATOM_ERR_PARAM Bad parameter
 * @retval ATOM_ERR_QUEUE Problem putting a thread on the ready queue
 * @retval ATOM_ERR_TIMER Problem cancelling a timeout
 */
static uint8_t queue_remove (ATOM_QUEUE *qptr, uint8_t* msgptr)
{
    uint8_t status;
    ATOM_TCB *tcb_ptr;

    /* Check parameters */
    if ((qptr == NULL) || (msgptr == NULL))
    {
        /* Bad pointer */
        status = ATOM_ERR_PARAM;
    }
    else
    {
        /* There is a message on the queue, copy it out */
        memcpy (msgptr, (qptr->buff_ptr + qptr->remove_index), qptr->unit_size);
        qptr->remove_index += qptr->unit_size;
        qptr->num_msgs_stored--;

        /* Check if the remove index should now wrap to the beginning */
        if (qptr->remove_index >= (qptr->unit_size * qptr->max_num_msgs))
            qptr->remove_index = 0;

        /**
         * If there are threads waiting to send, wake one up now. Waiting
         * threads are woken up in priority order, with same-priority
         * threads woken up in FIFO order.
         */
        tcb_ptr = tcbDequeueHead (&qptr->putSuspQ);
        if (tcb_ptr)
        {
            /* Move the waiting thread to the ready queue */
            if (tcbEnqueuePriority (&tcbReadyQ, tcb_ptr) == ATOM_OK)
            {
                /* Set OK status to be returned to the waiting thread */
                tcb_ptr->suspend_wake_status = ATOM_OK;

                /* If there's a timeout on this suspension, cancel it */
                if ((tcb_ptr->suspend_timo_cb != NULL)
                    && (atomTimerCancel (tcb_ptr->suspend_timo_cb) != ATOM_OK))
                {
                    /* There was a problem cancelling a timeout */
                    status = ATOM_ERR_TIMER;
                }
                else
                {
                    /* Flag as no timeout registered */
                    tcb_ptr->suspend_timo_cb = NULL;

                    /* Successful */
                    status = ATOM_OK;
                }
            }
            else
            {
                /**
                 * There was a problem putting the thread on the ready
                 * queue.
                 */
                status = ATOM_ERR_QUEUE;
            }
        }
        else
        {
            /* There were no threads waiting to send */
            status = ATOM_OK;
        }
    }

    return (status);
}


/**
 * \b queue_insert
 *
 * This is an internal function not for use by application code.
 *
 * Inserts a message onto a queue. Assumes that the queue has space for one
 * message, which has already been checked by the calling function with
 * interrupts locked out.
 *
 * Also wakes up a suspended thread if there are any waiting to receive on the
 * queue.
 *
 * Assumes interrupts are already locked out.
 *
 * @param[in] qptr Pointer to an ATOM_QUEUE object
 * @param[in] msgptr Source pointer for the message to be copied out of
 *
 * @retval ATOM_OK Success
 * @retval ATOM_ERR_PARAM Bad parameter
 * @retval ATOM_ERR_QUEUE Problem putting a thread on the ready queue
 * @retval ATOM_ERR_TIMER Problem cancelling a timeout
 */
static uint8_t queue_insert (ATOM_QUEUE *qptr, uint8_t* msgptr)
{
    uint8_t status;
    ATOM_TCB *tcb_ptr;

    /* Check parameters */
    if ((qptr == NULL) || (msgptr == NULL))
    {
        /* Bad pointer */
        status = ATOM_ERR_PARAM;
    }
    else
    {
        /* There is space in the queue, copy it in */
        memcpy ((qptr->buff_ptr + qptr->insert_index), msgptr, qptr->unit_size);
        qptr->insert_index += qptr->unit_size;
        qptr->num_msgs_stored++;

        /* Check if the insert index should now wrap to the beginning */
        if (qptr->insert_index >= (qptr->unit_size * qptr->max_num_msgs))
            qptr->insert_index = 0;

        /**
         * If there are threads waiting to receive, wake one up now. Waiting
         * threads are woken up in priority order, with same-priority
         * threads woken up in FIFO order.
         */
        tcb_ptr = tcbDequeueHead (&qptr->getSuspQ);
        if (tcb_ptr)
        {
            /* Move the waiting thread to the ready queue */
            if (tcbEnqueuePriority (&tcbReadyQ, tcb_ptr) == ATOM_OK)
            {
                /* Set OK status to be returned to the waiting thread */
                tcb_ptr->suspend_wake_status = ATOM_OK;

                /* If there's a timeout on this suspension, cancel it */
                if ((tcb_ptr->suspend_timo_cb != NULL)
                    && (atomTimerCancel (tcb_ptr->suspend_timo_cb) != ATOM_OK))
                {
                    /* There was a problem cancelling a timeout */
                    status = ATOM_ERR_TIMER;
                }
                else
                {
                    /* Flag as no timeout registered */
                    tcb_ptr->suspend_timo_cb = NULL;

                    /* Successful */
                    status = ATOM_OK;
                }
            }
            else
            {
                /**
                 * There was a problem putting the thread on the ready
                 * queue.
                 */
                status = ATOM_ERR_QUEUE;
            }
        }
        else
        {
            /* There were no threads waiting to send */
            status = ATOM_OK;
        }
    }

    return (status);
}
