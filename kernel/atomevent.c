/*
 * Copyright (c) 2017, jinsong yu, ramaxel. All rights reserved.
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
 * event library.
 *
 *
 * This module implements a event library with the following
 * features:
 *
 * \par Flexible blocking APIs with event
 * Threads which wish to get a event can choose whether to block,
 * block with timeout, or not block if the event timeout value is -1.
 *
 * \par Interrupt-safe calls
 * All APIs can be called from interrupt context. Any calls which could
 * potentially block have optional parameters to prevent blocking if you
 * wish to call them from interrupt context. Any attempt to make a call
 * which would block from interrupt context will be automatically and
 * safely prevented.
 *
 * \par Priority-based queueing
 * Where multiple threads are blocking on a event, they are woken in
 * order of the threads' priorities. Where multiple threads of the same
 * priority are blocking, they are woken in FIFO order.
 *
 * \par event gruop
 * event groups range up to a maximum of 32 bit flags.
 *
 * \par Smart event deletion
 * Where a event is deleted while threads are blocking on it, all blocking
 * threads are woken and returned a status code to indicate the reason for
 * being woken.
 *
 *
 * \n <b> Usage instructions: </b> \n
 *
 * All event objects must be initialised before use by calling
 * atomEventCreate(). Once initialised atomEventGet() and atomEventGet() are used to
 * put/get the event respectively.
 *
 * If event group is create with all flags cleared, further calls to atomEventGet() will block
 * the calling thread (unless the calling parameters of timeout request no blocking). If
 * a call is made to atomEventSet() to satify the requested flags,for the threads are blocking on a zero-count
 * event flags, the highest priority thread is woken. Where multiple threads of
 * the same priority are blocking, they are woken in the order in which the
 * threads started blocking. 

 *
 * Note that those considering using a event group flags are initialised to 0 in
 * Atomthreads.
 *
 */


#include "atom.h"
#include "atomevent.h"
#include "atomtimer.h"

#include <stdio.h>


/* Local data types */

typedef struct event_timer
{
    ATOM_TCB *tcb_ptr;  /* Thread which is suspended with timeout */
    ATOM_EVENT *event_ptr;  /* event the thread is suspended on */
} EVENT_TIMER;


static void atomEventTimerCallback (POINTER cb_data);

/**
 * \b atomEventCreate
 *
 * This function creates a group of 32 event flags.  All the flags are
 * initially in a cleared state.
 * Must be called before calling any other event library routines on a
 * event. Objects can be deleted later using atom_event_Delete().
 *
 * Does not allocate storage, the caller provides the
 *
 * This function can be called from interrupt context.
 * event_ptr is clear during creation
 *
 * @param[in] event_ptr Pointer to event flags group control block
 * @param[in] Pointer to event flags name
 *
 * @retval ATOM_OK Success
 * @retval ATOM_ERR_PARAM Bad parameters
 */
uint8_t  atomEventCreate(ATOM_EVENT *event_ptr, char *name_ptr)
{

    uint8_t status;

    /* Initialize event flags control block to all zeros.  */
    //memset(event_ptr, 0, sizeof(ATOM_EVENT));

    /* Parameter check */
    if (event_ptr == NULL)
    {
        /* Bad event pointer */
        status = ATOM_ERR_PARAM;
    }
    else
    {

        /* Initialize the suspended threads queue */
    	event_ptr->suspQ = NULL;

    	event_ptr->atom_event_current=0;

        /* Successful */
        status = ATOM_OK;
    }

    return (status);
}




/**
 * \b atomEventGet
 *
 * Perform a get operation on a event group flags.
 *
 * This check the 32bit event groups flags and returns.

  * Depending on the \c timeout value specified the call will do one of
 * the following if the count value is zero:
 *	@param[in] timeout
 * \c timeout == 0 : Call will block until event flag is satisfied or event group is deleted \n
 * \c timeout > 0 : Call will block until non-zero up to the specified timeout \n
 * \c timeout == -1 : Return immediately even the event flags requested which is not satified \n
 **
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
 * @param[in] event Pointer to event object
 * @param[in] timeout Max system ticks to block (0 = forever, -1 return immediately)
 *
 * @param[in] event_ptr   				Pointer to group control block
 * @param[in] requested_flags		Event flags requested
 * @param[in] get_option 				Specifies the all flags/any flags is satified by AND/OR
 * @param[in] actual_flags_ptr          Pointer to place the actual flags
 * @param[in] timeout               Suspension option
 *


 * @retval ATOM_OK Success
 * @retval ATOM_TIMEOUT event timed out before being woken
 * @retval ATOM_WOULDBLOCK Called with timeout == -1 but count is zero
 * @retval ATOM_ERR_DELETED event was deleted while suspended
 * @retval ATOM_ERR_CONTEXT Not called in thread context and attempted to block
 * @retval ATOM_ERR_PARAM Bad parameter
 * @retval ATOM_ERR_QUEUE Problem putting the thread on the suspend queue
 * @retval ATOM_ERR_TIMER Problem registering the timeout
 */

uint8_t  atomEventGet(ATOM_EVENT *event_ptr, uint32_t requested_flags,
                    uint8_t get_option, uint32_t *actual_flags_ptr, uint32_t timeout)
{
    uint8_t status;
    EVENT_TIMER timer_data;
    ATOM_TIMER timer_cb;
    ATOM_TCB *curr_tcb_ptr;

    uint32_t current_flags;
#ifdef DEBUG_EVENT
    printf("%s enter \n",__func__);
#endif
    CRITICAL_STORE;
    /* Check parameters
     * Check for an invalid destination for actual flags.
     * Check for invalid get option.
     * */
    if (event_ptr == NULL || actual_flags_ptr == NULL || get_option > ATOM_AND_CLEAR)
    {
        /* Bad event pointer */
    	return(ATOM_ERR_PARAM);
    }

    else
    {
        /* Protect access to the event object and OS queues */
        CRITICAL_START ();

        current_flags =  event_ptr -> atom_event_current;
#ifdef DEBUG_EVENT
        printf("atomEventGet current_flags %d \n",(int)event_ptr -> atom_event_current);
#endif
        /* Determine if the event flags are present, based on the get option.  */
        if (get_option & ATOM_EVENT_AND_MASK)
        {

            /* All flags must be present to satisfy request.  */
            if ((current_flags & requested_flags) == requested_flags)
            {
                /* Yes, all the events are present.  */
                status =  ATOM_OK;
            }
            else
            {

                /* No, not all the events are present.  */
                status =  ATOM_NO_EVENTS;
            }
        }
        else
        {

            /* Any of the events will satisfy the request.  so return immediately or will suspending the thread..*/
            if (current_flags & requested_flags)
            {

                /* Yes, one or more of the requested events are set.  */
                status =  ATOM_OK;
            }
            else
            {

                /* No, none of the events are currently set.  */
                status =  ATOM_NO_EVENTS;
            }
        }

        /* Now determine if the request can be satisfied immediately.  */
        if (status == ATOM_OK)
        {

            /* Yes, this request can be handled immediately.  */

            /* Return the actual event flags that satisfied the request.  */
            *actual_flags_ptr =  current_flags;

            /* Determine whether or not clearing needs to take place.  */
            if (get_option & ATOM_EVENT_CLEAR_MASK)
            {

                /* Yes, clear the flags that satisfied this request.  */
            	event_ptr -> atom_event_current =
            			event_ptr -> atom_event_current & ~requested_flags;
            }
            CRITICAL_END ();/*ensure each exit have chance to end the critical in each case*/
        }
        else
        {
#ifdef DEBUG_EVENT
        	printf("%s not meet the reqeust flag go here \n",__func__);
#endif
            /* Determine if the request specifies suspension.  it is timeout value ATOM_WAIT_FOREVER*/
            if (timeout >=0 )
            {
                /* Prepare for suspension of this thread.  */

                    /* Get the current TCB */
                    curr_tcb_ptr = atomCurrentContext();
                    /* Check we are actually in thread context */
                    if (curr_tcb_ptr)
                    {
                        /* Add current thread to the suspend list on this event */
                        if (tcbEnqueuePriority (&event_ptr->suspQ, curr_tcb_ptr) != ATOM_OK)
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
                            status = ATOM_OK;

                            /* Remember which event flags we are looking for.  */
                            curr_tcb_ptr -> suspend_info =  requested_flags;
                            curr_tcb_ptr -> suspend_option =  get_option;
#ifdef DEBUG_EVENT
                            printf("atomEventGet suspend_option %d \n",(int)curr_tcb_ptr -> suspend_option);
                            printf("atomEventGet suspend_info %d \n",(int)curr_tcb_ptr -> suspend_info);
#endif

                            /* Track errors */


                            /* Register a timer callback if requested */
                            if (timeout)
                            {
                                /* Fill out the data needed by the callback to wake us up */
                                timer_data.tcb_ptr = curr_tcb_ptr;
                                timer_data.event_ptr = event_ptr;

                                /* Fill out the timer callback request structure */
                                timer_cb.cb_func = atomEventTimerCallback;
                                timer_cb.cb_data = (POINTER)&timer_data;
                                timer_cb.cb_ticks = timeout;

                                /**
                                 * Store the timer details in the TCB so that we can
                                 * cancel the timer callback if the event is put
                                 * before the timeout occurs.
                                 */
                                curr_tcb_ptr->suspend_timo_cb = &timer_cb;

                                /* Register a callback on timeout */
                                if (atomTimerRegister (&timer_cb) != ATOM_OK)
                                {
                                    /* Timer registration failed */
                                    status = ATOM_ERR_TIMER;

                                    /* Clean up and return to the caller */
                                    (void)tcbDequeueEntry (&event_ptr->suspQ, curr_tcb_ptr);
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
                            if (status == ATOM_OK )

                            {
                                /**
                                 * Current thread now blocking, schedule in a new
                                 * one. We already know we are in thread context
                                 * so can call the scheduler from here.
                                 */
#ifdef DEBUG_EVENT
                            	printf("%s suspend here \n",__func__);
#endif
                                atomSched (FALSE);

#ifdef DEBUG_EVENT
                                printf("%s suspend go \n",__func__);
#endif

                                /**
                                 * Normal atomEventGet() wakeups will set ATOM_OK status,
                                 * while timeouts will set ATOM_TIMEOUT and event
                                 * deletions will set ATOM_ERR_DELETED.
                                 */
                                status = curr_tcb_ptr->suspend_wake_status;

                                /**
                                 * If we have been woken up with ATOM_OK then
                                 * another thread set the event flags and
                                 * handed control to this thread.
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

    }
#ifdef DEBUG_EVENT
    printf("status %d leave \n",(unsigned int)status);
    printf("%s leave \n",__func__);
#endif

    /* Return actual completion status.  */
    return(status);
}


/**
 * \b atomEventTimerCallback
 *
 * This is an internal function not for use by application code.
 *
 * Timeouts on suspended threads are notified by the timer system through
 * this generic callback. The timer system calls us back with a pointer to
 * the relevant \c EVENT_TIMER object which is used to retrieve the
 * event group flags.
 *
 * @param[in] cb_data Pointer to a EVENT_TIMER object
 */
static void atomEventTimerCallback (POINTER cb_data)
{
	EVENT_TIMER *timer_data_ptr;
#ifdef DEBUG_EVENT
	printf("%s enter \n",__func__);
#endif

    CRITICAL_STORE;

    /* Get the EVENT_TIMER structure pointer */
    timer_data_ptr = (EVENT_TIMER *)cb_data;

    /* Check parameter is valid */
    if (timer_data_ptr)
    {
        /* Enter critical region */
        CRITICAL_START ();

        /* Set status to indicate to the waiting thread that it timed out */
        timer_data_ptr->tcb_ptr->suspend_wake_status = ATOM_TIMEOUT;

        /* Flag as no timeout registered */
        timer_data_ptr->tcb_ptr->suspend_timo_cb = NULL;

        /* Remove this thread from the event suspend list */
        (void)tcbDequeueEntry (&timer_data_ptr->event_ptr->suspQ, timer_data_ptr->tcb_ptr);

        /* Put the thread on the ready queue */
        (void)tcbEnqueuePriority (&tcbReadyQ, timer_data_ptr->tcb_ptr);

        /* Exit critical region */
        CRITICAL_END ();

        /**
         * Note that we don't call the scheduler now as it will be called
         * when we exit the ISR by atomIntExit().
         */
    }
#ifdef DEBUG_EVENT
    printf("%s leave \n",__func__);
#endif

}
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function sets the specified flags in the event group based on  */
/*    the set option specified.  All threads suspended on the group whose */
/*    get request can now be satisfied are resumed.                       */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    group_ptr                         Pointer to group control block    */
/*    flags_to_set                      Event flags to set                */
/*    set_option                        Specified either AND or OR        */
/*                                        operation on the event flags    */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    TX_SUCCESS                        Always returns success            */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _tx_thread_system_preempt_check   Check for preemption              */
/*    _tx_thread_system_resume          Resume thread service             */
/*    _tx_thread_system_ni_resume       Non-interruptable resume thread   */
/*                                                                        */
/*  CALLED BY                                                             */

/**
 * \b atomEventSet
 *
 * This function sets the specified flags in the event group based on
 * the set option specified.  All threads suspended on the group whose
 * get request can now be satisfied are resumed.
 *  if there are threads blocking on the
 * event, the call will wake up the highest priority thread suspended. Only
 * one thread is woken per call to atomEventSet(). If multiple threads of the
 * same priority are suspended, they are woken in order of suspension (FIFO).
 *
 * This function can be called from interrupt context.
 *
 * @param[in] event_ptr group Pointer to event object
 * @param[in] flags_to_set to set
 * @param[in] set_option which define the OR/AND operation for the requested event flags in atomEventGet
 *
 * @retval ATOM_OK Success
 * @retval ATOM_ERR_PARAM Bad parameter
 * @retval ATOM_ERR_QUEUE Problem putting a woken thread on the ready queue
 * @retval ATOM_ERR_TIMER Problem cancelling a timeout for a woken thread
 */
uint8_t  atomEventSet(ATOM_EVENT *event_ptr, uint32_t flags_to_set, uint8_t set_option)
{

	uint8_t status;
    CRITICAL_STORE;
    ATOM_TCB *tcb_ptr;

#ifdef DEBUG_EVENT
    printf("%s enter \n",__func__);
#endif
    /* Check parameters
     * Check for an invalid destination for actual flags.
     * Check for invalid get option.
     * */
    if (event_ptr == NULL || ((set_option != ATOM_AND) && (set_option != ATOM_OR)) )
    {
        /* Bad event pointer */
    	return(ATOM_ERR_PARAM);
    }

    else
    {
    	CRITICAL_START ();
        /* Determine how to set this group's event flags.  */
        if (set_option & ATOM_EVENT_AND_MASK)
        {

            /* Previous set operation was not interrupted, simply clear the
               specified flags by "ANDing" the flags into the current events
               of the group.  */
        	event_ptr -> atom_event_current =
        			event_ptr -> atom_event_current & flags_to_set;

            /* There is no need to check for any suspended threads since no
               new bits are set.  */
        	CRITICAL_END ();

            /* Return successful status.  */
            return(ATOM_OK);
        }
        else
        {

            /* "OR" the flags into the current events of the group.  */
        	event_ptr -> atom_event_current =
        			event_ptr -> atom_event_current | flags_to_set;

            /* Determine if there are any delayed flags to clear.  */
        }


#ifdef DEBUG_EVENT
        printf(" event_ptr -> atom_event_current 0x%8x \n",(unsigned int)event_ptr -> atom_event_current);
#endif

        /* If any threads are blocking on the event, wake up one */
        if (event_ptr->suspQ)
        {
#ifdef DEBUG_EVENT
        	printf("tcbDequeueHead to pull out queue.\n");
#endif

        	tcb_ptr = tcbDequeueHead (&event_ptr->suspQ);
        	////////////////////////////////////////////////////////
        	/* copy from thread event get.***********/
            /* Determine if this thread's get event flag request has been met.  */
            if (tcb_ptr -> suspend_option & ATOM_EVENT_AND_MASK)
            {
#ifdef DEBUG_EVENT
            	printf("AND event_ptr -> atom_event_current 0x%8x \n",(unsigned int)event_ptr -> atom_event_current);
            	printf("AND event_ptr -> suspend_info 0x%8x \n",(unsigned int)tcb_ptr -> suspend_info);
#endif


                /* All flags must be present to satisfy request.  */
                if ((event_ptr -> atom_event_current & tcb_ptr -> suspend_info) ==
                		tcb_ptr -> suspend_info)

                    /* Yes, all the events are present.  */
                    status =  ATOM_OK;
                else

                    /* No, not all the events are present.  */
                    status =  ATOM_NO_EVENTS;
            }
            else
            {
#ifdef DEBUG_EVENT
            	printf("OR event_ptr -> atom_event_current 0x%8x \n",(unsigned int)event_ptr -> atom_event_current);
            	printf("OR event_ptr -> suspend_info 0x%8x \n",(unsigned int)tcb_ptr -> suspend_info);
#endif


                /* Any of the events will satisfy the request.  */
                if (event_ptr -> atom_event_current & tcb_ptr -> suspend_info)

                    /* Yes, one or more of the requested events are set.  */
                    status =  ATOM_OK;
                else

                    /* No, none of the events are currently set.  */
                    status =  ATOM_NO_EVENTS;
            }
#ifdef DEBUG_EVENT
            printf("check suspQ status %d \n",(unsigned int)status);
#endif



            /* Was the suspended thread's event request satisfied?  */
            /* enter this case it must return OK*/
            if (status == ATOM_OK)
            {

                /* Yes, resume the thread and apply any event flag
                   clearing.  */

                /* Determine whether or not clearing needs to take place.  */
                if (tcb_ptr ->suspend_option & ATOM_EVENT_CLEAR_MASK)
                {

                    /* Yes, clear the flags that satisfied this request.  */
                	event_ptr -> atom_event_current =
                			event_ptr -> atom_event_current & ~(tcb_ptr ->suspend_option);
                }


                /* Set OK status to be returned to the waiting thread */
                tcb_ptr->suspend_wake_status = ATOM_OK;


#ifdef DEBUG_EVENT
                printf ("enqueue the thread into ready queue \n");
#endif
                if (tcbEnqueuePriority (&tcbReadyQ, tcb_ptr) != ATOM_OK)
                {
                    /* Exit critical region */
                    CRITICAL_END ();

                    /* There was a problem putting the thread on the ready queue */
                    status = ATOM_ERR_QUEUE;
                }
                else
                {
                    if ((tcb_ptr->suspend_timo_cb != NULL)
                          && (atomTimerCancel (tcb_ptr->suspend_timo_cb) != ATOM_OK))
                      {
                          /* There was a problem cancelling a timeout on this event */
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

            else/* if not the ATOM_OK, enqueue the ptr checkout just now*/
            {

            	//case for ATOM_NO_EVENTS

#ifdef DEBUG_EVENT
            	printf("status %d \n",status);
            	printf("tcbEnqueuePriority to suspq\n");
#endif
            	if (tcb_ptr)
            	{
					if (tcbEnqueuePriority (&event_ptr->suspQ, tcb_ptr) != ATOM_OK)
					{

						/* There was a problem putting the thread on the ready queue */
						status = ATOM_ERR_QUEUE;
					}
            	}
            	/* Exit critical region */
                CRITICAL_END ();

            }

            /**
             * Threads are woken up in priority order, with a FIFO system
             * used on same priority threads. We always take the head,
             * ordering is taken care of by an ordered list enqueue.
             */

        }/*end of suspend quque*/
        else
        {
        	/* Exit critical region added 0329*/
            CRITICAL_END ();
        }

    }/* end of else event_ptr == NULL*/


#ifdef DEBUG_EVENT
    printf("status %d leave \n",(unsigned int)status);
    printf("%s leave \n",__func__);
#endif
    return (status);
}


/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function deletes the specified event flag group.  All threads  */
/*    suspended on the group are resumed with the ATOM_DELETED status       */

uint8_t atomEventDelete (ATOM_EVENT *event_ptr)
{
    uint8_t status;
    CRITICAL_STORE;
    ATOM_TCB *tcb_ptr;
    uint8_t woken_threads = FALSE;

    /* Parameter check */
    if (event_ptr == NULL)
    {
        /* Bad event pointer */
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
            tcb_ptr = tcbDequeueHead (&event_ptr->suspQ);

            /* A thread is suspended on the event group */
            if (tcb_ptr)
            {
#ifdef DEBUG_EVENT
            	printf("tcb_ptr->suspend_wake_status %d\n",tcb_ptr->suspend_wake_status);
#endif

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


