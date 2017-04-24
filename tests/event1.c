/*
 * Copyright (c) 2017, jinsong yu ramaxel. All rights reserved.
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


#include "atom.h"
#include "atomevent.h"
#include "atomtests.h"

#define RX_COMPLETE_FLAG 0x01
#define TX_COMPLETE_FLAG 0x02


/* Number of test threads */
#define NUM_TEST_THREADS      2


/* Test OS objects */
static ATOM_EVENT event1, event2;
static ATOM_TCB tcb[NUM_TEST_THREADS];
static uint8_t test_thread_stack[NUM_TEST_THREADS][TEST_THREAD_STACK_SIZE];


/* Forward declarations */
static void test1_thread_func (uint32_t param);
static void test2_thread_func (uint32_t param);

static int event_set_tx_flag();
//static void settimer2(uint32_t timeout);
static int event_set_rx_flag();
//static void settimer1(uint32_t timeout);


/**
 * \b test_start
 *
 * Start event get/set test,
 *
 * case1: create the thread1,event1,get with option ATOM_AND_CLEAR which means tx/rx here should be satisfied simulatiouly.
 * here set the tx/rx one by one to see what happens, it expect the thread2 get the flag after the event_set_tx_flag.
 *
 * case2: create the thread2,event2,get with option ATOM_AND_CLEAR which means tx/rx here should be satisfied simulatiouly.
 * here set the tx/rx one by one to see what happens, it expect the thread2 get the flag after the event_set_tx_flag.
 *
 * @retval Number of failures
 */
uint32_t test_start (void)
{
	int failures;

	failures = 0;
	//case1:
    ATOMLOG (_STR("test event1 case1: get option ATOM_OR_CLEAR ,set option ATOM_OR  \n"));

    if (atomEventCreate (&event1, 0) != ATOM_OK)
    {
        ATOMLOG (_STR("Error creating test event 1\n"));
        failures++;
    }
    else if (atomThreadCreate(&tcb[0], TEST_THREAD_PRIO, test1_thread_func, 0,
              &test_thread_stack[0][0],
              TEST_THREAD_STACK_SIZE, TRUE) != ATOM_OK)
    {
        /* Fail */
        ATOMLOG (_STR("Error creating test thread 1\n"));
        failures++;
    }
    else
    {
        /* let thread1 to exec */
        if (atomTimerDelay(SYSTEM_TICKS_PER_SEC*5) != ATOM_OK)
        {
            ATOMLOG (_STR("Failed timer delay\n"));
        }

        if( event_set_rx_flag() != ATOM_OK )
        {
            /* Fail */
            ATOMLOG (_STR("Error get the thread 1  flag \n"));
            failures++;
        }

    	atomTimerDelay (150*SYSTEM_TICKS_PER_SEC);
    	printf("thread1 test passed \n");
    }


    //case2:
    ATOMLOG (_STR("test event2 case2: get option ATOM_AND_CLEAR,set option ATOM_OR  \n"));

    if (atomEventCreate (&event2, 0) != ATOM_OK)
    {
        ATOMLOG (_STR("Error creating test event 2\n"));
        failures++;
    }
    else if (atomThreadCreate(&tcb[1], TEST_THREAD_PRIO, test2_thread_func, 0,
              &test_thread_stack[1][0],
              TEST_THREAD_STACK_SIZE, TRUE) != ATOM_OK)
    {
        /* Fail */
        ATOMLOG (_STR("Error creating test thread 2\n"));
        failures++;
    }
    else
    {
        /* let thread2 to exec */
        if (atomTimerDelay(SYSTEM_TICKS_PER_SEC*5) != ATOM_OK)
        {
            ATOMLOG (_STR("Failed timer delay\n"));
        }

        if( event_set_rx_flag() == ATOM_OK )
        {
            /* Fail */
            ATOMLOG (_STR("Error get the thread 2 flag \n"));
            failures++;
        }

        if (atomTimerDelay(SYSTEM_TICKS_PER_SEC*10) != ATOM_OK)
        {
            ATOMLOG (_STR("Failed timer delay\n"));
        }

        if( event_set_tx_flag() != ATOM_OK )
        {
            /* Fail */
            ATOMLOG (_STR("Error get the thread 2  flag \n"));
            failures++;
        }

    	atomTimerDelay (150*SYSTEM_TICKS_PER_SEC);
    	printf("thread2 test passed \n");
    }

    /* Check thread stack usage (if enabled) */
#ifdef ATOM_STACK_CHECKING
    {
        uint32_t used_bytes, free_bytes;
        int thread;

        /* Check all threads */
        for (thread = 0; thread < NUM_TEST_THREADS; thread++)
        {
            /* Check thread stack usage */
            if (atomThreadStackCheck (&tcb[thread], &used_bytes, &free_bytes) != ATOM_OK)
            {
                ATOMLOG (_STR("StackCheck\n"));
                failures++;
            }
            else
            {
                /* Check the thread did not use up to the end of stack */
                if (free_bytes == 0)
                {
                    ATOMLOG (_STR("StackOverflow %d\n"), thread);
                    failures++;
                }

                /* Log the stack usage */
#ifdef TESTS_LOG_STACK_USAGE
                ATOMLOG (_STR("StackUse:%d\n"), (int)used_bytes);
#endif
            }
        }
    }
#endif

    /* Quit */
    return failures;
}

/**
 * \b test1_thread_func
 *
 * Entry point for test thread 1
 *
 * @param[in] param Unused (optional thread entry parameter)
 *
 * @return None
 */
static void test1_thread_func (uint32_t param)
{
    uint8_t status;
    uint32_t actual_flags_ptr;
    uint32_t requested_flags;
    /* Compiler warnings */
    param = param;

    /*
     * Wait on event2 get the flag. We are expecting to be woken up
     * by after the .event_set_tx_flag
     */
    requested_flags = (RX_COMPLETE_FLAG | RX_COMPLETE_FLAG);
    printf("test1_thread_func get requested_flags %d \n",(unsigned int)requested_flags);
    requested_flags=3;

    status = atomEventGet(&event1, requested_flags,ATOM_OR_CLEAR,&actual_flags_ptr,0xf0000001);//ATOM_WAIT_FOREVER

    printf("test1_thread_func status11 %d\n",status);//can't get also exec to here???
    if (status != ATOM_OK)
    {
        ATOMLOG (_STR("Test1 thread failed to wakeup (%d)\n"), status);
    }

    /* Wait forever */
    while (1)
    {
        atomTimerDelay (SYSTEM_TICKS_PER_SEC);
    }
}


/**
 * \b test2_thread_func
 *
 * Entry point for test thread 2.
 *
 * @param[in] param Unused (optional thread entry parameter)
 *
 * @return None
 */
static void test2_thread_func (uint32_t param)
{
    uint8_t status;
    uint32_t actual_flags_ptr;
    uint32_t requested_flags;
    /* Compiler warnings */
    param = param;

    /*
     * Wait on event2 get the flag. We are expecting to be woken up
     * by after the .event_set_tx_flag
     */
    requested_flags = (RX_COMPLETE_FLAG | RX_COMPLETE_FLAG);
    printf("test2_thread_func get requested_flags %d \n",(unsigned int)requested_flags);
    requested_flags=3;

    status = atomEventGet(&event2, requested_flags,ATOM_AND_CLEAR,&actual_flags_ptr,0xf0000001);//ATOM_WAIT_FOREVER

    printf("test2_thread_func status11 %d\n",status);//can't get also exec to here???
    if (status != ATOM_OK)
    {
        ATOMLOG (_STR("Test2 thread failed to wakeup (%d)\n"), status);
    }


    /* Wait forever */
    while (1)
    {
        atomTimerDelay (SYSTEM_TICKS_PER_SEC);
    }
}



static int event_set_rx_flag()
{
	int status;

	printf("event_set_rx_flag atom_event_current 0x%8x \n",(unsigned int)event2.atom_event_current);
    if ((status = atomEventSet(&event2, RX_COMPLETE_FLAG,ATOM_OR)) != ATOM_OK)
    {
        ATOMLOG (_STR("event_set_rx_flag (%d)\n"), status);
    }
    return status;
}


static int event_set_tx_flag()
{
	int status;


	printf("event_set_tx_flag atom_event_current 0x%8x\n",(unsigned int)event2.atom_event_current);
    if ((status = atomEventSet(&event2, TX_COMPLETE_FLAG,ATOM_OR)) != ATOM_OK)
    {
        ATOMLOG (_STR("event_set_tx_flag failed (%d)\n"), status);
    }

    return status;

}
