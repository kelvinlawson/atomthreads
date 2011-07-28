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


#include "atom.h"
#include "atomtests.h"
#include "atomqueue.h"


/*   Number of queue entries */
#define QUEUE_ENTRIES           4


/* Number of test threads */
#define NUM_TEST_THREADS      4


/* Test OS objects */
static ATOM_QUEUE queue1;
static uint8_t queue1_storage[QUEUE_ENTRIES];
static ATOM_TCB tcb[NUM_TEST_THREADS];
static uint8_t test_thread_stack[NUM_TEST_THREADS][TEST_THREAD_STACK_SIZE];


/* Data updated by threads */
static volatile uint8_t wake_cnt;
static volatile uint8_t wake_order[4];

/* Forward declarations */
static void test_thread_func (uint32_t param);


/**
 * \b test_start
 *
 * Start queue test.
 *
 * With multiple threads blocking on a single queue, this test confirms that
 * they are woken in order when the queue is posted. The correct order for
 * waking is that the higher priority threads are woken first, followed by the
 * lower priority threads. Where multiple threads of the same priority are
 * waiting, the threads are woken in FIFO order (the order in which they started
 * waiting on the queue).
 *
 * To test this we create four threads which all wait on a single queue.
 * One pair of threads are running at high priority, with the other pair at a
 * lower priority:
 *
 * Thread 1: low prio thread A
 * Thread 2: low prio thread B
 * Thread 3: high prio thread A
 * Thread 4: high prio thread B
 *
 * The threads are forced to start blocking on the same queue in the
 * above order.
 *
 * We expect to see them woken up in the following order:
 *  3, 4, 1, 2
 *
 * This proves the multiple blocking thread ordering in terms of both
 * the priority-queueing and same-priority-FIFO-queueing.
 *
 * @retval Number of failures
 */
uint32_t test_start (void)
{
    int failures, count;
    uint8_t msg;

    /* Default to zero failures */
    failures = 0;

    /* Create empty queue */
    if (atomQueueCreate (&queue1, &queue1_storage[0], sizeof(uint8_t), QUEUE_ENTRIES) != ATOM_OK)
    {
        ATOMLOG (_STR("Error creating test q1\n"));
        failures++;
    }

    /* Start the threads */
    else
    {
        /*
         * The test threads all start by calling atomQueueGet() to receive
         * a message from the queue. Because the queue is empty, all test
         * threads should immediately block on the queue (until a message
         * is posted to it).
         */

        /* Create Thread 1 (lower priority thread A) */
        if (atomThreadCreate(&tcb[0], TEST_THREAD_PRIO+1, test_thread_func, 1,
              &test_thread_stack[0][0],
              TEST_THREAD_STACK_SIZE, TRUE) != ATOM_OK)
        {
            /* Fail */
            ATOMLOG (_STR("Error creating test thread\n"));
            failures++;
        }

        /* Delay to ensure the thread will start blocking on the queue */
        atomTimerDelay (SYSTEM_TICKS_PER_SEC/4);

        /* Create Thread 2 (lower priority thread B) */
        if (atomThreadCreate(&tcb[1], TEST_THREAD_PRIO+1, test_thread_func, 2,
              &test_thread_stack[1][0],
              TEST_THREAD_STACK_SIZE, TRUE) != ATOM_OK)
        {
            /* Fail */
            ATOMLOG (_STR("Error creating test thread\n"));
            failures++;
        }

        /* Delay to ensure the thread will start blocking on the queue */
        atomTimerDelay (SYSTEM_TICKS_PER_SEC/4);

        /* Create Thread 3 (higher priority thread A) */
        if (atomThreadCreate(&tcb[2], TEST_THREAD_PRIO, test_thread_func, 3,
              &test_thread_stack[2][0],
              TEST_THREAD_STACK_SIZE, TRUE) != ATOM_OK)
        {
            /* Fail */
            ATOMLOG (_STR("Error creating test thread\n"));
            failures++;
        }

        /* Delay to ensure the thread will start blocking on the queue */
        atomTimerDelay (SYSTEM_TICKS_PER_SEC/4);

        /* Create Thread 4 (higher priority thread B) */
        if (atomThreadCreate(&tcb[3], TEST_THREAD_PRIO, test_thread_func, 4,
              &test_thread_stack[3][0],
              TEST_THREAD_STACK_SIZE, TRUE) != ATOM_OK)
        {
            /* Fail */
            ATOMLOG (_STR("Error creating test thread\n"));
            failures++;
        }

        /* Delay to ensure the thread will start blocking on the queue */
        atomTimerDelay (SYSTEM_TICKS_PER_SEC/4);

        /* All four threads will now be blocking on queue1 */

        /*
         * Initialise wake count, used by threads to determine
         * what order they were woken in.
         */
        wake_cnt = 0;

        /* Loop waking all four threads */
        for (count = 0; count < 4; count++)
        {
            /*
             * Post a message to the queue. This will wake up one of the threads
             * blocking on it (because it is currently empty). That thread will
             * wake up, note the order at which it was woken, then go to sleep
             * forever leaving the queue empty again. This is done four times so
             * that all four threads are woken, noting their wake order.
             */
            msg = 0x66;
            if (atomQueuePut (&queue1, 0, &msg) != ATOM_OK)
            {
                ATOMLOG (_STR("Post fail\n"));
                failures++;
            }

            /*
             * Sleep to give the thread time to wake up and modify the shared
             * global data wake_cnt and wake_order[]. We deliberately do not
             * use a mutex for protecting access to this shared data, as we
             * are testing the queue module in isolation here.
             */
            atomTimerDelay (SYSTEM_TICKS_PER_SEC / 4);

        }

        /* All four threads now woken up, check they woke in correct order */
        if ((wake_order[0] != 3) && (wake_order[1] != 4)
            && (wake_order[2] != 1) && (wake_order[3] != 2))
        {
            ATOMLOG (_STR("Bad order %d,%d,%d,%d\n"),
                wake_order[0], wake_order[1], wake_order[2], wake_order[3]);
            failures++;
        }

        /* Delete queue, test finished */
        if (atomQueueDelete (&queue1) != ATOM_OK)
        {
            ATOMLOG (_STR("Delete failed\n"));
            failures++;
        }
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
 * \b test_thread_func
 *
 * Entry point for test thread. The same thread entry point is used for all
 * four test threads, with the thread number/ID (1-4) passed as the entry
 * point parameter.
 *
 * @param[in] param Thread number (1,2,3,4)
 *
 * @return None
 */
static void test_thread_func (uint32_t param)
{
    uint8_t thread_id;
    uint8_t msg;

    /* Thread ID is passed through the function parameter */
    thread_id = (uint8_t)param;

    /*
     * Wait for a message to appear on queue1. At creation of all test
     * threads the queue is empty, so all four threads will block here.
     */
    if (atomQueueGet (&queue1, 0, &msg) != ATOM_OK)
    {
        ATOMLOG (_STR("Get fail\n"));
    }
    else
    {
        /*
         * Store our thread ID in the array using the current
         * wake_cnt order. The threads are woken with large
         * pauses between, which provides protection for this
         * global data. This allows us to test queues without
         * assuming a working implementation of a mutex (or
         * similar protection mechanism).
         */
        wake_order[wake_cnt++] = thread_id;

    }

    /* Loop forever */
    while (1)
    {
        atomTimerDelay (SYSTEM_TICKS_PER_SEC);
    }
}
