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
#include "atomqueue.h"
#include "atomtests.h"


/* Test queue size */
#define QUEUE_ENTRIES       8


/* Number of test threads */
#define NUM_TEST_THREADS      1


/* Test OS objects */
static ATOM_QUEUE queue1;
static uint32_t queue1_storage[QUEUE_ENTRIES];
static ATOM_TCB tcb[NUM_TEST_THREADS];
static uint8_t test_thread_stack[NUM_TEST_THREADS][TEST_THREAD_STACK_SIZE];


/* Test message values (more values than can fit in an entire 8 message queue) */
uint32_t test_values[] =
{
    0x12345678,
    0xFF000000,
    0x00FF0000,
    0x0000FF00,
    0x000000FF,
    0xF000000F,
    0x0F0000F0,
    0x00F00F00,
    0x000FF000,
    0x87654321,
    0xABCD0000,
    0x0000CDEF
};

/* Test result tracking */
static volatile int g_result;


/* Forward declarations */
static void test1_thread_func (uint32_t param);


/**
 * \b test_start
 *
 * Start queue test.
 *
 * This tests basic operation of queues.
 *
 * The main test thread creates a second thread and posts
 * a series of messages to the second thread. The message
 * values are checked against the expected values.
 *
 * We test using 4-byte messages.
 *
 * @retval Number of failures
 */
uint32_t test_start (void)
{
    int failures, count;
    int num_entries;
    uint32_t msg;

    /* Default to zero failures */
    failures = 0;
    g_result = 0;

    /* Create test queue */
    if (atomQueueCreate (&queue1, (uint8_t *)&queue1_storage[0], sizeof(queue1_storage[0]), QUEUE_ENTRIES) != ATOM_OK)
    {
        ATOMLOG (_STR("Error creating test queue\n"));
        failures++;
    }

    /* Create a test thread that will block because the queue is empty */
    else if (atomThreadCreate(&tcb[0], TEST_THREAD_PRIO + 1, test1_thread_func, 0,
              &test_thread_stack[0][0],
              TEST_THREAD_STACK_SIZE, TRUE) != ATOM_OK)
    {
        /* Fail */
        ATOMLOG (_STR("Error creating test thread 1\n"));
        failures++;
    }
    else
    {

        /*
         * We have created an empty queue and a thread which should now
         * be blocking on the queue. The test thread is lower priority
         * than us.
         */

        /* Wait for the other thread to start blocking on queue1 */
        if (atomTimerDelay(SYSTEM_TICKS_PER_SEC) != ATOM_OK)
        {
            ATOMLOG (_STR("Failed timer delay\n"));
            failures++;
        }
        else
        {
            /*
             * Post all entries in the test array to the queue.
             * Because the second thread is lower priority than
             * us, we will post 8 messages until the queue is
             * full without waking up the second thread at all.
             * At that point, we will block and the second
             * thread will remove one message from the queue.
             * With a spare entry in the queue, this thread
             * will wake up again and post another message.
             * This will continue until this thread has posted
             * all messages, at which point the second thread
             * will drain all remaining messages from the
             * queue.
             *
             * Through this scheme we are able to test posting
             * to the queue at all possible fill levels.
             */
            num_entries = sizeof(test_values) / sizeof(test_values[0]);
            for (count = 0; count < num_entries; count++)
            {
                /* Increment through and post all test values to the queue */
                msg = test_values[count];
                if (atomQueuePut (&queue1, 0, (uint8_t *)&msg) != ATOM_OK)
                {
                    ATOMLOG (_STR("Failed post\n"));
                    failures++;
                }
            }

            /* Sleep a while for the second thread to finish */
            atomTimerDelay (SYSTEM_TICKS_PER_SEC);

            /* Check that the second thread has found all test values */
            if (g_result != 1)
            {
                ATOMLOG (_STR("Bad test vals\n"));
                failures++;
            }
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
 * \b test1_thread_func
 *
 * Entry point for test thread 1.
 *
 * @param[in] param Unused (optional thread entry parameter)
 *
 * @return None
 */
static void test1_thread_func (uint32_t param)
{
    uint32_t msg;
    int num_entries, count, failures;

    /* Compiler warnings */
    param = param;

    /* Default to no errors */
    failures = 0;

    /*
     * Loop receiving messages until we have received the number of
     * values in the test array.
     */
    num_entries = sizeof(test_values) / sizeof(test_values[0]);
    for (count = 0; count < num_entries; count++)
    {
        /* Receive a value from the queue */
        if (atomQueueGet (&queue1, 0, (uint8_t *)&msg) != ATOM_OK)
        {
            ATOMLOG (_STR("Failed get\n"));
            failures++;
        }

        /* Check that we received the expected value */
        else if (msg != test_values[count])
        {
            ATOMLOG (_STR("Val%d\n"), count);
            failures++;
        }
    }

    /*
     * Set g_result to indicate success if we had no failures.
     * Thread-protection is not required on g_result because it
     * is only ever set by this thread.
     */
    if (failures == 0)
    {
        /* No failures */
        g_result = 1;
    }

    /* Wait forever */
    while (1)
    {
        atomTimerDelay (SYSTEM_TICKS_PER_SEC);
    }
}
