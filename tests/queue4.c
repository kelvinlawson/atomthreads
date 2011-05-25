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


/*   Number of queue entries */
#define QUEUE_ENTRIES           8


/* Test OS objects */
static ATOM_QUEUE queue1, queue2;
static uint8_t queue1_storage[QUEUE_ENTRIES];
static uint8_t queue2_storage[QUEUE_ENTRIES];


/* Test result tracking */
static volatile int g_result;


/* Forward declarations */
static void testCallbackGet (POINTER cb_data);
static void testCallbackPut (POINTER cb_data);


/**
 * \b test_start
 *
 * Start queue test.
 *
 * This test exercises the atomQueueGet() and atomQueuePut() APIs
 * particularly forcing the various error indications which can be
 * returned from the APIs to ensure that handling for these corner
 * cases has been correctly implemented.
 *
 * @retval Number of failures
 */
uint32_t test_start (void)
{
    int failures;
    uint8_t msg;
    ATOM_TIMER timer_cb;
    int count;

    /* Default to zero failures */
    failures = 0;

    /* Create two test queues: queue1 is empty, queue2 is full */

    /* Empty queue1 creation */
    if (atomQueueCreate (&queue1, &queue1_storage[0], sizeof(uint8_t), QUEUE_ENTRIES) != ATOM_OK)
    {
        ATOMLOG (_STR("Queue1 create\n"));
        failures++;
    }

    /* Full queue2 creation */
    if (atomQueueCreate (&queue2, &queue2_storage[0], sizeof(uint8_t), QUEUE_ENTRIES) != ATOM_OK)
    {
        ATOMLOG (_STR("Queue2 create\n"));
        failures++;
    }
    else
    {
        /* Fill the queue */
        msg = 0x66;
        for (count = 0; count < QUEUE_ENTRIES; count++)
        {
            /* Add one message at a time */
            if (atomQueuePut (&queue2, 0, &msg) != ATOM_OK)
            {
                ATOMLOG (_STR("Queue2 put\n"));
                failures++;
            }
        }
    }

    /* Test parameter checks */
    if (atomQueueGet (NULL, 0, &msg) != ATOM_ERR_PARAM)
    {
        ATOMLOG (_STR("Get queue param failed\n"));
        failures++;
    }
    if (atomQueueGet (&queue1, 0, NULL) != ATOM_ERR_PARAM)
    {
        ATOMLOG (_STR("Get msg param failed\n"));
        failures++;
    }
    if (atomQueuePut (NULL, 0, &msg) != ATOM_ERR_PARAM)
    {
        ATOMLOG (_STR("Put queue param failed\n"));
        failures++;
    }
    if (atomQueuePut (&queue1, 0, NULL) != ATOM_ERR_PARAM)
    {
        ATOMLOG (_STR("Put msg param failed\n"));
        failures++;
    }

    /* Test atomQueueGet() can not be called from interrupt context */
    g_result = 0;

    /* Fill out the timer callback request structure */
    timer_cb.cb_func = testCallbackGet;
    timer_cb.cb_data = NULL;
    timer_cb.cb_ticks = SYSTEM_TICKS_PER_SEC;

    /* Request the timer callback to run in one second */
    if (atomTimerRegister (&timer_cb) != ATOM_OK)
    {
        ATOMLOG (_STR("Error registering timer\n"));
        failures++;
    }

    /* Wait two seconds for g_result to be set indicating success */
    else
    {
        atomTimerDelay (2 * SYSTEM_TICKS_PER_SEC);
        if (g_result != 1)
        {
            ATOMLOG (_STR("Get context check failed\n"));
            failures++;
        }
    }

    /* Test atomQueuePut() can not be called from interrupt context */
    g_result = 0;

    /* Fill out the timer callback request structure */
    timer_cb.cb_func = testCallbackPut;
    timer_cb.cb_data = NULL;
    timer_cb.cb_ticks = SYSTEM_TICKS_PER_SEC;

    /* Request the timer callback to run in one second */
    if (atomTimerRegister (&timer_cb) != ATOM_OK)
    {
        ATOMLOG (_STR("Error registering timer\n"));
        failures++;
    }

    /* Wait two seconds for g_result to be set indicating success */
    else
    {
        atomTimerDelay (2 * SYSTEM_TICKS_PER_SEC);
        if (g_result != 1)
        {
            ATOMLOG (_STR("Put context check failed\n"));
            failures++;
        }
    }

    /* Test ATOM_TIMEOUT is returned for Get/Put calls with timeout */

    /* Attempt atomQueueGet() on empty queue to force timeout */
    if (atomQueueGet (&queue1, SYSTEM_TICKS_PER_SEC, &msg) != ATOM_TIMEOUT)
    {
        ATOMLOG (_STR("Timeout q1 failed\n"));
        failures++;
    }

    /* Attempt atomQueuePut() on full queue to force timeout */
    msg = 0x66;
    if (atomQueuePut (&queue2, SYSTEM_TICKS_PER_SEC, &msg) != ATOM_TIMEOUT)
    {
        ATOMLOG (_STR("Timeout q2 failed\n"));
        failures++;
    }

    /* Test ATOM_WOULDBLOCK is returned for Get/Put calls with -1 timeout */

    /* Attempt atomQueueGet() on empty queue to force block */
    if (atomQueueGet (&queue1, -1, &msg) != ATOM_WOULDBLOCK)
    {
        ATOMLOG (_STR("Timeout q1 failed\n"));
        failures++;
    }

    /* Attempt atomQueuePut() on full queue to force block */
    msg = 0x66;
    if (atomQueuePut (&queue2, -1, &msg) != ATOM_WOULDBLOCK)
    {
        ATOMLOG (_STR("Timeout q2 failed\n"));
        failures++;
    }

    /* Delete the test queues */
    if (atomQueueDelete (&queue1) != ATOM_OK)
    {
        ATOMLOG (_STR("Error deleting q1\n"));
        failures++;
    }
    if (atomQueueDelete (&queue2) != ATOM_OK)
    {
        ATOMLOG (_STR("Error deleting q2\n"));
        failures++;
    }

    /* Quit */
    return failures;
}


/**
 * \b testCallbackGet
 *
 * Attempt an atomQueueGet() on (empty) queue1 from interrupt context.
 * Should receive an ATOM_ERR_CONTEXT error. Sets g_result if passes.
 *
 * @param[in] cb_data Not used
 */
static void testCallbackGet (POINTER cb_data)
{
    uint8_t msg;

    /* Check the return value from atomQueueGet() */
    if (atomQueueGet(&queue1, 0, &msg) == ATOM_ERR_CONTEXT)
    {
        /* Received the error we expected, set g_result to notify success */
        g_result = 1;
    }
    else
    {
        /* Did not get expected error, don't set g_result signifying fail */
    }

}


/**
 * \b testCallbackPut
 *
 * Attempt an atomQueuePut() on (full) queue2 from interrupt context.
 * Should receive an ATOM_ERR_CONTEXT error. Sets g_result if passes.
 *
 * @param[in] cb_data Not used
 */
static void testCallbackPut (POINTER cb_data)
{
    uint8_t msg;

    /* Check the return value from atomQueuePut() */
    msg = 0x66;
    if (atomQueuePut(&queue2, 0, &msg) == ATOM_ERR_CONTEXT)
    {
        /* Received the error we expected, set g_result to notify success */
        g_result = 1;
    }
    else
    {
        /* Did not get expected error, don't set g_result signifying fail */
    }

}
