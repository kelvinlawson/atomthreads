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


/* Test queue size */
#define QUEUE_ENTRIES       8


/* Test OS objects */
static ATOM_QUEUE queue1;
static uint8_t queue1_storage[QUEUE_ENTRIES];


/**
 * \b test_start
 *
 * Start queue test.
 *
 * This tests timeouts on a queue. We make a thread block with timeout
 * on a queue, and test that sufficient time has actually passed as
 * was requested by the timeout parameter.
 *
 * @retval Number of failures
 */
uint32_t test_start (void)
{
    int failures;
    uint32_t start_time, end_time;
    uint8_t msg;

    /* Default to zero failures */
    failures = 0;

    /* Create queue */
    if (atomQueueCreate (&queue1, &queue1_storage[0], sizeof(queue1_storage[0]), QUEUE_ENTRIES) != ATOM_OK)
    {
        ATOMLOG (_STR("Error creating Q\n"));
        failures++;
    }

    else
    {
        /* The queue is empty so atomQueueGet() calls will block */

        /* Take note of the start time */
        start_time = atomTimeGet();

        /* Block on the queue with two second timeout */
        if (atomQueueGet (&queue1, 2 * SYSTEM_TICKS_PER_SEC, &msg) != ATOM_TIMEOUT)
        {
            ATOMLOG (_STR("Failed get\n"));
            failures++;
        }

        /* Take note of the end time */
        end_time = atomTimeGet();

        /* Now check that two seconds have passed */
        if ((end_time < (start_time + (2 * SYSTEM_TICKS_PER_SEC)))
            || (end_time > (start_time + (2 * SYSTEM_TICKS_PER_SEC) + 1)))
        {
            ATOMLOG (_STR("Bad time\n"));
            failures++;
        }

        /* Delete queue, test finished */
        if (atomQueueDelete (&queue1) != ATOM_OK)
        {
            ATOMLOG (_STR("Delete failed\n"));
            failures++;
        }
    }

    /* Quit */
    return failures;

}
