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
#define QUEUE_ENTRIES       16


/* Test OS objects */
static ATOM_QUEUE queue1;
static uint8_t queue1_storage[QUEUE_ENTRIES];


/**
 * \b test_start
 *
 * Start queue test.
 *
 * This test exercises the queue creation and deletion APIs.
 *
 * Testing of deletion while threads are actually blocking in
 * queue APIs is tested in queue2.c and queue3.c.
 *
 * @retval Number of failures
 */
uint32_t test_start (void)
{
    int failures;
    uint32_t i;

    /* Default to zero failures */
    failures = 0;

    /* Test creation and deletion of queues: good values */
    for (i = 0; i < 1000; i++)
    {
        if (atomQueueCreate (&queue1, &queue1_storage[0], sizeof(uint8_t), QUEUE_ENTRIES) == ATOM_OK)
        {
            if (atomQueueDelete (&queue1) == ATOM_OK)
            {
                /* Success */
            }
            else
            {
                /* Fail */
                ATOMLOG (_STR("Error deleting queue\n"));
                failures++;
                break;
            }
        }
        else
        {
            /* Fail */
            ATOMLOG (_STR("Error creating queue\n"));
            failures++;
            break;
        }
    }

    /* Test creation and deletion of queues: creation checks */
    if (atomQueueCreate (NULL, &queue1_storage[0], sizeof(uint8_t), QUEUE_ENTRIES) != ATOM_OK)
    {
        /* Success */
    }
    else
    {
        /* Fail */
        ATOMLOG (_STR("Bad queue ptr check\n"));
        failures++;
    }
    if (atomQueueCreate (&queue1, NULL, sizeof(uint8_t), QUEUE_ENTRIES) != ATOM_OK)
    {
        /* Success */
    }
    else
    {
        /* Fail */
        ATOMLOG (_STR("Bad buff ptr check\n"));
        failures++;
    }
    if (atomQueueCreate (&queue1, &queue1_storage[0], 0, QUEUE_ENTRIES) != ATOM_OK)
    {
        /* Success */
    }
    else
    {
        /* Fail */
        ATOMLOG (_STR("Bad size check\n"));
        failures++;
    }
    if (atomQueueCreate (&queue1, &queue1_storage[0], sizeof(uint8_t), 0) != ATOM_OK)
    {
        /* Success */
    }
    else
    {
        /* Fail */
        ATOMLOG (_STR("Bad entries check\n"));
        failures++;
    }

    /* Test creation and deletion of queues: deletion checks */
    if (atomQueueDelete (NULL) != ATOM_OK)
    {
        /* Success */
    }
    else
    {
        /* Fail */
        ATOMLOG (_STR("Bad queue deletion checks\n"));
        failures++;
    }

    /* Quit */
    return failures;
}
