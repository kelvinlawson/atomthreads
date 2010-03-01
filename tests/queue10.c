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


/* Test OS objects */
static ATOM_QUEUE queue1;
static uint32_t queue1_storage[QUEUE_ENTRIES];


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


/**
 * \b test_start
 *
 * Start queue test.
 *
 * This tests basic operation of queues.
 *
 * Messages are posted to and received from the queue and checked
 * against expected values. To ensure correct ordering, queue posts
 * and receives are done in blocks of different amounts, such that
 * there will already be different numbers of messages in the queue
 * whenever messages are posted and received.
 *
 * We test using 4-byte messages.
 *
 * @retval Number of failures
 */
uint32_t test_start (void)
{
    int failures, tx_count, rx_count;
    uint32_t msg;

    /* Default to zero failures */
    failures = 0;

    /* Create test queue */
    if (atomQueueCreate (&queue1, (uint8_t *)&queue1_storage[0], sizeof(queue1_storage[0]), QUEUE_ENTRIES) != ATOM_OK)
    {
        ATOMLOG (_STR("Error creating test queue\n"));
        failures++;
    }

    else
    {
        /* Reset tx/rx counts */
        tx_count = rx_count = 0;

        /* Post 2 messages to the queue */
        for (; tx_count < 2; tx_count++)
        {
            msg = test_values[tx_count];
            if (atomQueuePut (&queue1, 0, (uint8_t *)&msg) != ATOM_OK)
            {
                ATOMLOG (_STR("Failed post\n"));
                failures++;
            }
        }

        /* Receive 1 message from the queue */
        for (; rx_count < 2; rx_count++)
        {
            if (atomQueueGet (&queue1, 0, (uint8_t *)&msg) != ATOM_OK)
            {
                ATOMLOG (_STR("Failed get\n"));
                failures++;
            }
            else if (msg != test_values[rx_count])
            {
                ATOMLOG (_STR("Val%d\n"), rx_count);
                failures++;
            }
        }

        /* Post 3 messages to the queue */
        for (; tx_count < 5; tx_count++)
        {
            msg = test_values[tx_count];
            if (atomQueuePut (&queue1, 0, (uint8_t *)&msg) != ATOM_OK)
            {
                ATOMLOG (_STR("Failed post\n"));
                failures++;
            }
        }

        /* Receive 2 messages from the queue */
        for (; rx_count < 3; rx_count++)
        {
            if (atomQueueGet (&queue1, 0, (uint8_t *)&msg) != ATOM_OK)
            {
                ATOMLOG (_STR("Failed get\n"));
                failures++;
            }
            else if (msg != test_values[rx_count])
            {
                ATOMLOG (_STR("Val%d\n"), rx_count);
                failures++;
            }
        }

        /* Post 5 messages to the queue */
        for (; tx_count < 10; tx_count++)
        {
            msg = test_values[tx_count];
            if (atomQueuePut (&queue1, 0, (uint8_t *)&msg) != ATOM_OK)
            {
                ATOMLOG (_STR("Failed post\n"));
                failures++;
            }
        }

        /* Receive 3 messages from the queue */
        for (; rx_count < 6; rx_count++)
        {
            if (atomQueueGet (&queue1, 0, (uint8_t *)&msg) != ATOM_OK)
            {
                ATOMLOG (_STR("Failed get\n"));
                failures++;
            }
            else if (msg != test_values[rx_count])
            {
                ATOMLOG (_STR("Val%d\n"), rx_count);
                failures++;
            }
        }

        /* Post 2 messages to the queue */
        for (; tx_count < 12; tx_count++)
        {
            msg = test_values[tx_count];
            if (atomQueuePut (&queue1, 0, (uint8_t *)&msg) != ATOM_OK)
            {
                ATOMLOG (_STR("Failed post\n"));
                failures++;
            }
        }

        /* Receive 6 messages from the queue */
        for (; rx_count < 12; rx_count++)
        {
            if (atomQueueGet (&queue1, 0, (uint8_t *)&msg) != ATOM_OK)
            {
                ATOMLOG (_STR("Failed get\n"));
                failures++;
            }
            else if (msg != test_values[rx_count])
            {
                ATOMLOG (_STR("Val%d\n"), rx_count);
                failures++;
            }
        }
    }

    /* Quit */
    return failures;
}
