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
#include "atomsem.h"
#include "atomtests.h"


/* Test OS objects */
static ATOM_SEM sem1, sem2;


/* Forward declarations */
static void testCallback (POINTER cb_data);


/**
 * \b test_start
 *
 * Start semaphore test.
 *
 * This test exercises the atomSemGet() and atomSemPut() APIs including
 * forcing the various error indications which can be returned from the
 * APIs to ensure that handling for these corner cases have been correctly
 * implemented.
 *
 * @retval Number of failures
 */
uint32_t test_start (void)
{
    int failures;
    uint8_t status;
    ATOM_TIMER timer_cb;

    /* Default to zero failures */
    failures = 0;

    /* Test semaphore wait timeouts */
    if (atomSemCreate (&sem1, 0) != ATOM_OK)
    {
        ATOMLOG (_STR("Error creating test semaphore 1\n"));
        failures++;
    }
    else
    {
        /* Wait on semaphore with timeout */
        if ((status = atomSemGet (&sem1, SYSTEM_TICKS_PER_SEC)) != ATOM_TIMEOUT)
        {
            ATOMLOG (_STR("Timo %d\n"), status);
            failures++;
        }
        else
        {
            /* Test semaphore still operates correctly after timeout */
            if (atomSemPut (&sem1) != ATOM_OK)
            {
                ATOMLOG (_STR("Put failed\n"));
                failures++;
            }
            else
            {
                /* Count should now be 1 */
                if (atomSemGet (&sem1, 0) != ATOM_OK)
                {
                    ATOMLOG (_STR("Get failed\n"));
                    failures++;
                }
                else
                {
                    /* Delete it, test finished */
                    if (atomSemDelete (&sem1) != ATOM_OK)
                    {
                        ATOMLOG (_STR("Delete failed\n"));
                        failures++;
                    }
                }
            }
        }
    }

    /* Test semaphore blocks if count is zero */
    if (atomSemCreate (&sem1, 0) != ATOM_OK)
    {
        ATOMLOG (_STR("Error creating test semaphore 1\n"));
        failures++;
    }
    else
    {
        /* Wait on semaphore with timeout */
        if ((status = atomSemGet (&sem1, SYSTEM_TICKS_PER_SEC)) != ATOM_TIMEOUT)
        {
            ATOMLOG (_STR("Timo %d\n"), status);
            failures++;
        }
        else
        {
            /* Delete it, test finished */
            if (atomSemDelete (&sem1) != ATOM_OK)
            {
                ATOMLOG (_STR("Delete failed\n"));
                failures++;
            }
        }
    }

    /* Test semaphore does not block if count is one */
    if (atomSemCreate (&sem1, 1) != ATOM_OK)
    {
        ATOMLOG (_STR("Error creating test semaphore 1\n"));
        failures++;
    }
    else
    {
        /* Wait on semaphore with timeout */
        if ((status = atomSemGet (&sem1, SYSTEM_TICKS_PER_SEC)) != ATOM_OK)
        {
            ATOMLOG (_STR("Get %d\n"), status);
            failures++;
        }
        else
        {
            /* Delete it, test finished */
            if (atomSemDelete (&sem1) != ATOM_OK)
            {
                ATOMLOG (_STR("Delete failed\n"));
                failures++;
            }
        }
    }

    /* Test parameter checks */
    if (atomSemGet (NULL, 0) != ATOM_ERR_PARAM)
    {
        ATOMLOG (_STR("Get param failed\n"));
        failures++;
    }
    if (atomSemPut (NULL) != ATOM_ERR_PARAM)
    {
        ATOMLOG (_STR("Put param failed\n"));
        failures++;
    }

    /* Test atomSemGet() cannot be called from interrupt context */
    if (atomSemCreate (&sem1, 0) != ATOM_OK)
    {
        ATOMLOG (_STR("Error creating test sem1\n"));
        failures++;
    }
    else if (atomSemCreate (&sem2, 0) != ATOM_OK)
    {
        ATOMLOG (_STR("Error creating test sem2\n"));
        failures++;
    }
    else
    {
        /* Fill out the timer callback request structure */
        timer_cb.cb_func = testCallback;
        timer_cb.cb_data = NULL;
        timer_cb.cb_ticks = SYSTEM_TICKS_PER_SEC;

        /* Request the timer callback to run in one second */
        if (atomTimerRegister (&timer_cb) != ATOM_OK)
        {
            ATOMLOG (_STR("Error registering timer\n"));
            failures++;
        }

        /* Wait up to two seconds for sem2 to be posted indicating success */
        else if (atomSemGet (&sem2, 2 * SYSTEM_TICKS_PER_SEC) != ATOM_OK)
        {
            ATOMLOG (_STR("Context check failed\n"));
            failures++;
        }

        /* Delete the two test semaphores */
        if (atomSemDelete (&sem1) != ATOM_OK)
        {
            ATOMLOG (_STR("Sem1 delete failed\n"));
            failures++;
        }
        if (atomSemDelete (&sem2) != ATOM_OK)
        {
            ATOMLOG (_STR("Sem2 delete failed\n"));
            failures++;
        }
    }

    /* Test for ATOM_WOULDBLOCK */
    if (atomSemCreate (&sem1, 0) != ATOM_OK)
    {
        ATOMLOG (_STR("Error creating test sem1\n"));
        failures++;
    }
    else
    {
        /* Semaphore count is zero so will block */
        if ((status = atomSemGet (&sem1, -1)) != ATOM_WOULDBLOCK)
        {
            ATOMLOG (_STR("Wouldblock err %d\n"), status);
            failures++;
        }

        /* Delete the test semaphore */
        if (atomSemDelete (&sem1) != ATOM_OK)
        {
            ATOMLOG (_STR("Sem1 delete failed\n"));
            failures++;
        }

    }

    /* Test no timeout */
    if (atomSemCreate (&sem1, 0) != ATOM_OK)
    {
        ATOMLOG (_STR("Error creating test sem1\n"));
        failures++;
    }
    else
    {
        /* Increment the semaphore */
        if (atomSemPut (&sem1) != ATOM_OK)
        {
            ATOMLOG (_STR("Error on put\n"));
            failures++;
        }

        /* Semaphore count is one so will not block */
        if (atomSemGet (&sem1, -1) != ATOM_OK)
        {
            ATOMLOG (_STR("Error on get\n"));
            failures++;
        }

        /* Delete the test semaphore */
        if (atomSemDelete (&sem1) != ATOM_OK)
        {
            ATOMLOG (_STR("Sem1 delete failed\n"));
            failures++;
        }

    }

    /* Test for semaphore counter overflows with too many puts */
    if (atomSemCreate (&sem1, 255) != ATOM_OK)
    {
        ATOMLOG (_STR("Error creating test sem1\n"));
        failures++;
    }
    else
    {
        /* Increment the semaphore (expect this to overflow the count at 256) */
        if (atomSemPut (&sem1) != ATOM_ERR_OVF)
        {
            ATOMLOG (_STR("Failed to detect overflow\n"));
            failures++;
        }

        /* Delete the test semaphore */
        if (atomSemDelete (&sem1) != ATOM_OK)
        {
            ATOMLOG (_STR("Sem1 delete failed\n"));
            failures++;
        }

    }

    /* Quit */
    return failures;
}


/**
 * \b testCallback
 *
 * Attempt an atomSemGet() on sem1 from interrupt context.
 * Should receive an ATOM_ERR_CONTEXT error. Posts sem2 if successful.
 *
 * @param[in] cb_data Not used
 */
static void testCallback (POINTER cb_data)
{
    /* Check the return value from atomSemGet() */
    if (atomSemGet(&sem1, 0) == ATOM_ERR_CONTEXT)
    {
        /* Received the error we expected, post sem2 to notify success */
        atomSemPut(&sem2);
    }
    else
    {
        /*
         * Did not get the expected error, don't post sem2 and the test
         * thread will time out, signifying an error.
         */
    }

}
