/*
 * Copyright (c) 2013, Kelvin Lawson. All rights reserved.
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
 * Driver for accurate high-speed hardware timers.
 *
 * Uses the TIMER1 hardware module of DM36x.
 */

#include "atom.h"
#include "atomport.h"
#include "dm36x-io.h"


/* Constants */

/** Register access macros: using TIMER1 */
#define TIMER_REG(offset)      *(volatile uint32_t *)(DM36X_TIMER1_BASE + offset)


/* Local data */

/*
 * Initialised flag
 */
static int initialised = FALSE;


/* Forward declarations */
static int timer_init (void);


/**
 * \b timer_init
 *
 * Initialisation of TIMER1 hardware.
 *
 * @retval ATOM_OK Success
 * @retval ATOM_ERROR Failed
 */
static int timer_init (void)
{
    int status;

    /* Check we are not already initialised */
    if (initialised == FALSE)
    {
        /* Initialise TIMER1 registers for free-running high-speed 24MHz timer */

        /* Reset & disable all TIMER1 timers */
        TIMER_REG(DM36X_TIMER_INTCTL_STAT) = 0;    /* Disable interrupts */
        TIMER_REG(DM36X_TIMER_TCR) = 0;            /* Disable all TIMER1 timers */
        TIMER_REG(DM36X_TIMER_TGCR) = 0;           /* Put all TIMER1 timers in reset */
        TIMER_REG(DM36X_TIMER_TIM12) = 0;          /* Clear Timer 1:2 */

        /* Set up Timer 1:2 in 32-bit unchained mode */
        TIMER_REG(DM36X_TIMER_TGCR) = (1 << 2);    /* Select 32-bit unchained mode (TIMMODE) */
        TIMER_REG(DM36X_TIMER_TGCR) |= (1 << 0);   /* Remove Timer 1:2 from reset (TIM12RS) */
        TIMER_REG(DM36X_TIMER_PRD12) = ~0;         /* Set period to free-running 24MHz clock (PRD12) */
        TIMER_REG(DM36X_TIMER_TCR) |= (0 << 8);    /* Select external clock source for Timer 1:2 (CLKSRC12) */

        /* Enable timer */
        TIMER_REG(DM36X_TIMER_TCR) |= (2 << 6);    /* Enable Timer 1:2 continuous (ENAMODE12) */

        /* Success */
        initialised = TRUE;
        status = ATOM_OK;
    }

    /* Already initialised */
    else
    {
        /* Success */
        status = ATOM_OK;
    }

    /* Finished */
    return (status);
}


/**
 * \b archUsleep
 *
 * Simple spin loop of at least the specified microseconds.
 *
 * @param[in] microsecs Number of microseconds to sleep
 *
 * @return None
 *
 */
void archUsleep (int32_t microsecs)
{
    int32_t start_time, delay_timer_ticks;

    /* Check we are initialised */
    if (initialised == FALSE)
    {
        timer_init();
    }

    /* Get the current 24MHz count */
    start_time = TIMER_REG(DM36X_TIMER_TIM12);

    /* Translate delay in usecs to delay in 24MHz ticks */
    delay_timer_ticks = ((TIMER_CLK / 1000000) * microsecs);

    /* Wait in a spin-loop for timer to expire */
    while (((int32_t)TIMER_REG(DM36X_TIMER_TIM12) - start_time) < delay_timer_ticks)
        ;
}


/**
 * \b archUsleepStart
 *
 * Start a usleep timer session.
 *
 * @retval Start time for use in subsequent archUsleepCheckExpired() calls
 *
 */
int32_t archUsleepStart (void)
{
    /* Check we are initialised */
    if (initialised == FALSE)
    {
        timer_init();
    }

    /* Return the current 24MHz count */
    return (TIMER_REG(DM36X_TIMER_TIM12));
}


/**
 * \b archUsleepCheckExpired
 *
 * Test whether a usleep timer session has expired.
 *
 * @param[in] start_time Beginning of timer expiry check session (returned by archUsleepStart())
 * @param[in] delay_usecs Number of microsecs to check have expired after start_timE
 *
 * @retval 1=Timer expired, 0=Not expired
 *
 */
int archUsleepCheckExpired (int32_t start_time, int32_t delay_usecs)
{
    int32_t delay_timer_ticks;
	int status;

    /* Translate delay in usecs to delay in 24MHz ticks */
    delay_timer_ticks = ((TIMER_CLK / 1000000) * delay_usecs);

    /* Check if timer has expired */
	status = (((int32_t)TIMER_REG(DM36X_TIMER_TIM12) - start_time) < delay_timer_ticks) ? 0 : 1;
	return (status);
}


/**
 * \b archUsecStart
 *
 * Start a usec timer session for use with archUsecDiff() layer.
 *
 * @retval Start time for use in subsequent archUsecDiff() calls
 *
 */
int32_t archUsecStart (void)
{
    /* Check we are initialised */
    if (initialised == FALSE)
    {
        timer_init();
    }

    /* Return the current 24MHz count */
    return (TIMER_REG(DM36X_TIMER_TIM12));
}


/**
 * \b archUsecDiff
 *
 * Calculate the usecs that have expired since the passed "start_time".
 *
 * The 24MHz timer rolls over every 178 seconds. The use of a signed
 * integer means that this cannot be used to measure periods over 89 seconds.
 *
 * @param[in] start_time Beginning of time difference session (returned by archUsecStart())
 *
 * @retval Number of microseconds expired since start_time
 *
 */
int32_t archUsecDiff (int32_t start_time)
{
    /* Translate diff in 24MHz ticks to usecs */
	return (((int32_t)TIMER_REG(DM36X_TIMER_TIM12) - start_time) / (TIMER_CLK / 1000000));
}
