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
		/* Initialise TIMER1 registers for microsecond accuracy timer */

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

    /* Finished */
    return (status);
}


/**
 * \b archNanosleep
 *
 * Simple spin loop of at least the specified nanoseconds.
 *
 * @param[in] nanosecs Number of nanoseconds to sleep
 *
 * @return None
 *
 */
void archNanosleep (int32_t nanosecs)
{
    /* Check we are initialised */
    if (initialised == FALSE)
    {
        timer_init();
    }

}
