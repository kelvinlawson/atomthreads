/*
 * Copyright (c) 2011, Himanshu Chauhan. All rights reserved.
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

#include <atomport-asm-macros.h>
#include <atomport.h>
#include <atom.h>
#include <atomport-private.h>

/** CPU frequency in MHz */
#define CPU_FREQ_MHZ					100

/** Number of counter counter should increase to get required ticks */
#define COUNTER_TICK_COUNT				((1000000 * SYSTEM_TICKS_PER_SEC) / CPU_FREQ_MHZ)

unsigned long long jiffies;

void mips_cpu_timer_enable(void)
{
	uint32_t sr = read_c0_status();
	sr |= ((0x1UL << 7) << 8);
	write_c0_status(sr);

	uint32_t cause = read_c0_cause();
	cause &= ~(0x1UL << 27);
	write_c0_cause(cause);
	write_c0_compare(read_c0_count() + COUNTER_TICK_COUNT);
}

void handle_mips_systick(void)
{
	/* clear EXL from status */
	uint32_t sr = read_c0_status();
	sr &= ~0x00000002;
	write_c0_status(sr);

	/* Call the interrupt entry routine */
	atomIntEnter();

	/* Call the OS system tick handler */
	atomTimerTick();

	write_c0_compare(read_c0_count() + COUNTER_TICK_COUNT);

	/* Call the interrupt exit routine */
	atomIntExit(TRUE);
}
