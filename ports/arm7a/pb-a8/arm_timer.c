/*
 * Copyright (c) 2011, Anup Patel. All rights reserved.
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

#include <atom.h>
#include <atomport.h>
#include <arm_io.h>
#include <arm_irq.h>
#include <arm_config.h>
#include <arm_plat.h>
#include <arm_timer.h>

unsigned long long jiffies;

void arm_timer_enable(void)
{
	uint32_t ctrl;

	ctrl = arm_readl((void *)(REALVIEW_PBA8_TIMER0_1_BASE + TIMER_CTRL));
	ctrl |= TIMER_CTRL_ENABLE;
	arm_writel(ctrl, (void *)(REALVIEW_PBA8_TIMER0_1_BASE + TIMER_CTRL));
}

void arm_timer_disable(void)
{
	uint32_t ctrl;

	ctrl = arm_readl((void *)(REALVIEW_PBA8_TIMER0_1_BASE + TIMER_CTRL));
	ctrl &= ~TIMER_CTRL_ENABLE;
	arm_writel(ctrl, (void *)(REALVIEW_PBA8_TIMER0_1_BASE + TIMER_CTRL));
}

void arm_timer_clearirq(void)
{
	arm_writel(1, (void *)(REALVIEW_PBA8_TIMER0_1_BASE + TIMER_INTCLR));
}

int arm_timer_irqhndl(uint32_t irq_no, pt_regs_t * regs)
{
	/* Call the OS system tick handler */
	atomTimerTick();

	arm_timer_clearirq();

	return 0;
}

int arm_timer_init(uint32_t ticks_per_sec)
{
	uint32_t val;

	/* 
	 * set clock frequency: 
	 *      REALVIEW_TIMCLK is 1MHz
	 */
	val = arm_readl((void *)REALVIEW_SCTL_BASE) | (REALVIEW_TIMCLK << 0x1);
	arm_writel(val, (void *)REALVIEW_SCTL_BASE);

	/* Register interrupt handler */
	arm_irq_register(IRQ_PBA8_TIMER0_1, &arm_timer_irqhndl);

	val = arm_readl((void *)(REALVIEW_PBA8_TIMER0_1_BASE + TIMER_CTRL));
	val &= ~TIMER_CTRL_ENABLE;
	val |= (TIMER_CTRL_32BIT | TIMER_CTRL_PERIODIC | TIMER_CTRL_IE);
	arm_writel(val, (void *)(REALVIEW_PBA8_TIMER0_1_BASE + TIMER_CTRL));
	arm_writel((1000000 / ticks_per_sec), 
		   (void *)(REALVIEW_PBA8_TIMER0_1_BASE + TIMER_LOAD));
	arm_writel((1000000 / ticks_per_sec), 
		   (void *)(REALVIEW_PBA8_TIMER0_1_BASE + TIMER_VALUE));

	return 0;
}
