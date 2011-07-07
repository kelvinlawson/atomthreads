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
#include <arm_config.h>
#include <arm_pic.h>
#include <arm_irq.h>

arm_irq_handler_t irq_hndls[NR_IRQS_PBA8];

void do_undefined_instruction(pt_regs_t *regs)
{
	/* Call the interrupt entry routine */
	atomIntEnter();

	/* Call the interrupt exit routine */
	atomIntExit(TRUE);
}

void do_software_interrupt(pt_regs_t *regs)
{
	/* Call the interrupt entry routine */
	atomIntEnter();

	/* Call the interrupt exit routine */
	atomIntExit(TRUE);
}

void do_prefetch_abort(pt_regs_t *regs)
{
	/* Call the interrupt entry routine */
	atomIntEnter();

	/* Call the interrupt exit routine */
	atomIntExit(TRUE);
}

void do_data_abort(pt_regs_t *regs)
{
	/* Call the interrupt entry routine */
	atomIntEnter();

	/* Call the interrupt exit routine */
	atomIntExit(TRUE);
}

void do_not_used(pt_regs_t *regs)
{
	/* Call the interrupt entry routine */
	atomIntEnter();

	/* Call the interrupt exit routine */
	atomIntExit(TRUE);
}

void do_irq(pt_regs_t *uregs)
{
	int rc = 0;
	int irq = arm_pic_active_irq();

	/* Call the interrupt entry routine */
	atomIntEnter();

	if (-1 < irq) {
		if (irq_hndls[irq]) {
			rc = irq_hndls[irq](irq, uregs);
			if (rc) {
				while (1);
			}
		}
		rc = arm_pic_ack_irq(irq);
		if (rc) {
			while (1);
		}
	}

	/* Call the interrupt exit routine */
	atomIntExit(TRUE);
}

void do_fiq(pt_regs_t *uregs)
{
	/* Call the interrupt entry routine */
	atomIntEnter();

	/* Call the interrupt exit routine */
	atomIntExit(TRUE);
}

void arm_irq_init(void)
{
	extern uint32_t _start_vect[];
	uint32_t *vectors = (uint32_t *)NULL;
	uint32_t *vectors_data = vectors + CPU_IRQ_NR;
	int vec;
 
	/*
	 * Loop through the vectors we're taking over, and copy the
	 * vector's insn and data word.
	 */
	for (vec = 0; vec < CPU_IRQ_NR; vec++) {
		vectors[vec] = _start_vect[vec];
		vectors_data[vec] = _start_vect[vec+CPU_IRQ_NR];
	}

	/*
	 * Check if verctors are set properly
	 */
	for (vec = 0; vec < CPU_IRQ_NR; vec++) {
		if ((vectors[vec] != _start_vect[vec]) ||
		    (vectors_data[vec] != _start_vect[vec+CPU_IRQ_NR])) {
			/* Hang */
			while(1);
		}
	}

	/*
	 * Reset irq handlers
	 */
	for (vec = 0; vec < NR_IRQS_PBA8; vec++) {
		irq_hndls[vec] = NULL;
	}

	/*
	 * Initialize Generic Interrupt Controller
	 */
	vec = arm_pic_init();
	if (vec) {
		while(1);
	}
}

void arm_irq_register(uint32_t irq, arm_irq_handler_t hndl)
{
	int rc = 0;
	if (irq < NR_IRQS_PBA8) {
		irq_hndls[irq] = hndl;
		if (irq_hndls[irq]) {
			rc = arm_pic_unmask(irq);
			if (rc) {
				while (1);
			}
		}
	}
}

void arm_irq_enable(void)
{
	__asm( "cpsie if" );
}

void arm_irq_disable(void)
{
	__asm( "cpsid if" );
}

irq_flags_t arm_irq_save(void)
{
	unsigned long retval;

	asm volatile (" mrs     %0, cpsr\n\t" " cpsid   i"	/* Syntax CPSID <iflags> {, #<p_mode>}
								 * Note: This instruction is supported 
								 * from ARM6 and above
								 */
		      :"=r" (retval)::"memory", "cc");

	return retval;
}

void arm_irq_restore(irq_flags_t flags)
{
	asm volatile (" msr     cpsr_c, %0"::"r" (flags)
		      :"memory", "cc");
}

