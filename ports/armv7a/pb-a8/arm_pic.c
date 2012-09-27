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

#include <arm_config.h>
#include <arm_io.h>
#include <arm_pic.h>

#define max(a,b)	((a) < (b) ? (b) : (a))

struct gic_chip_data {
	uint32_t irq_offset;
	virtual_addr_t dist_base;
	virtual_addr_t cpu_base;
};

static struct gic_chip_data gic_data[ARM_GIC_MAX_NR];

static inline void arm_gic_write(uint32_t val, virtual_addr_t addr)
{
	arm_writel(val, (void *)(addr));
}

static inline uint32_t arm_gic_read(virtual_addr_t addr)
{
	return arm_readl((void *)(addr));
}

int arm_gic_active_irq(uint32_t gic_nr)
{
	int ret = -1;

	if (ARM_GIC_MAX_NR <= gic_nr) {
		return -1;
	}

	ret = arm_gic_read(gic_data[gic_nr].cpu_base +
				GIC_CPU_INTACK) & 0x3FF;
	ret += gic_data[gic_nr].irq_offset;

	return ret;
}

int arm_gic_ack_irq(uint32_t gic_nr, uint32_t irq)
{
	uint32_t mask = 1 << (irq % 32);
	uint32_t gic_irq;

	if (ARM_GIC_MAX_NR <= gic_nr) {
		return -1;
	}

	if (irq < gic_data[gic_nr].irq_offset) {
		return -1;
	}

	gic_irq = irq - gic_data[gic_nr].irq_offset;

	arm_gic_write(mask, gic_data[gic_nr].dist_base + 
				GIC_DIST_ENABLE_CLEAR + (gic_irq / 32) * 4);
	arm_gic_write(gic_irq, gic_data[gic_nr].cpu_base + GIC_CPU_EOI);
	arm_gic_write(mask, gic_data[gic_nr].dist_base + 
				GIC_DIST_ENABLE_SET + (gic_irq / 32) * 4);

	return 0;
}

int arm_gic_mask(uint32_t gic_nr, uint32_t irq)
{
	uint32_t mask = 1 << (irq % 32);
	uint32_t gic_irq;

	if (ARM_GIC_MAX_NR <= gic_nr) {
		return -1;
	}

	if (irq < gic_data[gic_nr].irq_offset) {
		return -1;
	}

	gic_irq = irq - gic_data[gic_nr].irq_offset;

	arm_gic_write(mask, gic_data[gic_nr].dist_base +
			   GIC_DIST_ENABLE_CLEAR + (gic_irq / 32) * 4);

	return 0;
}

int arm_gic_unmask(uint32_t gic_nr, uint32_t irq)
{
	uint32_t mask = 1 << (irq % 32);
	uint32_t gic_irq;

	if (ARM_GIC_MAX_NR <= gic_nr) {
		return -1;
	}

	if (irq < gic_data[gic_nr].irq_offset) {
		return -1;
	}

	gic_irq = irq - gic_data[gic_nr].irq_offset;

	arm_gic_write(mask, gic_data[gic_nr].dist_base +
			   GIC_DIST_ENABLE_SET + (gic_irq / 32) * 4);

	return 0;
}

int arm_gic_dist_init(uint32_t gic_nr, virtual_addr_t base, uint32_t irq_start)
{
	unsigned int max_irq, i;
	uint32_t cpumask = 1 << 0;	/*smp_processor_id(); */

	if (ARM_GIC_MAX_NR <= gic_nr) {
		return -1;
	}

	cpumask |= cpumask << 8;
	cpumask |= cpumask << 16;

	gic_data[gic_nr].dist_base = base;
	gic_data[gic_nr].irq_offset = (irq_start - 1) & ~31;

	arm_gic_write(0, base + GIC_DIST_CTRL);

	/*
	 * Find out how many interrupts are supported.
	 */
	max_irq = arm_gic_read(base + GIC_DIST_CTR) & 0x1f;
	max_irq = (max_irq + 1) * 32;

	/*
	 * The GIC only supports up to 1020 interrupt sources.
	 * Limit this to either the architected maximum, or the
	 * platform maximum.
	 */
	if (max_irq > max(1020, ARM_GIC_NR_IRQS))
		max_irq = max(1020, ARM_GIC_NR_IRQS);

	/*
	 * Set all global interrupts to be level triggered, active low.
	 */
	for (i = 32; i < max_irq; i += 16)
		arm_gic_write(0, base + GIC_DIST_CONFIG + i * 4 / 16);

	/*
	 * Set all global interrupts to this CPU only.
	 */
	for (i = 32; i < max_irq; i += 4)
		arm_gic_write(cpumask, base + GIC_DIST_TARGET + i * 4 / 4);

	/*
	 * Set priority on all interrupts.
	 */
	for (i = 0; i < max_irq; i += 4)
		arm_gic_write(0xa0a0a0a0, base + GIC_DIST_PRI + i * 4 / 4);

	/*
	 * Disable all interrupts.
	 */
	for (i = 0; i < max_irq; i += 32)
		arm_gic_write(0xffffffff,
				   base + GIC_DIST_ENABLE_CLEAR + i * 4 / 32);

	arm_gic_write(1, base + GIC_DIST_CTRL);

	return 0;
}

int arm_gic_cpu_init(uint32_t gic_nr, virtual_addr_t base)
{
	if (ARM_GIC_MAX_NR <= gic_nr) {
		return -1;
	}

	gic_data[gic_nr].cpu_base = base;

	arm_gic_write(0xf0, base + GIC_CPU_PRIMASK);
	arm_gic_write(1, base + GIC_CPU_CTRL);

	return 0;
}

int arm_pic_active_irq(void)
{
	return arm_gic_active_irq(0);
}

int arm_pic_ack_irq(uint32_t irq)
{
	return arm_gic_ack_irq(0, irq);
}

int arm_pic_mask(uint32_t irq)
{
	return arm_gic_mask(0, irq);
}

int arm_pic_unmask(uint32_t irq)
{
	return arm_gic_unmask(0, irq);
}

int arm_pic_init(void)
{
	int rc = 0;

	rc = arm_gic_dist_init(0, REALVIEW_PBA8_GIC_DIST_BASE, 
							IRQ_PBA8_GIC_START);
	if (rc) {
		return rc;
	}
	rc = arm_gic_cpu_init(0, REALVIEW_PBA8_GIC_CPU_BASE);
	if (rc) {
		while(1);
	}

	return rc;
}


