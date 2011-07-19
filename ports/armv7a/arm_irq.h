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

#ifndef __ARM_IRQ_H
#define __ARM_IRQ_H

#include "atomport.h"
#include "atomport-private.h"
#include "arm_defines.h"

typedef int (*arm_irq_handler_t) (uint32_t irq_no, pt_regs_t * regs);

#define CPU_IRQ_NR					8

/** IRQ Numbers */
#define ARM_RESET_IRQ					0
#define ARM_UNDEF_INST_IRQ				1
#define ARM_SOFT_IRQ					2
#define ARM_PREFETCH_ABORT_IRQ				3
#define ARM_DATA_ABORT_IRQ				4
#define ARM_NOT_USED_IRQ				5
#define ARM_EXTERNAL_IRQ				6
#define ARM_EXTERNAL_FIQ				7

void arm_irq_init(void);
void arm_irq_register(uint32_t irq_no, arm_irq_handler_t hndl);
void arm_irq_enable(void);
void arm_irq_disable(void);
irq_flags_t arm_irq_save(void);
void arm_irq_restore(irq_flags_t flags);

#endif /* __ARM_IRQ_H */
