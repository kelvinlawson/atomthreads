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

#ifndef __DM36X_IO_H__
#define __DM36X_IO_H__

#include "atomport.h"


/*
 * IO Addresses for use with DM36x
 */

#define DM36X_TIMER0_BASE		0x01C21400 /* TIMER0 */
#define DM36X_TIMER_PID12		0x00
#define DM36X_TIMER_EMUMGT		0x04
#define DM36X_TIMER_TIM12		0x10
#define DM36X_TIMER_TIM34		0x14
#define DM36X_TIMER_PRD12		0x18
#define DM36X_TIMER_PRD34		0x1C
#define DM36X_TIMER_TCR			0x20
#define DM36X_TIMER_TGCR		0x24
#define DM36X_TIMER_WDTCR		0x28
#define DM36X_TIMER_REL12		0x34
#define DM36X_TIMER_REL34		0x38
#define DM36X_TIMER_CAP12		0x3C
#define DM36X_TIMER_CAP34		0x40
#define DM36X_TIMER_INTCTL_STAT	0x44

#define DM36X_INTC_BASE			0x01C48000 /* Interrupt controller */
#define DM36X_INTC_IRQ0			0x08
#define DM36X_INTC_IRQ1			0x0C
#define DM36X_INTC_IRQENTRY		0x10
#define DM36X_INTC_EINT1		0x1C
#define DM36X_INTC_INTCTL		0x20
#define DM36X_INTC_EABASE		0x24
#define DM36X_INTC_VEC_TINT0	32

#define DM36X_UART0_BASE		0x01C20000 /* UART0 */
#define DM36X_UART1_BASE		0x01D06000 /* UART1 */



/* Function prototypes */
extern int              low_level_init (void) ;


#endif /* __DM36X_IO_H__ */
