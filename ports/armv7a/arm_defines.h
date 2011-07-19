/*
 * Copyright (c) 2010, Atomthreads Project. All rights reserved.
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

#ifndef __ARM_DEFINES_H_
#define __ARM_DEFINES_H_

#define CPSR_VALIDBITS_MASK				0xFF0FFFFF
#define CPSR_USERBITS_MASK				0xFFFFFC00
#define CPSR_USERBITS_SHIFT				10
#define CPSR_PRIVBITS_MASK				0x000003FF
#define CPSR_PRIVBITS_SHIFT				0
#define CPSR_MODE_MASK					0x0000001f
#define CPSR_MODE_USER					0x00000010
#define CPSR_MODE_FIQ					0x00000011
#define CPSR_MODE_IRQ					0x00000012
#define CPSR_MODE_SUPERVISOR				0x00000013
#define CPSR_MODE_MONITOR				0x00000016
#define CPSR_MODE_ABORT					0x00000017
#define CPSR_MODE_UNDEFINED				0x0000001b
#define CPSR_MODE_SYSTEM				0x0000001f
#define CPSR_THUMB_ENABLED				(1 << 5)
#define CPSR_FIQ_DISABLED				(1 << 6)
#define CPSR_IRQ_DISABLED				(1 << 7)
#define CPSR_ASYNC_ABORT_DISABLED			(1 << 8)
#define CPSR_BE_ENABLED					(1 << 9)
#define CPSR_IT2_MASK					0x0000FC00
#define CPSR_IT2_SHIFT					10
#define CPSR_GE_MASK					0x000F0000
#define CPSR_GE_SHIFT					16
#define CPSR_JAZZLE_ENABLED				(1 << 24)
#define CPSR_IT1_MASK					0x06000000
#define CPSR_IT1_SHIFT					25
#define CPSR_COND_OVERFLOW_MASK				(1 << 28)
#define CPSR_COND_OVERFLOW_SHIFT			28
#define CPSR_COND_CARRY_MASK				(1 << 29)
#define CPSR_COND_CARRY_SHIFT				29
#define CPSR_COND_ZERO_MASK				(1 << 30)
#define CPSR_COND_ZERO_SHIFT				30
#define CPSR_COND_NEGATIVE_MASK				(1 << 31)
#define CPSR_COND_NEGATIVE_SHIFT			31

#endif /* __ARM_DEFINES_H_ */
