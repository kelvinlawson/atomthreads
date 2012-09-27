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

#include <atomport-asm-macros.h>

.section .text

/**
 * Function that performs the contextSwitch. Whether its a voluntary release
 * of CPU by thread or a pre-emption, under both conditions this function is
 * called. The signature is as follows:
 *
 * archContextSwitch(ATOM_TCB *old_tcb, ATOM_TCB *new_tcb)
 */
.globl archContextSwitch
archContextSwitch:
    move v0, a0 /* return old tcb when we return from here */
    lw k0, 0(a0) /* assume that sp_save_ptr is always at base of ATOM_TCB */
    SAVE_REG(s0, k0)
    SAVE_REG(s1, k0)
    SAVE_REG(s2, k0)
    SAVE_REG(s3, k0)
    SAVE_REG(s4, k0)
    SAVE_REG(s5, k0)
    SAVE_REG(s6, k0)
    SAVE_REG(s7, k0)
    SAVE_REG(s8, k0)
    SAVE_REG(sp, k0)
    SAVE_REG(gp, k0)
    SAVE_REG(ra, k0)

    lw k1, 0(a1)
    LOAD_REG(s0, k1)
    LOAD_REG(s1, k1)
    LOAD_REG(s2, k1)
    LOAD_REG(s3, k1)
    LOAD_REG(s4, k1)
    LOAD_REG(s5, k1)
    LOAD_REG(s6, k1)
    LOAD_REG(s7, k1)
    LOAD_REG(s8, k1)
    LOAD_REG(sp, k1)
    LOAD_REG(gp, k1)
    LOAD_REG(ra, k1)

    lw k0, (cp0_epc_IDX * 4)(k1)
    bnez k0, 1f
    nop
    li k0, 0x00000001
    sw k0, (cp0_epc_IDX * 4)(k1)
    LOAD_REG(a0, k1)
    LOAD_REG(a1, k1)
    LOAD_REG(a2, k1)
    LOAD_REG(a3, k1)
    enable_global_interrupts
1:
    jr ra
    nop

/**
 * archFirstThreadRestore(ATOM_TCB *new_tcb)
 *
 * This function is responsible for restoring and starting the first
 * thread the OS runs. It expects to find the thread context exactly
 * as it would be if a context save had previously taken place on it.
 * The only real difference between this and the archContextSwitch()
 * routine is that there is no previous thread for which context must
 * be saved.
 *
 * The final action this function must do is to restore interrupts.
 */
.globl archFirstThreadRestore
archFirstThreadRestore:
    move k0, a0 /* save the copy of tcb pointer in k0 */
    lw   k1, 0(k0) /* Assume that sp_save_ptr is always at base of ATOM_TCB */
    lw   a0, (a0_IDX * 4)(k1)
    lw   sp, (sp_IDX * 4)(k1)
    lw   s8, (s8_IDX * 4)(k1)
    lw   k0, (ra_IDX * 4)(k1)
    mtc0 k0, CP0_EPC
    nop
    nop
    nop
    ehb
    li k0, 0x00000001
    sw k0, (cp0_epc_IDX * 4)(k1)
    nop
    ehb
    enable_global_interrupts
    ehb
    nop
    nop
    eret
