/*
 * Copyright (c) 2015, Tido Klaassen. All rights reserved.
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

.syntax unified

.extern CTX_SW_NFO
.extern _stack
.extern vector_table

.equ FPU_USED,   0x00000010

.text

.global archFirstThreadRestore
.func   archFirstThreadRestore
.type   archFirstThreadRestore,%function
.thumb_func
archFirstThreadRestore:
    /**
     * Disable interrupts. They should be disabled anyways, but just
     * to make sure...
     */
    mov     r1,         #1 
    msr     PRIMASK,    r1

    /**
     * Reset main stack pointer to initial value, which is the first entry
     * in the vector table.
     */
    ldr     r1,         = vector_table
    ldr     r1,         [r1, #0]
    msr     MSP,        r1
    
    /* Update ctx_switch_info, set this thread as both running and next */
    ldr     r1,         = CTX_SW_NFO	
    str     r0,         [r1, #0]
    str	    r0,         [r1, #4]

    /* Get thread stack pointer from tcb. Conveniently the first element */
    ldr     r2,         [r0, #0]

    /* Discard dummy thread stack frame */
    add     r2,         r2, #36
    
    /**
     * New threads can't have used the FPU, so no need to check for
     * FPU stack frame here
     */

    /**
     * Load exception stack frame to registers r3-r10. xPSR ends up in r10,
     * pc in r9
     */
    ldmia   r2!,        {r3-r10}

    /* initialise xPSR and store store adjusted stack pointer in PSP */
    msr     APSR_nzcvq, r10
    msr     PSP,        r2

    /* set bit #1 in CONTROL. Causes switch to PSP */
    mrs     r1,         CONTROL
    orr     r1, r1,     #2
    msr     CONTROL,    r1
    
    /* enable interrupts */
    mov     r1,         #0
    msr PRIMASK,        r1
    
    /* branch to thread entry point */
    bx      r9
    nop
.size   archFirstThreadRestore, . - archFirstThreadRestore
.endfunc

.global pend_sv_handler
.func   pend_sv_handler
.type   pend_sv_handler,%function
.thumb_func
pend_sv_handler:
    /**
     * Disable interrupts. No need to check if they were enabled because,
     * well, we're an interrupt handler. Duh...
     */
    mov     r0,         #1
    msr     PRIMASK,    r0
    
    /* Get running thread's stack pointer*/
    mrs     r3,         PSP

#if (defined(__VFP_FP__) && !defined(__SOFTFP__))
    /* Check if FPU was used by thread and store registers if necessary */
    tst     lr,         FPU_USED
    it      eq
    vstmdbeq  r3!,      {s16-s31}

    /**
     * TODO: Defer stacking FPU context by disabling FPU and using a
     * fault handler to store the FPU registers if another thread
     * tries using it
     */
#endif
    
    /* Push running thread's remaining registers on stack */
    stmdb   r3!,        {r4-r11, lr}

    /* Fetch running thread's TCB and store its current stack pointer */
    ldr     r2,         = CTX_SW_NFO
    ldr     r1,         [r2, #0]
    str     r3,         [r1, #0]
    
    /**
     * Fetch pointer to next thread's tcb from ctx_switch_info.next_tcb and
     * use it to update ctx_switch_info.running_tcb.
     */
    ldr     r1,         [r2, #4]
    str     r1,         [r2, #0]
    
    /**
     * Fetch next thread's stack pointer from its TCB and restore
     * the thread's register context.
     */
    ldr     r3,         [r1, #0]
    ldmia   r3!,        {r4-r11, lr}

#if (defined(__VFP_FP__) && !defined(__SOFTFP__))
    /**
     * Check if FPU was used by new thread and restore registers if necessary.
     */
    tst     lr,         FPU_USED
    it      eq
    vldmiaeq  r3!,      {s16-s31}

    /**
     * TODO: only restore FPU registers if FPU was used by another thread
     * between this thread being scheduled out and now.
     */
#endif

    /* Restore process stack pointer */
    msr     PSP,        r3

    /* Re-enable interrupts */
    mov     r0,         #0
    msr     PRIMASK,    r0
    
    /* Return to new thread */
    bx      lr
    nop
.size   pend_sv_handler, . - pend_sv_handler
.endfunc
