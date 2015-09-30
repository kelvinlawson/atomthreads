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

#include "asm_offsets.h"

.syntax unified

/**
 * Create more readable defines for usable intruction set and FPU
 */
#undef THUMB_2
#undef WITH_FPU

#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
#define THUMB_2
#endif

#if defined(__VFP_FP__) && !defined(__SOFTFP__)
#define WITH_FPU
#endif

/**
 * Extern variables needed for context switching and first thread restore
 */
.extern CTX_SW_NFO
.extern vector_table

#if defined(__NEWLIB__)
/**
 * When using newlib, reentry context needs to be updated on task switch
 */
.extern _impure_ptr
#endif

/**
 * Some bit masks and registers used
 */
.equ FPU_USED,      0x00000010
.equ SCB_ICSR,      0xE000ED04
.equ PENDSVCLR,     0x08000000

.text

.global _archFirstThreadRestore
.func   _archFirstThreadRestore
.type   _archFirstThreadRestore,%function
.thumb_func
_archFirstThreadRestore:
    /**
     * Disable interrupts. They should be disabled anyway, but just
     * to make sure...
     */
    movs    r1,         #1
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
    str     r0,         [r1, #CTX_RUN_OFF]
    str     r0,         [r1, #CTX_NEXT_OFF]

#if defined(__NEWLIB__)
    /**
     * Store the thread's reentry context address in _impure_ptr. This
     * will have been stored in ctx_switch_info.reent.
     */ 
    ldr     r2,         [r1, #CTX_REENT_OFF]
    ldr     r3,         = _impure_ptr
    str     r2,         [r3, #0]
#endif

    /* Get thread stack pointer from tcb. Conveniently the first element */
    ldr     r1,         [r0, #0]
    msr     PSP,        r1

    /**
     * Set bit #1 in CONTROL. Causes switch to PSP, so we can work directly
     * with SP now and use pop/push.
     */
    movs    r1,         #2
    mrs     r2,         CONTROL
    orrs    r2,         r2,     r1
    msr     CONTROL,    r2

    /**
     * Initialise thread's register context from its stack frame. Since this
     * function gets called only once at system start up, execution time is
     * not critical. We can get away with using only Thumb-1 instructions that
     * will work on all Cortex-M devices.
     *
     * Initial stack looks like this:
     * xPSR
     * PC
     * lr
     * r12
     * r3
     * r2
     * r1
     * r0
     * exc_ret <- ignored here
     * r11
     * r10
     * r9
     * r8
     * r7
     * r6
     * r5
     * r4 <- thread's saved_sp points here
     */

    /**
     *
     * Move SP to position of r8 and restore high registers by loading
     * them to r4-r7 before moving them to r8-r11
     */
    add     SP,         #16
    pop     {r4-r7}
    mov     r8,         r4
    mov     r9,         r5
    mov     r10,        r6
    mov     r11,        r7

    /* move SP back to top of stack and load r4-r7 */
    sub     SP,         #32
    pop     {r4-r7}

    /*load r12, lr, pc and xpsr to r0-r3 and restore r12 and lr */
    add     SP,         #36
    pop     {r0-r3}
    mov     r12,        r0
    mov     lr,         r1

    /**
     * r2 contains the PC and r3 APSR, SP is now at the bottom of the stack. We
     * can't initialise APSR now because we will have to do a movs later when
     * enabling interrupts, so r3 must not be touched. We also need an extra
     * register holding the value that will be moved to PRIMASK. To do this,
     * we build a new stack containing only the initial values of r2, r3
     * and pc. In the end this will be directly popped into the registers,
     * finishing the thread restore and branching to the thread's entry point.
     */

    /* Save PC value */
    push    {r2}

    /* Move values for r2 and r3 to lie directly below value for pc */
    sub     SP,         #20
    pop     {r1-r2}
    add     SP,         #12
    push    {r1-r2}

    /* Load values for r0 and r1 from stack */
    sub     SP,         #20
    pop     {r0-r1}

    /* Move SP to start of our new r2,r3,pc mini stack */
    add     SP,         #12

    /* Restore xPSR and enable interrupts */
    movs    r2,         #0
    msr     APSR_nzcvq, r3
    msr     PRIMASK,    r2
    
    /* Pop r2,r3,pc from stack, thereby jumping to thread entry point */
    pop     {r2,r3,pc}
    nop

.size   _archFirstThreadRestore, . - _archFirstThreadRestore
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
    movs    r0,         #1
    msr     PRIMASK,    r0
    
    /**
     * Clear PendSv pending bit. There seems to exist a hardware race condition
     * in the NVIC that can prevent automatic clearing of the PENDSVSET. See
     * http://embeddedgurus.com/state-space/2011/09/whats-the-state-of-your-cortex/
     */
     ldr    r0,         = SCB_ICSR
     ldr    r1,         = PENDSVCLR
     str    r1,         [r0, #0]

    /**
     * Check if running and next thread are really different.
     * From here on we have
     * r0 = &ctx_switch_info
     * r1 = ctx_switch_info.running_tcb
     * r2 = ctx_switch_info.next_tcb
     *
     * If r1 == r2 we can skip the context switch. This may theoretically
     * happen if the running thread gets scheduled out and in again by
     * multiple nested or tail-chained ISRs before the PendSv handler
     * gets called.
     */
    ldr     r0,         = CTX_SW_NFO
    ldr     r1,         [r0, #CTX_RUN_OFF]
    ldr     r2,         [r0, #CTX_NEXT_OFF]
    cmp     r1,         r2
    beq     no_switch

    /**
     * Copy running thread's process stack pointer to r3 and use it to push
     * the thread's register context on its stack
     */
    mrs     r3,         PSP

#if defined(THUMB_2)
    /**
     * Save old thread's context on Cortex-M[34]
     */

#if defined(WITH_FPU)
    /* Check if FPU was used by thread and store registers if necessary */
    tst     lr,         FPU_USED
    it      eq
    vstmdbeq  r3!,      {s16-s31}

    /**
     * TODO: Defer stacking FPU context by disabling FPU and using a
     * fault handler to store the FPU registers if another thread
     * tries using it
     */
#endif // WITH_FPU
    
    /* Push running thread's remaining registers on stack */
    stmdb   r3!,        {r4-r11, lr}

#else // !THUMB2

    /**
     * Save old thread's register context on Cortex-M0.
     * Push running thread's remaining registers on stack.
     * Thumb-1 can use stm only on low registers, so we
     * have to do this in two steps.
     */

    /* Reserve space for r8-r11 + exc_return before storing r4-r7 */
    subs    r3,         r3,     #36
    stmia   r3!,        {r4-r7}

    /**
     * Move r8-r11 to low registers and use store multiple with automatic
     * post-increment to push them on the stack
     */
    mov     r4,         r8
    mov     r5,         r9
    mov     r6,         r10
    mov     r7,         r11
    stmia   r3!,        {r4-r7}

    /**
     * Move lr (contains the exc_return code) to low registers and store it
     * on the stack.
     */
    mov     r4,         lr
    str     r4,         [r3, #0]

    /* Re-adjust r3 to point at top of stack */
    subs    r3,         r3, #32
#endif // !THUMB_2
    /**
     * Address of running TCB still in r1. Store thread's current stack top
     * into its sp_save_ptr, which is the struct's first element.
     */
    str     r3,         [r1, #0]
    
    /**
     * ctx_switch_info.next_tcb is going to become ctx_switch_info.running_tcb,
     * so we update the pointer.
     */
    str     r2,         [r0, #CTX_RUN_OFF]
    
#if defined(__NEWLIB__)
    /**
     * Store the thread's reentry context address in _impure_ptr. This
     * will have been stored in ctx_switch_info.reent.
     */ 
    ldr     r4,         [r0, #CTX_REENT_OFF]
    ldr     r3,         = _impure_ptr
    str     r4,         [r3, #0]
#endif

    /**
     * Fetch next thread's stack pointer from its TCB's sp_save_ptr and restore
     * the thread's register context.
     */
    ldr     r3,         [r2, #0]

#if defined(THUMB_2)

    /* Cortex-M[34], restore thread's task stack frame */
    ldmia   r3!,        {r4-r11, lr}

#if defined(WITH_FPU)
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
#endif // WITH_FPU
#else // !THUMB_2

    /**
     * Thread restore for Cortex-M0
     * Restore thread's task stack frame. Because thumb 1 only supports
     * load multiple on low register, we have to do it in two steps and
     * adjust the stack pointer manually.
     */

    /* Restore high registers */
    adds    r3,         r3, #16
    ldmia   r3!,        {r4-r7}
    mov     r8,         r4
    mov     r9,         r5
    mov     r10,        r6
    mov     r11,        r7

    /* Restore lr */
    ldr     r4,         [r3, #0]
    mov     lr,         r4
    subs    r3,         r3, #32

    /**
     * Restore r4-r7 and adjust r3 to point at the top of the exception
     * stack frame.
     */
    ldmia   r3!,        {r4-r7}
    adds    r3,         r3, #20
#endif // !THUMB_2

    /* Set process stack pointer to new thread's stack*/
    msr     PSP,        r3

no_switch:
    /* Re-enable interrupts */
    movs    r0,         #0
    msr     PRIMASK,    r0
    
    /* Return to new thread */
    bx      lr
    nop
.size   pend_sv_handler, . - pend_sv_handler
.endfunc
