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

#include <string.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/sync.h>

#include "atomport.h"
#include "atomport-private.h"
#include "asm_offsets.h"

static void thread_shell(void);

struct task_switch_info ctx_switch_info asm("CTX_SW_NFO") =
{
    .running_tcb = NULL,
    .next_tcb    = NULL,
};

extern void _archFirstThreadRestore(ATOM_TCB *);
void archFirstThreadRestore(ATOM_TCB *new_tcb_ptr)
{
#if defined(__NEWLIB__)
    ctx_switch_info.reent = &(new_tcb_ptr->port_priv.reent);
    __dmb();
#endif

    _archFirstThreadRestore(new_tcb_ptr);
}

/**
 * We do not perform the context switch directly. Instead we mark the new tcb
 * as should-be-running in ctx_switch_info and trigger a PendSv-interrupt.
 * The pend_sv_handler will be called when all other pending exceptions have
 * returned and perform the actual context switch.
 * This way we do not have to worry if we are being called from task or
 * interrupt context, which would mean messing with either main or thread
 * stack format.
 *
 * One difference to the other architectures is that execution flow will
 * actually continue in the old thread context until interrupts are enabled
 * again. From a thread context this should make no difference, as the context
 * switch will be performed as soon as the execution flow would return to the
 * calling thread. Unless, of course, the thread called atomSched() with
 * disabled interrupts, which it should not do anyways...
 */
void __attribute__((noinline))
archContextSwitch(ATOM_TCB *old_tcb_ptr __maybe_unused, ATOM_TCB *new_tcb_ptr)
{
    if(likely(ctx_switch_info.running_tcb != NULL)){
        ctx_switch_info.next_tcb = new_tcb_ptr;
#if defined(__NEWLIB__)
        ctx_switch_info.reent = &(new_tcb_ptr->port_priv.reent);
#endif
        __dmb();

        SCB_ICSR = SCB_ICSR_PENDSVSET;
    }
}

void sys_tick_handler(void)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the OS system tick handler */
    atomTimerTick();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}

/**
 * Put chip into infinite loop if NMI or hard fault occurs
 */
void nmi_handler(void)
{
    while(1)
        ;
}

void hard_fault_handler(void)
{
    while(1)
        ;
}


/**
 * This function is called when a new thread is scheduled in for the first
 * time. It will simply call the threads entry point function.
 */
static void thread_shell(void)
{
    ATOM_TCB *task_ptr;

    /**
     * We "return" to here after being scheduled in by the pend_sv_handler.
     * We get a pointer to our TCB from atomCurrentContext()
     */
    task_ptr = atomCurrentContext();

    /**
     * Our thread entry point and parameter are stored in the TCB.
     * Call it if it is valid
     */
    if(task_ptr && task_ptr->entry_point){
        task_ptr->entry_point(task_ptr->entry_param);
    }

    /**
     * Thread returned or entry point was not valid.
     * Should never happen... Maybe we should switch MCU into debug mode here
     */
    while(1)
        ;
}

/**
 * Initialise a threads stack so it can be scheduled in by
 * archFirstThreadRestore or the pend_sv_handler.
 */
void archThreadContextInit(ATOM_TCB *tcb_ptr, void *stack_top,
                           void (*entry_point)(uint32_t), uint32_t entry_param)
{
    struct isr_stack *isr_ctx;
    struct task_stack *tsk_ctx;

    /**
     * Do compile time verification for offsets used in _archFirstThreadRestore
     * and pend_sv_handler. If compilation aborts here, you will have to adjust
     * the offsets for struct task_switch_info's members in asm-offsets.h
     */
    assert_static(offsetof(struct task_switch_info, running_tcb) == CTX_RUN_OFF);
    assert_static(offsetof(struct task_switch_info, next_tcb) == CTX_NEXT_OFF);
#if defined(__NEWLIB__)
    assert_static(offsetof(struct task_switch_info, reent) == CTX_REENT_OFF);
#endif

    /**
     * Enforce initial stack alignment
     */
    stack_top = STACK_ALIGN(stack_top, STACK_ALIGN_SIZE);

    /**
     * New threads will be scheduled from an exception handler, so we have to
     * set up an exception stack frame as well as task stack frame
     */
    isr_ctx = stack_top - sizeof(*isr_ctx);
    tsk_ctx = stack_top - sizeof(*isr_ctx) - sizeof(*tsk_ctx);

#if 0
    printf("[%s] tcb_ptr: %p stack_top: %p isr_ctx: %p tsk_ctx: %p entry_point: %p, entry_param: 0x%x\n",
            __func__, tcb_ptr, stack_top, isr_ctx, tsk_ctx, entry_point, entry_param);
    printf("[%s] isr_ctx->r0: %p isr_ctx->psr: %p tsk_ctx->r4: %p tsk_ctx->lr: %p\n",
            __func__, &isr_ctx->r0, &isr_ctx->psr, &tsk_ctx->r4, &tsk_ctx->lr);
#endif
    /**
     * We use the exception return mechanism to jump to our thread_shell()
     * function and initialise the PSR to the default value (thumb state
     * flag set and nothing else)
     */
    isr_ctx->psr = 0x01000000;
    isr_ctx->pc  = (uint32_t) thread_shell;

    /* initialise unused registers to silly value */
    isr_ctx->lr  = 0xEEEEEEEE;
    isr_ctx->r12 = 0xCCCCCCCC;
    isr_ctx->r3  = 0x33333333;
    isr_ctx->r2  = 0x22222222;
    isr_ctx->r1  = 0x11111111;
    isr_ctx->r0  = 0x00000000;

    /**
     * We use this special EXC_RETURN code to switch from main stack to our
     * thread stack on exception return
     */
    tsk_ctx->exc_ret = 0xFFFFFFFD;

    /* initialise unused registers to silly value */
    tsk_ctx->r11 = 0xBBBBBBBB;
    tsk_ctx->r10 = 0xAAAAAAAA;
    tsk_ctx->r9  = 0x99999999;
    tsk_ctx->r8  = 0x88888888;
    tsk_ctx->r7  = 0x77777777;
    tsk_ctx->r6  = 0x66666666;
    tsk_ctx->r5  = 0x55555555;
    tsk_ctx->r4  = 0x44444444;

    /**
     * Stack frames have been initialised, save it to the TCB. Also set
     * the thread's real entry point and param, so the thread shell knows
     * what function to call.
     */
    tcb_ptr->sp_save_ptr = tsk_ctx;
    tcb_ptr->entry_point = entry_point;
    tcb_ptr->entry_param = entry_param;

#if defined(__NEWLIB__)
    /**
     * Initialise thread's reentry context for newlib
     */ 
    _REENT_INIT_PTR(&(tcb_ptr->port_priv.reent));
#endif
}

