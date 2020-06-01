/*
 * AtomThreads project - Copyright (c) 2010, Kelvin Lawson. All rights reserved.
 * 
 * Please refer to the README file for further details and License information.
 *
 * x86 port by: ido@mm.st
 */

#include "atom.h"
#include "atomport.h"
#include "x86.h"
#include "plat.h"

static void thread_init(uint32_t entry_param)
{
	ATOM_TCB* tcb;
	entry_param = entry_param;
	
	tcb =  atomCurrentContext(); 

	x86_enable_int();
	if( tcb && tcb->entry_point)
		tcb->entry_point(tcb->entry_param);

	tcb->terminated = TRUE;
	atomSched(TRUE);
}

void archThreadContextInit (ATOM_TCB *tcb_ptr, void *stack_top, void (*entry_point)(uint32_t), uint32_t entry_param)
{
    uint32_t *stack_ptr = stack_top;

    *--stack_ptr = (uint32_t) thread_init;

    stack_ptr -= 9;
    (uint16_t*) stack_ptr--;

    tcb_ptr->sp_save_ptr = stack_ptr;
}

inline uint32_t  contextEnterCritical (void){
    uint32_t flags = x86_get_eflags();
    x86_disable_int();

    return flags;
}

inline void contextExitCritical(uint32_t flags) {
	x86_set_eflags(flags);
}
