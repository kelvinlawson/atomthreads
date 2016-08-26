/*
 * AtomThreads project - Copyright (c) 2010, Kelvin Lawson. All rights reserved.
 * 
 * Please refer to the README file for further details and License information.
 *
 * x86 port by: ido@mm.st
 */

#include "atom.h"
#include "atomport.h"
#include "atomtimer.h"
#include "plat.h"
#include "atomtests.h"

#define TEST_OK 0
#define TEST_FAILURE 1

#define APP_PRIORITY 16
#define STACK_SIZE 1024*4 

#define THROUGHPUT 5
#define TICKS_FACTOR 10

static ATOM_TCB tcb[THROUGHPUT];
static uint8_t stack[THROUGHPUT][STACK_SIZE];

static uint8_t counter = 0;

static uint32_t getThreadId()
{
	ATOM_TCB *tcb = atomCurrentContext();
	return tcb->entry_param;
}	

static void appTick()
{
	uint32_t t = atomTimeGet();
	
	while(1)
	{
		if(atomTimeGet() > t)
		{
			kprintf("%d] time: %d\n", getThreadId(), atomTimeGet());	
			t = atomTimeGet();
		}
		if(atomTimeGet()>getThreadId()*TICKS_FACTOR) break;
	}

	kprintf("%d] - is done!\n", getThreadId());

	--counter;	
}

static void thread_entry(uint32_t data)
{
	appTick();
}

uint32_t test_start()
{
	for(uint8_t i = 0; i < THROUGHPUT ; i++,counter++){
		if(ATOM_OK != atomThreadCreate(&tcb[i], 
				   APP_PRIORITY, 
				   thread_entry, 
				   i+1,
				   &stack[i],
				   STACK_SIZE,
				   TRUE)) return TEST_FAILURE;
	}

	while(1)
		if(tcb->next_tcb == tcb->prev_tcb && 0 == counter) break;

	return TEST_OK;
}
