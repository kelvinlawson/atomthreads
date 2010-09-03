#include <stdio.h>
#include <windows.h>

#include "atom.h"
#include "atomvm.h"
#include "atomport.h"


// #define UNIT_TESTS
#ifdef UNIT_TESTS
extern uint32_t test_start (void) ;
#endif

#define TEST_THREADS				51

#define TEST_STACK_BYTE_SIZE		0x10000
#define IDLE_STACK_BYTE_SIZE		0x10000
#define MONITOR_STACK_BYTE_SIZE		0x10000



static unsigned char	idle_stack[IDLE_STACK_BYTE_SIZE] ;
static unsigned char	monitor_stack[MONITOR_STACK_BYTE_SIZE] ;
static unsigned char	test_stack[TEST_THREADS+1][TEST_STACK_BYTE_SIZE] ;
static unsigned int		test_counter[TEST_THREADS+1] = {0} ;

static unsigned int		test2_counter = 0 ;
static unsigned int		test3_counter = 0 ;
static unsigned int		test_isr_count = 0 ;


static unsigned char	test2_stack[TEST_STACK_BYTE_SIZE] ;
static unsigned char	test3_stack[TEST_STACK_BYTE_SIZE] ;
static unsigned char	test_idle_stack[TEST_STACK_BYTE_SIZE] ;


static uint8_t			test_prio[60] = {	
								001,010,100,200,250, 200,200,200,200,200,
								150,150,150,150,150, 250,250,250,250,250,
								101,102,103,104,105, 202,204,206,208,210,
								150,150,150,150,150, 250,250,250,250,250,
								121,122,123,124,125, 061,063,065,067,061,
								150,150,150,150,150, 250,250,250,250,250 
								};

static uint32_t			test_interv[60] = {	
								001,001,001,001,001, 002,003,004,005,006,
								015,015,015,015,015, 025,024,023,022,021,
								905,005,005,005,805, 050,051,052,053,054,
								015,015,015,015,015, 025,024,023,022,021,
								030,030,030,030,030, 070,071,072,073,474,
								005,006,007,007,001, 001,001,003,003,005 
								};
									

ATOM_TCB				test_tcb[TEST_THREADS+1] ;
ATOM_TCB				monitor_tcb ;
ATOM_TCB				test2_tcb ;
ATOM_TCB				test3_tcb ;
ATOM_TCB				test_idle_tcb ;

DWORD WINAPI			isr_thread_proc (LPVOID lpParameter) ;
static HANDLE			isr_thread_1 ;
static HANDLE			isr_thread_2 ;
static HANDLE			isr_thread_3 ;
static HANDLE			isr_thread_4 ;

void 
monitor_thread (uint32_t parm)
{
	CRITICAL_STORE;
	int i  ;
	int c = 0 ;
	ATOM_TCB *tcb ;

	tcb = atomCurrentContext() ;

	for (;;)
    {
		
		CRITICAL_START();

		printf("Monitor # %04d (%08d)\n", c++, atomTimeGet()) ;
		printf("-------------------------\n") ;

		for (i=0; i<TEST_THREADS/3; i++) {
			printf("Thr %.2d cnt %08d\t",i,test_counter[i]);	
			printf("Thr %.2d cnt %08d\t",i+TEST_THREADS/3,test_counter[i+TEST_THREADS/3]);	
			printf("Thr %.2d cnt %08d\n",i+TEST_THREADS*2/3,test_counter[i+TEST_THREADS*2/3]);	
		}
		printf("\nIdle Threadd 1 Counter =   %d\nIdle Theadrd 2 Counter =   %d\nInterrupt Counter      =   %d",test2_counter,test3_counter,test_isr_count);
		printf ("\n\n") ;
		CRITICAL_END();

		atomTimerDelay (300) ;

	}
}


void 
test_thread (uint32_t parm)
{
	CRITICAL_STORE;

	for (;;) {		

		atomTimerDelay (test_interv[parm]) ;
		CRITICAL_START();
		test_counter[parm]++ ;	
		CRITICAL_END();

	}
}


void 
test2_thread (uint32_t parm)
{
	CRITICAL_STORE;

	for (;;) {	

		CRITICAL_START();
		test2_counter++ ;
		CRITICAL_END();

	}
}


void test3_thread(uint32_t parm)
{
	CRITICAL_STORE;

	for (;;) {		

		CRITICAL_START();
		test3_counter++ ;
		CRITICAL_END();

	}
}

#ifdef UNIT_TESTS
void unit_test_thread(uint32_t parm)
{
	unsigned int failures ;

	failures = test_start () ;
	printf ("test_start %d failures\n", failures) ;

	while(1) {

		 atomTimerDelay (100);
	}

}
#endif

 void 
__atomvmReset ()
{
	unsigned int i ;

	atomOSInit(&idle_stack[IDLE_STACK_BYTE_SIZE - sizeof(unsigned int)], IDLE_STACK_BYTE_SIZE - sizeof(unsigned int)) ;
#ifndef UNIT_TESTS

	for (i=0; i< TEST_THREADS;i++) {
		atomThreadCreate ((ATOM_TCB *)&test_tcb[i], test_prio[i], test_thread, i, &test_stack[i][TEST_STACK_BYTE_SIZE  - sizeof(unsigned int)], TEST_STACK_BYTE_SIZE  - sizeof(unsigned int));
	}

	atomThreadCreate ((ATOM_TCB *)&monitor_tcb, 50, monitor_thread, 0, &monitor_stack[(MONITOR_STACK_BYTE_SIZE) - sizeof(unsigned int)], MONITOR_STACK_BYTE_SIZE  - sizeof(unsigned int));
	
	atomThreadCreate ((ATOM_TCB *)&test2_tcb, 253, test2_thread, 0, &test2_stack[(TEST_STACK_BYTE_SIZE) - sizeof(unsigned int)], (TEST_STACK_BYTE_SIZE) - sizeof(unsigned int));
	atomThreadCreate ((ATOM_TCB *)&test3_tcb, 253, test3_thread, 0, &test3_stack[(TEST_STACK_BYTE_SIZE) - sizeof(unsigned int)], (TEST_STACK_BYTE_SIZE) - sizeof(unsigned int));
#else

	atomThreadCreate ((ATOM_TCB *)&test2_tcb, 16, unit_test_thread, 0, &test2_stack[(TEST_STACK_BYTE_SIZE) - sizeof(unsigned int)], (TEST_STACK_BYTE_SIZE) - sizeof(unsigned int));

#endif

	atomOSStart() ;
}

 void 
__atomvmClose () 
 {

 }



void
test_isr ()
{
	static int i = 0 ;
	test_isr_count++ ;
	if (i++==25) {
		//Sleep(3) ;
		i = 0;
	}

}


DWORD WINAPI 
isr_thread_proc (LPVOID lpParameter)
 {
	 int i = 0 ;
     int x ;
     int y = rand() % 100 ;
	 while (1) {
		atomvmCtrlIntRequest (the_atomvm, (uintptr_t)test_isr) ;
		if (i++==y) {
            x = rand()  % 50 ;
			Sleep (x) ;
            y = rand() % 100 ;
			i = 0 ;
		}
	 }

	return 0 ;
 }


void 
main ()
{
	atomvmRun () ;

#ifndef UNIT_TESTS
	isr_thread_1 = CreateThread (NULL, 0, isr_thread_proc, 0, CREATE_SUSPENDED, NULL) ;
	isr_thread_2 = CreateThread (NULL, 0, isr_thread_proc, 0, CREATE_SUSPENDED, NULL) ;
	isr_thread_3 = CreateThread (NULL, 0, isr_thread_proc, 0, CREATE_SUSPENDED, NULL) ;
	isr_thread_4 = CreateThread (NULL, 0, isr_thread_proc, 0, CREATE_SUSPENDED, NULL) ;

	ResumeThread (isr_thread_1) ;
	ResumeThread (isr_thread_2) ;
	ResumeThread (isr_thread_3) ;
	ResumeThread (isr_thread_4) ;
#endif

	while (1)  {
		Sleep(1) ;
		atomvmCtrlIntRequest (the_atomvm, (uintptr_t)archTimerTickIrqHandler) ;
	}

}
