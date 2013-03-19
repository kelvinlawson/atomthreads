#include <stdio.h>
#include "stress1.h"
#include "atom.h"
#include "atommutex.h"
#include "atomsem.h"
#include "atomport.h"
#include "atomport-tests.h"


#define MAX_TEST_THREADS                36

static unsigned char	idle_stack[IDLE_STACK_BYTE_SIZE] ;
static unsigned char	monitor_stack[MONITOR_STACK_BYTE_SIZE] ;
static unsigned char	stress_test_stack[MAX_TEST_THREADS+1][TEST_STACK_BYTE_SIZE] ;
static unsigned int		test_counter[MAX_TEST_THREADS+1] = {0} ;


//static unsigned char	test_idle_stack[TEST_STACK_BYTE_SIZE] ;


static uint8_t			test_prio[120] = {	
								005,010,100,200,250, 200,200,200,200,200,
								150,150,150,150,150, 250,250,250,250,250,
								101,102,103,104,105, 202,204,206,208,210,
								150,150,150,150,150, 250,250,250,250,250,
								121,122,123,124,125, 061,063,065,067,061,
								150,150,150,150,150, 250,250,250,250,250,
								005,010,100,200,250, 200,200,200,200,200,
								150,150,150,150,150, 250,250,250,250,250,
								101,102,103,104,105, 202,204,206,208,210,
								150,150,150,150,150, 250,250,250,250,250,
								121,122,123,124,125, 061,063,065,067,061,
								150,150,150,150,150, 250,250,250,250,250,
								};

static uint32_t			test_interv[120] = {	
								002,001,001,001,001, 002,003,004,005,006,
								015,015,015,015,015, 025,024,023,022,021,
								905,005,005,005,805, 050,051,052,053,054,
								015,015,015,015,015, 025,024,023,022,021,
								030,030,030,030,030, 070,071,072,073,474,
								005,006,007,007,001, 001,001,003,003,005,
								001,001,001,001,001, 002,003,004,005,006,
								015,015,015,015,015, 025,024,023,022,021,
								905,005,005,005,805, 050,051,052,053,054,
								015,015,015,015,015, 025,024,023,022,021,
								030,030,030,030,030, 070,071,072,073,474,
								005,006,007,007,001, 001,001,003,003,005,
								};
									

ATOM_TCB				test_tcb[MAX_TEST_THREADS+1] ;
ATOM_TCB				monitor_tcb ;
ATOM_TCB				test2_tcb ;
ATOM_TCB				test3_tcb ;
ATOM_TCB				test_idle_tcb ;

void 
monitor_thread (uint32_t parm)
{
	CRITICAL_STORE;
	int i  ;
	unsigned int counter = 0 ;
	ATOM_TCB *tcb ;
    uint32_t print_lines_count = 0 ;

	tcb = atomCurrentContext() ;

    if (parm) {
        print_lines_count = ((parm-1)>>2) + 1;
    }

	for (;;counter++)
    {
        uint32_t time = atomTimeGet() ;
		
		CRITICAL_START();
		ATOMLOG (_STR("\r\nMonitor %d threads # %d (%08d)\r\n"), (int)parm, counter, (unsigned int)time) ;
		ATOMLOG (_STR("------------------------------\r\n")) ;
        //CRITICAL_END();

		for (i=0; i<print_lines_count; i++) {
            //CRITICAL_START();
            ATOMLOG (_STR("Thr %.2d cnt %08d\tThr %.2lu cnt %08d\tThr %.2lu cnt %08d\tThr %.2lu cnt %08d\r\n"),
                        i,test_counter[i],
                        i+print_lines_count,test_counter[i+print_lines_count],
                        i+print_lines_count*2,test_counter[i+print_lines_count*2],
                        i+print_lines_count*3,test_counter[i+print_lines_count*3]);	
			//CRITICAL_END();
            
 		}
        CRITICAL_END();

		atomTimerDelay (200) ;

	}
}


void 
stress_test_thread (uint32_t parm)
{
	CRITICAL_STORE;

	for (;;) {		

		CRITICAL_START();
		test_counter[parm]++ ;	
		CRITICAL_END();
		atomTimerDelay (test_interv[parm]) ;

	}
}


uint32_t test_start (void)
{
	uint32_t i ;
    uint32_t failures = 0 ;
	CRITICAL_STORE;

    CRITICAL_START();
	ATOMLOG (_STR("\r\natomthreads_stress_test %.3d threads\r\n"), TEST_THREADS) ;
	ATOMLOG (_STR("-----------------------------------\r\n")) ;
    CRITICAL_END();


	for (i=0; i< TEST_THREADS;i++) {
        CRITICAL_START();
        ATOMLOG (_STR("stress_test_thread %.3d creating...\r\n"), (int)i) ;
        CRITICAL_END();
		if (atomThreadCreate ((ATOM_TCB *)&test_tcb[i], test_prio[i], stress_test_thread, i, 
                    &stress_test_stack[i][0], TEST_STACK_BYTE_SIZE, TRUE) != ATOM_OK) {
                        failures++ ;
                        break ;
        }
	}
    if (atomThreadCreate ((ATOM_TCB *)&monitor_tcb, 150, monitor_thread, TEST_THREADS, 
        &monitor_stack[0], MONITOR_STACK_BYTE_SIZE, TRUE) != ATOM_OK) {
                        failures++ ;
        }

    if (failures == 0) {

        while (1)  {
            atomTimerDelay (1000) ;
        }
    }

    return failures ;
}


void 
atomthreads_stress_test (uint32_t thread_count)
{
	unsigned int i ;
    if (thread_count > MAX_TEST_THREADS) {
        thread_count = MAX_TEST_THREADS ;
    }

	ATOMLOG (_STR("\r\natomthreads_stress_test %.3d threads\r\n"), (int)thread_count) ;
	ATOMLOG (_STR("-----------------------------------\r\n")) ;


	atomOSInit(&idle_stack[0], IDLE_STACK_BYTE_SIZE, TRUE) ;
	for (i=0; i< thread_count;i++) {
		atomThreadCreate ((ATOM_TCB *)&test_tcb[i], test_prio[i], stress_test_thread, i, 
            &stress_test_stack[i][0], TEST_STACK_BYTE_SIZE, TRUE);
	}

    atomThreadCreate ((ATOM_TCB *)&monitor_tcb, 150, monitor_thread, thread_count, 
        &monitor_stack[0], MONITOR_STACK_BYTE_SIZE, TRUE);

	atomOSStart() ;
}
