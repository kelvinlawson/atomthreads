#ifndef __STRESS1_H__
#define __STRESS1_H__

#include "types.h"

#define TEST_STACK_BYTE_SIZE		0x200
#define IDLE_STACK_BYTE_SIZE		0x200
#define MONITOR_STACK_BYTE_SIZE		0x400

#ifndef TEST_THREADS
#define TEST_THREADS        16
#endif

extern void         atomthreads_stress_test (uint32_t thread_count) ;
extern uint32_t     test_start (void) ;

#endif /* __STRESS1_H__ */

