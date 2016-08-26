/*
 * AtomThreads project - Copyright (c) 2010, Kelvin Lawson. All rights reserved.
 * 
 * Please refer to the README file for further details and License information.
 *
 * x86 port by: ido@mm.st
 */

#ifndef __ATOM_PORT_TESTS_H
#define __ATOM_PORT_TESTS_H

/* Include Atomthreads kernel API */
#include "atom.h"
#include "plat.h"
#include "print.h"

/* Logger macro for viewing test results */

#define ATOMLOG kprintf
/*
 * String location macro: for platforms which need to place strings in
 * alternative locations, e.g. on avr-gcc strings can be placed in
 * program space, saving SRAM. On most platforms this can expand to
 * empty.
 */
#define _STR(x)     x

/* Default thread stack size (in bytes). Allow plenty for printf(). */
#define TEST_THREAD_STACK_SIZE      1024*16 // 16K 

/* Uncomment to enable logging of stack usage to UART */
/* #define TESTS_LOG_STACK_USAGE */


#endif /* __ATOM_PORT_TESTS_H */

