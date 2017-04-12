/*
 * AtomThreads project - Copyright (c) 2010, Kelvin Lawson. All rights reserved.
 * 
 * Please refer to the README file for further details and License information.
 *
 * x86 port by: ido@mm.st
 */

#ifndef __ATOM_PORT_H
#define __ATOM_PORT_H

#include <stdint.h>
#include <stddef.h>


/* Required number of system ticks per second (normally 100 for 10ms tick) */
#define SYSTEM_TICKS_PER_SEC            100

/* Size of each stack entry / stack alignment size (32 bits on this platform) */
#define STACK_ALIGN_SIZE                sizeof(uint32_t)

/**
 * Architecture-specific types.
 * Most of these are available from stdint.h on this platform, which is
 * included above.
 */
#define POINTER void *

extern uint32_t  contextEnterCritical (void) ;
extern void      contextExitCritical (uint32_t posture) ;

#define CRITICAL_STORE          uint32_t __atom_critical
#define CRITICAL_START()        __atom_critical = contextEnterCritical()
#define CRITICAL_END()          contextExitCritical(__atom_critical)

/* Uncomment to enable stack-checking */
/* #define ATOM_STACK_CHECKING */

#endif /* __ATOM_PORT_H */
