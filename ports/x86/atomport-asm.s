/*
 * AtomThreads project - Copyright (c) 2010, Kelvin Lawson. All rights reserved.
 * 
 * Please refer to the README file for further details and License information.
 *
 * x86 port by: ido@mm.st
 */

.section .text

.global archContextSwitch
.type archContextSwitch, @function
archContextSwitch:
    pushl %ebp
    movl %esp,%ebp

    pushf
    pusha

    movl 8(%ebp),%eax
    movl 12(%ebp),%ebx

    movl %esp,(%eax)
    movl (%ebx),%esp

    popa
    popf 

    popl %ebp

    sti
    ret

.global    archFirstThreadRestore
.type archFirstThreadRestore, @function
archFirstThreadRestore:
    movl 4(%esp),%eax
    movl (%eax),%esp

    addl  $32,%esp 
    popf
    popl %ebp
    sti
    ret 
 
