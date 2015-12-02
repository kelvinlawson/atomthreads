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

#include <sys/stat.h>

#include <libopencm3/cm3/vector.h>

#include "atomport.h"

/**
 * _sbrk is needed by newlib for heap management.
 * The main stack usually starts at the top of RAM and grows downwards,
 * while the heap starts at the bottom and grows upwards. To prevent
 * the heap from colliding with the main stack, we reserve a certain amount
 * of memory and check that growing the heap won't touch that region.
 * The size of the protected area is set via the define MST_SIZE in the
 * board Makefile
 */

#ifndef MST_SIZE
#error Main stack size not defined. Please define MST_SIZE in board Makefile
#endif

/**
 * The vector table is provided by libopencm3/cm3/vector.h and contains
 * the initial main stack pointer value
 */
extern vector_table_t vector_table;

/**
 * The linker provides the start address of the unused system memory.
 * It is exported via the symbol 'end' and will be used as the bottom
 * of the heap
 */
extern char end;

static char *heap_end = 0;
caddr_t _sbrk(int incr)
{
    char *prev_end;
    CRITICAL_STORE;

    prev_end = NULL;

    CRITICAL_START();

    if(unlikely(heap_end == 0)){
        heap_end = &end;
    }

    /* make sure new heap size does not collide with main stack area*/
    if(heap_end + incr + MST_SIZE <= (char *) vector_table.initial_sp_value){
        prev_end = heap_end;
        heap_end += incr;
    }

    CRITICAL_END();

    return (caddr_t) prev_end;
}

/**
 * dummy stubs needed by newlib when not linked with libnosys
 */
int _close(int file __maybe_unused)
{
    return -1;
}

int _fstat(int file __maybe_unused, struct stat *st)
{
    st->st_mode = S_IFCHR;

    return 0;
}

int _isatty(int file __maybe_unused)
{
    return 1;
}

int _lseek(int file __maybe_unused,
           int ptr __maybe_unused,
           int dir __maybe_unused)
{
    return 0;
}

int _open(const char *name __maybe_unused,
          int flags __maybe_unused,
          int mode __maybe_unused)
{
    return -1;
}
