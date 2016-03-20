/*
 * Copyright (c) 2013, Kelvin Lawson. All rights reserved.
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


/** 
 * \file
 * Syscalls implementation for stdio and heap management.
 *
 *
 * Simple implementation of syscalls.c for ARM compiler toolchains built
 * without newlib. Allows usage of printf() and friends as well as heap
 * allocation.
 *
 * NOTE: Platform/BSP must implement uart_read() and uart_write().
 *
 * NOTE: Platform/BSP linker script must define "end" and "heap_top" which are
 * the heap base and top respectively.
 *
 * No file table is implemented. All file read/write operations are carried
 * out on the UART driver, regardless of file descriptor.
 *
 * Mostly based on code from http://balau82.wordpress.com
 *
 */

#include <sys/stat.h>
#include "uart.h"


/** 
 * Define all functions as weak so that the functions in this file will
 * only be used if the compiler toolchain doesn't already provide them.
 */
extern int _close(int file) __attribute__((weak));
extern int _fstat(int file, struct stat *st) __attribute__((weak));
extern int _isatty(int file) __attribute__((weak));
extern int _lseek(int file, int ptr, int dir) __attribute__((weak));
extern int _open(const char *name, int flags, int mode) __attribute__((weak));
extern int _read(int file, char *ptr, int len) __attribute__((weak));
extern caddr_t _sbrk(int incr) __attribute__((weak));
extern int _write(int file, char *ptr, int len) __attribute__((weak));
extern int _exit(int val) __attribute__((weak));


/**
 * \b _close
 *
 * Simple stub implementation with no file table. All parameters ignored.
 *
 */
int _close(int file)
{
    return 0;
}


/**
 * \b _fstat
 *
 * Simple stub implementation. Always return character device.
 *
 */
int _fstat(int file, struct stat *st)
{
    /* Only UART supported, always return character-oriented device file */
    st->st_mode = S_IFCHR;
    return 0;
}


/**
 * \b _isatty
 *
 * Simple stub implementation. Only UART supported so TTY always true.
 *
 */
int _isatty(int file)
{
    return 1;
}


/**
 * \b _lseek
 *
 * Simple stub implementation. All parameters ignored.
 *
 */
int _lseek(int file, int ptr, int dir)
{
    return 0;
}


/**
 * \b _open
 *
 * Simple stub implementation with no file table. All parameters ignored.
 *
 * We only support reading/writing to the UART, so we don't bother inspecting
 * the filename to decide which underlying device to use, _read() and _write()
 * only access the UART driver.
 *
 * This is currently only called once (each thread opens its own stdout when
 * it starts executing).
 *
 */
int _open(const char *name, int flags, int mode)
{
    return 0;
}


/**
 * \b _read
 *
 * Simple read file implementation. Ignores file descriptor parameter
 * and always reads from the UART driver.
 *
 * @param[in] file File descriptor (parameter ignored)
 * @param[in] ptr Pointer to receive buffer
 * @param[in] len Max bytes to read
 *
 * @retval Number of bytes read
 */
int _read(int file, char *ptr, int len)
{
    /* Read from the UART driver, regardless of file descriptor */
    return (uart_read (ptr, len));
}


/**
 * \b _write
 *
 * Simple write file implementation. Ignores file descriptor parameter
 * and always writes to the UART driver.
 *
 * @param[in] file File descriptor (parameter ignored)
 * @param[in] ptr Pointer to write buffer
 * @param[in] len Number of bytes to write
 *
 * @retval Number of bytes written
 */
int _write(int file, char *ptr, int len)
{
    /* Write to the UART driver, regardless of file descriptor */
    return (uart_write (ptr, len));
}


/**
 * \b _sbrk
 *
 * Simple heap implementation.
 *
 * The platform/BSP must define "end" and "heap_top" which are the heap
 * base and top respectively.
 *
 * @param[in] incr Chunk size
 *
 * @retval Pointer to allocated chunk start
 */
caddr_t _sbrk(int incr)
{
 extern char end;          /* Defined by the linker */
 extern char heap_top;     /* Defined by the linker */
 static char *heap_end = 0;
 char *prev_heap_end;

    /* First time in, initialise heap base using definition from linker script */
    if (heap_end == 0)
    {
        heap_end = &end;
    }

    /* Save the previous heap base */
    prev_heap_end = heap_end;

    /* Check we have not passed the heap top */
    if (heap_end + incr > &heap_top)
    {
        /* Heap top reached, failed to allocate */
        return (caddr_t)0;
    }

    /* New heap base */
    heap_end += incr;

    /* Return pointer to previous base (where our allocation starts) */
    return (caddr_t)prev_heap_end;
}

/**
 * \b _exit
 *
 * Simple stub implementation, exit() not needed or implemented.
 *
 */
int _exit(int val)
{
	return -1;
}

