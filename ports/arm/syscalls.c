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
 * Mostly based on code from http://balau82.wordpress.com
 *
 */

#include <sys/stat.h>

enum {
 UART_FR_RXFE = 0x10,
 UART_FR_TXFF = 0x20,
 UART0_ADDR = 0x16000000,
};

#define UART_DR(baseaddr) (*(unsigned int *)(baseaddr))
#define UART_FR(baseaddr) (*(((unsigned int *)(baseaddr))+6))


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


/**
 * \b _close
 *
 * Simple stub implementation.
 *
 */
int _close(int file)
{
    return -1;
}


/**
 * \b _fstat
 *
 * Simple stub implementation.
 *
 */
int _fstat(int file, struct stat *st)
{
    st->st_mode = S_IFCHR;
    return 0;
}


/**
 * \b _isatty
 *
 * Simple stub implementation.
 *
 */
int _isatty(int file)
{
    return 1;
}


/**
 * \b _lseek
 *
 * Simple stub implementation.
 *
 */
int _lseek(int file, int ptr, int dir)
{
    return 0;
}


/**
 * \b _open
 *
 * Simple stub implementation.
 *
 */
int _open(const char *name, int flags, int mode)
{
    return -1;
}


/**
 * \b _read
 *
 * Simple stub implementation.
 *
 */
int _read(int file, char *ptr, int len)
{
    int todo;

    if(len == 0)
        return 0;

    while(UART_FR(UART0_ADDR) & UART_FR_RXFE)
        ;

    *ptr++ = UART_DR(UART0_ADDR);
    for (todo = 1; todo < len; todo++)
    {
        if(UART_FR(UART0_ADDR) & UART_FR_RXFE)
        {
            break;
        }
        *ptr++ = UART_DR(UART0_ADDR);
    }
    return todo;
}


/**
 * \b _write
 *
 * Simple stub implementation.
 *
 */
int _write(int file, char *ptr, int len)
{
    int todo;

    for (todo = 0; todo < len; todo++)
    {
        while(UART_FR(UART0_ADDR) & UART_FR_TXFF)
            ;
        UART_DR(UART0_ADDR) = *ptr++;
    }

    return len;
}


/**
 * \b _sbrk
 *
 * Simple stub implementation.
 *
 */
caddr_t _sbrk(int incr)
{
 extern char end;          /* Defined by the linker */
 extern char heap_top;     /* Defined by the linker */
 static char *heap_end = 0;
 char *prev_heap_end;

    if (heap_end == 0)
    {
        heap_end = &end;
    }
    prev_heap_end = heap_end;

    if (heap_end + incr > &heap_top)
    {
        /* Heap and stack collision */
        return (caddr_t)0;
    }

    heap_end += incr;
    return (caddr_t)prev_heap_end;
}

