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

#include <errno.h>

#include <libopencm3/cm3/cortex.h>
#include <libopencm3/stm32/usart.h>

/**
 * _read and _write for STM32
 *
 * _read and _write are used by newlib's I/O routines (think printf, etc.)
 * If you want to use this code in your binary, you will have to initialise
 * the UART in your board's setup code and define STD_CON to your UART's
 * name in your board's Makefile
 */
int _read(int fd, void *buf, size_t count)
{
    int rcvd;
    char *ptr;

    if(fd <= 2){
        ptr = (char *) buf;
        rcvd = 0;
        while(count > 0){
            *ptr = usart_recv_blocking(STD_CON);
            if(*ptr == '\r'){
                *ptr = '\n';
            }
            ++rcvd;
            --count;
        }
    }else{
        rcvd = -1;
        errno = EIO;
    }

    return rcvd;
}

int _write(int fd, const void *buf, size_t count)
{
    int sent;
    char *ptr;

    if(fd <= 2){
        sent = count;
        ptr = (char *) buf;

        while(count > 0){
            if(*ptr == '\n'){
                usart_send_blocking(STD_CON, '\r');
            }
            usart_send_blocking(STD_CON, *ptr++);
            ++sent;
            --count;
        }

    }else{
        errno = EIO;
        sent = -1;
    }

    return sent;
}

