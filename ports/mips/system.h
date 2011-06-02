/*
 * Copyright (c) Himanshu Chauhan 2009-11.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 *     1. Redistributions of source code must retain the above copyright notice,
 *        this list of conditions and the following disclaimer.
 *    
 *     2. Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 * 
 *     3. Neither the name of Himanshu Chauhan nor the names of its contributors 
 *        may be used to endorse or promote products derived from this software
 *        without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _SYSTEM_H
#define _SYSTEM_H

#include <atomport.h>
#include <stdarg.h>

extern const uint8_t *kernel_name;
extern const uint8_t *kernel_version;
extern const uint8_t *kernel_bdate;
extern const uint8_t *kernel_btime;

extern void *memcpy (void *dest, const void *src, size_t count);
extern void *memset (void *dest, int8_t val, size_t count);
extern uint16_t *memsetw (uint16_t *dest, uint16_t val, size_t count);
extern size_t strlen (const int8_t *str);
extern int vsprintf (int8_t *buf, const int8_t *fmt, va_list args);
extern void init_console (void);
extern int32_t arch_init (void);
extern uint8_t ioreadb (void *addr);
extern void iowriteb (void *addr, uint8_t data);

#endif /* _SYSTEM_H */
