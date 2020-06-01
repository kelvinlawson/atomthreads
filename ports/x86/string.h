/*
 * AtomThreads project - Copyright (c) 2010, Kelvin Lawson. All rights reserved.
 * 
 * Please refer to the README file for further details and License information.
 *
 * x86 port by: ido@mm.st
 */

#ifndef	_STRING_H
#define	_STRING_H

#include <stddef.h>
#include <stdint.h>

void *memcpyw(uint16_t* restrict dest, const uint16_t* restrict src, size_t n);
uint16_t *memsetw(uint16_t *dest, uint16_t val, size_t count);
void *memcpy(void *restrict dest, const void *restrict src, size_t n);

#endif
