/*
 * AtomThreads project - Copyright (c) 2010, Kelvin Lawson. All rights reserved.
 * 
 * Please refer to the README file for further details and License information.
 *
 * x86 port by: ido@mm.st
 */

#include "string.h"

uint16_t *memsetw(uint16_t *dest, uint16_t val, size_t count)
{
    uint16_t *temp = dest;
    while(count--) *temp++ = val;
    return dest;
}

void *memcpyw(uint16_t* restrict dest, const uint16_t* restrict src, size_t n)
{
	uint8_t *d = (uint8_t*) dest;
	const uint8_t *s = (const uint8_t*) src;
	while(n--) *dest++ = *src++;
	return dest;
}

void *memcpy(void *restrict dest, const void *restrict src, size_t n)
{
	uint8_t *d = dest;
	const uint8_t *s = src;
	while(n--) *d++=*s++;
	return dest;
}

