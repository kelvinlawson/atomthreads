/*
 * Copyright (c) Himanshu Chauhan 2009-11.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 *     1. Redistributions of source code must retain the above copyright notice,
 *        this list of conditions and the following disclaimer.
 *    
 *     2. Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 * 
 *     3. Neither the name of Himanshu Chauhan nor the names of its 
 *        contributors may be used to endorse or promote products derived from
 *        this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <system.h>
#include <atomport.h>
#include <8250-serial.h>

#define PORT1 (void *)0xc00003f8

static inline unsigned int serial_in(int offset)
{
	return ioreadb(PORT1 + offset);
}

static inline void serial_out(int offset, int value)
{
	iowriteb(PORT1 + offset, value);
}

void putch(uint8_t c)
{
	while ((serial_in(UART_LSR) & UART_LSR_THRE) == 0)
		;

	serial_out(UART_TX, c);
}

void init_console()
{
	serial_out(1 , 0);   /* Turn off interrupts */

	/* Communication Settings */
	serial_out(3 , 0x80);  	/* SET DLAB ON */
	serial_out(0 , 0x01);  	/* Set Baud rate - Divisor Latch Low Byte */
  				/*         0x03 =  38,400 BPS */
  				/* Default 0x01 = 115,200 BPS */
  				/*         0x02 =  57,600 BPS */
  				/*         0x06 =  19,200 BPS */
  				/*         0x0C =   9,600 BPS */
  				/*         0x18 =   4,800 BPS */
  				/*         0x30 =   2,400 BPS */
	serial_out(1 , 0x00);  	/* Set Baud rate - Divisor Latch High Byte */
	serial_out(3 , 0x03);  	/* 8 Bits, No Parity, 1 Stop Bit */
	serial_out(2 , 0xC7);  	/* FIFO Control Register */
	serial_out(4 , 0x0B);  	/* Turn on DTR, RTS, and OUT2 */
}
