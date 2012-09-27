/*
 * Copyright (c) 2011, Anup Patel. All rights reserved.
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

#include <arm_io.h>
#include <arm_uart.h>

void arm_uart_putc(uint8_t ch)
{
	unsigned int base = 0x10009000;
	if(ch=='\n') {
		/* Wait until there is space in the FIFO */
		while (arm_readl((void*)(base + UART_PL01x_FR)) & UART_PL01x_FR_TXFF);

		/* Send the character */
		arm_writel('\r', (void*)(base + UART_PL01x_DR));
	}

	/* Wait until there is space in the FIFO */
	while (arm_readl((void*)(base + UART_PL01x_FR)) & UART_PL01x_FR_TXFF);

	/* Send the character */
	arm_writel(ch, (void*)(base + UART_PL01x_DR));
}

uint8_t arm_uart_getc(void)
{
	unsigned int base = 0x10009000;
	uint8_t data;

	/* Wait until there is data in the FIFO */
	while (arm_readl((void*)(base + UART_PL01x_FR)) & UART_PL01x_FR_RXFE);

	data = arm_readl((void*)(base + UART_PL01x_DR));

	/* Check for an error flag */
	if (data & 0xFFFFFF00) {
		/* Clear the error */
		arm_writel(0xFFFFFFFF, (void*)(base + UART_PL01x_ECR));
		return -1;
	}

	return data;
}

void arm_uart_init(void)
{
	unsigned int base = 0x10009000;
	unsigned int baudrate = 115200;
	unsigned int input_clock = 24000000;
	unsigned int divider;
	unsigned int temp;
	unsigned int remainder;
	unsigned int fraction;

	/* First, disable everything */
	arm_writel(0x0, (void*)(base + UART_PL011_CR));

	/*
	 * Set baud rate
	 *
	 * IBRD = UART_CLK / (16 * BAUD_RATE)
	 * FBRD = RND((64 * MOD(UART_CLK,(16 * BAUD_RATE))) 
	 * 	  / (16 * BAUD_RATE))
	 */
	temp = 16 * baudrate;
	divider = input_clock / temp;
	remainder = input_clock % temp;
	temp = (8 * remainder) / baudrate;
	fraction = (temp >> 1) + (temp & 1);

	arm_writel(divider, (void*)(base + UART_PL011_IBRD));
	arm_writel(fraction, (void*)(base + UART_PL011_FBRD));

	/* Set the UART to be 8 bits, 1 stop bit, 
	 * no parity, fifo enabled 
	 */
	arm_writel((UART_PL011_LCRH_WLEN_8 | UART_PL011_LCRH_FEN),
		(void*)(base + UART_PL011_LCRH));

	/* Finally, enable the UART */
	arm_writel((UART_PL011_CR_UARTEN | 
			UART_PL011_CR_TXE | 
			UART_PL011_CR_RXE),
		(void*)(base + UART_PL011_CR));
}

