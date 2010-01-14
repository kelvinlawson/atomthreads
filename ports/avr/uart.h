/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <joerg@FreeBSD.ORG> wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.        Joerg Wunsch
 * ----------------------------------------------------------------------------
 *
 * Stdio demo, UART declarations
 *
 * $Id: uart.h,v 1.1 2005/12/28 21:38:59 joerg_wunsch Exp $
 */

#include "atom.h"


/*
 * Perform UART startup initialization.
 */
int	uart_init(uint32_t baudrate);

/*
 * Send one character to the UART.
 */
int	uart_putchar(char c, FILE *stream);
