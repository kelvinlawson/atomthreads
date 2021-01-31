#include <stdio.h>

// device selection and project settings
#include "config.h"

#include "atom.h"
#include "atommutex.h"
#include "uart.h"


/*
 * Semaphore for single-threaded access to UART device
 */
static ATOM_MUTEX uart_mutex;


/*
 * Initialize the UART to requested baudrate, tx/rx, 8N1.
 */
int uart_init(uint32_t baudrate)
{
    uint16_t  tmp_u16;
    int       status;

    /**
     * Set up UART2 for putting out debug messages, change if required.
     */

    // set UART2 behaviour
    sfr_UART2.CR1.byte = sfr_UART2_CR1_RESET_VALUE;  // enable UART2, 8 data bits, no parity control
    sfr_UART2.CR2.byte = sfr_UART2_CR2_RESET_VALUE;  // no interrupts, disable sender/receiver
    sfr_UART2.CR3.byte = sfr_UART2_CR3_RESET_VALUE;  // no LIN support, 1 stop bit, no clock output(?)

    // set baudrate (note: BRR2 must be written before BRR1!)
    tmp_u16 = (uint16_t) (((uint32_t) FSYS_FREQ)/baudrate);
    sfr_UART2.BRR2.byte = (uint8_t) (((tmp_u16 & 0xF000) >> 8) | (tmp_u16 & 0x000F));
    sfr_UART2.BRR1.byte = (uint8_t) ((tmp_u16 & 0x0FF0) >> 4);

    // enable transmission, no transmission
    sfr_UART2.CR2.REN  = 1;  // enable receiver
    sfr_UART2.CR2.TEN  = 1;  // enable sender
    //sfr_UART2.CR2.TIEN = 1;  // enable transmit interrupt
    //sfr_UART2.CR2.RIEN = 1;  // enable receive interrupt
        

    /* Create a mutex for single-threaded putchar() access */
    if (atomMutexCreate (&uart_mutex) != ATOM_OK)
    {
        status = -1;
    }
    else
    {
        status = 0;
    }

    /* Finished */
    return (status);
}


/**
 * \b uart_putchar
 *
 * Write a char out via UART2
 *
 * @param[in] c Character to send
 *
 * @return Character sent
 */
char uart_putchar (char c)
{
    /* Block on private access to the UART */
    if (atomMutexGet(&uart_mutex, 0) == ATOM_OK)
    {
        /* Convert \n to \r\n */
        if (c == '\n')
            putchar('\r');

        /* Write a character to the UART2 */
        sfr_UART2.DR.byte = c;

        /* Loop until the end of transmission */
        while (!(sfr_UART2.SR.TXE))
            ;

        /* Return mutex access */
        atomMutexPut(&uart_mutex);

    }

    return (c);
}


/* Implement putchar() routine for stdio */
#if defined(__CSMC__)
  char putchar(char c)
#else // Standard C
  int putchar(int c)
#endif
{
    return (uart_putchar(c));
}
