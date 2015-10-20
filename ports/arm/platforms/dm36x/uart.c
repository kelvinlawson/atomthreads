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
 * Simple polled UART implementation for non-hosted compiler toolchains.
 *
 *
 * This is only required for non-hosted toolchains which don't implement
 * stdout automatically for use within QEMU.
 */

#include "atom.h"
#include "atommutex.h"
#include "atomport.h"
#include "dm36x-io.h"
#include "uart.h"


/* Constants */

/** Baudrate */
#define BAUDRATE         115200
 
/** Select relevant UART for this platform */
#define UART_BASE        DM36X_UART0_BASE

/** FR Register bits */
#define UART_FR_RXFE     0x10
#define UART_LSR_TEMT    0x40

/** UART register access macros */
#define UART_RBR(baseaddr) (*(unsigned int *)(baseaddr))
#define UART_THR(baseaddr) (*(unsigned int *)(baseaddr))
#define UART_IER(baseaddr) (*(((unsigned int *)(baseaddr + 0x04))))
#define UART_FCR(baseaddr) (*(((unsigned int *)(baseaddr + 0x08))))
#define UART_LCR(baseaddr) (*(((unsigned int *)(baseaddr + 0x0C))))
#define UART_MCR(baseaddr) (*(((unsigned int *)(baseaddr + 0x10))))
#define UART_LSR(baseaddr) (*(((unsigned int *)(baseaddr + 0x14))))
#define UART_DLL(baseaddr) (*(((unsigned int *)(baseaddr + 0x20))))
#define UART_DLH(baseaddr) (*(((unsigned int *)(baseaddr + 0x24))))
#define UART_PWR(baseaddr) (*(((unsigned int *)(baseaddr + 0x30))))


/* Local data */

/*
 * Semaphore for single-threaded access to UART device
 */
static ATOM_MUTEX uart_mutex;

/*
 * Initialised flag
 */
static int initialised = FALSE;


/* Forward declarations */
static int uart_init (void);
static void uart_write_char (const char c);


/**
 * \b uart_init
 *
 * Initialisation of UART driver. Creates a mutex that enforces
 * single-threaded access to the UART. We poll register bits
 * to check when space is available, which would not otherwise
 * be thread-safe.
 *
 * @retval ATOM_OK Success
 * @retval ATOM_ERROR Failed to create mutex
 */
static int uart_init (void)
{
    int status;
    uint32_t dummy;
    uint32_t divisor;

    /* Check we are not already initialised */
    if (initialised == FALSE)
    {
        /* Create a mutex for single-threaded UART access */
        if (atomMutexCreate (&uart_mutex) != ATOM_OK)
        {
            /* Mutex creation failed */
            status = ATOM_ERROR;
        }
        else
        {
            /* Reset Tx/Rx in PWREMU_MGMT */
            UART_PWR(UART_BASE) = 0x0;

            /* Set baudrate */
            divisor = (TIMER_CLK / BAUDRATE) / 16;
            UART_DLL(UART_BASE) = (divisor & 0xFF);
            UART_DLH(UART_BASE) = (divisor >> 8);

            /* Clear Tx/Rx FIFOs and enter non-FIFO mode */
            UART_FCR(UART_BASE) = 0x7;
            UART_FCR(UART_BASE) = 0x0;

            /* Set 8N1 */
            UART_LCR(UART_BASE) = 0x3;

            /* Disable loopback, flow-control, RTS/CTS */
            UART_MCR(UART_BASE) = 0x0;

            /* Disable interrupts */
            UART_IER(UART_BASE) = 0x0;

            /* Take Tx/Rx out of reset in PWREMU_MGMT */
            UART_PWR(UART_BASE) = 0xE001;

            /* Clear any receive characters */
            dummy = UART_RBR(UART_BASE);

            /* Success */
            initialised = TRUE;
            status = ATOM_OK;
        }
    }

    /* Already initialised */
    else
    {
        /* Success */
        status = ATOM_OK;
    }

    /* Finished */
    return (status);
}


/**
 * \b uart_read
 *
 * Simple polled UART read.
 *
 * @param[in] ptr Pointer to receive buffer
 * @param[in] len Max bytes to read
 *
 * @retval Number of bytes read
 *
 */
int uart_read (char *ptr, int len)
{
    int todo = 0;

    /* Check we are initialised */
    if (initialised == FALSE)
    {
        uart_init();
    }

    /* Check parameters */
    if ((ptr == NULL) || (len == 0))
    {
        return 0;
    }

    /* Block thread on private access to the UART */
    if (atomOSStarted && atomMutexGet(&uart_mutex, 0) == ATOM_OK)
    {
#if 0
        /* Wait for not-empty */
        while(UART_FR(UART_BASE) & UART_FR_RXFE)
            ;

        /* Read first byte */
        *ptr++ = UART_RBR(UART_BASE);

        /* Loop over remaining bytes until empty */
        for (todo = 1; todo < len; todo++)
        {
            /* Quit if receive FIFO empty */
            if(UART_FR(UART_BASE) & UART_FR_RXFE)
            {
                break;
            }

            /* Read next byte */
            *ptr++ = UART_RBR(UART_BASE);
        }
#endif

        /* Return mutex access */
        if (atomOSStarted)
        {
            atomMutexPut(&uart_mutex);
        }
    }

    /* Return number of bytes read */
    return todo;
}


/**
 * \b uart_write
 *
 * Simple polled UART write.
 *
 * @param[in] ptr Pointer to write buffer
 * @param[in] len Number of bytes to write
 *
 * @retval Number of bytes written
 */
int uart_write (const char *ptr, int len)
{
    int todo;

    /* Check we are initialised */
    if (initialised == FALSE)
    {
        uart_init();
    }

    /* Check parameters */
    if ((ptr == NULL) || (len == 0))
    {
        return 0;
    }

    /* Block thread on private access to the UART unless at interrupt context */
    if (atomOSStarted && ((atomCurrentContext() == NULL) || (atomMutexGet(&uart_mutex, 0) == ATOM_OK)))
    {
        /* Loop through all bytes to write */
        for (todo = 0; todo < len; todo++)
        {
            /* Convert \n to \r\n */
            if (*ptr == '\n')
                uart_write_char('\r');

            /* Write byte to UART */
            uart_write_char(*ptr++);
        }

        /* Return mutex access if not at interrupt context */
        if (atomOSStarted && (atomCurrentContext() != NULL))
        {
            atomMutexPut(&uart_mutex);
        }
    }

    /* Return bytes-written count */
    return len;
}


/**
 * \b uart_write_halt
 *
 * Simple polled UART write for handling critical failures
 * by printing out a message on the UART and looping forever.
 * Can be called from interrupt (unlike the standard
 * uart_write()) but is not thread-safe because it cannot
 * take the thread-safety mutex, and hence is only useful for
 * a last-resort catastrophic debug message.
 *
 * @param[in] ptr Pointer to write string
 */
void uart_write_halt (const char *ptr)
{
    /* Check parameters */
    if (ptr != NULL)
    {
        /* Loop through all bytes until NULL terminator encountered */
        while (*ptr != '\0')
        {
            /* Convert \n to \r\n */
            if (*ptr == '\n')
                uart_write_char('\r');

            /* Write byte to UART */
            uart_write_char(*ptr++);
        }
    }

    /* Loop forever */
    while (1)
        ;

}


/**
 * \b uart_putchar
 *
 * Simple polled UART write char.
 *
 * Assumes that the mutex has already been taken, or
 * is not expected to be taken (e.g. on interrupt).
 *
 * @param[in] c Char to write
 */
static void uart_write_char (const char c)
{
    /* Wait for empty */
    while ((UART_LSR(UART_BASE) & UART_LSR_TEMT) != UART_LSR_TEMT)
        ;

    /* Write byte to UART */
    UART_THR(UART_BASE) = (c & 0xFF);
}
