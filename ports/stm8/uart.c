/*
 * Copyright (c) 2013, Wei Shuai <cpuwolf@gmail.com>. All rights reserved.
 *
 * STM8L151K4T6 only supports UART1
 */
#include <stdio.h>
#include <stddef.h>

#include <stm8l15x.h>

#include <atom.h>
#include <atommutex.h>
#include "uart.h"

#ifdef _RAISONANCE_
#define PUTCHAR_PROTOTYPE int putchar (char c)
#define GETCHAR_PROTOTYPE int getchar (void)
#elif defined (_COSMIC_)
#define PUTCHAR_PROTOTYPE char putchar (char c)
#define GETCHAR_PROTOTYPE char getchar (void)
#else /* _IAR_ */
#define PUTCHAR_PROTOTYPE int putchar (int c)
#define GETCHAR_PROTOTYPE int getchar (void)
#endif /* _RAISONANCE_ */
/*
 * Semaphore for single-threaded access to UART device
 */
static ATOM_MUTEX uart_mutex;


/*
 * Initialize the UART to requested baudrate, tx/rx, 8N1.
 */
int uart_init(uint32_t baudrate)
{
  int status;
  
  /**
   * Set up UART1 for putting out debug messages.
   */
  USART_DeInit(USART1);

    /* USART configuration */
  USART_Init(USART1, baudrate,
             USART_WordLength_8b,
             USART_StopBits_1,
             USART_Parity_No,
             (USART_Mode_TypeDef)(USART_Mode_Tx | USART_Mode_Rx));

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
 * Write a char out via UART1
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

        /* Write a character to the USART1 */
         USART_SendData8(USART1, c);
        /* Loop until the end of transmission */
        while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);


        /* Return mutex access */
        atomMutexPut(&uart_mutex);

    }

    return (c);
}


/* COSMIC: Requires putchar() routine to override stdio */
#if defined(__CSMC__)
/**
 * \b putchar
 *
 * Retarget putchar() to use UART1
 *
 * @param[in] c Character to send
 *
 * @return Character sent
 */
char putchar (char c)
{
    return (uart_putchar(c));
}
#endif /* __CSMC__ */


/* RAISONANCE: Requires putchar() routine to override stdio */
#if defined(__RCSTM8__)
/**
 * \b putchar
 *
 * Retarget putchar() to use UART1
 *
 * @param[in] c Character to send
 *
 * @return 1 on success
 */
int putchar (char c)
{
    uart_putchar(c);
    return (1);
}
#endif /* __RCSTM8__ */


/* IAR: Requires __write() routine to override stdio */
#if defined(__IAR_SYSTEMS_ICC__)
/**
 * \b __write
 *
 * Override for IAR stream output
 *
 * @param[in] handle Stdio handle. -1 to flush.
 * @param[in] buf Pointer to buffer to be written
 * @param[in] bufSize Number of characters to be written
 *
 * @return Number of characters sent
 */
size_t __write(int handle, const unsigned char *buf, size_t bufSize)
{
    size_t chars_written = 0;
    
    /* Ignore flushes */
    if (handle == -1)
    {
      chars_written = (size_t)0; 
    }
    /* Only allow stdout/stderr output */
    else if ((handle != 1) && (handle != 2))
    {
      chars_written = (size_t)-1; 
    }
    /* Parameters OK, call the low-level character output routine */
    else
    {
        while (chars_written < bufSize)
        {
            uart_putchar (buf[chars_written]);
            chars_written++;
        }
    }
    
    return (chars_written);
}
#endif /* __IAR_SYSTEMS_ICC__ */
