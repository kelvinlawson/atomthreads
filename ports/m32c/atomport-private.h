/*
 * Copyright (c) 2014, Juan Angel Hernandez Hdez. for Atomthreads Project.
 * All rights reserved.
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

#ifndef __ATOM_PORT_PRIVATE_H
#define __ATOM_PORT_PRIVATE_H

/* Enumeration for interrupt level */
typedef enum
{
    INTERRUPT_LVL_0 = 0x00,
    INTERRUPT_LVL_1 = 0x01,
    INTERRUPT_LVL_2 = 0x02,
    INTERRUPT_LVL_3 = 0x03,
    INTERRUPT_LVL_4 = 0x04,
    INTERRUPT_LVL_5 = 0x05,
    INTERRUPT_LVL_6 = 0x06,
    INTERRUPT_LVL_7 = 0x07
} teINTERRUPT_PRIORITY_LEVEL;

/* CPU Frequency */
#define M32C_OSC_FREQUENCY  20000000L
#define PRESCALE_VALUE		8

/* 1MS tick defines */
#define COUNT_10MS 	(M32C_OSC_FREQUENCY/PRESCALE_VALUE)/100

/* Using simulator */
#define HEW_SIMULATOR

/* Function prototypes */
void init_timerb2(void);
void init_pin_P0(void);
void toggle_pin_P0(void);

/* Timer B2 registers */
#pragma ADDRESS TB2MR 035Dh
#pragma ADDRESS TB2   0354h
#pragma ADDRESS TB2IC 0096h
uint8_t  TB2MR;
uint16_t TB2;
uint8_t  TB2IC;

/* TIMER B2         (software int 23) */
#pragma interrupt _timer_b2(vect=23)

/* LED pin */
#pragma ADDRESS PD0	03E2h //Input or output 0:Input 1:Output
#pragma ADDRESS P0	03E0h //R/W
uint8_t PD0;
uint8_t P0;

#endif /* __ATOM_PORT_PRIVATE_H */
