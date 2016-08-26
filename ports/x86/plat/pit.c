/*
 * AtomThreads project - Copyright (c) 2010, Kelvin Lawson. All rights reserved.
 * 
 * Please refer to the README file for further details and License information.
 *
 * x86 port by: ido@mm.st
 */

#include "x86.h"

static uint64_t timer_delta_time;
static uint16_t divisor;

/* PIT (i8253) registers */
#define I8253_CONTROL_REG 0x43
#define I8253_DATA_REG  0x40

#define INTERNAL_FREQ 1193182ULL
#define INTERNAL_FREQ_3X 3579546ULL

void pit_init(uint32_t frequency)
{
    uint32_t count, remainder;

    /* figure out the correct divisor for the desired frequency */
    if (frequency <= 18) {
        count = 0xffff;
    } else if (frequency >= INTERNAL_FREQ) {
        count = 1;
    } else {
        count = INTERNAL_FREQ_3X / frequency;
        remainder = INTERNAL_FREQ_3X % frequency;

        if (remainder >= INTERNAL_FREQ_3X / 2) {
            count += 1;
        }

        count /= 3;
        remainder = count % 3;

        if (remainder >= 1) {
            count += 1;
        }
    }

    divisor = count & 0xffff;

    timer_delta_time = (3685982306ULL * count) >> 10;

    out8(I8253_CONTROL_REG, 0x34);
    out8(I8253_DATA_REG, divisor & 0xff); // LSB
    out8(I8253_DATA_REG, divisor >> 8); // MSB
}
