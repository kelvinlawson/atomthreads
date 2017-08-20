#include <avr/interrupt.h>

#include "atom.h"
#include "atomport-private.h"

/**
 * \b avrInitSystemTickTimer
 *
 * Initialise the system tick timer. Uses the AVR's timer1 facility.
 *
 * @return None
 */
void avrInitSystemTickTimer ( void )
{
    /* Set timer 1 compare match value for configured system tick,
     * with a prescaler of 256. We will get a compare match 1A
     * interrupt on every system tick, in which we must call the
     * OS's system tick handler. */
    OCR1A = (AVR_CPU_HZ / 256 / SYSTEM_TICKS_PER_SEC);

    /* Enable compare match 1A interrupt */
#ifdef TIMSK
    TIMSK = _BV(OCIE1A);
#else
    TIMSK1 = _BV(OCIE1A);
#endif

    /* Set prescaler 256 */
    TCCR1B = _BV(CS12) | _BV(WGM12);
}


/**
 *
 * System tick ISR.
 *
 * This is responsible for regularly calling the OS system tick handler.
 * The system tick handler checks if any timer callbacks are necessary,
 * and runs the scheduler.
 *
 * The compiler automatically saves all registers necessary before calling
 * out to a C routine. This will be (at least) R0, R1, SREG, R18-R27 and
 * R30/R31.
 *
 * The system may decide to schedule in a new thread during the call to
 * atomTimerTick(), in which case around half of the thread's context will
 * already have been saved here, ready for when we return here when the
 * interrupted thread is scheduled back in. The remaining context will be
 * saved by the context switch routine.
 *
 * As with all interrupts, the ISR should call atomIntEnter() and
 * atomIntExit() on entry and exit. This serves two purposes:
 *
 * a) To notify the OS that it is running in interrupt context
 * b) To defer the scheduler until after the ISR is completed
 *
 * We defer all scheduling decisions until after the ISR has completed
 * in case the interrupt handler makes more than one thread ready.
 *
 * @return None
 */
ISR (TIMER1_COMPA_vect)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the OS system tick handler */
    atomTimerTick();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}


/**
 *
 * Default (no handler installed) ISR.
 *
 * Installs a default handler to be called if any interrupts occur for
 * which we have not registered an ISR. This is empty and has only been
 * included to handle user-created code which may enable interrupts. The
 * core OS does not enable any interrupts other than the system timer
 * tick interrupt.
 *
 * @return None
 */
ISR (BADISR_vect)
{
    /* Empty */
}
