/**
  \file config.h

  \brief set project configurations

  set project configurations like used device or board etc.
*/

/*-----------------------------------------------------------------------------
    MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _CONFIG_H_
#define _CONFIG_H_

/*----------------------------------------------------------
    SELECT BOARD
----------------------------------------------------------*/
#define STM8S_DISCOVERY
//#define SDUINO
//#define MUBOARD


/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#if defined(STM8S_DISCOVERY)
  #include "STM8S105C6.h"
  #define LED_PORT   sfr_PORTD
  #define LED_PIN    PIN0
#elif defined(SDUINO)
  #include "STM8S105K6.h"
  #define LED_PORT   sfr_PORTC
  #define LED_PIN    PIN5
#elif defined(MUBOARD)
  #include "STM8S207MB.h"
  #define LED_PORT   sfr_PORTH
  #define LED_PIN    PIN2
#else
  #error undefined board
#endif


/*----------------------------------------------------------
    MACROS/DEFINES
----------------------------------------------------------*/

// system clock frequency [Hz]
#define FSYS_FREQ       16000000L

// base timer for systics (TIM2 (16bit) or TIM4 (8bit)).
// TIM2 requires less tweaking, but may be used e.g. for PWM generation
#define SYSTIMER        USE_TIM2
//#define SYSTIMER        USE_TIM4

// base timeslice for atomthreads [ms]
#define PERIOD_THREADS  10


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _CONFIG_H_
