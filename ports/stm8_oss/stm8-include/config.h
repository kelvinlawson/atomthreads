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
//#define STM8L_DISCOVERY
//#define SDUINO
//#define MUBOARD


/*----------------------------------------------------------
    INCLUDE FILES
----------------------------------------------------------*/
#if defined(STM8S_DISCOVERY)
  #include "STM8S105C6.h"
  #define LED_PORT   sfr_PORTD                       // port of LED pin
  #define LED_PIN    PIN0                            // bitmask for LED pin
  #define USE_TIM2                                   // TIM2(16bit) or TIM4(8bit). TIM2 requires less tweaking
  #define TIM2_ISR_VECTOR   _TIM2_OVR_UIF_VECTOR_    // TIM2 update interrupt vector
  #define TIM4_ISR_VECTOR   _TIM4_OVR_UIF_VECTOR_    // TIM4 update interrupt vector
  #define TIM2_ISR_UIF      sfr_TIM2.SR1.UIF         // TIM2 clear interrupt flag
  #define TIM4_ISR_UIF      sfr_TIM4.SR.UIF          // TIM4 clear interrupt flag
  #define USE_UART2                                  // UART for logging 
#elif defined(STM8L_DISCOVERY)
  #include "STM8L152C6.h"
  #define LED_PORT   sfr_PORTE
  #define LED_PIN    PIN7
  #define USE_TIM2
  #define TIM2_ISR_VECTOR   _TIM2_OVR_UIF_VECTOR_
  #define TIM4_ISR_VECTOR   _TIM4_UIF_VECTOR_
  #define TIM2_ISR_UIF      sfr_TIM2.SR1.UIF
  #define TIM4_ISR_UIF      sfr_TIM4.SR1.UIF
  #define USE_USART1
#elif defined(SDUINO)
  #include "STM8S105K6.h"
  #define LED_PORT   sfr_PORTC
  #define LED_PIN    PIN5
  #define USE_TIM2
  #define TIM2_ISR_VECTOR   _TIM2_OVR_UIF_VECTOR_
  #define TIM4_ISR_VECTOR   _TIM4_OVR_UIF_VECTOR_
  #define TIM2_ISR_UIF      sfr_TIM2.SR1.UIF
  #define TIM4_ISR_UIF      sfr_TIM4.SR.UIF
  #define USE_UART2
#elif defined(MUBOARD)
  #include "STM8S207MB.h"
  #define LED_PORT   sfr_PORTH
  #define LED_PIN    PIN2
  #define USE_TIM2
  #define TIM2_ISR_VECTOR   _TIM2_OVR_UIF_VECTOR_
  #define TIM4_ISR_VECTOR   _TIM4_OVR_UIF_VECTOR_
  #define TIM2_ISR_UIF      sfr_TIM2.SR1.UIF
  #define TIM4_ISR_UIF      sfr_TIM4.SR.UIF
#else
  #error undefined board
#endif


/*----------------------------------------------------------
    MACROS/DEFINES
----------------------------------------------------------*/

// system clock frequency [Hz]. Currently 16MHz, 20MHh, 24MHz (see atomport.c)
#define FSYS_FREQ       16000000L

// base timeslice for atomthreads [ms]
#define PERIOD_THREADS  10


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _CONFIG_H_
