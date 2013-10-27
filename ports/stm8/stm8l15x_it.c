/************************ (C) COPYRIGHT STMicroelectronics  */
/*
 * Copyright (c) 2013, Wei Shuai <cpuwolf@gmail.com>. All rights reserved.
 *
 */

/* Includes ------------------------------------------------------------------*/
#include <atom.h>
#include "stm8l15x_it.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

#ifdef _COSMIC_
/**
  * \b Dummy interrupt routine
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(NonHandledInterrupt, 0)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}
#endif

/**
  * \b TRAP interrupt routine
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER_TRAP(TRAP_IRQHandler)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}
/**
  * \b FLASH Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(FLASH_IRQHandler, 1)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}
/**
  * \b DMA1 channel0 and channel1 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(DMA1_CHANNEL0_1_IRQHandler, 2)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}
/**
  * \b DMA1 channel2 and channel3 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(DMA1_CHANNEL2_3_IRQHandler, 3)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}
/**
  * \b RTC / CSS_LSE Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(RTC_CSSLSE_IRQHandler, 4)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}
/**
  * \b External IT PORTE/F and PVD Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTIE_F_PVD_IRQHandler, 5)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}

/**
  * \b External IT PORTB / PORTG Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTIB_G_IRQHandler, 6)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}

/**
  * \b External IT PORTD /PORTH Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTID_H_IRQHandler, 7)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}

/**
  * \b External IT PIN0 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI0_IRQHandler, 8)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}

/**
  * \b External IT PIN1 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI1_IRQHandler, 9)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}

/**
  * \b External IT PIN2 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI2_IRQHandler, 10)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}

/**
  * \b External IT PIN3 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI3_IRQHandler, 11)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}

/**
  * \b External IT PIN4 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI4_IRQHandler, 12)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}

/**
  * \b External IT PIN5 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI5_IRQHandler, 13)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}

/**
  * \b External IT PIN6 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI6_IRQHandler, 14)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}

/**
  * \b External IT PIN7 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI7_IRQHandler, 15)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}
/**
  * \b LCD /AES Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(LCD_AES_IRQHandler, 16)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}
/**
  * \b CLK switch/CSS/TIM1 break Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(SWITCH_CSS_BREAK_DAC_IRQHandler, 17)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}

/**
  * \b ADC1/Comparator Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(ADC1_COMP_IRQHandler, 18)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}

/**
  * \b TIM2 Update/Overflow/Trigger/Break /USART2 TX Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM2_UPD_OVF_TRG_BRK_USART2_TX_IRQHandler, 19)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}

/**
  * \b Timer2 Capture/Compare / USART2 RX Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM2_CC_USART2_RX_IRQHandler, 20)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}


/**
  * \b Timer3 Update/Overflow/Trigger/Break Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM3_UPD_OVF_TRG_BRK_USART3_TX_IRQHandler, 21)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}
/**
  * \b Timer3 Capture/Compare /USART3 RX Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM3_CC_USART3_RX_IRQHandler, 22)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}
/**
 *
 * System tick ISR.
 *
 * This is responsible for regularly calling the OS system tick handler.
 * The system tick handler checks if any timer callbacks are necessary,
 * and runs the scheduler.
 *
 * The CPU automatically saves all registers before calling out to an
 * interrupt handler like this.
 *
 * The system may decide to schedule in a new thread during the call to
 * atomTimerTick(), in which case the program counter will be redirected
 * to the new thread's running location during atomIntExit(). This ISR
 * function will not actually complete until the thread we interrupted is
 * scheduled back in, at which point the end of this function will be
 * reached (after atomIntExit()) and the IRET call by the compiler will
 * return us to the interrupted thread as if we hadn't run any other
 * thread in the meantime. In other words the interrupted thread can be
 * scheduled out by atomIntExit() and several threads could run before we
 * actually reach the end of this function. When this function does
 * finally complete, the return address (the PC of the thread which was
 * interrupted) will be on the interrupted thread's stack because it was
 * saved on there by the CPU when the interrupt triggered.
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
/**
  * \b TIM1 Update/Overflow/Trigger/Commutation Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_COM_IRQHandler, 23)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the OS system tick handler */
    atomTimerTick();
    
    TIM1_ClearITPendingBit(TIM1_IT_Update);
    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}
/**
  * \b TIM1 Capture/Compare Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM1_CC_IRQHandler, 24)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}

/**
  * \b TIM4 Update/Overflow/Trigger Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM4_UPD_OVF_TRG_IRQHandler, 25)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}
/**
  * \b SPI1 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(SPI1_IRQHandler, 26)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}

/**
  * \b USART1 TX / TIM5 Update/Overflow/Trigger/Break Interrupt  routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(USART1_TX_TIM5_UPD_OVF_TRG_BRK_IRQHandler, 27)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}

/**
  * \b USART1 RX / Timer5 Capture/Compare Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(USART1_RX_TIM5_CC_IRQHandler, 28)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}

/**
  * \b I2C1 / SPI2 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(I2C1_SPI2_IRQHandler, 29)
{
    /* Call the interrupt entry routine */
    atomIntEnter();

    /* Call the interrupt exit routine */
    atomIntExit(TRUE);
}

