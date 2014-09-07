/***************************************************************************//**
 * @file preempt_priority.c
 * @brief Pre-emption Priority Example for EFM32.
 * @author Silicon Labs
 * @version 1.05
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/
#include <string.h>
#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_system.h"
#include "em_timer.h"
#include "em_chip.h"
#include "segmentlcd.h"

#define STRINGLENGTH 7

/* String indicating order of interrupts executed */
char interruptStatus[STRINGLENGTH];


/***************************************************************************//**
 * @brief TIMER0 Interrupt handler. 
 * Will print add "A" to string before triggering TIMER1 IRQ. Prints "B" after
 ******************************************************************************/
void TIMER0_IRQHandler(void)
{ 
  /* Clear interrupt flag */
  TIMER_IntClear(TIMER0, TIMER_IF_OF);
    
  /* Add "A" to string indicating first part of TIMER0 ISR*/
  strncat(interruptStatus, "A", STRINGLENGTH);
  
  /* Trigger TIMER1 OF interrupt */
  TIMER_IntSet(TIMER1, TIMER_IF_OF);
  
  /* Add "B" to string indicating second part of TIMER0 ISR */
  strncat(interruptStatus, "B", STRINGLENGTH);
}


/***************************************************************************//**
 * @brief TIMER1 Interrupt handler. Adds a "1" to the interruptStatus string.
 ******************************************************************************/
void TIMER1_IRQHandler(void)
{ 
  /* Clear interrupt flag */
  TIMER_IntClear(TIMER1, TIMER_IF_OF);
  
  /* Add "1" to string indicating in interrupt in TIMER1 */
  strncat(interruptStatus, "1", STRINGLENGTH);  
}

/***************************************************************************//**
 * @brief Main function. 
 * Enables TIMER0 and TIMER1 interrupts. TIMER0 interrupt is triggered by SW.
 * TIMER1 IRQ is then set in the middle of the TIMER0 ISR.
 * 
 * In the first round both IRQs have same preemption priority and TIMER0 ISR is
 * therefore done before TIMER1 ISR is started
 *
 * In the second round TIMER1 IRQ has a higher preemption priority than TIMER0
 * and the TIMER1 ISR is therefore executed immediately in the middle of the 
 * TIMER0 ISR. 
 ******************************************************************************/
int main(void)
{  
  /* Chip revision alignment and errata fixes */
  CHIP_Init();
  
  /* Initialize LCD to display results */
  SegmentLCD_Init(false);  
  
  /* Initialize LCD string with "-" to indicate start of string */
  strncpy(interruptStatus, "-", STRINGLENGTH);
  
  /* Set priority group to 7 preemption priorities and 1 sub priority */
  NVIC_SetPriorityGrouping(0);
  
  /* Enable clock for TIMER0 and TIMER1 modules */
  CMU_ClockEnable(cmuClock_TIMER0, true);
  CMU_ClockEnable(cmuClock_TIMER1, true);

  /* Enable overflow interrupt from TIMER0 and TIMER1 */
  TIMER_IntEnable(TIMER0, TIMER_IF_OF);
  TIMER_IntEnable(TIMER1, TIMER_IF_OF);
  
  /* Enable TIMER0 and TIMER1 interrupt vector in NVIC */
  NVIC_EnableIRQ(TIMER0_IRQn);
  NVIC_EnableIRQ(TIMER1_IRQn);
  
  /* Trigger TIMER0 interrupt.
   * TIMER1 IRQ will be triggered by TIMER0 ISR.
   * Because of equal preempt priority, TIMER0 ISR should finish before TIMER0
   * ISR starts */
  TIMER_IntSet(TIMER0, TIMER_IF_OF);
  
  /* Decrease preempt priority of TIMER0 IRQ (higher value = lower priority) */
  NVIC_SetPriority(TIMER0_IRQn, 1); 
  
  /* Trigger TIMER0 interrupt. 
   * TIMER1 IRQ will be triggered by TIMER0 ISR.
   * Because TIMER1 has higher preempt priority, TIMER1 ISR will start
   * executing in the middle of the TIMER0 ISR */
  TIMER_IntSet(TIMER0, TIMER_IF_OF);
  
  /* Print string to LCD indicating order of interrupt execution */
  SegmentLCD_Write(interruptStatus);
  
  /* Wait here at the end */
  while(1);     
}
