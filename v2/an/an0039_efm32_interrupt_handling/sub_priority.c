/***************************************************************************//**
 * @file sub_priority.c
 * @brief Sub Priority Example for EFM32.
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

#define STRINGLENGTH 5

/* String indicating order of interrupts executed */
char interruptStatus[STRINGLENGTH];

/***************************************************************************//**
 * @brief Configure TIMER0 and TIMER1 to give overflow interrupts in the same
 * cycle every 10000 cycles. 
 ******************************************************************************/
void initTimers()
{  
  TIMER_Init_TypeDef timerInit0 = TIMER_INIT_DEFAULT;
  TIMER_Init_TypeDef timerInit1 = TIMER_INIT_DEFAULT;
  
  /* Enable clock for TIMER0 and TIMER1 modules */
  CMU_ClockEnable(cmuClock_TIMER0, true);
  CMU_ClockEnable(cmuClock_TIMER1, true);
  
  /* Enable overflow interrupt */
  TIMER_IntEnable(TIMER0, TIMER_IF_OF);
  TIMER_IntEnable(TIMER1, TIMER_IF_OF);
  
  /* Enable TIMER0 and TIMER1 interrupt vector in NVIC */
  NVIC_EnableIRQ(TIMER0_IRQn);
  NVIC_EnableIRQ(TIMER1_IRQn);
  
  /* Set top value to the same value */
  TIMER_TopSet(TIMER0, 10000);
  TIMER_TopSet(TIMER1, 10000);
  
  /* Synch TIMER1 start with start of TIMER0 */
  timerInit1.sync = true;
  timerInit1.enable = false;
  
  /* Configure timers */
  TIMER_Init(TIMER1, &timerInit1);
  TIMER_Init(TIMER0, &timerInit0); /* This will start both TIMERs */
  
} 

/***************************************************************************//**
 * @brief TIMER0 Interrupt handler. Adds a "0" to the interruptStatus string
 * Initially the priority is set highest (0), but after first ISR
 * execution priority is set one lower (1)
 ******************************************************************************/
void TIMER0_IRQHandler(void)
{ 
  /* Clear interrupt flag */
  TIMER_IntClear(TIMER0, TIMER_IF_OF);
    
  /* Add "0" to string indicating an interrupt in TIMER0 */
  strncat(interruptStatus, "0", STRINGLENGTH);
  
  /* Decrease sub priority for TIMER0 (higher value = lower priority) */
  NVIC_SetPriority(TIMER0_IRQn, 1);  
}


/***************************************************************************//**
 * @brief TIMER1 Interrupt handler. Adds a "1" to the interruptStatus string.
 * Interrupt priority is always set to 0 for this ISR.
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
 * Sets up two timers to give interrupts in the same cycle
 * every 10000 cycles. Prints out order of ISR execution once each ISR has hit
 * twice. 
 *
 * First the interrupt priorities are equal for the two interrupts. Then TIMER0
 * ISR will execute first since its IRQ number is the lowest. 
 * 
 * For the second round, the subpriority of TIMER0 IRQ has been decreased, so 
 * TIMER1 ISR should be run first. 
 ******************************************************************************/
int main(void)
{ 
  /* Chip revision alignment and errata fixes */
  CHIP_Init();
  
  /* Initialize LCD to display results */
  SegmentLCD_Init(false);  
  
  /* Initialize LCD string with "-" to indicate start of string */
  strncpy(interruptStatus, "-", STRINGLENGTH);
  
  /* Set priority group to 1 preemption priority and 7 sub priorities */
  NVIC_SetPriorityGrouping(7);
  
  /* Initilize TIMER0 and TIMER1 */
  initTimers(); 

  /* Wait until string is full */
  while(strlen(interruptStatus)<STRINGLENGTH);
  
  /* Stop TIMERs */
  TIMER0->CMD = TIMER_CMD_STOP;
  
  /* Print string to LCD indicating order of interrupt execution */
  SegmentLCD_Write(interruptStatus);
  
  /* Wait here at the end */
  while(1);     
}
