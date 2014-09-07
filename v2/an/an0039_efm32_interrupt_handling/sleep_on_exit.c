/***************************************************************************//**
 * @file sleep_on_exit.c
 * @brief Sleep on Exit Example for EFM32.
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
#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_system.h"
#include "em_timer.h"
#include "segmentlcd.h"
#include "em_chip.h"

/* 1 second interval when assuming 1 MHz clock frequency and 1024x prescaling */
#define TOP (1000000/1024)

/* Interrupt counter */
volatile int interruptCount;

/**************************************************************************//**
 * @brief Initialize TIMER0 in Up Count mode and to give interrupt on overflow
 *****************************************************************************/
void initTimer()
{  
  TIMER_Init_TypeDef initValues = TIMER_INIT_DEFAULT;
  
  /* Enable clock for TIMER0 */
  CMU_ClockEnable(cmuClock_TIMER0, true);
  
  /* Enable overflow interrupt for TIMER0*/
  TIMER_IntEnable(TIMER0, TIMER_IF_OF);
  
  /* Enable TIMER0 interrupt vector in NVIC */
  NVIC_EnableIRQ(TIMER0_IRQn);
  
  /* Set TIMER0 Top value */
  TIMER_TopSet(TIMER0, TOP);
  
  /* Initialize TIMER0 in with 1024x prescaling */
  initValues.prescale = timerPrescale1024;
  TIMER_Init(TIMER0, &initValues);
  
  /* Start TIMER0 */
  TIMER0->CMD = TIMER_CMD_START;
}


/**************************************************************************//**
 * @brief TIMER0 Interrupt Handler. Writes interrupt count to LCD display
 *****************************************************************************/
void TIMER0_IRQHandler(void)
{ 
  /* Clear TIMER0 OF interrupt flag. */
  /* Do not need to check other flags since we know this is the only one enabled */
  TIMER_IntClear(TIMER0, TIMER_IFC_OF);

  /* Increment interrupt count and write to LCD */
  interruptCount++;
  SegmentLCD_Number(interruptCount);
  
  /* Wait until interrupt has been serviced 5 times */
  if (interruptCount >= 5)
  {
    /* Clear SLEEPONEXIT to go back to main after ISR */
     SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;
     
     /* Stop TIMER0 */
     TIMER0->CMD = TIMER_CMD_STOP;
  }
}


/**************************************************************************//**
 * @brief Main function
 *****************************************************************************/
int main(void)
{  
  /* Chip revision alignment and errata fixes */
  CHIP_Init();
  
  /* Set system frequency to 1 MHz */
  CMU_HFRCOBandSet(cmuHFRCOBand_1MHz);
  
  /* Initialize LCD */
  SegmentLCD_Init(false); 

  /* Initialize TIMER0 */
  initTimer();
  
  /* Enable Sleep-om-Exit */
  SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;
  
  /* Initialize interrupt count */
  interruptCount = 0;
  
  /* Enter EM1 until all TIMER0 interrupts are done
   * Notice that we only enter sleep once, as the MCU will fall asleep
   * immediately when the ISR is done without returning to main as long as
   * SLEEPONEXIT is set */
  EMU_EnterEM1();
  
  /* Signal that program is done */
  SegmentLCD_Write("DONE");
  while(1);
}
