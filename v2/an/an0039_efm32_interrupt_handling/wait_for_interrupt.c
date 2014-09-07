/***************************************************************************//**
 * @file wait_for_interrupt.c
 * @brief Wait-for-Interrupt (WFI) Example for EFM32.
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
#include "em_lcd.h"
#include "em_system.h"
#include "em_timer.h"
#include "segmentlcd.h"
#include "em_chip.h"

/* 1 second interval when assuming 1 MHz clock frequency and 1024x prescaling */
#define TOP (1000000/1024) 

/**************************************************************************//**
 * @brief Initialize TIMER0 in Up/Down Count mode and to give interrupts when 
 * turning from counting up to down (overflow) and from down to up (underflow)
 *****************************************************************************/
void initTimer()
{  
  TIMER_Init_TypeDef initValues = TIMER_INIT_DEFAULT;
  
  /* Enable clock for TIMER0 */
  CMU_ClockEnable(cmuClock_TIMER0, true);
  
  /* Enable underflow and overflow interrupt for TIMER0*/
  TIMER_IntEnable(TIMER0, TIMER_IF_OF);
  TIMER_IntEnable(TIMER0, TIMER_IF_UF);
  
  /* Enable TIMER0 interrupt vector in NVIC */
  NVIC_EnableIRQ(TIMER0_IRQn);
  
  /* Set TIMER0 Top value */
  TIMER_TopSet(TIMER0, TOP);
  
  /* Initialize TIMER0 in Up/Down mode with 1024x prescaling */
  initValues.prescale = timerPrescale1024;
  initValues.mode     = timerModeUpDown;
  TIMER_Init(TIMER0, &initValues);
  
  /* Start TIMER0 */
  TIMER0->CMD = TIMER_CMD_START;
}


/**************************************************************************//**
 * @brief TIMER0 Interrupt Handler. Writes status to LCD display
 *****************************************************************************/
void TIMER0_IRQHandler(void)
{ 

  /* Store the interrupt flags before clearing */
  uint16_t intFlags = TIMER_IntGet(TIMER0);
  
  /* Clear the interrupt flags. Only clear the flags that were stored in */
  /* intFlags in case other flags have been set since then */
  TIMER_IntClear(TIMER0, intFlags);

  /* Overflow interrupt occured */
  if(intFlags & TIMER_IF_OF)
  {
    SegmentLCD_Write("OVER");
  }
  
  /* Underflow interrupt occured */
  if(intFlags & TIMER_IF_UF)
  {
    SegmentLCD_Write("UNDER");
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
  
  /* Go to EM1 when not servicing interrupts */
  while(1)
  {
    EMU_EnterEM1();
  }
}
