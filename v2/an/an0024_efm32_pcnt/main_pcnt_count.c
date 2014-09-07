/**************************************************************************//**
 * @file main_pcnt_count.c
 * @brief Pulse Counter counting up example 
 * @author Silicon Labs
 * @version 1.08
 ******************************************************************************
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
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_lcd.h"
#include "em_gpio.h"
#include "em_pcnt.h"
#include "em_system.h"
#include "segmentlcd.h"

/**************************************************************************//**
 * @brief PCNT1_IRQHandler
 * Interrupt Service Routine for PCNT1 Interrupt Line
 *****************************************************************************/
void PCNT1_IRQHandler(void)
{   
  /* Clear PCNT1 overflow interrupt flag */
  PCNT_IntClear(PCNT1, PCNT_IF_OF);
  
  /* Update the number of pulses on LCD 
     The value of TOP is written instead of 
     count because the overflow occurs when
     CNT goes from TOP to 0 */
  SegmentLCD_Number(PCNT_TopGet(PCNT1));  
  
  /* if the TOP value is still under 250 (max is 255) */
  if(PCNT_TopGet(PCNT1)<250)
  {
    /* Increase the TOP value by 10 and counter gets the old TOP
       value because the current number is 0 (overflowed)*/
    PCNT_CounterTopSet(PCNT1, PCNT_TopGet(PCNT1), PCNT_TopGet(PCNT1)+10);
  }
  else
  {
    /* If the TOP goes over 250 (max 256) put it back to 10 and 
       counter back to 0 */
    PCNT_CounterTopSet(PCNT1, 0, 10);
  }
}

/**************************************************************************//**
 * @brief  Main function
 * Main is called from __iar_program_start, see assembly startup file
 *****************************************************************************/
int main(void)
{
  /* Align different chip revisions */
  CHIP_Init();
  
  /* Select LFRCO as clock source for LFA */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);
  
  /* Enabling all necessary clocks */
  CMU_ClockEnable(cmuClock_CORELE, true);     /* Enable CORELE clock */
  CMU_ClockEnable(cmuClock_GPIO, true);       /* Enable clock for GPIO module */
  CMU_ClockEnable(cmuClock_PCNT1, true);      /* Enable clock for PCNT module */
  
  /* Configure PC5 as pushpull with output high 
     This pin is located next to the input pin for
     the PCNT, so the user can make contact between
     the two pins to generate pulses */
  GPIO_PinModeSet(gpioPortC, 5, gpioModePushPull, 1);
  
  /* Configure PC4 as input to drive pulse counter */
  GPIO_PinModeSet(gpioPortC, 4, gpioModeInputPull, 0);
  
  /* Set configuration for pulse counter */
  PCNT_Init_TypeDef pcntInit =
  {
    .mode       = pcntModeOvsSingle,  /* clocked by LFACLK */
    .counter    = 0,                  /* Set initial value to 0 */
    .top        = 10,                 /* Set top to max value */
    .negEdge    = false,              /* positive edges */
    .countDown  = false,              /* up count */
    .filter     = true,               /* filter enabled */
  };
  
  /* Initialize Pulse Counter */
  PCNT_Init(PCNT1, &pcntInit);

  /* Enable PCNT overflow interrupt */
  PCNT_IntEnable(PCNT1, PCNT_IF_OF);
  
  /* Enable PCNT1 interrupt vector in NVIC */
  NVIC_EnableIRQ(PCNT1_IRQn);
  
  /* Route PCNT1 input to location 0 -> PCNT1_S0IN on PC4 */  
  PCNT1->ROUTE = PCNT_ROUTE_LOCATION_LOC0;
  
  /* Initialize LCD without boost */
  SegmentLCD_Init(false);
  
  /* Write UP (configured direction) and initial CNT value on LCD */
  SegmentLCD_Write("UP");
  SegmentLCD_Number(PCNT_CounterGet(PCNT1));
  
  while(1)
  {
    /* Go to EM2 */
    EMU_EnterEM2(false);
  }
 
}
