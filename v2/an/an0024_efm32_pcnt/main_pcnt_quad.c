/**************************************************************************//**
 * @file main_pcnt_quad.c
 * @brief Pulse Counter quadrature decoding example 
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
  /* Clear direction change interrupt flag */
  PCNT_IntClear(PCNT1, PCNT_IF_DIRCNG);
  
  /* Check the direction bit to write direction */
  if(PCNT1->STATUS & PCNT_STATUS_DIR)
    SegmentLCD_Write("DOWN");
  else
    SegmentLCD_Write("UP");
    
  /* Update the number of pulses on LCD */
  SegmentLCD_Number(PCNT_CounterGet(PCNT1));
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
  
  /* Configure PC6 and PC7 as pushpull with output high 
     These pins are located next to the input pins for
     the PCNT, so the user can make contact between
     the pins simulating pulses */
  GPIO_PinModeSet(gpioPortC, 6, gpioModePushPull, 1);
  GPIO_PinModeSet(gpioPortC, 7, gpioModePushPull, 1);
  
  /* Configure PC4 and PC5 as inputs to drive pulse counter */
  GPIO_PinModeSet(gpioPortC, 4, gpioModeInputPull, 0);
  GPIO_PinModeSet(gpioPortC, 5, gpioModeInputPull, 0);
  
  /* Set configuration for pulse counter */
  PCNT_Init_TypeDef pcntInit =
  {
    .mode       = pcntModeExtQuad,    /* clocked by PCNT1_S0IN */
    .counter    = 0,                  /* Set initial value to 0 */
    .top        = 10,                 /* Set top to max value */
    .negEdge    = false,              /* positive edges */
    .countDown  = false,              /* up count */
    .filter     = true,               /* filter enabled */
  };
  
  /* Initialize Pulse Counter */
  PCNT_Init(PCNT1, &pcntInit);

  /* Enable PCNT direction change interrupt */
  PCNT_IntEnable(PCNT1, PCNT_IF_DIRCNG);
  
  /* Enable PCNT1 interrupt vector in NVIC */
  NVIC_EnableIRQ(PCNT1_IRQn);
  
  /* Route PCNT1 input to location 0 -> PCNT1_S0IN on PC4 
     and PCNT1_S1IN in PC5 */  
  PCNT1->ROUTE = PCNT_ROUTE_LOCATION_LOC0;
  
  /* Initialize LCD without boost */
  SegmentLCD_Init(false);
  
  /* Write Start and initial CNT value on LCD */
  SegmentLCD_Write("START");
  SegmentLCD_Number(PCNT_CounterGet(PCNT1));
  
  while(1)
  {
    /* Go to EM2 */
    EMU_EnterEM2(false);
  }
 
}
