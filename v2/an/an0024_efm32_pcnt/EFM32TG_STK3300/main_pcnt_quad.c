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
 * @brief PCNT0_IRQHandler
 * Interrupt Service Routine for PCNT0 Interrupt Line
 *****************************************************************************/
void PCNT0_IRQHandler(void)
{   
  /* Clear direction change interrupt flag */
  PCNT_IntClear(PCNT0, PCNT_IF_DIRCNG); //0x2
  
  /* Check the direction bit to write direction */
  if(PCNT0->STATUS & PCNT_STATUS_DIR)
    SegmentLCD_Write("DOWN");
  else
    SegmentLCD_Write("UP");
    
  /* Update the number of pulses on LCD */
  SegmentLCD_Number(PCNT_CounterGet(PCNT0));
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
  CMU_ClockEnable(cmuClock_PCNT0, true);      /* Enable clock for PCNT module */
  
  /* Configure PC4 and PC5 as pushpull with output high 
     These pins are located next to the input pins for
     the PCNT, so the user can make contact between
     the pins simulating pulses */
  GPIO_PinModeSet(gpioPortC, 4, gpioModePushPull, 1);
  GPIO_PinModeSet(gpioPortC, 5, gpioModePushPull, 1);
  
  /* Configure PC13 and PC14 as inputs to drive pulse counter */
  GPIO_PinModeSet(gpioPortC, 13, gpioModeInputPull, 0);
  GPIO_PinModeSet(gpioPortC, 14, gpioModeInputPull, 0);
  
  /* Set configuration for pulse counter */
  PCNT_Init_TypeDef pcntInit =
  {
    .mode       = pcntModeExtQuad,    /* clocked by PCNT0_S0IN */
    .counter    = 0,                  /* Set initial value to 0 */
    .top        = 10,                 /* Set top to max value */
    .negEdge    = false,              /* positive edges */
    .countDown  = false,              /* up count */
    .filter     = true,               /* filter enabled */
  };
  
  /* Initialize Pulse Counter */
  PCNT_Init(PCNT0, &pcntInit);

  /* Enable PCNT direction change interrupt */
  PCNT_IntEnable(PCNT0, PCNT_IF_DIRCNG); 
  
  /* Enable PCNT0 interrupt vector in NVIC */
  NVIC_EnableIRQ(PCNT0_IRQn);
  
  /* Route PCNT0 input to location 0 -> PCNT0_S0IN on PC13 
     and PCNT0_S1IN in PC14 */  
  PCNT0->ROUTE = PCNT_ROUTE_LOCATION_LOC0;
  
  /* Initialize LCD without boost */
  SegmentLCD_Init(false);
  
  /* Write Start and initial CNT value on LCD */
  SegmentLCD_Write("START");
  SegmentLCD_Number(PCNT_CounterGet(PCNT0));
  
  while(1)
  {
    /* Go to EM2 */
    EMU_EnterEM2(false);
  }
 
}
