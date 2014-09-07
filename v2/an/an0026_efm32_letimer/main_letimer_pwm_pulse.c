/*****************************************************************************
 * @file main_letimer_pwm_pulse.c
 * @brief LETIMER demo application, PWM and pulse output
 * @author Silicon Labs
 * @version 1.07
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
#include "em_cmu.h"
#include "em_emu.h"
#include "em_letimer.h"
#include "em_gpio.h"
#include "em_chip.h"

/* Maximum value for compare registers */
#define COMPMAX 500

/* COMP1 is changed throughout the code to vary the PWM duty cycle 
   but starts with the maximum value (100% duty cycle) */
uint16_t comp1 = COMPMAX;

/**************************************************************************//**
 * @brief LETIMER0_IRQHandler
 * Interrupt Service Routine for LETIMER
 *****************************************************************************/
void LETIMER0_IRQHandler(void)
{ 
  /* Clear LETIMER0 underflow interrupt flag */
  LETIMER_IntClear(LETIMER0, LETIMER_IF_UF);

  /* If the value of comp1 is over 0, decrement it 
     or bring comp1 back to COMPMAX (500) */
  if(comp1 != 0) 
    comp1--;
  else
    comp1 = COMPMAX;
  
  /* Write the new compare value to COMP1 */
  LETIMER_CompareSet(LETIMER0, 1, comp1);
}

/**************************************************************************//**
 * @brief  LETIMER_setup
 * Configures and starts the LETIMER0
 *****************************************************************************/
void LETIMER_setup(void)
{
  /* Enable necessary clocks */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
  CMU_ClockEnable(cmuClock_CORELE, true);
  CMU_ClockEnable(cmuClock_LETIMER0, true);  
  CMU_ClockEnable(cmuClock_GPIO, true);
  
  /* Configure PD6 and PD7 as push pull so the
     LETIMER can override them */
  GPIO_PinModeSet(gpioPortD, 6, gpioModePushPull, 0);
  GPIO_PinModeSet(gpioPortD, 7, gpioModePushPull, 0);
  
  /* Set initial compare values for COMP0 and COMP1 
     COMP1 keeps it's value and is used as TOP value
     for the LETIMER.
     COMP1 gets decremented through the program execution
     to generate a different PWM duty cycle */
  LETIMER_CompareSet(LETIMER0, 0, COMPMAX);
  LETIMER_CompareSet(LETIMER0, 1, COMPMAX);
  
  /* Repetition values must be nonzero so that the outputs
     return switch between idle and active state */
  LETIMER_RepeatSet(LETIMER0, 0, 0x01);
  LETIMER_RepeatSet(LETIMER0, 1, 0x01);
  
  /* Route LETIMER to location 0 (PD6 and PD7) and enable outputs */
  LETIMER0->ROUTE = LETIMER_ROUTE_OUT0PEN | LETIMER_ROUTE_OUT1PEN | LETIMER_ROUTE_LOCATION_LOC0;
  
  /* Set configurations for LETIMER 0 */
  const LETIMER_Init_TypeDef letimerInit = 
  {
  .enable         = true,                   /* Start counting when init completed. */
  .debugRun       = false,                  /* Counter shall not keep running during debug halt. */
  .rtcComp0Enable = false,                  /* Don't start counting on RTC COMP0 match. */
  .rtcComp1Enable = false,                  /* Don't start counting on RTC COMP1 match. */
  .comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP0 is used as TOP */
  .bufTop         = false,                  /* Don't load COMP1 into COMP0 when REP0 reaches 0. */
  .out0Pol        = 0,                      /* Idle value for output 0. */
  .out1Pol        = 0,                      /* Idle value for output 1. */
  .ufoa0          = letimerUFOAPwm,         /* PWM output on output 0 */
  .ufoa1          = letimerUFOAPulse,       /* Pulse output on output 1*/
  .repMode        = letimerRepeatFree       /* Count until stopped */
  };
  
  /* Initialize LETIMER */
  LETIMER_Init(LETIMER0, &letimerInit); 
}



/**************************************************************************//**
 * @brief  Main function
 * Main is called from __iar_program_start, see assembly startup file
 *****************************************************************************/
int main(void)
{  
  /* Align different chip revisions */
  CHIP_Init();

  /* Initialize LETIMER */
  LETIMER_setup();

  //-----------------------------------------------------------------------------------
  // THE INTERRUPT IS SIMPLY TO DECREASE THE VALUE OF COMP1 TO VARY THE PWM DUTY CYCLE
  //-----------------------------------------------------------------------------------
  /* Enable underflow interrupt */  
  LETIMER_IntEnable(LETIMER0, LETIMER_IF_UF);  
  
  /* Enable LETIMER0 interrupt vector in NVIC*/
  NVIC_EnableIRQ(LETIMER0_IRQn);
  
  while(1)
  {
    /* Go to EM2 */
    EMU_EnterEM2(false);
  }
}
