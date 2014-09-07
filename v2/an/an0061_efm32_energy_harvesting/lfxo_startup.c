/******************************************************************************
 * @file lfxo_startup.c
 * @brief LFXO Startup in EM2
 * @author Silicon Labs
 * @version 1.03
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
#include "em_emu.h"
#include "em_cmu.h"
#include "em_rmu.h"
#include "em_rtc.h"


/* Function prototypes */
void startRTCTick(void);
void stopRTCTick(void);

/* Defines */
#define MS_TO_SLEEP  100 /* The number of ms to sleep between polling for LFXO startup */


/******************************************************************************
 * @brief  
 *   Main function
 *****************************************************************************/
int main(void)
{
  /* Initialize chip - handle erratas */
  CHIP_Init();

  /* Setup RTC for periodic wake-ups */
  startRTCTick();

  /* Start LFXO. Do not wait until it is stable */
  CMU_OscillatorEnable(cmuOsc_LFXO, true, false);

  /* Wait in EM2 until LFXO is ready */
  while (!(CMU->STATUS & CMU_STATUS_LFXORDY))
  {
    EMU_EnterEM2(false);
  }
  /* Stop the RTC */
  stopRTCTick();

  while (1)
  {
    /* Go to sleep */
    EMU_EnterEM1();
  }
}



/******************************************************************************
 * @brief 
 *   RTC Irq Handler. Clears the COMP0 flag
 *****************************************************************************/
void RTC_IRQHandler(void)
{
  RTC_IntClear( RTC_IF_COMP0 );
}




/******************************************************************************
 * @brief 
 *   Configures and starts RTC from ULFRCO
 *****************************************************************************/
void startRTCTick(void)
{
  /* Set-up RTC init struct */
  RTC_Init_TypeDef rtcInit;
  rtcInit.comp0Top = true;
  rtcInit.debugRun = false;
  rtcInit.enable = true;
  
  /* Enable clock to LF peripherals */
  CMU_ClockEnable(cmuClock_CORELE, true);

  /* Set ULFRCO as clock source for RTC */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);

  /* Turn on clock for RTC */
  CMU_ClockEnable(cmuClock_RTC, true);

  /* Enable RTC interrupt lines */
  NVIC_EnableIRQ(RTC_IRQn);
  RTC_IntEnable(RTC_IF_COMP0);

  /* Set compare register */
  RTC_CompareSet(0, MS_TO_SLEEP);

  /* Initialize and start RTC */
  RTC_Init(&rtcInit);
}



/******************************************************************************
 * @brief 
 *   Stops RTC. Turns off clocks and interrupts 
 *****************************************************************************/
void stopRTCTick(void)
{
  RTC_Enable(false);

  RTC->CNT = 0;

  /* Turn off clock for RTC */
  CMU_ClockEnable(cmuClock_RTC, false);

  /* Disable clock to LF peripherals */
  CMU_ClockEnable(cmuClock_CORELE, false);


  /* Disable RTC interrupts */
  NVIC_DisableIRQ(RTC_IRQn);
  RTC_IntDisable(RTC_IF_COMP0);
}

