/******************************************************************************
 * @file 4_energymodes_zg_solution.c
 * @brief Energy Modes example
 * @author Silicon Labs
 * @version 1.18
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

#include "em_chip.h"
#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_rtc.h"

#define LFRCO_FREQUENCY              32768
#define WAKEUP_INTERVAL_MS           5000
#define RTC_COUNT_BETWEEN_WAKEUP    ((LFRCO_FREQUENCY * WAKEUP_INTERVAL_MS) / 1000)

RTC_Init_TypeDef rtcInit;

/******************************************************************************
 * @brief RTC Interrupt Handler. Clears interrupt flag.
 *        The interrupt table is in assembly startup file startup_efm32.s
 *****************************************************************************/
void RTC_IRQHandler(void)
{ 
  /* Clear interrupt source */
  RTC_IntClear(RTC_IFC_COMP0);
}

/******************************************************************************
 * @brief  Main function
 *
 *****************************************************************************/
int main(void)
{ 
  /* Starting LFRCO and waiting until it is stable */
  CMU_OscillatorEnable(cmuOsc_LFRCO, true, true);

  /* Enable the RTC clock*/
  CMU_ClockEnable(cmuClock_RTC, true);

  /* Enabling clock to the interface of the low energy modules */
  CMU_ClockEnable(cmuClock_CORELE, true);
  
  /* Set up RTC init struct*/
  rtcInit.debugRun = false;
  rtcInit.comp0Top = true;
  rtcInit.enable   = true;
  
  /* Input RTC init struct in initialize funciton */
  RTC_Init(&rtcInit);
  
  /* Set RTC compare value */
  RTC_CompareSet(0, RTC_COUNT_BETWEEN_WAKEUP);
  
  /* Enable RTC interrupt from COMP0 */
  RTC_IntEnable(RTC_IF_COMP0);
  
  /* Enable RTC interrupt vector in NVIC */
  NVIC_EnableIRQ(RTC_IRQn);
  
  /* Enable RTC */
  RTC_Enable(true);

   
  /* Loop between EM2 and EM1 */
  while(1){
    /* Enter EM2 and wait for RTC interrupt */
    EMU_EnterEM2(false);
    
    /* Enter EM1 and wait for RTC interrupt */
    EMU_EnterEM1();
  }
}



