/******************************************************************************
 * @file 4_energymodes.c
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

#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_rtc.h"
#include "em_lcd.h"
#include "segmentlcd.h"
#include "em_chip.h"

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
  
  /* ENTER YOUR CODE HERE */
}

/******************************************************************************
 * @brief  Main function
 * Main is called from _program_start, see assembly startup file
 *****************************************************************************/
int main(void)
{ 
  /* Initialize chip */
  CHIP_Init();  
  
  /* ENTER YOUR CODE HERE */

  /* Starting LFRCO and waiting until it is stable */
 
  /* Enable the RTC clock*/
  
  /* Enabling clock to the interface of the low energy modules */
  
  
  /* Set up RTC init struct*/
  //rtcInit.debugRun = ?;
  //rtcInit.comp0Top = ?;
  //rtcInit.enable   = ?;
  
  /* Input RTC init struct in initialize funciton */

  /* Set RTC compare value */
  
  /* Enable RTC interrupt from COMP0 */
  
  /* Enable RTC interrupt vector in NVIC */
  
  /* Enable RTC */

  /* Initialize LCD without boost */
 
  /* Write to LCD */
    
  /* Enter EM2 and wait for RTC interrupt */

  /* Wait in this loop at end of program */
  while(1);
}



