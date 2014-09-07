/******************************************************************************
 * @file
 * @brief Register Operation Example
 * @author Energy Micro AS
 * @version x.xx
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2009 Energy Micro AS, http://www.energymicro.com</b>
 ******************************************************************************
 *
 * This source code is the property of Energy Micro AS. The source and compiled
 * code may only be used on Energy Micro "EFM32" microcontrollers.
 *
 * This copyright notice may not be removed from the source code nor changed.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 *****************************************************************************/
#include "efm32.h"
#include "em_chip.h"

/******************************************************************************
 * @brief  Main function
 * Main is called from _program_start, see assembly startup file
 *****************************************************************************/
int main(void)
{  
  /* Initialize chip */
  CHIP_Init();  
  
  /* Write a program that waits for 1000 cycles using the timer, before it 
  continues to the while(1)-loop at the end of the main function. */
  
  /* Enable clock for TIMER0 */
  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_TIMER0;
  
  /* Start TIMER0 */
  TIMER0->CMD = TIMER_CMD_START;
  
  /* Wait until counter value is over 1000 */
  while(TIMER0->CNT < 1000){
   /* Do nothing, just wait */ 
  }

  /* Stay in this loop forever at end of program */
  while (1) ;
}



