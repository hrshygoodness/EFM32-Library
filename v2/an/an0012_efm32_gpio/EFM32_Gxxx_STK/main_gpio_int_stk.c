/*****************************************************************************
 * @file main_gpio_int_stk.c
 * @brief GPIO Demo Application
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
#include "em_gpio.h"
#include "em_chip.h"

/**************************************************************************//**
 * @brief GPIO_ODD_IRQHandler
 * Interrupt Service Routine Odd GPIO Interrupt Line
 *****************************************************************************/
void GPIO_ODD_IRQHandler(void)
{ 
  /* clear flag for PC9 interrupt */
  GPIO_IntClear(0x0200);
}
/**************************************************************************//**
 * @brief  Main function
 * Main is called from __iar_program_start, see assembly startup file
 *****************************************************************************/
int main(void)
{  
  /* Initialize chip */
  CHIP_Init();
  
  /* Enable clock for GPIO module */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Configure PC with alternate drive strength of 0.5mA */
  GPIO_DriveModeSet(gpioPortC, gpioDriveModeLowest);
  
  /* Configure PB9 as an input for PB0 button with filter enable (out = 1)*/
  GPIO_PinModeSet(gpioPortB, 9, gpioModeInput, 1);
  
  /* Configure PC0 as a push pull for LED drive */
  GPIO_PinModeSet(gpioPortC, 0, gpioModePushPull, 0);
  
  /* Configure PC1 as a push pull with alternate
  strength for LED drive */
  GPIO_PinModeSet(gpioPortC, 1, gpioModePushPullDrive, 0);  
  
  /* Enable GPIO_ODD interrupt vector in NVIC */
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
  
  /* Configure PB9 interrupt on falling edge */
  GPIO_IntConfig(gpioPortB, 9, false, true, true);
  
  while (1)
  {
    /* Go to EM3 */
    EMU_EnterEM3(false);
    /* Wake up from interrupt and drive LEDs */
    GPIO_PortOutSet(gpioPortC, 0x3);
    /* Continue driving while button is pressed */
    while(!GPIO_PinInGet(gpioPortB, 9));
    /* When button unpressed, stop driving LEDs
    and go back to EM3 */
    GPIO_PortOutClear(gpioPortC, 0x3); 
  }
}
