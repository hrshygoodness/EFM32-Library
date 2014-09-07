/***************************************************************************//**
 * @file interrupt_disable.c
 * @brief Interrupt Disable Example for EFM32.
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
#include <stdbool.h>
#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_system.h"
#include "em_chip.h"
#include "em_int.h"

volatile bool lfrcoReady = false;

/***************************************************************************//**
 * @brief CMU Interrupt handler.
 ******************************************************************************/
void CMU_IRQHandler(void)
{ 
  /* Clear interrupt flag */
  CMU_IntClear(CMU_IF_LFRCORDY);

  /* Indicate that LFRCO is ready */
  lfrcoReady = true;
}


/***************************************************************************//**
 * @brief Main function. Enables LFRCO and waits in EM1 until it is ready
 ******************************************************************************/
int main(void)
{ 
  /* Chip revision alignment and errata fixes */
  CHIP_Init();
  
  /* Enable CMU IRQ when LFRCO is ready */
  CMU_IntEnable(CMU_IF_LFRCORDY);
  
  /* Enable CMU interrupt vector in NVIC */
  NVIC_EnableIRQ(CMU_IRQn);
  
  /* Enable LFRCO but do not wait until it is ready */
  CMU_OscillatorEnable(cmuOsc_LFRCO, true, false);
  
  /* Wait in EM1 until LFRCO is ready.
   * Disable interrupts first to avoid interrupt executing between lfrcoReady
   * check and _WFI(); This would have caused the program to get stuck! */
  INT_Disable(); 
  while(!lfrcoReady)
  {
    __WFI(); /* Pending and enabled IRQs will wake up the CPU, but not go to ISR */
    INT_Enable(); /* ISR for any pending and enabled IRQs will be executed after this */
  }
  
  /* Wait here at the end */
  while(1);     
}
