/**************************************************************************//**
 * @file main_stk.c
 * @brief USB Device Example
 * @author Silicon Labs
 * @version 1.02
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
#include <string.h>
#include <stdio.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_usb.h"
#include "em_gpio.h"
#include "em_emu.h"
#include "callbacks.h"
#include "descriptors.h"
#include "retargetserial.h"
#include "segmentlcd.h"

/* Messages to send when the user presses buttons on the kit */
EFM32_ALIGN(4)
uint8_t button0message[] = "PB0 pressed!";
EFM32_ALIGN(4)
uint8_t button1message[] = "PB1 pressed!";

/**********************************************************
 * Enable GPIO interrupts on both push buttons on the STK
 **********************************************************/
void gpioInit(void)
{
  /* Enable clock to GPIO */
  CMU_ClockEnable(cmuClock_GPIO, true);
   
  /* Enable interrupt on push button 0 */
  GPIO_PinModeSet(gpioPortB, 9, gpioModeInput, 0);
  GPIO_IntConfig(gpioPortB, 9, false, true, true);
  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
  
  /* Enable interrupt on push button 1 */
  GPIO_PinModeSet(gpioPortB, 10, gpioModeInput, 0);
  GPIO_IntConfig(gpioPortB, 10, false, true, true);
  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}   


int main(void)
{
  /* Chip errata */
  CHIP_Init();
  
  /* Enable HFXO */
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
    
  /* Enable deboug output over UART */
  RETARGET_SerialInit();                     
  RETARGET_SerialCrLf(1); 
  
  /* Enable the segment LCD */
  SegmentLCD_Init(false);
  SegmentLCD_Write("USB");
  
  printf("\nStarting USB Device...\n");
  
  /* Set up GPIO interrupts */
  gpioInit();
  
  /* Start USB stack. Callback routines in callbacks.c will be called
   * when connected to a host.  */
  USBD_Init(&initstruct);;

  /*
   * When using a debugger it is pratical to uncomment the following three
   * lines to force host to re-enumerate the device.
   */
  /* USBD_Disconnect(); */
  /* USBTIMER_DelayMs( 1000 ); */
  /* USBD_Connect(); */
    
  while(1)
  {
    if ( USBD_SafeToEnterEM2() )
    {
      /* Enter EM2 when in suspend or disconnected */
      EMU_EnterEM2(true);
    } 
    else
    {
      /* When USB is active we can sleep in EM1. */
      EMU_EnterEM1();
    }
  } 
}

/**********************************************************
 * Interrupt handler for push button 0 
 **********************************************************/
void GPIO_ODD_IRQHandler(void)
{
  /* Clear the interrupt flag */
  GPIO->IFC = 1 << 9;
  
  /* Send message */
  USBD_Write(EP_IN, button0message, sizeof(button0message), dataSentCallback);
}

/**********************************************************
 * Interrupt handler for push button 1
 **********************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  /* Clear the interrupt flag */
  GPIO->IFC = 1 << 10;
  
  /* Send message */
  USBD_Write(EP_IN, button1message, sizeof(button1message), dataSentCallback);
}

