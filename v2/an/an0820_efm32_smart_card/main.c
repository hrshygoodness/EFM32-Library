/**************************************************************************//**
 * @file main.c
 * @brief USBCCID USB device example.
 * @author Silicon Labs
 * @version 1.01
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/


#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_usb.h"
#include "segmentlcd.h"
#include "usart_driver.h"
#include "sc_driver.h"
#include "usb_ccid_device.h"
#include "swo_debug.h"

/**************************************************************************//**
 *
 * This example shows how a ccid usb device can be implemented.
 *
 *****************************************************************************/

/**************************************************************************//**
 * @brief main - the entrypoint after reset.
 *****************************************************************************/
int main(void)
{
    
  /* Chip errata */
  CHIP_Init();

  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
  SegmentLCD_Init(false);
  SegmentLCD_Write("usbccid");
  SegmentLCD_Symbol(LCD_SYMBOL_GECKO, true);

  /* For debug printf's */
  SWO_Setup();
  
  
  CCID_Init();
       
  for (;;)
  {
    CCID_Handler();
    USBTIMER_DelayMs( 10 );
  }
  
  
}
