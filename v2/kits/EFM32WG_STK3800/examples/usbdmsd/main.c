/**************************************************************************//**
 * @file main.c
 * @brief Mass Storage Device example.
 * @version 3.20.5
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

#include <stdio.h>
#include "em_device.h"
#include "em_usb.h"
#include "em_assert.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "retargetserial.h"
#include "msdd.h"
#include "msddmedia.h"
#include "usbconfig.h"
#include "segmentlcd.h"
#include "bsp_trace.h"

/**************************************************************************//**
 *
 * This example shows how a Mass Storage Device (MSD) can be implemented.
 *
 * Different kinds of media can be used for data storage. Modify the
 * MSD_MEDIA #define macro in msdmedia.h to select between the different ones.
 *
 *****************************************************************************/

/**************************************************************************//**
 * @brief main - the entrypoint after reset.
 *****************************************************************************/
int main( void )
{
  /* Chip errata */
  CHIP_Init();

  /* If first word of user data page is non-zero, enable eA Profiler trace */
  BSP_TraceProfilerSetup();

  CMU_ClockSelectSet( cmuClock_HF, cmuSelect_HFXO );
  CMU_OscillatorEnable(cmuOsc_LFXO, true, false);
  CMU_ClockEnable(cmuClock_GPIO, true);

  SegmentLCD_Init(false);

  if ( !MSDDMEDIA_Init() )
  {
    EFM_ASSERT( false );
    for( ;; ){}
  }

  SegmentLCD_Write("usbdmsd");
  SegmentLCD_Symbol(LCD_SYMBOL_GECKO, true);
  MSDD_Init(gpioPortE, 2);

  for (;;)
  {
    if ( MSDD_Handler() )
    {
      /* There is no pending activity in the MSDD handler.  */
      /* Enter sleep mode to conserve energy.               */

      if ( USBD_SafeToEnterEM2() )
        EMU_EnterEM2(true);
      else
        EMU_EnterEM1();
    }
  }
}
