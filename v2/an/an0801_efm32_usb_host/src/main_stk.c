/**************************************************************************//**
 * @file main_stk.c
 * @brief EFM32 USB Host Example
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

/**********************************************************
 * 
 * USB Host Session Example
 * 
 * This example shows how to do a generic USB session
 * as a USB Host. The application will sleep in EM3
 * until the user presses the PB1 push button (on DK/STK). 
 * 
 * The application will then try to connect to a USB
 * device. It expects the USB device as defined in 
 * application note AN0065 which is another EFM32
 * acting as a generic USB Device. 
 * 
 * One IN and one OUT pipe (channel) is defined. 
 * The Host will periodically send a message on the OUT
 * pipe, while listening for any messages on the IN pipe. 
 * 
 * To visualize the communication a UART adapter should be 
 * connected to the RS-232 port of the DK or to pins PD0/PD1
 * (TX/RX) on the STK. Both the application and USB Stack itself
 * will output debug information over UART. 
 * 
 * 
 **********************************************************/

#include <string.h>
#include <stdio.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_usb.h"
#include "em_gpio.h"
#include "em_emu.h"
#include "retargetserial.h"
#include "usb_host.h"
#include "rtc.h"
#include "segmentlcd.h"


/* This flag is used to start a USB session. The flag is set in 
 * the GPIO interrupt handler. */
volatile bool doTryToConnect = false;

/**********************************************************
 * Initialize debug output over UART or TFT
 *********************************************************/
static void debugInit(void)
{  
  /* Initialize debugging output  */
  RETARGET_SerialInit();                        
  
  /* Map LF to CRLF */
  RETARGET_SerialCrLf(1);                       
}

/**********************************************************
 * Configures the GPIO interrupt
 **********************************************************/
void gpioInit(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  
  GPIO_PinModeSet(gpioPortB, 10, gpioModeInputPull, 1);
  GPIO_IntConfig(gpioPortB, 10, false, true, true);
  
  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}

/**********************************************************
 * GPIO Interrupt Handler. If the correct button (PB1) is
 * pressed, enables the doTryToConnect flag.  
 **********************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  /* Get and clear flags */
  uint32_t flags = GPIO->IF;
  GPIO->IFC = flags;
  
  if ( flags & (1 << 10) )  /* Interrupt on PB10 */
  {
    doTryToConnect = true;
  }
}

/**********************************************************
 * GPIO Interrupt Handler. Is triggered by a positive edge 
 * transition on the D+ pin while USB is disabled. If a 
 * USB Device was inserted (PF11 was pulled high), 
 * schedule a connection attempt. 
 **********************************************************/
void GPIO_ODD_IRQHandler(void)
{
  GPIO->IFC = 1 << 11;
  
  doTryToConnect = true;
}


/**********************************************************
 * Enable or disable sensing for if a USB Device is 
 * attached. Sensing is done by measuring the state
 * of the D+ pin. Any connected (full-speed) device
 * will have a strong pull-up on the D+ pin. 
 * This function configures D+ with a weak pull-down
 * and enables interrupts for this pin. If a USB 
 * Device is connected it will pull the pin high. 
 * 
 * @param senseOn
 *   If true, enable sensing
 **********************************************************/
void enableSenseUsb(bool senseOn)
{   
  if ( senseOn )
  {
    /* Enable VBUS. Note that this requires that this requries
     * a 5V domain to be present on the board and connected to
     * a switch controlled by the VBUSEN pin. Disable USB 
     * control of the VBUSEN pin and set it by GPIO to allow
     * control of the pin in EM2/EM3 with USB disabled. */
    USB->ROUTE &= ~USB_ROUTE_VBUSENPEN;
    GPIO_PinModeSet( gpioPortF, 5, gpioModePushPull, 1 );
    
    /* Enable interrupt on D+ pin (PF11) */
    GPIO_PinModeSet(gpioPortF, 11, gpioModeInputPull, 0);
    GPIO_IntConfig(gpioPortF, 11, true, false, true);
    
    /* Insert a delay to avoid waking up on an already
     * attached device. */
    rtcDelayMs(100);
    
    GPIO->IFC = 1 << 11;
    NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
    NVIC_EnableIRQ(GPIO_ODD_IRQn);
  }
  else
  {
    /* Disable GPIO control and interrupt on D+ pin (PF11) */
    NVIC_DisableIRQ(GPIO_ODD_IRQn);
    GPIO_PinModeSet(gpioPortF, 11, gpioModeDisabled, 0);
    GPIO_IntConfig(gpioPortF, 11, false, false, false);

    /* Disable VBUS */
    GPIO_PinModeSet( gpioPortF, 5, gpioModePushPull, 0 );
  }
}


int main(void)
{
  /* Select 48 MHz HFXO for HFCLK. Needed for USB. */
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
   
  /* Enable UART debug output. Debug output from the USB API
   * is enabled in usbconfig.h */
  debugInit();
  
  /* Set up GPIO interrupt. Used to wake the MCU and start
   * a USB session. */
  gpioInit();
  
  SegmentLCD_Init(false);
  SegmentLCD_Write("Idle");
  
  printf("\nEFM32 USB Host Example.");
  printf("\nPress PB1 to connect");
    
  /* Whenever the user presses the PB1 button 
   * or inserts a USB cable a USB session is attempted */
  while (1)
  {
    
    if ( doTryToConnect )
    {
      /* Disable USB Device sensing */
      enableSenseUsb(false);
      
      /* Reset flag */
      doTryToConnect = false;
      
      /* Try to connect to device */
      if ( usbConnect() )
      {
        /* Connected sucessfully */
        SegmentLCD_Write("Connctd");
        SegmentLCD_Symbol(LCD_SYMBOL_ANT, 1);
        
        /* Start a USB session */
        if ( !usbDoSession() ) 
        {
          printf("\nError occured during USB session");
        }
      }
            
      /* Shut down USB and inform user */
      usbShutDown();
      SegmentLCD_Write("Idle");
      SegmentLCD_Symbol(LCD_SYMBOL_ANT, 0);
      printf("\nPress PB1 to connect");
    }
    else
    {
      /* Sleep in EM2 to save power. Enable sensing
       * to wake up if a device is inserted. */
      enableSenseUsb(true);
      EMU_EnterEM2(true);
    }
  }
}

