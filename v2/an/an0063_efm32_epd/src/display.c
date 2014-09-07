/*****************************************************************************
 * @file display.c
 * @brief Contains functions for displaying frames on EPD
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
#include "em_device.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "delay.h"
#include "pwm.h"
#include "spi.h"
#include "cog.h"
#include "frames.h"
#include "config.h"
#include "display.h"


/* The next frame to be rendered to display */
uint8_t newFrame[FRAME_SIZE];

/* The previus frame shown on the display */
uint8_t oldFrame[FRAME_SIZE];

/* Storage for the frame to be sent to the COG driver
 * in the format expected by the COG */
uint8_t epdFrame[PANEL_FRAME_SIZE];

/* The frame buffer used by emWin */
uint8_t emwinFrameBuffer[EMWIN_FRAME_SIZE];


/**********************************************************
 * Sets up pins and peripherals needed to drive the EPD
 **********************************************************/
void epdInit(void)
{
  pwmInit();
  
  spiInit();
  
  delayInit();
  
  GPIO_PinModeSet(EPD_PIN_PANEL_ON, gpioModePushPull, 0);
  GPIO_PinModeSet(EPD_PIN_DISCHARGE, gpioModePushPull, 0);
  GPIO_PinModeSet(EPD_PIN_RESET, gpioModePushPull, 0);
  GPIO_PinModeSet(EPD_PIN_BORDER, gpioModePushPull, 1);
  GPIO_PinModeSet(EPD_PIN_PANEL_VDD, gpioModePushPull, 0);
}

/**********************************************************
 * Copies pixels from the emWin frame buffer to the 
 * frame buffer used to update the display. The emWin
 * frame usually have to be slightly larger than the 
 * dimensions of the panel (because of a bug in emWin
 * the display dimension are required to be divisible by 
 * 32). Therefore this function basically performs clipping
 * and only copies the pixels that will actually 
 * fit in the display. 
 **********************************************************/
void copyFromEmwinBuffer(void)
{
  int a, b;
  int i,j;
  
  a = 0;
  b = 0;
  
  for ( j=0; j<PANEL_HEIGHT; j++ )
  {
    for ( i=0; i<EMWIN_SIZE(PANEL_WIDTH)/8; i++ )
    {
      if ( i < PANEL_WIDTH/8 )
      {
        newFrame[a++] = emwinFrameBuffer[b];
      }
      b++;
    }
  }
}

/**********************************************************
 * Displays the current frame buffer on the EPD panel. 
 * This function will copy the current emWin frame buffer, 
 * power up the EPD panel, transmit the image and 
 * power down the panel again. It also saves the
 * frame in a separate buffer, since the old image
 * is used by the update process to get the best
 * possible contrast and minimal ghosting. 
 **********************************************************/
void showImage(void)
{
  int i;

  /* Copy the current emWin buffer */
  copyFromEmwinBuffer();
  
  /* Power up the panel and COG driver */
  cogPowerUp();
  
  /* Initialize the COG */
  cogInit();
  
  /* Loop through the 4 stages and transmit each frame
   * multiple times to achieve an acceptable contrast
   * and ghosting level. */
  for ( i=1; i<=4; i++ )
  {  
    /* Run off HFXO while generating frame. Allows it to complete faster
     * and thus saves power */
    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
    
    /* Generate an EPD frame from the old and new frame buffers and current stage */
    generateFrame(oldFrame, newFrame, epdFrame, i);
    
    /* Run off HFRCO while transmitting. This step is timed so there 
     * is no reason to run off a faster clock */
    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);
    
    /* Send the EPD frame to the COG */
    updateStage(epdFrame);
  }
  
  /* Power down the COG and panel */
  cogPowerOff();

  /* Keep the newly transmitted frame. It is used
   * for the next panel update. */
  memcpy(oldFrame, newFrame, FRAME_SIZE);
}
