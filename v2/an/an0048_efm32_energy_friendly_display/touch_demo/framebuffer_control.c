/**************************************************************************//**
 * @file framebuffer_control.c
 * @brief Contains functions to manipulate and retrieve the frame buffer
 * @author Silicon Labs
 * @version 1.05
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

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "GUI.h"
#include "em_device.h"


/* Size of the physical display */
#define DISPLAY_WIDTH  128
#define DISPLAY_HEIGHT 128

/* Width of the frame buffer. 4 extra bytes are added to each line. 
 * 2 bytes are added to accommodate the control signals and dummy bits
 * for the SPI protocol. The 2 last bytes are added because of a bug
 * in the current emWin version (v 5.16), which does not handle a frame buffer
 * width of 144 correctly. 
*/
#define BUFFER_VIRTUAL_WIDTH 160


/* Size defines, defined for readability in code below */

#define DISPLAY_HALFWORDS_PER_LINE     (DISPLAY_WIDTH / 16)

#define DISPLAY_BYTES_PER_LINE         (DISPLAY_HALFWORDS_PER_LINE * 2)

#define BUFFER_HALFWORDS_PER_LINE      (BUFFER_VIRTUAL_WIDTH / 16)

#define BUFFER_SIZE_HALFWORDS          (BUFFER_HALFWORDS_PER_LINE * DISPLAY_HEIGHT)

#define BUFFER_SIZE_BYTES              (BUFFER_SIZE_HALFWORDS * 2)


/* Two frame buffers are used. While transferring one buffer to the display
 * the next frame is begin drawn to the other frame buffer. */
uint16_t frameBuffer[BUFFER_SIZE_HALFWORDS];
uint16_t frameBuffer2[BUFFER_SIZE_HALFWORDS];

/* Flag indicating which frame buffer is active */
bool useBuffer1 = true;

/* Extra buffer holding the background image. Copied to the active 
 * frame buffer before the start of drawing each frame. */
uint16_t backgroundBuffer[BUFFER_SIZE_HALFWORDS];

/** Returns the address of the active frame buffer */
uint16_t *FB_getActiveBuffer(void)
{
  if ( useBuffer1 ) {
    return frameBuffer;
  } else {
    return frameBuffer2;
  }
}



uint8_t *FB_getBackgroundBuffer(void)
{
  return (uint8_t *)backgroundBuffer;
}


/** Direct emWin drawing output to the background buffer */
void FB_activateBackgroundBuffer(void)
{
  LCD_SetVRAMAddrEx(0, (void *)backgroundBuffer);
}

/** Direct emWin drawing output to the frame buffer */
void FB_activateFrameBuffer(void)
{
  if ( useBuffer1 ) {
    LCD_SetVRAMAddrEx(0, (void *)frameBuffer);
  } else {
    LCD_SetVRAMAddrEx(0, (void *)frameBuffer2);
  }
}


/** Copy the entire background buffer to the frame buffer */
void FB_copyBackgroundToFrameBuffer(void)
{
  if ( useBuffer1 ) {
    memcpy(frameBuffer, backgroundBuffer, BUFFER_SIZE_BYTES);  
  } else {
    memcpy(frameBuffer2, backgroundBuffer, BUFFER_SIZE_BYTES);  
  }
}

void FB_copyPrevBuffer(void)
{
  if ( useBuffer1 ) {
    memcpy(frameBuffer, frameBuffer2, BUFFER_SIZE_BYTES);  
  } else {
    memcpy(frameBuffer2, frameBuffer, BUFFER_SIZE_BYTES);  
  }
}


void FB_flipBuffer(void)
{
  if ( useBuffer1 ) {
    LCD_SetVRAMAddrEx(0, (void *)frameBuffer2);
  } else {
    LCD_SetVRAMAddrEx(0, (void *)frameBuffer);
  }
  useBuffer1 = !useBuffer1;
}

void FB_copyRenderBufferToDisplayBuffer(void)
{
  memcpy(frameBuffer2, frameBuffer, BUFFER_SIZE_BYTES);
}
  




/** Write the SPI control signals (line adresses and dummy data) to frame buffer */
void FB_writeControlSignals(uint8_t *buffer)
{
  int i,j;
  
  for ( i=0; i<DISPLAY_HEIGHT; i++ )
  {
    /* Clear all pixels */
    for ( j=0; j<DISPLAY_BYTES_PER_LINE; j++ ) 
    {
      *buffer++ = 0x00;
    }
    
    /* Write dummy data and address of next line */
    
    *buffer++ = 0xff;   /* Dummy data for MEMLCD */
    *buffer++ = (i+2);  /* Address of next line */
    *buffer++ = 0xff;   /* Filler data to make virtual display 160px wide */
    *buffer++ = 0xff;   /* Filler data */
  }
}

/** Clears the current buffer */
void FB_clearBuffer(void)
{
  FB_writeControlSignals((uint8_t *)FB_getActiveBuffer());
}
