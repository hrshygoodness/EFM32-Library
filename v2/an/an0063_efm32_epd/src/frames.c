/*****************************************************************************
 * @file frames.c
 * @brief Generates frames for EPD panel
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
#include "config.h"
#include "cog.h"

/* EPD pixel definitions */
#define BLACK    0x3
#define WHITE    0x2
#define NOTHING  0x1


extern EPD_Config epdConfig;


/**********************************************************
 * Helper function to create one pixel for the EPD
 * frame based on the current stage and the pixel
 * from either the old or new frame buffer. 
 * The returned pixel is 2 bits with one of the
 * values: 
 * 
 *   BLACK   (0b11)
 *   WHITE   (0b10)
 *   NOTHING (0b01)
 * 
 * @param oldPixel
 *   A pixel from the old frame buffer. Is either 0 or 1. 
 * 
 * @param newPixel
 *   A pixel from the new frame buffer. Is either 0 or 1. 
 * 
 * @param stage
 *   The current stage number (1-4)
 * 
 * @returns
 *   A pixel value (2 bits) for the EPD frame buffer
 **********************************************************/
uint8_t createPixel(int oldPixel, int newPixel, int stage)
{
  switch (stage)
  {
  
  /* Stages 1 and 2 only care about old image */
  case 1:
    return oldPixel ? WHITE : BLACK;
  case 2:
    return oldPixel ? NOTHING : WHITE;
    
  /* Stages 3 and 4 only care about new image */
  case 3: 
    return newPixel ? NOTHING : BLACK;
  case 4: 
    return newPixel ? BLACK : WHITE;
  }
  
  /* Should never reach this */
  while (1);
}

/**********************************************************
 * Generates an EPD frame from the supplied stage number
 * and the two frame buffers in memory. During stage
 * 1 and 2 the frame is determined from the old (previous)
 * frame. Stage 3 and 4 use the new frame. 
 * The EPD frame buffer is put in outFrame. The caller must
 * make sure that enough memory is allocated for this buffer. 
 * 
 * @param oldFrame
 *   The old (previous) frame buffer
 * 
 * @param newFrame
 *   The new frame buffer (to be displayed)
 * 
 * @param outFrame
 *   The EPD frame is put in this buffer
 * 
 * @param stage
 *   The current stage number (1-4)
 **********************************************************/
void generateFrame(uint8_t *oldFrame, uint8_t *newFrame, uint8_t *outFrame, int stage)
{
  /* Temp variable for pixel EPD pixel data. 2 bits per pixel. */
  uint8_t epdPx;
  
  /* Temp variables for frame buffer pixel data. 1 bit per pixel */
  uint8_t oldPx, newPx;

  /* Screen coordinates */
  int x, y;
  
  /* Number of shifts to perform when saving the EPD pixel to buffer */
  int shifts;
  
  /* Used to calculate position of the current line in the scan line array */
  uint8_t scanByte, scanShift;
  
  /* Get width and height from the configuration */
  int width  =  epdConfig.horizontalSize;
  int height =  epdConfig.verticalSize;
  
  /* Clear frame */
  memset(outFrame, 0, PANEL_FRAME_SIZE);
  
  y = 0;
  while ( y < height )
  {
    /* Border byte check */
    if ( epdConfig.borderByte )
    {
      *outFrame++ = 0x00;
    }
    
    /* Start at rightmost pixel */
    x = width - 1;
    
    /* Start writing to the upper 2 bits of the byte (MSB) */
    shifts = 6;
  
    /* Generate even pixels, starting with the rightmost */
    while ( x > 0 )
    {
      /* Extract pixel at (x,y) from old frame */
      oldPx = (oldFrame[y * width/8 + x/8] >> (x%8)) & 0x1;
      
      /* Extract pixel at (x,y) from new frame */
      newPx = (newFrame[y * width/8 + x/8] >> (x%8)) & 0x1;
      
      /* Generate EPD pixel based on old and new pixel and current stage */
      epdPx = createPixel(oldPx, newPx, stage);

      /* Write pixel to EPD frame */
      *outFrame |= epdPx << shifts;

      /* Get next even pixel */
      x -= 2;
      
      /* Decrease shift. If current byte is full, move to the next */
      shifts -= 2;
      if ( shifts < 0 )
      {
        shifts = 6;
        outFrame++;
      }
    }
    
    /* Calculate the position of the current line in the scan lines buffer. */
    scanByte = y / 4;
    scanShift = 6 - (y%4)*2;
    
    /* The scan line should have 0b11 at the current line and 0 for the rest. */
    *(outFrame + scanByte) = 0x3 << scanShift;
    
    /* Skip over all the scan bytes */
    outFrame += height / 4;
    
    /* Start at leftmost pixel */
    x = 0;
    
    /* Generate odd pixels, starting with the leftmost */
    while ( x < width )
    {
      /* Extract pixel at (x,y) from old frame */
      oldPx = (oldFrame[y * width/8 + x/8] >> (x%8)) & 0x1;
      
      /* Extract pixel at (x,y) from new frame */
      newPx = (newFrame[y * width/8 + x/8] >> (x%8)) & 0x1;
      
      /* Generate EPD pixel based on old and new pixel and current stage */
      epdPx = createPixel(oldPx, newPx, stage);

      /* Write pixel to EPD frame */
      *outFrame |= epdPx << shifts;
      
      /* Get next odd pixel */
      x += 2;
      
      /* Decrease shift. If current byte is full, move to the next */
      shifts -= 2;
      if ( shifts < 0 )
      {
        shifts = 6;
        outFrame++;
      }
    }
    
    /* Add dummy byte if needed */
    if ( epdConfig.dummyByte )
    {
      *outFrame++ = 0x00; 
    }
    
    /* Move to the next line */
    y++;
  }
}
