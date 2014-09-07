/**************************************************************************//**
 * @file slide.c
 * @brief Takes care of the sliding animation when using the touch slider.
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
#include <math.h>
#include "GUI.h"
#include "em_device.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "screens.h"
#include "slide.h"
#include "framebuffers.h"

/* Top counter value for sliding animation */
#define TOP 4000

#define WIDTH 128
#define HEIGHT 128

/* This is the threshold (in px) we have to drag before the screens will switch */
#define SLIDE_THRESHOLD 30

/* Frame buffers holding the different screens when sliding */
static uint16_t leftBuffer[WIDTH * HEIGHT / 16];
static uint16_t rightBuffer[WIDTH * HEIGHT / 16];
static uint16_t centerBuffer[WIDTH * HEIGHT / 16];

/* Coefficients for the sliding function */
static int32_t alpha, beta;

/* Copies a frame to the given local buffer */
static void copyToBuffer(uint32_t *dst, uint32_t *src)
{
  uint8_t i,j;
  for ( j=0; j<128; j++ ) {
    for ( i=0; i<5; i++ ) {
      
      /* Skip metadata and only copy pixels */
      if ( i<4 ) {
        *dst++ = *src;
      }
      src++;
    }
  }
}

static double roundDouble(double r) {
    return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);
}

/* Clears a local buffer */
static void clearBuffer(uint32_t *buffer)
{
  memset(buffer, 0, WIDTH * HEIGHT / 8);
}

/* Will draw the screens at a specified 'sliding position' which is
 * between -128 (drawing the left image) and +128 (drawing the right image)
 * This function will copy the pixels from the different buffers
 * according to the slide position.
 */
void drawSlideAt(int32_t slidePos)
{
  uint32_t i,j;
  
  /* Clamp to [-128, 128] */
  if ( slidePos < -128 ) slidePos = -128;
  if ( slidePos > 128 )  slidePos = 128;
  
  /* Change to positive value */
  slidePos = slidePos + 128;
  
  /* Find the window to copy pixels from */
  uint8_t shifts = slidePos % 32;
  uint8_t firstWord = slidePos / 32;
  
  /* If the position is aligned to 32 pixels we only need to copy from 
   * 4 words. If not we have to read from 5 words */
  uint8_t lastWord = shifts == 0 ? firstWord + 3 : firstWord + 4;
  
  uint32_t *left   = (uint32_t *)leftBuffer;
  uint32_t *right  = (uint32_t *)rightBuffer;
  uint32_t *center = (uint32_t *)centerBuffer;
  uint32_t *dst = (uint32_t *)FB_getActiveBuffer();
  
  FB_clearBuffer();
    
  /* Loop over all lines */
  for ( j = 0; j<128; j++ ) 
  {
    
    /* Copy 4 words (=128px) per line. The value of 
     * i determines which buffer we should copy from.
     * For each source word we generally need to write
     * to two different words (unless slidePos is aligned to a word)
     */
    for ( i = firstWord; i<=lastWord; i++ ) 
    {
      /* Copy from left buffer */
      if ( i < 4 )
      {
        
        /* Copy the remaining pixels to the previous word.
         * Only if we are not at the first word on a line. */
        if ( i - firstWord > 0 ) {
          *(dst-1) |= (*(left + j * 4 + i)) << (32 - shifts);
        }
        
        /* Copy pixels and increase dest. Do not to this for the 5th word on a line */
        if ( i - firstWord < 4 ) {
          *dst = (*(left + j * 4 + i)) >> shifts;
          dst++;
        }
        
      }
      
      /* Copy from center buffer */
      else if ( i < 8 ) 
      {
        
        /* Copy the remaining pixels to the previous word.
         * Only if we are not at the first word on a line. */
        if ( i - firstWord > 0 ) {
          *(dst-1) |= (*(center + j * 4 + (i-4))) << (32 - shifts);
        }
        
        /* Copy pixels and increase dest. Do not to this for the 5th word on a line */
        if ( i - firstWord < 4 ) {
          *dst = (*(center + j * 4 + (i-4))) >> shifts;
          dst++;
        }
      }
      
      /* Copy from right buffer */
      else 
      {
        
        /* Copy the remaining pixels to the previous word.
         * Only if we are not at the first word on a line. */
        if ( i - firstWord > 0 ) {
          *(dst-1) |= (*(right + j * 4 + (i-8))) << (32 - shifts);
        }
        
        /* Copy pixels and increase dest. Do not to this for the 5th word on a line */
        if ( i - firstWord < 4 ) {
          *dst = (*(right + j * 4 + (i-8))) >> shifts;
          dst++;
        }
      }
    }
    
    /* Skip over meta and dummy data */
    dst++;
  }
}

/* Prepares the buffers for the sliding animations. Will copy a preview of each 
 * screen into the respective buffers */
void prepareSwipe(ScreenConfig *screens, uint8_t curScreen, uint8_t numScreens)
{
  /* Copy the current screen to the center buffer */
  copyToBuffer((uint32_t*)centerBuffer, (uint32_t*)FB_getActiveBuffer());
  
  /* Disable custom buffer (if in use) */
  FB_disableCustomBuffer();
  
  /* Render a preview and copy it to the left buffer */
  if ( curScreen == 0 ) {
    clearBuffer((uint32_t*)leftBuffer);
  } else {
    screens[curScreen-1].preview();
    copyToBuffer((uint32_t*)leftBuffer, (uint32_t*)FB_getActiveBuffer());
  }
  
  /* Render a preview and copy it to the right buffer */
  if ( curScreen == numScreens-1 ) {
    clearBuffer((uint32_t*)rightBuffer);
  } else {
    screens[curScreen+1].preview();
    copyToBuffer((uint32_t*)rightBuffer, (uint32_t*)FB_getActiveBuffer());
  }
}


/* Start the animation where a screen glides into place. This is called when the 
 * user releases his/hers finger from the touch slider */
uint32_t startSlideAnimation( int32_t startPos, uint32_t curScreen, uint32_t numScreens ) 
{
  uint32_t ret = curScreen; 
  
  /* Slide to left */
  if ( startPos < -SLIDE_THRESHOLD && curScreen > 0 ) {
    
    /* Set coefficients */
    alpha = -1;
    beta = 1;
    
    /* Set next screen */
    ret = curScreen - 1;
    
    /* Initialize timer counter to a value representing the current sliding position */
    TIMER1->CNT = (uint32_t)(((float)(-startPos) / 128) * TOP);
    
  } 
  
  /* Slide back to center from left */
  else if ( startPos < 0 ) {
    
    /* Set coefficients */
    alpha = -1;
    beta = -1;
    
    /* Set next screen */
    ret = curScreen;
    
    /* Initialize timer counter to a value representing the current sliding position */
    TIMER1->CNT = (uint32_t)(((128 + (float)startPos) / 128) * TOP);

  } 
  
  /* Slide to right */  
  else if ( startPos > SLIDE_THRESHOLD && curScreen < numScreens - 1 ) {
    
    /* Set coefficients */
    alpha = 1;
    beta = -1; 
    
    /* Set next screen */
    ret = curScreen + 1;
    
    /* Initialize timer counter to a value representing the current sliding position */
    TIMER1->CNT = (uint32_t)(((float)startPos / 128) * TOP);
  
  } 
  
  /* Slide back to center from right */  
  else {     

    /* Set coefficients */
    alpha = 1;
    beta = 1;
    
    /* Set next screen */
    ret = curScreen;
    
    /* Initialize timer counter to a value representing the current sliding position */
    TIMER1->CNT = (uint32_t)(((128 - (float)startPos) / 128) * TOP);
  } 

  TIMER1->CMD = TIMER_CMD_START;
  
  return ret;
}


/* Drawing function for the sliding animation. This function is called
 * repeatedly when the sliding animation is active */
bool drawSlideAnimation(void)
{
  uint32_t cnt = 0;
  int32_t pos = 0;
  float theta;
  
  /* The current position is calculated from the counter value */
  cnt = TIMER1->CNT;
  if ( cnt > TOP ) cnt = TOP;
  
  /* Calculate sliding position. This will be between
   * -128 and +128  */
  theta = (float)cnt / TOP * 3.14;
  pos = (int32_t)roundDouble(64 * (alpha + beta * cos(theta)));
  
  /* Draw the screens */
  drawSlideAt(pos);
  
  /* Stop the animation if we are finished */
  if ( cnt >= TOP ) {
    TIMER1->CMD = TIMER_CMD_STOP;
    return false;
  } else {
    return true;
  }
}


/* Configures TIMER1 for use with the sliding animation */
void initTimerForSlide(void)
{
  CMU_ClockEnable(cmuClock_TIMER1, true);
  
  TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
  
  timerInit.prescale = timerPrescale1024;
  timerInit.oneShot = true;
  
  TIMER_Init(TIMER1, &timerInit);
}
