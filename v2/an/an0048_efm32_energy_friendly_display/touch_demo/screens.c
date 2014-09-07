/**************************************************************************//**
 * @file screens.c
 * @brief Handles the different visible screens and the transitions between them
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
#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_emu.h"
#include "screens.h"
#include "caplesense.h"
#include "slide.h"
#include "framebuffers.h"
#include "dma.h"
#include "anim_timer.h"
#include "main.h"

#include "analog_watch.h"
#include "digital_watch.h"
#include "animation.h"
#include "light_sensor.h"
#include "static_image.h"

#define MAX_SCREENS 10


/* Screen config */
static ScreenConfig screens[MAX_SCREENS];

/* The total number of used screens */
static uint8_t numScreens;

/* The index of the current visible screen */
static uint8_t curScreen;

/* This flag is true when the system is switching between two screens */
static volatile bool isSwitching = false;

/* This flag is true when the user is touching the slider */
static volatile bool touchMode = false;

/* This flag is true when starting touch mode, i.e. just after the user put his finger down */
static volatile bool touchInit = false;

/* The previous screen, used when switching */
int32_t lastScreen;

/* The point on the slider where the user put his finger down */
static int32_t sliderStartPos = 0;

/* A helper variable used when starting touch mode. The pads needs to be scanned 
 * a few times before the readings are valid. This variable is used to keep track
 * of how many times the pads have been scanned */
static volatile uint32_t scanCounter = 0;


/* The currently used drawing limits */
DrawLimits drawLimits = {0, 127};  

/* The current position of the finger on the touch slider */
int32_t curSliderPos = 0;

/* The relative position of the finger to the point where he put it down on the slider */
int32_t relativeSliderPos = 0;



/***********************************************************
 * This function is used to set up the different screens. 
 * Edit this function to set the number of screens and 
 * provide function pointers to the drawing routines for each 
 * screen. 
 * 
 * The following functions must be implemented for each screen:
 *
 *    preview()
 *       This function will draw the image that will be used when switching screens
 * 
 *    prepare() 
 *       This function is called before a screen is made active. It is typically used 
 *       to initialize variables or start the animation timer. 
 *
 *    draw()
 *       This function will render the actual screen. The draw() method is called repeatedly
 *       when the screen is active. The draw() function should also update the drawing limits. 
 *
 *    finish()
 *       The function is called when the screen is made inactive, i.e. the user has switched 
 *       to another screen. This is typically used to clean up variables or stop the animation timer. 
 *
 *    getLimits()
 *       Retrives the limits, the first and last row of the display that needs to be updated. This
 *       is used by the DMA transfer to avoid send uneccesary rows
 **********************************************************/
void initScreens(void)
{
  /* Total number of screens */
  numScreens = 5; 
  
  /* Default screen */
  curScreen = 0;
  lastScreen = 0;
  
  screens[0].preview   = ANALOG_preview;
  screens[0].prepare   = ANALOG_prepare;
  screens[0].draw      = ANALOG_draw;
  screens[0].finish    = ANALOG_finish;
  screens[0].getLimits = ANALOG_getLimits;
  
  screens[1].preview   = DIGITAL_preview;
  screens[1].prepare   = DIGITAL_prepare;
  screens[1].draw      = DIGITAL_draw;
  screens[1].finish    = DIGITAL_finish;
  screens[1].getLimits = DIGITAL_getLimits;
  
  screens[2].preview   = ANIM_preview;
  screens[2].prepare   = ANIM_prepare;
  screens[2].draw      = ANIM_draw;
  screens[2].finish    = ANIM_finish;
  screens[2].getLimits = ANIM_getLimits;
  
  screens[3].preview   = IMAGE_preview;
  screens[3].prepare   = IMAGE_prepare;
  screens[3].draw      = IMAGE_draw;
  screens[3].finish    = IMAGE_finish;
  screens[3].getLimits = IMAGE_getLimits;

  screens[4].preview   = LIGHT_preview;
  screens[4].prepare   = LIGHT_prepare;
  screens[4].draw      = LIGHT_draw;  
  screens[4].finish    = LIGHT_finish;
  screens[4].getLimits = LIGHT_getLimits;
}

/* Start 'touch mode'. This function is called when the user puts his finger 
 * down on the slider */
static uint32_t initTouchMode(void)
{
  /* Set caplesense in active mode and wait a few scans to 
   * make sure the values have settled */
  scanCounter = 0;
  CAPLESENSE_setupLESENSE(false);
  while ( scanCounter < 2 );

  sliderStartPos = CAPLESENSE_getSliderPosition();

  /* Set up the frame buffers to be ready for swiping */
  prepareSwipe(screens, curScreen, numScreens);
  
  /* Update flag to indicate that we have performed the initialization */
  touchInit = false;
  
  /* Return the current slider position */
  return sliderStartPos;
}


/* Draw the screens when the user is touching the slider. This function will
 * get the current position of the finger and draw the screens accordingly */
static void drawTouchMode(void)
{
  if ( touchInit ) 
  {
    /* The just touched the slider */
    curSliderPos = initTouchMode();
  } else 
  {
    /* We are already in touch mode */
    curSliderPos = CAPLESENSE_getSliderPosition();
  }
    
  
  if ( curSliderPos == -1 ) 
  {
    /* The user lifted the finger. Exit touch mode and start slide animation */  
    touchMode = false;
    
    /* Set CAPLESENSE in 'sleep mode' */
    CAPLESENSE_setupLESENSE(true);
    
    lastScreen = curScreen;
    
    /* Start animation */
    curScreen = startSlideAnimation(-relativeSliderPos, curScreen, numScreens);
    
  } else {
  
    /* Calculate the relative slider position */
    relativeSliderPos = (curSliderPos - sliderStartPos) * 8 / 3;
    
    /* Handle the case where the user is pulling away from the outermost screens. 
     * In this case we assign a lower value to make it feel that it is 'harder'
     * to pull in this direction */
    if ( (curScreen == 0 && relativeSliderPos > 0) || (curScreen == numScreens-1 && relativeSliderPos < 0) ) {
      relativeSliderPos /= 4;
    }
    
    /* Clamp to [-128, 128] */
    if ( relativeSliderPos >= 128 ) {
      relativeSliderPos = 128;
    } 
    if ( relativeSliderPos <= -128 ) {
      relativeSliderPos = -128;
    }
    
    /* Draw the screens at the current sliding position */
    drawSlideAt(-relativeSliderPos);
    
    /* Transfer frame to display */
    dmaStartFrameTransfer(0,127);
    FB_flipBuffer();
  }
}

/* Draw animation where the screen 'glides' into place */
void drawAnimation(void)
{        
  /* Update the flag. This will be false when the animation has completed */
  isSwitching = drawSlideAnimation();
  
  /* Draw to display */
  dmaStartFrameTransfer(0,127);
  
  FB_flipBuffer();
}

/* The normal draw routine. This function will draw the currently active 
 * screen by using the callback methods */
static void drawScreen(void) 
{
  /* If we have switched screens, finish the previous one and
   * initialize the current */
  if ( lastScreen != curScreen ) {
    screens[lastScreen].finish();
    screens[curScreen].prepare();
    requestDisplayUpdate();
  }

  /* Check if it is time to update the display */
  if( shouldUpdateDisplay() ){
    
    /* Tell the animation timer that we have updated the display */
    displayUpdated();
    
    /* Transfer frame to display. The frame for the last iteration of the 
     * drawing loop is transferred while we are rendering the next to 
     * maximize the time we can spend in EM2 */
    dmaStartFrameTransfer(drawLimits.min, drawLimits.max);
    
    /* Switch frame buffers so we can draw a new frame while transferring
     * the previous one */
    FB_flipBuffer();
    
    /* Draw the current frame */
    screens[curScreen].draw();
    
    /* Retrive the drawing limits */
    drawLimits = screens[curScreen].getLimits();
    
    /* Set low core frequency while transferring data over SPI with DMA */
    setLowFrequency();
    
    /* Wait until SPI transfer is done */
    while ( DMA->CHENS & DMA_CHENS_CH0ENS ) {
      EMU_EnterEM1();
    }

    /* Set high frequency while the CPU is active */
    setHighFrequency();
    
  } else {
    
    /* Enter EM2 to save power when we do not have to update the display */
    EMU_EnterEM2(false);
  }

  lastScreen = curScreen;
}


/* The main drawing loop. This function takes care of calling the drawing routine
 * for the current visible screen and activate the DMA transfer to update the 
 * display from the rendered frame buffer. When a user it touching the touch
 * slider it will switch to 'touch' mode where the user can drag along the 
 * touch slider to switch between screens. When the slider is released a short
 * animation 'snaps' the new screen into place */
void drawLoop(void)
{  
  /* Start CAPLESENSE in 'sleep mode' */
  CAPLESENSE_setupLESENSE(true);
  
  /* Call prepare on the first visible screen to properly initialize it */
  screens[curScreen].prepare();

  while (1) {
    if ( isSwitching ) {
      if ( touchMode ) {
        /* Draw screens at the current sliding position */
        drawTouchMode();
      } else {
        /* Draw sliding animation */
        drawAnimation();
      }
    } else {
      /* Draw the current visible screen */
      drawScreen();   
    }
  }
}


/**************************************************************************//**
 * @brief  Callback for sensor scan complete.
 *****************************************************************************/
void capSenseScanComplete(void)
{
  scanCounter++;
}

/**************************************************************************//**
 * @brief  Callback for sensor channel triggered.
 *****************************************************************************/
void capSenseChTrigger(void)
{
  /* Ignore subsequent calls while in touch mode */
  if ( touchMode || isSwitching ) {
    return;
  }
  
  /* Set the flags to indicate that we should start swiping */
  isSwitching = true;
  touchMode = true;
  touchInit = true;
}
