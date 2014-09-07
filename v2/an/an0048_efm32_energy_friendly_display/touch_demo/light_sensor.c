/**************************************************************************//**
 * @file light_sensor.c
 * @brief Display light sensor measurement
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

#include "GUI.h"
#include "em_device.h"
#include "em_acmp.h"
#include "em_cmu.h"
#include "em_timer.h"
#include "em_gpio.h"
#include "em_emu.h"
#include "memlcd.h"
#include "light_sensor.h"
#include "anim_timer.h"
#include "framebuffers.h"
#include "main.h"


/* Macros used to plot coordinates */
#define XCOORD(t) ((t) % 128)
#define YCOORD(t) (20 + (63 - light_values[(t) % 128]))

/* The last measured light values in a circular buffer */
uint8_t light_values[128];

/* The current index of the light_values array */
uint8_t array_index = 0;

/* The lines we need to send to the display. This will be updated by drawing routine */
static DrawLimits drawLimits = {0, 127};


static void initLightSensor(void)
{
  /* Enable clocks required */
  CMU_ClockEnable(cmuClock_ACMP0, true);

  const ACMP_Init_TypeDef acmp_init =
  {
    false,                              /* Full bias current*/
    false,                              /* Half bias current */
    4,                                  /* Biasprog current configuration */
    false,                              /* Enable interrupt for falling edge */
    false,                              /* Enable interrupt for rising edge */
    acmpWarmTime256,                    /* Warm-up time in clock cycles, >140 cycles for 10us with 14MHz */
    acmpHysteresisLevel1,               /* Hysteresis configuration */
    0,                                  /* Inactive comparator output value */
    false,                              /* Enable low power mode */
    16,                                  /* Vdd reference scaling */
    true,                              /* Enable ACMP */
  };

  /* Init and set ACMP channel */
  ACMP_Init(ACMP0, &acmp_init);
  ACMP_ChannelSet(ACMP0, acmpChannelVDD, acmpChannel6);
  
  
  while((ACMP0->STATUS & ACMP_STATUS_ACMPACT));
}

/* Retrieves the light value by using binary search with the ACMP */
static int32_t getLightLevel(void)
{
  int32_t result = 0;
  int i;
  uint8_t vddlevel_min = 0x0;
  uint8_t vddlevel_max = 0x3f;
  uint8_t vddlevel = ((vddlevel_max-vddlevel_min)/2);
  
  /* Enable ACMP */
  ACMP0->CTRL |= ACMP_CTRL_EN;
  
  /* Excite light sensor */
  GPIO_PinModeSet(gpioPortD, 6, gpioModePushPull, 1);
  
  /* Slow down MCU frequency while measuring. This saves power since 
   * we have to wait for the light sensor to stabilize anyway */
  setLowFrequency();

  /* Wait for the previous display update to complete. 
   * The light sensor only uses one frame buffer. Therefore
   * we have to wait until the frame buffer has been sent before 
   * we can start rendering the next frame. */
  while ( DMA->CHENS & DMA_CHENS_CH0ENS ) {
    EMU_EnterEM1();
  }
  
  uint32_t acmp_inputsel_reg = ACMP0->INPUTSEL & ~(_ACMP_INPUTSEL_VDDLEVEL_MASK);
  
  /* Set reference at half initially. */
  ACMP0->INPUTSEL = acmp_inputsel_reg | (vddlevel<<_ACMP_INPUTSEL_VDDLEVEL_SHIFT);
    
  for(i = 0;i<5;i++){
    if((ACMP0->STATUS & ACMP_STATUS_ACMPOUT) >> _ACMP_STATUS_ACMPOUT_SHIFT){
      /* go up */
      vddlevel_min = vddlevel;
      vddlevel += ((vddlevel_max-vddlevel_min)/2)+1; 
    }else{
      /* go down */
      vddlevel_max = vddlevel;
      vddlevel -= ((vddlevel_max-vddlevel_min)/2);
    }
    ACMP0->INPUTSEL = acmp_inputsel_reg | (vddlevel<<_ACMP_INPUTSEL_VDDLEVEL_SHIFT);
    
  }
  
  if((ACMP0->STATUS & ACMP_STATUS_ACMPOUT) >> _ACMP_STATUS_ACMPOUT_SHIFT){
    result = vddlevel;
  }else{
    result = vddlevel-1;
  }
  
  /* Disable light sensor and ACMP */
  ACMP0->CTRL &= ~ACMP_CTRL_EN;
  GPIO_PinModeSet(gpioPortD, 6, gpioModeDisabled, 0);
  
  /* Set high frequency again */
  setHighFrequency();

  return result;
}


void LIGHT_preview(void)
{
  FB_clearBuffer();
  GUI_DispStringHCenterAt("Light sensor",  63, 110);
}


void LIGHT_prepare(void)
{
  initLightSensor();
  GUI_DispStringHCenterAt("Light sensor",  63, 110);
  startAnimTimer(100);
}


void LIGHT_draw(void)
{  
  int i;
  uint8_t x[] = {0,0};
  uint8_t y[] = {0,0};
  
  FB_flipBuffer();
  
  /* Get the current light sensor value */
  light_values[array_index] = getLightLevel();
  
  /* Start with extreme values for the draw limits */
  drawLimits.min = 127;
  drawLimits.max = 0;
  
  /* Clear previous line */
  if ( array_index + 4 != 128 ) {
    
    /* Set black color to erase lines */
    GUI_SetColor(GUI_BLACK);
    
    /* Calculate coordinates */
    x[0] = XCOORD(array_index+3);
    x[1] = XCOORD(array_index+4);
    
    y[0] = YCOORD(array_index+3);
    y[1] = YCOORD(array_index+4);
    
    /* Draw line to frame buffer */
    GUI_DrawLine( x[0], y[0], x[1], y[1] );
    
    /* Update draw limits */
    for ( i=0; i<2; i++ ) {
      if ( y[i] < drawLimits.min ) {
        drawLimits.min = y[i];
      }
      if ( y[i] > drawLimits.max ) {
        drawLimits.max = y[i];
      }
    }
  }

  if ( array_index != 0 ) {
    
    /* Set white color to draw lines */
    GUI_SetColor(GUI_WHITE);
    
    /* Calculate coordinates */
    x[0] = XCOORD(array_index-1);
    x[1] = XCOORD(array_index);
    
    y[0] = YCOORD(array_index-1);
    y[1] = YCOORD(array_index);
    
    /* Draw line to frame buffer */
    GUI_DrawLine( x[0], y[0], x[1], y[1] );
    
    /* Update draw limits */
    for ( i=0; i<2; i++ ) {
      if ( y[i] < drawLimits.min ) {
        drawLimits.min = y[i];
      }
      if ( y[i] > drawLimits.max ) {
        drawLimits.max = y[i];
      }
    }
  }
  
  /* Reset default color */
  GUI_SetColor(GUI_WHITE);
  
  
  /* Make sure we send at least 4 lines. The DMA transfer is also 
   * used as a delay to make the light sensor settle. 
   * With this we make sure the DMA transfer is long enough */
  while ( drawLimits.max - drawLimits.min < 4 ) {
    if ( ++drawLimits.max >= 127 ) drawLimits.max = 127;
    if ( --drawLimits.min <= 0 ) drawLimits.min = 0;
  }
  
  
  /* Increase the array index each time */
  if( ++array_index >= 128){
    array_index = 0;
  }
}

void LIGHT_finish(void)
{
  GPIO_PinModeSet(gpioPortD, 6, gpioModeDisabled, 0);
  ACMP_Disable(ACMP0);
  stopAnimTimer();
}

DrawLimits LIGHT_getLimits(void)
{
  return drawLimits;
}

