/**************************************************************************//**
 * @file modeRGB.c
 * @brief Driving a Graphical Display in RGB Mode Example
 * @author Silicon Labs
 * @version 1.06
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
#include "GUI.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "tftdirect.h"
#include "bsp.h"
#include "bsp_trace.h"

#include "display_conf.h"

/* Logo generated from bitmap with emWin bitmap converter */
extern GUI_CONST_STORAGE GUI_BITMAP bmlogo;


/*********************************************************************
* Configure the Direct Drive Controller
**********************************************************************
*/ 
static const EBI_TFTInit_TypeDef tftInit =
{ 
  ebiTFTBank2,                  /* Select EBI Bank 2 */
  ebiTFTWidthHalfWord,          /* Select 2-byte (16-bit RGB565) increments */
  ebiTFTColorSrcMem,            /* Use memory as source for mask/blending */
  ebiTFTInterleaveUnlimited,    /* Unlimited interleaved accesses */
  ebiTFTFrameBufTriggerVSync,   /* VSYNC as frame buffer update trigger */
  false,                        /* Drive DCLK from negative edge of internal clock */
  ebiTFTMBDisabled,             /* No masking and alpha blending enabled */
  ebiTFTDDModeExternal,         /* Drive from external memory */
  ebiActiveLow,                 /* CS Active Low polarity */
  ebiActiveHigh,                /* DCLK Active High polarity */
  ebiActiveLow,                 /* DATAEN Active Low polarity */
  ebiActiveLow,                 /* HSYNC Active Low polarity */
  ebiActiveLow,                 /* VSYNC Active Low polarity */
  320,                          /* Horizontal size in pixels */
  1,                            /* Horizontal Front Porch */
  30,                           /* Horizontal Back Porch */
  2,                            /* Horizontal Synchronization Pulse Width */
  240,                          /* Vertical size in pixels */
  1,                            /* Vertical Front Porch */
  4,                            /* Vertical Back Porch */
  2,                            /* Vertical Synchronization Pulse Width */
  0x0000,                       /* Frame Address pointer offset to EBI memory base */
  4,                            /* DCLK Period */
  0,                            /* DCLK Start cycles */
  0,                            /* DCLK Setup cycles */
  0,                            /* DCLK Hold cycles */
};


/** Initialize Direct Drive. This function is called from emWin during initialization. */
void initDisplayController(void) 
{
  /* Set up Direct Drive and configure display controller */
  TFT_DirectInit(&tftInit);
}

/* Select clock source and enable SysTick interrupt */
void initClocks(void) 
{  
  /* Configure for 48MHz HFXO operation of core clock */
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO); 

  /* Enable SysTick interrupt, used by GUI software timer */
  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000))  while (1);
}


/** Simple pseudorandom number generator */
int randnum(int lim)
{
  static long a = 3;
  a = (((a * 214013L + 2531011L) >> 16) & 32767);
  
  return ((a % lim) + 1);
}

void drawingLoop(void) 
{
  int x,y;
    
  while (1) {

    /* Draw something */
    x = randnum(LCD_WIDTH - bmlogo.XSize);
    y = randnum(LCD_HEIGHT - bmlogo.YSize);
    GUI_DrawGradientV(0, 0, LCD_WIDTH, LCD_HEIGHT, GUI_BLUE, GUI_GREEN);
    GUI_DrawBitmap(&bmlogo, x, y);    
    
    /* Drawing completed. Direct Drive will drive display. */
    
   
    /* Wait a little between each frame */
    GUI_Delay(2000);
  }
}



int main(void)
{
  /* Chip errata */
  CHIP_Init();
  
  /* Initialize clocks  */
  initClocks();
  
  /* Initialize EBI banks (Board Controller, external PSRAM, ..) */
  BSP_Init(BSP_INIT_DK_EBI);
  
  /* Initialize emWin Library. Will call initDisplayController to initialize Direct Drive. */
  GUI_Init();

  /* Initialization done, enter drawing loop. 
   * More emWin examples can be viewed by copy pasting into this file and 
   * uncommenting the following line calling MainTask() instead of drawingLoop()
   * emWin examples can be found under reptile/emwin/examples
   */
  
  drawingLoop();
  
  //MainTask();
  
  return 0;
}

