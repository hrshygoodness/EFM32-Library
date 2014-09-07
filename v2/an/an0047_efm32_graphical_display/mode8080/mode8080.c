/**************************************************************************//**
 * @file mode8080.c
 * @brief Driving a Graphical Display in 8080 Mode Example
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
#include <string.h>
#include <stdio.h>
#include "GUI.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_ebi.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "bsp.h"
#include "bsp_trace.h"
#include "tftamapped.h"
#include "dma_frame_copy.h"
#include "dmd_ssd2119.h"
#include "dmdif_ssd2119_ebi.h"

/* Application specific defines */
#include "display_conf.h"


extern GUI_CONST_STORAGE GUI_BITMAP bmlogo;

/** Initialize clocks */
void initClocks(void) 
{
  /* Configure for 48MHz HFXO operation of core clock */
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO); 
  
  /* Enable SysTick interrupt, used by GUI software timer */
  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000)) while (1) ;
}


/** Initialize the display controller. This is hardware dependent and will have to be
 * changed when using a different display controller. */
void initDisplayController(void) 
{  
  /* Try to init display until we have control over it */
  while ( !TFT_AddressMappedInit() );
  
  /* Prepare the display controller to accept pixel data */
  DMDIF_prepareDataAccess();
}


/** Simple pseudorandom number generator */
int randnum(int lim)
{
  static long a = 3;
  a = (((a * 214013L + 2531011L) >> 16) & 32767);
  
  return ((a % lim) + 1);
}


/** Draw a image on different locations on the screen */
void drawingLoop(void) {
  int x,y;
    
  while (1) {

    /* Draw something */
    x = randnum(LCD_WIDTH - bmlogo.XSize);
    y = randnum(LCD_HEIGHT - bmlogo.YSize);
    GUI_DrawGradientV(0, 0, LCD_WIDTH, LCD_HEIGHT, GUI_BLUE, GUI_GREEN);
    GUI_DrawBitmap(&bmlogo, x, y);    
    
    /* Drawing completed */
    
    /* Start a DMA transfer to copy the entire frame to the display */  
    copyFrameToLcd( (uint16_t *)VRAM_ADDR_START, LCD_WIDTH*LCD_HEIGHT);
   
    /* Wait a little between each frame */
    GUI_Delay(2000);
  }
}



int main(void)
{  
  /* Chip errata */
  CHIP_Init();
  
  /* Initialize clocks */
  initClocks();
  
  /* Initialize EBI configuration for external RAM and display controller */
  BSP_Init(BSP_INIT_DK_EBI);

  /* Initialize DMA for frame buffer transfer */  
  initDMA();
  
  /* Initialize emWin */
  GUI_Init();
  
  /* Enter drawing loop */
  drawingLoop();
  
  return 0;
}
