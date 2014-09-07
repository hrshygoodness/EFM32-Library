/**************************************************************************//**
 * @file modeSSD2119.c
 * @brief Driving a Graphical Display with SSD2119 driver 
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

#define LCD_WIDTH 320
#define LCD_HEIGHT 240


extern GUI_CONST_STORAGE GUI_BITMAP bmlogo;

/** Initialize clocks */
void initClocks(void) 
{
  /* Configure for 48MHz HFXO operation of core clock */
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO); 
  
  /* on Giant & Leopard we have to reduce core clock when 
   * talking with slow TFT controller */
  CMU_ClockDivSet(cmuClock_CORE, cmuClkDiv_2);
  
  /* Enable SysTick interrupt, used by GUI software timer */
  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000)) while (1) ;
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
  
  /* Initialize emWin */
  GUI_Init();
  
  /* Enter drawing loop */
  drawingLoop();
  
  return 0;
}
