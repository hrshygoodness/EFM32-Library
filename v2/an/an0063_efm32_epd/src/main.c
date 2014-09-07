/**************************************************************************//**
 * @file main.c
 * @brief EPD application main file
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
#include "GUI.h"
#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_emu.h"
#include "frames.h"
#include "config.h"
#include "display.h"
#include "letimer.h"
#include "main.h"

/* Precompiled images. These are generated with the 
 * emWin bitmap conversion tool */
extern GUI_CONST_STORAGE GUI_BITMAP bmgecko;
extern GUI_CONST_STORAGE GUI_BITMAP bmsilabs;
extern GUI_CONST_STORAGE GUI_BITMAP bmsegger;
extern GUI_CONST_STORAGE GUI_BITMAP bmss_logo;
extern GUI_CONST_STORAGE GUI_BITMAP bmpdi;

/* The currently visible screen */
static int screen = 0;

/* The previous screen */
static int lastScreen = -1;

/* Flag indicating wheter we are in slideshow mode */
bool slideShow = false;

/* Offset the image when rendering on a larger panel
 * so the image is centered */
#if defined(PANEL_270)
int xOff = 30;
int yOff = 30;
#else
int xOff = 0;
int yOff = 0;
#endif


/**********************************************************
 * Configures the push buttons on the kit as input
 * with interrupts enabled.
 **********************************************************/
void buttonsInit(void)
{
  GPIO_PinModeSet(gpioPortB, 9, gpioModeInput, 0);
  GPIO_IntConfig(gpioPortB, 9, false, true, true);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
  
  GPIO_PinModeSet(gpioPortB, 10, gpioModeInput, 0);
  GPIO_IntConfig(gpioPortB, 10, false, true, true);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}


/**********************************************************
 * This program sets up the EPD and renders one of the
 * predefined screens. The user can toggle between
 * each of the screens by using push button 0 on the 
 * STK. Push button 1 toggles slideshow mode where
 * the screens are automatically changed repeatedly.
 **********************************************************/
int main(void)
{
  /* Handle possible chip errata */
  CHIP_Init();

  /* Select HFRCO */
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO); 
  
  /* Initialize peripherals and pins to control the EPD */
  epdInit();
  
  /* Initialize LETIMER0 to use for slideshow timing */
  letimerInit();
  
  /* Set up push buttons with interrupts */
  buttonsInit();

  /* Start the emWin system */
  GUI_Init();
      
  /* Main loop */
  while (1)
  {
    /* Disable GPIO IRQs when drawing. Any interrupts triggered
     * while drawing will be handled after drawing has completed */
    NVIC_DisableIRQ(GPIO_ODD_IRQn);
    NVIC_DisableIRQ(GPIO_EVEN_IRQn);
    
    if ( screen != lastScreen )
    {    
      
      /* Run off the HFXO while rendering. This makes the rendering
       * step complete faster and saves power */
      CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
      
      /* Render an image depending on which screen is selected. 
       * The image is rendered by emWin to a frame buffer in SRAM. */
      switch (screen)
      {
      case 0:
      default:
        GUI_Clear();
        GUI_DrawBitmap(&bmgecko,        0 + xOff,    0 + yOff);
        GUI_DispStringAt("EFM32",       140 + xOff, 10 + yOff);
        break;
      case 1:
        GUI_Clear();
        GUI_DrawBitmap(&bmsilabs,               50 + xOff, 10 + yOff);
        GUI_DispStringHCenterAt("Silicon Labs", 99 + xOff, 63 + yOff);
        GUI_DispStringHCenterAt("EFM32 MCU",    99 + xOff, 80 + yOff);
        break;
      case 2: 
        GUI_Clear();
        GUI_DrawBitmap(&bmpdi,                                  22 + xOff, 30 + yOff);
        GUI_DispStringHCenterAt("Pervasive Displays",           99 + xOff, 63 + yOff);
        GUI_DispStringHCenterAt("Electronic Paper Display",     99 + xOff, 80 + yOff);
        break;
      case 3: 
        GUI_Clear();
        GUI_DrawBitmap(&bmsegger,                       50 + xOff, 10 + yOff);
        GUI_DispStringHCenterAt("Segger emWin",         99 + xOff, 63 + yOff);
        GUI_DispStringHCenterAt("Graphical Library",    99 + xOff, 80 + yOff);
        break;
      case 4:
        GUI_Clear();
        GUI_DispStringHCenterAt("Download Simplicity",  99 + xOff, 15 + yOff);
        GUI_DispStringHCenterAt("Studio today",         99 + xOff, 15 + 16 + yOff);
        GUI_DispStringHCenterAt("energymicro.com",      99 + xOff, 15 + 2 * 16 + yOff);
        GUI_DispStringHCenterAt("/simplicity",          99 + xOff, 15 + 3 * 16 + yOff);
        break;      
      }
      
      /* Switch back to HFRCO */
      CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);
      
      /* Send image to display */
      showImage();
      
      lastScreen = screen;
    }
    
    
    /* Enable button interrupts again */
    __disable_irq();
    NVIC_EnableIRQ(GPIO_ODD_IRQn);
    NVIC_EnableIRQ(GPIO_EVEN_IRQn);
    
    /* Enter EM2 sleep. Any pending interrupts will wake up the MCU. */
    EMU_EnterEM2(true);
    
    /* Enable interrupts. Interrupts will be handled after this. */
    __enable_irq();
    
  }
}

/**********************************************************
 * Switch to the next screen. This function only updates
 * the screen variable. It is up to the caller to make sure
 * the display is redrawn. 
 **********************************************************/
void nextScreen(void)
{
  if ( ++screen > 4 )
  {
    screen = 0;
  }
}


/**********************************************************
 * Interrupt handler for push button 0 (pin PB9). 
 * Switches to the next screen when not in slideshow
 * mode. In slideshow mode, this handler does nothing. 
 **********************************************************/
void GPIO_ODD_IRQHandler(void)
{
  GPIO->IFC = GPIO->IF;
  
  if ( !slideShow )
  {
    nextScreen();
  }
}

/**********************************************************
 * Interrupt handler for push button 1 (pin PB10). 
 * Toggles on/off slideshow mode. 
 **********************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  GPIO->IFC = GPIO->IF;

  slideShow = !slideShow;
  
  if ( slideShow )
  {
    letimerStart();
  } 
  else
  { 
    letimerStop();
  }
}
