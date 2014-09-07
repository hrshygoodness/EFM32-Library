/**************************************************************************//**
 * @file main.c
 * @brief Energy Friendly Display Example
 * @author Silicon Labs
 * @version 1.05
 * 
 * This example shows how to optimize your code in order to drive 
 * a graphical display in an energy friendly way. 
 * 
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
#include <math.h>
#include <stdio.h>
#include "GUI.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_rtc.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_prs.h"
#include "em_letimer.h"
#include "em_timer.h"
#include "memlcd.h"
#include "caplesense.h"
#include "dma.h"
#include "clock.h"
#include "screens.h"
#include "slide.h"
#include "animation.h"


/* The 'high' and 'low' frequency bands used */
#define CORE_FREQ_HIGH cmuHFRCOBand_28MHz
#define CORE_FREQ_LOW  cmuHFRCOBand_1MHz

/* The number to use for USART CLKDIV when in 'low frequency' mode */
#define USART_LOW_FREQ_CLKDIV (0 << _USART_CLKDIV_DIV_SHIFT)
  

/* Flag to check if the SPI transfer is currently active */
extern volatile bool spiTransferActive;

/* Display configuration. See memlcd.h */
MEMLCD_Config memlcdConf = MEMLCD_CONFIG_STK3700_EXPBOARD;

/* Temporary storage for the USART CLKDIV register. Needed when changing clock 
 * frequency. */
static uint32_t usart_clk_div;


void initLETIMER(void)
{
  CMU_ClockEnable(cmuClock_LETIMER0, true);  
  
  GPIO_PinModeSet(memlcdConf.extcomin.port, memlcdConf.extcomin.pin, gpioModePushPull, 0);
    
  /* Set initial compare values for COMP0 */
  LETIMER_CompareSet(LETIMER0, 0, 32768);

  /* Route LETIMER to location 3 (OUT0 - PC4) and enable output */
  LETIMER0->ROUTE = LETIMER_ROUTE_OUT0PEN | LETIMER_ROUTE_LOCATION_LOC3;
    
  /* The current version of emlib (3.0.0) does not properly set 
   * REP0 to a nonzero value while enabling LETIMER in Free mode. 
   * Therefore we set REP0 manually here */
  LETIMER0->REP0 = 0x01;
  
  /* Set configurations for LETIMER 0 */
  const LETIMER_Init_TypeDef letimerInit = 
  {
  .enable         = true,                   /* Don't start counting when init completed - only with RTC compare match */
  .debugRun       = false,                  /* Counter shall not keep running during debug halt. */
  .rtcComp0Enable = false,                  /* Don't start counting on RTC COMP0 match. */
  .rtcComp1Enable = false,                  /* Don't start counting on RTC COMP1 match. */
  .comp0Top       = true,                   /* Load COMP0 register into CNT when counter underflows. COMP is used as TOP */
  .bufTop         = false,                  /* Don't load COMP1 into COMP0 when REP0 reaches 0. */
  .out0Pol        = 0,                      /* Idle value for output 0. */
  .out1Pol        = 0,                      /* Idle value for output 1. */
  .ufoa0          = letimerUFOAToggle,      /* Pulse output on output 0 */
  .ufoa1          = letimerUFOANone,        /* No output on output 1*/
  .repMode        = letimerRepeatFree       /* Repeat indefinitely */
  };
  
  /* Initialize LETIMER */
  LETIMER_Init(LETIMER0, &letimerInit); 
}



/* Set the HFRCO band to a 'low' frequency. This is used when transferring
 * the frame over SPI with DMA */
void setLowFrequency(void)
{
  usart_clk_div = memlcdConf.usart->CLKDIV;
  
  CMU_HFRCOBandSet(CORE_FREQ_LOW);  
  memlcdConf.usart->CLKDIV = USART_LOW_FREQ_CLKDIV;
}

/* Set the HFRCO to 'high frequency'. The high frequency is used 
 * when the CPU is working */
void setHighFrequency(void)
{
  CMU_HFRCOBandSet(CORE_FREQ_HIGH);
  memlcdConf.usart->CLKDIV = usart_clk_div;
}


int main(void)
{  
  CHIP_Init(); 

  /* Start the clock to interface low frequency peripherals */
  CMU_ClockEnable(cmuClock_CORELE, true);  
  
  /* Set up DMA for frame transfer to the MemoryLCD */
  initDMA(&memlcdConf);
   
  /* Power down unused RAM blocks. This can save additional power
   * if the memory is not used */
  //EMU->MEMCTRL = EMU_MEMCTRL_POWERDOWN_BLK123;
  
  /* Initialize the MemoryLCD */
  MEMLCD_Init(&memlcdConf);
  
  /* Init the BURTC to keep track of the current time */
  initClock();
  
  /* Init the TIMER to handle slide animations */
  initTimerForSlide();
  
  /* Initialize the list of screens */
  initScreens();
   
  /* Start LETIMER to toggle EXTCOMIN @ 1 Hz for the MemoryLCD */
  initLETIMER();
  
  /* Start LESENSE to measure the capacitive touch slider */
  CAPLESENSE_Init();
  CAPLESENSE_setupCallbacks(&capSenseScanComplete, &capSenseChTrigger);
  
  /* Initialize emWin */
  GUI_Init();
  
  /* Prerender frames for the animation */
  ANIM_prerenderFrames();
  
  /* Enter draw loop */
  drawLoop();
  
  return 0;
}
