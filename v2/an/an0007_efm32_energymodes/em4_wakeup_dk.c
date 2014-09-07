/******************************************************************************
 * @file em4_wakeup_dk.c
 * @brief EM4 Wakeup Example
 * @author Silicon Labs
 * @version 1.09
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
#include "em_device.h"
#include "em_chip.h"
#include "em_rtc.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_burtc.h"
#include "em_rmu.h"
#include "em_gpio.h"
#include "bsp.h"
#include "tftamapped.h"
#include "glib.h"


/* Set sleep time */
#define ULFRCO_FREQUENCY       	1000
#define BURTC_TIMEOUT_S         5
#define BURTC_COUNT_TO_WAKEUP   (ULFRCO_FREQUENCY * BURTC_TIMEOUT_S)

/* Strings */
#define STR_WAKEUP_BURTC  "Woke up from: BURTC"
#define STR_WAKEUP_RESET  "Woke up from: RESET"
#define STR_SLEEPING      "Sleeping..."
#define STR_WAKEUP_GPIO    "Woke up from: GPIO"


/* The retention registers used for counter values */
#define BURTC_WAKEUP_COUNT BURTC->RET[0].REG
#define GPIO_WAKEUP_COUNT BURTC->RET[1].REG

/* System ms tick counter */
uint32_t msTicks = 0;

static GLIB_Context_t gc;


/** 
 * @brief Triggered every ms
 */
void SysTick_Handler(void)
{
  msTicks++;       /* increment counter necessary in Delay()*/
}


/**************************************************************************//**
 * @brief Delays number of msTick Systicks (typically 1 ms)
 * @param dlyTicks Number of ticks to delay
 *****************************************************************************/
void Delay(uint32_t dlyTicks)
{
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks) ;
} 

/** Enables all the clocks needed for this example */
void enableClocks(void) 
{
  CMU_ClockEnable(cmuClock_CORELE, true);
  CMU_ClockEnable(cmuClock_GPIO, true);
}

/** Configure settings for EM4 before going to sleep */
void configEM4(void) 
{
  EMU_EM4Init_TypeDef em4Init = EMU_EM4INIT_DEFAULT;

  em4Init.lockConfig    = true;              		/* Lock regulator, oscillator and BOD configuration. 
                                                         * This needs to be set when using the 
                                                         * voltage regulator in EM4 */																
  em4Init.osc           = emuEM4Osc_ULFRCO;             /* Select ULFRCO */
  em4Init.buRtcWakeup   = true;                         /* BURTC compare or overflow will generate reset */
  em4Init.vreg          = true;                         /* Enable voltage regulator. Needed for BURTC */
  EMU_EM4Init( &em4Init );
}

/** Set up BURTC to count and trigger a wakeup in EM4 */
void configBURTC(void) 
{
  BURTC_Init_TypeDef burtcInit = BURTC_INIT_DEFAULT; 
  
  burtcInit.mode      = burtcModeEM4;       		/* BURTC is enabled in EM0-EM4 */
  burtcInit.clkSel    = burtcClkSelULFRCO;  		/* Select ULFRCO as clock source */
  burtcInit.clkDiv    = burtcClkDiv_2;      		/* Choose 1kHz ULFRCO clock frequency */
  
  BURTC_CompareSet(0, BURTC_COUNT_TO_WAKEUP);  	        /* Set top value for comparator */
  BURTC_IntEnable( BURTC_IF_COMP0 );		        /* Enable compare interrupt flag */
  BURTC_Init(&burtcInit);
}

/** Enable wakeup from a GPIO pin in EM4 */
void configGPIO(void) 
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  
  /* Set pin PF2 mode to input with pull-up resistor */
  GPIO_PinModeSet(gpioPortF, 2, gpioModeInput, 1);
  
  /* Enable GPIO pin mode retention in EM4 */
  GPIO->CTRL |= GPIO_CTRL_EM4RET;
  
  /* Clear wake up requests */
  GPIO->CMD |= GPIO_CMD_EM4WUCLR;
  
  /* Enable wakeup on PF2 */
  GPIO->EM4WUEN = GPIO_EM4WUEN_EM4WUEN_F2;
}

int main(void)
{
  char burtcCounterString[40];
  char gpioCounterString[40];
  
  /* Chip errata */
  CHIP_Init();
  
  /* Enable the neccesary clocks */
  enableClocks();
  
  /* Enable board controller on DK. Needed to use display */
  BSP_Init(BSP_INIT_DEFAULT);
  
  /* Enable SysTick interrupt, necessary for Delay() */
  if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000)) while (1) ;
  
  /* Initialize TFT */
  while (!TFT_AddressMappedInit());
  
  /* Initialize GLIB */
  if ( GLIB_contextInit(&gc) != GLIB_OK )
  {
    while(1);
  }
  
  /* Clear background */
  gc.backgroundColor = Black;
  GLIB_clear(&gc);
  gc.foregroundColor = White;
  
  /* Get the cause of reset and clear the flags */
  uint32_t resetCause = RMU->RSTCAUSE;
  RMU_ResetCauseClear();

  /* Enable access to BURTC registers */
  RMU_ResetControl(rmuResetBU, false);
  BURTC_Enable(true);
  
  /* Check if reset was caused by a wakeup from EM4 */
  if ( resetCause & RMU_RSTCAUSE_EM4WURST ) 
  {
    /* The reset is a wakeup from EM4 */
    if ( GPIO->EM4WUCAUSE & GPIO_EM4WUCAUSE_EM4WUCAUSE_F2 ) 
    {
      /* The wakeup was triggered by GPIO. Increase counter. */
      GLIB_drawString( &gc, STR_WAKEUP_GPIO, strlen(STR_WAKEUP_GPIO), 0, 0, 1 );
      GPIO_WAKEUP_COUNT++;
    }  
    else 
    {
      /* The reset was triggered by BURTC. Increase counter. */
      GLIB_drawString( &gc, STR_WAKEUP_BURTC, strlen(STR_WAKEUP_BURTC), 0, 0, 1 );
      BURTC_WAKEUP_COUNT++;
    }
  } 
  else 
  {
    /* A different reset was triggered. Reset counters. */
    GLIB_drawString( &gc, STR_WAKEUP_RESET, strlen(STR_WAKEUP_RESET), 0, 0, 1 );
    BURTC_WAKEUP_COUNT = 0;
    GPIO_WAKEUP_COUNT = 0;    
  }
  
  /* Print the number of times woken up by BURTC or GPIO */
  sprintf(burtcCounterString, "BURTC wakeup count: %d", (int) BURTC_WAKEUP_COUNT);
  sprintf(gpioCounterString, "GPIO wakeup count: %d", (int) GPIO_WAKEUP_COUNT);
  GLIB_drawString( &gc, burtcCounterString, strlen(burtcCounterString), 0, 16, 1 );
  GLIB_drawString( &gc, gpioCounterString, strlen(gpioCounterString), 0, 32, 1 );
  Delay(5000);
	
  /* Initialize GPIO, BURTC and EM4 registers before going to sleep */
  configEM4();
  configBURTC();
  configGPIO();

  /* Print a message while sleeping */  
  GLIB_clear(&gc);
  GLIB_drawString( &gc, STR_SLEEPING, strlen(STR_SLEEPING), 0, 0, 1 );  
  
  /* Enter Shutoff Mode. BURTC or GPIO will trigger wakeup */
  EMU_EnterEM4();
  
  /* Never reached */
  return 0;
}
