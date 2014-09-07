/******************************************************************************
 * @file main_idac_demo.c
 * @brief IDAC demo
 * @author Silicon Labs
 * @version 1.01
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
#include <stdio.h>
#include "em_device.h"
#include "em_idac.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_emu.h"

#include "display.h"
#include "retargettextdisplay.h"
#include "textdisplay.h"

#define IDAC_STEPSEL_MAX (_IDAC_CURPROG_STEPSEL_MASK >> _IDAC_CURPROG_STEPSEL_SHIFT)

/* IDAC current range base current in nA */
uint32_t idacRangeStart[4] = {50, 1600, 500, 2000}; 

/* IDAC current step size in nA */
uint32_t idacStepSize[4] = {50, 100, 500, 2000}; 


/* These values are used to program IDAC current output. 
 * They are changed by the GPIO interrupt handlers.
 */
static volatile IDAC_Range_TypeDef idacRange = idacCurrentRange0; 
static volatile uint32_t idacStep = 0;


/* Flag that keeps track of when display should be updated */
static volatile bool update = true;

/* Use iprintf on GCC for smaller codesize */ 
#if !defined(__ICCARM__) && !defined (__CC_ARM) && !defined (__CROSSWORKS_ARM)
  #define printf iprintf
#endif



/**************************************************************************//**
 * @brief updateScreen
 * Prints IDAC information to screen
 *****************************************************************************/
void displayUpdate(void)
{
  uint32_t current = 0;

  /* Calculate current in nA*/ 
  switch( idacRange )
  {
  case idacCurrentRange0:
    current = idacRangeStart[0] + idacStep*idacStepSize[0];
    break;
  case idacCurrentRange1:
    current = idacRangeStart[1] + idacStep*idacStepSize[1];
    break;
  case idacCurrentRange2:
    current = idacRangeStart[2] + idacStep*idacStepSize[2];
    break;
  case idacCurrentRange3:
    current = idacRangeStart[3] + idacStep*idacStepSize[3];
    break; 
  default:
    current = 99999;
    break;
  }  

  /* Print to display */
  printf( TEXTDISPLAY_ESC_SEQ_CURSOR_HOME_VT100 );
  printf("\n\n EFM32_ZG_IDAC\n\n\n\n" );
  printf("  Range: %2ld\n", (uint32_t)idacRange );
  printf("  Step:  %2ld\n\n\n", idacStep );
  printf("Current:%5ld nA\n\n\n\n", current );
  printf("   Range  Step\n");
  printf("   PB1    PB0");
  
  update = false;
}

/**************************************************************************//**
 * @brief GPIO_EVEN_IRQHandler
 * Interrupt Service Routine Even GPIO, increments IDAC Step
 *****************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{ 
  /* clear flag for PC8 interrupt */
  GPIO_IntClear(1<<8);
  
  idacStep++;
  /* if over max value, wrap around */
  if( idacStep > IDAC_STEPSEL_MAX)
	{
		idacStep = 0;
	}
 
  IDAC_RangeSet(IDAC0, idacRange );
  IDAC_StepSet(IDAC0, idacStep );
  
  update = true;
}

/**************************************************************************//**
 * @brief GPIO_ODD_IRQHandler
 * Interrupt Service Routine Odd GPIO, Increments IDAC Range
 *****************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  /* clear flag for PC9 interrupt */
  GPIO_IntClear(1<<9);
  
	switch( idacRange )
  {
  case idacCurrentRange0:
		idacRange = idacCurrentRange1;
    break;
  case idacCurrentRange1:
		idacRange = idacCurrentRange2;
    break;
  case idacCurrentRange2:
		idacRange = idacCurrentRange3;
    break;
  case idacCurrentRange3:
		idacRange = idacCurrentRange0;
    break; 
  default:
		idacRange = idacCurrentRange0;
		break;
  }  
	
  IDAC_RangeSet(IDAC0, idacRange);
  IDAC_StepSet(IDAC0, idacStep);
  
  update = true;
}

/**************************************************************************//**
 * @brief idacSetup
 * Configures the IDAC to output current on pin
 *****************************************************************************/
void idacSetup()
{
  /* Enable IDAC clock */
  CMU_ClockEnable(cmuClock_IDAC0, true);
 
  IDAC_Init_TypeDef idacInit = 
  {
    .enable 		= true,
    .outMode 		= idacOutputPin,
    .prsEnable 	= false,
    .prsSel 		= idacPRSSELCh0,
    .sinkEnable = false,
  };
  
  IDAC_Init(IDAC0, &idacInit);
  
  IDAC_RangeSet(IDAC0, idacRange);
  IDAC_StepSet(IDAC0, idacStep);
  
  /* Enable IDAC Current out */
  IDAC_OutEnable(IDAC0, true); 
}

/**************************************************************************//**
 * @brief gpioSetup
 * Sets up interrupt on the push buttons
 *****************************************************************************/
void gpioSetup()
{
  /* ZG STK:
	 * UIF_PB0 = PC8, 
	 * UIF_PB1 = PC9 */  
  
  /* Enable clock for GPIO module */
  CMU_ClockEnable(cmuClock_GPIO, true);
  
  /* Configure PC8 and PC9 as an input for PB0 and PB1 button with filter enable (out = 1)*/
  GPIO_PinModeSet(gpioPortC, 8, gpioModeInput, 1);
  GPIO_PinModeSet(gpioPortC, 9, gpioModeInput, 1);
  
  /* Configure PC8 interrupt on falling edge */
  GPIO_IntConfig(gpioPortC, 8, false, true, true);  
  
  /* Configure PC9 interrupt on falling edge */
  GPIO_IntConfig(gpioPortC, 9, false, true, true);    
	
	/* Enable GPIO_EVEN interrupt vector in NVIC */
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  
  /* Enable GPIO_ODD interrupt vector in NVIC */
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
}


/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  /* Chip errata */
  CHIP_Init();
  
 /* Initialize the display module. */
  DISPLAY_Init();

  /* Retarget stdio to the display. */
  RETARGET_TextDisplayInit();
  
  /* Setup GPIO for UI buttons */
  gpioSetup();
  
  /* Initialize IDAC */
  idacSetup(); 
   
  
  while (1)
  {
    if( update )
    {
      displayUpdate();
    }
    EMU_EnterEM2(false);
  }
}

