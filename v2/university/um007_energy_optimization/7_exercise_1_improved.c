/**************************************************************************//**
 * @file
 * @brief Exercise 1 - Improved Version
 * @author Energy Micro AS
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silicon Labs Software License Agreement. See 
 * "http://developer.silabs.com/legal/version/v11/Silicon_Labs_Software_License_Agreement.txt"  
 * for details. Before using this software for any purpose, you must agree to the 
 * terms of that agreement.
 *
 ******************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include "efm32.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_lcd.h"
#include "em_rtc.h"
#include "em_gpio.h"

/* Drivers */
#include "segmentlcd.h"

#if defined(_EFM32_GIANT_FAMILY)

/* Defines for Push Button 0 */
#define PB0_PORT    gpioPortB
#define PB0_PIN     9

#else 

/* Defines for Push Button 0 */
#define PB0_PORT    gpioPortD
#define PB0_PIN     8

#endif

/* Time of the stopwatch */
uint32_t time;

int main(void)
{
	return 0;
}

/* Enter your code here */


