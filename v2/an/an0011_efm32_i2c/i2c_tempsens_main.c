/**************************************************************************//**
 * @file i2c_tempsens_main.c
 * @brief Temperature example for EFM32_G8xx_DK
 * @details Show temperature using sensor on DVK.
 * @author Silicon Labs
 * @version 1.07
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

#include <string.h>
#include <stdlib.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"
#include "segmentlcd.h"
#include "rtcdrv.h"
#include "i2c_tempsens.h"

/** Flag used to indicate if displaying in Celsius or Fahrenheit */
#define SHOW_FAHRENHEIT 0

/* Local prototypes */
void temperatureUpdateLCD(TEMPSENS_Temp_TypeDef *temp);

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
  TEMPSENS_Temp_TypeDef temp;

  /* Chip revision alignment and errata fixes */
  CHIP_Init();
  
  /* Initialize LCD controller without boost */
  SegmentLCD_Init(false);

  I2C_Tempsens_Init();

  /* Main loop - just read temperature and update LCD */
  while (1)
  {
    if (TEMPSENS_TemperatureGet(I2C0,
                                TEMPSENS_DVK_ADDR,
                                &temp) < 0)
    {
      SegmentLCD_Write("ERROR");
      /* Enter EM2, no wakeup scheduled */
      EMU_EnterEM2(true);
    }

    /* Update LCD display */ 
    temperatureUpdateLCD(&temp);

    /* Read every 2 seconds which is more than it takes worstcase to */
    /* finish measurement inside sensor. */
    RTCDRV_Trigger(2000, NULL);
    EMU_EnterEM2(true);
  }
}

/**************************************************************************//**
 * @brief Update LCD with temperature
 * @param[in] temp Temperature to display.
 *****************************************************************************/
void temperatureUpdateLCD(TEMPSENS_Temp_TypeDef *temp)
{
  char text[8];
  TEMPSENS_Temp_TypeDef dtemp;

  /* Work with local copy in case conversion to Fahrenheit is required */
  dtemp = *temp;

  memset(text, ' ', sizeof(text) - 1);
  text[sizeof(text) - 1] = 0;

  if (SHOW_FAHRENHEIT)
  {
    text[5] = 'F';
    TEMPSENS_Celsius2Fahrenheit(&dtemp);
  }
  else
  {
    text[5] = 'C';
  }

  /* Round temperature to nearest 0.5 */
  if (dtemp.f >= 0)
  {
    dtemp.i += (dtemp.f + 2500) / 10000;
    dtemp.f = (((dtemp.f + 2500) % 10000) / 5000) * 5000;
  }
  else
  {
    dtemp.i += (dtemp.f - 2500) / 10000;
    dtemp.f = (((dtemp.f - 2500) % 10000) / 5000) * 5000;
  }

  /* 100s */
  if (abs(dtemp.i) >= 100)
    text[0] = '0' + (abs(dtemp.i) / 100);

  /* 10s */
  if (abs(dtemp.i) >= 10)
    text[1] = '0' + ((abs(dtemp.i) % 100) / 10);

  /* 1s */
  text[2] = '0' + (abs(dtemp.i) % 10);

  /* 0.1s */
  text[3] = '0' + (abs(dtemp.f) / 1000);
   
  SegmentLCD_Write(text);
  SegmentLCD_Symbol(LCD_SYMBOL_DP4, 1);

  if ((dtemp.i < 0) || (dtemp.f < 0))
  {
    SegmentLCD_Symbol(LCD_SYMBOL_MINUS, 1);
  }
  else
  {
    SegmentLCD_Symbol(LCD_SYMBOL_MINUS, 0);
  }
}
