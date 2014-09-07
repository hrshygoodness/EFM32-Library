/******************************************************************************
 * @file lcd_hello_world.c
 * @brief LCD Hello World Example
 * @author Silicon Labs
 * @version 1.04
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

#include "em_device.h"
#include "em_lcd.h"
#include "segmentlcd.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_cmu.h"

/**************************************************************************//**
 * @brief  Sets up LCD animation
 *****************************************************************************/
void SetupAnimation(void)
{
  /* Configuration structure for LCD animation. */
  static const LCD_AnimInit_TypeDef animInit =
  {
    .enable      = true,                  /* Enable animation. */
    .AReg        = 0x00,                  /* Set up animation start data. */
    .AShift      = lcdAnimShiftLeft,      /* Register A Shift direction. */
    .BReg        = 0x01,                  /* Set up animation start data. */
    .BShift      = lcdAnimShiftRight,     /* Register B Shift direction. */
    .animLogic   = lcdAnimLogicOr,        /* Logic function. */
    
  /* Adapt animation segments to the different kits. */
  /* Leave at default for other kits: STK G8xx, STK3300, DK3550, DK G8xx */
  
  /* Note that STK3700 only has half the animation ring connected to animation */
  /* enabled segments. */    
  #if defined( STK3700 )                       
    .startSeg    = 8                      /* STK3700 has animation circle on seg8-15. */
  #endif    
  };

   /* Configuration structure for frame counter. */
  static const LCD_FrameCountInit_TypeDef fcInit =
  {
    .enable      = true,                  /* Enable frame counter. */
    .top         = 0x3,                   /* Frame counter period. */
    .prescale    = lcdFCPrescDiv1         /* Set frame counter prescaler. */
  }; 
  
  /* Initialize Animation */
  LCD_AnimInit(&animInit);

  /* Configure and start the framecounter which clocks the animation state machine. */
  LCD_FrameCountInit(&fcInit);
}

/******************************************************************************
 * @brief  Main function
 *****************************************************************************/
int main(void)
{  
  /* Initialize chip */
  CHIP_Init();  
  
  /* Initialize LCD controller without boost. */
  SegmentLCD_Init(false);
  
  /* Write something to text field on LCD display. */
  SegmentLCD_Write("HELLO");
      
  /* Turn on Gecko symbol. */
  SegmentLCD_Symbol(LCD_SYMBOL_GECKO, 1);
    
  /* Configure Animation. */
  SetupAnimation();
  
  /* Enable segment blinking. */
  LCD_BlinkEnable(false);        /* Set to true to see blink feature. */ 
      
  /* Play around with settings below to adjust contrast and display refreshrate. */
  /* These settings affect the current consumption of LCD. */
  
  /* Set contrast to suitable level for 3.3V. */
  LCD_ContrastSet(0x1A);
    
  /* LCD Controller Prescaler (divide LFACLK / 64) */
  /* LFACLK_LCDpre = 512 Hz */
  /* Set FDIV=0, means 512/1 = 512 Hz */
  /* With octaplex mode, 512/16 => 32 Hz Frame Rate */  
  CMU_ClockDivSet(cmuClock_LCDpre, cmuClkDiv_64);     /* divide LFACLK / 64 */
  CMU_LCDClkFDIVSet(cmuClkDiv_1);                     /* FDIV = 0 */
    
  /* Enable boost converter. To save energy, first try to adjust framerate */
  /* and contrast to maximum, in many cases this is enough to account for lower */
  /* supply voltage. */
  if(0){
    /* Set Vboost level, for lowest current consumption, set as low as possible. */
    LCD_VBoostSet(lcdVBoostLevel3);   
    LCD_VLCDSelect(lcdVLCDSelVExtBoost);  /* Select External LCD supply. */
    
    /* Adjust boost converter frequency, can be adjusted to reduce current consumption. */
    CMU->LCDCTRL = (CMU->LCDCTRL & (~_CMU_LCDCTRL_VBFDIV_MASK)) | CMU_LCDCTRL_VBFDIV_DIV4;
    
    CMU->LCDCTRL |= CMU_LCDCTRL_VBOOSTEN; /* Enable boost converter. */
  }
    
  /* Stay in this loop forever at end of program. LCD works autonomously with animation in EM2. */
  while (1){
     EMU_EnterEM2(false);
  }
}



