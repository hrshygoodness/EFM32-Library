/*!
 * File:
 *  spi.c
 *  author: Shaoxian Luo
 *
 * Description:
 *  This file is the drivers of the kit.
 *
 * Silicon Laboratories Confidential
 * Copyright 2013 Silicon Laboratories, Inc.
 */

#include "bsp_def.h"


volatile uint32_t msTicks; /* counts 1ms timeTicks */

/**************************************************************************//**
 * @brief SysTick_Handler
 * Interrupt Service Routine for system tick counter
 *****************************************************************************/
void SysTick_Handler(void)
{
  msTicks++;       /* increment counter necessary in Delay()*/
}

uint32_t SysTick_GetTick(void)
{
  return msTicks;
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

uint8_t BSP_GetPB(uint8_t biPbNum)
{
  uint8_t s = 0;
  
  switch(biPbNum)
  {
    case 1:
      s = GPIO_PinInGet(PUSH_BUTTON0_PORT, PUSH_BUTTON0_PIN);
      break;
    case 2:
#ifdef EFM32LG990F256
      s = GPIO_PinInGet(PUSH_BUTTON1_PORT, PUSH_BUTTON1_PIN);
#endif
      
#ifdef EFM32TG840F32
      s = 0;
#endif      
      break;
    case 3:
      s = 0;
      break;
    case 4:
      s = 0;
      break;
    default:
      s = 0;
      break;
  }

  //return with the value of the button
  return s;
}

