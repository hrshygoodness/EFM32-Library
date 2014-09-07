/**************************************************************************//**
 * @file main_codec_nb8k.c
 * @brief SPEEX codec demo for EFM32GG
 * @author Silicon Labs
 * @version 1.06
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/


#include <stdlib.h>
#include <stdbool.h>

#include "em_device.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "adc.h"
#include "clock.h"
#include "dac.h"
#include "dma.h"
#include "flash.h"
#include "speexfunc.h"
#include "timer.h"

//#define GG_STK

#ifdef GG_STK
#include "gpio.h"
#else
#include "bsp.h"
#include "voice8k.h"
#endif

struct speexEncFile playList[LIST_SIZE];

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
#ifndef GG_STK
  /* Initialize DK board register access, necessary if run on DK */
  BSP_Init(BSP_INIT_DEFAULT);
  BSP_PeripheralAccess(BSP_AUDIO_IN, true);
  BSP_PeripheralAccess(BSP_AUDIO_OUT, true);
#endif
  
  /* Initialize peripherals */
  initClock();
  initDma();
  initAdc();
  initAdcDma();
  initDac();
  initDacDma();
  initFlashDma();
  initTimer();
#ifdef GG_STK
  initGpio();
#endif

  while(1)
  {
/*** Operation for EFM32GG-DK3750 ***/    
#ifndef GG_STK
    /* Wait key press */
    while(!BSP_PushButtonsGet())
      ;
    if (BSP_PushButtonsGet() == BC_UIF_PB1)
    {
      BSP_LedsSet(0x0001);
      /* Wait key release */
      while(BSP_PushButtonsGet()!=0);
      /* Start record */      
      speexRecordFlash512(1, RECORD_FRAMES);
      BSP_LedsSet(0x0000);

      /* Start playback */      
      BSP_LedsSet(0x0100);
      playList[0].frameStart = (char *)FLASHSTART;
      playList[1].frameStart = NULL;
      playList[0].frameNum = RECORD_FRAMES;
      speexPlayBack(1, NARROWBAND8K, FRAME_SIZE_8K, &playList[0]);
      BSP_LedsSet(0x0000);
    }
    if (BSP_PushButtonsGet() == BC_UIF_PB2)
    {    
      BSP_LedsSet(0x0100);
      /* Wait key release */
      while(BSP_PushButtonsGet()!=0);
      /* Start playback */      
      playList[0].frameStart = (char *)FLASHSTART;
      playList[1].frameStart = NULL;
      playList[0].frameNum = RECORD_FRAMES;
      speexPlayBack(1, NARROWBAND8K, FRAME_SIZE_8K, &playList[0]);
      BSP_LedsSet(0x0000);
    }
    if (BSP_PushButtonsGet() == BC_UIF_PB3)
    {    
      BSP_LedsSet(0x0002);
      /* Wait key release */
      while(BSP_PushButtonsGet()!=0);
      /* Start play files */      
      playList[0].frameStart = (char *)voice8k;
      playList[1].frameStart = NULL;
      playList[0].frameNum = sizeof(voice8k)/FRAME_SIZE_8K;
      speexPlayBack(0, NARROWBAND8K, FRAME_SIZE_8K, &playList[0]);
      BSP_LedsSet(0x0000);
    }
    
/*** Operation for EFM32GG-STK3750 ***/        
#else
    /* Wait key press */
    EMU_EnterEM3(true);

    switch (keyCheck())
    {
    case 1:
      GPIO_PinOutSet(gpioPortE, 3);
      /* Start record */      
      speexRecordFlash512(1, RECORD_FRAMES);
      GPIO_PinOutClear(gpioPortE, 3);

      /* Start playback */      
      GPIO_PinOutSet(gpioPortE, 2);
      playList[0].frameStart = (char *)FLASHSTART;
      playList[1].frameStart = NULL;
      playList[0].frameNum = RECORD_FRAMES;
      speexPlayBack(1, NARROWBAND8K, FRAME_SIZE_8K, &playList[0]);
      GPIO_PinOutClear(gpioPortE, 2);
      break;
      
    case 2:
      GPIO_PinOutSet(gpioPortE, 2);
      /* Start playback */      
      playList[0].frameStart = (char *)FLASHSTART;
      playList[1].frameStart = NULL;
      playList[0].frameNum = RECORD_FRAMES;
      speexPlayBack(1, NARROWBAND8K, FRAME_SIZE_8K, &playList[0]);
      GPIO_PinOutClear(gpioPortE, 2);
      break;
      
    default:
      break;
    }
#endif    
  }
}
