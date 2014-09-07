/**************************************************************************//**
 * @file main_decoder_nb8k.c
 * @brief SPEEX NB 8KHz decoder demo for EFM32LG/GG
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
#include "clock.h"
#include "dac.h"
#include "dma.h"
#include "speexfunc.h"
#include "timer.h"

//#define GG_STK

#ifdef GG_STK
#include "gpio.h"
#else
#include "bsp.h"
#endif

#include "voice8k.h"

struct speexEncFile playList[LIST_SIZE];

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int main(void)
{
#ifndef GG_STK
  /* Initialize DK board register access, necessary if run on DK */
  BSP_Init(BSP_INIT_DEFAULT);
  BSP_PeripheralAccess(BSP_AUDIO_OUT, true);
#endif
  
  /* Initialize peripherals */
  initClock();
  initDma();
  initDac();
  initDacDma();
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
    EMU_EnterEM1();

    switch (keyCheck())
    {
    case 2:
      /* Start play files */      
      playList[0].frameStart = (char *)voice8k;
      playList[1].frameStart = NULL;
      playList[0].frameNum = sizeof(voice8k)/FRAME_SIZE_8K;
      speexPlayBack(0, NARROWBAND8K, FRAME_SIZE_8K, &playList[0]);
      break;
      
    default:
      break;
    }
#endif
  }
}
