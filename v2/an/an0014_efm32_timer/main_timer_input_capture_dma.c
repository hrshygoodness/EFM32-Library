/*****************************************************************************
 * @file main_timer_input_capture_dma.c
 * @brief TIMER Input Capture with DMA Demo Application
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

#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_prs.h"
#include "em_dma.h"
#include "em_system.h"
#include "em_timer.h"
#include "em_chip.h"
#include "stdio.h"
#include "dmactrl.h"

#define DMA_CHANNEL_TIMER0       0
#define BUFFERSIZE               8

volatile uint16_t bufferA[BUFFERSIZE];
volatile uint16_t bufferB[BUFFERSIZE];

/* DMA callback structure */
DMA_CB_TypeDef cb;

/* TOP reset value is 0xFFFF so it doesn't need 
   to be written for this example */
#define TOP 0xFFFF

/* 13761 Hz -> 14Mhz (clock frequency) / 1024 (prescaler) */
#define TIMER_FREQ 13671

#if defined ( STK3700 )
  #define PB0_PORT gpioPortB
  #define PB0_PIN 9 
  #define PB0_PRS_SIGSEL_PIN PRS_CH_CTRL_SIGSEL_GPIOPIN9
#elif defined ( STK3300 )
  #define PB0_PORT gpioPortD
  #define PB0_PIN 8
  #define PB0_PRS_SIGSEL_PIN PRS_CH_CTRL_SIGSEL_GPIOPIN8
#elif defined ( STKG8XX )
  #define PB0_PORT gpioPortB
  #define PB0_PIN 9
  #define PB0_PRS_SIGSEL_PIN PRS_CH_CTRL_SIGSEL_GPIOPIN9
#else
  #error "undefined KIT"
#endif


/**************************************************************************//**
 * @brief  Call-back called when transfer is complete
 *****************************************************************************/
void transferComplete(unsigned int channel, bool primary, void *user)
{
  (void) user;
  
  /* Re-activate the DMA */
  DMA_RefreshPingPong(channel,
                      primary,
                      false,
                      NULL,
                      NULL,
                      BUFFERSIZE - 1,
                      false);
}



/**************************************************************************//**
 * @brief Configure DMA for Ping-Pong transfers
 *****************************************************************************/
void setupDma(void)
{
  DMA_Init_TypeDef        dmaInit;
  DMA_CfgChannel_TypeDef  chnlCfg;
  DMA_CfgDescr_TypeDef    descrCfg;
  
  /* Initializing the DMA */
  dmaInit.hprot        = 0;
  dmaInit.controlBlock = dmaControlBlock;
  DMA_Init(&dmaInit);
  
  /* Setup call-back function */  
  cb.cbFunc  = transferComplete;
  cb.userPtr = NULL;

  /* Setting up channel */
  chnlCfg.highPri   = false;
  chnlCfg.enableInt = true;
  chnlCfg.select    = DMAREQ_TIMER0_CC0;
  chnlCfg.cb        = &cb;
  DMA_CfgChannel(DMA_CHANNEL_TIMER0, &chnlCfg);

  /* Setting up channel descriptor */
  descrCfg.dstInc  = dmaDataInc2;
  descrCfg.srcInc  = dmaDataIncNone;
  descrCfg.size    = dmaDataSize2;
  descrCfg.arbRate = dmaArbitrate1;
  descrCfg.hprot   = 0;
  DMA_CfgDescr(DMA_CHANNEL_TIMER0, true, &descrCfg);
  DMA_CfgDescr(DMA_CHANNEL_TIMER0, false, &descrCfg);

  /* Enabling PingPong Transfer*/  
  DMA_ActivatePingPong(DMA_CHANNEL_TIMER0,
                       false,
                       (void *)&bufferA,
                       (void *)&(TIMER0->CC[0].CCV),
                       BUFFERSIZE - 1,
                       (void *)&bufferB,
                       (void *)&(TIMER0->CC[0].CCV),
                        BUFFERSIZE - 1);
}


/**************************************************************************//**
 * @brief  Main function
 * Main is called from __iar_program_start, see assembly startup file
 *****************************************************************************/
int main(void)
{  
  /* Initialize chip */
  CHIP_Init();
    
  /* Enable clock for GPIO module */
  CMU_ClockEnable(cmuClock_GPIO, true);
  
  /* Enable clock for TIMER0 module */
  CMU_ClockEnable(cmuClock_TIMER0, true);
   
  /* Enable clock for PRS module */
  CMU_ClockEnable(cmuClock_PRS, true);
  
  /* Setup DMA */
  setupDma();
  
  /* Configure PB0_PIN as an input for PB0 button with filter and pull-up (dout = 1)*/
  GPIO_PinModeSet(PB0_PORT, PB0_PIN, gpioModeInputPullFilter, 1); 
  
  /* Select PB0_PIN as external interrupt source*/
  GPIO_IntConfig(PB0_PORT, PB0_PIN, false, false, false);

  /* Enable PRS sense on GPIO and disable interrupt sense */
  GPIO_InputSenseSet(GPIO_INSENSE_PRS, _GPIO_INSENSE_RESETVALUE);
  
  /* Select GPIO as source and PB0_PRS_SIGSEL_PIN as signal for PRS channel 0 */
  PRS_SourceSignalSet(0, PRS_CH_CTRL_SOURCESEL_GPIOH, PB0_PRS_SIGSEL_PIN, prsEdgeOff);
    
  /* Select CC channel parameters */
  TIMER_InitCC_TypeDef timerCCInit = 
  {
    .eventCtrl  = timerEventEveryEdge,
    .edge       = timerEdgeRising,
    .prsSel     = timerPRSSELCh0,
    .cufoa      = timerOutputActionNone,
    .cofoa      = timerOutputActionNone,
    .cmoa       = timerOutputActionNone,
    .mode       = timerCCModeCapture,
    .filter     = true,
    .prsInput   = true,
    .coist      = false,
    .outInvert  = false,
  };
  
  /* Configure CC channel 0 */
  TIMER_InitCC(TIMER0, 0, &timerCCInit);

  /* Select timer parameters */  
  TIMER_Init_TypeDef timerInit =
  {
    .enable     = false,
    .debugRun   = true,
    .prescale   = timerPrescale1024,
    .clkSel     = timerClkSelHFPerClk,
    .fallAction = timerInputActionReloadStart,
    .riseAction = timerInputActionStop,
    .mode       = timerModeUp,
    .dmaClrAct  = false,
    .quadModeX4 = false,
    .oneShot    = false,
    .sync       = false,
  };
  
  /* Configure timer */
  TIMER_Init(TIMER0, &timerInit);
   
  while(1)
  {
    /* Go to EM1 */
    EMU_EnterEM1();
  }
  
}
