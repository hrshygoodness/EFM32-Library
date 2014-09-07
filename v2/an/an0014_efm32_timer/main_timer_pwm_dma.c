/*****************************************************************************
 * @file main_timer_pwm_dma.c
 * @brief TIMER Pulse Width Modulation with DMA Demo Application
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
#include "em_dma.h"
#include "em_prs.h"
#include "em_system.h"
#include "em_timer.h"
#include "em_chip.h"
#include "dmactrl.h"

/* Define PWM frequency value */
#define PWM_FREQ 10000

#define DMA_CHANNEL_TIMER0       0
#define PWM_TABLE_SIZE           8

volatile uint16_t PWMTableA[PWM_TABLE_SIZE];
volatile uint16_t PWMTableB[PWM_TABLE_SIZE];

/* DMA callback structure */
DMA_CB_TypeDef cb;

/**************************************************************************//**
 * @brief  Call-back called when DMA transfer is complete
 *****************************************************************************/
void transferComplete(unsigned int channel, bool primary, void *user)
{
  (void) user;
  
  int i ;
  uint32_t topValue = TIMER_TopGet(TIMER0);
  
  /* Change the compare values in the inactive buffer. As an example the values
  are inverted to generate alternating increasing and decreasing duty cycles. */
  if (primary)
  {
    for (i=0; i<PWM_TABLE_SIZE; i++)
    {
      PWMTableA[i]= topValue-PWMTableA[i];
    }
  }
  else
  {
    for (i=0; i<PWM_TABLE_SIZE; i++)
    {
      PWMTableB[i]= topValue-PWMTableB[i];
    }
  }
  
  /* Re-activate the DMA */
  DMA_RefreshPingPong(channel,
                      primary,
                      false,
                      NULL,
                      NULL,
                      PWM_TABLE_SIZE - 1,
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
  descrCfg.dstInc  = dmaDataIncNone;
  descrCfg.srcInc  = dmaDataInc2;
  descrCfg.size    = dmaDataSize2;
  descrCfg.arbRate = dmaArbitrate1;
  descrCfg.hprot   = 0;
  DMA_CfgDescr(DMA_CHANNEL_TIMER0, true, &descrCfg);
  DMA_CfgDescr(DMA_CHANNEL_TIMER0, false, &descrCfg);

  /* Enabling PingPong Transfer*/  
  DMA_ActivatePingPong(DMA_CHANNEL_TIMER0,
                       false,
                       (void *)&(TIMER0->CC[0].CCVB),
                       (void *)&PWMTableA,
                       PWM_TABLE_SIZE - 1,
                       (void *)&(TIMER0->CC[0].CCVB),
                       (void *)&PWMTableB,
                       PWM_TABLE_SIZE - 1);
}


/**************************************************************************//**
 * @brief  Main function
 * Main is called from __iar_program_start, see assembly startup file
 *****************************************************************************/
int main(void)
{ 
  int i;
  uint32_t topValue;
  
  /* Initialize chip */
  CHIP_Init();
    
  /* Enable clock for GPIO module */
  CMU_ClockEnable(cmuClock_GPIO, true);
  
  /* Enable clock for TIMER0 module */
  CMU_ClockEnable(cmuClock_TIMER0, true);
  
  /* Setup DMA */
  setupDma();
   
  /* Set CC0 location 3 pin (PD1) as output */
  GPIO_PinModeSet(gpioPortD, 1, gpioModePushPull, 0);
  
  /* Select CC channel parameters */
  TIMER_InitCC_TypeDef timerCCInit = 
  {
    .eventCtrl  = timerEventEveryEdge,
    .edge       = timerEdgeBoth,
    .prsSel     = timerPRSSELCh0,
    .cufoa      = timerOutputActionNone,
    .cofoa      = timerOutputActionNone,
    .cmoa       = timerOutputActionToggle,
    .mode       = timerCCModePWM,
    .filter     = false,
    .prsInput   = false,
    .coist      = false,
    .outInvert  = false,
  };
  
  /* Configure CC channel 0 */
  TIMER_InitCC(TIMER0, 0, &timerCCInit);

  /* Route CC0 to location 3 (PD1) and enable pin */  
  TIMER0->ROUTE |= (TIMER_ROUTE_CC0PEN | TIMER_ROUTE_LOCATION_LOC3); 
  
  /* Set Top Value */
  topValue = CMU_ClockFreqGet(cmuClock_HFPER)/PWM_FREQ;
  TIMER_TopSet(TIMER0, topValue);
  
  /* Fill in some initial data in PWM Tables */
  for (i=0; i<PWM_TABLE_SIZE; i++)
  {
    /* Create a list of gradually increasing duty cycles spread across the two
    PWM tables */
    PWMTableA[i] = i*topValue/PWM_TABLE_SIZE/2;
    PWMTableB[i] = (i+PWM_TABLE_SIZE)*topValue/PWM_TABLE_SIZE/2;
  }
  
  /* Set compare value starting at first value in PWMTableA */
  TIMER_CompareSet(TIMER0, 0, PWMTableA[0]);

  /* Select timer parameters */  
  TIMER_Init_TypeDef timerInit =
  {
    .enable     = true,
    .debugRun   = true,
    .prescale   = timerPrescale1,
    .clkSel     = timerClkSelHFPerClk,
    .fallAction = timerInputActionNone,
    .riseAction = timerInputActionNone,
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

