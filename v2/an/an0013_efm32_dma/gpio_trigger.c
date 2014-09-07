/*****************************************************************************
 * @file gpio_trigger.c
 * @brief DMA GPIO Trigger Example
 * @author Silicon Labs
 * @version 2.06
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

#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_dma.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "dmactrl.h"

#define DMA_CHANNEL_TIMER0       0

/* DMA callback structure */
DMA_CB_TypeDef cb;

/* GPIO Transfer Data */
const uint16_t gpioData[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
#define GPIODATA_LENGTH (sizeof(gpioData)/sizeof(uint16_t))

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
                      GPIODATA_LENGTH - 1,
                      false);
}


/**************************************************************************//**
 * @brief  Enabling clocks
 *****************************************************************************/
void setupCmu(void)
{
  /* Enabling clocks */
  CMU_ClockEnable(cmuClock_DMA, true);  
  CMU_ClockEnable(cmuClock_TIMER0, true);  
  CMU_ClockEnable(cmuClock_GPIO, true);    
}


/**************************************************************************//**
 * @brief  Configure GPIO pins
 *****************************************************************************/
void setupGpio(void)
{
  int i;
  /* Set PORT C pins 0-3 to push pull outputs */
  for (i=0; i<4; i++)
  {
    GPIO_PinModeSet(gpioPortC, i, gpioModePushPull, 0);
  }
  
  /* Configure PD1 as an input to receive requests.(TIMER0 CC0 #3) */
  GPIO_PinModeSet(gpioPortD, 1, gpioModeInput, 0);
}


/**************************************************************************//**
 * @brief  Configure TIMER0
 * TIMER is set up to start running on a falling edge of the CC0 input.
 * As the TOP value is set to 0 this will give an immediate overflow that
 * generates a DMA request. As the TIMER is set up in One-Shot Mode, it will
 * stop immediately on the overflow, requiring another falling edge to start
 * again. 
 *****************************************************************************/
void setupTimer(void)
{
  TIMER_Init_TypeDef   init   = TIMER_INIT_DEFAULT;
  
  /* Configure TIMER0 to set a DMA request on falling input on CC0 */
  init.fallAction  = timerInputActionStart ; /* Start TIMER on falling edge on CC0 pin */
  init.oneShot     = true;  /* One shot, stop on overflow */
  init.enable      = false; /* Do not start timer */
  init.dmaClrAct   = true;  /* Clear DMA request when selected channel is active */
  TIMER_Init(TIMER0, &init);
  
    /* Set TOP value to 0 to generate overflow immediately once TIMER starts */
  TIMER_TopSet(TIMER0, 0);
  
  /* Enable routing of TIMER0 CC0 pin to GPIO LOCATION 3 (PD1) */
  TIMER0->ROUTE = TIMER_ROUTE_CC0PEN | TIMER_ROUTE_LOCATION_LOC3;
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
  chnlCfg.select    = DMAREQ_TIMER0_UFOF;
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
                       (void *)&(GPIO->P[2].DOUT),
                       (void *)gpioData,
                       GPIODATA_LENGTH - 1,
                       (void *)&(GPIO->P[2].DOUT),
                       (void *)gpioData,
                       GPIODATA_LENGTH - 1);
}


/**************************************************************************//**
 * @brief  Main function
 * This example uses a TIMER to generate a DMA request on a falling edge of 
 * PD1. The DMA copies data from a RAM buffer to a GPIO port (PC0-PC3) every
 * time the falling edge occurs. When using the Gecko starter kit, connect PD1
 * PB9 with a wire. When push button 0 is pressed, the LEDs should show a
 * counter value increasing. The counter value is stored in RAM and copied
 * with the DMA. 
 *****************************************************************************/
int main(void)
{ 
  /* Initialize chip */
  CHIP_Init();
  
  /* Configuring clocks in the Clock Management Unit (CMU) */
  setupCmu();
  
  /* Configure GPIOs as outputs */
  setupGpio();
  
  /* Configure TIMER to generate DMA request on GPIO falling edge */
  setupTimer();
  
  /* Configure DMA ping-pong transfer to GPIO */      
  setupDma();

  /* Run forever */
  while (1)
  {
   EMU_EnterEM1(); 
  }
}
