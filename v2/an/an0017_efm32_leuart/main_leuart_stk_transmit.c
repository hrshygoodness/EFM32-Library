/**************************************************************************//**
 * @file main_leuart_stk_transmit.c
 * @brief LEUART Demo Application
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

#include "em_chip.h"
#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_leuart.h"
#include "em_dma.h"
#include "em_gpio.h"
#include "em_rtc.h"


/* DEFINES */

#define DMA_CHANNEL           0

#define BUF_MAX               7

#define WAKEUP_INTERVAL_MS    2000

/* GLOBAL VARIABLES */

/* DMA control block, must be aligned to 256. */
#if defined (__ICCARM__)
#pragma data_alignment=256
DMA_DESCRIPTOR_TypeDef dmaControlBlock[DMA_CHAN_COUNT * 2];
#elif defined (__CC_ARM)
DMA_DESCRIPTOR_TypeDef dmaControlBlock[DMA_CHAN_COUNT * 2] __attribute__ ((aligned(256)));
#elif defined (__GNUC__)
DMA_DESCRIPTOR_TypeDef dmaControlBlock[DMA_CHAN_COUNT * 2] __attribute__ ((aligned(256)));
#else
#error Undefined toolkit, need to define alignment
#endif

char hello[] = { 'H', 'E', 'L', 'L', 'O', 0, '\r' };
char there[] = { 'T', 'H', 'E', 'R', 'E', 0, '\r' };

/* Switch to allow alternation between the two strings */
bool     alternateSwitch = false;

uint32_t rtcCountBetweenWakeup;

/* DMA callback structure */
DMA_CB_TypeDef cb[DMA_CHAN_COUNT];

/* Defining the LEUART1 initialization data */
LEUART_Init_TypeDef leuart1Init =
{
  .enable   = leuartEnableTx,       /* Activate data reception on LEUn_TX pin. */
  .refFreq  = 0,                    /* Inherit the clock frequenzy from the LEUART clock source */
  .baudrate = 9600,                 /* Baudrate = 9600 bps */
  .databits = leuartDatabits8,      /* Each LEUART frame containes 8 databits */
  .parity   = leuartNoParity,       /* No parity bits in use */
  .stopbits = leuartStopbits2,      /* Setting the number of stop bits in a frame to 2 bitperiods */
};

/* DMA init structure */
DMA_Init_TypeDef dmaInit = {
  .hprot        = 0,                    /* No descriptor protection */
  .controlBlock = dmaControlBlock,      /* DMA control block alligned to 256 */
};

/* Setting up DMA channel */
DMA_CfgChannel_TypeDef chnlCfg =
{
  .highPri   = false,                   /* Normal priority */
  .enableInt = false,                   /* No interupt for callback function */
  .select    = DMAREQ_LEUART1_TXBL,     /* Set LEUART1 TX buffer empty, as source of DMA signals */
  .cb        = &(cb[DMA_CHANNEL]),      /* Callback */
};

/* Setting up channel descriptor */
DMA_CfgDescr_TypeDef descrCfg =
{
  .dstInc  = dmaDataIncNone,    /* Do not increment destination address */
  .srcInc  = dmaDataInc1,       /* Increment source address by one byte */
  .size    = dmaDataSize1,      /* Data size is one byte */
  .arbRate = dmaArbitrate1,     /* Rearbitrate for each byte recieved */
  .hprot   = 0,                 /* No read/write source protection */
};

/* Set up RTC init struct*/
RTC_Init_TypeDef rtcInit =
{
  .debugRun = false,
  .comp0Top = true,
  .enable   = true,
};


/**************************************************************************//**
 * @brief RTC Interrupt Handler.
 *
 * This routine will run every other second, when the RTC times out, and generate
 * an interrupt. After clearing the interrupt source, the DMA source address is
 * set, and a new DMA transfer is initiated. When the routine is finished, the
 * system will again enter EM2 while the DMA continues to transfer data from the
 * memory to the LEUART. The LEUART DMA wake-up on TX is enabled to allow a 
 * LEUART DMA request to wake up the DMA. 
 *
 *****************************************************************************/
void RTC_IRQHandler(void)
{
  /* Clear interrupt source */
  RTC_IntClear(RTC_IFC_COMP0);

  switch (alternateSwitch)
  {
  case true:
    /* Set new DMA source address directly in the DMA descriptor */
    dmaControlBlock->SRCEND = there + BUF_MAX - 1;
    alternateSwitch         = false;
    break;

  case false:
    /* Set new DMA source address directly in the DMA descriptor */
    dmaControlBlock->SRCEND = hello + BUF_MAX - 1;
    alternateSwitch         = true;
    break;
  }
  
  /* Enable DMA wake-up from LEUART1 TX */
  LEUART1->CTRL = LEUART_CTRL_TXDMAWU;

  /* (Re)starting the transfer. Using Basic Mode */
  DMA_ActivateBasic(DMA_CHANNEL,                  /* Activate channel selected */
                    true,                         /* Use primary descriptor */
                    false,                        /* No DMA burst */
                    NULL,                         /* Keep destination address */
                    NULL,                         /* Keep source address*/
                    BUF_MAX - 1);                 /* Size of buffer minus1 */
}

  
/**************************************************************************//**
 * @brief  DMA Callback function
 *
 * When the DMA transfer is completed, disables the DMA wake-up on TX in the 
 * LEUART to enable the DMA to sleep even when the LEUART buffer is empty.
 *
 ******************************************************************************/
void dmaTransferDone(unsigned int channel, bool primary, void *user)
{
  (void) channel;
  (void) primary;
  (void) user;
  
  /* Disable DMA wake-up from LEUART1 TX */
  LEUART1->CTRL &= ~LEUART_CTRL_TXDMAWU;
}


/**************************************************************************//**
 * @brief  Initialize Low Energy UART 1
 *
 * Here the LEUART is initialized with the chosen settings. It is then routed
 * to location 0 to avoid conflict with the LCD pinout. Finally the GPIO mode
 * is set to push pull.
 *
 *****************************************************************************/
void initLeuart(void)
{
  /* Reseting and initializing LEUART1 */
  LEUART_Reset(LEUART1);
  LEUART_Init(LEUART1, &leuart1Init);

  /* Route LEUART1 TX pin to DMA location 0 */
  LEUART1->ROUTE = LEUART_ROUTE_TXPEN |
                   LEUART_ROUTE_LOCATION_LOC0;

  /* Enable GPIO for LEUART1. TX is on C6 */
  GPIO_PinModeSet(gpioPortC,                /* GPIO port */
                  6,                        /* GPIO port number */
                  gpioModePushPull,         /* Pin mode is set to push pull */
                  1);                       /* High idle state */
}


/**************************************************************************//**
 * @brief  Setup Low Energy UART with DMA operation
 *
 * The LEUART/DMA interaction is defined, and the DMA, channel and descriptor
 * is initialized. The destination for all the DMA transfers through this
 * channel is set to be the LEUART1 TXDATA register, and transfer complete
 * interrupt is enabled.
 *
 *****************************************************************************/
void setupLeuartDma(void)
{
  /* Setting call-back function */  
  cb[DMA_CHANNEL].cbFunc  = dmaTransferDone;
  cb[DMA_CHANNEL].userPtr = NULL;
  
  /* Initializing DMA, channel and desriptor */
  DMA_Init(&dmaInit);  
  DMA_CfgChannel(DMA_CHANNEL, &chnlCfg);
  DMA_CfgDescr(DMA_CHANNEL, true, &descrCfg);

  /* Set new DMA destination address directly in the DMA descriptor */
  dmaControlBlock->DSTEND = &LEUART1->TXDATA;

  /* Enable DMA Transfer Complete Interrupt */
  DMA->IEN = DMA_IEN_CH0DONE;

  /* Enable DMA interrupt vector */
  NVIC_EnableIRQ(DMA_IRQn);
}


/**************************************************************************//**
 * @brief  Setup Real Time Clock (RTC)
 *
 * The RTC is initialized, and is set up to generate an interrupt every other
 * second.
 *
 *****************************************************************************/
void setupRtc(void)
{
  /* Input RTC init struct in initialize funciton */
  RTC_Init(&rtcInit);

  /* Set RTC compare value */
  rtcCountBetweenWakeup = ((SystemLFXOClockGet() * WAKEUP_INTERVAL_MS) / 1000);
  RTC_CompareSet(0, rtcCountBetweenWakeup);

  /* Enable RTC interrupt from COMP0 */
  RTC_IntEnable(RTC_IF_COMP0);

  /* Enable RTC interrupt vector in NVIC */
  NVIC_EnableIRQ(RTC_IRQn);

  /* Enable RTC */
  RTC_Enable(true);
}


/**************************************************************************//**
 * @brief  Main function
 *
 * This example demonstrates a way to use the Low Energy UART to maintain full
 * UART communication capabilities, while spending a great majority of the time
 * in deep sleep mode EM2. The LEUART is in this example driven by the LFXO,
 * which provide good accuracy while consuming only small amounts of energy. In
 * addition the DMA is set up to read the data to be transmitted by the LEUART
 * directly from the system memory. This relieves the CPU from doing anything
 * other than initializing the transfer, and handle possible interrupts triggered
 * when the transfer is finished. In this case the strings "HELLO" and "THERE"
 * are alternatingly transmitted every other second through the LEUART.
 *
 *****************************************************************************/
int main(void)
{
  /* Initialize chip */
  CHIP_Init();

  /* Start LFXO, and use LFXO for low-energy modules */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
  CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);

  /* Enabling clocks, all other remain disabled */
  CMU_ClockEnable(cmuClock_CORELE, true);     /* Enable CORELE clock */
  CMU_ClockEnable(cmuClock_DMA, true);        /* Enable DMA clock */
  CMU_ClockEnable(cmuClock_GPIO, true);       /* Enable GPIO clock */
  CMU_ClockEnable(cmuClock_LEUART1, true);    /* Enable LEUART1 clock */
  CMU_ClockEnable(cmuClock_RTC, true);        /* Enable RTC clock */

  /* Initialize LEUART */
  initLeuart();

  /* Setup LEUART with DMA */
  setupLeuartDma();

  /* Setup RTC as interrupt source */
  setupRtc();

  while (1)
  {
    /* Enable deep sleep */
    EMU_EnterEM2(false);
  }
}
