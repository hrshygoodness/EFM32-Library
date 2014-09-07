/**************************************************************************//**
 * @file main_leuart_dvk_receive_lcd.c
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
#include "bsp.h"
#include "segmentlcd.h"
#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_leuart.h"
#include "em_dma.h"
#include "em_lcd.h"
#include "em_gpio.h"


/* DEFINES */

#define DMA_CHANNEL    0
#define BUF_MAX        1023


/* GLOBAL VARIABLES */

char rxbuf[BUF_MAX];

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

uint32_t leuartif;
uint32_t len;

/* Defining the LEUART1 initialization data */
LEUART_Init_TypeDef leuart1Init =
{
  .enable   = leuartEnableRx,     /* Activate data reception on LEUn_RX pin. */
  .refFreq  = 0,                  /* Inherit the clock frequenzy from the LEUART clock source */
  .baudrate = 9600,               /* Baudrate = 9600 bps */
  .databits = leuartDatabits8,    /* Each LEUART frame containes 8 databits */
  .parity   = leuartNoParity,     /* No parity bits in use */
  .stopbits = leuartStopbits2,    /* Setting the number of stop bits in a frame to 2 bitperiods */
};

/* DMA init structure */
DMA_Init_TypeDef dmaInit =
{
  .hprot        = 0,                  /* No descriptor protection */
  .controlBlock = dmaControlBlock,    /* DMA control block alligned to 256 */
};

/* Setting up channel */
DMA_CfgChannel_TypeDef chnlCfg =
{
  .highPri   = false,                     /* Normal priority */
  .enableInt = false,                     /* No interupt enabled for callback functions */
  .select    = DMAREQ_LEUART1_RXDATAV,    /* Set LEUART1 RX data avalible as source of DMA signals */
  .cb        = NULL,                      /* No callback funtion */
};

/* Setting up channel descriptor */
DMA_CfgDescr_TypeDef descrCfg =
{
  .dstInc  = dmaDataInc1,       /* Increment destination address by one byte */
  .srcInc  = dmaDataIncNone,    /* Do not increment source address  */
  .size    = dmaDataSize1,      /* Data size is one byte */
  .arbRate = dmaArbitrate1,     /* Rearbitrate for each byte recieved*/
  .hprot   = 0,                 /* No read/write source protection */
};

/* LCD Animation Configuration */
LCD_AnimInit_TypeDef animInit =
{
  .enable    = true,                /* Enable Animation at end of initialization */
  .AReg      = 0x01,                /* Initial Animation Register A Value */
  .AShift    = lcdAnimShiftLeft,    /* Shift operation of Animation Register A */
  .BReg      = 0x00,                /* Initial Animation Register B Value */
  .BShift    = lcdAnimShiftNone,    /* Shift operation of Animation Register B */
  .animLogic = lcdAnimLogicOr,      /* A and B Logical Operation to use for mixing and outputting resulting segments */
};

/* LCD Frame Control Initialization */
LCD_FrameCountInit_TypeDef fcInit =
{
  .enable   = true,              /* Enable at end */
  .top      = 8,                 /* Frame Counter top value */
  .prescale = lcdFCPrescDiv1,    /* Frame Counter clock prescaler */
};


/**************************************************************************//**
 * @brief LEUART IRQ handler
 *
 * When the signal frame is detected by the LEUART, this interrupt routine will
 * zero-terminate the char array, write the received string the to the LCD, and
 * reset the DMA for new data.
 *
 *****************************************************************************/
void LEUART1_IRQHandler(void)
{
  /* Store and reset pending interupts */
  leuartif = LEUART_IntGet(LEUART1);
  LEUART_IntClear(LEUART1, leuartif);

  /* Signal frame found */
  if (leuartif & LEUART_IF_SIGF)
  {
    /* Zero-terminate rx buffer */
    len            = BUF_MAX - 1 - (( dmaControlBlock->CTRL >> 4 ) & 0x3FF);
    rxbuf[len - 1] = 0;

    /* Write recieved string to LCD */
    SegmentLCD_Write(rxbuf);

    /* Reactivate DMA */
    DMA_ActivateBasic(DMA_CHANNEL,     /* Activate DMA channel 0 */
                      true,            /* Activate using primary descriptor */
                      false,           /* No DMA burst */
                      NULL,            /* Keep source */
                      NULL,            /* Keep destination */
                      BUF_MAX - 1);    /* Number of DMA transfer elements (minus 1) */
  }
}


/**************************************************************************//**
 * @brief  Initialize Low Energy UART 1
 *
 * Here the LEUART is initialized with the chosen settings. It is then routed
 * to location 0 to avoid conflict with the LCD pinout. Finally the GPIO mode
 * is set to input only with pull-up. This keeps the output high when idle,
 * just like the LEUART.
 *
 *****************************************************************************/
void initLeuart(void)
{
  /* Reseting and initializing LEUART1 */
  LEUART_Reset(LEUART1);
  LEUART_Init(LEUART1, &leuart1Init);

  /* Route LEUART1 RX pin to DMA location 0 */
  LEUART1->ROUTE = LEUART_ROUTE_RXPEN |
                   LEUART_ROUTE_LOCATION_LOC0;

  /* Enable GPIO for LEUART1. RX is on C7 */
  GPIO_PinModeSet(gpioPortC,            /* Port */
                  7,                    /* Port number */
                  gpioModeInputPull,    /* Pin mode is set to input only, with pull direction given bellow */
                  1);                   /* Pull direction is set to pull-up */
}


/**************************************************************************//**
 * @brief  Setup Low Energy UART with DMA operation
 *
 * The LEUART/DMA interaction is defined, and the DMA, channel and descriptor
 * is initialized. Then DMA starts to wait for and receive data, and the LEUART1
 * is set up to generate an interrupt when it receives the defined signal frame.
 * The signal frame is set to '\r', which is the "carriage return" symbol.
 *
 *****************************************************************************/
void setupLeuartDma(void)
{
  /* Initializing DMA, channel and desriptor */
  DMA_Init(&dmaInit);
  DMA_CfgChannel(DMA_CHANNEL, &chnlCfg);
  DMA_CfgDescr(DMA_CHANNEL, true, &descrCfg);

  /* Starting the transfer. Using Basic Mode */
  DMA_ActivateBasic(DMA_CHANNEL,                /* Activate channel selected */
                    true,                       /* Use primary descriptor */
                    false,                      /* No DMA burst */
                    (void *) &rxbuf,            /* Destination address */
                    (void *) &LEUART1->RXDATA,  /* Source address*/
                    BUF_MAX - 1);               /* Size of buffer minus1 */

  /* Set LEUART signal frame */
  LEUART1->SIGFRAME = '\r';

  /* Enable LEUART Signal Frame Interrupt */
  LEUART_IntEnable(LEUART1, LEUART_IEN_SIGF);

  /* Enable LEUART1 interrupt vector */
  NVIC_EnableIRQ(LEUART1_IRQn);

  /* Make sure the LEUART wakes up the DMA on RX data */
  LEUART1->CTRL = LEUART_CTRL_RXDMAWU;
}


/**************************************************************************//**
 * @brief  Setup and activate LCD ring animation
 *
 * The LCD animation registers are set, and the animation is setup with speed
 * given by the rate of frame counter interrupts.
 *
 *****************************************************************************/
void startLCDAnimation(void)
{
  /* Show animation */
  LCD_AnimInit(&animInit);

  /* Configure LCD to give a frame counter interrupt every 8th frame. */
  LCD_FrameCountInit(&fcInit);
  LCD_SyncBusyDelay(LCD_SYNCBUSY_BACTRL);
  LCD_IntClear(LCD_IFC_FC);
  LCD_IntEnable(LCD_IEN_FC);
}


/**************************************************************************//**
 * @brief  Main function
 *
 * This example demonstrates a way to use the Low Energy UART to maintain full
 * UART communication capabilities, while spending a great majority of the time
 * in deep sleep mode EM2. The LEUART is in this example driven by the LFXO,
 * which provide good accuracy while consuming only small amounts of energy. In
 * addition the DMA is set up to write the data received by the LEUART
 * directly to the system memory. This relieves the CPU from doing anything
 * before all the data is received, and an interrupt is triggered. The received
 * data is the written to the LCD.
 *
 *****************************************************************************/
int main(void)
{
  /* Initialize chip */
  CHIP_Init();

  /* Enable RS232 port B on the DVK */
  BSP_Init(BSP_INIT_DEFAULT);
  BSP_PeripheralAccess(BSP_RS232B, true);
  BSP_Disable();

  /* Start LFXO, and use LFXO for low-energy modules */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
  CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);

  /* Enabling clocks, all other remain disabled */
  CMU_ClockEnable(cmuClock_CORELE, true);     /* Enable CORELE clock */
  CMU_ClockEnable(cmuClock_DMA, true);        /* Enable DMA clock */
  CMU_ClockEnable(cmuClock_GPIO, true);       /* Enable GPIO clock */
  CMU_ClockEnable(cmuClock_LCD, true);        /* Enable LCD clock */
  CMU_ClockEnable(cmuClock_LEUART1, true);    /* Enable LEUART1 clock */

  /* Initialize LCD */
  SegmentLCD_Init(false);

  /* Initialize LEUART */
  initLeuart();

  /* Setup LEUART with DMA */
  setupLeuartDma();

  /* Show antenna symbol */
  SegmentLCD_Symbol(LCD_SYMBOL_ANT, 1);

  /* Activate the LCD animation */
  startLCDAnimation();

  /* Write LEUART on LCD */
  SegmentLCD_Write("LEUART");

  while (1)
  {
    /* Enable deep sleep */
    EMU_EnterEM2(false);
  }
}
