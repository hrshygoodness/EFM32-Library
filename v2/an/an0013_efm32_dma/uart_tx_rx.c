/*****************************************************************************
 * @file uart_tx_rx.c
 * @brief DMA UART transmit/receive example
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
#include "bsp.h"
#include "em_chip.h"
#include "em_usart.h"
#include "em_gpio.h"
#include "em_dma.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_timer.h"
#include "em_int.h"
#include "em_prs.h"
#include "dmactrl.h"

#define DMA_CHANNEL_RX       0
#define DMA_CHANNEL_TX       1
#define DMA_CHANNELS         2
#define RX_BUFFER_SIZE       32 /* Even numbers only */
#define TX_BUFFER_SIZE       RX_BUFFER_SIZE

#define RX_TIMEOUT_MS     2000

/* DMA callback structure */
DMA_CB_TypeDef cb[DMA_CHANNELS];

/* RX Ring Buffer */
struct 
{
  volatile char buffer[RX_BUFFER_SIZE];
  volatile int startIndex;  /* Position of first unread data */
  volatile int stopIndex;   /* Position after last unread data */
} rxBuffer = {.startIndex = 0, .stopIndex = 0};


/* Flag indicating that new data is available in RX ring buffer */
volatile bool dataAvailable = false;



/**************************************************************************//**
 * @brief  TIMER0 Interrupt handler
 * The TIMER OF interrupt is triggered when the RX line has been stationary for 
 * longer than the timout period. This ISR then checks how many bytes have been
 * transferred and updates the rxStopIndex accordingly. 
 *****************************************************************************/
void TIMER0_IRQHandler(void)
{
  DMA_DESCRIPTOR_TypeDef *primDescr;
  DMA_DESCRIPTOR_TypeDef *altDescr;
  DMA_CB_TypeDef         *dmaCallback;
  
  /* Clear interrupt flag */
  TIMER_IntClear(TIMER0, TIMER_IFC_OF);
  
  /* Get primary descriptor */
  primDescr = ((DMA_DESCRIPTOR_TypeDef *)(DMA->CTRLBASE)) + DMA_CHANNEL_RX;
  
  /* Get alternate descriptor */
  altDescr = ((DMA_DESCRIPTOR_TypeDef *)(DMA->ALTCTRLBASE)) + DMA_CHANNEL_RX;
  
  /* Get callback to check if primary or alternate structure is used */
  dmaCallback = (DMA_CB_TypeDef *)(primDescr->USER);
  
  /* Check in alternate or primary descriptors how many bytes were transferred */
  /* and move rxStopIndex accordingly */
  if (dmaCallback->primary)
  {
    rxBuffer.stopIndex = RX_BUFFER_SIZE/2 - ((primDescr->CTRL & _DMA_CTRL_N_MINUS_1_MASK) >>_DMA_CTRL_N_MINUS_1_SHIFT)-1;
  }
  else
  {
    rxBuffer.stopIndex = RX_BUFFER_SIZE - ((altDescr->CTRL & _DMA_CTRL_N_MINUS_1_MASK) >>_DMA_CTRL_N_MINUS_1_SHIFT)-1; 
  }
  
  /* Indicate new data available */
  dataAvailable = true;
}



/**************************************************************************//**
 * @brief  Call-back called when RX is complete
 *****************************************************************************/
void rxDmaComplete(unsigned int channel, bool primary, void *user)
{ 
  (void) user;
  
  /* Re-arm DMA channel for RX */
  DMA_RefreshPingPong(channel,
                      primary,
                      false,
                      NULL,
                      NULL,
                      RX_BUFFER_SIZE/2 - 1,
                      false);
  
  /* Move stop index when a descriptor has finished */
  if  (primary)
  {
    rxBuffer.stopIndex = RX_BUFFER_SIZE/2-1;
  }
  else
  {
    rxBuffer.stopIndex = RX_BUFFER_SIZE-1;
  }
  
}



/**************************************************************************//**
 * @brief  Enable clocks
 *****************************************************************************/
void setupCmu(void)
{
  /* Set HFRCO frequency to 7 MHz */
  CMU_HFRCOBandSet(cmuHFRCOBand_7MHz);
  
  /* Enable clocks */
  CMU_ClockEnable(cmuClock_DMA,    true); 
  CMU_ClockEnable(cmuClock_PRS,    true);  
  CMU_ClockEnable(cmuClock_GPIO,   true);  
  CMU_ClockEnable(cmuClock_UART0,  true);  
  CMU_ClockEnable(cmuClock_TIMER0, true);
}



/**************************************************************************//**
 * @brief Configure DMA for UART RX and TX
 * RX uses ping pong, where the primary and alternate descriptors operate on
 * half of the RX buffer each. 
 * TX uses basic transfer as timing is controlled by sender and ping-pong is
 * therefore not needed. 
 *****************************************************************************/
void setupDma(void)
{
  DMA_Init_TypeDef         dmaInit;
  DMA_CfgChannel_TypeDef   rxChnlCfg;
  DMA_CfgChannel_TypeDef   txChnlCfg;
  DMA_CfgDescr_TypeDef     rxDescrCfg;
  DMA_CfgDescr_TypeDef     txDescrCfg;
  
  /* Initializing the DMA */
  dmaInit.hprot        = 0;
  dmaInit.controlBlock = dmaControlBlock;
  DMA_Init(&dmaInit);
    
  /*** Setting up RX DMA ***/
   
  /* Setting up call-back function */  
  cb[DMA_CHANNEL_RX].cbFunc  = rxDmaComplete;
  cb[DMA_CHANNEL_RX].userPtr = NULL;

  /* Setting up channel */
  rxChnlCfg.highPri   = false;
  rxChnlCfg.enableInt = true;
  rxChnlCfg.select    = DMAREQ_UART0_RXDATAV;
  rxChnlCfg.cb        = &(cb[DMA_CHANNEL_RX]);
  DMA_CfgChannel(DMA_CHANNEL_RX, &rxChnlCfg);

  /* Setting up channel descriptor */
  rxDescrCfg.dstInc  = dmaDataInc1;
  rxDescrCfg.srcInc  = dmaDataIncNone;
  rxDescrCfg.size    = dmaDataSize1;
  rxDescrCfg.arbRate = dmaArbitrate1;
  rxDescrCfg.hprot   = 0;
  DMA_CfgDescr(DMA_CHANNEL_RX, true, &rxDescrCfg);
  DMA_CfgDescr(DMA_CHANNEL_RX, false, &rxDescrCfg);
  
  /* Activate Alternate and Primary channels */
  DMA_ActivatePingPong(DMA_CHANNEL_RX,
                       false,
                       (void *)&rxBuffer.buffer,
                       (void *)&(UART0->RXDATA),
                       RX_BUFFER_SIZE/2 - 1,
                       (void *)((&rxBuffer.buffer[RX_BUFFER_SIZE/2])),
                       (void *)&(UART0->RXDATA),
                       RX_BUFFER_SIZE/2 - 1);
  
  /*** Setting up TX DMA ***/
  
   /* No callback needed for TX */
  cb[DMA_CHANNEL_TX].cbFunc  = NULL;
  cb[DMA_CHANNEL_TX].userPtr = NULL;
  
   /* Setting up channel */
  txChnlCfg.highPri   = false;
  txChnlCfg.enableInt = false;
  txChnlCfg.select    = DMAREQ_UART0_TXBL;
  txChnlCfg.cb        = &(cb[DMA_CHANNEL_TX]);
  DMA_CfgChannel(DMA_CHANNEL_TX, &txChnlCfg);

  /* Setting up channel descriptor */
  txDescrCfg.dstInc  = dmaDataIncNone;
  txDescrCfg.srcInc  = dmaDataInc1;
  txDescrCfg.size    = dmaDataSize1;
  txDescrCfg.arbRate = dmaArbitrate1;
  txDescrCfg.hprot   = 0;
  DMA_CfgDescr(DMA_CHANNEL_TX, true, &txDescrCfg);
}



/**************************************************************************//**
 * @brief  Start DMA Transmit of UART Data
 *****************************************************************************/
void sendUartData(void *buffer, int bytes)
{
  /* Wait until channel becomes available */
  while(DMA_ChannelEnabled(DMA_CHANNEL_TX));
        
  /* Activate DMA channel for TX */      
  DMA_ActivateBasic(DMA_CHANNEL_TX,
                    true,
                    false,
                    (void *)&(UART0->TXDATA),
                    buffer,
                    bytes - 1);
}



/**************************************************************************//**
 * @brief  Setup UART
 *****************************************************************************/
void setupUart(void)
{
  USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;  
  
  /* Configure GPIO pins for UART */
  GPIO_PinModeSet(gpioPortE, 0,  gpioModePushPullDrive, 0); /* TX */
  GPIO_PinModeSet(gpioPortE, 1,  gpioModeInput,         0); /* RX */
  
  /* Initialize SPI */
  uartInit.baudrate = 115200;
  USART_InitAsync(UART0, &uartInit);
  
  /* Enable UART transmit and receive */
  USART_Enable(UART0, usartEnable);  
 
  /* Enable routing for UART pins from UART0 to location 1 */
  UART0->ROUTE = UART_ROUTE_TXPEN | 
                 UART_ROUTE_RXPEN | 
                 UART_ROUTE_LOCATION_LOC1;
}



/**************************************************************************//**
 * @brief  Setup Timer
 * Configure TIMER to reset and start counter every time the RX pin on 
 * the UART has a falling edge. If the RX line is idle for longer than the
 * timeout period the overflow interrupt is called indicating that a full
 * message has been received.
 *****************************************************************************/
void setupTimer(void)
{
  TIMER_Init_TypeDef   init   = TIMER_INIT_DEFAULT;
  TIMER_InitCC_TypeDef initCc = TIMER_INITCC_DEFAULT;
  
  /* Configure TIMER0 to set a DMA request on falling input on CC0 */
  init.fallAction  = timerInputActionReloadStart ; /* Reload and Start TIMER on falling edge on CC0 input */
  init.oneShot     = true;                         /* One shot, stop on overflow */
  init.enable      = false;                        /* Do not start timer */
  init.prescale    = timerPrescale1024;            /* Prescale by 1024 */
  TIMER_Init(TIMER0, &init);
  
  /* Configure CC0 to listen to PRS CH0 */
  initCc.prsInput = true;
  initCc.prsSel   = timerPRSSELCh0;
  TIMER_InitCC(TIMER0, 0, &initCc);
  
  /* Set TOP value according to timout value*/
  TIMER_TopSet(TIMER0, (CMU_ClockFreqGet(cmuClock_TIMER0)/1000)*RX_TIMEOUT_MS/1024);
  
  /* Enable input sensing for PRS */
  GPIO_InputSenseSet(GPIO_INSENSE_PRS, GPIO_INSENSE_PRS);
  
  /* Configure pin 1 interrupt/PRS to port E. Setting is shared by interrupts and PRS */
  GPIO_IntConfig(gpioPortE, 1, false, false, false);
  
  /* Configure PRS channel 0 to listen to pin 1 PRS signals */
  PRS_SourceSignalSet(0, PRS_CH_CTRL_SOURCESEL_GPIOL, PRS_CH_CTRL_SIGSEL_GPIOPIN1, prsEdgeOff);
  
  /* Generate interrupt on overflow (timeout) */
  TIMER_IntEnable(TIMER0, TIMER_IEN_OF);
  NVIC_EnableIRQ(TIMER0_IRQn);
}



/**************************************************************************//**
 * @brief  Pull data from DMA Receive Buffer
 * This function checks if there is any new data available in the DMA RX ring
 * buffer. If so, the data is returned and the rxStartIndex is moved accordingly
 *****************************************************************************/
int getUartData(char *buffer, int maxLength)
{
  int i, readLength, stopIndexLocal, dmaInt, timerInt;
  int readCount=0;
  bool dataAvailableLocal;
  
  /*** Halt TIMER0 and DMA interrupts while copying to local variables ***/
  dmaInt = DMA->IEN;
  timerInt = TIMER0->IEN;
  
  DMA->IEN    &= ~(1<<DMA_CHANNEL_RX);
  TIMER0->IEN &= ~TIMER_IEN_OF;
    
  /* Copy to local variables */
  stopIndexLocal     = rxBuffer.stopIndex;
  dataAvailableLocal = dataAvailable;
  
  /* Set flag low to be ready to receive new data */
  dataAvailable      = false; 
    
  /* Re-enable interrupts */  
  DMA->IEN    = dmaInt;
  TIMER0->IEN = timerInt;
  
  /* Check if new data is available */
  if(dataAvailableLocal)
  {
    if (rxBuffer.startIndex>stopIndexLocal)
    {
      readCount = stopIndexLocal+RX_BUFFER_SIZE-rxBuffer.startIndex;
    }
    else
    {
      readCount = stopIndexLocal-rxBuffer.startIndex;
    }
  
    /* Only read maxLength number of bytes */
    if (readCount > maxLength)
    {
      readLength = maxLength;
    }
    else
    {
      readLength = readCount;
    }
  
    /* Copy data to returned buffer */
    for (i=0; i<readLength; i++)
    {
      buffer[i] = rxBuffer.buffer[(rxBuffer.startIndex+i)%RX_BUFFER_SIZE]; 
    }
  
    /* Move start pointer */
    rxBuffer.startIndex = (rxBuffer.startIndex+readLength) % RX_BUFFER_SIZE;
  }
  
  return readLength;
}



/**************************************************************************//**
 * @brief  Main function
 * This example sets up a UART connection with both RX and TX enabled. Incoming
 * data is copied by the DMA to a ring buffer. A full message must be followed
 * by an idle period on the RX lines which is longer than the timeout set. 
 * When this timeout is reached, the main loop reads the characters in the
 * buffer and inverts the case of all letters before starting a DMA transfer
 * to the UART TX with the manipulated string. 
*****************************************************************************/
int main(void)
{ 
  int bytesReceived, i;
  char txBuffer[RX_BUFFER_SIZE];
  
  /* Initialize chip */
  CHIP_Init();
  
  /* Initialize DVK board register access through SPI */
  BSP_Init(BSP_INIT_DEFAULT);

  /* Enable RS232 port on DK */
  BSP_PeripheralAccess(BSP_RS232_UART, true);
  
  /* Disable DVK as we only needed this connection to enable RS232 port */
  BSP_Disable();
  
  /* Configuring clocks in the Clock Management Unit (CMU) */
  setupCmu();
  
  /* Configure UART operation */
  setupUart();
    
  /* Configure DMA transfer to/from UART */      
  setupDma();
  
  /* Configure TIMER0 to give timout interrupt when RX line is idle */
  setupTimer();

  /* This loop checks if there is new data available every time an IRQ wakes 
  * the device up. Interrupts are disabled before the check is made in case
  * the TIMER ISR increases the available data after the check, but before going
  * to EM1. If this were to happen the device would not wake-up again before the 
  * next message was received. With the interrupts disabled any IRQ will wake
  * the device up. Any pending interrupts will be handled after the interrupts
  * have been enabled again. */
  while (1)
  {
    /* Disable interrupts */
    INT_Disable();
    
    /* Get new data from RX buffer if available */
    bytesReceived = getUartData(txBuffer, RX_BUFFER_SIZE);
    
    if (bytesReceived > 0)
    {
      
      /* Enable interrupts */
      INT_Enable();
      
      /* Manipulate received data before sending */
      for (i=0; i<bytesReceived; i++)
      {
        /* Turn lowercase letters into uppercase */
        if (96<txBuffer[i] && txBuffer[i]<123)
        {
          txBuffer[i] -= 32;
        }
        
        /* Turn uppercase letters into lowercase */
        else if (64<txBuffer[i] && txBuffer[i]<91)
        {
          txBuffer[i] += 32;
        }
      }
    
      /* Send received bytes back */
      sendUartData(&txBuffer, bytesReceived);
      
    }
    else
    {
      /* Go to sleep */
      EMU_EnterEM1();
      
      /* Enable interrupts again */
      INT_Enable();
    }
    
  }
}
