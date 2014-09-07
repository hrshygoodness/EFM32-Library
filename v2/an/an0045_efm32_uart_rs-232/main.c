/******************************************************************************
 * @file main.c
 * @brief USART/UART Asynchronous mode Application Note software example
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

#include <stdint.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "bsp.h"


/* Function prototypes */
void uartSetup(void);
void cmuSetup(void);
void uartPutData(uint8_t * dataPtr, uint32_t dataLen);
uint32_t uartGetData(uint8_t * dataPtr, uint32_t dataLen);
void    uartPutChar(uint8_t charPtr);
uint8_t uartGetChar(void);


/* Declare some strings */
const char     welcomeString[]  = "EFM32 RS-232 - Please press a key\n";
const char     overflowString[] = "\n---RX OVERFLOW---\n";
const uint32_t welLen           = sizeof(welcomeString) - 1;
const uint32_t ofsLen           = sizeof(overflowString) - 1;

/* Define termination character */
#define TERMINATION_CHAR    '.'

/* Declare a circular buffer structure to use for Rx and Tx queues */
#define BUFFERSIZE          256

volatile struct circularBuffer
{
  uint8_t  data[BUFFERSIZE];  /* data buffer */
  uint32_t rdI;               /* read index */
  uint32_t wrI;               /* write index */
  uint32_t pendingBytes;      /* count of how many bytes are not yet handled */
  bool     overflow;          /* buffer overflow indicator */
} rxBuf, txBuf = { {0}, 0, 0, 0, false };


/* Setup UART1 in async mode for RS232*/
static USART_TypeDef           * uart   = UART1;
static USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;




/******************************************************************************
 * @brief  Main function
 *
 *****************************************************************************/
int main(void)
{
  /* Initialize chip - handle erratas */
  CHIP_Init( );

  /* Initialize clocks and oscillators */
  cmuSetup( );

  /* Initialize UART peripheral */
  uartSetup( );

  /* Initialize Development Kit in EBI mode */
  BSP_Init(BSP_INIT_DEFAULT);

  /* Enable RS-232 transceiver on Development Kit */
  BSP_PeripheralAccess(BSP_RS232_UART, true);

  /* When DVK is configured, and no more DVK access is needed, the interface can safely be disabled to save current */
  BSP_Disable();


  /* Write welcome message to UART */
  uartPutData((uint8_t*) welcomeString, welLen);

  /*  Eternal while loop
   *  CPU will sleep during Rx and Tx. When a byte is transmitted, an interrupt
   *  wakes the CPU which copies the next byte in the txBuf queue to the
   *  UART TXDATA register.
   *
   *  When the predefined termiation character is received, the all pending
   *  data in rxBuf is copied to txBuf and echoed back on the UART */
  while (1)
  {
    /* Wait in EM1 while UART transmits */
    EMU_EnterEM1();

    /* Check if RX buffer has overflowed */
    if (rxBuf.overflow)
    {
      rxBuf.overflow = false;
      uartPutData((uint8_t*) overflowString, ofsLen);
    }

    /* Check if termination character is received */
    if (rxBuf.data[(rxBuf.wrI - 1) % BUFFERSIZE] == TERMINATION_CHAR)
    {
      /* Copy received data to UART transmit queue */
      uint8_t tmpBuf[BUFFERSIZE];
      int     len = uartGetData(tmpBuf, 0);
      uartPutData(tmpBuf, len);
    }
  }
}




/******************************************************************************
* @brief  uartSetup function
*
******************************************************************************/
void uartSetup(void)
{
  /* Enable clock for GPIO module (required for pin configuration) */
  CMU_ClockEnable(cmuClock_GPIO, true);
  /* Configure GPIO pins */
  GPIO_PinModeSet(gpioPortB, 9, gpioModePushPull, 1);
  GPIO_PinModeSet(gpioPortB, 10, gpioModeInput, 0);


  /* Prepare struct for initializing UART in asynchronous mode*/
  uartInit.enable       = usartDisable;   /* Don't enable UART upon intialization */
  uartInit.refFreq      = 0;              /* Provide information on reference frequency. When set to 0, the reference frequency is */
  uartInit.baudrate     = 115200;         /* Baud rate */
  uartInit.oversampling = usartOVS16;     /* Oversampling. Range is 4x, 6x, 8x or 16x */
  uartInit.databits     = usartDatabits8; /* Number of data bits. Range is 4 to 10 */
  uartInit.parity       = usartNoParity;  /* Parity mode */
  uartInit.stopbits     = usartStopbits1; /* Number of stop bits. Range is 0 to 2 */
  uartInit.mvdis        = false;          /* Disable majority voting */
  uartInit.prsRxEnable  = false;          /* Enable USART Rx via Peripheral Reflex System */
  uartInit.prsRxCh      = usartPrsRxCh0;  /* Select PRS channel if enabled */

  /* Initialize USART with uartInit struct */
  USART_InitAsync(uart, &uartInit);

  /* Prepare UART Rx and Tx interrupts */
  USART_IntClear(uart, _UART_IFC_MASK);
  USART_IntEnable(uart, UART_IEN_RXDATAV);
  NVIC_ClearPendingIRQ(UART1_RX_IRQn);
  NVIC_ClearPendingIRQ(UART1_TX_IRQn);
  NVIC_EnableIRQ(UART1_RX_IRQn);
  NVIC_EnableIRQ(UART1_TX_IRQn);

  /* Enable I/O pins at UART1 location #2 */
  uart->ROUTE = UART_ROUTE_RXPEN | UART_ROUTE_TXPEN | UART_ROUTE_LOCATION_LOC2;

  /* Enable UART */
  USART_Enable(uart, usartEnable);
}



/******************************************************************************
 * @brief  uartGetChar function
 *
 *  Note that if there are no pending characters in the receive buffer, this
 *  function will hang until a character is received.
 *
 *****************************************************************************/
uint8_t uartGetChar( )
{
  uint8_t ch;

  /* Check if there is a byte that is ready to be fetched. If no byte is ready, wait for incoming data */
  if (rxBuf.pendingBytes < 1)
  {
    while (rxBuf.pendingBytes < 1) ;
  }

  /* Copy data from buffer */
  ch        = rxBuf.data[rxBuf.rdI];
  rxBuf.rdI = (rxBuf.rdI + 1) % BUFFERSIZE;

  /* Decrement pending byte counter */
  rxBuf.pendingBytes--;

  return ch;
}




/******************************************************************************
 * @brief  uartPutChar function
 *
 *****************************************************************************/
void uartPutChar(uint8_t ch)
{
  /* Check if Tx queue has room for new data */
  if ((txBuf.pendingBytes + 1) > BUFFERSIZE)
  {
    /* Wait until there is room in queue */
    while ((txBuf.pendingBytes + 1) > BUFFERSIZE) ;
  }

  /* Copy ch into txBuffer */
  txBuf.data[txBuf.wrI] = ch;
  txBuf.wrI             = (txBuf.wrI + 1) % BUFFERSIZE;

  /* Increment pending byte counter */
  txBuf.pendingBytes++;

  /* Enable interrupt on USART TX Buffer*/
  USART_IntEnable(uart, UART_IEN_TXBL);
}




/******************************************************************************
 * @brief  uartPutData function
 *
 *****************************************************************************/
void uartPutData(uint8_t * dataPtr, uint32_t dataLen)
{
  uint32_t i = 0;

  /* Check if buffer is large enough for data */
  if (dataLen > BUFFERSIZE)
  {
    /* Buffer can never fit the requested amount of data */
    return;
  }

  /* Check if buffer has room for new data */
  if ((txBuf.pendingBytes + dataLen) > BUFFERSIZE)
  {
    /* Wait until room */
    while ((txBuf.pendingBytes + dataLen) > BUFFERSIZE) ;
  }

  /* Fill dataPtr[0:dataLen-1] into txBuffer */
  while (i < dataLen)
  {
    txBuf.data[txBuf.wrI] = *(dataPtr + i);
    txBuf.wrI             = (txBuf.wrI + 1) % BUFFERSIZE;
    i++;
  }

  /* Increment pending byte counter */
  txBuf.pendingBytes += dataLen;

  /* Enable interrupt on USART TX Buffer*/
  USART_IntEnable(uart, UART_IEN_TXBL);
}

/******************************************************************************
 * @brief  uartGetData function
 *
 *****************************************************************************/
uint32_t uartGetData(uint8_t * dataPtr, uint32_t dataLen)
{
  uint32_t i = 0;

  /* Wait until the requested number of bytes are available */
  if (rxBuf.pendingBytes < dataLen)
  {
    while (rxBuf.pendingBytes < dataLen) ;
  }

  if (dataLen == 0)
  {
    dataLen = rxBuf.pendingBytes;
  }

  /* Copy data from Rx buffer to dataPtr */
  while (i < dataLen)
  {
    *(dataPtr + i) = rxBuf.data[rxBuf.rdI];
    rxBuf.rdI      = (rxBuf.rdI + 1) % BUFFERSIZE;
    i++;
  }

  /* Decrement pending byte counter */
  rxBuf.pendingBytes -= dataLen;

  return i;
}

/***************************************************************************//**
 * @brief Set up Clock Management Unit
 ******************************************************************************/
void cmuSetup(void)
{
  /* Start HFXO and wait until it is stable */
  /* CMU_OscillatorEnable( cmuOsc_HFXO, true, true); */

  /* Select HFXO as clock source for HFCLK */
  /* CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO ); */

  /* Disable HFRCO */
  /* CMU_OscillatorEnable( cmuOsc_HFRCO, false, false ); */

  /* Enable clock for HF peripherals */
  CMU_ClockEnable(cmuClock_HFPER, true);

  /* Enable clock for USART module */
  CMU_ClockEnable(cmuClock_UART1, true);
}


/**************************************************************************//**
 * @brief UART1 RX IRQ Handler
 *
 * Set up the interrupt prior to use
 *
 * Note that this function handles overflows in a very simple way.
 *
 *****************************************************************************/
void UART1_RX_IRQHandler(void)
{
  /* Check for RX data valid interrupt */
  if (uart->IF & UART_IF_RXDATAV)
  {
    /* Copy data into RX Buffer */
    uint8_t rxData = USART_Rx(uart);
    rxBuf.data[rxBuf.wrI] = rxData;
    rxBuf.wrI             = (rxBuf.wrI + 1) % BUFFERSIZE;
    rxBuf.pendingBytes++;

    /* Flag Rx overflow */
    if (rxBuf.pendingBytes > BUFFERSIZE)
    {
      rxBuf.overflow = true;
    }
  }
}

/**************************************************************************//**
 * @brief UART1 TX IRQ Handler
 *
 * Set up the interrupt prior to use
 *
 *****************************************************************************/
void UART1_TX_IRQHandler(void)
{
  /* Check TX buffer level status */
  if (uart->IF & UART_IF_TXBL)
  {
    if (txBuf.pendingBytes > 0)
    {
      /* Transmit pending character */
      USART_Tx(uart, txBuf.data[txBuf.rdI]);
      txBuf.rdI = (txBuf.rdI + 1) % BUFFERSIZE;
      txBuf.pendingBytes--;
    }

    /* Disable Tx interrupt if no more bytes in queue */
    if (txBuf.pendingBytes == 0)
    {
      USART_IntDisable(uart, UART_IEN_TXBL);
    }
  }
}

