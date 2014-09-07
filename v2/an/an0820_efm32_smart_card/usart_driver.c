/******************************************************************************
 * @file usart_driver.c
 * @brief USART/UART ISO 7816 buffered driver
 * @author Silicon Labs
 * @version 1.01
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

#include <stdint.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "hw_config.h"
#include "usart_driver.h"


/* Declare a circular buffer structure to use for Rx and Tx queues */
#define BUFFERSIZE          128

volatile struct circularBuffer
{
  uint8_t  data[BUFFERSIZE];  /* data buffer */
  uint32_t rdI;               /* read index */
  uint32_t wrI;               /* write index */
  uint32_t pendingBytes;      /* count of how many bytes are not yet handled */
  bool     acceptRXData;      /* Should the RX buffer be filled with received data or not */
  bool     overflow;          /* buffer overflow indicator */
} rxBuf = { {0}, 0, 0, 0, false, false };

volatile uint8_t rxErrorCounter = 0;
volatile uint8_t txErrorCounter = 0;
volatile int8_t txStatus = 0;
volatile uint8_t rxByteOnTx;

/* Setup USART1 in async mode for RS232*/
static USART_TypeDef           * usart   = SC_USART_MODULE;
static USART_InitAsync_TypeDef usartInit = USART_INITASYNC_DEFAULT;



/******************************************************************************
* @brief  usartSetup function
*
******************************************************************************/
void usartSetup(void)
{
  cmuSetup();
  
  /* Configure GPIO pin as open drain */
  GPIO_PinModeSet(SC_GPIO_DATA_PORT, SC_GPIO_DATA_PIN, gpioModeWiredAndPullUp, 1);

  /* Prepare struct for initializing USART in asynchronous mode*/
  usartInit.enable       = usartDisable;   /* Don't enable USART upon intialization */
  usartInit.refFreq      = 0;              /* Provide information on reference frequency. When set to 0, the reference frequency is */
  usartInit.baudrate     = SC_BAUD_RATE;   /* Baud rate */
  usartInit.oversampling = usartOVS16;     /* Oversampling. Range is 4x, 6x, 8x or 16x */
  usartInit.databits     = usartDatabits8; /* Number of data bits. Range is 4 to 10 */
  usartInit.parity       = usartEvenParity;  /* Parity mode */
  usartInit.stopbits     = usartStopbits1p5; /* Number of stop bits. Range is 0 to 2, 1.5 for smartcard. */
  usartInit.mvdis        = false;          /* Disable majority voting */
  usartInit.prsRxEnable  = false;          /* Enable USART Rx via Peripheral Reflex System */
  usartInit.prsRxCh      = usartPrsRxCh0;  /* Select PRS channel if enabled */

  /* Initialize USART with usartInit struct */
  USART_InitAsync(usart, &usartInit);
  
  /* Smart card specific settings for T0 mode. */
  usart->CTRL |= USART_CTRL_SCMODE | USART_CTRL_AUTOTRI | USART_CTRL_LOOPBK;

  /* Prepare USART Rx interrupt */
  USART_IntClear(usart, _USART_IF_MASK);
  USART_IntEnable(usart, USART_IF_RXDATAV);
  NVIC_ClearPendingIRQ(USART1_RX_IRQn);
  NVIC_EnableIRQ(USART1_RX_IRQn);
  
  /* Enable I/O pins at USART1 location #2 */
  usart->ROUTE = USART_ROUTE_TXPEN | SC_USART_LOCATION;

  /* Disable reception before enabling uart to discard erroneus stuff while card is unpowered. */
  usartFlushBuffer();
  usartAcceptRX(false);
  
  /* Enable USART */
  USART_Enable(usart, usartEnable);
}

/***************************************************************************//**
 * @brief
 *   Block/Unblock receiver
 *
 * @param[in] block
 *   true to block, false to unblock.
 ******************************************************************************/
void usartBlockReceiver(bool block)
{

  if(block == true)
  {
    usart->CMD = USART_CMD_RXBLOCKEN;
  }
  else
  {
    usart->CMD = USART_CMD_RXBLOCKDIS;
  }
}

/***************************************************************************//**
 * @brief
 *   Flush uart buffer, Always use this when switching between tx/rx operation.
 *
 ******************************************************************************/
void usartFlushBuffer(void)
{
  uint8_t byte;
  
  /* Flush buffer first to make sure we don't have stale data when switching between RX/TX. */
  while(usartGetChar(&byte) != UART_STATUS_NOT_ENOUGH_BYTES_RECEIVED);
  
}

/***************************************************************************//**
 * @brief
 *   Accept/not accept data on uart. Always use this to not accept data
 *   before sending data with uart. This makes sure TX bytes don't end up in rx
 *   queue.  
 *
 * @param[in] accept
 *   true to accept data and put in RX buffer, false to not accept data.
 ******************************************************************************/
void usartAcceptRX(bool accept)
{
  rxBuf.acceptRXData = accept;
}

/******************************************************************************
 * @brief  usartGetChar function
 *
 *  This function will return UART_STATUS based on what happens. 
 *
 * @param[out] receivedChar
 *   Pointer to unsigned char to put received data in. 
 *
 * @return
 *   UART_STATUS status enum.
 *****************************************************************************/
uint8_t usartGetChar(uint8_t * receivedChar)
{

  /* If error counter is too high, report error. */
  if(rxErrorCounter > MAX_ERROR_COUNT)
  {
    rxErrorCounter = 0;
    return UART_STATUS_FRAMING_OR_PARITY_ERROR;
  }  

  /* Check if there is a byte that is ready to be fetched. If no byte is ready, return error. */
  if (rxBuf.pendingBytes < 1)
  {
    return UART_STATUS_NOT_ENOUGH_BYTES_RECEIVED;
  }
  
  /* Copy data from buffer */
  *receivedChar = rxBuf.data[rxBuf.rdI];
  rxBuf.rdI = (rxBuf.rdI + 1) % BUFFERSIZE;

  /* Decrement pending byte counter */
  rxBuf.pendingBytes--;

  return UART_STATUS_SUCCESS;

}


/******************************************************************************
 * @brief  usartPutChar function
 *
 * This function will send single char return UART_STATUS based on what happens. 
 *
 * @param[in] ch
 *   unsigned char to transmit. 
 *
 * @return
 *   UART_STATUS status enum.
 *****************************************************************************/  
 uint8_t usartPutChar(uint8_t ch)
{
  int8_t txSuccess = 0;
  
  /* This function should only be called with rx accept = false. */
  if(rxBuf.acceptRXData){
    return UART_STATUS_USAGE_ERROR;
  }
  
  /* Transmit byte and retransmit if error is detected, up to MAX_ERROR_COUNT times. */
  
  while(txSuccess == 0){
    txStatus = 0;
    USART_Tx(usart, ch);
    
    /* Wait for rx interrupt to receive the transmittet byte and indicate so. */
    while (txStatus == 0);
    
    if( txStatus == -1 )
    {
      /* Error, retransmit */
      txErrorCounter++;
      if(txErrorCounter > MAX_ERROR_COUNT)
      {
        txSuccess = -1;
        txErrorCounter = 0;
      }
    }
    else if( txStatus == 1 && ch == rxByteOnTx)
    {
      /* success */
      txSuccess = 1;
    }
    else
    {
      /* unspecified error, success transmitting wrong byte or something... */
      /* Could be caused by calling this putChar while receiving data, this should */
      /* never happen as the ccid should have full a-priori knowledge of comm-direction. */
      return UART_STATUS_USAGE_ERROR;
    }
    
  }
  
  if(txSuccess == 1)
  {
    return UART_STATUS_SUCCESS;
  }
  else
  {
    return UART_STATUS_FRAMING_OR_PARITY_ERROR;
  }
  
}




/******************************************************************************
 * @brief  usartPutData function
 *
 * This function will send char array and return UART_STATUS based on what happens. 
 *
 * @param[in] dataPtr
 *  pointer to unsigned char buffer to transmit. 
 *
 * @param[in] dataLen
 *  length of buffer to transmit.
 *
 * @return
 *   UART_STATUS status enum.
 *****************************************************************************/
uint8_t usartPutData(uint8_t * dataPtr, uint32_t dataLen)
{
  uint32_t i = 0;
  uint8_t status;
  
  while (i < dataLen)
  {
    status = usartPutChar(*(dataPtr + i));
    i++;
    
    if( status != UART_STATUS_SUCCESS){
      return status;
    }
  }  
    
  return status;
}

/******************************************************************************
 * @brief  usartGetData function
 *
 * This function will try to receive char array and
 * return UART_STATUS based on what happens. 
 *
 * @param[in] dataPtr
 *  pointer to unsigned char buffer to receive to. 
 *
 * @param[in] dataLen
 *  length of buffer to receive, if 0, receive whatever is in receive buffer.
 *
 * @param[out] bytesReceived
 *  pointer to uint32_t, indicating how many bytes was actually received.
 *
 * @return
 *   UART_STATUS status enum.
 *****************************************************************************/
uint8_t usartGetData(uint8_t * dataPtr, uint32_t dataLen, uint32_t* bytesReceived)
{
  uint32_t i = 0;

  /* If error counter is too high, report error. */
  if(rxErrorCounter > MAX_ERROR_COUNT)
  {
    rxErrorCounter = 0;
    return UART_STATUS_FRAMING_OR_PARITY_ERROR;
  }
  
  /* If too few bytes received, return error. */
  if (rxBuf.pendingBytes < dataLen)
  {
    return UART_STATUS_NOT_ENOUGH_BYTES_RECEIVED;
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

  *bytesReceived = i;
  return UART_STATUS_SUCCESS;

}

/***************************************************************************//**
 * @brief Set up Clock Management Unit
 ******************************************************************************/
void cmuSetup(void)
{

  /* Enable clock for HF peripherals */
  CMU_ClockEnable(cmuClock_HFPER, true);

  /* Enable clock for USART module */
  CMU_ClockEnable(cmuClock_USART1, true);
  
  /* Enable clock for GPIO module (required for pin configuration) */
  CMU_ClockEnable(cmuClock_GPIO, true);
  
}


/**************************************************************************//**
 * @brief USART1 RX IRQ Handler
 *
 * Set up the interrupt prior to use
 *
 * Note that this function handles overflows in a very simple way.
 * The interrupt is used to handle both RX and TX bytes, depending on 
 * acceptRXData status.
 * 
 *****************************************************************************/
void USART1_RX_IRQHandler(void)
{
  /* Check for RX data valid interrupt */
  if (usart->STATUS & USART_STATUS_RXDATAV)
  {
    /* Read extended rx reg to get Framing and Parity errors. */
    uint16_t rxData = USART_RxExt(usart);
    
    if(((rxData & 0xC000) == 0) && rxBuf.acceptRXData)
    {
    
      /* Copy data into RX Buffer if no error, reset error counter. */
      rxBuf.data[rxBuf.wrI] = (rxData & 0x00ff);
      rxBuf.wrI             = (rxBuf.wrI + 1) % BUFFERSIZE;
      rxBuf.pendingBytes++;

      /* Successfully received one byte, reset RX error counter. */
      rxErrorCounter = 0;
      
      /* Flag Rx overflow */
      if (rxBuf.pendingBytes > BUFFERSIZE)
      {
        rxBuf.overflow = true;
      }
      
    }
    else if(((rxData & 0xC000) != 0) && rxBuf.acceptRXData)
    {
      /* We have framing or parity error on reception. */
      if(rxErrorCounter < 255)
      {
        rxErrorCounter++;
      }
    }
    else if(((rxData & 0xC000) == 0) && !(rxBuf.acceptRXData))
    {
      /* We have successful TX */
      /* Successfully transmitted one byte, reset TX error counter. */
      txErrorCounter = 0;
      rxByteOnTx = (rxData & 0x00ff);
      txStatus = 1;
    }
    else
    {
      /* TX error */
      if(txErrorCounter < 255)
      {
          txErrorCounter++;
      }
      rxByteOnTx = (rxData & 0x00ff);
      txStatus = -1;
    }
  }
}
