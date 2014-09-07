/******************************************************************************
 * @file usart_driver.h
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

#ifndef __USART_DRIVER_H
#define __USART_DRIVER_H

typedef enum
{
  UART_STATUS_SUCCESS,
  UART_STATUS_NOT_ENOUGH_BYTES_RECEIVED,
  UART_STATUS_FRAMING_OR_PARITY_ERROR,
  UART_STATUS_USAGE_ERROR,
}UART_STATUS;

#define MAX_ERROR_COUNT (100)

/******************************************************************************
* @brief  usartSetup function
*
******************************************************************************/
void usartSetup(void);

/***************************************************************************//**
 * @brief
 *   Block/Unblock receiver
 *
 * @param[in] block
 *   true to block, false to unblock.
 ******************************************************************************/
void usartBlockReceiver(bool block);

/***************************************************************************//**
 * @brief
 *   Flush uart buffer, Always use this when switching between tx/rx operation.
 *
 ******************************************************************************/
void usartFlushBuffer(void);

/***************************************************************************//**
 * @brief
 *   Accept/not accept data on uart. Always use this to not accept data
 *   before sending data with uart. This makes sure TX bytes don't end up in rx
 *   queue.  
 *
 * @param[in] accept
 *   true to accept data and put in RX buffer, false to not accept data.
 ******************************************************************************/
void usartAcceptRX(bool accept);

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
uint8_t usartGetChar(uint8_t * receivedChar);

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
 uint8_t usartPutChar(uint8_t ch);

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
uint8_t usartPutData(uint8_t * dataPtr, uint32_t dataLen);

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
uint8_t usartGetData(uint8_t * dataPtr, uint32_t dataLen, uint32_t* bytesReceived);

/***************************************************************************//**
 * @brief Set up Clock Management Unit
 ******************************************************************************/
void cmuSetup(void);

#endif /* __USART_DRIVER_H */
