/*
 * termio.h
 *
 *  Created on: Feb 25, 2014
 *      Author: John Alabi
 */

#ifndef TERMIO_H_
#define TERMIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

/** USART used for debugging. */
#define DEBUG_USART                USART1
#define DEBUG_USART_CLOCK          CMU_HFPERCLKEN0_USART1
#define DEBUG_USART_LOCATION       USART_ROUTE_LOCATION_LOC1

#ifdef STK_3300
#define RETARGET_IRQ_NAME  USART1_RX_IRQHandler
#define RETARGET_CLK       cmuClock_USART1
#define RETARGET_IRQn      USART1_RX_IRQn
#define RETARGET_UART      USART1
#define RETARGET_TX        USART_Tx
#define RETARGET_RX        USART_Rx
#define RETARGET_LOCATION  USART_ROUTE_LOCATION_LOC1
#define RETARGET_TXPORT    gpioPortD
#define RETARGET_TXPIN     0
#define RETARGET_RXPORT    gpioPortD
#define RETARGET_RXPIN     1

#elif defined ITM
#define RETARGET_IRQ_NAME  USART0_RX_IRQHandler
#define RETARGET_CLK       cmuClock_USART0
#define RETARGET_IRQn      USART0_RX_IRQn
#define RETARGET_UART      USART0
#define RETARGET_TX        USART_Tx
#define RETARGET_RX        USART_Rx
#define RETARGET_LOCATION  USART_ROUTE_LOCATION_LOC0
#define RETARGET_TXPORT    gpioPortD
#define RETARGET_TXPIN     10
#define RETARGET_RXPORT    gpioPortD
#define RETARGET_RXPIN     11
#endif
  
#if defined(RETARGET_UART)
#include "em_usart.h"
#endif

#if defined(RETARGET_LEUART)
#include "em_leuart.h"
#endif
  
#define clrScreen() 	printf("\x1b[2J")
#define goHome()	printf("\x1b[1,1H")
#define clrLine()	printf("\x1b[K")
#ifdef DEBUG
#define dputs   puts
#endif
  
// For a lil-endian machine
#define htonl(v)                ((uint32_t)(__REV(v)))
#define htons(v)                ((uint16_t)(__REV16(v)))
#define ntohl(v)                ((uint32_t)(__REV(v)))
#define ntohs(v)                ((uint16_t)((__REV16(v))))

  
int dprintf(const char *fmt, ...); 
  
/* receives character from the terminal channel - BLOCKING */
uint8_t TERMIO_GetChar(void);
/* sends a character to the terminal channel
   wait for output buffer empty */
void TERMIO_PutChar(uint8_t ch);
/* initializes the communication channel */
/* set baud rate to 115.2 kbaud and turn on Rx and Tx */
void TERMIO_Init(void);
/* checks for a character from the terminal channel */
int kbhit(void);
void TERMIO_SerialCrLf(int on);
int RETARGET_ReadChar(void);
uint8_t RETARGET_WriteChar(uint8_t c);

#if defined(ccs)
#include <file.h>

int uart_open(const char *path, unsigned flags, int llv_fd);
int uart_close(int dev_fd);
int uart_read(int dev_fd, char *buf, unsigned count);
int uart_write(int dev_fd, const char *buf, unsigned count);
off_t uart_lseek(int dev_fd, off_t offset, int origin);
int uart_unlink(const char *path);
int uart_rename(const char * old_name, const char * new_name);

int mapStdioToUart(void);

#endif

#ifdef __cplusplus
}
#endif

#endif /* TERMIO_H_ */
