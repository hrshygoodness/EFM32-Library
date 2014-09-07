/**************************************************************************//**
 * @file main_legacy_slave.c
 * @brief UART Flow Control Example
 * @author Silicon Labs
 * @version 1.03
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
#include <stdbool.h>
#include <string.h>
#include "em_device.h"
#include "em_usart.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_emu.h"
#include "em_rtc.h"
#include "uart_flow_control.h"

/* This is the actual size of the receive buffer */
#define BUFFER_SIZE 30
   
/* This is the number of elements before we activate flow control */
#define BUFFER_THRESHOLD 20


/* A dummy message to send */
char *message = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";


/* The buffer used to store incoming data */
uint8_t buffer[BUFFER_SIZE];

/* The current index of the receive buffer */
uint8_t bufferIndex = 0;

/* Flag to see if it is time to send a new message */
bool txEvent = false;


void initUART(void)
{
  /* Enable the required clocks */
  CMU_ClockEnable(cmuClock_USART1, true);
  CMU_ClockEnable(cmuClock_GPIO, true);
  
  /* Initialize the UART */
  USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;
  USART_InitAsync(USART1, &uartInit);  
  
  /* Enable TX and RX pin */
  GPIO_PinModeSet(gpioPortD, 1, gpioModeInput, 0);
  GPIO_PinModeSet(gpioPortD, 0, gpioModePushPull, 1);
  USART1->ROUTE |= USART_ROUTE_RXPEN | USART_ROUTE_TXPEN | USART_ROUTE_LOCATION_LOC1;
  
  /* Enable interrupt when character arrives */
  USART1->IEN |= USART_IEN_RXDATAV;
  NVIC_EnableIRQ(USART1_RX_IRQn);
}

void messageSent(void)
{
  /* Do nothing */
}

void rxHandler(uint8_t receivedByte)
{
  /* Store character in buffer */
  buffer[bufferIndex++] = receivedByte;
  
  /* Simply wrap around buffer when the end is reached */
  if ( bufferIndex >= BUFFER_SIZE ) {
    bufferIndex = 0;
  }
}

void initFlowControl(void)
{
  UART_FC_Config_TypeDef fcConfig;
  
  /* Configure the RTS pin */
  fcConfig.rtsPin.port = gpioPortD;
  fcConfig.rtsPin.pin = 2;
  fcConfig.rtsPolarity = activeLow;
  
  /* Configure the CTS pin */
  fcConfig.ctsPin.port = gpioPortD;
  fcConfig.ctsPin.pin = 3;
  fcConfig.ctsPolarity = activeLow;
  
  /* Set the mode */
  fcConfig.mode = modeLegacyHW_Slave;
  
  /* Register callback functions */
  fcConfig.txCallback = messageSent;
  fcConfig.rxCallback = rxHandler;
  
  /* Initialize Flow Control */
  UART_FC_Init(&fcConfig);
}


void initRTC(void)
{  
  /* Enable the clock for the RTC */
  CMU_ClockEnable(cmuClock_RTC, true);
  
  /* The CORELE clock must be enabled before the CPU 
   * can access the LF peripherals */
  CMU_ClockEnable(cmuClock_CORELE, true);
  
  /* Prescale the RTC */
  CMU_ClockDivSet(cmuClock_RTC, cmuClkDiv_512);
  
  /* Configure RTC to wrap around on COMP0 */
  RTC_Init_TypeDef rtcInit = RTC_INIT_DEFAULT;
  rtcInit.comp0Top = true;
  rtcInit.enable = false;
  RTC_Init(&rtcInit);
  
  /* Overflow every second */
  RTC->COMP0 = CMU_ClockFreqGet(cmuClock_RTC)-1;
  
  /* Enable interrupt */
  RTC->IEN = RTC_IEN_COMP0;
  NVIC_EnableIRQ(RTC_IRQn);
  
  /* Start the RTC */
  RTC_Enable(true);
}

void processBuffer(void)
{
  uint32_t i;
    
  /* Disable reception by deasserting RTS signal. 
   * Note that the actual RX interrupt is never disabled. 
   * We assume the other end will respect the signal.  */
  UART_FC_disableRx();
  
  /* Pretend that we have to do a large processing */
  for ( i=0; i<100000; i++ );
  
  /* Empty the buffer */
  bufferIndex = 0;
  
  /* Enable reception again */
  UART_FC_enableRx();
}
  
  

int main(void)
{
  CHIP_Init();
  
  /* A crystal is usually needed for UART communication */
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
  
  /* Enable the LFXO for RTC */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);

  initUART();
  initFlowControl();
  initRTC();
  
  while(1) {
    
    /* Stay in sleep until something happens */
    EMU_EnterEM1();
    
    /* It is time to send a message */
    if ( txEvent ) {
      UART_FC_StartTx( (uint8_t*)message, strlen(message) );
      txEvent = false;
    }
    
    /* The buffer is almost filled up */
    if ( bufferIndex > BUFFER_THRESHOLD )
    {
      /* Do some processing. This will temporarily halt
       * transmission by deasserting RTS */
      processBuffer();
    }
  }
}



void RTC_IRQHandler(void)
{
  /* Clear the flag */
  RTC->IFC = RTC_IF_COMP0;
 
  /* Request a new message to be sent */
  txEvent = true;
}
