/******************************************************************************
 * @file main_spi.c
 * @brief SPI Demo Application
 * @author Silicon Labs
 * @version 1.14
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

#include <string.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "spi.h"
#include "usart.h"

/* Buffers */
char transmitBuffer[] = "EFM32 SPI";
#define            BUFFERSIZE    (sizeof(transmitBuffer) / sizeof(char))
char receiveBuffer[BUFFERSIZE];
char receiveBuffer2[BUFFERSIZE];



/******************************************************************************
 * @brief  initializing spi settings
 *
 *****************************************************************************/
void init(void)
{
  /* Enabling clock to USART 1 and 2*/
  CMU_ClockEnable(cmuClock_USART1, true);
  CMU_ClockEnable(cmuClock_USART2, true);
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Setup UART */
  SPI_setup(USART1_NUM, GPIO_POS1, false);
  SPI_setup(USART2_NUM, GPIO_POS0, true);
}



/******************************************************************************
 * @brief  Main function
 * Main is called from _program_start, see assembly startup file
 *****************************************************************************/
int main(void)
{
  /* Initialize chip */
  CHIP_Init();

  /* Initalizing */
  init();

  /* Data transmission to slave */
  /* ************************** */
  /*Setting up both RX and TX interrupts for slave */
  SPI1_setupSlaveInt(receiveBuffer2, BUFFERSIZE, NO_TX, NO_TX);
  /* Setting up RX interrupt for master */
  SPI2_setupRXInt(NO_RX, NO_RX);
  /* Transmitting data */
  USART2_sendBuffer(transmitBuffer, BUFFERSIZE);


  /* Data reception from slave */
  /* ************************** */
  /*Setting up both RX and TX interrupts for slave */
  SPI1_setupSlaveInt(NO_RX, NO_RX, transmitBuffer, BUFFERSIZE);
  /* Setting up RX interrupt for master */
  SPI2_setupRXInt(receiveBuffer, BUFFERSIZE);
  /* Receiving data by transmitting dummy data to slave */
  USART2_sendBuffer(NO_TX, BUFFERSIZE);

  
  /* Clearing the receive buffers */
  memset(receiveBuffer, '\0', BUFFERSIZE);
  memset(receiveBuffer2, '\0', BUFFERSIZE);

  
  /* Transmitting data both ways simultanously */
  /* ***************************************** */
  /*Setting up both RX and TX interrupts for slave */
  SPI1_setupSlaveInt(receiveBuffer2, BUFFERSIZE, transmitBuffer, BUFFERSIZE);
  /* Setting up RX interrupt for master */
  SPI2_setupRXInt(receiveBuffer, BUFFERSIZE);
  /* Receiving data by transmitting dummy data to slave */
  USART2_sendBuffer(transmitBuffer, BUFFERSIZE);
  

  while (1) ;
}



