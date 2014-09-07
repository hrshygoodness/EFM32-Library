/**************************************************************************//**
 * @file uart_flow_control.h
 * @brief UART Flow Control Header File
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
#ifndef __UART_FLOW_CONTROL
#define __UART_FLOW_CONTROL

#include "em_gpio.h"

typedef enum
{
  /* Signal is low when asserted */
  activeLow = 0,
  
  /* Signal is high when asserted */
  activeHigh = 1

} UART_FC_Polarity;

typedef enum
{
  modeHW,
  modeSW,
  modeLegacyHW_Master,
  modeLegacyHW_Slave
} UART_FC_Mode;
  


typedef struct _USART_FC_Pin
{
  GPIO_Port_TypeDef port;
  unsigned int      pin;
} UART_FC_Pin;



typedef struct
{
  /* The location of the RTS pin */
  UART_FC_Pin rtsPin;
  
  /* Active polarity of RTS pin */
  UART_FC_Polarity rtsPolarity;
  
  /* The location of the CTS pin */
  UART_FC_Pin ctsPin;
  
  /* Active polarity of CTS pin */
  UART_FC_Polarity ctsPolarity;
  
  /* Selects the type of flow control */
  UART_FC_Mode mode;
  
  /* Callback function which is called when
   * the entire message has been sent. 
   * If a callback is not used, this field MUST
   * be set to NULL. */
  void (*txCallback)(void);
  
  /* Callback function which is called for every byte received. 
   * This MUST be defined */
  void (*rxCallback)(uint8_t);
  
} UART_FC_Config_TypeDef;


void UART_FC_Init(UART_FC_Config_TypeDef *);

bool UART_FC_StartTx(uint8_t *data, uint32_t numBytes);

void UART_FC_enableRx(void);

void UART_FC_disableRx(void);


#endif /* __UART_FLOW_CONTROL */


