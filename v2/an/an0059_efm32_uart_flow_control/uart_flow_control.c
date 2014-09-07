/**************************************************************************//**
 * @file uart_flow_control.c
 * @brief UART Flow Control
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
 /**
 * This file implements a simple interface to add hardware flow control to an UART
 * The API functions are: 
 * 
 * void UART_FC_Init(UART_FC_Config_TypeDef *config) 
 * bool UART_FC_StartTx(uint8_t *data, uint32_t numBytes)   
 * 
 * In addition, two callback functions are defined in the config struct. These are
 * called when a byte is received and the full message has been transmitted, respectively. 
 * 
 * Three different methods of flow control can be chosen, Hardware, Software or Legacy Hardware.
 * See the uart_flow_control.h header file for allowed values in the config enums. 
 * 
 *****************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_usart.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_int.h"
#include "uart_flow_control.h"

#define XON  0x11
#define XOFF 0x13


/* Pointer to the next character to be sent */
static uint8_t *nextByte = 0;

/* Number of bytes left of the message. Will be zero when no message 
 * is pending */
static uint32_t remainingBytes = 0;

/* Local copy of the config */
static UART_FC_Config_TypeDef activeConfig;

/* Flag indicating the status of the flow control. 
 * If this is true we are allowed to send more data */
static bool txEnabled = false;


/* Sends the software flow control signal XON */
static void _sendXON(void) 
{ 
  /* Disable TXBL interrupt in case there already is an 
   * ongoing transmission. */ 
  USART1->IEN &= ~USART_IEN_TXBL;
  
  /* Wait until TX buffer is ready */
  while ( !(USART1->STATUS & USART_STATUS_TXBL) );
  
  /* Send signal */
  USART_Tx(USART1, XON);
  
  /* Enable TXBL interrupt again */
  INT_Disable();
  if ( txEnabled && remainingBytes > 0 ) {
    USART1->IEN |= USART_IEN_TXBL;
  }
  INT_Enable();
}

/* Sends the software flow control signal XOFF */
static void _sendXOFF(void)
{
  /* Disable TXBL interrupt in case there already is an 
   * ongoing transmission. */ 
  USART1->IEN &= ~USART_IEN_TXBL;
  
  /* Wait until TX buffer is ready */
  while ( !(USART1->STATUS & USART_STATUS_TXBL) );
  
  /* Send signal */
  USART_Tx(USART1, XOFF);
  
  /* Enable TXBL interrupt again */
  INT_Disable();
  if ( txEnabled && remainingBytes > 0 ) {
    USART1->IEN |= USART_IEN_TXBL;
  }
  INT_Enable();
}



/*****************************************************************************
 * @brief Indicate that we are ready to receive data. 
 *****************************************************************************/
void UART_FC_enableRx(void)
{
  switch (activeConfig.mode) {
  case modeHW:
    GPIO_PortOutSetVal(activeConfig.rtsPin.port, activeConfig.rtsPolarity << activeConfig.rtsPin.pin, 1 << activeConfig.rtsPin.pin);
    break;
  case modeSW:
    _sendXON();
    break;
  case modeLegacyHW_Slave:
    GPIO_PortOutSetVal(activeConfig.ctsPin.port, activeConfig.ctsPolarity << activeConfig.ctsPin.pin, 1 << activeConfig.ctsPin.pin);
    break;
  case modeLegacyHW_Master:
    /* This is an error, the master cannot control the CTS pin */
    while(1);
  }
}
/*****************************************************************************
 * @brief Indicate that we are NOT ready to receive more data. 
 *****************************************************************************/
void UART_FC_disableRx(void)
{
  switch(activeConfig.mode) {
  case modeHW:
    GPIO_PortOutSetVal(activeConfig.rtsPin.port, !activeConfig.rtsPolarity << activeConfig.rtsPin.pin, 1 << activeConfig.rtsPin.pin);
    break;
  case modeSW:
    _sendXOFF();
    break;
  case modeLegacyHW_Slave:
    GPIO_PortOutSetVal(activeConfig.ctsPin.port, !activeConfig.ctsPolarity << activeConfig.ctsPin.pin, 1 << activeConfig.ctsPin.pin);
    break;
  case modeLegacyHW_Master:
    /* This is an error, the master cannot control the CTS pin */
    while(1);
  }
}

/*****************************************************************************
 * @brief Called when a message has been sent
 * Will call the callback function, if it is set 
 *****************************************************************************/
static void _UART_FC_finishTransmission(void)
{
  switch(activeConfig.mode)
  {
  case modeHW:
    break;
  case modeSW:
    break;
  case modeLegacyHW_Master:
    /* Disable RTS to tell that we are finished transmitting */
    GPIO_PortOutSetVal(activeConfig.rtsPin.port, !activeConfig.rtsPolarity << activeConfig.rtsPin.pin, 1 << activeConfig.rtsPin.pin);
    break;
  case modeLegacyHW_Slave:
    UART_FC_disableRx();
    break;
  }
  
  /* Execute callback function */
  if ( activeConfig.txCallback != NULL ) {
    activeConfig.txCallback();
  }
}

/*****************************************************************************
 * Initialize Hardware Flow Control
 *****************************************************************************/
static void _UART_FC_InitHW(void)
{
  /* Enable RTS initially */
  GPIO_PinModeSet(activeConfig.rtsPin.port, activeConfig.rtsPin.pin, gpioModePushPull, activeConfig.rtsPolarity);

  /* Check if CTS is asserted and update the flag */
  if ( (GPIO->P[activeConfig.ctsPin.port].DIN & (1 << activeConfig.ctsPin.pin)) >> activeConfig.ctsPin.pin == activeConfig.ctsPolarity ) {
    txEnabled = true;
  }

  /* Enable interrupt on CTS pin. */
  GPIO_PinModeSet(activeConfig.ctsPin.port, activeConfig.ctsPin.pin, gpioModeInput, 0);
  GPIO_IntConfig(activeConfig.ctsPin.port, activeConfig.ctsPin.pin, true, true, true);
  if ( activeConfig.ctsPin.pin % 2 == 0 ) {
    NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  } else {
    NVIC_EnableIRQ(GPIO_ODD_IRQn);
  }
}

/*****************************************************************************
 * Initialize Legacy Hardware Flow Control, Master (CTE) Mode
 *****************************************************************************/
static void _UART_FC_InitLegacyHWMaster(void)
{
  /* Disable RTS initially */
  GPIO_PinModeSet(activeConfig.rtsPin.port, activeConfig.rtsPin.pin, gpioModePushPull, !activeConfig.rtsPolarity);
  
  /* Check if CTS is asserted and update the flag */
  if ( (GPIO->P[activeConfig.ctsPin.port].DIN & (1 << activeConfig.ctsPin.pin)) >> activeConfig.ctsPin.pin == activeConfig.ctsPolarity ) {
    txEnabled = true;
  }
  
  /* Enable interrupt on CTS pin. */
  GPIO_PinModeSet(activeConfig.ctsPin.port, activeConfig.ctsPin.pin, gpioModeInput, 0);
  GPIO_IntConfig(activeConfig.ctsPin.port, activeConfig.ctsPin.pin, true, true, true);
  if ( activeConfig.ctsPin.pin % 2 == 0 ) {
    NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  } else {
    NVIC_EnableIRQ(GPIO_ODD_IRQn);
  }
}

/*****************************************************************************
 * Initialize Legacy Hardware Flow Control, Slave (DCE) Mode
 *****************************************************************************/
static void _UART_FC_InitLegacyHWSlave(void)
{
  /* Disable CTS initially */
  GPIO_PinModeSet(activeConfig.ctsPin.port, activeConfig.ctsPin.pin, gpioModePushPull, !activeConfig.ctsPolarity);
  
  /* Enable TX. The slave is always allowed to send data */
  txEnabled = true;
  
  /* Enable interrupt on RTS pin. */
  GPIO_PinModeSet(activeConfig.rtsPin.port, activeConfig.rtsPin.pin, gpioModeInput, 0);
  GPIO_IntConfig(activeConfig.rtsPin.port, activeConfig.rtsPin.pin, true, true, true);
  if ( activeConfig.rtsPin.pin % 2 == 0 ) {
    NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  } else {
    NVIC_EnableIRQ(GPIO_ODD_IRQn);
  }
}


/*****************************************************************************
 * @brief Configures the pins and interrupts according to the config struct. 
 *
 * @param[in] config The configuration struct. 
 *****************************************************************************/
void UART_FC_Init(UART_FC_Config_TypeDef *config) 
{
  /* Save config */
  memcpy(&activeConfig, config, sizeof(UART_FC_Config_TypeDef));
 
  /* Enable TX interrupt in NVIC. Note that the TXBL interrupt in USART1 is not
   * enabled here. The TXBL interrupt will be enabled and disabled based on 
   * the CTS signal */
  NVIC_EnableIRQ(USART1_TX_IRQn);
  
  /* Initialize the chosen mode */
  if ( activeConfig.mode == modeHW ) {
    _UART_FC_InitHW();
  } else if ( activeConfig.mode == modeSW ) {
    txEnabled = true;
  } else if ( activeConfig.mode == modeLegacyHW_Master ) {
    _UART_FC_InitLegacyHWMaster();
  } else if ( activeConfig.mode == modeLegacyHW_Slave ) {
    _UART_FC_InitLegacyHWSlave();
  }
}


/*****************************************************************************
 * @brief Start sending a message with with flow control
 * 
 * @details 
 *   Will start sending a message with flow control. The method returns immediately
 *   and uses interrupts to send the actual message. If a previus transmission is incomplete
 *   this method will not start a new transmission and the method will return false.  
 * 
 * @returns True if the transmission was started. 
 *****************************************************************************/
bool UART_FC_StartTx(uint8_t *data, uint32_t numBytes)
{
  /* Wait until previous transfer is complete */
  if ( remainingBytes > 0 ) {
    return false;
  }

  /* Store location and size of the message */
  nextByte = data;
  remainingBytes = numBytes;
  
  switch (activeConfig.mode) {
  case modeHW:
  case modeSW:
    /* Enable TXBL interrupt if allowed by flow control
     * Interrupts must be disabled because the GPIO interrupt
     * can change the txEnabled variable */
    INT_Disable();
    if ( txEnabled )
    {
      /* Enable TXBL interrupt */
      USART1->IEN |= USART_IEN_TXBL;
    }
    INT_Enable();
    break;
    
  case modeLegacyHW_Slave:
    /* The slave is always allowed to transmit */
    USART1->IEN |= USART_IEN_TXBL;   
    
  case modeLegacyHW_Master:
    /* Assert RTS to tell the slave we want to transmit */
    GPIO_PortOutSetVal(activeConfig.rtsPin.port, activeConfig.rtsPolarity << activeConfig.rtsPin.pin, 1 << activeConfig.rtsPin.pin);
    
    /* Enable TXBL interrupt if CTS is asserted. 
     * Interrupts must be disabled because the GPIO interrupt
     * can change the txEnabled variable */
    INT_Disable();
    if ( txEnabled )
    {
      /* Enable TXBL interrupt */
      USART1->IEN |= USART_IEN_TXBL;
    }
    INT_Enable();
    break;
  }
    
  return true;
}




/*****************************************************************************
 * Handler for the TXBL interrupt. 
 *****************************************************************************/
void USART1_TX_IRQHandler(void)
{
  uint32_t flags = USART1->IF;
  USART1->IFC = flags;
  

  if ( flags & USART_IF_TXC && remainingBytes == 0 ) 
  {
    /* Disable TXC interrupt */
    USART1->IEN &= ~USART_IEN_TXC;
    NVIC_ClearPendingIRQ(USART1_TX_IRQn);
    
    /* Finish up the transmission and call the callback function */
    _UART_FC_finishTransmission();
  } 
  else if ( flags & USART_IF_TXBL ) 
  {
    if ( remainingBytes > 0 ) {
      
      /* Send next byte and increment pointer */
      USART1->TXDATA = *nextByte++;
          
      if ( --remainingBytes == 0 )
      {
        /* Disable TXBL and enable TXC */
        USART1->IEN &= ~USART_IEN_TXBL;
        NVIC_ClearPendingIRQ(USART1_TX_IRQn);
        USART1->IEN |= USART_IEN_TXC;
      }
    }
  }
}

/*****************************************************************************
 * Handler for the RX interrupt. 
 *****************************************************************************/
void USART1_RX_IRQHandler(void)
{
  /* Clear the flag(s) */
  USART1->IFC = USART1->IF;
  
  uint8_t rxData = USART1->RXDATA;
  
  /* If the mode is SW, check first if the byte was one of the flow control characters */
  if ( activeConfig.mode == modeSW && rxData == XON ) 
  {
    /* XON received, enable TX */
    txEnabled = true;
    if ( remainingBytes > 0 ) {
      USART1->IEN |= USART_IEN_TXBL;
    }
  } 
  else if ( activeConfig.mode == modeSW && rxData == XOFF ) 
  {
    /* XOFF received, disable TX */
    txEnabled = false;
    USART1->IEN &= ~USART_IEN_TXBL;
  } else {
    
    /* Call the callback function with the received byte */
    activeConfig.rxCallback(rxData);
  }
}

/*****************************************************************************
 * This handler is triggered on any transition on the CTS pin in HW mode. When triggered
 * it will update the state of the txEnabled flag. If a message is currently pending
 * it will also toggle on/off the TXBL interrupt.
 *****************************************************************************/
void modeHW_CTSHandler(void)
{
  uint32_t flags = GPIO->IF;
  GPIO->IFC = flags; 
  
  if ( flags && (1 << activeConfig.ctsPin.pin) ) {
    
    /* Check if CTS is asserted */
    if ( (GPIO->P[activeConfig.ctsPin.port].DIN & (1 << activeConfig.ctsPin.pin)) >> activeConfig.ctsPin.pin == activeConfig.ctsPolarity ) 
    {
      /* CTS is asserted. Enable TXBL interrupt if a message is pending */
      txEnabled = true;
      if ( remainingBytes > 0 ) {
        USART1->IEN |= USART_IEN_TXBL;
      }
    } else {
      /* CTS is deasserted. Disable TXBL interrupt */
      txEnabled = false;
      USART1->IEN &= ~USART_IEN_TXBL;
    }
  }
}

/*****************************************************************************
 * This handler is used by the Legacy Hardware Master Mode. It is triggered on any transition on the CTS pin. 
 * When triggered it will update the state of the txEnabled flag. If a message is currently pending
 * it will also toggle on/off the TXBL interrupt.
 *****************************************************************************/
void modeLegacyHW_CTSHandler(void)
{
  uint32_t flags = GPIO->IF;
  GPIO->IFC = flags; 
  
  if ( flags && (1 << activeConfig.ctsPin.pin) ) {
    
    /* Check if CTS is asserted */
    if ( (GPIO->P[activeConfig.ctsPin.port].DIN & (1 << activeConfig.ctsPin.pin)) >> activeConfig.ctsPin.pin == activeConfig.ctsPolarity ) 
    {
      /* CTS is asserted. Enable TXBL interrupt if a message is pending */
      txEnabled = true;
      if ( remainingBytes > 0 ) {
        USART1->IEN |= USART_IEN_TXBL;
      }
    } else {
      /* CTS is deasserted. Disable TXBL interrupt */
      txEnabled = false;
      USART1->IEN &= ~USART_IEN_TXBL;
    }
  }
}  

/*****************************************************************************
 * This handler is used by the Legacy Hardware Slave Mode. It is triggered on any transition on the RTS pin. 
 * If RTS is asserted this function will assert CTS. If RTS is deasserted it will deassert CTS. 
 *****************************************************************************/
void modeLegacyHW_RTSHandler(void)
{
  uint32_t flags = GPIO->IF;
  GPIO->IFC = flags; 
  
  if ( flags && (1 << activeConfig.rtsPin.pin) ) {
    
    /* Check if RTS is asserted */
    if ( (GPIO->P[activeConfig.rtsPin.port].DIN & (1 << activeConfig.rtsPin.pin)) >> activeConfig.rtsPin.pin == activeConfig.rtsPolarity ) 
    {
      UART_FC_enableRx();
    } else {
      UART_FC_disableRx();
    }
  }
}


/*****************************************************************************
 * Common IRQ handler for the GPIO interrupts 
 *****************************************************************************/
void GPIO_COMMON_IRQHandler(void)
{
  switch (activeConfig.mode) {
  case modeHW:
    modeLegacyHW_CTSHandler();
    break;
  case modeLegacyHW_Master:
    modeLegacyHW_CTSHandler();
    break;
  case modeLegacyHW_Slave:
    modeLegacyHW_RTSHandler();
    break;
  default:
    /* Error */
    while(1);
  }
}


void GPIO_ODD_IRQHandler(void)
{
  GPIO_COMMON_IRQHandler();
}

void GPIO_EVEN_IRQHandler(void)
{
  GPIO_COMMON_IRQHandler();
}

