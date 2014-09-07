/*
*  [one-line description]
*
*  Copyright (c) 2011-2012 Pervasive Displays Inc.
*
*  Authors: Pervasive Displays Inc. <techsupport@pervasivedisplays.com>
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*  1. Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include "USB_Driver.h"

/*** Typedef's and defines. ***/

/* Define USB endpoint addresses */
#define EP_DATA_OUT       0x01  /* Endpoint for USB data reception.       */
#define EP_DATA_IN        0x81  /* Endpoint for USB data transmission.    */
#define EP_NOTIFY         0x82  /* The notification endpoint (not used).  */

#define BULK_EP_SIZE     USB_MAX_EP_SIZE  /* This is the max. ep size.    */
#define USB_RX_BUF_SIZ   BULK_EP_SIZE /* Packet size when receiving on USB*/
#define USB_TX_BUF_SIZ   127    /* Packet size when transmitting on USB.  */

/* Calculate a timeout in ms corresponding to 5 char times on current     */
/* baudrate. Minimum timeout is set to 10 ms.                             */
#define RX_TIMEOUT    EFM32_MAX(10U, 50000 / (cdcLineCoding.dwDTERate))

/* The serial port LINE CODING data structure, used to carry information  */
/* about serial port baudrate, parity etc. between host and device.       */
EFM32_PACK_START(1)
typedef struct
{
  uint32_t dwDTERate;               /** Baudrate                            */
  uint8_t  bCharFormat;             /** Stop bits, 0=1 1=1.5 2=2            */
  uint8_t  bParityType;             /** 0=None 1=Odd 2=Even 3=Mark 4=Space  */
  uint8_t  bDataBits;               /** 5, 6, 7, 8 or 16                    */
  uint8_t  dummy;                   /** To ensure size is a multiple of 4 bytes.*/
} __attribute__ ((packed)) cdcLineCoding_TypeDef;
EFM32_PACK_END()

/*** Function prototypes. ***/

static int  UsbDataReceived(USB_Status_TypeDef status, uint32_t xferred,uint32_t remaining);
static int  LineCodingReceived(USB_Status_TypeDef status,uint32_t xferred,uint32_t remaining);
static int  SetupCmd(const USB_Setup_TypeDef *setup);
static void StateChange(USBD_State_TypeDef oldState,USBD_State_TypeDef newState);
static void UartRxTimeout(void);
static int UsbDataTransmitted(USB_Status_TypeDef status,uint32_t xferred,uint32_t remaining);
/*** Include device descriptor definitions. ***/

#include "descriptors.h"


/*** Variables ***/

/*
 * The LineCoding variable must be 4-byte aligned as it is used as USB
 * transmit and receive buffer
 */
EFM32_ALIGN(4)
EFM32_PACK_START(1)
static cdcLineCoding_TypeDef __attribute__ ((aligned(4))) cdcLineCoding =
{
  115200, 0, 0, 8, 0
};
EFM32_PACK_END()

STATIC_UBUF(usbRxBuffer0, USB_RX_BUF_SIZ);    /* USB receive buffers.   */
//STATIC_UBUF(usbRxBuffer1, USB_RX_BUF_SIZ);
//STATIC_UBUF(uartRxBuffer0, USB_TX_BUF_SIZ);   /* UART receive buffers.  */
//STATIC_UBUF(uartRxBuffer1, USB_TX_BUF_SIZ);

static  uint8_t  *usbRxBuffer =  usbRxBuffer0;
//static  uint8_t  *uartRxBuffer =  uartRxBuffer0;

//static int    usbRxIndex, usbBytesReceived;
//static int    uartRxIndex, uartRxCount;
static USBReceiveHandler _OnUSBReceiveHandler=NULL;
int  USB_Driver_Init( USBReceiveHandler OnUSBReceiveHandler)
{
	       int result;
 		result=USBD_Init(&initstruct);
		 /*
		   * When using a debugger it is practical to uncomment the following three
		   * lines to force host to re-enumerate the device.
		   */
		  USBD_Disconnect();
		  USBTIMER_DelayMs(1000);
		  USBD_Connect();
		  _OnUSBReceiveHandler=OnUSBReceiveHandler;
		  return result;
}
void USB_Driver_Disconnect(void)
{
	USBD_Disconnect();
}
void USB_Driver_Connect(void)
{
  	USBD_Connect();
}

void USB_Driver_Restart(void)
{
  	USBD_Disconnect();
	USBTIMER_DelayMs(1000);
	USBD_Connect();
}

void USB_Send(void *data, int byteCount)
{
	 USBD_Write(EP_DATA_IN, (void*) data, byteCount, UsbDataTransmitted);
}
/**************************************************************************//**
 * @brief Callback function called whenever a new packet with data is received
 *        on USB.
 *
 * @param[in] status    Transfer status code.
 * @param[in] xferred   Number of bytes transferred.
 * @param[in] remaining Number of bytes not transferred.
 *
 * @return USB_STATUS_OK.
 *****************************************************************************/
static int UsbDataReceived(USB_Status_TypeDef status,
                           uint32_t xferred,
                           uint32_t remaining)
{
  (void) remaining;            /* Unused parameter */

  if ((status == USB_STATUS_OK) && (xferred > 0))
  {
 
      /* Start a new USB receive transfer. */
      USBD_Read(EP_DATA_OUT, (void*)usbRxBuffer, xferred, UsbDataReceived);
       if(_OnUSBReceiveHandler!=NULL)_OnUSBReceiveHandler( (void*)usbRxBuffer, xferred);
  }
  return USB_STATUS_OK;
}


/**************************************************************************//**
 * @brief Callback function called whenever a packet with data has been
 *        transmitted on USB
 *
 * @param[in] status    Transfer status code.
 * @param[in] xferred   Number of bytes transferred.
 * @param[in] remaining Number of bytes not transferred.
 *
 * @return USB_STATUS_OK.
 *****************************************************************************/
  
static int UsbDataTransmitted(USB_Status_TypeDef status,
                              uint32_t xferred,
                              uint32_t remaining)
{
  (void) xferred;              
  (void) remaining;            

  if (status == USB_STATUS_OK)
  {
 
     // USBD_Write(EP_DATA_IN, (void*) uartRxBuffer[ uartRxIndex ^ 1],
      //           uartRxCount, UsbDataTransmitted);
     // uartRxCount = 0;
     // USBTIMER_Start(0, RX_TIMEOUT, UartRxTimeout);
      
  }
  return USB_STATUS_OK;
}


/**************************************************************************//**
 * @brief
 *   Callback function called each time the USB device state is changed.
 *   Starts CDC operation when device has been configured by USB host.
 *
 * @param[in] oldState The device state the device has just left.
 * @param[in] newState The new device state.
 *****************************************************************************/
static void StateChange(USBD_State_TypeDef oldState,
                        USBD_State_TypeDef newState)
{
  if (newState == USBD_STATE_CONFIGURED)
  {
    /* We have been configured, start CDC functionality ! */

    if (oldState == USBD_STATE_SUSPENDED)   /* Resume ?   */
    {
    }

    /* Start receiving data from USB host. */

    USBD_Read(EP_DATA_OUT, (void*) usbRxBuffer,
              USB_RX_BUF_SIZ, UsbDataReceived);

  
    USBTIMER_Start(0, RX_TIMEOUT, UartRxTimeout);
  }

  else if ((oldState == USBD_STATE_CONFIGURED) &&
           (newState != USBD_STATE_SUSPENDED))
  {
    /* We have been de-configured, stop CDC functionality */
    USBTIMER_Stop(0);

  }

  else if (newState == USBD_STATE_SUSPENDED)
  {
    /* We have been suspended, stop CDC functionality */
    /* Reduce current consumption to below 2.5 mA.    */
    USBTIMER_Stop(0);
  }
}

/**************************************************************************//**
 * @brief
 *   Handle USB setup commands. Implements CDC class specific commands.
 *
 * @param[in] setup Pointer to the setup packet received.
 *
 * @return USB_STATUS_OK if command accepted.
 *         USB_STATUS_REQ_UNHANDLED when command is unknown, the USB device
 *         stack will handle the request.
 *****************************************************************************/
static int SetupCmd(const USB_Setup_TypeDef *setup)
{
  int retVal = USB_STATUS_REQ_UNHANDLED;

  if ((setup->Type == USB_SETUP_TYPE_CLASS) &&
      (setup->Recipient == USB_SETUP_RECIPIENT_INTERFACE))
  {
    switch (setup->bRequest)
    {
    case USB_CDC_GETLINECODING:
      /********************/
      if ((setup->wValue == 0) &&
          (setup->wIndex == 0) &&               /* Interface no.            */
          (setup->wLength == 7) &&              /* Length of cdcLineCoding  */
          (setup->Direction == USB_SETUP_DIR_IN))
      {
        /* Send current settings to USB host. */
        USBD_Write(0, (void*) &cdcLineCoding, 7, NULL);
        retVal = USB_STATUS_OK;
      }
      break;

    case USB_CDC_SETLINECODING:
      /********************/
      if ((setup->wValue == 0) &&
          (setup->wIndex == 0) &&               /* Interface no.            */
          (setup->wLength == 7) &&              /* Length of cdcLineCoding  */
          (setup->Direction != USB_SETUP_DIR_IN))
      {
        /* Get new settings from USB host. */
        USBD_Read(0, (void*) &cdcLineCoding, 7, LineCodingReceived);
        retVal = USB_STATUS_OK;
      }
      break;

    case USB_CDC_SETCTRLLINESTATE:
      /********************/
      if ((setup->wIndex == 0) &&               /* Interface no.  */
          (setup->wLength == 0))                /* No data        */
      {
        /* Do nothing ( Non compliant behaviour !! ) */
        retVal = USB_STATUS_OK;
      }
      break;
    }
  }

  return retVal;
}

/**************************************************************************//**
 * @brief
 *   Callback function called when the data stage of a CDC_SET_LINECODING
 *   setup command has completed.
 *
 * @param[in] status    Transfer status code.
 * @param[in] xferred   Number of bytes transferred.
 * @param[in] remaining Number of bytes not transferred.
 *
 * @return USB_STATUS_OK if data accepted.
 *         USB_STATUS_REQ_ERR if data calls for modes we can not support.
 *****************************************************************************/
static int LineCodingReceived(USB_Status_TypeDef status,
                              uint32_t xferred,
                              uint32_t remaining)
{
  uint32_t frame = 0;
  (void) remaining;

  /* We have received new serial port communication settings from USB host */
  if ((status == USB_STATUS_OK) && (xferred == 7))
  {
    /* Check bDataBits, valid values are: 5, 6, 7, 8 or 16 bits */
    if (cdcLineCoding.bDataBits == 5)
      frame |= UART_FRAME_DATABITS_FIVE;

    else if (cdcLineCoding.bDataBits == 6)
      frame |= UART_FRAME_DATABITS_SIX;

    else if (cdcLineCoding.bDataBits == 7)
      frame |= UART_FRAME_DATABITS_SEVEN;

    else if (cdcLineCoding.bDataBits == 8)
      frame |= UART_FRAME_DATABITS_EIGHT;

    else if (cdcLineCoding.bDataBits == 16)
      frame |= UART_FRAME_DATABITS_SIXTEEN;

    else
      return USB_STATUS_REQ_ERR;

    /* Check bParityType, valid values are: 0=None 1=Odd 2=Even 3=Mark 4=Space  */
    if (cdcLineCoding.bParityType == 0)
      frame |= UART_FRAME_PARITY_NONE;

    else if (cdcLineCoding.bParityType == 1)
      frame |= UART_FRAME_PARITY_ODD;

    else if (cdcLineCoding.bParityType == 2)
      frame |= UART_FRAME_PARITY_EVEN;

    else if (cdcLineCoding.bParityType == 3)
      return USB_STATUS_REQ_ERR;

    else if (cdcLineCoding.bParityType == 4)
      return USB_STATUS_REQ_ERR;

    else
      return USB_STATUS_REQ_ERR;

    /* Check bCharFormat, valid values are: 0=1 1=1.5 2=2 stop bits */
    if (cdcLineCoding.bCharFormat == 0)
      frame |= UART_FRAME_STOPBITS_ONE;

    else if (cdcLineCoding.bCharFormat == 1)
      frame |= UART_FRAME_STOPBITS_ONEANDAHALF;

    else if (cdcLineCoding.bCharFormat == 2)
      frame |= UART_FRAME_STOPBITS_TWO;

    else
      return USB_STATUS_REQ_ERR;

    /* Program new UART baudrate etc. */
    UART1->FRAME = frame;
   // USART_BaudrateAsyncSet(UART1, 0, cdcLineCoding.dwDTERate, usartOVS16);

    return USB_STATUS_OK;
  }
  return USB_STATUS_REQ_ERR;
}

static void UartRxTimeout(void)
{
  
}
