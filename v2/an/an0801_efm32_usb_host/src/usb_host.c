/***************************************************************************//**
 * @file usb_host.c
 * @brief EFM32 USB Host Example
 * @author Silicon Labs
 * @version 1.02
 *******************************************************************************
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
#include "em_device.h"
#include "em_usb.h"
#include "em_emu.h"
#include "rtc.h"
#include "usb_host.h"

#ifdef STK
#include "segmentlcd.h"
#endif


/* Read and write buffers */
EFM32_ALIGN(4)
char writeBuffer[] = "tick";

EFM32_ALIGN(4)
uint8_t readBuffer[READ_BUFFER_SIZE];

/* This buffer is used to read out device information (descriptors) */
STATIC_UBUF( tmpBuf, 1024 );

/* USB Configuration Structs */
static USBH_Ep_TypeDef endpoints[NUM_HC_USED];
static USBH_Device_TypeDef device;
static USB_DeviceDescriptor_TypeDef *deviceDescriptor;
static USB_ConfigurationDescriptor_TypeDef *configurationDescriptor;
static USB_InterfaceDescriptor_TypeDef *interfaceDescriptor;
static USBH_Ep_TypeDef *outEndpoint;
static USBH_Ep_TypeDef *inEndpoint;

/* Set whenever data has been read. Cleared when the message
 * has been processed by the application. */
volatile bool dataRead = false;

/* Set whenever data has been sent. Cleared before
 * attempting to send a message. This is used to 
 * prevent sending overflows (trying to send data before
 * previous message has been sent). */
volatile bool dataSent = true;

/* Counts the number of messages sent on the OUT pipe */
volatile int sentMessages = 0;

/* Counts the number of messages received on the IN pipe */
volatile int receivedMessages = 0;

/* Set in RTC interrupt. Cleared when acknowledged by application. */
extern volatile bool rtcTick;
   
/* This flags is set on a USB error. */
volatile bool usbTransferError = false;



/**********************************************************
 * Callback for the 'Data Sent' event. This function 
 * is also called when a transfer failed for some reason. 
 * The @param status parameter can be used to identify
 * the cause of the error. 
 **********************************************************/
int usbDataSentCallback(USB_Status_TypeDef status, uint32_t transferred, uint32_t remaining)
{
	/* Avoid unused parameter warnings */
	(void)transferred;
	(void)remaining;
	
  if ( status != USB_STATUS_OK )
  {
    /* Inform application that an error has occured */
    usbTransferError = true;
    
    printf("\nSend error: %s", USB_GetErrorMsgString(status));
    
    return status;
  }
  else
  {
    /* Notify the application that the message was sent successfully */
    sentMessages++;
    dataSent = true;
    return USB_STATUS_OK;
  }
}

/**********************************************************
 * Callback for the 'Data Received' event. This function 
 * is also called when a transfer failed for some reason. 
 * The @param status parameter can be used to identify
 * the cause of the error. 
 **********************************************************/
int usbDataReceivedCallback(USB_Status_TypeDef status, uint32_t transferred, uint32_t remaining)
{
	/* Avoid unused parameter warnings */
	(void)transferred;
	(void)remaining;

  if ( status != USB_STATUS_OK )
  {
    /* Inform application that an error has occured */
    usbTransferError = true;
    
    printf("\nRecv error: %s", USB_GetErrorMsgString(status));
    
    return status; 
  }
  else
  {
    /* Notify the application that new data has been read */
    dataRead = true;
    
    /* Schedule a new read */
    USBH_Read( inEndpoint, readBuffer, READ_BUFFER_SIZE, 0, usbDataReceivedCallback );
  }

  return status;
}

/**********************************************************
 * Enumerates and configures the attached device. 
 * When this function completes (successfully) the
 * device is ready to communicate.
 * 
 * @returns
 *    USB_STATUS_OK if configuration was successful. 
 *    The appropriate error code if not.
 *********************************************************/
int usbConfigureDevice(void)
{
  int status;
  int i;
  
  /* Populate device and endpoint data structures with data retrieved during enumeration. */
  status = USBH_InitDeviceData( &device, tmpBuf, endpoints, NUM_HC_USED, USBH_GetPortSpeed() );
  if ( status != USB_STATUS_OK )
  {
    printf("\nError intializing device data: %s", USB_GetErrorMsgString(status));
    return status;
  }
  
  /* Set the device address and configuration */
  status = USBH_SetAddressB( &device, DEVICE_ADDR );
  if ( status != USB_STATUS_OK )
  {
    printf("\nError setting device address: %s", USB_GetErrorMsgString(status));
    return status;
  }
  
  /* Set the configuration. */
  status = USBH_SetConfigurationB( &device, device.confDesc.bConfigurationValue );
  if ( status != USB_STATUS_OK )
  {
    printf("\nError setting device configuration: %s", USB_GetErrorMsgString(status));
    return status;
  }
  
  /* Get the endpoints we will use */
  for ( i=0; i<NUM_HC_USED; i++ ) 
  {
    switch (endpoints[i].epDesc.bEndpointAddress)
    {
    case EP_IN:
      inEndpoint = &endpoints[i];
      break;
    case EP_OUT:
      outEndpoint = &endpoints[i];
      break;
    }
  }
  
  /* Assign host channels to device endpoints. Note that host channels 0 and 1
   * are already used for the control endpoint (EP0). */
  status = USBH_AssignHostChannel( outEndpoint, 2 );
  if ( status != USB_STATUS_OK )
  {
    printf("\nError assigning host channel: %s", USB_GetErrorMsgString(status));
    return status;
  }
  
  status = USBH_AssignHostChannel( inEndpoint, 3 );
  if ( status != USB_STATUS_OK )
  {
    printf("\nError assigning host channel: %s", USB_GetErrorMsgString(status));
    return status;
  }
  
  /* Configuration set successfully */
  return USB_STATUS_OK;
}


/**********************************************************
 * Sends 10 messages with 1 second delay. Listens for
 * incoming messages. 
 *********************************************************/
bool usbMessageLoop(void)
{ 
  /* Start listening for incoming messages. Do not resubmit the request
   * if we are already listening. */
  if ( inEndpoint->state == H_EP_IDLE )
  {
    USBH_Read(inEndpoint, readBuffer, READ_BUFFER_SIZE, 0, usbDataReceivedCallback);
  }
  
  /* Start RTC. Generates a tick every second */
  rtcStartTick();
  
  /* Loop until 10 messages has been sent */
  sentMessages = 0;
  while ( sentMessages < 10 )
  {
    /* Abort USB connection on transfer errors */
    if ( usbTransferError ) 
    {
      return false;
    }
    
    /* Print message every RTC tick */
    if ( rtcTick )
    {
      /* Clear the RTC flag to be ready for the next tick */
      rtcTick = false;
      
      if ( !dataSent )
      {
        /* Previous message has not been sent yet. */
        printf("\nSend overflow");
      }
      else
      {
        /* Send the message */
        printf("\nSending tick");
        dataSent = false;
        USBH_Write(outEndpoint, writeBuffer, sizeof(writeBuffer), 0, usbDataSentCallback);
      }
    }
    
    /* If we received data print it out */
    if ( dataRead )
    {
      /* Clear the flag to acknowledge the data */
      dataRead = false;
      
      /* On STK, increment number on LCD for each message */
#ifdef STK
      SegmentLCD_Number(++receivedMessages);
#endif
        
      /* Print out the message over the debug UART */
      printf("\nData received: %s", readBuffer);
    }
    
    /* Enter EM1 while waiting for the next event (RTC tick or received message) */
    EMU_EnterEM1();
  }
  
  rtcStopTick();

  /* Completed successfully */
  return true;
}


/**********************************************************
 * Suspends the port and waits 5 seconds before
 * resuming the port again.  
 **********************************************************/
bool usbSuspend(void)
{
  int status; 
  int tickCount = 0;
    
#ifdef STK
  SegmentLCD_Write("Suspend");
#endif  
  printf("\nSuspending port");
  
  /* Suspend USB port */
  USBH_PortSuspend();
  
  /* Start RTC ticks. Generates interrupts every second. */
  rtcStartTick();
  
  /* Wait for 5 seconds */
  while ( tickCount < 5 ) 
  {
    if ( rtcTick )
    {
      rtcTick = false;
      tickCount++;
    }
    EMU_EnterEM1();
  } 
  
#ifdef STK
  SegmentLCD_Write("Connctd");
#endif
  printf("\nResuming port");
  
  /* Stop RTC and resume port */
  rtcStopTick();
  status = USBH_PortResume(); 
  
  if ( status != USB_STATUS_OK )
  {
    printf("\nError resuming port: %s", USB_GetErrorMsgString(status));
    return false;
  }
  
  /* Completed successfully */
  return true;
}

/**********************************************************
 * Performs an example USB session. This function will 
 * both send and receive messages over the configured 
 * IN/OUT pipes and show how to Suspend and Resume a port. 
 * 
 * @returns
 *    True if the entire USB session completed successfully, 
 *    false if any error occured. 
 **********************************************************/
bool usbDoSession(void)
{
  /* Enter the message loop. Send and receive messages for 10 seconds */
  if ( !usbMessageLoop() )
  {
    return false;
  }
  
  /* Suspend port for 5 seconds */
  if ( !usbSuspend() ) 
  {
    return false;
  }

  /* Enter the message loop. Send and receive messages for 10 seconds */
  if ( !usbMessageLoop() ) 
  {
    return false;
  }
  
  /* Completed successfully */
  return true;
}


/**********************************************************
 * Attempts to connect and enumerate an attached device. 
 * If this function returns true, the USB device is
 * ready to communicate.
 * 
 * @returns
 *    True if the connection and enumeration
 *    was successful
 **********************************************************/
bool usbConnect(void)
{
  int status; 
  
  USBH_Init_TypeDef is = USBH_INIT_DEFAULT;
  
  /* Reset error flag */
  usbTransferError = false;
  
  /* Start USB Host Stack */
  USBH_Init(&is);
  
  printf("\nWaiting for device");
  
  /* Try to connect to a device. Times out after 1 second */
  status = USBH_WaitForDeviceConnectionB( tmpBuf, 1 );
 
  if ( status != USB_STATUS_OK )
  {
    printf("\nConnection error: %s", USB_GetErrorMsgString(status));
  }
  else
  {
    /* Enumerate device, retrieve device and configuration descriptors from device */
    USBH_QueryDeviceB( tmpBuf, sizeof( tmpBuf ), USBH_GetPortSpeed() );
    
    /* Parse the descriptors */
    deviceDescriptor        = USBH_QGetDeviceDescriptor(tmpBuf);
    configurationDescriptor = USBH_QGetConfigurationDescriptor(tmpBuf, 0);
    interfaceDescriptor     = USBH_QGetInterfaceDescriptor(tmpBuf, 0, 0);
    
    /* Qualify the device. Make sure that the connected the device is
     * the one we expect. */
    if ( ( deviceDescriptor->idVendor               == 0x2544 ) &&
         ( deviceDescriptor->idProduct              == 0x0007 ) &&
         ( deviceDescriptor->bNumConfigurations     == 1      ) &&
         ( configurationDescriptor->bNumInterfaces  == 1      ) &&
         ( interfaceDescriptor->bInterfaceClass     == 0xFF   ) &&
         ( interfaceDescriptor->bNumEndpoints       == 2      ))
    {
      
      printf("\nDevice connected.");
      
      /* Try to configure device */
      status = usbConfigureDevice();
      if ( status == USB_STATUS_OK )
      {
        /* Successfully connected and enumerated device */
        return true;
      }
    }
    else
    {
      printf("\nUnknown device.");
    }
  }
  
  return false;
}

/**********************************************************
 * Shut down USB
 **********************************************************/
void usbShutDown(void)
{
  rtcStopTick();
  
  printf("\nShutting down USB");
  
  /* Disable USB peripheral, power down USB port. */
  USBH_Stop();
}
