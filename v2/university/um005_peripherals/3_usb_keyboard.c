/**************************************************************************//**
 * @file 
 * @brief USB HID keyboard device example.
 * @author Energy Micro AS
 * @version 1.0.0
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silicon Labs Software License Agreement. See 
 * "http://developer.silabs.com/legal/version/v11/Silicon_Labs_Software_License_Agreement.txt"  
 * for details. Before using this software for any purpose, you must agree to the 
 * terms of that agreement.
 *
 ******************************************************************************/
#include "efm32.h"
#include "em_usb.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "segmentlcd.h"
#include "bsp.h"
#include "bsp_trace.h"
/**************************************************************************//**
 *
 * This example shows how a HID keyboard can be implemented.
 *
 *****************************************************************************/

/*** Typedef's and defines. ***/

#define POLL_TIMER              0
#define DEFAULT_POLL_TIMEOUT    24
#define HEARTBEAT_MASK          0xF
#define KEYLED_MASK             0x8000
#define KBDLED_MASK             0xF00
#define INTR_IN_EP_ADDR         0x81
#define ACTIVITY_LED            gpioPortE,2 /* The blue led labeled STATUS. */
#define BUTTON                  gpioPortB,9


/*** Function prototypes. ***/

static int  OutputReportReceived(USB_Status_TypeDef status,
                                 uint32_t xferred,
                                 uint32_t remaining);
static int  SetupCmd(const USB_Setup_TypeDef *setup);
static void StateChange(USBD_State_TypeDef oldState,
                        USBD_State_TypeDef newState);


/*** Include device descriptor definitions. ***/

#include "descriptors.h"

/*** Variables ***/

static int      keySeqNo;           /* Current position in report table */
static bool     keyPushed;          /* Current pushbutton status  */
static int      pollTimeout;        /* Key poll rate, unit is ms. */
static uint8_t  idleRate;
static uint32_t tmpBuffer;


/**************************************************************************//**
 * @brief main - the entrypoint after reset.
 *****************************************************************************/
int main(void)
{
  /* Chip errata */
  CHIP_Init();

  /* If first word of user data page is non-zero, enable eA Profiler trace */
  BSP_TraceProfilerSetup();
  
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(BUTTON, gpioModeInputPull, 1);
  GPIO_PinModeSet(ACTIVITY_LED, gpioModePushPull,  0);
  CMU_ClockSelectSet( cmuClock_HF, cmuSelect_HFXO );
  SegmentLCD_Init(false);
  SegmentLCD_Write("usb kbd");
  SegmentLCD_Symbol(LCD_SYMBOL_GECKO, true);

  USBD_Init(&initstruct);

  /*
   * When using a debugger it is practical to uncomment the following three
   * lines to force host to re-enumerate the device.
   */
  /* USBD_Disconnect();      */
  /* USBTIMER_DelayMs(1000); */
  /* USBD_Connect();         */

 
  while(1)
  {
  }
}

/**************************************************************************//**
 * @brief
 *   Called on timer elapsed event. This function is called at a rate set
 *   by the host driver with the SET_IDLE setup command.
 *****************************************************************************/
static void PollTimeout(void)
{
  bool pushed;

  /* Check pushbutton */
  pushed = GPIO_PinInGet( BUTTON ) == 0;

  if (!keyPushed)
    GPIO_PinOutToggle(ACTIVITY_LED);

  if (pollTimeout != 0)     /* Send report with current key state */
  {
    if (pushed)
    {
      /* Send a key pushed report */
      USBD_Write(INTR_IN_EP_ADDR, (void*) &reportTable[ keySeqNo ],
                 sizeof(KeyReport_TypeDef), NULL);
    }
    else
    {
      /* Send an empty (key released) report */
      USBD_Write(INTR_IN_EP_ADDR,
                 (void*) &noKeyReport, sizeof(KeyReport_TypeDef), NULL);
    }
  }
  else        /* pollTimeout == 0, only send report on key status change */
  {
    if (pushed != keyPushed)        /* Any change ? */
    {
      if (pushed)
      {
        /* Send a key pushed report */
        USBD_Write(INTR_IN_EP_ADDR, (void*) &reportTable[ keySeqNo ],
                   sizeof(KeyReport_TypeDef), NULL);
      }
      else
      {
        /* Send an empty (key released) report */
        USBD_Write(INTR_IN_EP_ADDR,
                   (void*) &noKeyReport, sizeof(KeyReport_TypeDef), NULL);
      }
    }
  }

  /* Keep track of the new keypush event (if any) */
  if (pushed && !keyPushed)
  {
    /* Advance to next position in report table */
    keySeqNo++;
    if (keySeqNo == (sizeof(reportTable) / sizeof(KeyReport_TypeDef)))
    {
      keySeqNo = 0;
    }
    GPIO_PinOutSet(ACTIVITY_LED);
  }
  keyPushed = pushed;

  /* Restart HID poll timer */
  if (pollTimeout)
  {
    USBTIMER_Start(POLL_TIMER, pollTimeout, PollTimeout);
  }
  else
  {
    USBTIMER_Start(POLL_TIMER, DEFAULT_POLL_TIMEOUT, PollTimeout);
  }
}

/**************************************************************************//**
 * @brief
 *   Callback function called each time the USB device state is changed.
 *   Starts HID operation when device has been configured by USB host.
 *
 * @param[in] oldState The device state the device has just left.
 * @param[in] newState The new device state.
 *****************************************************************************/
static void StateChange(USBD_State_TypeDef oldState,
                        USBD_State_TypeDef newState)
{
  if (newState == USBD_STATE_CONFIGURED)
  {
    /* We have been configured, start HID functionality ! */
    if (oldState != USBD_STATE_SUSPENDED)   /* Resume ?   */
    {
      keySeqNo    = 0;
      keyPushed   = false;
      idleRate    = DEFAULT_POLL_TIMEOUT / 4;
      pollTimeout = DEFAULT_POLL_TIMEOUT;
      GPIO_PinOutSet(ACTIVITY_LED);
    }
    USBTIMER_Start(POLL_TIMER, DEFAULT_POLL_TIMEOUT, PollTimeout);
  }

  else if ((oldState == USBD_STATE_CONFIGURED) &&
           (newState != USBD_STATE_SUSPENDED))
  {
    /* We have been de-configured, stop HID functionality */
    USBTIMER_Stop(POLL_TIMER);
    GPIO_PinOutClear(ACTIVITY_LED);
  }

  else if (newState == USBD_STATE_SUSPENDED)
  {
    /* We have been suspended, stop HID functionality */
    /* Reduce current consumption to below 2.5 mA.    */
    GPIO_PinOutClear(ACTIVITY_LED);
    USBTIMER_Stop(POLL_TIMER);
  }
}

/**************************************************************************//**
 * @brief
 *   Handle USB setup commands. Implements HID class specific commands.
 *
 * @param[in] setup Pointer to the setup packet received.
 *
 * @return USB_STATUS_OK if command accepted.
 *         USB_STATUS_REQ_UNHANDLED when command is unknown, the USB device
 *         stack will handle the request.
 *****************************************************************************/
static int SetupCmd(const USB_Setup_TypeDef *setup)
{
  STATIC_UBUF( hidDesc, USB_HID_DESCSIZE );

  int retVal = USB_STATUS_REQ_UNHANDLED;

  if ((setup->Type == USB_SETUP_TYPE_STANDARD) &&
      (setup->Direction == USB_SETUP_DIR_IN) &&
      (setup->Recipient == USB_SETUP_RECIPIENT_INTERFACE))
  {
    /* A HID device must extend the standard GET_DESCRIPTOR command   */
    /* with support for HID descriptors.                              */
    switch (setup->bRequest)
    {
    case GET_DESCRIPTOR:
      /********************/
      if ((setup->wValue >> 8) == USB_HID_REPORT_DESCRIPTOR)
      {
        USBD_Write(0, (void*) ReportDescriptor,
                   EFM32_MIN(sizeof(ReportDescriptor), setup->wLength),
                   NULL);
        retVal = USB_STATUS_OK;
      }
      else if ((setup->wValue >> 8) == USB_HID_DESCRIPTOR)
      {
        /* The HID descriptor might be misaligned ! */
        memcpy( hidDesc,
                &configDesc[ USB_CONFIG_DESCSIZE + USB_INTERFACE_DESCSIZE ],
                USB_HID_DESCSIZE );
        USBD_Write(0, hidDesc, EFM32_MIN(USB_HID_DESCSIZE, setup->wLength),
                   NULL);
        retVal = USB_STATUS_OK;
      }
      break;
    }
  }

  else if ((setup->Type == USB_SETUP_TYPE_CLASS) &&
           (setup->Recipient == USB_SETUP_RECIPIENT_INTERFACE))
  {
    /* Implement the necessary HID class specific commands.           */
    switch (setup->bRequest)
    {
    case USB_HID_SET_REPORT:
      /********************/
      if (((setup->wValue >> 8) == 2) &&            /* Output report */
          ((setup->wValue & 0xFF) == 0) &&          /* Report ID     */
          (setup->wIndex == 0) &&                   /* Interface no. */
          (setup->wLength == 1) &&                  /* Report length */
          (setup->Direction != USB_SETUP_DIR_IN))
      {
        USBD_Read(0, (void*) &tmpBuffer, 1, OutputReportReceived);
        retVal = USB_STATUS_OK;
      }
      break;

    case USB_HID_GET_REPORT:
      /********************/
      if (((setup->wValue >> 8) == 1) &&            /* Input report  */
          ((setup->wValue & 0xFF) == 0) &&          /* Report ID     */
          (setup->wIndex == 0) &&                   /* Interface no. */
          (setup->wLength == 8) &&                  /* Report length */
          (setup->Direction == USB_SETUP_DIR_IN))
      {
        if (keyPushed)
        {
          /* Send a key pushed report */
          USBD_Write(0, (void*) &reportTable[ keySeqNo ],
                     sizeof(KeyReport_TypeDef), NULL);
        }
        else
        {
          /* Send an empty (key released) report */
          USBD_Write(0, (void*) &noKeyReport,
                     sizeof(KeyReport_TypeDef), NULL);
        }
        retVal = USB_STATUS_OK;
      }
      break;

    case USB_HID_SET_IDLE:
      /********************/
      if (((setup->wValue & 0xFF) == 0) &&          /* Report ID     */
          (setup->wIndex == 0) &&                   /* Interface no. */
          (setup->wLength == 0) &&
          (setup->Direction != USB_SETUP_DIR_IN))
      {
        idleRate    = setup->wValue >> 8;
        pollTimeout = 4 * idleRate;
        if (pollTimeout > DEFAULT_POLL_TIMEOUT)
        {
          pollTimeout = DEFAULT_POLL_TIMEOUT;
        }
        retVal = USB_STATUS_OK;
      }
      break;

    case USB_HID_GET_IDLE:
      /********************/
      if ((setup->wValue == 0) &&                   /* Report ID     */
          (setup->wIndex == 0) &&                   /* Interface no. */
          (setup->wLength == 1) &&
          (setup->Direction == USB_SETUP_DIR_IN))
      {
        *(uint8_t*)&tmpBuffer = idleRate;
        USBD_Write(0, (void*) &tmpBuffer, 1, NULL);
        retVal = USB_STATUS_OK;
      }
      break;
    }
  }

  return retVal;
}

/**************************************************************************//**
 * @brief
 *   Callback function called when the data stage of a USB_HID_SET_REPORT
 *   setup command has completed.
 *
 * @param[in] status    Transfer status code.
 * @param[in] xferred   Number of bytes transferred.
 * @param[in] remaining Number of bytes not transferred.
 *
 * @return USB_STATUS_OK.
 *****************************************************************************/
static int OutputReportReceived(USB_Status_TypeDef status,
                                uint32_t xferred,
                                uint32_t remaining)
{
  (void) status;
  (void) xferred;
  (void) remaining;

  return USB_STATUS_OK;
}
