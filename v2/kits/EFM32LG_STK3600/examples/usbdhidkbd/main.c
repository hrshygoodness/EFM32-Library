/**************************************************************************//**
 * @file main.c
 * @brief USB HID keyboard device example.
 * @version 3.20.5
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

#include "em_device.h"
#include "em_usb.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "segmentlcd.h"
#include "bsp_trace.h"

/**************************************************************************//**
 *
 * This example shows how a HID keyboard can be implemented.
 *
 *****************************************************************************/

/*** Typedef's and defines. ***/

#define SCAN_TIMER              1       /* Timer used to scan keyboard. */
#define SCAN_RATE               50

#define IDLE_TIMER              0       /* Timer used to implement the idle-  */
#define DEFAULT_IDLE_RATE       500     /* rate defined in the HID class doc. */

#define POLL_RATE               24      /* The bInterval reported with the    */
                                        /* interrupt IN endpoint descriptor.  */

#define HEARTBEAT_MASK          0xF
#define KEYLED_MASK             0x8000
#define KBDLED_MASK             0xF00
#define INTR_IN_EP_ADDR         0x81
#define ACTIVITY_LED            gpioPortE,2 /* LED0. */
#define BUTTON                  gpioPortB,9


/*** Function prototypes. ***/

static int  OutputReportReceived(USB_Status_TypeDef status,
                                 uint32_t xferred,
                                 uint32_t remaining);
static int  SetupCmd(const USB_Setup_TypeDef *setup);
static void StateChange(USBD_State_TypeDef oldState,
                        USBD_State_TypeDef newState);

static bool QueueEmpty( void );
static bool QueueFull( void );
static bool QueueGet( int *element );
static void QueueInit( void );
static bool QueuePut( int element );

/*** Include device descriptor definitions. ***/
#include "descriptors.h"

/*** Variables ***/

static int      keySeqNo;           /* Current position in report table. */
static bool     keyPushed;          /* Current pushbutton status. */
static int      lastReportIndex;    /* Last report sent to host. */
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

  for (;;)
  {
  }
}

/**************************************************************************//**
 * @brief
 *   Timeout function for the idleRate timer. The idleRate is set by the host
 *   device driver with the SET_IDLE setup command.
 *   If idleRate is set to 0 the idleRate timer is completely stopped.
 *   This function will always send a keyboard report to host.
 *****************************************************************************/
static void IdleTimeout(void)
{
  int reportIndex;

  /* If there is a keyboard event in the queue, we send it to the host. */
  /* If not we just resend the last one sent. */
  if ( !QueueEmpty() )
  {
    /* A new keyboard event. */
    QueueGet( &reportIndex );
    lastReportIndex = reportIndex;
  }
  else
  {
    /* The previous keyboard event. */
    reportIndex = lastReportIndex;
  }

  if ( reportIndex == -1 )
  {
    /* Send an empty (key released) report */
    USBD_Write( INTR_IN_EP_ADDR, (void*) &noKeyReport,
                sizeof(KeyReport_TypeDef), NULL);
  }
  else
  {
    /* Send a key pushed report */
    USBD_Write( INTR_IN_EP_ADDR, (void*) &reportTable[ reportIndex ],
                sizeof(KeyReport_TypeDef), NULL);
  }

  /* Schedule next idle event at current idle rate, idleRate unit is 4 ms. */
  USBTIMER_Start( IDLE_TIMER, idleRate * 4,  IdleTimeout );
}

/**************************************************************************//**
 * @brief
 *   Timeout function for keyboard scan timer.
 *   Scan keyboard to check for key press/release events.
 *   This function is called at a fixed rate.
 *****************************************************************************/
static void ScanTimeout(void)
{
  bool pushed;

  /* Check pushbutton */
  pushed = GPIO_PinInGet( BUTTON ) == 0;

  if (!keyPushed)
    GPIO_PinOutToggle(ACTIVITY_LED);

  if (pushed != keyPushed)  /* Any change in keyboard status ? */
  {
    if ( idleRate != 0 )    /* Put keyboard events into a queue. */
    {
      /* Put the kbd event in the queue, it will be retrieved and reported */
      /* to host in the idleRate timeout function.                         */
      if (pushed)
      {
        QueuePut( keySeqNo );
      }
      else
      {
        QueuePut( -1 ); /* Use -1 as a key release marker. */
      }
    }
    else /* idleRate == 0, send report immediately. */
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

  /* Restart keyboard scan timer */
  USBTIMER_Start( SCAN_TIMER, SCAN_RATE, ScanTimeout);
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
      keySeqNo        = 0;
      keyPushed       = false;
      lastReportIndex = -1;
      idleRate        = DEFAULT_IDLE_RATE / 4;     /* Unit is 4 millisecond. */
      GPIO_PinOutSet(ACTIVITY_LED);
      QueueInit();
    }
    USBTIMER_Start( SCAN_TIMER, SCAN_RATE, ScanTimeout);
    if ( idleRate )
    {
      USBTIMER_Start( IDLE_TIMER, idleRate * 4,  IdleTimeout);
    }
  }

  else if ((oldState == USBD_STATE_CONFIGURED) &&
           (newState != USBD_STATE_SUSPENDED))
  {
    /* We have been de-configured, stop HID functionality */
    USBTIMER_Stop(SCAN_TIMER);
    USBTIMER_Stop(IDLE_TIMER);
    GPIO_PinOutClear(ACTIVITY_LED);
  }

  else if (newState == USBD_STATE_SUSPENDED)
  {
    /* We have been suspended, stop HID functionality */
    /* Reduce current consumption to below 2.5 mA.    */
    GPIO_PinOutClear(ACTIVITY_LED);
    USBTIMER_Stop(SCAN_TIMER);
    USBTIMER_Stop(IDLE_TIMER);
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
        idleRate = setup->wValue >> 8;
        if ( ( idleRate != 0 ) && ( idleRate < ( POLL_RATE / 4 ) ) )
        {
          idleRate = POLL_RATE / 4;
        }
        USBTIMER_Stop( IDLE_TIMER );
        if ( idleRate != 0 )
        {
          IdleTimeout();
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

/* Minimal circular buffer implementation. */

#define QUEUE_SIZE 16             /* Must be 2^n !! */

typedef struct ringBuffer_t
{
  unsigned int putIdx;
  unsigned int getIdx;
  int          buf[ QUEUE_SIZE ];
} ringBuffer_t;

ringBuffer_t queue;

static bool QueueEmpty( void )
{
  return ( queue.putIdx - queue.getIdx ) == 0;
}

static bool QueueFull( void )
{
  return ( queue.putIdx - queue.getIdx ) >= QUEUE_SIZE;
}

static bool QueueGet( int *element )
{
  if ( !QueueEmpty() )
  {
    *element = queue.buf[ queue.getIdx++ & (QUEUE_SIZE - 1) ];
    return true;
  }
  return false;
}

static void QueueInit( void )
{
  queue.getIdx = 0;
  queue.putIdx = 0;
}

static bool QueuePut( int element )
{
  if ( !QueueFull() )
  {
    queue.buf[ queue.putIdx++ & (QUEUE_SIZE - 1) ] = element;
    return true;
  }
  return false;
}
