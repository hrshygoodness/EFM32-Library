/***************************************************************************//**
 * @file descriptors.h
 * @brief USB descriptors for HID keyboard example project.
 * @version 3.20.5
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/


/* Created with usb.org's "HID Descriptor Tool(DT)", version 2.4 */

#ifdef __cplusplus
extern "C" {
#endif

EFM32_ALIGN(4)
static const char ReportDescriptor[69] __attribute__ ((aligned(4)))=
{
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x01,                    //   INPUT (Cnst,Ary,Abs)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x06,                    //   REPORT_COUNT (6)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0x05, 0x08,                    //   USAGE_PAGE (LEDs)
    0x19, 0x01,                    //   USAGE_MINIMUM (Num Lock)
    0x29, 0x03,                    //   USAGE_MAXIMUM (Scroll Lock)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x03,                    //   REPORT_COUNT (3)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x05,                    //   REPORT_COUNT (5)
    0x91, 0x01,                    //   OUTPUT (Cnst,Ary,Abs)
    0xc0                           // END_COLLECTION
};


EFM32_ALIGN(4)
static const USB_DeviceDescriptor_TypeDef deviceDesc __attribute__ ((aligned(4)))=
{
  .bLength            = USB_DEVICE_DESCSIZE,
  .bDescriptorType    = USB_DEVICE_DESCRIPTOR,
  .bcdUSB             = 0x0200,
  .bDeviceClass       = 0,
  .bDeviceSubClass    = 0,
  .bDeviceProtocol    = 0,
  .bMaxPacketSize0    = USB_EP0_SIZE,
  .idVendor           = 0x10C4,
  .idProduct          = 0x0002,
  .bcdDevice          = 0x0000,
  .iManufacturer      = 1,
  .iProduct           = 2,
  .iSerialNumber      = 3,
  .bNumConfigurations = 1
};

EFM32_ALIGN(4)
static const uint8_t configDesc[] __attribute__ ((aligned(4)))=
{
  /*** Configuration descriptor ***/
  USB_CONFIG_DESCSIZE,    /* bLength                                   */
  USB_CONFIG_DESCRIPTOR,  /* bDescriptorType                           */

  USB_CONFIG_DESCSIZE +   /* wTotalLength (LSB)                        */
  USB_INTERFACE_DESCSIZE +
  USB_HID_DESCSIZE +
  (USB_ENDPOINT_DESCSIZE * NUM_EP_USED),

  (USB_CONFIG_DESCSIZE +  /* wTotalLength (MSB)                        */
  USB_INTERFACE_DESCSIZE +
  USB_HID_DESCSIZE +
  (USB_ENDPOINT_DESCSIZE * NUM_EP_USED))>>8,

  1,                      /* bNumInterfaces                            */
  1,                      /* bConfigurationValue                       */
  0,                      /* iConfiguration                            */

#if defined(BUSPOWERED)
  CONFIG_DESC_BM_RESERVED_D7,    /* bmAttrib: Bus powered              */
#else
  CONFIG_DESC_BM_RESERVED_D7 |   /* bmAttrib: Self powered             */
  CONFIG_DESC_BM_SELFPOWERED,
#endif

  CONFIG_DESC_MAXPOWER_mA( 100 ), /* bMaxPower: 100 mA                 */

  /*** Interface descriptor ***/
  USB_INTERFACE_DESCSIZE, /* bLength               */
  USB_INTERFACE_DESCRIPTOR,/* bDescriptorType      */
  0,                      /* bInterfaceNumber      */
  0,                      /* bAlternateSetting     */
  NUM_EP_USED,            /* bNumEndpoints         */
  0x03,                   /* bInterfaceClass (HID) */
  0,                      /* bInterfaceSubClass    */
  1,                      /* bInterfaceProtocol    */
  0,                      /* iInterface            */

  /*** HID descriptor ***/
  USB_HID_DESCSIZE,       /* bLength               */
  USB_HID_DESCRIPTOR,     /* bDescriptorType       */
  0x11,                   /* bcdHID (LSB)          */
  0x01,                   /* bcdHID (MSB)          */
  0,                      /* bCountryCode          */
  1,                      /* bNumDescriptors       */
  USB_HID_REPORT_DESCRIPTOR,     /* bDecriptorType        */
  sizeof( ReportDescriptor ),    /* wDescriptorLength(LSB)*/
  sizeof( ReportDescriptor )>>8, /* wDescriptorLength(MSB)*/

  /*** Endpoint descriptor ***/
  USB_ENDPOINT_DESCSIZE,  /* bLength               */
  USB_ENDPOINT_DESCRIPTOR,/* bDescriptorType       */
  INTR_IN_EP_ADDR,        /* bEndpointAddress (IN) */
  USB_EPTYPE_INTR,        /* bmAttributes          */
  USB_MAX_EP_SIZE,        /* wMaxPacketSize (LSB)  */
  0,                      /* wMaxPacketSize (MSB)  */
  POLL_RATE,              /* bInterval             */
};

STATIC_CONST_STRING_DESC_LANGID( langID, 0x04, 0x09 );
STATIC_CONST_STRING_DESC( iManufacturer, 'S','i','l','i','c','o','n',' ','L', \
                                         'a','b','o','r','a','t','o','r','i', \
                                         'e','s',' ','I','n','c','.' );
STATIC_CONST_STRING_DESC( iProduct     , 'E','F','M','3','2',' ','U','S','B', \
                                         ' ','H','I','D',' ','K','e','y','b', \
                                         'o','a','r','d' );
STATIC_CONST_STRING_DESC( iSerialNumber, '0','0','0','0','0','0',             \
                                         '0','0','1','2','3','4' );

static const void * const strings[] =
{
  &langID,
  &iManufacturer,
  &iProduct,
  &iSerialNumber
};

/* Endpoint buffer sizes */
/* 1 = single buffer, 2 = double buffering, 3 = triple buffering ... */
static const uint8_t bufferingMultiplier[ NUM_EP_USED + 1 ] = { 1, 1 };

static const USBD_Callbacks_TypeDef callbacks =
{
  .usbReset        = NULL,
  .usbStateChange  = StateChange,
  .setupCmd        = SetupCmd,
  .isSelfPowered   = NULL,
  .sofInt          = NULL
};

static const USBD_Init_TypeDef initstruct =
{
  .deviceDescriptor    = &deviceDesc,
  .configDescriptor    = configDesc,
  .stringDescriptors   = strings,
  .numberOfStrings     = sizeof(strings)/sizeof(void*),
  .callbacks           = &callbacks,
  .bufferingMultiplier = bufferingMultiplier,
  .reserved            = 0
};

typedef uint8_t KeyReport_TypeDef[ 8 ];

static const KeyReport_TypeDef noKeyReport =
{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* A sequence of keystroke input reports. */
EFM32_ALIGN(4)
static const KeyReport_TypeDef reportTable[] __attribute__ ((aligned(4)))=
{
  { 0x02, 0x00, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00 },  /* 'S'   */
  { 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00 },  /* 'i'   */
  { 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00 },  /* 'l'   */
  { 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00 },  /* 'i'   */
  { 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00 },  /* 'c'   */
  { 0x00, 0x00, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00 },  /* 'o'   */
  { 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00 },  /* 'n'   */
  { 0x00, 0x00, 0x2C, 0x00, 0x00, 0x00, 0x00, 0x00 },  /* space */
  { 0x02, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00 },  /* 'L'   */
  { 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00 },  /* 'a'   */
  { 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00 },  /* 'b'   */
  { 0x00, 0x00, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00 },  /* 's'   */
  { 0x00, 0x00, 0x2C, 0x00, 0x00, 0x00, 0x00, 0x00 },  /* space */
  { 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00 },  /* '-'   */
  { 0x00, 0x00, 0x2C, 0x00, 0x00, 0x00, 0x00, 0x00 },  /* space */
};

#ifdef __cplusplus
}
#endif
