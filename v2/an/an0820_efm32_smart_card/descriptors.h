/***************************************************************************//**
 * @file descriptors.h
 * @brief USB descriptors for usb ccid example project.
 * @author Silicon Labs
 * @version 1.01
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

#ifdef __cplusplus
extern "C" {
#endif

#ifndef USB_SMARTCARD_DESCRIPTOR
  #define USB_SMARTCARD_DESCRIPTOR          0x21  /**< Smartcard usb-ccid-specific Descriptor Type.      */
#endif  
#ifndef USB_SMARTCARD_DESCSIZE
  #define USB_SMARTCARD_DESCSIZE            54    /**< CCID descriptor size.                             */
#endif

/* Macros for taking 16 bit word or 32 bit word and expanding to 4 bytes in uint8_t structs. */
#define HALFWORDTOBYTES(x) ((x) & 0xFF),(((x) >> 8) & 0xFF)
#define WORDTOBYTES(x) ((x) & 0xFF), (((x) >> 8) & 0xFF), (((x) >> 16) & 0xFF), (((x) >> 24) & 0xFF)
  
EFM32_ALIGN(4)
static const USB_DeviceDescriptor_TypeDef deviceDesc __attribute__ ((aligned(4)))=
{
  .bLength            = USB_DEVICE_DESCSIZE,
  .bDescriptorType    = USB_DEVICE_DESCRIPTOR,
  .bcdUSB             = 0x0110,
  .bDeviceClass       = 0,
  .bDeviceSubClass    = 0,
  .bDeviceProtocol    = 0,
  .bMaxPacketSize0    = USB_EP0_SIZE,
  .idVendor           = 0x10C4,
  .idProduct          = 0x000A,
  .bcdDevice          = 0x0100,
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
  USB_SMARTCARD_DESCSIZE +  
  (USB_ENDPOINT_DESCSIZE * NUM_EP_USED),

  (USB_CONFIG_DESCSIZE +  /* wTotalLength (MSB)                        */
  USB_INTERFACE_DESCSIZE +
  USB_SMARTCARD_DESCSIZE +   
  (USB_ENDPOINT_DESCSIZE * NUM_EP_USED))>>8,

  1,                      /* bNumInterfaces                            */
  1,                      /* bConfigurationValue                       */
  0,                      /* iConfiguration                            */
  CONFIG_DESC_BM_RESERVED_D7 |   /* bmAttrib: Self powered             */
  CONFIG_DESC_BM_SELFPOWERED,
  CONFIG_DESC_MAXPOWER_mA( 100 ),/* bMaxPower: 100 mA                  */

  /*** Interface descriptor ***/
  USB_INTERFACE_DESCSIZE, /* bLength               */
  USB_INTERFACE_DESCRIPTOR,/* bDescriptorType      */
  0,                      /* bInterfaceNumber      */
  0,                      /* bAlternateSetting     */
  NUM_EP_USED,            /* bNumEndpoints         */
  0x0B,                   /* bInterfaceClass       smart card device class*/
  0,                      /* bInterfaceSubClass    */
  0,                      /* bInterfaceProtocol    */
  0,                      /* iInterface            */
  
  /*** Interface descriptor ***/
  USB_SMARTCARD_DESCSIZE,
  USB_SMARTCARD_DESCRIPTOR,
  HALFWORDTOBYTES(0x0100),      /* bcdCCID */
  0,                            /* maxSlotIndex */
  2,                            /* VoltageSupport */
  WORDTOBYTES(1),               /* dwProtocols */
  WORDTOBYTES(3580),               /* dwDefaultClock */
  WORDTOBYTES(3580),               /* dwMaxClock */
  1,                            /* bNumClockSupported */
  WORDTOBYTES(9600),               /* dwDataRate */
  WORDTOBYTES(9600),               /* dwMaxDataRate */
  1,                            /* bNumDataRatesSupported */
  WORDTOBYTES(0xfe),               /* dwMaxIFSD */
  WORDTOBYTES(0),               /* dwSynchProtocols */
  WORDTOBYTES(0),               /* dwMechanical */
  WORDTOBYTES(0x00010000),         /* dwFeatures */
  WORDTOBYTES(271),           /* dwMaxCCIDMessageLength, set to minimum */
  0,                            /* bClassGetResponse */
  0,                            /* bClassEnvelope */
  HALFWORDTOBYTES(0),           /* wLcdLayout */
  0,                            /* bPINSupport */
  1,                            /* bMaxCCIDBusySlots */

  /*** Endpoint descriptor, BULK OUT ***/
  7,  /* bLength               */
  5,/* bDescriptorType       */
  0x01,        /* bEndpointAddress (IN) */
  2,        /* bmAttributes          */
  64,        /* wMaxPacketSize (LSB)  */
  0,                      /* wMaxPacketSize (MSB)  */
  0,   /* bInterval             */
  
  /*** Endpoint descriptor, BULK IN ***/
  7,  /* bLength               */
  5,/* bDescriptorType       */
  0x82,        /* bEndpointAddress (IN) */
  2,        /* bmAttributes          */
  64,        /* wMaxPacketSize (LSB)  */
  0,                      /* wMaxPacketSize (MSB)  */
  0,   /* bInterval             */
  
  /*** Endpoint descriptor, INTERRUPT ***/
  7,  /* bLength               */
  5,/* bDescriptorType       */
  0x83,        /* bEndpointAddress (IN) */
  3,        /* bmAttributes          */
  64,        /* wMaxPacketSize (LSB)  */
  0,                      /* wMaxPacketSize (MSB)  */
  255,   /* bInterval             */  
  
};


/* Define the String Descriptor for the device. String must be properly
 * aligned and unicode encoded. The first element defines the language id. 
 * Here 0x04 = United States, 0x09 = English. 
 * Refer to the USB Language Identifiers documentation. */
STATIC_CONST_STRING_DESC_LANGID( langID, 0x04, 0x09 );
STATIC_CONST_STRING_DESC( iManufacturer, 'S','i','l','i','c','o','n',' ',
                                         'L', 'a', 'b', 's' );
STATIC_CONST_STRING_DESC( iProduct     , 'E','F','M','3','2',' ',
                                         'E','x','a','m','p','l','e',' ',
                                         'U','S','B',' ', 
                                         'D','e','v','i','c','e');
STATIC_CONST_STRING_DESC( iSerialNumber, '0','0','0','0','0','0',             
                                         '0','0','1','2','3','4' );

static const void * const strings[] =
{
  &langID,
  &iManufacturer,
  &iProduct,
  &iSerialNumber
};

/* Endpoint buffer sizes */
/* 1 = single buffer, 2 = double buffering, 3 = tripple buffering ... */
static const uint8_t bufferingMultiplier[ NUM_EP_USED + 1 ] = { 1, 1, 1, 1 };

static const USBD_Callbacks_TypeDef callbacks =
{
  .usbReset        = NULL,
  .usbStateChange  = usbStateChange,
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

#ifdef __cplusplus
}
#endif
