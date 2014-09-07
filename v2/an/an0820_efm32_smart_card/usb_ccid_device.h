/******************************************************************************
 * @file usb_ccid_device.h
 * @brief USB CCID device class driver
 * @author Silicon Labs
 * @version 1.01
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



#ifndef __USB_CCID_DEVICE_H
#define __USB_CCID_DEVICE_H

/* Slot Status register bitfield defines, these are or'ed together to form the slot status 8 bit word */
#define SLOT_STATUS_ICC_PRESENT_ACTIVE          (0x00)
#define SLOT_STATUS_ICC_PRESENT_INACTIVE        (0x01)
#define SLOT_STATUS_ICC_NOT_PRESENT             (0x02)
#define SLOT_STATUS_CMD_NO_ERROR                (0x00)
#define SLOT_STATUS_CMD_FAILED                  (0x40)
#define SLOT_STATUS_CMD_TIME_EXT_REQ            (0x80)

/* Slot Error register codes when bmCommandStatus = 1 */
#define SLOT_ERROR_CODE_CMD_ABORTED                     (0xFF)
#define SLOT_ERROR_CODE_ICC_MUTE                        (0xFE)
#define SLOT_ERROR_CODE_XFR_PARITY_ERROR                (0xFD)
#define SLOT_ERROR_CODE_XFR_OVERRUN                     (0xFC)
#define SLOT_ERROR_CODE_HW_ERROR                        (0xFB)
#define SLOT_ERROR_CODE_BAD_ATR_TS                      (0xF8)
#define SLOT_ERROR_CODE_BAD_ATR_TCK                     (0xF7)
#define SLOT_ERROR_CODE_ICC_PROTOCOL_NOT_SUPPORTED      (0xF6)
#define SLOT_ERROR_CODE_ICC_CLASS_NOT_SUPPORTED         (0xF5)
#define SLOT_ERROR_CODE_PROCEDURE_BYTE_CONFLICT         (0xF4)
#define SLOT_ERROR_CODE_DEACTIVATED_PROTOCOL            (0xF3)
#define SLOT_ERROR_CODE_BUSY_WITH_AUTO_SEQUENCE         (0xF2)
#define SLOT_ERROR_CODE_PIN_TIMEOUT                     (0xF0)
#define SLOT_ERROR_CODE_PIN_CANCELLED                   (0xEF)
#define SLOT_ERROR_CODE_CMD_SLOT_BUSY                   (0xE0)
#define SLOT_ERROR_CODE_CMD_NOT_SUPPORTED               (0x00)

/* Bulk out messages */
#define PC_to_RDR_IccPowerOn            (0x62)
#define PC_to_RDR_IccPowerOff           (0x63)
#define PC_to_RDR_GetSlotStatus         (0x65)
#define PC_to_RDR_XfrBlock              (0x6F)
#define PC_to_RDR_GetParameters         (0x6C)
#define PC_to_RDR_ResetParameters       (0x6D)
#define PC_to_RDR_SetParameters         (0x61)
#define PC_to_RDR_Escape                (0x6B)
#define PC_to_RDR_IccClock              (0x6E)
#define PC_to_RDR_T0APDU                (0x6A)
#define PC_to_RDR_Secure                (0x69)
#define PC_to_RDR_Mechanical            (0x71)
#define PC_to_RDR_Abort                 (0x72)
#define PC_to_RDR_SetDataRateAndClockFrequency (0x73)

/* Bulk in messages */
#define RDR_to_PC_DataBlock                     (0x80)
#define RDR_to_PC_SlotStatus                    (0x81)
#define RDR_to_PC_Parameters                    (0x82)
#define RDR_to_PC_Escape                        (0x83)
#define RDR_to_PC_DataRateAndClockFrequency     (0x84)

/* Interrupt Message Types*/
#define INT_MESSAGE_SLOTCHANGE      0x50

#define EP_BULK_IN      0x82   
#define EP_BULK_OUT     0x01
#define EP_INT_IN       0x83
   
//#define DEBUG_PRINTF

#ifdef DEBUG_PRINTF     
  #define USBCCID_DEBUG_PRINTF( s ) printf(s)
#else
  #define USBCCID_DEBUG_PRINTF( s )
#endif
   

/* Structs defined according to usb ccid specification document. */
EFM32_PACK_START(1)
typedef struct _RDR_to_PC_NotifySlotChange_INT
{
    uint8_t MessageType;
    uint8_t SlotICCState;
} RDR_to_PC_NotifySlotChange_INT;
EFM32_PACK_END()

EFM32_PACK_START(1)
typedef struct _RDR_to_PC_SlotStatus_BULK_IN
{
    uint8_t bMessageType;
    uint32_t dwLength;
    uint8_t bSlot;
    uint8_t bSeq;
    uint8_t bStatus;
    uint8_t bError;
    uint8_t bClockStatus;
} RDR_to_PC_SlotStatus_BULK_IN;
EFM32_PACK_END()

EFM32_PACK_START(1)
typedef struct _RDR_to_PC_DataBlock_BULK_IN
{
    uint8_t bMessageType;
    uint32_t dwLength;
    uint8_t bSlot;
    uint8_t bSeq;
    uint8_t bStatus;
    uint8_t bError;
    uint8_t bChainParameter;
    uint8_t abData[53];
} RDR_to_PC_DataBlock_BULK_IN;
EFM32_PACK_END()

EFM32_PACK_START(1)
typedef struct _PC_to_RDR_XfrBlock_BULK_OUT
{
    uint8_t bMessageType;
    uint32_t dwLength;
    uint8_t bSlot;
    uint8_t bSeq;
    uint8_t bBWI;
    uint16_t wLevelParameter;
    uint8_t abData[53];
} PC_to_RDR_XfrBlock_BULK_OUT;
EFM32_PACK_END()

typedef enum
{
  CCID_STATE_NO_CARD             = 0,                    
  CCID_STATE_CARD_INSERTED       = 1,                   
  CCID_STATE_CARD_PRESENT        = 2,                    
  CCID_STATE_CARD_REMOVED        = 3,                    
} CCID_State_TypeDef;

/***************************************************************************//**
 * @brief
 *   Initialize GPIO, TIMER and UART necessary for smart card communication
 *
 ******************************************************************************/
void CCID_Init(void);

/***************************************************************************//**
 * @brief
 *   Handle usb message from host according to USBCCID standard.
 *   Some error conditions handled, but not exhaustive list of errors handled.
 *
 ******************************************************************************/
void CCID_Handler(void);
 



#endif /* __USB_CCID_DEVICE_H */
