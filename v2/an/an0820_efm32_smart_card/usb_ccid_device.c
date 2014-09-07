/******************************************************************************
 * @file usb_ccid_device.c
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

#include <stdint.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_usb.h"
#include "usart_driver.h"
#include "sc_driver.h"
#include "usb_ccid_device.h"

/*** Function prototypes ***/
static int  SetupCmd(const USB_Setup_TypeDef *setup);
static void usbStateChange(USBD_State_TypeDef oldState, USBD_State_TypeDef newState);
static void handleOutMsgIccPowerOn(void);
static void handleOutMsgIccPowerOff(void);
static void handleOutMsgGetSlotStatus(void);
static void handleOutMsgXfrBlock(void);
static void handleOutMsgGetParameters(void);
static void handleOutMsgResetParameters(void);
static void handleOutMsgSetParameters(void);
static void handleOutMsgEscape(void);
static void handleOutMsgIccClock(void);
static void handleOutMsgT0APDU(void);
static void handleOutMsgSecure(void);
static void handleOutMsgMechanical(void);
static void handleOutMsgAbort(void);
static void handleOutMsgSetDataRateAndClockFrequency(void);

int  CCID_DataReceivedCallback(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining);
int  CCID_DataSentCallback(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining);

/* usb message structs */
RDR_to_PC_NotifySlotChange_INT IntMsg;
RDR_to_PC_SlotStatus_BULK_IN bulkIn_SlotStatusMsg;
RDR_to_PC_DataBlock_BULK_IN bulkIn_DataBlockMsg;

#define USB_BUFFERSIZE 500

STATIC_UBUF(receiveBuffer, USB_BUFFERSIZE);
PC_to_RDR_XfrBlock_BULK_OUT* bulkOut_XfrBlockMsg = (PC_to_RDR_XfrBlock_BULK_OUT*) receiveBuffer;

volatile uint8_t usbDeviceConfigured = 0;

volatile uint8_t unhandled_message = 0;

static CCID_State_TypeDef CARD_STATE = CCID_STATE_NO_CARD;


/*** Include device descriptor definitions. ***/

#include "descriptors.h"

/***************************************************************************//**
 * @brief
 *   Initialize GPIO, TIMER and UART necessary for smart card communication
 *
 ******************************************************************************/
void CCID_Init(void)
{
  USBD_Init(&initstruct);

  /*
   * When using a debugger it is practical to uncomment the following three
   * lines to force host to re-enumerate the device.
   */
  /* USBD_Disconnect();      */
  /* USBTIMER_DelayMs(1000); */
  /* USBD_Connect();         */
    
  usartSetup();
  SC_Init();
  
}

/***************************************************************************//**
 * @brief
 *   Handle usb message from host according to USBCCID standard.
 *   Some error conditions handled, but not exhaustive list of errors handled.
 *
 ******************************************************************************/
void handleHostMessage(void)
{
  uint8_t messageType = bulkOut_XfrBlockMsg->bMessageType;
    
  /* See USB CCID documentation for description of message types and appropriate responses. */
  switch(messageType)
  {
    case PC_to_RDR_IccPowerOn:
      handleOutMsgIccPowerOn();
   
    break;
    case PC_to_RDR_IccPowerOff:
      handleOutMsgIccPowerOff();
   
    break;
    case PC_to_RDR_GetSlotStatus:
      handleOutMsgGetSlotStatus();
            
    break;
    case PC_to_RDR_XfrBlock:
      handleOutMsgXfrBlock();
  
    break;
    case PC_to_RDR_GetParameters:
      handleOutMsgGetParameters();
            
    break;
    case PC_to_RDR_ResetParameters:
      handleOutMsgResetParameters();
      
    break;
    case PC_to_RDR_SetParameters:
      handleOutMsgSetParameters();
            
    break;
    case PC_to_RDR_Escape:
      handleOutMsgEscape();
      
    break;
    case PC_to_RDR_IccClock:
      handleOutMsgIccClock();
      
    break;
    case PC_to_RDR_T0APDU:
      handleOutMsgT0APDU();
      
    break;
    case PC_to_RDR_Secure:
      handleOutMsgSecure();
      
    break;
    case PC_to_RDR_Mechanical:
      handleOutMsgMechanical();
      
      
    break;
    case PC_to_RDR_Abort:
      handleOutMsgAbort();
     
    break;
    case PC_to_RDR_SetDataRateAndClockFrequency:
      handleOutMsgSetDataRateAndClockFrequency();
      
    break;
    }
    
  /* Prepare to accept the next message */
  USBD_Read(EP_BULK_OUT, receiveBuffer, USB_BUFFERSIZE, CCID_DataReceivedCallback);
  
}

/***************************************************************************//**
 * @brief
 *   Simple state-machine handling slot-change interrupt messages upon 
 *   Card insertion/removal.
 *   Also calls hostMessageHandler-function if there are any unhandled usb-messages.
 *
 ******************************************************************************/
void CCID_Handler(void)
{
  /* Check if card is present. */
  uint8_t cardIsInserted = SC_IsCardInserted();
    
  /* Only run statemachine if usb is configured */
  if(usbDeviceConfigured)
  {
    
    switch(CARD_STATE)
    {
      case CCID_STATE_NO_CARD:
        if(cardIsInserted)
        {
          CARD_STATE = CCID_STATE_CARD_INSERTED;
        
        }
      break;
      case CCID_STATE_CARD_INSERTED:
        if(cardIsInserted)
        {
          CARD_STATE = CCID_STATE_CARD_PRESENT;
          
          /* Send Int message with card inserted. */
          IntMsg.MessageType = INT_MESSAGE_SLOTCHANGE;
          IntMsg.SlotICCState = 0x03;
          USBD_Write(EP_INT_IN, &IntMsg, sizeof(IntMsg), CCID_DataSentCallback);    
          
          
        }else
        {
          CARD_STATE = CCID_STATE_CARD_REMOVED;
        }     
        
      break;
      case CCID_STATE_CARD_PRESENT:
        if(cardIsInserted){
          /* Handle card communication */
          if(unhandled_message)
          {
            handleHostMessage();
            unhandled_message = 0;
          }
       
        }else
        {
          CARD_STATE = CCID_STATE_CARD_REMOVED;
        }
        
        
      break;
      case CCID_STATE_CARD_REMOVED:
        if(!cardIsInserted)
        {
          CARD_STATE = CCID_STATE_NO_CARD;
		  /* Immediately power off card and notify host. */
		  SC_AssertReset();
		  SC_ClockOff();
		  SC_PowerOff();
		  
          /* Send Int message with card removed. */
          IntMsg.MessageType = INT_MESSAGE_SLOTCHANGE;
          IntMsg.SlotICCState = 0x02;
          
          USBD_Write(EP_INT_IN, &IntMsg, sizeof(IntMsg), CCID_DataSentCallback);    
          
          /* Handle card communication */
          if(unhandled_message)
          {
            handleHostMessage();
            unhandled_message = 0;
          }          
          
        }else
        {
          CARD_STATE = CCID_STATE_CARD_INSERTED;      
        }    
      break;
    
    }
  }
}

/**********************************************************
 * Performs power on and reset of card, get ATR and sends 
 * response message over usb.
 **********************************************************/
void handleOutMsgIccPowerOn(void)
{
  uint8_t status;
  uint32_t messageLength;

  USBCCID_DEBUG_PRINTF("PC_to_RDR_IccPowerOn\n");
  
  bulkIn_DataBlockMsg.bMessageType = RDR_to_PC_DataBlock;
  bulkIn_DataBlockMsg.dwLength = 0;
  bulkIn_DataBlockMsg.bSlot = bulkOut_XfrBlockMsg->bSlot;
  bulkIn_DataBlockMsg.bSeq = bulkOut_XfrBlockMsg->bSeq;
  
  /* Card state, is card present or not. */
  if(CARD_STATE == CCID_STATE_CARD_PRESENT)
  {
    bulkIn_DataBlockMsg.bStatus = SLOT_STATUS_ICC_PRESENT_ACTIVE;
    bulkIn_DataBlockMsg.bError = 0;
    
    /* reset card and fetch Answer to Reset. */
    /* Indicate smart card communication activity, toggle LED. */
    SC_LedOn();
    status = SC_Reset(bulkIn_DataBlockMsg.abData, &messageLength);
    SC_LedOff();
    
    bulkIn_DataBlockMsg.dwLength = messageLength;
    
    switch(status)
    {
    case APDU_STATUS_TIMEOUT:
        /* CMD timed out. */
        bulkIn_DataBlockMsg.dwLength = 0;
        bulkIn_DataBlockMsg.bStatus = SLOT_STATUS_CMD_FAILED; 
        bulkIn_DataBlockMsg.bError = SLOT_ERROR_CODE_ICC_MUTE;
    break; 
    case APDU_STATUS_XFR_ERROR:
        /* CMD timed out. */
        bulkIn_DataBlockMsg.dwLength = 0;
        bulkIn_DataBlockMsg.bStatus = SLOT_STATUS_CMD_FAILED; 
        bulkIn_DataBlockMsg.bError = SLOT_ERROR_CODE_XFR_PARITY_ERROR;
    break;         
    }
    
  }else
  {
    bulkIn_DataBlockMsg.dwLength = 0;
    bulkIn_DataBlockMsg.bStatus = SLOT_STATUS_ICC_NOT_PRESENT;
    bulkIn_DataBlockMsg.bError = SLOT_ERROR_CODE_ICC_MUTE;
  }

  
  bulkIn_DataBlockMsg.bChainParameter = 0x00;
  
  USBD_Write(EP_BULK_IN, &bulkIn_DataBlockMsg, 10 + bulkIn_DataBlockMsg.dwLength, CCID_DataSentCallback);      
}

/**********************************************************
 * Performs power off and sends 
 * response message over usb.
 **********************************************************/
void handleOutMsgIccPowerOff(void)
{
  USBCCID_DEBUG_PRINTF("PC_to_RDR_IccPowerOff\n");
      
  /* Block receiver before powering off to discard glitches etc. */
  usartBlockReceiver(true);

  SC_AssertReset();
  SC_ClockOff();
  SC_PowerOff();
  SC_DisableReset();

  bulkIn_SlotStatusMsg.bMessageType = RDR_to_PC_SlotStatus;
  bulkIn_SlotStatusMsg.dwLength = 0;
  bulkIn_SlotStatusMsg.bSlot = bulkOut_XfrBlockMsg->bSlot;
  bulkIn_SlotStatusMsg.bSeq = bulkOut_XfrBlockMsg->bSeq;

  /* Card state, is card present or not. */
  if(CARD_STATE == CCID_STATE_CARD_PRESENT)
  {
    bulkIn_SlotStatusMsg.bStatus = SLOT_STATUS_ICC_PRESENT_ACTIVE;
    bulkIn_SlotStatusMsg.bError = 0;
  }else
  {
    bulkIn_SlotStatusMsg.bStatus = SLOT_STATUS_ICC_NOT_PRESENT;
    bulkIn_SlotStatusMsg.bError = SLOT_ERROR_CODE_ICC_MUTE;
  }

  bulkIn_SlotStatusMsg.bClockStatus = 0x00;
        
  USBD_Write(EP_BULK_IN, &bulkIn_SlotStatusMsg, 10, CCID_DataSentCallback);  

}
  
/**********************************************************
 * Respons with slot status usb message.
 **********************************************************/
void handleOutMsgGetSlotStatus(void)
{
  USBCCID_DEBUG_PRINTF("PC_to_RDR_GetSlotStatus\n");

  bulkIn_SlotStatusMsg.bMessageType = RDR_to_PC_SlotStatus;
  bulkIn_SlotStatusMsg.dwLength = 0;
  bulkIn_SlotStatusMsg.bSlot = bulkOut_XfrBlockMsg->bSlot;
  bulkIn_SlotStatusMsg.bSeq = bulkOut_XfrBlockMsg->bSeq;

  /* Card state, is card present or not. */
  if(CARD_STATE == CCID_STATE_CARD_PRESENT)
  {
    bulkIn_SlotStatusMsg.bStatus = SLOT_STATUS_ICC_PRESENT_ACTIVE;
    bulkIn_SlotStatusMsg.bError = 0;
  }else
  {
    bulkIn_SlotStatusMsg.bStatus = SLOT_STATUS_ICC_NOT_PRESENT;
    bulkIn_SlotStatusMsg.bError = SLOT_ERROR_CODE_ICC_MUTE;
  }

  bulkIn_SlotStatusMsg.bClockStatus = 0x00;
        
  USBD_Write(EP_BULK_IN, &bulkIn_SlotStatusMsg, 10, CCID_DataSentCallback); 

}
      
/**********************************************************
 * Sends apdu to smart card and get status respons.
 * Sends apdu from card back to host over usb.
 **********************************************************/
void handleOutMsgXfrBlock(void)
{
  uint8_t status;
  uint32_t messageLength;
        
  USBCCID_DEBUG_PRINTF("PC_to_RDR_XfrBlock\n");

  bulkIn_DataBlockMsg.bMessageType = RDR_to_PC_DataBlock;
  bulkIn_DataBlockMsg.dwLength = 0;
  bulkIn_DataBlockMsg.bSlot = bulkOut_XfrBlockMsg->bSlot;
  bulkIn_DataBlockMsg.bSeq = bulkOut_XfrBlockMsg->bSeq;

  bulkIn_DataBlockMsg.bStatus = 0x00;

  bulkIn_DataBlockMsg.bError = 0;
  bulkIn_DataBlockMsg.bChainParameter = 0x00;

  /* Indicate smart card communication activity, toggle LED. */
  SC_LedOn();
  status = SC_SendAPDU(bulkOut_XfrBlockMsg, bulkIn_DataBlockMsg.abData, &messageLength);
  SC_LedOff();

  bulkIn_DataBlockMsg.dwLength = messageLength;

  /* Catch error conditions and set status-flag to indicate error. Not an exhaustive list. */
  /* Most status bytes result in command not supported error. */
  switch(status)
  {
    case SC7816_SW1_SUCCESS:
        /* Do nothing, just return SW1. */
    break;
    case SC7816_SW1_PURSE_ERROR:
        /* Do nothing, just return SW1. */
    break;        
    case SC7816_SW1_RESPONSE_BYTES:
        /* Do nothing, just return SW1. */
    break;    
    case SC7816_SW1_WARNING:
        /* Do nothing, just return SW1. */
    break;            
    case SC7816_SW1_WARNING_CHANGE:
        /* Do nothing, just return SW1. */  
    break;
    case SC7816_SW1_EXC_ERROR:
        /* CMD not supported. */
        bulkIn_DataBlockMsg.bStatus = SLOT_STATUS_CMD_FAILED;
        bulkIn_DataBlockMsg.bError = SLOT_ERROR_CODE_CMD_NOT_SUPPORTED;   
    break;        
    case SC7816_SW1_EXC_ERROR_CHANGE:
        /* CMD not supported. */
        bulkIn_DataBlockMsg.bStatus = SLOT_STATUS_CMD_FAILED;
        bulkIn_DataBlockMsg.bError = SLOT_ERROR_CODE_CMD_NOT_SUPPORTED;    
    break;            
    case SC7816_SW1_ERROR_WRONG_LENGTH:
        /* CMD not supported. */
        bulkIn_DataBlockMsg.bStatus = SLOT_STATUS_CMD_FAILED;
        bulkIn_DataBlockMsg.bError = SLOT_ERROR_CODE_CMD_NOT_SUPPORTED;
    break;            
    case SC7816_SW1_CLA_ERROR:
        /* CMD not supported. */
        bulkIn_DataBlockMsg.bStatus = SLOT_STATUS_CMD_FAILED;
        bulkIn_DataBlockMsg.bError = SLOT_ERROR_CODE_CMD_NOT_SUPPORTED;
    break;            
    case SC7816_SW1_COMMAND_ERROR:
        /* CMD not supported. */
        bulkIn_DataBlockMsg.bStatus = SLOT_STATUS_CMD_FAILED;
        bulkIn_DataBlockMsg.bError = SLOT_ERROR_CODE_CMD_NOT_SUPPORTED;     
    break;            
    case SC7816_SW1_P1_P2_ERROR:
        /* CMD not supported. */
        bulkIn_DataBlockMsg.bStatus = SLOT_STATUS_CMD_FAILED;
        bulkIn_DataBlockMsg.bError = SLOT_ERROR_CODE_CMD_NOT_SUPPORTED;   
    break;            
    case SC7816_SW1_LE_ERROR:
        /* CMD not supported. */
        bulkIn_DataBlockMsg.bStatus = SLOT_STATUS_CMD_FAILED;
        bulkIn_DataBlockMsg.bError = SLOT_ERROR_CODE_CMD_NOT_SUPPORTED;
    break;            
    case SC7816_SW1_INS_ERROR:
        /* CMD not supported. */
        bulkIn_DataBlockMsg.bStatus = SLOT_STATUS_CMD_FAILED;
        bulkIn_DataBlockMsg.bError = SLOT_ERROR_CODE_CMD_NOT_SUPPORTED;   
    break;            
    case SC7816_SW1_CLA_NOT_SUPPORTED:
        /* CMD not supported. */
        bulkIn_DataBlockMsg.bStatus = SLOT_STATUS_CMD_FAILED;
        bulkIn_DataBlockMsg.bError = SLOT_ERROR_CODE_CMD_NOT_SUPPORTED;
    break;            
    case APDU_STATUS_INVALID_PROCEDURE_BYTE:
        /* CMD not supported. */
        bulkIn_DataBlockMsg.bStatus = SLOT_STATUS_CMD_FAILED;
        bulkIn_DataBlockMsg.bError = SLOT_ERROR_CODE_CMD_NOT_SUPPORTED;  
    break;
    case APDU_STATUS_TIMEOUT:
        /* CMD timed out. */
        bulkIn_DataBlockMsg.bStatus = SLOT_STATUS_CMD_FAILED; 
        bulkIn_DataBlockMsg.bError = SLOT_ERROR_CODE_ICC_MUTE;
    break; 
    case APDU_STATUS_XFR_ERROR:
        /* CMD timed out. */
        bulkIn_DataBlockMsg.bStatus = SLOT_STATUS_CMD_FAILED; 
        bulkIn_DataBlockMsg.bError = SLOT_ERROR_CODE_XFR_PARITY_ERROR;
    break;  
    case APDU_STATUS_INVALID_CMD:
        /* CMD timed out. */
        bulkIn_DataBlockMsg.bStatus = SLOT_STATUS_CMD_FAILED; 
        bulkIn_DataBlockMsg.bError = SLOT_ERROR_CODE_CMD_NOT_SUPPORTED;
    break;         
    default:
      USBCCID_DEBUG_PRINTF("PC_to_RDR_XfrBlock defaulted, case not handled\n");
      /* Treat as unsupported command. */
      bulkIn_DataBlockMsg.bStatus = SLOT_STATUS_CMD_FAILED; 
      bulkIn_DataBlockMsg.bError = SLOT_ERROR_CODE_CMD_NOT_SUPPORTED;        
      
    break;
  } 

  USBD_Write(EP_BULK_IN, &bulkIn_DataBlockMsg, 10 + bulkIn_DataBlockMsg.dwLength, CCID_DataSentCallback);
}

/**********************************************************
 * Sends parameter response message over usb.
 **********************************************************/
void handleOutMsgGetParameters(void)
{
  USBCCID_DEBUG_PRINTF("PC_to_RDR_GetParameters\n");

  bulkIn_DataBlockMsg.bMessageType = RDR_to_PC_Parameters;
  bulkIn_DataBlockMsg.dwLength = 5;
  bulkIn_DataBlockMsg.bSlot = bulkOut_XfrBlockMsg->bSlot;
  bulkIn_DataBlockMsg.bSeq = bulkOut_XfrBlockMsg->bSeq;

  /* Card state, is card present or not. */
  if(CARD_STATE == CCID_STATE_CARD_PRESENT)
  {
    bulkIn_DataBlockMsg.bStatus = SLOT_STATUS_ICC_PRESENT_ACTIVE;
    bulkIn_DataBlockMsg.bError = 0;
  }else
  {
    bulkIn_DataBlockMsg.bStatus = SLOT_STATUS_ICC_NOT_PRESENT;
    bulkIn_DataBlockMsg.bError = SLOT_ERROR_CODE_ICC_MUTE;
  }

  bulkIn_DataBlockMsg.bChainParameter = 0x00;
  /* These are the default values, see table 7 and 9 in ISO/IEC 7816-3:1997 standard. */
  bulkIn_DataBlockMsg.abData[0] = 0x11;
  bulkIn_DataBlockMsg.abData[1] = 0x00;
  bulkIn_DataBlockMsg.abData[2] = 0x00;
  bulkIn_DataBlockMsg.abData[3] = 0x0a;
  bulkIn_DataBlockMsg.abData[4] = 0x00;

  USBD_Write(EP_BULK_IN, &bulkIn_DataBlockMsg, 10 + bulkIn_DataBlockMsg.dwLength, CCID_DataSentCallback);      

}

/**********************************************************
 * Sends ResetParameters response message over usb.
 **********************************************************/
void handleOutMsgResetParameters(void)
{
        
  USBCCID_DEBUG_PRINTF("PC_to_RDR_ResetParameters\n");
  bulkIn_DataBlockMsg.bMessageType = RDR_to_PC_Parameters;
  bulkIn_DataBlockMsg.dwLength = 0;
  bulkIn_DataBlockMsg.bSlot = bulkOut_XfrBlockMsg->bSlot;
  bulkIn_DataBlockMsg.bSeq = bulkOut_XfrBlockMsg->bSeq;

  /* CMD not supported. */
  bulkIn_DataBlockMsg.bStatus = SLOT_STATUS_CMD_FAILED;
  bulkIn_DataBlockMsg.bError = SLOT_ERROR_CODE_CMD_NOT_SUPPORTED;

  bulkIn_DataBlockMsg.bChainParameter = 0x00;

  USBD_Write(EP_BULK_IN, &bulkIn_DataBlockMsg, 10, CCID_DataSentCallback);  
 }

/**********************************************************
 * Sends SetParameters response message over usb.
 **********************************************************/
void handleOutMsgSetParameters(void)
{
  
  USBCCID_DEBUG_PRINTF("PC_to_RDR_SetParameters\n");
  bulkIn_DataBlockMsg.bMessageType = RDR_to_PC_Parameters;
  bulkIn_DataBlockMsg.dwLength = 0;
  bulkIn_DataBlockMsg.bSlot = bulkOut_XfrBlockMsg->bSlot;
  bulkIn_DataBlockMsg.bSeq = bulkOut_XfrBlockMsg->bSeq;

  /* CMD not supported. */
  bulkIn_DataBlockMsg.bStatus = SLOT_STATUS_CMD_FAILED;
  bulkIn_DataBlockMsg.bError = SLOT_ERROR_CODE_CMD_NOT_SUPPORTED;

  bulkIn_DataBlockMsg.bChainParameter = 0x00;

  USBD_Write(EP_BULK_IN, &bulkIn_DataBlockMsg, 10, CCID_DataSentCallback);  
}

/**********************************************************
 * Sends Escape response message over usb.
 **********************************************************/
void handleOutMsgEscape(void)
{
       
  USBCCID_DEBUG_PRINTF("PC_to_RDR_Escape\n");

  bulkIn_DataBlockMsg.bMessageType = RDR_to_PC_Escape;
  bulkIn_DataBlockMsg.dwLength = 0;
  bulkIn_DataBlockMsg.bSlot = bulkOut_XfrBlockMsg->bSlot;
  bulkIn_DataBlockMsg.bSeq = bulkOut_XfrBlockMsg->bSeq;

  /* CMD not supported. */
  bulkIn_DataBlockMsg.bStatus = SLOT_STATUS_CMD_FAILED;
  bulkIn_DataBlockMsg.bError = SLOT_ERROR_CODE_CMD_NOT_SUPPORTED;

  bulkIn_DataBlockMsg.bChainParameter = 0x00;

  USBD_Write(EP_BULK_IN, &bulkIn_DataBlockMsg, 10, CCID_DataSentCallback);  
}

/**********************************************************
 * Sends Icc Clock response message over usb.
 **********************************************************/
void handleOutMsgIccClock(void)
{
  USBCCID_DEBUG_PRINTF("PC_to_RDR_IccClock\n");

  bulkIn_SlotStatusMsg.bMessageType = RDR_to_PC_SlotStatus;
  bulkIn_SlotStatusMsg.dwLength = 0;
  bulkIn_SlotStatusMsg.bSlot = bulkOut_XfrBlockMsg->bSlot;
  bulkIn_SlotStatusMsg.bSeq = bulkOut_XfrBlockMsg->bSeq;

  /* CMD not supported. */
  bulkIn_SlotStatusMsg.bStatus = SLOT_STATUS_CMD_FAILED;
  bulkIn_SlotStatusMsg.bError = SLOT_ERROR_CODE_CMD_NOT_SUPPORTED;

  bulkIn_SlotStatusMsg.bClockStatus = 0x00;
        
  USBD_Write(EP_BULK_IN, &bulkIn_SlotStatusMsg, 10, CCID_DataSentCallback);      
}

/**********************************************************
 * Sends T0APDU response message over usb.
 **********************************************************/
void handleOutMsgT0APDU(void)
{
  USBCCID_DEBUG_PRINTF("PC_to_RDR_T0APDU\n");

  bulkIn_SlotStatusMsg.bMessageType = RDR_to_PC_SlotStatus;
  bulkIn_SlotStatusMsg.dwLength = 0;
  bulkIn_SlotStatusMsg.bSlot = bulkOut_XfrBlockMsg->bSlot;
  bulkIn_SlotStatusMsg.bSeq = bulkOut_XfrBlockMsg->bSeq;

  /* CMD not supported. */
  bulkIn_SlotStatusMsg.bStatus = SLOT_STATUS_CMD_FAILED;
  bulkIn_SlotStatusMsg.bError = SLOT_ERROR_CODE_CMD_NOT_SUPPORTED;

  bulkIn_SlotStatusMsg.bClockStatus = 0x00;
        
  USBD_Write(EP_BULK_IN, &bulkIn_SlotStatusMsg, 10, CCID_DataSentCallback);        
}

/**********************************************************
 * Sends Secure response message over usb.
 **********************************************************/
void handleOutMsgSecure(void)
{
  USBCCID_DEBUG_PRINTF("PC_to_RDR_Secure\n");

  bulkIn_DataBlockMsg.bMessageType = RDR_to_PC_DataBlock;
  bulkIn_DataBlockMsg.dwLength = 0;
  bulkIn_DataBlockMsg.bSlot = bulkOut_XfrBlockMsg->bSlot;
  bulkIn_DataBlockMsg.bSeq = bulkOut_XfrBlockMsg->bSeq;

  /* CMD not supported. */
  bulkIn_DataBlockMsg.bStatus = SLOT_STATUS_CMD_FAILED;
  bulkIn_DataBlockMsg.bError = SLOT_ERROR_CODE_CMD_NOT_SUPPORTED;

  bulkIn_DataBlockMsg.bChainParameter = 0x00;

  USBD_Write(EP_BULK_IN, &bulkIn_DataBlockMsg, 10, CCID_DataSentCallback);       
}

/**********************************************************
 * Sends Mechanical response message over usb.
 **********************************************************/
void handleOutMsgMechanical(void)
{
  USBCCID_DEBUG_PRINTF("PC_to_RDR_Mechanical\n");

  bulkIn_SlotStatusMsg.bMessageType = RDR_to_PC_SlotStatus;
  bulkIn_SlotStatusMsg.dwLength = 0;
  bulkIn_SlotStatusMsg.bSlot = bulkOut_XfrBlockMsg->bSlot;
  bulkIn_SlotStatusMsg.bSeq = bulkOut_XfrBlockMsg->bSeq;

  /* CMD not supported. */
  bulkIn_SlotStatusMsg.bStatus = SLOT_STATUS_CMD_FAILED;
  bulkIn_SlotStatusMsg.bError = SLOT_ERROR_CODE_CMD_NOT_SUPPORTED;

  bulkIn_SlotStatusMsg.bClockStatus = 0x00;
        
  USBD_Write(EP_BULK_IN, &bulkIn_SlotStatusMsg, 10, CCID_DataSentCallback);      
}

/**********************************************************
 * Sends Abort response message over usb.
 **********************************************************/
void handleOutMsgAbort(void)
{
  USBCCID_DEBUG_PRINTF("PC_to_RDR_Abort\n");

  bulkIn_SlotStatusMsg.bMessageType = RDR_to_PC_SlotStatus;
  bulkIn_SlotStatusMsg.dwLength = 0;
  bulkIn_SlotStatusMsg.bSlot = bulkOut_XfrBlockMsg->bSlot;
  bulkIn_SlotStatusMsg.bSeq = bulkOut_XfrBlockMsg->bSeq;

  /* CMD not supported. */
  bulkIn_SlotStatusMsg.bStatus = SLOT_STATUS_CMD_FAILED;
  bulkIn_SlotStatusMsg.bError = SLOT_ERROR_CODE_CMD_NOT_SUPPORTED;

  bulkIn_SlotStatusMsg.bClockStatus = 0x00;
        
  USBD_Write(EP_BULK_IN, &bulkIn_SlotStatusMsg, 10, CCID_DataSentCallback);       
}

/**********************************************************
 * Sends SetDataRateAndClockFrequency response message over usb.
 **********************************************************/
void handleOutMsgSetDataRateAndClockFrequency(void)
{
  int i;
  uint32_t value;
  USBCCID_DEBUG_PRINTF("PC_to_RDR_SetDataRateAndClockFrequency\n");

  bulkIn_DataBlockMsg.bMessageType = RDR_to_PC_DataRateAndClockFrequency;
  bulkIn_DataBlockMsg.dwLength = 8;
  bulkIn_DataBlockMsg.bSlot = bulkOut_XfrBlockMsg->bSlot;
  bulkIn_DataBlockMsg.bSeq = bulkOut_XfrBlockMsg->bSeq;

  bulkIn_DataBlockMsg.bStatus = SLOT_STATUS_ICC_PRESENT_ACTIVE | SLOT_STATUS_CMD_NO_ERROR;

  bulkIn_DataBlockMsg.bError = 0x00;

  bulkIn_DataBlockMsg.bChainParameter = 0x00;

  value = 3571;
  for(i = 0; i < 4; i++){
       bulkIn_DataBlockMsg.abData[i] = value >> (i*8);      
  }

  value = 9600;
  for(i = 0; i < 4; i++){
       bulkIn_DataBlockMsg.abData[i+4] = value >> (i*8);      
  }
      
  USBD_Write(EP_BULK_IN, &bulkIn_DataBlockMsg, 10 + bulkIn_DataBlockMsg.dwLength, CCID_DataSentCallback);  
            
}


/**********************************************************
 * Called when data is sent on the IN endpoint. 
 * 
 * @param status
 *   The transfer status. Should be USB_STATUS_OK if the
 *   transfer completed successfully.
 * 
 * @param xferred
 *   The number of bytes actually transmitted
 * 
 * @param remaining
 *   The number of bytes remaining (not transferred)
 **********************************************************/
int CCID_DataSentCallback(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining)
{
  /* Remove warnings for unused variables */
  (void)xferred;
  (void)remaining;
  
  if ( status != USB_STATUS_OK )
  {
    /* Handle error */
    EFM_ASSERT( false );
  }
  
  return USB_STATUS_OK;
}

/**********************************************************
 * If data received on usb, sets unhandled message to 1
 *
 * @param status
 *   The transfer status. Should be USB_STATUS_OK if the
 *   transfer completed successfully.
 * 
 * @param xferred
 *   The number of bytes actually received
 * 
 * @param remaining
 *   The number of bytes remaining (not transferred)
 **********************************************************/
int CCID_DataReceivedCallback(USB_Status_TypeDef status, uint32_t xferred, uint32_t remaining)
{
  /* Remove warnings for unused variables */
  (void)xferred;
  (void)remaining;
  
  /* Check status to verify that the transfer has completed successfully */
  if ( status == USB_STATUS_OK )
  {
   
    unhandled_message = 1;
 
  }
  else
  {
    /* Handle errors here.  */
    EFM_ASSERT( false );
  }
  
  
  return USB_STATUS_OK;
}



/**************************************************************************//**
 * @brief
 *   Handle USB setup commands.
 *
 * @param[in] setup Pointer to the setup packet received.
 *
 * @return USB_STATUS_REQ_UNHANDLED, the USB device
 *         stack will handle the request.
 *****************************************************************************/
static int SetupCmd(const USB_Setup_TypeDef *setup)
{
  /* Remove warnings for unused variables */
  (void)setup;
  
  int             retVal;

  retVal = USB_STATUS_REQ_UNHANDLED;
      
  return retVal;
}

/**************************************************************************//**
 * @brief
 *   Callback function called each time the USB device state is changed.
 *   Starts CDC operation when device has been configured by USB host.
 *
 * @param[in] oldState The device state the device has just left.
 * @param[in] newState The new device state.
 *****************************************************************************/
static void usbStateChange(USBD_State_TypeDef oldState, USBD_State_TypeDef newState)
{
  if (newState == USBD_STATE_CONFIGURED)
  {
    /* We have been configured, start CCID functionality ! */

    if (oldState == USBD_STATE_SUSPENDED)   /* Resume ?   */
    { 
      USBCCID_DEBUG_PRINTF("resume\n");
    }

    /* Start receiving data from USB host. */
    usbDeviceConfigured = 1;
    /* Prepare to accept the next message */
    USBD_Read(EP_BULK_OUT, receiveBuffer, USB_BUFFERSIZE, CCID_DataReceivedCallback);
 
    USBCCID_DEBUG_PRINTF("configured\n");
  }

  else if ((oldState == USBD_STATE_CONFIGURED) &&
           (newState != USBD_STATE_SUSPENDED))
  {
    /* We have been de-configured, stop CCID functionality */
    usbDeviceConfigured = 0;
    USBCCID_DEBUG_PRINTF("de-configured\n");
  }

  else if (newState == USBD_STATE_SUSPENDED)
  {
    /* We have been suspended, stop CCID functionality */
    /* Reduce current consumption to below 2.5 mA.    */
    usbDeviceConfigured = 0;
    USBCCID_DEBUG_PRINTF("suspended\n");
  }
}
