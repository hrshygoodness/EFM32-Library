/******************************************************************************
 * @file sc_driver.h
 * @brief Smart Card Driver header file
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

#ifndef __SC_DRIVER_H
#define __SC_DRIVER_H

/* Status and errors returned from card-comm function */
/* These errors must not be within valid SW1 range (>0x60) */
typedef enum
{
	APDU_STATUS_SUCCESS,
	APDU_STATUS_INVALID_CMD,
	APDU_STATUS_INVALID_INS,
	APDU_STATUS_INVALID_P1P2,
	APDU_STATUS_INVALID_P3,
	APDU_STATUS_UNKNOWN_ERROR,
	APDU_STATUS_INVALID_PROCEDURE_BYTE,
	APDU_STATUS_TIMEOUT,
        APDU_STATUS_XFR_ERROR,
}APDU_STATUS;

/* Errors returned from Card */
#define SC7816_SW1_SUCCESS               0x90
#define SC7816_SW1_PURSE_ERROR           0x91
#define SC7816_SW1_RESPONSE_BYTES        0x61
#define SC7816_SW1_WARNING               0x62
#define SC7816_SW1_WARNING_CHANGE        0x63
#define SC7816_SW1_EXC_ERROR             0x64
#define SC7816_SW1_EXC_ERROR_CHANGE      0x65
#define SC7816_SW1_ERROR_WRONG_LENGTH    0x67
#define SC7816_SW1_CLA_ERROR             0x68
#define SC7816_SW1_COMMAND_ERROR         0x69
#define SC7816_SW1_P1_P2_ERROR           0x6a
#define SC7816_SW1_LE_ERROR              0x6c
#define SC7816_SW1_INS_ERROR             0x6d
#define SC7816_SW1_CLA_NOT_SUPPORTED     0x6e


/***************************************************************************//**
 * @brief
 *   Initialize GPIO, TIMER and UART necessary for smart card communication
 *
 ******************************************************************************/
void SC_Init(void);

/***************************************************************************//**
 * @brief
 *   Turn on smartcard clock
 *
 ******************************************************************************/
void SC_ClockOn(void);

/***************************************************************************//**
 * @brief
 *   Turn off smartcard clock
 *
 ******************************************************************************/
void SC_ClockOff(void);

/***************************************************************************//**
 * @brief
 *   Turn on smartcard power
 *
 ******************************************************************************/
void SC_PowerOn(void);

/***************************************************************************//**
 * @brief
 *   Turn off smartcard power
 *
 ******************************************************************************/
void SC_PowerOff(void);

/***************************************************************************//**
 * @brief
 *   Disable Reset
 *
 ******************************************************************************/
void SC_DisableReset(void);

/***************************************************************************//**
 * @brief
 *   Pull smartcard reset low
 *
 ******************************************************************************/
void SC_AssertReset(void);

/***************************************************************************//**
 * @brief
 *   Release smartcard reset to high state
 *
 ******************************************************************************/
void SC_DeassertReset(void);

/***************************************************************************//**
 * @brief
 *   Check gpio for smart card insertion
 *
 * @return
 *   1 if smartcard inserted, 0 otherwise.
 ******************************************************************************/    
uint8_t SC_IsCardInserted(void);

/***************************************************************************//**
 * @brief
 *   Turn activity LED on
 *
 ******************************************************************************/
void SC_LedOn(void);

/***************************************************************************//**
 * @brief
 *   Turn activity LED off
 *
 ******************************************************************************/
void SC_LedOff(void);

/***************************************************************************//**
 * @brief
 *   Perform smart card reset sequence and get Answer to Reset
 *
 * @param[out] atr_buf
 *   pointer to buffer for ATR.
 *
 * @param[in] len
 *   max length of ATR to wait for.
 *
 * @param[out] receivedATRLength
 *   pointer to uint32 for actual received ATR length.
 *
 * @return
 *   APDU status enum.
 ******************************************************************************/
uint8_t SC_Reset(uint8_t *atr_buf, uint32_t * receivedATRLength);

/***************************************************************************//**
 * @brief
 *   Perform smart card APDU transaction sequence and gets smart card response.
 *   This function operates according to T=0 protocol with standard length APDU.
 *   It expects 5 command bytes and data in/out, then accepts and acts upon 
 *   procedure byte and status byte (SW1 and SW2) from smart card.  
 *
 * @param[in] XfrBlock
 *   pointer to buffer for transfer block which contains APDU to send to card.
 *
 * @param[out] response_abData
 *   Pointer to buffer to write smart card output data to. 
 *
 * @param[out] response_dwLength
 *   pointer to uint32 for actual received response length.
 *
 * @return
 *   APDU status enum.
 ******************************************************************************/
uint8_t SC_SendAPDU(void* XfrBlockStruct, uint8_t* response_abData, uint32_t* response_dwLength);


#endif /* __SC_DRIVER_H */
