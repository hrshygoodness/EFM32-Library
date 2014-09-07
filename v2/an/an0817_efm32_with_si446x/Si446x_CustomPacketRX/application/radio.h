/*! @file radio.h
 * @brief This file is contains the public radio interface functions.
 *
 * @b COPYRIGHT
 * @n Silicon Laboratories Confidential
 * @n Copyright 2012 Silicon Laboratories, Inc.
 * @n http://www.silabs.com
 */

#ifndef RADIO_H_
#define RADIO_H_

/*****************************************************************************
 *  Global Macros & Definitions
 *****************************************************************************/
/*! Maximal packet length definition (FIFO size) */
#define RADIO_MAX_PACKET_LENGTH     64u

/*****************************************************************************
 *  Global Typedefs & Enums
 *****************************************************************************/
typedef struct
{
    uint8_t   *Radio_ConfigurationArray;

    uint8_t   Radio_ChannelNumber;
    uint8_t   Radio_PacketLength;
    uint8_t   Radio_State_After_Power_Up;

    uint16_t  Radio_Delay_Cnt_After_Reset;

    uint8_t   Radio_CustomPayload[RADIO_MAX_PACKET_LENGTH];
} tRadioConfiguration;

/*****************************************************************************
 *  Global Variable Declarations
 *****************************************************************************/
extern tRadioConfiguration *pRadioConfiguration;
extern uint8_t fixRadioPacket[RADIO_MAX_PACKET_LENGTH];

/*! Si446x configuration array */
extern uint8_t Radio_Configuration_Data_Array[];

/*****************************************************************************
 *  Global Function Declarations
 *****************************************************************************/
void  vRadio_Init(void);
uint8_t   gRadio_CheckReceived(void);
uint8_t   gRadio_CheckReceived_VariablePacket(void);
void  vRadio_StartRX(uint8_t);
uint8_t    bRadio_Check_Ezconfig(uint16_t);

#endif /* RADIO_H_ */
