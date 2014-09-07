/*! @file radio.c
 * @brief This file contains functions to interface with the radio chip.
 *
 * @b COPYRIGHT
 * @n Silicon Laboratories Confidential
 * @n Copyright 2013 Silicon Laboratories, Inc.
 * @n http://www.silabs.com
 */
#include "bsp_def.h"

/*****************************************************************************
 *  Local Macros & Definitions
 *****************************************************************************/

/*****************************************************************************
 *  Global Variables
 *****************************************************************************/
uint8_t Radio_Configuration_Data_Array[] = RADIO_CONFIGURATION_DATA_ARRAY;

tRadioConfiguration RadioConfiguration = RADIO_CONFIGURATION_DATA;

tRadioConfiguration *pRadioConfiguration = &RadioConfiguration;


/*****************************************************************************
 *  Local Function Declarations
 *****************************************************************************/
void vRadio_PowerUp(void);

/*!
 *  Power up the Radio.
 *
 *  @note
 *
 */
void vRadio_PowerUp(void)
{
  uint16_t wDelay = 0u;

  /* Hardware reset the chip */
  si446x_reset();

  /* Wait until reset timeout or Reset IT signal */
  for (; wDelay < pRadioConfiguration->Radio_Delay_Cnt_After_Reset; wDelay++);
}

/*!
 *  Radio Initialization.
 *
 *  @author Sz. Papp
 *
 *  @note
 *
 */
void vRadio_Init(void)
{
  /* Power Up the radio chip */
  vRadio_PowerUp();

  /* Load radio configuration */
  while (SI446X_SUCCESS != si446x_configuration_init(pRadioConfiguration->Radio_ConfigurationArray))
  {
    /* Error hook */
    BSP_LedSet(0);
    Delay(2);
    BSP_LedClear(0);

    /* Power Up the radio chip */
    vRadio_PowerUp();
  }

  // Read ITs, clear pending ones
  si446x_get_int_status(0u, 0u, 0u);
}

/*!
 *  Check if Packet sent IT flag is pending.
 *
 *  @return   TRUE / FALSE
 *
 *  @note
 *
 */
uint8_t gRadio_CheckTransmitted(void)
{
  if (radio_hal_NirqLevel() == FALSE)
  {
    /* Read ITs, clear pending ones */
    si446x_get_int_status(0u, 0u, 0u);

    /* check the reason for the IT */
    if (Si446xCmd.GET_INT_STATUS.PH_PEND & SI446X_CMD_GET_INT_STATUS_REP_PACKET_SENT_PEND_BIT)
    {
      return TRUE;
    }
  }

  return FALSE;
}

/*!
 *  Set Radio to TX mode, fixed packet length.
 *
 *  @param channel Freq. Channel, Packet to be sent
 *
 *  @note
 *
 */
void  vRadio_StartTx(uint8_t channel, uint8_t *pioFixRadioPacket)
{
  /* Reset TX FIFO */
  si446x_fifo_info(SI446X_CMD_FIFO_INFO_ARG_TX_BIT);

  // Read ITs, clear pending ones
  si446x_get_int_status(0u, 0u, 0u);

  /* Fill the TX fifo with datas */
  si446x_write_tx_fifo(RadioConfiguration.Radio_PacketLength, pioFixRadioPacket);

  /* Start sending packet, channel 0, START immediately, Packet length according to PH, go READY when done */
  si446x_start_tx(channel, 0x30,  0x00);
}
