/*! @file radio.c
 * @brief This file contains functions to interface with the radio chip.
 *
 * @b COPYRIGHT
 * @n Silicon Laboratories Confidential
 * @n Copyright 2012 Silicon Laboratories, Inc.
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
tRadioConfiguration *pRadioConfiguration =  &RadioConfiguration;
uint8_t fixRadioPacket[RADIO_MAX_PACKET_LENGTH];

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
 *  Check if Packet received IT flag is pending.
 *
 *  @return   TRUE - Packet successfully received / FALSE - No packet pending.
 *
 *  @note
 *
 */
uint8_t gRadio_CheckReceived(void)
{

  if (radio_hal_NirqLevel() == FALSE)
  {
    /* Read ITs, clear pending ones */
    si446x_get_int_status(0u, 0u, 0u);

    /* check the reason for the IT */
    if (Si446xCmd.GET_INT_STATUS.MODEM_PEND & SI446X_CMD_GET_INT_STATUS_REP_SYNC_DETECT_BIT)
    {
        /* We do nothing here because there is not enough LED on EFM32LG-STK3600 kit */ 
    }

    if (Si446xCmd.GET_INT_STATUS.PH_PEND & SI446X_CMD_GET_INT_STATUS_REP_PACKET_RX_PEND_BIT)
    {
      /* Blink once LED1 as CRC OK or not enabled */
      vHmi_ChangeLedState(eHmi_Led1_c, eHmi_LedBlinkOnce_c);

      /* Packet RX */
      si446x_read_rx_fifo(pRadioConfiguration->Radio_PacketLength, fixRadioPacket);

      return TRUE;
    }

    if (Si446xCmd.GET_INT_STATUS.PH_PEND & SI446X_CMD_GET_INT_STATUS_REP_CRC_ERROR_BIT)
    {
      /* Reset FIFO */
      si446x_fifo_info(SI446X_CMD_FIFO_INFO_ARG_RX_BIT);
    }
  }

  return FALSE;
}

/*!
 *  Check if Packet received IT flag is pending.
 *
 *  @return   TRUE - Packet successfully received / FALSE - No packet pending.
 *
 *  @note
 *
 */
uint8_t gRadio_CheckReceived_VariablePacket(void)
{
  if (radio_hal_NirqLevel() == FALSE)
  {
    /* Read ITs, clear pending ones */
    si446x_get_int_status(0u, 0u, 0u);

	if (Si446xCmd.GET_INT_STATUS.CHIP_PEND & SI446X_CMD_GET_CHIP_STATUS_REP_CMD_ERROR_PEND_BIT)
    {
      /* State change to */
      si446x_change_state(SI446X_CMD_CHANGE_STATE_ARG_NEW_STATE_ENUM_SLEEP);
	
	  /* Reset FIFO */
      si446x_fifo_info(SI446X_CMD_FIFO_INFO_ARG_RX_BIT);
      
	  /* State change to */
      si446x_change_state(SI446X_CMD_CHANGE_STATE_ARG_NEW_STATE_ENUM_RX);
    }

    /* check the reason for the IT */
    if (Si446xCmd.GET_INT_STATUS.MODEM_PEND & SI446X_CMD_GET_INT_STATUS_REP_SYNC_DETECT_BIT)
    {
        /* We do nothing here because there is not enough LED on EFM32LG-STK3600 kit */ 
    }

    if (Si446xCmd.GET_INT_STATUS.PH_PEND & SI446X_CMD_GET_INT_STATUS_REP_PACKET_RX_PEND_BIT)
    {
#ifdef EFM32LG990F256
      /* Blink once LED1 as CRC OK or not enabled */
      vHmi_ChangeLedState(eHmi_Led1_c, eHmi_LedBlinkOnce_c);
#endif

#ifdef EFM32TG840F32
        /* We do nothing here because there is not enough LED on EFM32TG-STK3300 kit */ 
#endif
      /* Read the length of RX_FIFO */
      si446x_fifo_info(0u);

      /* Packet RX */
      si446x_read_rx_fifo(Si446xCmd.FIFO_INFO.RX_FIFO_COUNT, &fixRadioPacket[0]);

      return TRUE;
    }

    if (Si446xCmd.GET_INT_STATUS.PH_PEND & SI446X_CMD_GET_INT_STATUS_REP_CRC_ERROR_BIT)
    {
      /* Reset FIFO */
      si446x_fifo_info(SI446X_CMD_FIFO_INFO_ARG_RX_BIT);
    }
  }

  return FALSE;
}



/*!
 *  Set Radio to RX mode, fixed packet length.
 *
 *  @param channel Freq. Channel
 *
 *  @note
 *
 */
void vRadio_StartRX(uint8_t channel)
{
  // Read ITs, clear pending ones
  si446x_get_int_status(0u, 0u, 0u);

  /* Start Receiving packet, channel 0, START immediately, Packet length according to PH */
  si446x_start_rx(channel, 0u, 0x00,
                  SI446X_CMD_START_RX_ARG_RXTIMEOUT_STATE_ENUM_NOCHANGE,
                  SI446X_CMD_START_RX_ARG_RXVALID_STATE_ENUM_RX,
                  SI446X_CMD_START_RX_ARG_RXINVALID_STATE_ENUM_RX );

/* Do not show RX state on LED1 because there is not enough LED on EFM32LG-STK3600 kit*/

}
