/***************************************************************************//**
 * @file
 * @brief Driver for Micrel KSZ8851SNL ethernet controller.
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


#include "ksz8851snl.h"
#include "ethspi.h"
#include "em_gpio.h"
#include <stdio.h>

/** @cond DO_NOT_INCLUDE_WITH_DOXYGEN */
static uint16_t frameId = 0;        /**< Raw ethernet frame sent over the network */
static uint8_t  macAddress[6];      /**< Unique MAC address */

#if KSZ8851SNL_DEBUG
static void     KSZ8851SNL_SetDigitalLoopbackMode(void);
static void     KSZ8851SNL_DumpRegisters(void);
#endif /* KSZ8851SNL_DEBUG */
static void     KSZ8851SNL_ExceptionHandler(enum exceptionType_e exc_type, char* param);
static void     KSZ8851SNL_ReleaseIncosistentFrame(void);
static uint8_t  KSZ8851SNL_DwordAllignDiff(uint8_t val);
/** @endcond */

/**************************************************************************//**
 * @brief enables the chip interrupts
 *****************************************************************************/
void KSZ8851SNL_EnableInterupts(void)
{
  uint16_t data;
  /* Enable interupts */
  data = INT_MASK_EXAMPLE;
  ETHSPI_WriteRegister(INT_ENABLE_REG, 2, &data);
}


/***************************************************************************//**
 * @brief Checks for any interrrupts and if found, clears their status
 *        and prepair for interrupt handler routines
 * @note  Programmer needs to re-enable the KSZ8851SNL interrupts after
 *        calling this function
 *
 * @return
 *     found interrupts
 *
 *****************************************************************************/
uint16_t KSZ8851SNL_CheckIrqStat(void)
{
  uint16_t data, ISR_stat, found_INT;
  found_INT = 0;
  ETHSPI_ReadRegister(INT_STATUS_REG, 2, &ISR_stat);

  /* Disable interupts on KSZ8851SNL */
  data = NO_INT;
  ETHSPI_WriteRegister(INT_ENABLE_REG, 2, &data);

  /* Resolve the RX completion interrupt */
  if (ISR_stat & INT_RX_DONE)
  {
    /* Clear RX Interrupt flag */
    ETHSPI_ReadRegister(INT_STATUS_REG, 2, &data);
    data = INT_RX_DONE;
    ETHSPI_WriteRegister(INT_STATUS_REG, 2, &data);
    found_INT |= INT_RX_DONE;
  }
  /* Resolve the Link change interrupt */
  if (ISR_stat & INT_LINK_CHANGE)
  {
    /* Clear Link change Interrupt flag */
    ETHSPI_ReadRegister(INT_STATUS_REG, 2, &data);
    data = INT_LINK_CHANGE;
    ETHSPI_WriteRegister(INT_STATUS_REG, 2, &data);
    found_INT |= INT_LINK_CHANGE;
  }
  /* Resolve the RX overrun interrupt */
  if (ISR_stat & INT_RX_OVERRUN)
  {
    /* Clear RX overrun Interrupt flag */
    ETHSPI_ReadRegister(INT_STATUS_REG, 2, &data);
    data = INT_RX_OVERRUN;
    ETHSPI_WriteRegister(INT_STATUS_REG, 2, &data);
    found_INT |= INT_RX_OVERRUN;
  }
  /* Resolve the TX stopped interrupt */
  if (ISR_stat & INT_TX_STOPPED)
  {
    /* Clear TX stopped Interrupt flag */
    ETHSPI_ReadRegister(INT_STATUS_REG, 2, &data);
    data = INT_TX_STOPPED;
    ETHSPI_WriteRegister(INT_STATUS_REG, 2, &data);
    found_INT |= INT_TX_STOPPED;
  }
  /* Resolve the RX stopped interrupt */
  if (ISR_stat & INT_RX_STOPPED)
  {
    /* Clear RX stopped Interrupt flag */
    ETHSPI_ReadRegister(INT_STATUS_REG, 2, &data);
    data = INT_RX_STOPPED;
    ETHSPI_WriteRegister(INT_STATUS_REG, 2, &data);
    found_INT |= INT_RX_STOPPED;
  }
  /* Resolve the RX of a WakeOnLan frame interrupt */
  if (ISR_stat & INT_RX_WOL_FRAME)
  {
    /* Clear RX of a WakeOnLan Interrupt flag */
    ETHSPI_ReadRegister(INT_STATUS_REG, 2, &data);
    data = INT_RX_WOL_FRAME;
    ETHSPI_WriteRegister(INT_STATUS_REG, 2, &data);
    found_INT |= INT_RX_WOL_FRAME;
  }
  /* Resolve the RX of a magic frame interrupt */
  if (ISR_stat & INT_MAGIC)
  {
    /* Clear RX of a magic frame Interrupt flag */
    ETHSPI_ReadRegister(INT_STATUS_REG, 2, &data);
    data = INT_MAGIC;
    ETHSPI_WriteRegister(INT_STATUS_REG, 2, &data);
    found_INT |= INT_MAGIC;
  }
  /* Resolve the RX of a LINKUP interrupt */
  if (ISR_stat & INT_LINKUP)
  {
    /* Clear RX of a LINKUP Interrupt flag */
    ETHSPI_ReadRegister(INT_STATUS_REG, 2, &data);
    data = INT_LINKUP;
    ETHSPI_WriteRegister(INT_STATUS_REG, 2, &data);
    found_INT |= INT_LINKUP;
  }
  /* Resolve the RX of a Energy interrupt */
  if (ISR_stat & INT_ENERGY)
  {
    /* Clear RX of a Energy Interrupt flag */
    ETHSPI_ReadRegister(INT_STATUS_REG, 2, &data);
    data = INT_ENERGY;
    ETHSPI_WriteRegister(INT_STATUS_REG, 2, &data);
    found_INT |= INT_ENERGY;
  }
  /* Resolve the SPI Error interrupt */
  if (ISR_stat & INT_SPI_ERROR)
  {
    /* Clear SPI Error Interrupt flag */
    ETHSPI_ReadRegister(INT_STATUS_REG, 2, &data);
    data = INT_SPI_ERROR;
    ETHSPI_WriteRegister(INT_STATUS_REG, 2, &data);
    found_INT |= INT_SPI_ERROR;
  }
  /* Resolve the TX space interrupt */
  if (ISR_stat & INT_TX_SPACE)
  {
    /* Clear TX space Interrupt flag */
    ETHSPI_ReadRegister(INT_STATUS_REG, 2, &data);
    data = INT_TX_SPACE;
    ETHSPI_WriteRegister(INT_STATUS_REG, 2, &data);
    found_INT |= INT_TX_SPACE;
  }
  return found_INT;
}


/***************************************************************************//**
 * @brief Returns the size of the currently received frame
 *
 * @return
 *     the printed string
 *
 *****************************************************************************/
uint16_t KSZ8851SNL_CurrFrameSize(void)
{
  uint16_t data;

  /* Read the byte size of the received frame */
  ETHSPI_ReadRegister(RX_FRH_BC_REG, 2, &data);

  data &= RX_BYTE_CNT_MASK;

  return data;
}


/***************************************************************************//**
 * @brief Returns the difference in bytes to be DWORD alligned
 *
 * @param val
 *     value that needs to be alligned
 * @return
 *     the number of bytes needed to be added so that the value is alligned
 *
 *****************************************************************************/
static uint8_t KSZ8851SNL_DwordAllignDiff(uint8_t val)
{
  if (val % 4 == 0) return 0;
  else return val % 4;
}


/***************************************************************************//**
 * @brief Prints a message with a predetermined exception level
 *        Support method used for debugging.
 *
 * @param exc_type
 *     the exception level
 * @param param
 *     the printed string
 *
 *****************************************************************************/
static void KSZ8851SNL_ExceptionHandler(enum exceptionType_e exc_type, char* param)
{
  uint16_t data;
  (void)param; /* unused param */
  /* Disable interupts on KSZ8851SNL */
  data = NO_INT;
  ETHSPI_WriteRegister(INT_ENABLE_REG, 2, &data);
  switch (exc_type)
  {
  case ERROR:
    DEBUG_PRINT("ERROR:%s\n", param);
    while (1) ;
  case INFO:
    DEBUG_PRINT("INFO:%s\n", param);
    break;
  }
  /* Enable interupts */
  data = INT_MASK_EXAMPLE;
  ETHSPI_WriteRegister(INT_ENABLE_REG, 2, &data);
}


/***************************************************************************//**
 * @brief Dumps the Management Information Base Counters
 * @note  Support method used for debugging.
 *
 * @param param
 *     the string representing the moment of the register dump
 *
 *****************************************************************************/
void KSZ8851SNL_ReadMIBCounters(char* param)
{
  EFM_ASSERT(param != NULL);
  uint16_t data, dataLow, dataHigh;
  DEBUG_PRINT("#################################################################\n");
  DEBUG_PRINT("Dumping MIB Counters values @%s\n", param);
  int i;
  for (i = 0; i < 0x20; i++)
  {
    data = MIB_MASK | i;
    ETHSPI_WriteRegister(IND_ACC_CTRL_REG, 2, &data);
    ETHSPI_ReadRegister(IND_ACC_DATA_LOW_REG, 2, &dataLow);
    ETHSPI_ReadRegister(IND_ACC_DATA_HIGH_REG, 2, &dataHigh);
    DEBUG_PRINT("MIB_REG[%2X] contains %X - %X\n", i, dataHigh, dataLow);
  }
  DEBUG_PRINT("#################################################################\n");
}


#if KSZ8851SNL_DEBUG
/***************************************************************************//**
 * @brief Prints the value of the registers of the ethernet controller.
 * @note  Support method used for debugging.
 *****************************************************************************/
static void KSZ8851SNL_DumpRegisters(void)
{
  uint16_t data;

  DEBUG_PRINT("#################################################################\n");
  DEBUG_PRINT("Dumping Register values\n");

  DEBUG_PRINT("#################################################################\n");
  DEBUG_PRINT("Dumping ALL Register values\n");

  int i;
  for (i = 0; = 0x00; i < 0xFF; i += 0x02)
  {
    ETHSPI_ReadRegister(i, 2, &data);
    DEBUG_PRINT("REG[0x%X] contains 0x%X\n", i, data);
  }
  DEBUG_PRINT("#################################################################\n");
  DEBUG_PRINT("Dumping used registers values\n");

  ETHSPI_ReadRegister(LOW_QMU_MAC_REG, 2, &data);
  DEBUG_PRINT("REG[%2X] contains %2X\n", LOW_QMU_MAC_REG, data);
  ETHSPI_ReadRegister(MID_QMU_MAC_REG, 2, &data);
  DEBUG_PRINT("REG[%2X] contains %2X\n", MID_QMU_MAC_REG, data);
  ETHSPI_ReadRegister(HIGH_QMU_MAC_REG, 2, &data);
  DEBUG_PRINT("REG[%2X] contains %2X\n", HIGH_QMU_MAC_REG, data);
  ETHSPI_ReadRegister(OBC_REG, 2, &data);
  DEBUG_PRINT("REG[%2X] contains %2X\n", OBC_REG, data);
  ETHSPI_ReadRegister(GLOBAL_RESET_REG, 2, &data);
  DEBUG_PRINT("REG[%2X] contains %2X\n", GLOBAL_RESET_REG, data);
  ETHSPI_ReadRegister(TX_FLOW_CTRL_REG, 2, &data);
  DEBUG_PRINT("REG[%2X] contains %2X\n", TX_FLOW_CTRL_REG, data);
  ETHSPI_ReadRegister(RX_FLOW_CTRL1_REG, 2, &data);
  DEBUG_PRINT("REG[%2X] contains %2X\n", RX_FLOW_CTRL1_REG, data);
  ETHSPI_ReadRegister(RX_FLOW_CTRL2_REG, 2, &data);
  DEBUG_PRINT("REG[%2X] contains %2X\n", RX_FLOW_CTRL2_REG, data);
  ETHSPI_ReadRegister(TX_MEM_INFO_REG, 2, &data);
  DEBUG_PRINT("REG[%2X] contains %2X\n", TX_MEM_INFO_REG, data);
  ETHSPI_ReadRegister(RX_FRH_STAT_REG, 2, &data);
  DEBUG_PRINT("REG[%2X] contains %2X\n", RX_FRH_STAT_REG, data);
  ETHSPI_ReadRegister(TXQ_CMD_REG, 2, &data);
  DEBUG_PRINT("REG[%2X] contains %2X\n", TXQ_CMD_REG, data);
  ETHSPI_ReadRegister(RXQ_CMD_REG, 2, &data);
  DEBUG_PRINT("REG[%2X] contains %2X\n", RXQ_CMD_REG, data);
  ETHSPI_ReadRegister(TX_FD_PTR_REG, 2, &data);
  DEBUG_PRINT("REG[%2X] contains %2X\n", TX_FD_PTR_REG, data);
  ETHSPI_ReadRegister(RX_FD_PTR_REG, 2, &data);
  DEBUG_PRINT("REG[%2X] contains %2X\n", RX_FD_PTR_REG, data);
  ETHSPI_ReadRegister(INT_ENABLE_REG, 2, &data);
  DEBUG_PRINT("REG[%2X] contains %2X\n", INT_ENABLE_REG, data);
  ETHSPI_ReadRegister(INT_STATUS_REG, 2, &data);
  DEBUG_PRINT("REG[%2X] contains %2X\n", INT_STATUS_REG, data);
  ETHSPI_ReadRegister(RX_FRAME_THRES_REG, 2, &data);
  DEBUG_PRINT("REG[%2X] contains %2X\n", RX_FRAME_THRES_REG, data);
  ETHSPI_ReadRegister(TX_NEXT_FRS_REG, 2, &data);
  DEBUG_PRINT("REG[%2X] contains %2X\n", TX_NEXT_FRS_REG, data);
  ETHSPI_ReadRegister(CIDER_REG, 2, &data);
  DEBUG_PRINT("REG[%2X] contains %2X\n", CIDER_REG, data);
  ETHSPI_ReadRegister(PHY_RST_REG, 2, &data);
  DEBUG_PRINT("REG[%2X] contains %2X\n", PHY_RST_REG, data);
  ETHSPI_ReadRegister(PHY1_CTRL_REG, 2, &data);
  DEBUG_PRINT("REG[%2X] contains %2X\n", PHY1_CTRL_REG, data);
  ETHSPI_ReadRegister(PORT1_CTRL_REG, 2, &data);
  DEBUG_PRINT("REG[%2X] contains %2X\n", PORT1_CTRL_REG, data);
  DEBUG_PRINT("#################################################################\n");
}
#endif /* KSZ8851SNL_DEBUG */


/***************************************************************************//**
 * @brief Initialize the registers of the ethernet controller.
 *****************************************************************************/
void KSZ8851SNL_Init(void)
{
  uint16_t data;

  /* Initialize SPI Interface */
  ETHSPI_Init();

  /* Reset Soft (clear registers of PHY, MAC, QMU, DMA) */
  data = GLOBAL_SOFT_RESET;
  ETHSPI_WriteRegister(GLOBAL_RESET_REG, 2, &data);
  ETHSPI_ReadRegister(GLOBAL_RESET_REG, 2, &data);
  data &= ~GLOBAL_SOFT_RESET;
  ETHSPI_WriteRegister(GLOBAL_RESET_REG, 2, &data);

  /* Reset QMU Modules(flush out TXQ and RXQ) */
  data = QMU_MODULE_SOFT_RESET;
  ETHSPI_WriteRegister(GLOBAL_RESET_REG, 2, &data);
  ETHSPI_ReadRegister(GLOBAL_RESET_REG, 2, &data);
  data &= ~QMU_MODULE_SOFT_RESET;
  ETHSPI_WriteRegister(GLOBAL_RESET_REG, 2, &data);

#ifdef DIGITAL_PHY_LOOPBACK
  KSZ8851SNL_SetDigitalLoopbackMode();
#endif /* DIGITAL_PHY_LOOPBACK */

  /* Read the chip ID and check if that is correct */
  ETHSPI_ReadRegister(CIDER_REG, 2, &data);

  /* The CIDER lower bits [3..1] are defined as revision number,
   *   thus a mask needs to be applied
   */
  if ((data & CHIP_ID_MASK) != KSZ8851SNL_CHIP_ID)
  {
    KSZ8851SNL_ExceptionHandler(ERROR, "ETH: Incorrect Device ID");
  }

  /* Write the Queue Management Unit MAC Address */
  KSZ8851SNL_GetMacAddress(macAddress);
  /* Write the appropriate KSZ8851SNL MAC registers
   *   starting from the HIGH part towards the lower one
   *   going with a step of 2
   */
  int i;
  for (i = 0; (i < 6); i += 2)
  {
    data = (macAddress[i] << MSB_POS) | macAddress[i + 1];
    ETHSPI_WriteRegister(HIGH_QMU_MAC_REG - i, 2, &data);
  }

  /* Enable QMU Transmit Frame Data Pointer Auto Increment */
  data = FD_PTR_AUTO_INC;
  ETHSPI_WriteRegister(TX_FD_PTR_REG, 2, &data);

  /* FLUSH TX queue */
  data |= TX_FLOW_CTRL_FLUSH_QUEUE;
  ETHSPI_WriteRegister(TX_FLOW_CTRL_REG, 2, &data);

  /* Enable QMU Transmit:
   *  flow control,
   *  padding,
   *  CRC,
   *  IP/TCP/UDP/ICMP checksum generation.
   */
  data = TX_FLOW_CTRL_EXAMPLE;
  ETHSPI_WriteRegister(TX_FLOW_CTRL_REG, 2, &data);

  /* Enable QMU Receive Frame Data Pointer Auto Increment */
  data = FD_PTR_AUTO_INC;
  ETHSPI_WriteRegister(RX_FD_PTR_REG, 2, &data);

  /* Configure Receive Frame Threshold for one frame */
  data = ONE_FRAME_THRES;
  ETHSPI_WriteRegister(RX_FRAME_THRES_REG, 2, &data);

  /* Enable QMU Receive:
   *  flow control,
   *  receive all broadcast frame,
   *  receive unicast frame,
   *  IP/TCP/UDP/ICMP checksum generation.
   */
  data = RX_FLOW_CTRL1_EXAMPLE;
  ETHSPI_WriteRegister(RX_FLOW_CTRL1_REG, 2, &data);

  /* Enable QMU Receive:
   *  ICMP/UDP Lite frame checksum verification,
   *  UDP Lite frame checksum generation,
   *  IPv6 UDP fragment frame pass.
   */
  data = RX_FLOW_CTRL2_EXAMPLE;
  ETHSPI_WriteRegister(RX_FLOW_CTRL2_REG, 2, &data);

  /* Enable QMU Receive:
   *  IP Header Two-Byte Offset,
   *  Receive Frame Count Threshold,
   *  RXQ Auto-Dequeue frame.
   */
  data = RXQ_CMD_EXAMPLE;
  ETHSPI_WriteRegister(RXQ_CMD_REG, 2, &data);

  /* Restart Port 1 auto-negotiation */
  ETHSPI_ReadRegister(PORT1_CTRL_REG, 2, &data);
  data |= PORT1_AUTO_NEG_RESTART;
  ETHSPI_WriteRegister(PORT1_CTRL_REG, 2, &data);

  /* Force link in half duplex if auto-negotiation failed  */
  ETHSPI_ReadRegister(PORT1_CTRL_REG, 2, &data);
  if ((data & PORT1_AUTO_NEG_RESTART) != PORT1_AUTO_NEG_RESTART)
  {
    data &= ~PORT1_FORCE_FULL_DUPLEX;
    ETHSPI_WriteRegister(PORT1_CTRL_REG, 2, &data);
  }
  /* Configure Low Watermark to 6KByte available buffer space out of 12KByte */
  data = WATERMARK_6KB;
  ETHSPI_WriteRegister(FLOW_CTRL_LOW_WATERMARK, 2, &data);

  /* Configure High Watermark to 4KByte available buffer space out of 12KByte */
  data = WATERMARK_4KB;
  ETHSPI_WriteRegister(FLOW_CTRL_HIGH_WATERMARK, 2, &data);

  /* Clear the interrupts status */
  data = CLEAR_INT;
  ETHSPI_WriteRegister(INT_STATUS_REG, 2, &data);

  /* Enable Interrupts on:
   *  Link Change
   *  Transmit
   *  Receive
   *  Receive Overrun
   *  Transmit Process Stop
   *  Receive Process Stop
   */
  data = INT_MASK_EXAMPLE;
  ETHSPI_WriteRegister(INT_ENABLE_REG, 2, &data);

  /* Enable QMU Transmit */
  ETHSPI_ReadRegister(TX_FLOW_CTRL_REG, 2, &data);
  data |= TX_FLOW_CTRL_ENABLE;
  ETHSPI_WriteRegister(TX_FLOW_CTRL_REG, 2, &data);

  /* Enable QMU Receive */
  ETHSPI_ReadRegister(RX_FLOW_CTRL1_REG, 2, &data);
  data |= RX_FLOW_CTRL_ENABLE;
  ETHSPI_WriteRegister(RX_FLOW_CTRL1_REG, 2, &data);

  KSZ8851SNL_ExceptionHandler(INFO, "ETH: Initialization complete");
}


/***************************************************************************//**
 * @brief Performs the actual transmit of a raw frame over the network.
 *
 * @param pTXLength
 *     the length of the transmitted frame
 * @param pTXData
 *     the data of the transmitted frame
 *****************************************************************************/
void KSZ8851SNL_Send(uint16_t pTXLength, uint8_t *pTXData)
{
  EFM_ASSERT(pTXData != NULL);

  uint16_t txmir;
  uint16_t data, reqSize;
  uint8_t  outbuf[4];

  /* Check if TXQ has enough memory for the transmission of the package */
  ETHSPI_ReadRegister(TX_MEM_INFO_REG, 2, &data);
  txmir = data & TX_MEM_AVAIL_MASK;

  reqSize = pTXLength + EXTRA_SIZE;
  if (txmir < reqSize)
  {
    KSZ8851SNL_ExceptionHandler(INFO, "I will wait until mem is available\n");
    /* Enable TX memory available monitor */
    ETHSPI_WriteRegister(TX_NEXT_FRS_REG, 2, &reqSize);
    ETHSPI_ReadRegister(TXQ_CMD_REG, 2, &data);
    data |= TXQ_MEM_AVAILABLE_INT;
    ETHSPI_WriteRegister(TXQ_CMD_REG, 2, &data);

    /* Wait until enough space is available */
    while (1)
    {
      ETHSPI_ReadRegister(INT_STATUS_REG, 2, &data);
      if ((data & INT_TX_SPACE) == INT_TX_SPACE)
      {
        break;
      }
    }
    KSZ8851SNL_ExceptionHandler(INFO, "Done\n");

    /* Clear flag */
    ETHSPI_ReadRegister(INT_STATUS_REG, 2, &data);
    data &= ~INT_TX_SPACE;
    ETHSPI_WriteRegister(INT_STATUS_REG, 2, &data);
  }

  /* Disable interupts on KSZ8851SNL */
  data = NO_INT;
  ETHSPI_WriteRegister(INT_ENABLE_REG, 2, &data);

  /* Enable TXQ write access */
  ETHSPI_ReadRegister(RXQ_CMD_REG, 2, &data);
  data |= RXQ_START;
  ETHSPI_WriteRegister(RXQ_CMD_REG, 2, &data);

  /* Write frame ID, control word and byte count */
  outbuf[0] = (frameId++ & frameId_MASK) | TX_INT_on_COMPLETION;
  outbuf[1] = 0;
  outbuf[2] = pTXLength & LSB_MASK;
  outbuf[3] = pTXLength >> MSB_POS;

  /* Start the SPI Transfer */
  ETHSPI_StartWriteFIFO();
  /* Send the frame header info */
  ETHSPI_WriteFifoContinue(4, outbuf);

  /* Send the actual data */
  ETHSPI_WriteFifoContinue(pTXLength, pTXData);

  /* Send dummy bytes to align data to DWORD */
  ETHSPI_WriteFifoContinue(KSZ8851SNL_DwordAllignDiff(pTXLength), pTXData);

  /* Stop the SPI Transfer */
  ETHSPI_StopFIFO();

  /* Disable TXQ write access */
  ETHSPI_ReadRegister(RXQ_CMD_REG, 2, &data);
  data &= ~RXQ_START;
  ETHSPI_WriteRegister(RXQ_CMD_REG, 2, &data);

  /* Start TXQ Manual Engue */
  ETHSPI_ReadRegister(TXQ_CMD_REG, 2, &data);
  data |= TXQ_ENQUEUE;
  ETHSPI_WriteRegister(TXQ_CMD_REG, 2, &data);

  /* Wait until transmit command clears */
  while (1)
  {
    ETHSPI_ReadRegister(TXQ_CMD_REG, 2, &data);
    if (!(data & TXQ_ENQUEUE))
      break;
  }

  /* Enable interupts */
  data = INT_MASK_EXAMPLE;
  ETHSPI_WriteRegister(INT_ENABLE_REG, 2, &data);
}


/***************************************************************************//**
 * @brief Performs the initialisation of the transmission of a long raw frame
 *        over the network.
 *
 * @param pTXLength
 *     the length of the transmitted frame
 *****************************************************************************/
void KSZ8851SNL_InitiateLongTransmit(uint16_t pTXLength)
{
  uint16_t txmir;
  uint16_t data, reqSize;
  uint8_t  outbuf[4];

  /* Check if TXQ has enough memory for the transmission of the package */
  ETHSPI_ReadRegister(TX_MEM_INFO_REG, 2, &data);
  txmir = data & TX_MEM_AVAIL_MASK;

  reqSize = pTXLength + EXTRA_SIZE;
  if (txmir < reqSize)
  {
    KSZ8851SNL_ExceptionHandler(INFO, "I will wait until mem is available\n");
    /* Enable TX memory available monitor */
    ETHSPI_WriteRegister(TX_NEXT_FRS_REG, 2, &reqSize);
    ETHSPI_ReadRegister(TXQ_CMD_REG, 2, &data);
    data |= TXQ_MEM_AVAILABLE_INT;
    ETHSPI_WriteRegister(TXQ_CMD_REG, 2, &data);

    /* Wait until enough space is available */
    while (1)
    {
      ETHSPI_ReadRegister(INT_STATUS_REG, 2, &data);
      if ((data & INT_TX_SPACE) == INT_TX_SPACE)
      {
        break;
      }
    }
    KSZ8851SNL_ExceptionHandler(INFO, "Done\n");

    /* Clear flag */
    ETHSPI_ReadRegister(INT_STATUS_REG, 2, &data);
    data &= ~INT_TX_SPACE;
    ETHSPI_WriteRegister(INT_STATUS_REG, 2, &data);
  }

  /* Disable interupts on KSZ8851SNL */
  data = NO_INT;
  ETHSPI_WriteRegister(INT_ENABLE_REG, 2, &data);

  /* Enable TXQ write access */
  ETHSPI_ReadRegister(RXQ_CMD_REG, 2, &data);
  data |= RXQ_START;
  ETHSPI_WriteRegister(RXQ_CMD_REG, 2, &data);

  /* Write frame ID, control word and byte count */
  outbuf[0] = (frameId++ & frameId_MASK) | TX_INT_on_COMPLETION;
  outbuf[1] = 0;
  outbuf[2] = pTXLength & LSB_MASK;
  outbuf[3] = pTXLength >> MSB_POS;

  /* Start the SPI Transfer */
  ETHSPI_StartWriteFIFO();
  /* Send the frame header info */
  ETHSPI_WriteFifoContinue(4, outbuf);
}


/***************************************************************************//**
 * @brief Performs the actual transmission of a long raw frame over the network.
 *
 * @param pTXLength
 *     the length of the transmitted frame
 * @param pTXData
 *     the data of the transmitted frame
 *****************************************************************************/
void KSZ8851SNL_LongTransmit(uint16_t pTXLength, uint8_t *pTXData)
{
  EFM_ASSERT(pTXData != NULL);
  /* Send the actual data */
  ETHSPI_WriteFifoContinue(pTXLength, pTXData);
}


/***************************************************************************//**
 * @brief Performs the clean up procedures after the transmission of a long raw
 *        frame over the network.
 *
 * @param pTXLength
 *     the length of the transmitted frame
 * @param pTXData
 *     the data of the transmitted frame
 *****************************************************************************/
void KSZ8851SNL_TerminateLongTransmit(uint16_t pTXLength, uint8_t *pTXData)
{
  EFM_ASSERT(pTXData != NULL);

  uint16_t data;

  /* Send dummy bytes to align data to DWORD */
  ETHSPI_WriteFifoContinue(KSZ8851SNL_DwordAllignDiff(pTXLength), pTXData);

  /* Stop the SPI Transfer */
  ETHSPI_StopFIFO();

  /* Disable TXQ write access */
  ETHSPI_ReadRegister(RXQ_CMD_REG, 2, &data);
  data &= ~RXQ_START;
  ETHSPI_WriteRegister(RXQ_CMD_REG, 2, &data);

  /* Start TXQ Manual Engue */
  ETHSPI_ReadRegister(TXQ_CMD_REG, 2, &data);
  data |= TXQ_ENQUEUE;
  ETHSPI_WriteRegister(TXQ_CMD_REG, 2, &data);

  /* Wait until transmit command clears */
  while (1)
  {
    ETHSPI_ReadRegister(TXQ_CMD_REG, 2, &data);
    if (!(data & TXQ_ENQUEUE))
      break;
  }

  /* Enable interupts */
  data = INT_MASK_EXAMPLE;
  ETHSPI_WriteRegister(INT_ENABLE_REG, 2, &data);
}


/***************************************************************************//**
 * @brief Realease the current frame if it is inconsistent.
 * @note  Support method used for minimizing the code size.
 *****************************************************************************/
static void KSZ8851SNL_ReleaseIncosistentFrame(void)
{
  uint16_t data;
  /* Issue the Release error frame command */
  ETHSPI_ReadRegister(RXQ_CMD_REG, 2, &data);
  data |= RXQ_RELEASE_CUR_FR;
  ETHSPI_WriteRegister(RXQ_CMD_REG, 2, &data);
  KSZ8851SNL_ExceptionHandler(INFO, "The frame was inconsistent\n");
}


#if KSZ8851SNL_DEBUG
/***************************************************************************//**
 * @brief Initialize the loopback mode of the ethernet controller.
 * @note  Support method used for minimizing the code size.
 *****************************************************************************/
static void KSZ8851SNL_SetDigitalLoopbackMode(void)
{
  uint16_t data;
  /* Reset PHY. */
  data = PHY_RESET;
  ETHSPI_WriteRegister(PHY_RST_REG, 2, &data);
  /* Disable Auto-negotiation.1. Reset PHY. */
  /* Set Speed to either 100Base-TX or 10Base-T. */
  /* Set Duplex to full-duplex. */
  /* Set PHY register 0.14 to �1� to enable Local Loop-back. */
  data  = DIGITAL_LOOPBACK | FORCE_FULL_DUPLEX | FORCE_100;
  data &= ~AUTO_NEG;
  ETHSPI_WriteRegister(PHY1_CTRL_REG, 2, &data);

  KSZ8851SNL_ExceptionHandler(INFO, "Loopback mode initiated");
}
#endif /* KSZ8851SNL_DEBUG */


/***************************************************************************//**
 * @brief Performs the actual receive of a raw frame over the network.
 *
 * @param pRXLength
 *     the length of the received frame
 * @param pRXData
 *     the data of the received frame
 *
 * @return
 *     received packet length, 0 in case of failure
 *****************************************************************************/
uint16_t KSZ8851SNL_Receive(uint8_t *pRXData, uint16_t *pRXLength)
{
  uint16_t data;
  uint8_t  *pDummy = pRXData;
  uint16_t rxFrameCount, rxftr;
  uint16_t rxStatus;

  EFM_ASSERT(pRXData != NULL);
  EFM_ASSERT(pRXLength != NULL);

  /* Read the frame count and threshold register */
  ETHSPI_ReadRegister(RX_FRAME_THRES_REG, 2, &rxftr);
  /* Extract the actual number of frames from RX_FRAME_THRES_REG*/
  rxFrameCount = rxftr >> MSB_POS;

  while (rxFrameCount > 0)
  {
    /* Read the received frame status */
    ETHSPI_ReadRegister(RX_FRH_STAT_REG, 2, &rxStatus);

    /* Check the consistency of the frame */
    if (!(rxStatus >> RECEIVED_FRAME_VALID_POS == 0) || ((rxStatus & RECEIVE_VALID_FRAME_MASK) == RECEIVE_VALID_FRAME_MASK))
    {
      /* Issue the Release error frame command */
      KSZ8851SNL_ReleaseIncosistentFrame();
    }
    else
    {
      /* Read the byte size of the received frame */
      ETHSPI_ReadRegister(RX_FRH_BC_REG, 2, pRXLength);

      *pRXLength &= RX_BYTE_CNT_MASK;

      if (*pRXLength <= 0)
      {
        /* Issue the Release error frame command */
        KSZ8851SNL_ReleaseIncosistentFrame();
        /* Discard the frame*/
        rxFrameCount--;
        /* continue to next frame */
        continue;
      }

      /* Reset QMU RXQ frame pointer to zero */
      data = FD_PTR_AUTO_INC;
      ETHSPI_WriteRegister(RX_FD_PTR_REG, 2, &data);

      /* Start QMU DMA transfer */
      ETHSPI_ReadRegister(RXQ_CMD_REG, 2, &data);
      data |= RXQ_START;
      ETHSPI_WriteRegister(RXQ_CMD_REG, 2, &data);

      /* Start the SPI transfer */
      ETHSPI_StartReadFIFO();

      /* Read 4 dummy, 2 status word and 2 bytecount bytes of the frame header */
      ETHSPI_ReadFifoContinue(4, pDummy);
      ETHSPI_ReadFifoContinue(2, pDummy);
      ETHSPI_ReadFifoContinue(2, pDummy);

      /* Remove the first 2 extra bytes consisting of the 2 bytes offset*/
      ETHSPI_ReadFifoContinue(2, pDummy);
      *pRXLength -= 2;

      /* Read the received data */
      ETHSPI_ReadFifoContinue(*pRXLength, pRXData);

      /* Stoping the SPI transfer */
      ETHSPI_StopFIFO();

      /* Stop QMU DMA transfer */
      ETHSPI_ReadRegister(RXQ_CMD_REG, 2, &data);
      data &= ~RXQ_START;
      ETHSPI_WriteRegister(RXQ_CMD_REG, 2, &data);

      /* Enable interupts */
      data = INT_MASK_EXAMPLE;
      ETHSPI_WriteRegister(INT_ENABLE_REG, 2, &data);

      /* Remove the last 4 extra bytes consisting of the CRC field*/
      *pRXLength -= 4;

      /* After reading one frame, we return its length.
       *   Can also be implemented as callback function
       *   but it is not the purpose of lwIP implementation
       */
      return *pRXLength;
    }

    /* Finished reading one frame */
    rxFrameCount--;

    /* Enable interupts */
    data = INT_MASK_EXAMPLE;
    ETHSPI_WriteRegister(INT_ENABLE_REG, 2, &data);
  }
  return 0;
}


/***************************************************************************//**
 * @brief Get the MAC address of the current board.
 * @note  Support method used for minimizing the code size.
 * @param[out] macAddress
 *     data buffer to store the macAddress
 *****************************************************************************/
void KSZ8851SNL_GetMacAddress(uint8_t *macAddress)
{
  /* TODO:  Get MAC based on actual MAC and not on the CMU unique ID. */

  EFM_ASSERT(macAddress != NULL);

  /* set the first 3 bytes given by the EM MAC Address space */
  macAddress[0] = HIGH_QMU_MAC_H;
  macAddress[1] = HIGH_QMU_MAC_L;
  macAddress[2] = MID_QMU_MAC_H;
  /* set the next 3 bytes given by the CMU unique ID */
  int i;
  for (i = 0; i < 3; i++)
  {
    macAddress[5 - i] = (DEVINFO->UNIQUEL & (BYTE_MASK << i * BYTE_SIZE)) >> i * BYTE_SIZE;
  }
}

