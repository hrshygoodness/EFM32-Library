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


#ifndef _ksz8851snl_H_
#define _ksz8851snl_H_

/**************************************************************************//**
* @addtogroup Drivers
* @{
******************************************************************************/

/**************************************************************************//**
* @addtogroup ksz8851snl
* @{
* The low level communication between EFM32 chips and the ethernet controller Micrel
* KSZ8851 SNL is realized by the functions from this file.
* The file contains all the necesarry functions to be
* used as a stand alone program to send/receive raw ethernet frames or can be
* incorporated into a higher layer to be used with an IP stack.
******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** DEBUG macros */
#define KSZ8851SNL_DEBUG    0
#if KSZ8851SNL_DEBUG
/** DEBUG macro for printf function */
#define DEBUG_PRINT(...)    printf(__VA_ARGS__)
#define DIGITAL_PHY_LOOPBACK
#define DUMP_REGS
#else
/** DEBUG macro for printf function */
#define DEBUG_PRINT(...)    (void)0
#endif /* KSZ8851SNL_DEBUG */

/* Register definitions */
#define LOW_QMU_MAC_REG             0x10 /**< MAC Address Low */
#define MID_QMU_MAC_REG             0x12 /**< MAC Address Middle*/
#define HIGH_QMU_MAC_REG            0x14 /**< MAC Address High*/
#define OBC_REG                     0x20 /**< On-Chip Bus Control Register */
#define GLOBAL_RESET_REG            0x26 /**< Global Reset Register */
#define TX_FLOW_CTRL_REG            0x70 /**< Transmit Flow Control Register */
#define RX_FLOW_CTRL1_REG           0x74 /**< Receive Flow Control Register 1 */
#define RX_FLOW_CTRL2_REG           0x76 /**< Receive Flow Control Register 2 */
#define TX_MEM_INFO_REG             0x78 /**< TXQ Memory Information Register */
#define RX_FRH_STAT_REG             0x7C /**< Receive Frame Header Status Register */
#define RX_FRH_BC_REG               0x7E /**< Receive Frame Header Bytecount Register */
#define TXQ_CMD_REG                 0x80 /**< TXQ Command Register */
#define RXQ_CMD_REG                 0x82 /**< RXQ Command Register */
#define TX_FD_PTR_REG               0x84 /**< TX Frame Data Pointer Register */
#define RX_FD_PTR_REG               0x86 /**< RX Frame Data Pointer Register */
#define INT_ENABLE_REG              0x90 /**< Interrupt Enable Register */
#define INT_STATUS_REG              0x92 /**< Interrupt Status Register */
#define RX_FRAME_THRES_REG          0x9C /**< RX Frame Count & Threshold Register */
#define TX_NEXT_FRS_REG             0x9E /**< TX Next Frame size register */
#define FLOW_CTRL_LOW_WATERMARK     0xB0 /**< Configure Low Watermark to 6KByte */
#define FLOW_CTRL_HIGH_WATERMARK    0xB2 /**< Configure High Watermark to 4KByte */
#define CIDER_REG                   0xC0 /**< Chip ID and Enable Register */
#define IND_ACC_CTRL_REG            0xC8 /**< Indirect access control Register */
#define IND_ACC_DATA_LOW_REG        0xD0 /**< Indirect access data low Register */
#define IND_ACC_DATA_HIGH_REG       0xD2 /**< Indirect access data low Register */
#define PHY_RST_REG                 0xD8 /**< PHY Reset Register  */
#define PHY1_CTRL_REG               0xE4 /**< PHY1 MII-Register Basic Control Register */
#define PORT1_CTRL_REG              0xF6 /**< Port 1 Control Register */

/* Magic numbers */
#define KSZ8851SNL_CHIP_ID          0x8870 /**< Default Chip ID for KSZ8851SNL */
#define CHIP_ID_MASK                0xFFF0 /**< Used to mask the revision ID */
#define ONE_FRAME_THRES             0x0001 /**< RX INT after one frame */
#define FD_PTR_AUTO_INC             0x4000 /**< Used to reset the FD pointer */
#define CLEAR_INT                   0xFFFF /**< Used to clear INT_STATUS_REG */
#define NO_INT                      0x0000 /**< Used to disable the interupts */
#define TX_MEM_AVAIL_MASK           0x1FFF /**< Used to mask the reserved bits */
#define frameId_MASK                0x003F /**< Used to mask the reserved bits */
#define RECEIVE_VALID_FRAME_MASK    0x3C17 /**< CRC OK for ICMP, IP, TCP, UDP; /
                                          *   MII error; /
                                          *   Frame too long error */
#define RECEIVED_FRAME_VALID_POS    0x0010 /**< Received valid frame byte pos */
#define RX_BYTE_CNT_MASK            0x0FFF /**< Used to mask the reserved bits */
#define LSB_MASK                    0x00FF /**< Used to mask the LSB */
#define MSB_POS                     0x0008 /**< Used to mark the MSB pos */
#define TX_INT_on_COMPLETION        0x8000 /**< TX INT on completion */
#define WORD_SIZE                   0x0004 /**< Word size in # of bytes */
#define EXTRA_SIZE                  0x0008 /**< Needed for the frame header */
#define BLOCKING_RECEIVE            0      /**< Determines if receive will block */
#define WATERMARK_6KB               0x0600 /**< 6KByte Watermark */
#define WATERMARK_4KB               0x0400 /**< 4KByte Watermark */
/* Energy Micro's MAC address space */
#define HIGH_QMU_MAC_H              0xD0   /**< 1st segment of the MAC address */
#define HIGH_QMU_MAC_L              0xCF   /**< 2nd segment of the MAC address */
#define MID_QMU_MAC_H               0x5E   /**< 3rd segment of the MAC address */
#define MID_QMU_MAC_L               0x00   /**< 4th segment of the MAC address */
#define LOW_QMU_MAC_H               0x00   /**< 5th segment of the MAC address */
#define LOW_QMU_MAC_L               0x00   /**< 6th segment of the MAC address */
#define BYTE_MASK                   0x00FF /**< Used to mask the LSB */
#define BYTE_SIZE                   0x0008 /**< Used to mark the MSB pos */

/* TX Flow Control Register Options */

/** Enable Transmit Checksum Generation for ICMP */
#define   TX_FLOW_CTRL_ICMP_CHECKSUM    0x0100
/** Enable Transmit Checksum Generation for UDP */
#define   TX_FLOW_CTRL_UDP_CHECKSUM     0x0080
/** Enable Transmit Checksum Generation for TCP */
#define   TX_FLOW_CTRL_TCP_CHECKSUM     0x0040
/** Enable Transmit Checksum Generation for IP */
#define   TX_FLOW_CTRL_IP_CHECKSUM      0x0020
/** Flush Transmit Queue */
#define   TX_FLOW_CTRL_FLUSH_QUEUE      0x0010
/** Transmit flow control enable*/
#define   TX_FLOW_CTRL_FLOW_ENABLE      0x0008
/** Transmit Padding enable */
#define   TX_FLOW_CTRL_PAD_ENABLE       0x0004
/** Transmit CRC Enable */
#define   TX_FLOW_CTRL_CRC_ENABLE       0x0002
/** Enable tranmsit */
#define   TX_FLOW_CTRL_ENABLE           0x0001

/** TX FLOW CONTROL Initialization collection */
#define   TX_FLOW_CTRL_EXAMPLE          (TX_FLOW_CTRL_ICMP_CHECKSUM | \
                                         TX_FLOW_CTRL_UDP_CHECKSUM |  \
                                         TX_FLOW_CTRL_TCP_CHECKSUM |  \
                                         TX_FLOW_CTRL_IP_CHECKSUM |   \
                                         TX_FLOW_CTRL_FLOW_ENABLE |   \
                                         TX_FLOW_CTRL_PAD_ENABLE |    \
                                         TX_FLOW_CTRL_CRC_ENABLE)

/* TXQ Command Register Options */
/** Enable Auto-Enqueue TXQ Frame */
#define   TXQ_AUTO_ENQUEUE         0x0004
/** Enable INT generation when TXQ Memory Available */
#define   TXQ_MEM_AVAILABLE_INT    0x0002
/** Enable Manual Engueue TXQ Frame */
#define   TXQ_ENQUEUE              0x0001


/* RX Flow Control Register 1 Options */
/** Flush Receive Queue */
#define   RX_FLOW_CTRL_FLUSH_QUEUE       0x8000
/** Enable Receive UDP Frame Checksum Check */
#define   RX_FLOW_CTRL_UDP_CHECKSUM      0x4000
/** Enable Receive TCP Frame Checksum Check */
#define   RX_FLOW_CTRL_TCP_CHECKSUM      0x2000
/** Enable Receive IP Frame Checksum Check */
#define   RX_FLOW_CTRL_IP_CHECKSUM       0x1000
/** Receive Physical Address Filtering with MAC Address Enable */
#define   RX_FLOW_CTRL_MAC_FILTER        0x0800
/** Enable Receive Flow Control */
#define   RX_FLOW_CTRL_FLOW_ENABLE       0x0400
/** Enable Receive Error Frames */
#define   RX_FLOW_CTRL_BAD_PACKET        0x0200
/** Receive Multicast Address Filtering with MAC Address Enable */
#define   RX_FLOW_CTRL_MULTICAST         0x0100
/** Enable Receive Broadcast frames */
#define   RX_FLOW_CTRL_BROADCAST         0x0080
/** Enable Receive Multicast frames */
#define   RX_FLOW_CTRL_ALL_MULTICAST     0x0040
/** Enable Receive Unicast frames */
#define   RX_FLOW_CTRL_UNICAST           0x0020
/** Receive all incoming frames */
#define   RX_FLOW_CTRL_PROMISCUOUS       0x0012
/** Receive Inverse Filtering */
#define   RX_FLOW_CTRL_INVERSE_FILTER    0x0002
/** Enable receive */
#define   RX_FLOW_CTRL_ENABLE            0x0001

/** RX FLOW CONTROL1 Initialization collection */
#define   RX_FLOW_CTRL1_EXAMPLE          (RX_FLOW_CTRL_UDP_CHECKSUM |  \
                                          RX_FLOW_CTRL_TCP_CHECKSUM |  \
                                          RX_FLOW_CTRL_IP_CHECKSUM |   \
                                          RX_FLOW_CTRL_MAC_FILTER |    \
                                          RX_FLOW_CTRL_FLOW_ENABLE |   \
                                          RX_FLOW_CTRL_BROADCAST |     \
                                          RX_FLOW_CTRL_ALL_MULTICAST | \
                                          RX_FLOW_CTRL_UNICAST)

/* RX Flow Control Register 2 Options */
/* SPI Receive Data Burst Length */
/** Receive Flow Control Burst Length mask */
#define   RX_FLOW_CTRL_BURST_LEN_MASK        0x00E0
/** 4 bytes length */
#define   RX_FLOW_CTRL_BURST_LEN_4           0x0000
/** 8 Bytes length */
#define   RX_FLOW_CTRL_BURST_LEN_8           0x0020
/** 16 Bytes length */
#define   RX_FLOW_CTRL_BURST_LEN_16          0x0040
/** 32 Bytes length */
#define   RX_FLOW_CTRL_BURST_LEN_32          0x0060
/** Full frame length */
#define   RX_FLOW_CTRL_BURST_LEN_FRAME       0x0080
/** IPV4/IPV6/UDP Fragment Frame Pass */
#define   RX_FLOW_CTRL_IPV6_UDP_FRAG_PASS    0x0010
/** IPV4/IPV6/UDP Frame Checksum Equal Zero */
#define   RX_FLOW_CTRL_IPV6_UDP_ZERO_PASS    0x0008
/** Enable UDP Lite frame */
#define   RX_FLOW_CTRL_UDP_LITE_CHECKSUM     0x0004
/** Enable ICMP frame */
#define   RX_FLOW_CTRL_ICMP_CHECKSUM         0x0002
/** Receive Source Address Filtering */
#define   RX_FLOW_CTRL_BLOCK_MAC             0x0001

/** RX FLOW CONTROL2 Initialization collection */
#define   RX_FLOW_CTRL2_EXAMPLE              (RX_FLOW_CTRL_IPV6_UDP_FRAG_PASS | \
                                              RX_FLOW_CTRL_UDP_LITE_CHECKSUM |  \
                                              RX_FLOW_CTRL_ICMP_CHECKSUM |      \
                                              RX_FLOW_CTRL_BURST_LEN_FRAME)

/* RXQ Command Register Options */
/** RX interrupt is occured on timer duration */
#define   RXQ_ON_TIME_INT            0x1000
/** RX interrupt is occured on byte count threshold */
#define   RXQ_ON_BYTE_CNT_INT        0x0800
/** RX interrupt is occured on frame count threshold */
#define   RXQ_ON_FRAME_CNT_INT       0x0400
/** Enable adding 2-bytes offset before IP frame header */
#define   RXQ_TWOBYTE_OFFSET         0x0200
/** Enable RX interrupt on timer duration */
#define   RXQ_EN_ON_TIME_INT         0x0080
/** Enable RX interrupt on byte count threshold */
#define   RXQ_EN_ON_BYTE_CNT_INT     0x0040
/** Enable RX interrupt on frame count threshold */
#define   RXQ_EN_ON_FRAME_CNT_INT    0x0020
/** Enable Auto Dequeue RXQ Frame */
#define   RXQ_AUTO_DEQUEUE           0x0010
/** Start QMU transfer operation */
#define   RXQ_START                  0x0008
/** Release RX Error Frame */
#define   RXQ_RELEASE_CUR_FR         0x0001

/** RX COMMAND Initialization collection */
#define   RXQ_CMD_EXAMPLE            (RXQ_EN_ON_FRAME_CNT_INT | \
                                      RXQ_TWOBYTE_OFFSET |      \
                                      RXQ_AUTO_DEQUEUE)

/* Port 1 Control Register Options */
/** Turn off port LEDs */
#define   PORT1_LED_OFF               0x8000
/** Disable port transmit */
#define   PORT1_TX_DISABLE            0x4000
/** Restart auto-negotiation */
#define   PORT1_AUTO_NEG_RESTART      0x2000
/** Set port power-down */
#define   PORT1_POWER_DOWN            0x0800
/** Disable auto MDI/MDI-X */
#define   PORT1_AUTO_MDIX_DISABLE     0x0400
/** Force MDI-X */
#define   PORT1_FORCE_MDIX            0x0200
/** Enable auto-negotiation */
#define   PORT1_AUTO_NEG_ENABLE       0x0080
/** Force PHY 100Mbps */
#define   PORT1_FORCE_100_MBIT        0x0040
/** Force PHY in full duplex mode */
#define   PORT1_FORCE_FULL_DUPLEX     0x0020
/** Advertise flow control capability */
#define   PORT1_AUTO_NEG_SYM_PAUSE    0x0010
/** Advertise 100BT full-duplex capability */
#define   PORT1_AUTO_NEG_100BTX_FD    0x0008
/** Advertise 100BT half-duplex capability */
#define   PORT1_AUTO_NEG_100BTX       0x0004
/** Advertise 10BT full-duplex capability  */
#define   PORT1_AUTO_NEG_10BT_FD      0x0002
/** Advertise 10BT half-duplex capability  */
#define   PORT1_AUTO_NEG_10BT         0x0001

/* Interrupt Enable Register Options */
/** Enable link change interrupt */
#define   INT_LINK_CHANGE     0x8000
/** Enable transmit done interrupt */
#define   INT_TX_DONE         0x4000
/** Enable receive interrupt */
#define   INT_RX_DONE         0x2000
/** Enable receive overrun interrupt */
#define   INT_RX_OVERRUN      0x0800
/** Enable transmit process stopped interrupt */
#define   INT_TX_STOPPED      0x0200
/** Enable receive process stopped interrupt */
#define   INT_RX_STOPPED      0x0100
/** Enable transmit space available interrupt */
#define   INT_TX_SPACE        0x0040
/** Enable WOL on receive wake-up frame detect interrupt */
#define   INT_RX_WOL_FRAME    0x0020
/** Enable magic packet detect interrupt */
#define   INT_MAGIC           0x0010
/** Enable link up detect interrupt */
#define   INT_LINKUP          0x0008
/** Enable detect interrupt */
#define   INT_ENERGY          0x0004
/** Enable receive SPI bus error interrupt */
#define   INT_SPI_ERROR       0x0002

/** Interrupt mask initialization collection */
#define   INT_MASK_EXAMPLE    (INT_RX_DONE |    \
                               INT_RX_OVERRUN | \
                               INT_TX_STOPPED | \
                               INT_RX_STOPPED | \
                               INT_TX_DONE |    \
                               INT_LINK_CHANGE)

/* Global Reset Register Options */
/** QMU Reset */
#define QMU_MODULE_SOFT_RESET    0x0002
/** Global reset */
#define GLOBAL_SOFT_RESET        0x0001

/** PHY Reset Register Options */
#define PHY_RESET                0x0001

/* Global Reset Register Options */
/** Enable Digital loopback mode */
#define DIGITAL_LOOPBACK            0x4000
/** Force the speed to 100MBps */
#define FORCE_100                   0x2000
/** Force auto negotiation */
#define AUTO_NEG                    0x1000
/** Restart auto negotiation */
#define RESTART_AUTO_NEG            0x0200
/** Force full duplex */
#define FORCE_FULL_DUPLEX           0x0100

/* Management information base registers */
#define MIB_MASK                    0x1C00       /**< MIB Mask */
#define MIB_RxByte                  0x00         /**< # of received bytes */
#define MIB_XXX                     0x01         /**< MIB Reserved byte */
#define MIB_RxUndersizePkt          0x02         /**< # of received undersized packets */
#define MIB_RxFragments             0x03         /**< # of received fragments */
#define MIB_RxOversize              0x04         /**< # of received oversized packets */
#define MIB_RxJabbers               0x05         /**< # of received jabbers */
#define MIB_RxSynbolError           0x06         /**< # of received error symbols */
#define MIB_RxCRCError              0x07         /**< # of received packets with CRC error */
#define MIB_RxAlignmentError        0x08         /**< # of received missaligned packets */
#define MIB_RxControl8808Pkts       0x09         /**< # of received control packets */
#define MIB_RxPausePkts             0x0A         /**< # of received pause packets */
#define MIB_RxBroadcast             0x0B         /**< # of received broadcast packets */
#define MIB_RxMulticast             0x0C         /**< # of received multicast packets */
#define MIB_RxUnicast               0x0D         /**< # of received unicast packets */
#define MIB_Rx64Octets              0x0E         /**< # of received packets with size of 64 bytes */
#define MIB_Rx65to127Octets         0x0F         /**< # of received packets with size between 65 and 127 bytes */
#define MIB_Rx128to255Octets        0x10         /**< # of received packets with size between 128 and 255 bytes */
#define MIB_Rx256to511Octets        0x11         /**< # of received packets with size between 256 and 511 bytes */
#define MIB_Rx512to1023Octets       0x12         /**< # of received packets with size between 512 and 1023 bytes */
#define MIB_Rx1024to1521Octets      0x13         /**< # of received packets with size between 1024 and 1521 bytes */
#define MIB_Rx1522to2000Octets      0x14         /**< # of received packets with size between 1522 and 2000 bytes */
#define MIB_TxByte                  0x15         /**< # of transmitted bytes */
#define MIB_TxLateCollision         0x16         /**< # of transmitted late collision packets */
#define MIB_TxPausePkts             0x17         /**< # of transmitted pause packets */
#define MIB_TxBroadcastPkts         0x18         /**< # of transmitted broadcast packets */
#define MIB_TxMulticastPkts         0x19         /**< # of transmitted multicast packets */
#define MIB_TxUnicastPkts           0x1A         /**< # of transmitted unicast packets */
#define MIB_TxDeferred              0x1B         /**< # of transmitted deferred packets */
#define MIB_TxTotalCollision        0x1C         /**< # of transmitted total collisions */
#define MIB_TxExcessiveCollision    0x1D         /**< # of transmitted excessive collisions */
#define MIB_TxSingleCollision       0x1E         /**< # of transmitted single collisions */
#define MIB_TxMultipleCollision     0x1F         /**< # of transmitted multiple collisions */

/** enumeration used for exception handling */
enum exceptionType_e
{
  ERROR,
  INFO
};

void     KSZ8851SNL_Init(void);
void     KSZ8851SNL_Send(uint16_t packetLength, uint8_t *packetData);
uint16_t KSZ8851SNL_Receive(uint8_t *pRXData, uint16_t *pRXLength);
void     KSZ8851SNL_GetMacAddress(uint8_t *macAddress);
void     KSZ8851SNL_ReadMIBCounters(char* param);
uint16_t KSZ8851SNL_CheckIrqStat(void);
uint16_t KSZ8851SNL_CurrFrameSize(void);
void     KSZ8851SNL_TerminateLongTransmit(uint16_t pTXLength, uint8_t *pTXData);
void     KSZ8851SNL_InitiateLongTransmit(uint16_t pTXLength);
void     KSZ8851SNL_LongTransmit(uint16_t pTXLength, uint8_t *pTXData);
void     KSZ8851SNL_EnableInterupts(void);


#ifdef __cplusplus
}
#endif

/** @} (end group ksz8851snl) */
/** @} (end group Drivers) */

#endif
