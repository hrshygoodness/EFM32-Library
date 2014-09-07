/*----------------------------------------------------------------------------
*      AX88796 Register Definitions
*----------------------------------------------------------------------------
*      Name:    Ax88796.h
*      Purpose: ASIX AX88796 hardware definitions
*----------------------------------------------------------------------------
*      Copyright (c) 2003 Keil Software. All rights reserved.
*---------------------------------------------------------------------------*/

#ifndef __AX88796C_H
#define __AX88796C_H

#include "lwip/netif.h"

#define TX_OVERHEAD           8
#define TX_EOP_SIZE           4

/* Speed Duplex Mode */
#define SPEED_DUPLEX_AUTO     0x05E1
#define SPEED_DUPLEX_100FD    0x0501
#define SPEED_DUPLEX_100HD    0x0081
#define SPEED_DUPLEX_10FD     0x0441
#define SPEED_DUPLEX_10HD     0x0021

#define RESTART_AUTONEG       0x3300


/* Definition of SPI command */
#define AX_SPICMD_WRITE_TXQ      0x02
#define AX_SPICMD_READ_REG       0x03
#define AX_SPICMD_READ_STATUS    0x05
#define AX_SPICMD_READ_RXQ       0x0B
#define AX_SPICMD_BIDIR_WRQ      0xB2
#define AX_SPICMD_WRITE_REG      0xD8
#define AX_SPICMD_EXIT_PWD       0xAB
#define AX_SPICMD_RESET_SPI      0xFF


/* AX88796C Pages index */
#define PAGE0    0x00
#define PAGE1    0x01
#define PAGE2    0x02
#define PAGE3    0x03
#define PAGE4    0x04
#define PAGE5    0x05
#define PAGE6    0x06
#define PAGE7    0x07


/* A88796C register definition */
#define AX_SHIFT(x)    ((x) << 0)

/* A88796C register definition */
/* Definition of PAGE0 */
#define P0_PSR                        (0x00)
    #define PSR_DEV_READY             (1 << 7)
    #define PSR_RESET                 (0 << 15)
    #define PSR_RESET_CLR             (1 << 15)
#define P0_BOR                        (0x02)
#define P0_FER                        (0x04)
    #define FER_IPALM                 (1 << 0)
    #define FER_DCRC                  (1 << 1)
    #define FER_RH3M                  (1 << 2)
    #define FER_HEADERSWAP            (1 << 7)
    #define FER_WSWAP                 (1 << 8)
    #define FER_BSWAP                 (1 << 9)
    #define FER_INTHI                 (1 << 10)
    #define FER_INTLO                 (0 << 10)
    #define FER_IRQ_PULL              (1 << 11)
    #define FER_RXEN                  (1 << 14)
    #define FER_TXEN                  (1 << 15)
#define P0_ISR                        (0x06)
    #define ISR_RXPKT                 (1 << 0)
    #define ISR_MDQ                   (1 << 4)
    #define ISR_TXT                   (1 << 5)
    #define ISR_TXPAGES               (1 << 6)
    #define ISR_TXERR                 (1 << 8)
    #define ISR_LINK                  (1 << 9)
#define P0_IMR                        (0x08)
    #define IMR_RXPKT                 (1 << 0)
    #define IMR_MDQ                   (1 << 4)
    #define IMR_TXT                   (1 << 5)
    #define IMR_TXPAGES               (1 << 6)
    #define IMR_TXERR                 (1 << 8)
    #define IMR_LINK                  (1 << 9)
    #define IMR_MASKALL               (0xFFFF)
    #define IMR_DEFAULT               (IMR_TXERR)
#define P0_WFCR                       (0x0A)
    #define WFCR_PMEIND               (1 << 0)  /* PME indication */
    #define WFCR_PMETYPE              (1 << 1)  /* PME I/O type */
    #define WFCR_PMEPOL               (1 << 2)  /* PME polarity */
    #define WFCR_PMERST               (1 << 3)  /* Reset PME */
    #define WFCR_SLEEP                (1 << 4)  /* Enable sleep mode */
    #define WFCR_WAKEUP               (1 << 5)  /* Enable wakeup mode */
    #define WFCR_WAITEVENT            (1 << 6)  /* Reserved */
    #define WFCR_CLRWAKE              (1 << 7)  /* Clear wakeup */
    #define WFCR_LINKCH               (1 << 8)  /* Enable link change */
    #define WFCR_MAGICP               (1 << 9)  /* Enable magic packet */
    #define WFCR_WAKEF                (1 << 10) /* Enable wakeup frame */
    #define WFCR_PMEEN                (1 << 11) /* Enable PME pin */
    #define WFCR_LINKCHS              (1 << 12) /* Link change status */
    #define WFCR_MAGICPS              (1 << 13) /* Magic packet status */
    #define WFCR_WAKEFS               (1 << 14) /* Wakeup frame status */
    #define WFCR_PMES                 (1 << 15) /* PME pin status */
#define P0_PSCR                       (0x0C)
    #define PSCR_PS_MASK              (0xFFF0)
    #define PSCR_PS_D0                (0)
    #define PSCR_PS_D1                (1 << 0)
    #define PSCR_PS_D2                (1 << 1)
    #define PSCR_FPS                  (1 << 3)  /* Enable fiber mode PS */
    #define PSCR_SWPS                 (1 << 4)  /* Enable software PS control */
    #define PSCR_WOLPS                (1 << 5)  /* Enable WOL PS */
    #define PSCR_SWWOL                (1 << 6)  /* Enable software select WOL PS */
    #define PSCR_PHYOSC               (1 << 7)  /* Internal PHY OSC control */
    #define PSCR_FOFEF                (1 << 8)  /* Force PHY generate FEF */
    #define PSCR_FOF                  (1 << 9)  /* Force PHY in fiber mode */
    #define PSCR_PHYPD                (1 << 10) /* PHY power down. Active high */
    #define PSCR_PHYRST               (1 << 11) /* PHY reset signal. Active low */
    #define PSCR_PHYCSIL              (1 << 12) /* PHY cable energy detect */
    #define PSCR_PHYCOFF              (1 << 13) /* PHY cable off */
    #define PSCR_PHYLINK              (1 << 14) /* PHY link status */
    #define PSCR_EEPOK                (1 << 15) /* EEPROM load complete */
#define P0_MACCR                      (0x0E)
    #define MACCR_RXEN                (1 << 0)
    #define MACCR_DUPLEXFULL          (1 << 1)
    #define MACCR_SPEED100            (1 << 2)
    #define MACCR_RXFC_ENABLE         (1 << 3)
    #define MACCR_RXFC_MASK           0xFFF7
    #define MACCR_TXFC_ENABLE         (1 << 4)
    #define MACCR_TXFC_MASK           0xFFEF
    #define MACCR_PF                  (1 << 7)
    #define MACCR_PMM_BITS            8
    #define MACCR_PMM_MASK            (0x1F00)
    #define MACCR_PMM_RESET           (1 << 8)
    #define MACCR_PMM_WAIT            (2 << 8)
    #define MACCR_PMM_READY           (3 << 8)
    #define MACCR_PMM_D1              (4 << 8)
    #define MACCR_PMM_D2              (5 << 8)
    #define MACCR_PMM_WAKE            (7 << 8)
    #define MACCR_PMM_D1_WAKE         (8 << 8)
    #define MACCR_PMM_D2_WAKE         (9 << 8)
    #define MACCR_PMM_SLEEP           (10 << 8)
    #define MACCR_PMM_PHY_RESET       (11 << 8)
    #define MACCR_PMM_SOFT_D1         (16 << 8)
    #define MACCR_PMM_SOFT_D2         (17 << 8)

#define P0_TFBFCR                     (0x10)
    #define TFBFCR_SCHE_FREE_PAGE     0xE07F
    #define TFBFCR_FREE_PAGE_BITS     0x07
    #define TFBFCR_FREE_PAGE_LATCH    (1 << 6)
    #define TFBFCR_SET_FREE_PAGE(x)    ((x & 0x3F) << TFBFCR_FREE_PAGE_BITS)
    #define TFBFCR_TX_PAGE_SET        (1 << 13)
    #define TFBFCR_MANU_ENTX          (1 << 15)
    #define TX_FREEBUF_MASK           0x003F
    #define TX_DPTSTART               0x4000

#define P0_TSNR                       (0x12)
    #define TXNR_TXB_ERR              (1 << 5)
    #define TXNR_TXB_IDLE             (1 << 6)
    #define TSNR_PKT_CNT(x)    (((x) & 0x3F) << 8)
    #define TXNR_TXB_REINIT           (1 << 14)
    #define TSNR_TXB_START            (1 << 15)
#define P0_RTDPR                      (0x14)
#define P0_RXBCR1                     (0x16)
    #define RXBCR1_RXB_DISCARD        (1 << 14)
    #define RXBCR1_RXB_START          (1 << 15)
#define P0_RXBCR2                     (0x18)
    #define RXBCR2_PKT_MASK           (0xFF)
    #define RXBCR2_RXPC_MASK          (0x7F)
    #define RXBCR2_RXB_READY          (1 << 13)
    #define RXBCR2_RXB_IDLE           (1 << 14)
    #define RXBCR2_RXB_REINIT         (1 << 15)
#define P0_RTWCR                      (0x1A)
    #define RTWCR_RXWC_MASK           (0x3FFF)
    #define RTWCR_RX_LATCH            (1 << 15)
#define P0_RCPHR                      (0x1C)

/* Definition of PAGE1 */
#define P1_RPPER                      (0x22)
    #define RPPER_RXEN                (1 << 0)
#define P1_MRCR                       (0x28)
#define P1_MDR                        (0x2A)
#define P1_RMPR                       (0x2C)
#define P1_TMPR                       (0x2E)
#define P1_RXBSPCR                    (0x30)
    #define RXBSPCR_STUF_WORD_CNT(x)    (((x) & 0x7000) >> 12)
    #define RXBSPCR_STUF_ENABLE       (1 << 15)
#define P1_MCR                        (0x32)
    #define MCR_SBP                   (1 << 8)
    #define MCR_SM                    (1 << 9)
    #define MCR_CRCENLAN              (1 << 11)
    #define MCR_STP                   (1 << 12)
/* Definition of PAGE2 */
#define P2_CIR                        (0x42)
#define P2_POOLCR                     (0x44)
    #define POOLCR_POLL_EN            (1 << 0)
    #define POOLCR_POLL_FLOWCTRL      (1 << 1)
    #define POOLCR_POLL_BMCR          (1 << 2)
    #define POOLCR_PHYID(x)    ((x) << 8)
#define P2_PHYSR                      (0x46)
#define P2_MDIODR                     (0x48)
#define P2_MDIOCR                     (0x4A)
    #define MDIOCR_RADDR(x)    ((x) & 0x1F)
    #define MDIOCR_FADDR(x)    (((x) & 0x1F) << 8)
    #define MDIOCR_VALID              (1 << 13)
    #define MDIOCR_READ               (1 << 14)
    #define MDIOCR_WRITE              (1 << 15)
#define P2_LCR0                       (0x4C)
    #define LCR_LED0_EN               (1 << 0)
    #define LCR_LED0_100MODE          (1 << 1)
    #define LCR_LED0_DUPLEX           (1 << 2)
    #define LCR_LED0_LINK             (1 << 3)
    #define LCR_LED0_ACT              (1 << 4)
    #define LCR_LED0_COL              (1 << 5)
    #define LCR_LED0_10MODE           (1 << 6)
    #define LCR_LED0_DUPCOL           (1 << 7)
    #define LCR_LED1_EN               (1 << 8)
    #define LCR_LED1_100MODE          (1 << 9)
    #define LCR_LED1_DUPLEX           (1 << 10)
    #define LCR_LED1_LINK             (1 << 11)
    #define LCR_LED1_ACT              (1 << 12)
    #define LCR_LED1_COL              (1 << 13)
    #define LCR_LED1_10MODE           (1 << 14)
    #define LCR_LED1_DUPCOL           (1 << 15)
#define P2_LCR1                       (0x4E)
    #define LCR_LED2_MASK             (0xFF00)
    #define LCR_LED2_EN               (1 << 0)
    #define LCR_LED2_100MODE          (1 << 1)
    #define LCR_LED2_DUPLEX           (1 << 2)
    #define LCR_LED2_LINK             (1 << 3)
    #define LCR_LED2_ACT              (1 << 4)
    #define LCR_LED2_COL              (1 << 5)
    #define LCR_LED2_10MODE           (1 << 6)
    #define LCR_LED2_DUPCOL           (1 << 7)
#define P2_IPGCR                      (0x50)
#define P2_FLHWCR                     (0x54)
#define P2_RXCR                       (0x56)
    #define RXCR_PRO                  (1 << 0)
    #define RXCR_AMALL                (1 << 1)
    #define RXCR_SEP                  (1 << 2)
    #define RXCR_AB                   (1 << 3)
    #define RXCR_AM                   (1 << 4)
    #define RXCR_AP                   (1 << 5)
    #define RXCR_ARP                  (1 << 6)
#define P2_JLCR                       (0x58)
#define P2_MPLR                       (0x5C)

/* Definition of PAGE3 */
#define P3_MACASR0                    (0x62)
    #define P3_MACASR(x)    (P3_MACASR0 + 2 * x)
    #define MACASR_LOWBYTE_MASK       0x00FF
    #define MACASR_HIGH_BITS          0x08
#define P3_MACASR1                    (0x64)
#define P3_MACASR2                    (0x66)
#define P3_MFAR01                     (0x68)
#define P3_MFAR_BASE                  (0x68)
    #define P3_MFAR(x)    (P3_MFAR_BASE + 2 * x)

#define P3_MFAR23                     (0x6A)
#define P3_MFAR45                     (0x6C)
#define P3_MFAR67                     (0x6E)
#define P3_VID0FR                     (0x70)
#define P3_VID1FR                     (0x72)
#define P3_EECSR                      (0x74)
#define P3_EEDR                       (0x76)
#define P3_EECR                       (0x78)
    #define EECR_ADDR_MASK            (0x00FF)
    #define EECR_READ_ACT             (1 << 8)
    #define EECR_WRITE_ACT            (1 << 9)
    #define EECR_WRITE_DISABLE        (1 << 10)
    #define EECR_WRITE_ENABLE         (1 << 11)
    #define EECR_EE_READY             (1 << 13)
    #define EECR_RELOAD               (1 << 14)
    #define EECR_RESET                (1 << 15)
#define P3_TPCR                       (0x7A)
    #define TPCR_PATT_MASK            (0xFF)
    #define TPCR_RAND_PKT_EN          (1 << 14)
    #define TPCR_FIXED_PKT_EN         (1 << 15)
#define P3_TPLR                       (0x7C)
/* Definition of PAGE4 */
#define P4_SPICR                      (0x8A)
    #define SPICR_RCEN                (1 << 0)
    #define SPICR_QCEN                (1 << 1)
    #define SPICR_RBRE                (1 << 3)
    #define SPICR_PMM                 (1 << 4)
    #define SPICR_LOOPBACK            (1 << 8)
    #define SPICR_CORE_RES_CLR        (1 << 10)
    #define SPICR_SPI_RES_CLR         (1 << 11)
#define P4_SPIISMR                    (0x8C)

#define P4_COERCR0                    (0x92)
    #define COERCR0_RXIPCE            (1 << 0)
    #define COERCR0_RXIPVE            (1 << 1)
    #define COERCR0_RXV6PE            (1 << 2)
    #define COERCR0_RXTCPE            (1 << 3)
    #define COERCR0_RXUDPE            (1 << 4)
    #define COERCR0_RXICMP            (1 << 5)
    #define COERCR0_RXIGMP            (1 << 6)
    #define COERCR0_RXICV6            (1 << 7)

    #define COERCR0_RXTCPV6           (1 << 8)
    #define COERCR0_RXUDPV6           (1 << 9)
    #define COERCR0_RXICMV6           (1 << 10)
    #define COERCR0_RXIGMV6           (1 << 11)
    #define COERCR0_RXICV6V6          (1 << 12)

    #define COERCR0_DEFAULT           (COERCR0_RXIPCE | COERCR0_RXV6PE | \
                                       COERCR0_RXTCPE | COERCR0_RXUDPE | \
                                       COERCR0_RXTCPV6 | COERCR0_RXUDPV6)
#define P4_COERCR1                    (0x94)
    #define COERCR1_IPCEDP            (1 << 0)
    #define COERCR1_IPVEDP            (1 << 1)
    #define COERCR1_V6VEDP            (1 << 2)
    #define COERCR1_TCPEDP            (1 << 3)
    #define COERCR1_UDPEDP            (1 << 4)
    #define COERCR1_ICMPDP            (1 << 5)
    #define COERCR1_IGMPDP            (1 << 6)
    #define COERCR1_ICV6DP            (1 << 7)
    #define COERCR1_RX64TE            (1 << 8)
    #define COERCR1_RXPPPE            (1 << 9)
    #define COERCR1_TCP6DP            (1 << 10)
    #define COERCR1_UDP6DP            (1 << 11)
    #define COERCR1_IC6DP             (1 << 12)
    #define COERCR1_IG6DP             (1 << 13)
    #define COERCR1_ICV66DP           (1 << 14)
    #define COERCR1_RPCE              (1 << 15)

    #define COERCR1_DEFAULT           (COERCR1_RXPPPE)
#define P4_COETCR0                    (0x96)
    #define COETCR0_TXIP              (1 << 0)
    #define COETCR0_TXTCP             (1 << 1)
    #define COETCR0_TXUDP             (1 << 2)
    #define COETCR0_TXICMP            (1 << 3)
    #define COETCR0_TXIGMP            (1 << 4)
    #define COETCR0_TXICV6            (1 << 5)
    #define COETCR0_TXTCPV6           (1 << 8)
    #define COETCR0_TXUDPV6           (1 << 9)
    #define COETCR0_TXICMV6           (1 << 10)
    #define COETCR0_TXIGMV6           (1 << 11)
    #define COETCR0_TXICV6V6          (1 << 12)

    #define COETCR0_DEFAULT           (COETCR0_TXIP | COETCR0_TXTCP |    \
                                       COETCR0_TXUDP | COETCR0_TXTCPV6 | \
                                       COETCR0_TXUDPV6)
#define P4_COETCR1                    (0x98)
    #define COETCR1_TX64TE            (1 << 0)
    #define COETCR1_TXPPPE            (1 << 1)

#define P4_COECEDR                    (0x9A)
#define P4_L2CECR                     (0x9C)

/* Definition of PAGE5 */
#define P5_WFTR                       (0xA2)
    #define WFTR_2MS                  (0x01)
    #define WFTR_4MS                  (0x02)
    #define WFTR_8MS                  (0x03)
    #define WFTR_16MS                 (0x04)
    #define WFTR_32MS                 (0x05)
    #define WFTR_64MS                 (0x06)
    #define WFTR_128MS                (0x07)
    #define WFTR_256MS                (0x08)
    #define WFTR_512MS                (0x09)
    #define WFTR_1024MS               (0x0A)
    #define WFTR_2048MS               (0x0B)
    #define WFTR_4096MS               (0x0C)
    #define WFTR_8192MS               (0x0D)
    #define WFTR_16384MS              (0x0E)
    #define WFTR_32768MS              (0x0F)
#define P5_WFCCR                      (0xA4)
#define P5_WFCR03                     (0xA6)
    #define WFCR03_F0_EN              (1 << 0)
    #define WFCR03_F1_EN              (1 << 4)
    #define WFCR03_F2_EN              (1 << 8)
    #define WFCR03_F3_EN              (1 << 12)
#define P5_WFCR47                     (0xA8)
    #define WFCR47_F4_EN              (1 << 0)
    #define WFCR47_F5_EN              (1 << 4)
    #define WFCR47_F6_EN              (1 << 8)
    #define WFCR47_F7_EN              (1 << 12)
#define P5_WF0BMR0                    (0xAA)
#define P5_WF0BMR1                    (0xAC)
#define P5_WF0CR                      (0xAE)
#define P5_WF0OBR                     (0xB0)
#define P5_WF1BMR0                    (0xB2)
#define P5_WF1BMR1                    (0xB4)
#define P5_WF1CR                      (0xB6)
#define P5_WF1OBR                     (0xB8)
#define P5_WF2BMR0                    (0xBA)
#define P5_WF2BMR1                    (0xBC)

/* Definition of PAGE6 */
#define P6_WF2CR                      (0xC2)
#define P6_WF2OBR                     (0xC4)
#define P6_WF3BMR0                    (0xC6)
#define P6_WF3BMR1                    (0xC8)
#define P6_WF3CR                      (0xCA)
#define P6_WF3OBR                     (0xCC)
#define P6_WF4BMR0                    (0xCE)
#define P6_WF4BMR1                    (0xD0)
#define P6_WF4CR                      (0xD2)
#define P6_WF4OBR                     (0xD4)
#define P6_WF5BMR0                    (0xD6)
#define P6_WF5BMR1                    (0xD8)
#define P6_WF5CR                      (0xDA)
#define P6_WF5OBR                     (0xDC)

/* Definition of PAGE7 */
#define P7_WF6BMR0                    (0xE2)
#define P7_WF6BMR1                    (0xE4)
#define P7_WF6CR                      (0xE6)
#define P7_WF6OBR                     (0xE8)
#define P7_WF7BMR0                    (0xEA)
#define P7_WF7BMR1                    (0xEC)
#define P7_WF7CR                      (0xEE)
#define P7_WF7OBR                     (0xF0)
#define P7_WFR01                      (0xF2)
#define P7_WFR23                      (0xF4)
#define P7_WFR45                      (0xF6)
#define P7_WFR67                      (0xF8)
#define P7_WFPC0                      (0xFA)
#define P7_WFPC1                      (0xFC)

/* Tx header feilds mask */
#define TX_HDR_SOP_DICF               0x8000
#define TX_HDR_SOP_CPHI               0x4000
#define TX_HDR_SOP_INT                0x2000
#define TX_HDR_SOP_MDEQ               0x1000
#define TX_HDR_SOP_PKTLEN             0x07FF
#define TX_HDR_SOP_SEQNUM             0xF800
#define TX_HDR_SOP_PKTLENBAR          0x07FF

#define TX_HDR_SEG_FS                 0x8000
#define TX_HDR_SEG_LS                 0x4000
#define TX_HDR_SEG_SEGNUM             0x3800
#define TX_HDR_SEG_SEGLEN             0x0700
#define TX_HDR_SEG_EOFST              0xC000
#define TX_HDR_SEG_SOFST              0x3800
#define TX_HDR_SEG_SEGLENBAR          0x07FF

#define TX_HDR_EOP_SEQNUM             0xF800
#define TX_HDR_EOP_PKTLEN             0x07FF
#define TX_HDR_EOP_SEQNUMBAR          0xF800
#define TX_HDR_EOP_PKTLENBAR          0x07FF

/* Rx header fields mask */
#define RX_HDR1_MCBC                  0x8000
#define RX_HDR1_STUFF_PKT             0x4000
#define RX_HDR1_MII_ERR               0x2000
#define RX_HDR1_CRC_ERR               0x1000
#define RX_HDR1_PKT_LEN               0x07FF
#define RX_HDR2_SEQ_NUM               0xF800
#define RX_HDR2_PKT_LEN_BAR           0x7FFF
#define RX_HDR3_CE                    (1 << 15)
#define RX_HDR3_L3_PKT_TYPE(x)    (((x) >> 13) & 0x0003)
#define RX_HDR3_L4_PKT_TYPE(x)    (((x) >> 10) & 0x0007)
#define RX_HDR3_L3_ERR                (1 << 9)
#define RX_HDR3_L4_ERR                (1 << 8)
#define RX_HDR3_PRIORITY(x)       (((x) >> 4) & 0x0007)
#define RX_HDR3_STRIP                 (1 << 3)
#define RX_HDR3_VLAN_ID(x)        (x & 0x0007)

#define AX_RX_CHECKSUM                1
#define AX_TX_CHECKSUM                2


#define MYMAC_1                       0x00
#define MYMAC_2                       0x12
#define MYMAC_3                       0x34
#define MYMAC_4                       0x56
#define MYMAC_5                       0x78
#define MYMAC_6                       0x99


/* Rx headers structure */
struct rx_header
{
  unsigned short flags_len;
  unsigned short seq_lenbar;
  unsigned short flags;
};
/* Tx headers structure */
struct tx_sop_header
{
  /* bit 15-11: flags, bit 10-0: packet length */
  unsigned short flags_pktlen;
  /* bit 15-11: sequence number, bit 11-0: packet length bar */
  unsigned short seqnum_pktlenbar;
};

struct tx_segment_header
{
  /* bit 15-14: flags, bit 13-11: segment number, bit 10-0: segment length */
  unsigned short f_segnum_seglen;
  /* bit 15-14: end offset, bit 13-11: start offset */
  /* bit 10-0: segment length bar */
  unsigned short eo_so_seglenbar;
};

struct tx_eop_header
{
  /* bit 15-11: sequence number, bit 10-0: packet length */
  unsigned short seqnum_pktlen;
  /* bit 15-11: sequence number bar, bit 10-0: packet length bar */
  unsigned short seqnumbar_pktlenbar;
};

struct tx_header
{
  struct tx_sop_header     sop;
  struct tx_segment_header seg;
};

u16_t SwapBytes(u16_t Data);
u32_t timer_tick(void);
void timer_reset(void);
u8_t ax88796c_check_free_pages(u8_t need_pages);
void etherdev_chkmedia(void);
void ax_set_csums(void);
void ax_phy_init(void);
u16_t axspi_read_reg(u8_t reg);
void axspi_write_reg(uint16_t value, uint8_t reg);
void spi_write_fifo_pio(uint8_t *buf, uint16_t count);
void spi_read_fifo_pio(uint8_t *buf, uint16_t count);
void ax88796c_set_mac_addr(struct netif *netif);
void ax88796c_load_mac_addr(struct netif *netif);
uint16_t ax88796c_mdio_read(uint16_t phy_id, uint16_t loc);
static void ax88796c_mdio_write(uint16_t phy_id, uint16_t loc, uint16_t val);

#endif

/*----------------------------------------------------------------------------
* end of file
*---------------------------------------------------------------------------*/

