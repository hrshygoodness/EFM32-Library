/**************************************************************************//**
 * @file AX88796C.c
 * @brief Ethernet packet-driver for use with LAN-controller AX88796C
 * @author Silicon Labs
 * @version 1.09
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/

#include "em_chip.h"
#include "netif/AX88796C.h"
#include <stdio.h>
#include "lwip/tcpip.h"
#include "lwip/debug.h"
#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/stats.h"
#include "lwip/sys.h"
#include "netif/etharp.h"
#include "lwip/ip_addr.h"
#include "spi.h"

unsigned long tick_count = 0;

/**************************************************************************//**
 * Function Name:  SwapBytes
 * Purpose:
 * Params:
 * Returns:
 * Note:
 *****************************************************************************/
u16_t SwapBytes(u16_t Data)
{
  return (Data >> 8) | (Data << 8);
}

/**************************************************************************//**
 * Function Name:  timer0_tick
 * Purpose:
 * Params:
 * Returns:
 * Note:
 *****************************************************************************/
u32_t timer_tick(void)
{
  return tick_count;
}

/**************************************************************************//**
 * Function Name:  timer_reset
 * Purpose:
 * Params:
 * Returns:
 * Note:
 *****************************************************************************/
void timer_reset(void)
{
  tick_count = 0;
}

/**************************************************************************//**
 * Function Name:   ax88796c_check_free_pages
 * Purpose:  Check free pages of TX buffer
 * Params:
 * Returns:
 * Note:
 *****************************************************************************/
u8_t ax88796c_check_free_pages(u8_t need_pages)
{
  u8_t  free_pages;
  u16_t tmp16;
  tmp16      = axspi_read_reg(P0_TFBFCR);
  free_pages = tmp16 & TX_FREEBUF_MASK;

  if (free_pages < need_pages)
  {
    /* schedule free page interrupt */
    tmp16 = axspi_read_reg(P0_TFBFCR);
    axspi_write_reg(tmp16 | TFBFCR_TX_PAGE_SET | TFBFCR_SET_FREE_PAGE(need_pages), P0_TFBFCR);
    return 1;
  }
  return 0;
}

/**************************************************************************//**
 * Function Name: etherdev_chkmedia
 * Purpose:
 * Params:	none
 * Returns:
 * Note:
 *****************************************************************************/
void etherdev_chkmedia(void)
{
  u16_t media;
  u8_t  cable = 0;

  while (1)
  {
    media = axspi_read_reg(P0_PSCR);
    if (media & PSCR_PHYLINK)
      break;

    if (!(media & PSCR_PHYCOFF))
    {
      /* The ethernet cable has been plugged */
      if (!cable)
      {
        cable = 1;
      }
      else
      {
        if (cable++ == 8)
        {
          ax88796c_mdio_write(0x10, 0, SPEED_DUPLEX_AUTO);
          cable = 1;
        }
      }
    }
    else
    {
      cable = 0;
    }
  }
}

/**************************************************************************//**
 * Function Name: axspi_read_reg
 * Purpose:
 * Params:	none
 * Returns:
 * Note:
 *****************************************************************************/
u16_t axspi_read_reg(u8_t reg)
{
  u16_t ret;
  u8_t  a, b;

  /* Activate Slave Select */
  CS_LOW();

  /* Step1: Send the READ command */
  spiSendByte(AX_SPICMD_READ_REG);

  /* Step2 :Send register address */
  spiSendByte(reg);

  /* Step3 :Send dummy address */
  spiSendByte(0xFF);

  /* Step4 :Send dummy address */
  spiSendByte(0xFF);

  /* Step5: Read data */
  a = spiGetByte();

  /* Step6: Read data */
  b = spiGetByte();

  ret  = b;
  ret  = ret << 8;
  ret |= a;

  /* Deactivate Slave Select */
  CS_HIGH();

  return ret;
}

/**************************************************************************//**
 * Function Name: axspi_write_reg
 * Purpose: write register
 * Params:
 * Returns:
 * Note:
 *****************************************************************************/
void axspi_write_reg(uint16_t value, uint8_t reg)
{
  uint8_t tx_buf[4];

  tx_buf[0] = AX_SPICMD_WRITE_REG;      /* OP code read register //0xD8 */
  tx_buf[1] = reg;                      /* Register address */
  tx_buf[2] = value;                    /*  MSB */
  tx_buf[3] = value >> 8;               /* LSB */

  /* Activate Slave Select*/
  CS_LOW();

  /* Step1: Send the Write command */
  spiSendByte(tx_buf[0]);

  /* Step2 :Send register address */
  spiSendByte(tx_buf[1]);

  /* Step3: Send the  (MSB first) */
  spiSendByte(tx_buf[2]);

  /* Step4: Send the (LSB last) */
  spiSendByte(tx_buf[3]);

  /*Deactivate Slave Select*/
  CS_HIGH();
}

/**************************************************************************//**
 * Function Name: spi_write_fifo_pio
 * Purpose: write fifo
 * Params:
 * Returns:	none
 * Note:
 *****************************************************************************/
void spi_write_fifo_pio(uint8_t *buf, uint16_t count)
{
  uint16_t i;

  /* Activate Slave Select */
  CS_LOW();

  spiSendByte(AX_SPICMD_WRITE_TXQ);

  /*Dummy*/
  for (i = 0; i < 3; i++)
  {
    spiSendByte(0xFF);
  }

  /*Data*/
  for (i = 0; i < count; i++)
  {
    spiSendByte(*(buf + i));
  }

  /*Deactivate Slave Select*/
  CS_HIGH();
}

/**************************************************************************//**
 * Function Name: spi_read_fifo_pio
 * Purpose: read fifo
 * Params:
 * Returns:	none
 * Note:
 *****************************************************************************/
void spi_read_fifo_pio(uint8_t *buf, uint16_t count)
{
  uint16_t i;

  /*Activate Slave Select*/
  CS_LOW();

  spiSendByte(AX_SPICMD_READ_RXQ);

  /*Dummy*/
  for (i = 0; i < 4; i++)
  {
    spiSendByte(0xFF);
  }

  /*Data*/
  for (i = 0; i < count; i++)
  {
    *(buf + i) = spiGetByte();
  }

  /*Deactivate Slave Select */
  CS_HIGH();
}

/**************************************************************************//**
 * Function Name: ax88796c_set_mac_addr
 * Purpose: Set up AX88796C MAC address
 * Params:
 * Returns:	none
 * Note:
 *****************************************************************************/
void ax88796c_set_mac_addr(struct netif *netif)
{
  uint16_t tmp16;

  tmp16 = ((unsigned short)(netif->hwaddr[4] << 8) | (unsigned short)(netif->hwaddr[5]));
  axspi_write_reg(tmp16, P3_MACASR0);

  tmp16 = ((unsigned short)(netif->hwaddr[2] << 8) | (unsigned short)(netif->hwaddr[3]));
  axspi_write_reg(tmp16, P3_MACASR1);

  tmp16 = ((unsigned short)(netif->hwaddr[0] << 8) | (unsigned short)(netif->hwaddr[1]));
  axspi_write_reg(tmp16, P3_MACASR2);
}

/**************************************************************************//**
 * Function Name: ax88796c_load_mac_addr
 * Purpose: Read MAC address from AX88796C
 * Params:
 * Returns:	none
 * Note:
 *****************************************************************************/
void ax88796c_load_mac_addr(struct netif *netif)
{
  u16_t tmp16;

  /* Read the MAC address from AX88796C */
  tmp16            = axspi_read_reg(P3_MACASR0);
  netif->hwaddr[5] = (unsigned char) tmp16;
  netif->hwaddr[4] = (unsigned char)(tmp16 >> 8);

  tmp16            = axspi_read_reg(P3_MACASR1);
  netif->hwaddr[3] = (unsigned char) tmp16;
  netif->hwaddr[2] = (unsigned char)(tmp16 >> 8);

  tmp16            = axspi_read_reg(P3_MACASR2);
  netif->hwaddr[1] = (unsigned char) tmp16;
  netif->hwaddr[0] = (unsigned char)(tmp16 >> 8);
}


/**************************************************************************//**
 * Function Name: ax_set_csums
 * Purpose: csums initialize procedure.
 * Params:	none
 * Returns:	none
 * Note:
 *****************************************************************************/
void ax_set_csums(void)
{
  /* Enable RX checksum */
  axspi_write_reg((COERCR0_RXIPCE | COERCR0_RXTCPE | COERCR0_RXUDPE), P4_COERCR0);

  /* Enable TX checksum */
  axspi_write_reg((COETCR0_TXIP | COETCR0_TXTCP | COETCR0_TXUDP), P4_COETCR0);
}

/**************************************************************************//**
 * Function Name: ax_phy_init
 * Purpose: phy initialize procedure.
 * Params:	none
 * Returns:	none
 * Note:
 *****************************************************************************/
void ax_phy_init(void)
{
  uint16_t tmp16;

  /* Setup LED mode */
  axspi_write_reg((LCR_LED0_EN | LCR_LED0_DUPLEX | LCR_LED1_EN |
                   LCR_LED1_100MODE), P2_LCR0);

  tmp16 = axspi_read_reg(P2_LCR1);
  axspi_write_reg((tmp16 & LCR_LED2_MASK) | LCR_LED2_EN | LCR_LED2_LINK, P2_LCR1);

  tmp16 = (POOLCR_PHYID(0x10) | POOLCR_POLL_EN | POOLCR_POLL_FLOWCTRL | POOLCR_POLL_BMCR);
  axspi_write_reg(tmp16, P2_POOLCR);

  ax88796c_mdio_write(0x10, 0x04, SPEED_DUPLEX_AUTO);
  ax88796c_mdio_write(0x10, 0x00, RESTART_AUTONEG);
}



/**************************************************************************//**
 * Function Name: ax88796c_mdio_read
 * Purpose: read mdio procedure.
 * Params:
 * Returns:	none
 * Note:
 *****************************************************************************/
uint16_t ax88796c_mdio_read(uint16_t phy_id, uint16_t loc)
{
  uint16_t val;
  uint16_t timeout;

  axspi_write_reg((MDIOCR_RADDR(loc) | MDIOCR_FADDR(phy_id) | MDIOCR_READ), P2_MDIOCR);
  for (timeout = 0; timeout < 10000; timeout++)
  {
    val = axspi_read_reg(P2_MDIOCR);
    if ((val & MDIOCR_VALID) != 0)
    {
      break;
    }
  }

  val = axspi_read_reg(P2_MDIODR);

  return val;
}

/**************************************************************************//**
 * Function Name: etherdev_init
 * Purpose: ax88796 initialize procedure.
 * Params:
 * Returns:	none
 * Note:
 *****************************************************************************/
static void ax88796c_mdio_write(uint16_t phy_id, uint16_t loc, uint16_t val)
{
  uint16_t timeout;
  uint16_t tmp16;

  axspi_write_reg(val, P2_MDIODR);
  axspi_write_reg((MDIOCR_RADDR(loc) | MDIOCR_FADDR(phy_id) | MDIOCR_WRITE), P2_MDIOCR);

  for (timeout = 0; timeout < 10000; timeout++)
  {
    tmp16 = axspi_read_reg(P2_MDIOCR);
    if ((tmp16 & MDIOCR_VALID) != 0)
    {
      break;
    }
  }
}

