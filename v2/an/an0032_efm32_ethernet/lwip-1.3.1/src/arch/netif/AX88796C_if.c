/**************************************************************************//**
 * @file AX88796C_if.c
 * @brief This file is derived from the ``ethernetif.c'' skeleton Ethernet network
 *        interface driver for lwIP.
 * @author Silicon Labs
 * @version 1.09
 ******************************************************************************/

 /**
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

#include "em_device.h"
#include "em_chip.h"
#include "netif/AX88796C.h"
#include "netif/AX88796C_if.h"
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

static unsigned short seq_num = 0;

#define TRUE     (1 == 1)
#define FALSE    (1 == 0)
/**
 * Sanity Check:  This interface driver will NOT work if the following defines
 * are incorrect.
 *
 */
#if (PBUF_LINK_HLEN != 14)
#error "PBUF_LINK_HLEN must be 14 for this interface driver!"
#endif

#if 0
#if (!SYS_LIGHTWEIGHT_PROT)
         #error "SYS_LIGHTWEIGHT_PROT must be enabled for this interface driver!"
#endif
#endif

/**
 * Number of pbufs supported in low-level tx/rx pbuf queue.
 *
 */
#ifndef NUM_PBUF_QUEUE
#define NUM_PBUF_QUEUE    20
#endif

/**
 * Setup processing for PTP (IEEE-1588).
 *
 */
#if LWIP_PTPD
extern void lwIPHostGetTime(u32_t *time_s, u32_t *time_ns);
#endif

/* Define those to better describe your network interface. */
#define IFNAME0         'e'
#define IFNAME1         '0'

#define _MNAND_RYBN3    (1UL << 24)

/* Helper struct to hold a queue of pbufs for transmit and receive. */
struct pbufq
{
  struct pbuf   *pbuf[NUM_PBUF_QUEUE];
  unsigned long qwrite;
  unsigned long qread;
  unsigned long overflow;
};

/* Helper macros for accessing pbuf queues. */
#define PBUF_QUEUE_EMPTY(q) \
  (((q)->qwrite == (q)->qread) ? TRUE : FALSE)

#define PBUF_QUEUE_FULL(q)                                  \
  ((((((q)->qwrite + 1) % NUM_PBUF_QUEUE)) == (q)->qread) ? \
   TRUE : FALSE)

/**
 * Helper struct to hold private data used to operate your ethernet interface.
 * Keeping the ethernet address of the MAC in this struct is not necessary
 * as it is already kept in the struct netif.
 * But this is only an example, anyway...
 */
struct ethernetif
{
  struct eth_addr *ethaddr;
  /* Add whatever per-interface state that is needed here. */
  struct pbufq    txq;
  struct pbufq    rxq;
};

/**
 * Global variable for this interface's private data.  Needed to allow
 * the interrupt handlers access to this information outside of the
 * context of the lwIP netif.
 *
 */
static struct ethernetif ethernetif_data;


/**
 * Event router interrupt 0 handler
 *
 * @param q is the packet queue from which to pop the pbuf.
 *
 * @return pointer to pbuf packet if available, NULL otherswise.
 */
void evn_router_interrupt0_handler(void)
{
  extern struct netif lwip_netif;
  AX88796Cif_interrupt(&lwip_netif);
  AX88796Cif_input(&lwip_netif);
}

/**
 * Pop a pbuf packet from a pbuf packet queue
 *
 * @param q is the packet queue from which to pop the pbuf.
 *
 * @return pointer to pbuf packet if available, NULL otherswise.
 */
static struct pbuf *
dequeue_packet(struct pbufq *q)
{
  struct pbuf *pBuf;
  SYS_ARCH_DECL_PROTECT(lev);

  /**
   * This entire function must run within a "critical section" to preserve
   * the integrity of the transmit pbuf queue.
   *
   */
  SYS_ARCH_PROTECT(lev);

  if (PBUF_QUEUE_EMPTY(q))
  {
    /* Return a NULL pointer if the queue is empty. */
    pBuf = (struct pbuf *) NULL;
  }
  else
  {
    /**
     * The queue is not empty so return the next frame from it
     * and adjust the read pointer accordingly.
     *
     */
    pBuf     = q->pbuf[q->qread];
    q->qread = ((q->qread + 1) % NUM_PBUF_QUEUE);
  }

  /* Return to prior interrupt state and return the pbuf pointer. */
  SYS_ARCH_UNPROTECT(lev);
  return(pBuf);
}

/**
 * Push a pbuf packet onto a pbuf packet queue
 *
 * @param p is the pbuf to push onto the packet queue.
 * @param q is the packet queue.
 *
 * @return 1 if successful, 0 if q is full.
 */
static int
enqueue_packet(struct pbuf *p, struct pbufq *q)
{
  SYS_ARCH_DECL_PROTECT(lev);
  int ret;

  /**
   * This entire function must run within a "critical section" to preserve
   * the integrity of the transmit pbuf queue.
   *
   */
  SYS_ARCH_PROTECT(lev);

  if (!PBUF_QUEUE_FULL(q))
  {
    /**
     * The queue isn't full so we add the new frame at the current
     * write position and move the write pointer.
     *
     */
    q->pbuf[q->qwrite] = p;
    q->qwrite          = ((q->qwrite + 1) % NUM_PBUF_QUEUE);
    ret                = 1;
  }
  else
  {
    /**
     * The stack is full so we are throwing away this value.  Keep track
     * of the number of times this happens.
     *
     */
    q->overflow++;
    ret = 0;
  }

  /* Return to prior interrupt state and return the pbuf pointer. */
  SYS_ARCH_UNPROTECT(lev);
  return(ret);
}

/**
 * In this function, the hardware should be initialized.
 * Called from AX88796Cif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
static void
low_level_init(struct netif *netif)
{
  unsigned short tmp16;
  /* set MAC hardware address length */
  netif->hwaddr_len = ETHARP_HWADDR_LEN;
  netif->hwaddr[0]  = 0x00;
  netif->hwaddr[1]  = 0x12;
  netif->hwaddr[2]  = 0x34;
  netif->hwaddr[3]  = 0x56;
  netif->hwaddr[4]  = 0x78;
  netif->hwaddr[5]  = 0x9A;

  /* maximum transfer unit */
  netif->mtu = 1500;

  /* device capabilities */
  /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
  netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;


  /* intialize the AX88796C chip */

  /* Hardware reset ax88796 */
  timer_reset();
  while (1)
  {
    if (timer_tick() > 1000) break;
    RESET_LOW();
  }

  RESET_HIGH();

  /*Software Reset*/
  axspi_write_reg(PSR_RESET, P0_PSR);
  axspi_write_reg(PSR_RESET_CLR, P0_PSR);

  /* Make sure AX88796C is ready */
  timer_reset();
  while (1)
  {
    tmp16 = axspi_read_reg(P0_PSR);
    if ((tmp16 & PSR_DEV_READY) && (tmp16 != 0xFFFF))
    {
      break;
    }

    if (timer_tick() > 2000)
    {
      LWIP_DEBUGF(NETIF_DEBUG, ("Device not ready..\n\r"));
      /* return 0; */
    }
  }
  tmp16 = axspi_read_reg(P0_BOR);

  while (1)
  {
    tmp16 = axspi_read_reg(P0_BOR);
    if (tmp16 == 0x1234) break;
  }

  /*Register compresion = 0 */
  tmp16 = axspi_read_reg(P4_SPICR);
  axspi_write_reg((tmp16 & ~(SPICR_RCEN | SPICR_QCEN)), P4_SPICR);

  /*Set mac for AX88796C chip*/
  ax88796c_set_mac_addr(netif);

  /*Set crc*/
  ax_set_csums();

  /* Stuffing packet */
  tmp16 = axspi_read_reg(P1_RXBSPCR);
  /* Disable stuffing packet */
  /* Enable stuffing packet */
  axspi_write_reg(tmp16 | RXBSPCR_STUF_ENABLE, P1_RXBSPCR);

  /* Enable RX packet process */
  axspi_write_reg(RPPER_RXEN, P1_RPPER);


  /*set INT pin */
  tmp16 = axspi_read_reg(P0_FER);
  tmp16 = (tmp16 | FER_RXEN | FER_TXEN | FER_BSWAP | FER_IRQ_PULL | FER_INTLO);
  axspi_write_reg(tmp16, P0_FER);

  /*set PHY */
  ax_phy_init();
}

/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf might be
 * chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 * @note This function MUST be called with interrupts disabled or with the
 *       AX88796C Ethernet transmit fifo protected.
 */
static err_t
low_level_transmit(struct netif *netif, struct pbuf *p)
{
  struct pbuf          *q;
  struct tx_header     txhdr;
  struct tx_eop_header tx_eop;
  u16_t                length;
  u16_t                loop_cnt, pkt_len;
  u8_t                 *point;
  u16_t                tmp16A, tmp16B;
  u16_t                tmp16;
  u8_t                 need_pages;
  u8_t                 *ptr;

  length = p->tot_len;

  pkt_len = length;

  loop_cnt = (length + 3) & 0xFFFC;

  need_pages = (pkt_len + TX_OVERHEAD + 127) >> 7;


  if (ax88796c_check_free_pages(need_pages) != 0)
  {
    return ERR_IF;
  }


  for (q = p; q != NULL; q = q->next)
  {
    ptr = (u8_t * ) q->payload;

    /* Prepare Sop header */
    txhdr.sop.flags_pktlen     = SwapBytes(pkt_len);
    txhdr.sop.seqnum_pktlenbar = SwapBytes((seq_num << 11) | (~pkt_len & TX_HDR_SOP_PKTLENBAR));

    /* Prepare Segment header */
    txhdr.seg.f_segnum_seglen = SwapBytes((pkt_len | TX_HDR_SEG_FS | TX_HDR_SEG_LS));
    txhdr.seg.eo_so_seglenbar = SwapBytes((~pkt_len & TX_HDR_SEG_SEGLENBAR));


    axspi_write_reg((TSNR_TXB_START | TSNR_PKT_CNT(1)), P0_TSNR);

    /* Write TX header */
    point = (unsigned char *) &txhdr;
    spi_write_fifo_pio(point, 8);

    /* Write packet */
    point = (unsigned char *) ptr;
    spi_write_fifo_pio(point, loop_cnt);

    /* Prepare EOP header */
    tx_eop.seqnum_pktlen       = SwapBytes((seq_num << 11) | pkt_len);
    tx_eop.seqnumbar_pktlenbar = SwapBytes(((~seq_num << 11) & TX_HDR_EOP_SEQNUMBAR) | (~pkt_len & TX_HDR_EOP_PKTLENBAR));

    /* Write TX end of packet */
    point = (unsigned char *) &tx_eop;
    spi_write_fifo_pio(point, 4);

    /* ++ */
    seq_num = (seq_num + 1) & 0x1F;
  }

  tmp16A = axspi_read_reg(P0_TSNR);
  tmp16B = axspi_read_reg(P0_ISR);
  if (((tmp16A & TXNR_TXB_IDLE) == 0) || ((ISR_TXERR & tmp16B) != 0))
  {
    /* Ack tx error int */
    axspi_write_reg(tmp16B | ISR_TXERR, P0_ISR);
    /* Reinitial tx bridge */
    axspi_write_reg(tmp16A | TXNR_TXB_IDLE, P0_TSNR);
    seq_num = tmp16A & 0x1F;
  }


  /*Step 8: Configure the ISR register (bit 8) to check the transmission result.*/
  tmp16 = axspi_read_reg(P0_ISR);
  if (tmp16 & ISR_TXERR)
  {
    /*Read the TSNR register (bit 0-4) to get the last TX sequence number that*/
    /*the AX88796C have received from the software successfully for packet retransmission.*/
    tmp16   = axspi_read_reg(P0_TSNR);
    seq_num = tmp16 & 0x01F;

    /*Write 1 into the bit 14 of TSNR register to re-initialize the TX Bridge function */
    axspi_write_reg(tmp16 | TXNR_TXB_REINIT, P0_TSNR);
  }

  /* Dereference the pbuf from the queue. */
  pbuf_free(p);

  LINK_STATS_INC(link.xmit);

  return(ERR_OK);
}

/**
 * This function with either place the packet into the AX88796C transmit fifo,
 * or will place the packet in the interface PBUF Queue for subsequent
 * transmission when the transmitter becomes idle.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 */
static err_t
low_level_output(struct netif *netif, struct pbuf *p)
{
  struct ethernetif *ethernetif = netif->state;
  unsigned short    reg;

  SYS_ARCH_DECL_PROTECT(lev);

  /**
   * This entire function must run within a "critical section" to preserve
   * the integrity of the transmit pbuf queue.
   *
   */
  SYS_ARCH_PROTECT(lev);

  /**
   * Bump the reference count on the pbuf to prevent it from being
   * freed till we are done with it.
   *
   */
  pbuf_ref(p);

  /**
   * If the transmitter is idle, and there is nothing on the queue,
   * send the pbuf now.
   *
   */
  reg = axspi_read_reg(P0_RXBCR2);
  if (PBUF_QUEUE_EMPTY(&ethernetif->txq) &&
      (reg & RXBCR2_RXB_IDLE))
  {
    low_level_transmit(netif, p);
  }
  /* Otherwise place the pbuf on the transmit queue. */
  else
  {
    /* Add to transmit packet queue */
    if (!enqueue_packet(p, &(ethernetif->txq)))
    {
      /* if no room on the queue, free the pbuf reference and return error. */
      pbuf_free(p);
      SYS_ARCH_UNPROTECT(lev);
      return(ERR_MEM);
    }
  }

  /* Return to prior interrupt state and return. */
  SYS_ARCH_UNPROTECT(lev);
  return ERR_OK;
}

/**
 * This function will read a single packet from the AX88796C ethernet
 * interface, if available, and return a pointer to a pbuf.  The timestamp
 * of the packet will be placed into the pbuf structure.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return pointer to pbuf packet if available, NULL otherswise.
 */
static struct pbuf *
low_level_receive(struct netif *netif)
{
  struct pbuf      *p, *q;
  struct rx_header rx_hdr;
  u8_t             *ptr;
  u16_t            tmp16;
  u16_t            pkt_cnt = 0, pkt_len = 0;
  u16_t            w_count;
  u8_t             *point;

  /*Step 5: Configure the RTWCR register (bit 15) to latch RX data and
   * read the RXBCR2 register (bit 0-7) to get the total packet count in  */
  /* RX SRAM buffer */
  tmp16 = axspi_read_reg(P0_RTWCR);
  axspi_write_reg(tmp16 | RTWCR_RX_LATCH, P0_RTWCR);

  /* check rx packet and total word count */
  tmp16   = axspi_read_reg(P0_RXBCR2);
  pkt_cnt = tmp16 & RXBCR2_PKT_MASK;
  if (!pkt_cnt) return(NULL);

  /* Step 4: Configure the RCPHR register (bit 0-10) to get the length */
  /* of the next packet and count the burst length */
  tmp16   = axspi_read_reg(P0_RCPHR);
  pkt_len = tmp16 & 0x7FF;
  if (!pkt_len) return(NULL);


  w_count = ((pkt_len + 6 + 3) & 0xFFFC) >> 1;

  if (w_count > IP_FRAG_MAX_MTU)
  {
    LWIP_DEBUGF(NETIF_DEBUG, ("%s: Couldn't allocate a sk_buff of size %d\n",
                              w_count * 2));
    axspi_write_reg(RXBCR1_RXB_DISCARD, P0_RXBCR1);
    return(NULL);
  }

  /* We allocate a pbuf chain of pbufs from the pool. */
  p = pbuf_alloc(PBUF_RAW, ((2 * w_count) & 0xFFFC), PBUF_POOL);
  if (p != NULL)
  {
    /* Step 6: Configure the RXBCR1 register (bit 0-13) to set the word count of */
    /* this reception. */
    /* Step 7: Configure the RXBCR1 register (bit 15) to start the reception operation. */
    axspi_write_reg((RXBCR1_RXB_START | w_count), P0_RXBCR1);

    /* Read packet */
    point = (unsigned char *) &(rx_hdr.flags_len);
    spi_read_fifo_pio(point, 6);

    rx_hdr.flags_len  = SwapBytes(rx_hdr.flags_len);
    rx_hdr.seq_lenbar = SwapBytes(rx_hdr.seq_lenbar);
    rx_hdr.flags      = SwapBytes(rx_hdr.flags);

    LWIP_DEBUGF(NETIF_DEBUG, ("AX88796_input:packet len %u\n", len));

    for (q = p; q != NULL; q = q->next)
    {
      LWIP_DEBUGF(NETIF_DEBUG, ("AX88796_input:pbuf@%p len %u\n", q, q->len));
      ptr = q->payload;

      /* Read packet */
      point = ptr;
      spi_read_fifo_pio(point, q->len);
    }
  }
  else
  {
    pbuf_free(p);
    axspi_write_reg(RXBCR1_RXB_DISCARD | w_count, P0_RXBCR1);

    /* Adjust the link statistics */
    LINK_STATS_INC(link.memerr);
    LINK_STATS_INC(link.drop);
  }

  return(p);
}

/**
 * This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this ethernetif
 */
int
AX88796Cif_input(struct netif *netif)
{
  struct ethernetif *ethernetif;
  struct pbuf       *p;
  int               count = 0;

  ethernetif = netif->state;

  /* move received packet into a new pbuf */
  while ((p = dequeue_packet(&ethernetif->rxq)) != NULL)
  {
    count++;
    /* process the packet. */
    if (ethernet_input(p, netif) != ERR_OK)
    {
      LWIP_DEBUGF(NETIF_DEBUG, ("AX88796Cif_input: input error\n"));
      pbuf_free(p);
      p = NULL;
    }
  }

  return(count);
}

/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t
AX88796Cif_init(struct netif *netif)
{
  LWIP_ASSERT("netif != NULL", (netif != NULL));

#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
  netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

  /*
   * Initialize the snmp variables and counters inside the struct netif.
   * The last argument should be replaced with your link speed, in units
   * of bits per second.
   */
  NETIF_INIT_SNMP(netif, snmp_ifType_ethernet_csmacd, 1000000);

  netif->state   = &ethernetif_data;
  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;
  /* We directly use etharp_output() here to save a function call.
   * You can instead declare your own function an call etharp_output()
   * from it if you have to do some checks before sending (e.g. if link
   * is available...) */
  netif->output     = etharp_output;
  netif->linkoutput = low_level_output;

  ethernetif_data.ethaddr      = (struct eth_addr *) &(netif->hwaddr[0]);
  ethernetif_data.txq.qread    = ethernetif_data.txq.qwrite = 0;
  ethernetif_data.txq.overflow = 0;
  ethernetif_data.rxq.qread    = ethernetif_data.rxq.qwrite = 0;
  ethernetif_data.rxq.overflow = 0;

  /* initialize the hardware */
  low_level_init(netif);

  return ERR_OK;
}

/**
 * Process tx and rx packets at the low-level interrupt.
 *
 * Should be called from the AX88796C Ethernet Interrupt Handler.  This
 * function will read packets from the AX88796C Ethernet fifo and place them
 * into a pbuf queue.  If the transmitter is idle and there is at least one packet
 * on the transmit queue, it will place it in the transmit fifo and start the
 * transmitter.
 *
 */
void
AX88796Cif_interrupt(struct netif *netif)
{
  struct ethernetif *ethernetif;
  struct pbuf       *p = NULL;
  u16_t             isr, reg;

  /* setup pointer to the if state data */
  ethernetif = netif->state;

  /**
   * Process the transmit and receive queues as long as there is receive
   * data available
   *
   */
  /* read isr register */
  isr = axspi_read_reg(P0_ISR);

  if (isr & ISR_TXERR)
  {
    LWIP_DEBUGF(NETIF_DEBUG, ("  TXERR interrupt\n"));

    /*Write 1 into the bit 8 of ISR register to clear the TXERR interrupt status.*/
    axspi_write_reg(isr | ISR_TXERR, P0_ISR);

    reg = axspi_read_reg(P0_TSNR);
    axspi_write_reg(reg | TXNR_TXB_REINIT, P0_TSNR);
    seq_num = reg & 0x1F;
  }

  if (isr & ISR_TXPAGES)
  {
    LWIP_DEBUGF(NETIF_DEBUG, ("  TXPAGES interrupt\n"));
    axspi_write_reg(isr | ISR_TXPAGES, P0_ISR);
  }

  if (ISR_RXPKT & isr)
  {
    /* clear RXPKT  interrupt status */
    axspi_write_reg(isr | ISR_RXPKT, P0_ISR);

    /* set RXPKT mask status */
    axspi_write_reg(~IMR_RXPKT, P0_IMR);

    p = low_level_receive(netif);
    while (p)
    {
      /* Add the rx packet to the rx queue */
      if (!enqueue_packet(p, &ethernetif->rxq))
      {
        /* Could not place the packet on the queue, bail out. */
        pbuf_free(p);
      }
      /* send packet */
      reg = axspi_read_reg(P0_RXBCR2);
      if (reg & RXBCR2_RXB_IDLE)
      {
        p = dequeue_packet(&ethernetif->txq);
        if (p != NULL)
        {
          low_level_transmit(netif, p);
        }
      }
      reg = axspi_read_reg(P0_RXBCR2);
      if (reg & RXBCR2_RXB_IDLE)
      {
        p = low_level_receive(netif);
      }
    }
    /* One more check of the transmit queue/fifo */
    reg = axspi_read_reg(P0_RXBCR2);
    if (reg & RXBCR2_RXB_IDLE)
    {
      p = dequeue_packet(&ethernetif->txq);
      if (p != NULL)
      {
        low_level_transmit(netif, p);
      }
    }
  }
}

#if NETIF_DEBUG
/* Print an IP header by using LWIP_DEBUGF
 * @param p an IP packet, p->payload pointing to the IP header
 */
void
AX88796Cif_debug_print(struct pbuf *p)
{
  struct eth_hdr *ethhdr = (struct eth_hdr *) p->payload;
  u16_t          *plen   = (u16_t *) p->payload;

  LWIP_DEBUGF(NETIF_DEBUG, ("ETH header:\n"));
  LWIP_DEBUGF(NETIF_DEBUG, ("Packet Length:%5" U16_F " \n", *plen));
  LWIP_DEBUGF(NETIF_DEBUG, ("Destination: %02" X8_F "-%02" X8_F "-%02" X8_F "-%02" X8_F "-%02" X8_F "-%02" X8_F "\n",
                            ethhdr->dest.addr[0],
                            ethhdr->dest.addr[1],
                            ethhdr->dest.addr[2],
                            ethhdr->dest.addr[3],
                            ethhdr->dest.addr[4],
                            ethhdr->dest.addr[5]));
  LWIP_DEBUGF(NETIF_DEBUG, ("Source: %02" X8_F "-%02" X8_F "-%02" X8_F "-%02" X8_F "-%02" X8_F "-%02" X8_F "\n",
                            ethhdr->src.addr[0],
                            ethhdr->src.addr[1],
                            ethhdr->src.addr[2],
                            ethhdr->src.addr[3],
                            ethhdr->src.addr[4],
                            ethhdr->src.addr[5]));
  LWIP_DEBUGF(NETIF_DEBUG, ("Packet Type:0x%04" U16_F " \n", ethhdr->type));
}
#endif /* NETIF_DEBUG */
