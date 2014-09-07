/**************************************************************************//**
 * @file httpd.c
 * @brief This file is derived from the ``httpd.c'' skeleton.
 * @author Silicon Labs
 * @@version x.xx
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
 
#include "lwip/tcp.h"
#include "lwip/ip_addr.h"
/* This is the data for the actual web page. */
extern int  temperature;

char        indexdata[700];
static char indexdata1[] =
  "HTTP/1.0 200 OK\r\n\
Content-type: text/html\r\n\
Pragma: no-cache\r\n\
Refresh: 1\r\n\
\r\n\
<html>\
<head><title>Hello World</title><head>\
<body>\
<h1>Hello World!</h1>  <br><br>\
<B>This page is served from an EFM32G890F128 and uses the lwIP 1.3.1 TCP/IP stack.</B>\
<br><br>\
  <B> Number of times this page was refreshed: "                                                                                                                                                                                                                                                                                                                  ;

static char indexdata3[] =
  "<br><br>\
  <B> EFM32G890F128 temperature is now: "              ;

static char indexdata4[] =
  " C</B>\
</body>\
</html>"                      ;

/* This is the callback function that is called
 * when a TCP segment has arrived in the connection. */
static err_t http_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
  (void) arg;
  (void) err;
  
  static unsigned char temp_var = 0;
  unsigned short       counter, i;
  char                 *rq;
  /* If we got a NULL pbuf in p, the remote end has closed
   * the connection. */
  if (p != NULL)
  {
    /* The payload pointer in the pbuf contains the data
     * in the TCP segment. */
    rq = p->payload;
    /* Check if the request was an HTTP "GET / HTTP/1.1". */
    if (rq[0] == 'G' && rq[1] == 'E' && rq[2] == 'T')
    {
      /* Send the web page to the remote host. A zero
       * in the last argument means that the data should
       * not be copied into internal buffers. */

      counter = 0;

      for (i = 0; i < sizeof(indexdata1) - 1; i++)
      {
        indexdata[counter] = indexdata1[i];
        counter++;
      }

      indexdata[counter] = ' ';
      counter++;

      /*Copy page counter MSB*/
      indexdata[counter] = temp_var / 10 + 0x30;
      counter++;

      /*Copy page counter LSB*/
      indexdata[counter] = temp_var % 10 + 0x30;
      counter++;

      temp_var++;
      if (temp_var > 100) temp_var = 1;

      for (i = 0; i < sizeof(indexdata3) - 1; i++)
      {
        indexdata[counter] = indexdata3[i];
        counter++;
      }

      /*Set temperature sign*/
      if (temperature < 0)
      {
        indexdata[counter] = '-';
        counter++;
      }

      /*Copy temperature MSB*/
      indexdata[counter] = temperature / 10 + 0x30;
      counter++;

      /*Copy temperature LSB*/
      indexdata[counter] = temperature % 10 + 0x30;
      counter++;

      for (i = 0; i < sizeof(indexdata4); i++)
      {
        indexdata[counter] = indexdata4[i];
        counter++;
      }


      tcp_write(pcb, indexdata, counter, 1);
    }
    /* Free the pbuf. */
    pbuf_free(p);
  }
  /* Close the connection. */
  tcp_close(pcb);
  return ERR_OK;
}

/* This is the callback function that is called when
 * a connection has been accepted. */
static err_t http_accept(void *arg, struct tcp_pcb *pcb, err_t err)
{
  (void) arg;
  (void) err;
  
  /* Set up the function http_recv() to be called when data
   * arrives. */
  tcp_recv(pcb, http_recv);
  return ERR_OK;
}

/* The initialization function. */
void httpd_init(void)
{
  struct tcp_pcb *pcb;
  /* Create a new TCP PCB. */
  pcb = tcp_new();
  /* Bind the PCB to TCP port 80. */
  tcp_bind(pcb, NULL, 80);
  /* Change TCP state to LISTEN. */
  pcb = tcp_listen(pcb);
  /* Set up http_accet() function to be called
   * when a new connection arrives. */
  tcp_accept(pcb, http_accept);
}

