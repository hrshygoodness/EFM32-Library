/**
* \file
*
* \brief The definition of UART driver
*
* Copyright (c) 2012-2013 Pervasive Displays Inc. All rights reserved.
*
*  Authors: Pervasive Displays Inc.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*  1. Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __UART_H_
#define __UART_H_

#define   SERIAL_TX_MAX_LEN         64
#define   SERIAL_RX_MAX_LEN          128

/**
 * \brief UART serial buffer structure
 */
typedef struct
{
    uint8_t put_index,get_index;
    uint8_t count;
    char buffer[SERIAL_RX_MAX_LEN];
} serial_buffer_t;

typedef void (*receive_event_handler)(uint8_t *Rx_data,uint8_t len);

void data_interface_init(receive_event_handler OnRxEventHandle);
void data_transmit (uint8_t *s,uint8_t len);
void data_interface_detach(void);
#endif

