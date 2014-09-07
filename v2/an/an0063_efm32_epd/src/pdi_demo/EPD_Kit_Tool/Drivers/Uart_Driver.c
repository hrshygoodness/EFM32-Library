/**
 * \file
 *
 * \brief The functions of UART driver
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

#include "EPD_Kit_Tool_Process.h"
#include "Uart_Driver.h"
#include "USB_Driver.h"

static receive_event_handler _RxEventHandle;
uint8_t tx_buf[SERIAL_TX_MAX_LEN];

static void USB_REV(uint8_t *data, int byteCount)
{
	if(_RxEventHandle!=NULL) _RxEventHandle(data,byteCount);
}
/** \brief Initialize the Rx event and start USB Device stack
 */
void data_interface_init(receive_event_handler OnRxEventHandle) {
	_RxEventHandle=OnRxEventHandle;
    	USB_Driver_Init(USB_REV);
}

void data_interface_detach(void) {
	USB_Driver_Connect();
}

/** \brief Transmit data to UART
 */
void data_transmit(uint8_t *s, uint8_t len) {
	USB_Send(s,len);
}


