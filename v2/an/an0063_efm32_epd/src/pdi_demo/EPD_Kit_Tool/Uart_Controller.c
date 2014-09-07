/**
* \file
*
* \brief The functions to control and return system packets
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
#include "Uart_Controller.h"


static receive_packets_event _receive_packets_event;
static serial_buffer_t data_Rx_buffer;
/** declare the number of system packet used interchangeably, default=2 */
static system_packets_t system_packets[__System_Buffer_Size];
static uint8_t system_packet_count;
static uint8_t system_packet_get_index, system_packet_put_index;

/**
* \brief Clear system packet buffer  */
static void clear_system_buffer(void) {
	system_packet_count=0;
	system_packet_get_index=0;
	system_packet_put_index=0;
}

/**
* \brief Input data to system packet buffer
*
* \param system_packet The received new system packet
* \param length The length of received system packet
*
* \return There is buffer to be used or not
*/
static uint8_t put_system_buffer(uint8_t *system_packet,uint8_t length) {
	memcpy ((void *)(&system_packets[system_packet_put_index]), system_packet,length-1);
	if(system_packet_count < __System_Buffer_Size) {
		system_packet_count++;
		system_packet_put_index++;
		system_packet_put_index &=__System_Buffer_Mark;
		return TRUE;
	}
	return FALSE;
}

/**
* \brief Get data from system packet buffer
*
* \return The system packet buffer index
*/
static void get_system_buffer(system_packets_t * value) {
  if(system_packet_count >0) {
	   memcpy (value, (void *)(&system_packets[system_packet_get_index]), 
               system_packets[system_packet_get_index].packet_length-1);
       system_packet_count--;
       system_packet_get_index++;
       system_packet_get_index &=__System_Buffer_Mark; 
   } 
}

/**
* \brief Return the number of system packet buffer has been used
*/
static uint8_t number_of_system_buffer(void) {
	return system_packet_count;
}

/**
 * \brief Check the format of system packets
 */
static void system_packets_format_check(void) {
	uint8_t i,len,crc;
	uint8_t  tmp_data[__System_Packet_Length_Max];
	if(data_Rx_buffer.count>=__System_Packet_Length_Min) {
		/** the system packet header of extension board with EPD Kit Tool is 0xB3 */
		if(data_Rx_buffer.buffer[data_Rx_buffer.get_index]==__System_Packet_Header) {
			len=data_Rx_buffer.buffer[data_Rx_buffer.get_index+Sys_Packets_Length];
			if(len<__System_Packet_Length_Min || len>__System_Packet_Length_Max) {
				data_Rx_buffer.count--;
				data_Rx_buffer.get_index++;
			} else if(len<=data_Rx_buffer.count) {
				crc=0;
				for(i=0; i<len; i++) {
					/** Generate CRC byte for checking later */
					crc^=data_Rx_buffer.buffer[(uint8_t)(i+data_Rx_buffer.get_index)
					                           & __System_Packet_Length_Mark];
					tmp_data[i]=data_Rx_buffer.buffer[(uint8_t)(i+data_Rx_buffer.get_index)
					                                  & __System_Packet_Length_Mark];
				}
				if(crc==0) {
					put_system_buffer((uint8_t *)tmp_data,len);
					data_Rx_buffer.count-=len;
					data_Rx_buffer.get_index+=len;
				} else {
					data_Rx_buffer.count--;
					data_Rx_buffer.get_index++;
				}
			}
		} else {
			data_Rx_buffer.count--;
			data_Rx_buffer.get_index++;
		}
		data_Rx_buffer.get_index &= __System_Packet_Length_Mark;
	}
}

/**
 * \brief Save receiving data to buffer
 *
 * \param data The address pointer of receiving system packet
 * \param len The data length of receiving system packet
 */
static void data_receive_handle(uint8_t *data,uint8_t len) {
	while(len--) {
		data_Rx_buffer.buffer[data_Rx_buffer.put_index] = *data++;
		data_Rx_buffer.put_index++;
		data_Rx_buffer.put_index &=__System_Packet_Length_Mark;
		data_Rx_buffer.count++;
		data_Rx_buffer.count &=__System_Packet_Length_Mark;
	}
	system_packets_format_check();
}

/**
 * \brief Polling data from system packet buffer
 */
void poll_system_packet_buffer(void) {
    system_packets_t tmpsystem_packets;
	if(number_of_system_buffer()>0) {
		if(_receive_packets_event!=NULL) {
            get_system_buffer(&tmpsystem_packets);
			_receive_packets_event(&tmpsystem_packets);
		} else get_system_buffer(&tmpsystem_packets);
	}
}

/**
 * \brief Return system packet
 *
 * \param packet The address pointer of system packet that will return
 */
void return_system_packets(system_packets_t *packet) {
	uint8_t i;
	static uint8_t *buf;
	buf=(uint8_t *)packet;
	buf[packet->packet_length-1]=0;
	for(i=0; i<packet->packet_length-1; i++) {
		buf[packet->packet_length-1]^=buf[i];
	}
	data_transmit(buf,packet->packet_length);
}

/**
 * \brief Return system packet result
 *
 * \param packet The address pointer of system packet
 * \param data The data byte will return
 * \param len The length of return data packet
 */
void return_packets(system_packets_t *packet,uint8_t *data,uint8_t len) {
	uint8_t i;
	uint8_t *buf;
	packet->packet_length=6+len;
	memcpy((uint8_t *)&packet->data[0],data,len);
	buf=(uint8_t *)packet;
	buf[packet->packet_length-1]=0;
	for(i=0; i<packet->packet_length-1; i++) {
		buf[packet->packet_length-1]^=buf[i];
	}
	data_transmit(buf,packet->packet_length);
}

/**
 * \brief Return system packet result
 *
 * \param packet The address pointer of system packet
 * \param result Success or failure
 */
void return_system_packet_result(system_packets_t *packet,uint8_t result) {
	return_packets(packet,(uint8_t *)&result,1);
}

/**
* \brief Initialize the UART data buffer and trigger receiving (Rx) packets
*
* \param receive_packets_event For trigger Rx packet
*/
void data_controller_init(receive_packets_event OnRxPacketEvent) {
	data_Rx_buffer.count=0;
	data_Rx_buffer.get_index=0;
	data_Rx_buffer.put_index=0;
	clear_system_buffer();
	data_interface_init(data_receive_handle);
	_receive_packets_event=OnRxPacketEvent;
}
