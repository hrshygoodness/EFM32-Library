/**
* \file
*
* \brief The definition of system packet and command type
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

#ifndef __DATACONTROLLER_H_
#define __DATACONTROLLER_H_
#include "Uart_Driver.h"

/** The definition of system packet ******************************************/
#define   __System_Buffer_Size BUFFER_SIZE /*!< number of system packet used interchangeably, default=1 */
#define   __System_Buffer_Mark __System_Buffer_Size-1

#define  __System_Packet_Header      0xB3         /*!< 0xB3 header is for PDi Extension Kit */
#define  __System_Packet_Length_Min  6            /*!< Minimum system packet length without data[] */
#define  __System_Packet_Length_Max  PAYLOAD_SIZE /*!< Maximum system packet length */
#define  __System_Packet_Length_Mark (__System_Packet_Length_Max-1) /*!< System packet maximum position */

/** The definition of command type *******************************************/
#define  __Kit_ID                  0x10
#define  __Temperature             0x11
#define  __EPD_Board               0x12
#define  __Firmware_Version        0x1F

#define  __Clear_Image             0x20
#define  __Load_Image              0x21
#define  __Show_Image              0x22

#define  __Clear_ASCII             0x30
#define  __Load_ASCII              0x31
#define  __Show_ASCII              0x32

#define  __Clear_Custom_Image      0x40
#define  __Load_Custom_Image       0x41
#define  __Show_Custom_Image       0x42
#define  __Show_Index_Custom_Image 0x43

#define  __Clear_Slideshow_Image   0x50
#define  __Load_Slideshow_Image    0x51
#define  __Show_Slideshow_Image    0x52
#define  __Slideshow_On            0x53
#define  __Slideshow_Off           0x54

#define  __Reload_Current_Image    0x60
#define  __Clear_All_Flash         0x61
#define  __Trigger_LED             0x62

/******************************************************************/
enum 
{
     Sys_Packets_Header = 0,
     Sys_Packets_Length ,
     Sys_Packets_Address ,
     Sys_Packets_Command_Type      
};


/**
 * \brief System packet structure
 *
 * \note
 * ========================
 * | Byte | Name          |
 * |------|---------------|
 * | 0    | Packet Header |
 * | 1    | Packet Length |
 * | 2-3  | Kit ID        |
 * | 4    | Command Type  |
 * | 5-62 | Data          |
 * | 63   | CRC           |
 * - Header is __System_Packet_Header
 * - The CRC byte is at the end of packets excluding in this structure.
 */
typedef struct {
	uint8_t     packet_header;
	uint8_t     packet_length;
	uint16_t    kit_id;
	uint8_t     command_type;
	uint8_t     data[__System_Packet_Length_Max];
} system_packets_t;

typedef void (*receive_packets_event)(system_packets_t * packet);


/*****************************************************************/
void poll_system_packet_buffer(void);
void data_controller_init(receive_packets_event OnRxPacketEvent);
void return_system_packets(system_packets_t *packet);
void return_packets(system_packets_t *packet,uint8_t *Datas,uint8_t len);
void return_system_packet_result(system_packets_t *packet,uint8_t Result);

#endif
