/**
 * \file
 *
 * \brief The functions that working with EPD Kit Tool
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

#include <Pervasive_Displays_small_EPD.h>
#include "EPD_Kit_tool_Process.h"

slideshow_information_t slideshow_parameter;
long write_flash_address;
static image_information_t image_info;
uint8_t board_is_connected;
uint16_t image_count;
uint8_t  line_count,rest_data_count;
uint16_t address_offset;
uint8_t slideshow_index;

/** \brief Check the EPD extension board
 *
 * \return The EPD extension board is connected(1) or not(0)
 */
static void check_EPD_extension_board(void) {
	if(is_flash_existed()==1)board_is_connected=1;
	else board_is_connected=0;
}

/**
 * \brief Read Flash data into buffer
 *
 * \param flash_address The start address of Flash
 * \param target_buffer The target address of buffer will be read
 * \param byte_length The data length will be read
 */
static void read_flash_handle(long flash_address,uint8_t *target_buffer,
                              uint8_t byte_length) {
	read_flash(flash_address,target_buffer,byte_length);
}

/**
 * \brief Execute slideshow function
 *
 * \return 1=Continue to run slideshow, 0=stop running slideshow
 */
static uint8_t slideshow_run(void) {

	image_info.EPD_size=slideshow_parameter.EPD_size;
	image_info.image_index=slideshow_index;
	/** Read new image */
	image_info.extend_address.slideshow_image_address=
	    get_slideshow_image_address(image_info.EPD_size,image_info.image_index,FALSE);

	if(slideshow_index==0)image_info.image_index=slideshow_parameter.image_end_index+1;

	/** Read previous image */
	image_info.previous_image_address=
	    get_slideshow_image_address(image_info.EPD_size,((image_info.image_index-1) &
	                                (_image270_slideshow_page_max-1)),FALSE);

	if(image_info.EPD_size>EPD_270) return 0;

	/** Show image on EPD from Flash*/
	EPD_display_from_flash(image_info.EPD_size,image_info.previous_image_address,
	                       image_info.extend_address.custom_image_address,read_flash_handle);

	image_info.extend_address.custom_image_address=_NULL_address;
	slideshow_index++;

	/** Reset from beginning */
	if(slideshow_index>slideshow_parameter.image_end_index)
		slideshow_index=slideshow_parameter.image_start_index;

	LED_OFF();
	return 1;
}


/**
* \brief Define UART command packet (system packet) and work flow by command type
*
* \param packet The system packet
*/
static void uart_command_handle(system_packets_t * packet) {
	int16_t tmp2=0;
	uint8_t tmp=0,tmp3=0;
	ASCII_info_t tmp_ASCII_info;
	switch(packet->command_type) {
	case __Kit_ID:
		packet->packet_length+=2; // return 2 data bytes
		packet->data[0]=(uint8_t)(KitID_Number >> 8); /*!< IC vendor */
		packet->data[1]=(uint8_t)(KitID_Number & 0xFF); /*!< Development kit */
		return_system_packets(packet);
		break;
	case __Temperature:
		initialize_temperature();
		packet->packet_length+=1; // return 1 data byte
		tmp2=get_temperature();
		packet->data[0]=tmp2;
		return_system_packets(packet);
		break;
	case __EPD_Board:
		spi_attach();
		packet->packet_length+=1; // return 1 data byte
		check_EPD_extension_board();
		check_EPD_extension_board();
		packet->data[0]=board_is_connected;
		return_system_packets(packet);
		break;
	case __Firmware_Version:
		packet->packet_length+=4; // return 4 data bytes
		memcpy ((uint8_t *)&packet->data[0], (uint8_t *)Firmware_Version,4);
		return_system_packets(packet);
		break;

		/** Send image process from host:
		    send clear image command - return true - send load image command - return true - send show image command */
	case __Clear_Image:
	case __Clear_ASCII:
	case __Clear_Custom_Image:
	case __Clear_Slideshow_Image:
		memcpy ((uint8_t *)&image_info, (uint8_t *)&packet->data[0], sizeof(image_information_t)-4);
		get_flash_image_info(&image_info);
		EPD_power_init(image_info.EPD_size);
		if(packet->command_type==__Clear_Image || packet->command_type==__Clear_ASCII) {
			image_info.extend_address.mark_image_address= get_flash_mark_image_info(image_info.EPD_size);
			write_flash_address=image_info.new_image_address;
		} else if(packet->command_type==__Clear_Custom_Image) {
			image_info.extend_address.custom_image_address= get_custom_image_address(image_info.EPD_size,
			        image_info.image_index,TRUE);
			write_flash_address=image_info.extend_address.custom_image_address;
		} else if(packet->command_type== __Clear_Slideshow_Image) {
			image_info.extend_address.slideshow_image_address= get_slideshow_image_address(image_info.EPD_size,
			        image_info.image_index,TRUE);
			write_flash_address=image_info.extend_address.slideshow_image_address;
		}
		image_count=image_info.number_of_images;
		address_offset=0;
		LED_Trigger();
		//write image header to flash
		write_mark(write_flash_address);
		return_system_packet_result(packet,TRUE);
		break;
	case __Load_Image:
	case __Load_Custom_Image:
	case __Load_Slideshow_Image:
		/** Deal with data packet as line data then as image data */
		if(image_count>0) {
			LED_Trigger();
			tmp3=0;
			tmp2=COG_parameters[image_info.EPD_size].horizontal_size;
			line_count =(packet->packet_length-6)+address_offset; // -6 to remove packet header
			rest_data_count =(uint8_t)(line_count%tmp2);
			line_count =(uint8_t)(line_count/tmp2);
			for(tmp=0; tmp<line_count; tmp++) {
				write_flash((write_flash_address+address_offset),(uint8_t *)&packet->data[tmp3],
				            tmp2-address_offset);
				write_flash_address+=_flash_line_size;
				tmp3+=(tmp2-address_offset);
				address_offset=0;
			}
			if(rest_data_count>0 && line_count>0) {
				write_flash((write_flash_address+address_offset),
				            (uint8_t *)&packet->data[tmp3],rest_data_count);
			} else if(rest_data_count>0) {
				write_flash((write_flash_address+address_offset),
				            (uint8_t *)&packet->data[tmp3],(rest_data_count-address_offset));
			}
			address_offset=rest_data_count;
			if((--image_count)==0) {
				return_system_packet_result(packet,TRUE);
			}
		}
		break;
	case __Load_ASCII:
		memcpy ((uint8_t *)&tmp_ASCII_info, (uint8_t *)&packet->data[0], sizeof(ASCII_info_t));
		write_ascii(image_info.new_image_address,image_info.extend_address.mark_image_address,
		           tmp_ASCII_info.x,tmp_ASCII_info.y,(char *)&tmp_ASCII_info.str);
		return_system_packet_result(packet,TRUE);
		break;

	case __Show_Image:
		EPD_display_from_flash_Ex(image_info.EPD_size,image_info.previous_image_address,
		                          image_info.new_image_address,read_flash_handle);
		image_info.extend_address.last_address=_NULL_address;
		image_info.previous_image_address=image_info.new_image_address;
		return_system_packet_result(packet,TRUE);
		break;
	case __Show_Custom_Image:
		EPD_display_from_flash_Ex(image_info.EPD_size,image_info.previous_image_address,
		                          image_info.extend_address.custom_image_address,read_flash_handle);
		image_info.previous_image_address=image_info.extend_address.custom_image_address;
		return_system_packet_result(packet,TRUE);
		break;
	case __Show_Slideshow_Image:
		EPD_display_from_flash_Ex(image_info.EPD_size,image_info.previous_image_address,
		                          image_info.extend_address.slideshow_image_address,read_flash_handle);
		image_info.previous_image_address=image_info.extend_address.slideshow_image_address;
		return_system_packet_result(packet,TRUE);
		break;
	case __Show_ASCII:
		EPD_display_partialupdate(image_info.EPD_size,image_info.previous_image_address,image_info.new_image_address,
		                          image_info.extend_address.mark_image_address,read_flash_handle);
		image_info.previous_image_address=image_info.new_image_address;
		image_info.extend_address.last_address=_NULL_address;
		return_system_packet_result(packet,TRUE);
		break;
	case __Show_Index_Custom_Image:
		memcpy ((uint8_t *)&image_info, (uint8_t *)&packet->data[0], sizeof(image_information_t)-4);
		image_info.previous_image_address=image_info.extend_address.custom_image_address;
		image_info.extend_address.custom_image_address= get_custom_image_address(image_info.EPD_size,image_info.image_index,FALSE);
		EPD_display_from_flash(image_info.EPD_size,image_info.previous_image_address,
		                       image_info.extend_address.custom_image_address,read_flash_handle);
		image_info.previous_image_address=image_info.extend_address.custom_image_address;
		return_system_packet_result(packet,TRUE);
		break;

	case __Slideshow_On:
		memcpy ((uint8_t *)&slideshow_parameter, (uint8_t *)&packet->data[0], sizeof(slideshow_information_t));
		write_slideshow_parameters(&slideshow_parameter);
		slideshow_index=slideshow_parameter.image_start_index;
		return_system_packet_result(packet,TRUE);
		LED_ON();
		slideshow_run();
		start_EPD_timer();
		break;
	case __Slideshow_Off:
		slideshow_parameter.EPD_size=0x0;
		slideshow_parameter.interval=0x0;
		write_slideshow_parameters(&slideshow_parameter);
		stop_EPD_timer();
		spi_detach ();
		return_system_packet_result(packet,TRUE);
		break;

	case __Reload_Current_Image:
		return_system_packet_result(packet,TRUE);
		EPD_display_from_flash(image_info.EPD_size,image_info.previous_image_address,image_info.previous_image_address,read_flash_handle);
		break;

	case __Trigger_LED:
		LED_Trigger();
		return_system_packet_result(packet,TRUE);
		break;

	case __Clear_All_Flash:
		spi_attach();
		CMD_CE();
		delay_ms(500);
		return_system_packet_result(packet,TRUE);
		break;
	}
}

/**
 * \brief Initialize the UART data buffer, check extension board is connected and
 *        and check slideshow is enabled
 */
void EPD_Kit_Tool_process_init(void) {
	/** Initialize the UART data buffer and start receiving system packets */
	data_controller_init(uart_command_handle);
	delay_ms(1000);
	check_EPD_extension_board();
	image_info.previous_image_address=_NULL_address;
	image_info.extend_address.last_address=_NULL_address;
	if(board_is_connected==1) {
		/** Check if slideshow is enabled */
		read_slideshow_parameters(&slideshow_parameter);
		if(slideshow_parameter.interval>0) {
			LED_Trigger();
			if(slideshow_run()) {
				start_EPD_timer();
				LED_ON();
			}
		}
	}
}

/**
 * \brief Polling system packet to data buffer
 */
void EPD_Kit_tool_process_task(void) {

	poll_system_packet_buffer();

	/** Run slideshow if interval has value and not zero */
	if(slideshow_parameter.interval>0 && slideshow_parameter.interval!=0xff) {
		if(get_current_time_tick()>=(slideshow_parameter.interval*1000)) {
			if(slideshow_run()) {
				start_EPD_timer();
				LED_Trigger();
			}
		}
	}
}
