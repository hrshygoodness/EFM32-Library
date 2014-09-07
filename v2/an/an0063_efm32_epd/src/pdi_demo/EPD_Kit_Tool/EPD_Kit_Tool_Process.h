/**
 * \file
 *
 * \brief The definition of EPD Kit Tool
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

#ifndef EPD_KIT_TOO_PROCESS_H_
#define EPD_KIT_TOO_PROCESS_H_

#include <Pervasive_Displays_small_EPD.h>

/**
 * \brief Structure of the image information  */
typedef struct {
	uint8_t  EPD_size;                /**< the EPD size */
	uint8_t	 image_index;             /**< the page index of image in memory segment B */
	uint16_t number_of_images;        /**< the number of image have been loaded */
	long     new_image_address;       /**< the new image address */
	long 	 previous_image_address;  /**< the previous image address */
	union {
		long last_address;            /**< the last image address for "reload" function used*/
		long mark_image_address;      /**< the mark image address for "ASCII" function used*/
		long slideshow_image_address; /**< the slideshow image address */
		long custom_image_address;    /**< the custom image address */
	} extend_address;
} image_information_t;


/**
 * \brief Structure of the slideshow information  */
typedef struct {
	uint8_t    EPD_size;          /**< the EPD size */
	uint8_t    interval;          /**< the interval between images */
	uint8_t    image_start_index; /**< the start index of slideshow images */
	uint8_t    image_end_index;   /**< the end index of slideshow images */
} slideshow_information_t;

#define Firmware_Version EPD_KIT_TOOL_VERSION
#define KitID_Number     EPD_KIT_TOOL_ID

#if !defined(BUFFER_SIZE)
#define BUFFER_SIZE		2
#endif

#if !defined(PAYLOAD_SIZE)
#define PAYLOAD_SIZE	64
#endif

#include "EPD_Led.h"
#include "Char.h"
#include "Mem_Flash.h"
#include "Uart_Driver.h"
#include "Uart_Controller.h"

void EPD_Kit_Tool_process_init(void);
void EPD_Kit_tool_process_task(void);
extern void EPD_display_partialupdate (uint8_t EPD_type_index, long previous_image_address,
                                       long new_image_address,long mark_image_address,
                                       EPD_read_flash_handler On_EPD_read_flash);
#endif /* EPD_KIT_TOO_PROCESS_H_ */

