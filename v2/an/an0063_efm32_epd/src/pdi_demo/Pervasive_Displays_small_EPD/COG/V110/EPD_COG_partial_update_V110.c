/**
 * \file
 *
 * \brief The partial update waveform of driving processes and updating stages
 of G1 COG with V110 EPD
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

#include "EPD_COG_process.h"

extern void read_flash(long Address, uint8_t *target_address,
		uint8_t byte_length);
extern void write_flash(long Address, uint8_t *source_address,
		uint8_t byte_length);
extern void erase_image(long address, uint8_t ptype);
void EPD_display_partialupdate(uint8_t EPD_type_index,
		long previous_image_address, long new_image_address,
		long mark_image_address, EPD_read_flash_handler On_EPD_read_flash);
#define _flash_line_size	64 //bytes of a line size in flash
/**
 * \brief Save the image combined with inputted ASCII string for next
 *
 * \param EPD_type_index The defined EPD size
 * \param previous_image_address The previous image address
 * \param new_image_address The new (canvas) image address
 * \param mark_image_address The mark image address
 */
static void save_partial_image(uint8_t EPD_type_index, long previous_address,
		long new_address, long mark_address) {
	uint16_t y, x;
	long mark_image_original = mark_address;
	uint8_t *previous_lin, *new_line, *mark_line;
	spi_attach();
	previous_lin = (uint8_t*) malloc(
			COG_parameters[EPD_type_index].horizontal_size);
	new_line = (uint8_t*) malloc(
			COG_parameters[EPD_type_index].horizontal_size);
	mark_line = (uint8_t*) malloc(
			COG_parameters[EPD_type_index].horizontal_size);

	for (y = 0; y <= COG_parameters[EPD_type_index].vertical_size; y++) {
		read_flash(previous_address, previous_lin,
				COG_parameters[EPD_type_index].horizontal_size);
		read_flash(new_address, new_line,
				COG_parameters[EPD_type_index].horizontal_size);
		read_flash(mark_address, mark_line,
				COG_parameters[EPD_type_index].horizontal_size);
		for (x = 0; x < COG_parameters[EPD_type_index].horizontal_size; x++) {
			/** Only move the non-marked area of previous image. Keep ASCII text */
			if (mark_line[x] == 0xFF) {
				new_line[x] = previous_lin[x];
			}
		}
		write_flash(new_address, new_line,
				COG_parameters[EPD_type_index].horizontal_size);
		previous_address += _flash_line_size;
		new_address += _flash_line_size;
		mark_address += _flash_line_size;
	}

	free(previous_lin);
	free(new_line);
	free(mark_line);

	//Clear Mark Image
	erase_image(mark_image_original, EPD_type_index);
	spi_detach();
}

/**
 * \brief The driving stages for getting Odd/Even data and writing the data
 * from Flash memory to COG using partial update
 *
 * \note
 * - Refer to stage_handle_array comment note in EPD_COG_process_V110.c
 * - Partial update uses one buffer driving method which is discarded the previous
 *   image compensate and run 2 stages only.
 *   -# The 1st stage is black/white alternately
 *   -# The 2nd stage is to show new image
 * - just the marked area will be changed.
 *
 * \param EPD_type_index The defined EPD size
 * \param previous_image_address The previous image address
 * \param new_image_address The new (canvas) image address
 * \param mark_image_address The mark image address
 * \param stage_no The assigned stage number that will proceed
 */
static void stage_handle_partial_update(uint8_t EPD_type_index,
		long previous_image_address, long new_image_address,
		long mark_image_address, uint8_t stage_no) {
	/** x for horizontal_size loop, y for vertical_size loop, which are EPD pixel size */
	uint16_t x, y, k;
	static volatile uint8_t temp_byte=0; // Temporary storage for image data check
	long address_offset;
	uint8_t *new_line, *mark_line;
	uint8_t frame_count; //count for sending black or white
	/** Get line data array of EPD size */
	COG_driver_EPDtype_select(EPD_type_index);

	new_line = (uint8_t*) malloc(
			COG_parameters[EPD_type_index].horizontal_size);
	mark_line = (uint8_t*) malloc(
			COG_parameters[EPD_type_index].horizontal_size);

	current_frame_time = COG_parameters[EPD_type_index].frame_time_offset;

	/* Start a system SysTick timer to ensure the same duration of each stage  */
	start_EPD_timer();

	/* Do while total time of frames exceed stage time
	 * Per frame */
	do {
		address_offset = 0;
		frame_count = 0;
		/* Per data line (vertical size) */
		for (y = 0; y < COG_parameters[EPD_type_index].vertical_size; y++) {
			/* Set charge pump voltage level reduce voltage shift */
			SPI_send_byte(0x04, COG_parameters[EPD_type_index].voltage_level);

			k = COG_parameters[EPD_type_index].horizontal_size - 1;
			if (_On_EPD_read_flash != NULL) {
				_On_EPD_read_flash((new_image_address + address_offset),
						new_line,
						COG_parameters[EPD_type_index].horizontal_size);
				_On_EPD_read_flash((mark_image_address + address_offset),
						mark_line,
						COG_parameters[EPD_type_index].horizontal_size);
			}
			/** Per dot/pixel */
			for (x = 0; x < COG_parameters[EPD_type_index].horizontal_size;
					x++) {
				switch (stage_no) {
				case Stage1: /*!< black/white alternately */
					/** change the pixel(x) of marked line only if not 0xFF.
					 If 0xFF, send Nothing(01) means not change.
					 The other pixels, send white(10)/black(11) alternately by frame_count. */
					if (mark_line[x] == 0xFF) {
						data_line_odd[x] = NOTHING; //Nothing(01), 01010101
						data_line_even[k--] = NOTHING;
					} else {
						if ((frame_count % 2)) {
							data_line_odd[x] = 0xAA; //white(10), 10101010
							data_line_even[k--] = 0xAA;

						} else {
							data_line_odd[x] = 0xFF; //black(11), 11111111
							data_line_even[k--] = 0xFF;
						}
					}
					break;

				case Stage2: /*!< new image */
					/** change the pixel(x) of marked line only if not 0xFF.
					 If 0xFF, send Nothing(01) means not change.
					 The other pixels, send new image data. */
					if (mark_line[x] == 0xFF) {
						data_line_odd[x] = NOTHING;
						data_line_even[k--] = NOTHING;
					} else {
						temp_byte = new_line[x];
						/** Example to get Even and Odd data
						 * +---------+----+----+----+----+----+----+----+----+
						 * |         |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
						 * |temp_byte+----+----+----+----+----+----+----+----+
						 * |         |  1 |  0 |  1 |  1 |  0 |  1 |  0 |  0 |
						 * +---------+----+----+----+----+----+----+----+----+ */
						data_line_odd[x]  = ((temp_byte & 0x80) ? WHITE3 : BLACK3);
						data_line_odd[x] |= ((temp_byte & 0x20) ? WHITE2 : BLACK2);
						data_line_odd[x] |= ((temp_byte & 0x08) ? WHITE1 : BLACK1);
						data_line_odd[x] |= ((temp_byte & 0x02) ? WHITE0 : BLACK0);
						/** WHITE3 = 0x80 = 1000 0000, WHITE2 = 0x20 = 0010 0000
						 *  BLACK1 = 0x0C = 0000 1100, BLACK0 = 0x03 = 0000 0011
						 *  data_line_odd[] = 1000 0000 | 0010 0000 | 0000 1100 | 0000 0011 = 1010 1111 */
						data_line_even[k]    = ((temp_byte & 0x01) ? WHITE3 : BLACK3);
						data_line_even[k]   |= ((temp_byte & 0x04) ? WHITE2 : BLACK2);
						data_line_even[k]   |= ((temp_byte & 0x10) ? WHITE1 : BLACK1);
						data_line_even[k--] |= ((temp_byte & 0x40) ? WHITE0 : BLACK0);
						/** BLACK3 = 0xC0 = 1100 0000, WHITE2 = 0x20 = 0010 0000
						 *  WHITE1 = 0x08 = 0000 1000, BLACK0 = 0x03 = 0000 0011
						 *  data_line_even[] = 1100 0000 | 0010 0000 | 0000 1000 | 0000 0011 = 1110 1011
						 * +---------+----+----+----+----+----+----+----+----+
						 * |         |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
						 * |temp_byte+----+----+----+----+----+----+----+----+
						 * |         |  1 |  0 |  1 |  1 |  0 |  1 |  0 |  0 |
						 * +---------+----+----+----+----+----+----+----+----+
						 * | Color   |  W |  B |  W |  W |  B |  W |  B |  B | W=White, B=Black, N=Nothing
						 * +---------+----+----+----+----+----+----+----+----+
						 * | Stage 2 |  W |  B |  W |  W |  B |  W |  B |  B |
						 * +---------+----+----+----+----+----+----+----+----+
						 * | Input   | 10 | 11 | 10 | 10 | 11 | 10 | 11 | 11 | W=10, B=11, N=01
						 * +---------+----+----+----+----+----+----+----+----+
						 * |Odd data | 10 |    | 10 |    | 11 |    | 11 |    | = 1010 1111
						 * +---------+----+----+----+----+----+----+----+----+
						 * |Even data|    | 11 |    | 10 |    | 10 |    | 11 | = 1110 1011
						 * +---------+----+----+----+----+----+----+----+----+ */
					}
					break;
				}
			}
			address_offset += LINE_SIZE;
			/* Scan byte shift per data line */
			data_line_scan[(y >> 2)] = SCAN_TABLE[(y % 4)];

			/* For 1.44 inch EPD, the border uses the internal signal control byte. */
			if (EPD_type_index == EPD_144)
				COG_Line.line_data_by_size.line_data_for_144.border_byte = 0x00;

			/* Sending data */
			SPI_send(0x0A, (uint8_t *) &COG_Line.uint8,
					COG_parameters[EPD_type_index].data_line_size);

			/* Turn on Output Enable */
			SPI_send_byte(0x02, 0x2F);

			data_line_scan[(y >> 2)] = 0;
		}
		/* Count the frame time with offset */
		current_frame_time = (uint16_t) get_current_time_tick()
				+ COG_parameters[EPD_type_index].frame_time_offset;
		frame_count++;
	} while (stage_time > current_frame_time);

	/* Do while the SysTick timer fulfills the stage time */
	while (stage_time > get_current_time_tick())
		;
	free(new_line);
	free(mark_line);
	/* Stop system timer */
	stop_EPD_timer();
}

/**
 * \brief Write image data from Flash memory to the EPD using partial update
 *
 * \param EPD_type_index The defined EPD size
 * \param previous_image_address The previous image address
 * \param new_image_address The new (canvas) image address
 * \param mark_image_address The mark image address
 * \param On_EPD_read_flash Developer needs to create an external function to read flash
 */
void EPD_display_partialupdate(uint8_t EPD_type_index,
		long previous_image_address, long new_image_address,
		long mark_image_address, EPD_read_flash_handler On_EPD_read_flash) {

	_On_EPD_read_flash = On_EPD_read_flash;
	/** partial update uses two stages: black/white and new image */
	stage_handle_partial_update(EPD_type_index, previous_image_address,
			new_image_address, mark_image_address, Stage1);
	stage_handle_partial_update(EPD_type_index, previous_image_address,
			new_image_address, mark_image_address, Stage2);

	/** Power off COG Driver */
	EPD_power_off(EPD_type_index);
	/** Save image combines with ASCII text  */
	if (previous_image_address != new_image_address) {
		save_partial_image(EPD_type_index, previous_image_address,
				new_image_address, mark_image_address);
	}
}
