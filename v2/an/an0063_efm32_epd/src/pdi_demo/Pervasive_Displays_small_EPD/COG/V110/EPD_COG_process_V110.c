/**
* \file
*
* \brief The waveform driving processes and updating stages of G1 COG with V110 EPD
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
/**
 * \brief The COG parameters of different EPD size
 */
const struct COG_parameters_t COG_parameters[COUNT_OF_EPD_TYPE] = {
	{
		// FOR 1.44"
		{0x00,0x00,0x00,0x00,0x00,0x0F,0xFF,0x00},
		0x03,
		(128/8),
		96,
		((((128+96)*2)/8)+1),
		0,
		480
	},
	{
		// For 2.0"
		{0x00,0x00,0x00,0x00,0x01,0xFF,0xE0,0x00},
		0x03,
		(200/8),
		96,
		((((200+96)*2)/8)+1),
		0,
		480
	},
	{
		// For 2.7"
		{0x00,0x00,0x00,0x7F,0xFF,0xFE,0x00,0x00},
		0x00,
		(264/8),
		176,
		((((264+176)*2)/8)+1),
		0,
		630
	}
};

/* Temperature factor combines with stage time for each driving stage */
const uint16_t temperature_table[3][8] = {
	{(480*17),(480*12),(480*8),(480*4),(480*3),(480*2),(480*1),(480*0.7)},
	{(480*17),(480*12),(480*8),(480*4),(480*3),(480*2),(480*1),(480*0.7)},
	{(630*17),(630*12),(630*8),(630*4),(630*3),(630*2),(630*1),(630*0.7)},
};

const uint8_t   SCAN_TABLE[4] = {0xC0,0x30,0x0C,0x03};
static uint16_t stage_time;
static COG_line_data_packet_type COG_Line;
static EPD_read_flash_handler _On_EPD_read_flash=NULL;
static uint16_t current_frame_time;
static uint8_t  *data_line_even;
static uint8_t  *data_line_odd;
static uint8_t  *data_line_scan;

/**
* \brief According to EPD size and temperature to get stage_time
* \note Refer to COG document Section 5.3 for more details
*
* \param EPD_type_index The defined EPD size
*/
static void set_temperature_factor(uint8_t EPD_type_index) {
	int8_t temperature;
	temperature = get_temperature();
	if (temperature <= -10) {
		stage_time = temperature_table[EPD_type_index][0];
		} else if (-5 >= temperature && temperature > -10) {
		stage_time = temperature_table[EPD_type_index][1];
		} else if (5 >= temperature && temperature > -5) {
		stage_time = temperature_table[EPD_type_index][2];
		} else if (10 >= temperature && temperature > 5) {
		stage_time = temperature_table[EPD_type_index][3];
		} else if (15 >= temperature && temperature > 10) {
		stage_time = temperature_table[EPD_type_index][4];
		} else if (20 >= temperature && temperature > 15) {
		stage_time = temperature_table[EPD_type_index][5];
		} else if (40 >= temperature && temperature > 20) {
		stage_time = temperature_table[EPD_type_index][6];
	} else stage_time = temperature_table[EPD_type_index][7];
}

/**
* \brief Initialize the EPD hardware setting
*/
void EPD_init(void) {
	EPD_display_hardware_init();
	EPD_cs_low();
	EPD_pwm_low();
	EPD_rst_low();
	EPD_discharge_low();
	EPD_border_low();
}


/**
* \brief Select the EPD size to get line data array for driving COG
*
* \param EPD_type_index The defined EPD size
*/
void COG_driver_EPDtype_select(uint8_t EPD_type_index) {
	switch(EPD_type_index) {
		case EPD_144:
		data_line_even = &COG_Line.line_data_by_size.line_data_for_144.even[0];
		data_line_odd  = &COG_Line.line_data_by_size.line_data_for_144.odd[0];
		data_line_scan = &COG_Line.line_data_by_size.line_data_for_144.scan[0];
		break;
		case EPD_200:
		data_line_even = &COG_Line.line_data_by_size.line_data_for_200.even[0];
		data_line_odd  = &COG_Line.line_data_by_size.line_data_for_200.odd[0];
		data_line_scan = &COG_Line.line_data_by_size.line_data_for_200.scan[0];
		break;
		case EPD_270:
		data_line_even = &COG_Line.line_data_by_size.line_data_for_270.even[0];
		data_line_odd  = &COG_Line.line_data_by_size.line_data_for_270.odd[0];
		data_line_scan = &COG_Line.line_data_by_size.line_data_for_270.scan[0];
		break;
	}
}


/**
* \brief Power on COG Driver
* \note For detailed flow and description, please refer to the COG document Section 3.
*/
void EPD_power_on (void) {
	/* Initial state */
	EPD_discharge_low();
	EPD_rst_low();
	EPD_cs_low();
	spi_init();
	spi_attach();

	PWM_run(5); //The PWM signal starts toggling
	EPD_Vcc_turn_on(); //Vcc and Vdd >= 2.7V
	PWM_run(10);
	EPD_cs_high(); // /CS=1
	EPD_border_high(); //BORDER=1
	EPD_rst_high(); // /RESET=1
	PWM_run(5);
	EPD_rst_low(); // /RESET=0
	PWM_run(5);
	EPD_rst_high(); // /RESET=1
	PWM_run(5);
}


/**
* \brief Initialize COG Driver
* \note For detailed flow and description, please refer to the COG document Section 4.
*
* \param EPD_type_index The defined EPD size
*/
uint8_t EPD_initialize_driver (uint8_t EPD_type_index) {
	uint8_t SendBuffer[2];
	uint16_t k;

	// Empty the Line buffer
	for (k = 0; k <= LINE_BUFFER_DATA_SIZE; k ++) {
		COG_Line.uint8[k] = 0x00;
	}
	// Determine the EPD size for driving COG
	COG_driver_EPDtype_select(EPD_type_index);

	// Sense temperature to determine Temperature Factor
	set_temperature_factor(EPD_type_index);
	k = 0;
	while (EPD_IsBusy()) {
		if((k++) >= 0x0FFF) return ERROR_BUSY;
	}
	// Channel select
	SPI_send (0x01, (uint8_t *)&COG_parameters[EPD_type_index].channel_select, 8);

	// DC/DC frequency setting
	SPI_send_byte (0x06, 0xFF);

	// High power mode OSC setting
	SPI_send_byte (0x07, 0x9D);

	// Disable ADC
	SPI_send_byte (0x08, 0x00);

	// Set Vcom level
	SendBuffer[0] = 0xD0;
	SendBuffer[1] = 0x00;
	SPI_send (0x09, SendBuffer, 2);

	// Gate and source voltage level
	SPI_send_byte (0x04,COG_parameters[EPD_type_index].voltage_level);
	PWM_run(5);

	// Driver latch on (cancel register noise)
	SPI_send_byte(0x03, 0x01);

	// Driver latch off
	SPI_send_byte(0x03, 0x00);

	// Start charge pump positive V VGH & VDH on
	SPI_send_byte (0x05, 0x01);
	PWM_run(30);

	// Start charge pump neg voltage VGL & VDL on
	SPI_send_byte (0x05, 0x03);
	delay_ms (30);

	// Set charge pump Vcom_Driver to ON
	SPI_send_byte(0x05, 0x0F);
	delay_ms(30);

	// Output enable to disable
	SPI_send_byte(0x02, 0x24);

	return RES_OK;
}

/**
 * \brief The driving stages for getting Odd/Even data and writing the data
 * from memory array to COG
 *
 * \note
 * - There are 4 stages to complete an image update on EPD.
 * - Each of the 4 stages time should be the same uses the same number of frames.
 * - One dot/pixel is comprised of 2 bits which are White(10), Black(11) or Nothing(01).
 *   The image data bytes must be divided into Odd and Even bytes.
 * - The COG driver uses a buffer to write one line of data (FIFO) - interlaced
 *   Even byte {D(200,y),D(198,y), D(196,y), D(194,y)}, ... ,{D(8,y),D(6,y),D(4,y), D(2,y)}
 *   Scan byte {S(1), S(2)...}, Odd{D(1,y),D(3,y)...}
 *   Odd byte  {D(1,y),D(3,y), D(5,y), D(7,y)}, ... ,{D(193,y),D(195,y),D(197,y), D(199,y)}
 * - One data bit can be
 * - For more details on the driving stages, please refer to the COG document Section 5.
 *
 * \param EPD_type_index The defined EPD size
 * \param image_ptr The pointer of memory that stores image that will send to COG
 * \param stage_no The assigned stage number that will proceed
 */
static void stage_handle_array(uint8_t EPD_type_index,uint8_t *image_prt,uint8_t stage_no) {
	/* x for horizontal_size loop, y for vertical_size loop, which are EPD pixel size */
	uint16_t x,y,k;
	static volatile uint8_t	temp_byte; // Temporary storage for image data check
	uint8_t *backup_image_prt; // Backup image address pointer
	backup_image_prt = image_prt;
	current_frame_time = COG_parameters[EPD_type_index].frame_time_offset;
	/* Start a system SysTick timer to ensure the same duration of each stage  */
	start_EPD_timer();

	/* Do while total time of frames exceed stage time
	* Per frame */
	do {
		image_prt=backup_image_prt;

		/* Per data line (vertical size) */
		for (y = 0; y < COG_parameters[EPD_type_index].vertical_size; y++) {
			/* Set charge pump voltage level reduce voltage shift */
			SPI_send_byte (0x04, COG_parameters[EPD_type_index].voltage_level);

			k = COG_parameters[EPD_type_index].horizontal_size-1;
			for (x = 0; x < COG_parameters[EPD_type_index].horizontal_size; x++) {
				temp_byte =*image_prt++;
				switch(stage_no) {
					case Stage1: // Compensate, Inverse previous image
					/* Example at stage 1 to get Even and Odd data
					* +---------+----+----+----+----+----+----+----+----+
					* |         |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
					* |temp_byte+----+----+----+----+----+----+----+----+
					* |         |  1 |  0 |  1 |  1 |  0 |  1 |  0 |  0 |
					* +---------+----+----+----+----+----+----+----+----+ */
					data_line_odd[x]     = ((temp_byte & 0x80) ? BLACK3  : WHITE3); // BLACK3 = 0xC0 = 1100 0000
					data_line_odd[x]    |= ((temp_byte & 0x20) ? BLACK2  : WHITE2); // BLACK2 = 0x30 = 0011 0000
					data_line_odd[x]    |= ((temp_byte & 0x08) ? BLACK1  : WHITE1); // WHITE1 = 0x08 = 0000 1000
					data_line_odd[x]    |= ((temp_byte & 0x02) ? BLACK0  : WHITE0); // WHITE0 = 0x02 = 0000 0010
					/* data_line_odd[x] = 1100 0000 | 0011 0000 | 0000 1000 | 0000 0010 = 1111 1010
					* See Even data row at the table below*/

					data_line_even[k]    = ((temp_byte & 0x01) ? BLACK3  : WHITE3); // WHITE3 = 0x80 = 1000 0000
					data_line_even[k]   |= ((temp_byte & 0x04) ? BLACK2  : WHITE2); // BLACK2 = 0x30 = 0011 0000
					data_line_even[k]   |= ((temp_byte & 0x10) ? BLACK1  : WHITE1); // BLACK1 = 0x0C = 0000 1100
					data_line_even[k--] |= ((temp_byte & 0x40) ? BLACK0  : WHITE0); // WHITE0 = 0x02 = 0000 0010
					/* data_line_even[k] = 1000 0000 | 0011 0000 | 0000 1100 | 0000 0010 = 1011 1110
					* See Odd data row at the table below
					* +---------+----+----+----+----+----+----+----+----+
					* |         |bit7|bit6|bit5|bit4|bit3|bit2|bit1|bit0|
					* |temp_byte+----+----+----+----+----+----+----+----+
					* |         |  1 |  0 |  1 |  1 |  0 |  1 |  0 |  0 |
					* +---------+----+----+----+----+----+----+----+----+
					* | Color   |  W |  B |  W |  W |  B |  W |  B |  B | W=White, B=Black, N=Nothing
					* +---------+----+----+----+----+----+----+----+----+
					* | Stage 1 |  B |  W |  B |  B |  W |  B |  W |  W | Inverse
					* +---------+----+----+----+----+----+----+----+----+
					* | Input   | 11 | 10 | 11 | 11 | 10 | 11 | 10 | 10 | W=10, B=11, N=01
					* +---------+----+----+----+----+----+----+----+----+
					* |Odd data | 11 |    | 11 |    | 10 |    | 10 |    | = 1111 1010
					* +---------+----+----+----+----+----+----+----+----+
					* |Even data|    | 10 |    | 11 |    | 11 |    | 10 | = 1011 1110
					* +---------+----+----+----+----+----+----+----+----+ */
						break;
					case Stage2: // White
					data_line_odd[x]     = ((temp_byte & 0x80) ?  WHITE3 : NOTHING3);
					data_line_odd[x]    |= ((temp_byte & 0x20) ?  WHITE2 : NOTHING2);
					data_line_odd[x]    |= ((temp_byte & 0x08) ?  WHITE1 : NOTHING1);
					data_line_odd[x]    |= ((temp_byte & 0x02) ?  WHITE0 : NOTHING0);

					data_line_even[k]    = ((temp_byte & 0x01) ?  WHITE3 : NOTHING3);
					data_line_even[k]   |= ((temp_byte & 0x04) ?  WHITE2 : NOTHING2);
					data_line_even[k]   |= ((temp_byte & 0x10) ?  WHITE1 : NOTHING1);
					data_line_even[k--] |= ((temp_byte & 0x40) ?  WHITE0 : NOTHING0);
						break;
					case Stage3: // Inverse new image
					data_line_odd[x]     = ((temp_byte & 0x80) ? BLACK3  : NOTHING3);
					data_line_odd[x]    |= ((temp_byte & 0x20) ? BLACK2  : NOTHING2);
					data_line_odd[x]    |= ((temp_byte & 0x08) ? BLACK1  : NOTHING1);
					data_line_odd[x]    |= ((temp_byte & 0x02) ? BLACK0  : NOTHING0);

					data_line_even[k]    = ((temp_byte & 0x01) ? BLACK3  : NOTHING3);
					data_line_even[k]   |= ((temp_byte & 0x04) ? BLACK2  : NOTHING2);
					data_line_even[k]   |= ((temp_byte & 0x10) ? BLACK1  : NOTHING1);
					data_line_even[k--] |= ((temp_byte & 0x40) ? BLACK0  : NOTHING0);
						break;
					case Stage4: // New image
					data_line_odd[x]     = ((temp_byte & 0x80) ? WHITE3  : BLACK3 );
					data_line_odd[x]    |= ((temp_byte & 0x20) ? WHITE2  : BLACK2 );
					data_line_odd[x]    |= ((temp_byte & 0x08) ? WHITE1  : BLACK1 );
					data_line_odd[x]    |= ((temp_byte & 0x02) ? WHITE0  : BLACK0 );

					data_line_even[k]    = ((temp_byte & 0x01) ? WHITE3  : BLACK3 );
					data_line_even[k]   |= ((temp_byte & 0x04) ? WHITE2  : BLACK2 );
					data_line_even[k]   |= ((temp_byte & 0x10) ? WHITE1  : BLACK1 );
					data_line_even[k--] |= ((temp_byte & 0x40) ? WHITE0  : BLACK0 );
						break;
				}
			}

			/* Scan byte shift per data line */
			data_line_scan[(y>>2)] = SCAN_TABLE[(y%4)];

			/* For 1.44 inch EPD, the border uses the internal signal control byte. */
			if(EPD_type_index == EPD_144)
				COG_Line.line_data_by_size.line_data_for_144.border_byte=0x00;

			/* Sending data */
			SPI_send (0x0A, (uint8_t *)&COG_Line.uint8,
				COG_parameters[EPD_type_index].data_line_size);

			/* Turn on Output Enable */
			SPI_send_byte (0x02, 0x2F);

			data_line_scan[(y>>2)]=0;
		}
		/* Count the frame time with offset */
		current_frame_time=(uint16_t)get_current_time_tick()+
			COG_parameters[EPD_type_index].frame_time_offset;
	} while (stage_time>current_frame_time);

	/* Do while the SysTick timer fulfills the stage time */
	while(stage_time>get_current_time_tick());

	/* Stop system timer */
	stop_EPD_timer();
}

/**
 * \brief The driving stages for getting Odd/Even data and writing the data
 * from Flash memory to COG
 *
 * \note
 * - Refer to stage_handle_array comment note.
 * - For more details on the driving stages, please refer to the COG document Section 5.
 *
 * \param EPD_type_index The defined EPD size
 * \param image_data_address The address of flash memory that stores image
 * \param stage_no The assigned stage number that will proceed
 */
static void stage_handle_flash(uint8_t EPD_type_index,long image_data_address,uint8_t stage_no) {
	/* x for horizontal_size loop, y for vertical_size loop, which are EPD pixel size */
	uint16_t x,y,k;
	static volatile uint8_t	temp_byte; // Temporary storage for image data check
	long original_image_address; // Backup original image address
	uint8_t byte_array[LINE_BUFFER_DATA_SIZE];
	original_image_address=image_data_address;
	current_frame_time=COG_parameters[EPD_type_index].frame_time_offset;

	/* Start a system SysTick timer to ensure the same duration of each stage  */
	start_EPD_timer();

	/* Do while total time of frames exceed stage time
	* Per frame */
	do {
		image_data_address=original_image_address;

		/* Per data line (vertical size) */
		for (y = 0; y < COG_parameters[EPD_type_index].vertical_size; y++){
			/* Set charge pump voltage level reduce voltage shift */
			SPI_send_byte (0x04, COG_parameters[EPD_type_index].voltage_level);

			k = COG_parameters[EPD_type_index].horizontal_size-1;
			if(_On_EPD_read_flash!=NULL) {
				_On_EPD_read_flash(image_data_address,(uint8_t *)&byte_array,
				COG_parameters[EPD_type_index].horizontal_size);
			}

			for (x = 0; x < COG_parameters[EPD_type_index].horizontal_size; x++){
				temp_byte =byte_array[x];
				switch(stage_no) {
					case Stage1: // Compensate, Inverse previous image
					/* See the stage #1 example in stage_handle_array */
					data_line_odd[x]	 =  ((temp_byte & 0x80) ? BLACK3  : WHITE3);
					data_line_odd[x]	|=  ((temp_byte & 0x20) ? BLACK2  : WHITE2);
					data_line_odd[x]	|=  ((temp_byte & 0x08) ? BLACK1  : WHITE1);
					data_line_odd[x]	|=  ((temp_byte & 0x02) ? BLACK0  : WHITE0);

					data_line_even[k]    = ((temp_byte & 0x01) ? BLACK3  : WHITE3);
					data_line_even[k]   |= ((temp_byte & 0x04) ? BLACK2  : WHITE2);
					data_line_even[k]   |= ((temp_byte & 0x10) ? BLACK1  : WHITE1);
					data_line_even[k--] |= ((temp_byte & 0x40) ? BLACK0  : WHITE0);
						break;

					case Stage2: // White
					data_line_odd[x] 	 =  ((temp_byte & 0x80) ?  WHITE3 : NOTHING3);
					data_line_odd[x]	|=  ((temp_byte & 0x20) ?  WHITE2 : NOTHING2);
					data_line_odd[x]	|=  ((temp_byte & 0x08) ?  WHITE1 : NOTHING1);
					data_line_odd[x]	|=  ((temp_byte & 0x02) ?  WHITE0 : NOTHING0);

					data_line_even[k]    =  ((temp_byte & 0x01) ?  WHITE3 : NOTHING3);
					data_line_even[k]   |=  ((temp_byte & 0x04) ?  WHITE2 : NOTHING2);
					data_line_even[k]   |=  ((temp_byte & 0x10) ?  WHITE1 : NOTHING1);
					data_line_even[k--] |=  ((temp_byte & 0x40) ?  WHITE0 : NOTHING0);
						break;

					case Stage3: // Inverse new image
					data_line_odd[x]     = ((temp_byte & 0x80) ? BLACK3  : NOTHING3);
					data_line_odd[x]    |= ((temp_byte & 0x20) ? BLACK2  : NOTHING2);
					data_line_odd[x]    |= ((temp_byte & 0x08) ? BLACK1  : NOTHING1);
					data_line_odd[x]    |= ((temp_byte & 0x02) ? BLACK0  : NOTHING0);

					data_line_even[k]    = ((temp_byte & 0x01) ? BLACK3  : NOTHING3);
					data_line_even[k]   |= ((temp_byte & 0x04) ? BLACK2  : NOTHING2);
					data_line_even[k]   |= ((temp_byte & 0x10) ? BLACK1  : NOTHING1);
					data_line_even[k--] |= ((temp_byte & 0x40) ? BLACK0  : NOTHING0);
						break;

					case Stage4: // New image
					data_line_odd[x]     = ((temp_byte & 0x80) ? WHITE3  : BLACK3 );
					data_line_odd[x]    |= ((temp_byte & 0x20) ? WHITE2  : BLACK2 );
					data_line_odd[x]    |= ((temp_byte & 0x08) ? WHITE1  : BLACK1 );
					data_line_odd[x]    |= ((temp_byte & 0x02) ? WHITE0  : BLACK0 );

					data_line_even[k]    = ((temp_byte & 0x01) ? WHITE3  : BLACK3 );
					data_line_even[k]   |= ((temp_byte & 0x04) ? WHITE2  : BLACK2 );
					data_line_even[k]   |= ((temp_byte & 0x10) ? WHITE1  : BLACK1 );
					data_line_even[k--] |= ((temp_byte & 0x40) ? WHITE0  : BLACK0 );
						break;

                }
            }
			image_data_address+=LINE_SIZE;
			/* Scan byte shift per data line */
			data_line_scan[(y>>2)]= SCAN_TABLE[(y%4)];

			/* For 1.44 inch EPD, the border uses the internal signal control byte. */
			if(EPD_type_index==EPD_144)
				COG_Line.line_data_by_size.line_data_for_144.border_byte=0x00;

			/* Sending data */
			SPI_send (0x0A, (uint8_t *)&COG_Line.uint8,
				COG_parameters[EPD_type_index].data_line_size);

			/* Turn on Output Enable */
			SPI_send_byte (0x02, 0x2F);

			data_line_scan[(y>>2)]=0;
		}
		/* Count the frame time with offset */
		current_frame_time=(uint16_t)get_current_time_tick()+
			COG_parameters[EPD_type_index].frame_time_offset ;
	} while (stage_time>current_frame_time);

	/* Do while the SysTick timer fulfills the stage time */
	while(stage_time>get_current_time_tick());

	/* Stop system timer */
	stop_EPD_timer();
}

/**
* \brief Write image data from memory array to the EPD
* \note For more detail on driving stages, please refer to COG document Section 5.
*
* \param EPD_type_index The defined EPD size
* \param previous_image_ptr The pointer of memory that stores previous image
* \param new_image_ptr The pointer of memory that stores new image
*/
void EPD_display_from_array_prt (uint8_t EPD_type_index, uint8_t *previous_image_ptr,
uint8_t *new_image_ptr) {
	stage_handle_array(EPD_type_index,previous_image_ptr,Stage1);
	stage_handle_array(EPD_type_index,previous_image_ptr,Stage2);
	stage_handle_array(EPD_type_index,new_image_ptr,Stage3);
	stage_handle_array(EPD_type_index,new_image_ptr,Stage4);
}

/**
* \brief Write image data from Flash memory to the EPD
*
* \param EPD_type_index The defined EPD size
* \param previous_image_flash_address The previous image address of flash memory
* \param new_image_flash_address The new image address of flash memory
* \param On_EPD_read_flash Developer needs to create an external function to read flash
*/
void EPD_display_from_flash_prt (uint8_t EPD_type_index, long previous_image_flash_address,
     long new_image_flash_address,EPD_read_flash_handler On_EPD_read_flash) {
	_On_EPD_read_flash=On_EPD_read_flash;
	stage_handle_flash(EPD_type_index,previous_image_flash_address,Stage1);
	stage_handle_flash(EPD_type_index,previous_image_flash_address,Stage2);
	stage_handle_flash(EPD_type_index,new_image_flash_address     ,Stage3);
	stage_handle_flash(EPD_type_index,new_image_flash_address     ,Stage4);
}

/**
* \brief Write Nothing Frame to COG
* \note A frame whose all D(x,y) are N(01). 0101=0x55=NOTHING
*
* \param EPD_type_index The defined EPD size
*/
static inline void nothing_frame (uint8_t EPD_type_index) {
	uint16_t i;
	for (i = 0; i <  COG_parameters[EPD_type_index].horizontal_size; i++) {
		data_line_even[i]=NOTHING;
		data_line_odd[i]=NOTHING;
	}
	for (i = 0; i < COG_parameters[EPD_type_index].vertical_size; i++) {
		/* Set charge pump voltage level reduce voltage shift */
		SPI_send_byte (0x04, COG_parameters[EPD_type_index].voltage_level);

		/* Scan byte shift per data line */
		data_line_scan[(i>>2)]=SCAN_TABLE[(i%4)];

		/* Sending data */
		SPI_send (0x0A, (uint8_t *)&COG_Line.uint8, COG_parameters[EPD_type_index].data_line_size);

		/* Turn on Output Enable */
		SPI_send_byte (0x02, 0x2F);

		data_line_scan[(i>>2)]=0;
	}
}

/**
* \brief Write Dummy Line to COG
* \note A line whose all Scan Bytes are 0x00
*
* \param EPD_type_index The defined EPD size
*/
static inline void dummy_line(uint8_t EPD_type_index) {
	uint8_t	i;
	for (i = 0; i < (COG_parameters[EPD_type_index].vertical_size/8); i++) {
		switch(EPD_type_index) {
			case EPD_144:
				COG_Line.line_data_by_size.line_data_for_144.scan[i]=0x00;
				break;
			case EPD_200:
				COG_Line.line_data_by_size.line_data_for_200.scan[i]=0x00;
				break;
			case EPD_270:
				COG_Line.line_data_by_size.line_data_for_270.scan[i]=0x00;
				break;
		}
	}
	/* Set charge pump voltage level reduce voltage shift */
	SPI_send_byte (0x04, COG_parameters[EPD_type_index].voltage_level);

	/* Sending data */
	SPI_send (0x0A, (uint8_t *)&COG_Line.uint8, COG_parameters[EPD_type_index].data_line_size);

	/* Turn on Output Enable */
	SPI_send_byte (0x02, 0x2F);
}

/**
* \brief Power Off COG Driver
* \note For detailed flow and description, please refer to the COG document Section 6.
*
* \param EPD_type_index The defined EPD size
*/
uint8_t EPD_power_off (uint8_t EPD_type_index) {

	nothing_frame (EPD_type_index);

	dummy_line(EPD_type_index);
	delay_ms (25);

	EPD_border_low();
	delay_ms (200);

	EPD_border_high();
	// Latch reset turn on
	SPI_send_byte (0x03, 0x01);

	// Output enable off
	SPI_send_byte (0x02, 0x05);

	// Power off charge pump Vcom
	SPI_send_byte (0x05, 0x0E);

	// Power off charge negative voltage
	SPI_send_byte (0x05, 0x02);

	// Discharge
	SPI_send_byte (0x04, 0x0C);
	delay_ms (120);

	// Turn off all charge pumps
	SPI_send_byte (0x05, 0x00);

	// Turn off osc
	SPI_send_byte (0x07, 0x0D);

	// Discharge internal
	SPI_send_byte (0x04, 0x50);
	delay_ms (40);

	// Discharge internal
	SPI_send_byte (0x04, 0xA0);
	delay_ms (40);

	// Discharge internal
	SPI_send_byte (0x04, 0x00);

	// Set power and signals = 0
	EPD_rst_low ();
	spi_detach ();
	EPD_cs_low ();
	EPD_Vcc_turn_off ();
	EPD_border_low();

	// External discharge = 1
	EPD_discharge_high ();
	delay_ms (150);

	// External discharge = 0
	EPD_discharge_low ();

	return RES_OK;
}
