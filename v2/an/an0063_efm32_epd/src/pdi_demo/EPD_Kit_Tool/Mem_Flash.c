/**
* \file
*
* \brief The functions of working with flash memory
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

#include "Mem_Flash.h"


static uint8_t Mark_Random_Index=0;

/**
 * \brief Set Flash_CS pin to high and EPD_CS to low
 */
void Flash_cs_high(void) {
	EPD_flash_cs_high();
	EPD_cs_low ();
}

/**
 * \brief Set EPD_CS to high and Flash_CS to low
 */
void Flash_cs_low(void) {
	EPD_cs_high();
	EPD_flash_cs_low();
}

void Flash_init(void) {
	//Flash_CS_High();
}
 

/**
 * \brief Send one data byte to SPI
 *
 * \param data The data to be sent out
 */
static void send_byte(uint8_t byte_value) {
	SPI_write(byte_value);
}

/**
 * \brief Get one data byte from SPI
 *
 * \return Get one data byte
 */
static uint8_t get_byte(void) {
	return SPI_read(0);
}

/**
 * \brief Send address to Flash
 *
 * \param flash_address The address
 */
static void send_flash_address(long flash_address) {
	/** A23-A0 */
	send_byte( (flash_address >> 16) );
	send_byte( (flash_address >> 8) );
	send_byte( (flash_address));
}


/**
 * \brief Read Flash data into buffer
 *
 * \param flash_address The start address of Flash
 * \param target_buffer The target address of buffer will be read
 * \param byte_length The data length will be read
 */
static void flash_cmd_read( long flash_address, uint8_t *target_buffer, long byte_length ) {
	long index;

	/** Chip select go low to start a flash command */
	Flash_cs_low();

	/** Write READ command and address */
	send_byte( FLASH_CMD_FASTREAD );
	send_flash_address( flash_address );
	send_byte(0);

	/** Set a loop to read data into buffer */
	for( index=0; index < byte_length; index++ ) {
		// Read one data byte at a time
		*(target_buffer + index) = get_byte();
	}

	/** Chip select go high to end a flash command */
	Flash_cs_high();
}

/**
 * \brief Check the flash memory of EPD extension board is existed or not in order
 *         to determine the board is connected
 * \return The Flash is existed
 */
uint8_t is_flash_existed(void) {
	uint8_t	get_data_buffer=0;
	uint8_t check_times=3, flash_existed=0;
	spi_attach();
	while(check_times--) {
		Flash_cs_high();
		delay_ms(1);
		/** Chip select go low to start a flash command */
		Flash_cs_low();
		delay_ms(1);
		/** Send command to check the Electronic ID of Flash */
		if(SPI_write_ex( FLASH_CMD_RES )==1) {
			SPI_write_ex(0);
			SPI_write_ex(0);
			SPI_write_ex(0);
			get_data_buffer = get_byte();
		}
		/** Chip select go high to end a flash command */
		Flash_cs_high();
		if(get_data_buffer==ElectronicID) {
			flash_existed=1;
			break;
		}
	};
	return flash_existed;
}

/**
 * \brief Read status register
 *
 * \return Register status
 */
static uint8_t CMD_RDSR(void) {
	uint8_t	gDataBuffer;

	// Chip select go low to start a flash command
	Flash_cs_low();

	// Send command
	send_byte( FLASH_CMD_RDSR );
	gDataBuffer = get_byte();

	// Chip select go high to end a flash command
	Flash_cs_high();

	return gDataBuffer;
}

/**
 * \brief Check if the flash is busy
 *
 * \return 1=busy, 0=not busy
 */
static uint8_t IsFlashBusy( void ) {
	uint8_t	 gDataBuffer;

	gDataBuffer= CMD_RDSR();
	if( (gDataBuffer & FLASH_WIP_MASK)  == FLASH_WIP_MASK )
		return 1;
	else
		return 0;
}

/**
 * \brief Set flash write enable
 */
static void CMD_WREN( void ) {
	// Chip select go low to start a flash command
	Flash_cs_low();

	// Write Enable command = 0x06, Setting Write Enable Latch Bit
	send_byte( FLASH_CMD_WREN );

	// Chip select go high to end a flash command
	Flash_cs_high();
}

/**
 * \brief Write the data to flash
 *
 * \param flash_address 32 bit flash memory address
 * \param source_address The source address of buffer will be written
 * \param byte_length The data length will be read
 */
static void CMD_PP( long flash_address, uint8_t *source_address, uint8_t byte_length ) {
	long index;
	while( IsFlashBusy()) _NOP();
	// Setting Write Enable Latch bit
	CMD_WREN();

	// Chip select go low to start a flash command
	Flash_cs_low();

	// Write Page Program command
	send_byte( FLASH_CMD_PP );
	send_flash_address( flash_address);

	// Set a loop to download whole page data into flash's buffer
	// Note: only last 256 byte will be programmed
	for( index=0; index < byte_length; index++ ) {
		send_byte( *(source_address + index) );
	}

	// Chip select go high to end a flash command
	Flash_cs_high();
	while( IsFlashBusy()) _NOP();
}

/**
 * \brief Erase the data of the chosen sector (4KB) to be "1"
 *
 * \param flash_address 32 bit flash memory address
 */
void CMD_SE( long flash_address ) {
	while( IsFlashBusy()) _NOP();
	// Setting Write Enable Latch bit
	CMD_WREN();

	// Chip select go low to start a flash command
	Flash_cs_low();

	//Write Sector Erase command = 0x20;
	send_byte( FLASH_CMD_SE );

	send_flash_address( flash_address );

	// Chip select go high to end a flash command
	Flash_cs_high();
	while(IsFlashBusy());
}

/**
 * \brief Erase all of the flash memory
 */
void CMD_CE(void) {
	while( IsFlashBusy()) _NOP();
	// Setting Write Enable Latch bit
	CMD_WREN();
	// Chip select go low to start a flash command
	Flash_cs_low();
	//Write Chip Erase command = 0x60;
	send_byte( FLASH_CMD_CE);
	// Chip select go high to end a flash command
	Flash_cs_high();

}


/**
 * \brief Read Flash data into buffer
 *
 * \param flash_address The start address of Flash
 * \param target_buffer The target address of buffer will be read
 * \param byte_length The data length will be read
 */
void read_flash(long flash_address,uint8_t *target_buffer, uint8_t byte_length) {

	flash_cmd_read(flash_address,target_buffer,byte_length);
}

/**
 * \brief Write Flash data from buffer
 *
 * \param flash_address 32 bit flash memory address
 * \param source_address The source address of buffer will be written
 * \param byte_length The data length will be read
 */
void write_flash(long flash_address,uint8_t *source_address, uint8_t byte_length) {

	CMD_PP(flash_address,source_address,byte_length);

}


/**
 * \brief To erase the image data
 *
 * \param address The start address to be erased
 * \param EPD_size The EPD size
 */
void erase_image(long address,uint8_t EPD_size) {
	uint8_t i,multiple_of_image_size=2;

	switch(EPD_size) {
	case EPD_144:
		multiple_of_image_size=2; // 8 kbytes
		break;
	case EPD_200:
		multiple_of_image_size=2; // 8 kbytes
		break;
	case EPD_270:
		multiple_of_image_size=3; // 12kbytes
		break;
	}
	for(i=0; i<multiple_of_image_size; i++) {
		CMD_SE(address); //Erase data of the chosen sector
		address+=_flash_sector_size; //4K
		delay_ms(300);
	}
}

/**
 * \brief Get the slideshow image address and clear the image or not
 *
 * \param EPD_size The EPD size
 * \param image_index The current image index to show
 * \param is_clear Whether to clear slideshow images
 */
long get_slideshow_image_address(uint8_t EPD_size,uint8_t image_index,uint8_t is_clear) {
	long addr=0;
	uint8_t IsExit=0xFF;

	/** number of maximum slideshow images = 4 */
	if(image_index>_image200_slideshow_page_max) image_index=0;

	switch( EPD_size) {
	case EPD_144:
		addr=_image144_slideshow_address(image_index);
		break;
	case EPD_200:
		addr=_image200_slideshow_address(image_index);
		break;
	case EPD_270:
		addr=_image270_slideshow_address(image_index);
		break;
	}
	/** To erase slideshow image */
	if(is_clear) {
		/** Read image mark offset byte to IsExit */
		flash_cmd_read(addr+_image_header_mark_offset,(uint8_t *)&IsExit,1);
		if(IsExit!=_image_state_is_empty) erase_image(addr,EPD_size);
	}

	return addr;
}

/**
 * \brief Get the custom image address and clear the image or not
 *
 * \param EPD_size The EPD size
 * \param image_index The current image index to show
 * \param is_clear Whether to clear custom images
 */
long get_custom_image_address(uint8_t  EPD_size,uint8_t image_index,uint8_t is_clear) {
	long addr=0;
	uint8_t IsExit=0xFF;

	/** number of maximum custom images = 8 */
	if(image_index>_image200_custom_page_max) image_index=0;

	switch( EPD_size) {
	case EPD_144:
		addr=_image144_custom_address(image_index);
		break;
	case EPD_200:
		addr=_image200_custom_address(image_index);
		break;
	case EPD_270:
		addr=_image270_custom_address(image_index);
		break;
	}
	/** To erase custom image */
	if(is_clear) {
		flash_cmd_read(addr+_image_header_mark_offset,(uint8_t *)&IsExit,1);
		if(IsExit!=_image_state_is_empty) erase_image(addr,EPD_size);
	}

	return addr;
}

/**
 * \brief Get mark image information from flash
 *
 * \param EPD_size The EPD size
 */
long get_flash_mark_image_info(uint8_t EPD_size) {
	long mark_address=0;
	uint8_t IsExit=0xFF;

	switch(EPD_size) {
	case EPD_144:
		mark_address=_image144_mark_image_address(Mark_Random_Index);
		break;
	case EPD_200:
		mark_address=_image200_mark_image_address(Mark_Random_Index);
		break;
	case EPD_270:
		mark_address=_image270_mark_image_address(Mark_Random_Index);
		break;
	}
	Mark_Random_Index=(Mark_Random_Index+1) % (_image144_mark_image_page_max-1);
	flash_cmd_read(mark_address+_image_header_mark_offset,(uint8_t *)&IsExit,1);
	if(IsExit!=_image_state_is_empty) erase_image(mark_address,EPD_size);

	return mark_address;
}

/**
 * \brief Get image information from flash
 *
 * \param image_info The data structure of image information
 */
void get_flash_image_info(image_information_t * image_info) {
	uint8_t i;
	uint8_t previous_address_offset,new_address_offset,empty_address_offset;
	long next_new_empty_address=0;
	long previous_address=0;
	spi_attach();
	for(i=0; i<_image_page_max; i++) {
		switch(image_info->EPD_size) {
		case EPD_144:
			previous_address=_image144_address(i);
			image_info->new_image_address=_image144_address(((i+1) & _image_page_mark));
			next_new_empty_address=_image144_address(((i+2) & _image_page_mark));
			break;
		case EPD_200:
			previous_address=_image200_address(i);
			image_info->new_image_address=_image200_address(((i+1) & _image_page_mark));
			next_new_empty_address=_image200_address(((i+2) & _image_page_mark));
			break;
		case EPD_270:
			previous_address=_image270_address(i);
			image_info->new_image_address=_image270_address(((i+1) & _image_page_mark));
			next_new_empty_address=_image200_address(((i+2) & _image_page_mark));
			break;
		}
		flash_cmd_read(previous_address+_image_header_mark_offset,(uint8_t *)&previous_address_offset,1);
		flash_cmd_read(image_info->new_image_address+_image_header_mark_offset,(uint8_t *)&new_address_offset,1);
		flash_cmd_read(next_new_empty_address+_image_header_mark_offset,(uint8_t *)&empty_address_offset,1);
		if(previous_address_offset!=_image_state_is_empty && new_address_offset==_image_state_is_empty)break;
	}

	previous_address_offset=(i & _image_page_mark);
	new_address_offset=((i+1) & _image_page_mark);
	empty_address_offset=((i+2) & _image_page_mark);

	switch(image_info->EPD_size) {
	case EPD_144:
		image_info->previous_image_address=_image144_address(previous_address_offset);
		image_info->new_image_address=_image144_address(new_address_offset);
		//erase next space
		if(empty_address_offset!=_image_state_is_empty)
			erase_image(_image144_address(empty_address_offset),image_info->EPD_size);
		break;
	case EPD_200:
		image_info->previous_image_address=_image200_address(previous_address_offset);
		image_info->new_image_address=_image200_address(new_address_offset);
		//erase next space
		if(empty_address_offset!=_image_state_is_empty)
			erase_image(_image200_address(empty_address_offset),image_info->EPD_size);
		break;
	case EPD_270:
		image_info->previous_image_address=_image270_address(previous_address_offset);
		image_info->new_image_address=_image270_address(new_address_offset);
		//erase next space
		if(empty_address_offset!=_image_state_is_empty)
			erase_image(_image270_address(empty_address_offset),image_info->EPD_size);
		break;
	}
	if(image_info->extend_address.last_address!=_NULL_address)
		image_info->previous_image_address=image_info->extend_address.last_address;
}

/**
 * \brief Write image header to flash
 *
 * \param address The image address
 */
void write_mark(long address) {
	uint8_t mark_byte=_image_state_in_use;
	spi_attach();
	CMD_PP(address+_image_header_mark_offset,&mark_byte,1);
}

/**
 * \brief Write ASCII data to canvas image of flash
 *
 * \note
 * - Introduce Mark Image (partial update function)
 *   -# Mark image is now used for ASCII and partial update function.
 *   -# [Previous Image] - [New Image/Canvas Image] - [Mark Image]
 *   -# Previous Image will save the image data that user downloads on EPD first.
 *   -# When user types ASCII string by coordinate on EPD Kit Tool, the Mark Image
 *      will mark the area that ASCII data will show on EPD.
 *   -# New Image likes a canvas. System will compare the mark area with Previous
 *      Image, only the mark area needs to be scanned and updated, the other area
 *      will send Nothing byte.
 *   -# The final output image is stored in New Image combines with Previous Image
 *      and ASCII data.
 *
 * \param canvas_address The canvas image address
 * \param mark_address The mark image address
 * \param coordinate_X The location of horizontal of inputted string
 * \param coordinate_Y The location of vertical of inputted string
 * \param Text The pointer of inputted string
 */
void write_ascii(long canvas_address,long mark_address,uint16_t coordinate_X,
                 uint16_t coordinate_Y,char *Text) {
	uint8_t tmp2,y,x_offset;
	uint8_t cnt=0,len=0;
	char *tmp;
	uint8_t *Text_Array;
	uint8_t r0,r2;
	spi_attach();
	/** Get mark and canvas image address */
	if(mark_address!=_NULL_address)	mark_address=mark_address+
		        (_flash_line_size*coordinate_Y+(coordinate_X/__TEXT_Width));
	canvas_address=canvas_address+(_flash_line_size*coordinate_Y+(coordinate_X/__TEXT_Width));

	x_offset=coordinate_X%__TEXT_Width;

	tmp=Text;
	while( (*tmp++) >= __ASCII_OFFSET) len++;
	if(len==0)return;
	if(x_offset>0) len++;
	Text_Array = (uint8_t*) malloc(len);
	for(y=0; y<__TEXT_High; y++) {
		tmp=Text;
		cnt=0;
		if(x_offset!=0) r0=~((1<<(__TEXT_Width-x_offset))-1);
		else r0=0xff;
		/** Get ASCII character data and save to Text_array */
		while(*tmp>=__ASCII_OFFSET) {
			tmp2=ASCII_character_bitmaps[*tmp-__ASCII_OFFSET][y];
			if(x_offset!=0) {
				r2=((tmp2 & ((1<<x_offset)-1)))<<(__TEXT_Width-x_offset);
				tmp2>>=x_offset;
				tmp2|=r0;
				r0=r2;
			}
			Text_Array[cnt]=tmp2;
			tmp++;
			cnt++;
		}
		if(x_offset>0)  Text_Array[cnt]=r0 | ((1<<(__TEXT_Width-x_offset))-1);

		/** write Text_array to Canvas memory */
		write_flash(canvas_address,Text_Array,len);
		canvas_address+=_flash_line_size;

		/** write mark memory */
		if(mark_address!=_NULL_address) {
			for(cnt=0; cnt<len; cnt++)Text_Array[cnt]=0;
			write_flash(mark_address,Text_Array,len);
			mark_address+=_flash_line_size;
		}
	}
	free(Text_Array);
}

/**
 * \brief Update slideshow parameters
 *
 * \param slideshow_info The pointer address of slideshow information
 */
void write_slideshow_parameters(slideshow_information_t * slideshow_info) {
	uint8_t Result;
	spi_attach();
	long addr=_parameters_address;
	do {
		flash_cmd_read(addr,(uint8_t *)&Result,1);
		if(addr>=_parameters_address_max) {
			addr=_parameters_address;
			CMD_SE(addr);
		}
		addr+=sizeof(slideshow_information_t);
	} while(Result!=0xFF);
	addr-=sizeof(slideshow_information_t);
	write_flash(addr,(uint8_t *)slideshow_info,sizeof(slideshow_information_t));
}

/**
 * \brief Read slideshow parameters from defined Flash segment
 *
 * \param slideshow_info The structure of slideshow information
 */
void read_slideshow_parameters(slideshow_information_t * slideshow_info) {
	uint8_t Result;
	long addr=_parameters_address;
	spi_attach();
	do {
		/** Read parameters from address */
		flash_cmd_read(addr,(uint8_t *)&Result,1);
		addr+=sizeof(slideshow_information_t);
		if(Result==0xFF && addr==_parameters_address+sizeof(slideshow_information_t)) break;
	} while(Result!=0xFF); /*!< Do while the data byte is empty */
	/*!< Forward two data length is the last saved position of parameter address */
	addr-=(sizeof(slideshow_information_t)*2);
	flash_cmd_read(addr,(uint8_t *)slideshow_info,sizeof(slideshow_information_t));
}
