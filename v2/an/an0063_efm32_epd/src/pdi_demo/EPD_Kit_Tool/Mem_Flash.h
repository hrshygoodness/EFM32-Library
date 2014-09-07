/**
* \file
*
* \brief The definition of flash memory
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

#ifndef MEM_FLASH_H_
#define MEM_FLASH_H_

#include "EPD_Kit_Tool_Process.h"

/******************************************************************************
 * \brief Each EPD size has 32 pages of image
 *
 * \note
 * - Divided 32 pages into 2 segments A and B. Each segment has 16 pages.
 * - Segment A: for sequence image buffer at "Drawing" tab of EPD Kit Tool
 * - Segment B: for Slideshow, ASCII and custom assigned images of EPD Kit Tool
 */

/** Flash map *****************************************************************/
#define _flash_sector_size          (long)4*1024                //4K
#define _page_size_144_200          (long)_flash_sector_size*2  //8k
#define _page_size_270              (long)_flash_sector_size*3  //12k
#define _image_page_max	            16 //16 pages
#define _flash_line_size            64
//must be 16, 32 or 64. Due to 2.7"=264x176,the 264=33bytes

/** the last page if using mark image for partial update */
#define _image_page_mark            ((uint8_t)(_image_page_max-1))

/** the last two bytes of first flash line to flag the image state */
#define _image_header_mark_offset   _flash_line_size-2


/** 1.44" Flash Map ***********************************************************
 * Segment A: 0x00000~0x1FFFF, 16 pages of 1.44" image
 * 1.44" resolution=128*96, 128=16bytes. 96*64=6K, allocate 8K (64=_flash_line_size) */
#define _image144_size                      13 //8K=2^13, for bit shift
#define _image144_SOF                       (long)0x0000 //start address of flash of 1.44"
#define _image144_address(x)                _image144_SOF+((long)x<<_image144_size)

/** Segment B: 0x20000~0x27FFF, 4 pages of marked image */
#define _image144_mark_image_page_max       4
#define _image144_mark_image_SOF            (long)(_image144_SOF+(_image_page_max*_page_size_144_200))
#define _image144_mark_image_address(x)     _image144_mark_image_SOF+((long)x<<_image144_size)

/** Segment B: 0x28000~0x2FFFF, 4 pages of slideshow image */
#define _image144_slideshow_page_max        4
#define _image144_slideshow_SOF             (long)_image144_mark_image_SOF+ \
                                            (_image144_mark_image_page_max*_page_size_144_200)
#define _image144_slideshow_address(x)      _image144_slideshow_SOF+((long)x<<_image144_size)

/** Segment B: 0x30000~0x3FFFF, 8 pages of custom image */
#define _image144_custom_page_max           8
#define _image144_custom_SOF                (long)_image144_slideshow_SOF+ \
                                            (_image144_slideshow_page_max*_page_size_144_200)
#define _image144_custom_address(x)         _image144_custom_SOF+((long)x<<_image144_size)


/** 2" Flash Map **************************************************************
 * Segment A: 0x40000~0x5FFFF, 16 pages of 2" image
 * 2" resolution=200*96, 200=25bytes. 96*64=6K, allocate 8K (64=_flash_line_size) */
#define _image200_size                      13 //8K=2^13, for bit shift
#define _image200_SOF                       (long)_image144_custom_SOF+ \
                                            (_image144_custom_page_max*_page_size_144_200)
#define _image200_address(x)                _image200_SOF+((long)x<<_image200_size)

/** Segment B: 0x60000~0x67FFF, 4 pages of marked image */
#define _image200_mark_image_page_max		4
#define _image200_mark_image_SOF            (long)_image200_SOF+(_image_page_max*_page_size_144_200)
#define _image200_mark_image_address(x)     _image200_mark_image_SOF+((long)x<<_image200_size)

/** Segment B: 0x68000~0x6FFFF, 4 pages of slideshow image */
#define _image200_slideshow_page_max        4
#define _image200_slideshow_SOF				(long)_image200_mark_image_SOF+ \
                                            (_image200_mark_image_page_max*_page_size_144_200)
#define _image200_slideshow_address(x)      _image200_slideshow_SOF+((long)x<<_image200_size)

/** Segment B: 0x70000~0x7FFFF, 8 pages of custom image */
#define _image200_custom_page_max           8
#define _image200_custom_SOF                (long)_image200_slideshow_SOF+ \
                                            (_image200_slideshow_page_max*_page_size_144_200)
#define _image200_custom_address(x)         _image200_custom_SOF+((long)x<<_image200_size)


/** 2.7" Flash Map **************************************************************
 * Segment A: 0x80000~0xAFFFF, 16 pages of 2.7" image
 * 2.7" resolution=264*176, 264=33bytes. 176*64=11K, allocate 12K (64=_flash_line_size) */
#define _image270_SOF                       (long)_image200_custom_SOF+ \
                                            (_image200_custom_page_max*_page_size_144_200)
#define _image270_address(x)                _image270_SOF + (((long)x<<13)+((long)x<<12)) //(8*1024) + (4*1024) = 2^13 + 2^12

/** Segment B: 0xB0000~0xBBFFF, 4 pages of marked image */
#define _image270_mark_image_page_max       4
#define _image270_mark_image_SOF            (long)_image270_SOF+(_image_page_max*_page_size_270)
#define _image270_mark_image_address(x)     _image270_mark_image_SOF +(((long)x<<13)+((long)x<<12))

/** Segment B: 0xBC000~0xC7FFF, 4 pages of slideshow image */
#define _image270_slideshow_page_max        4
#define _image270_slideshow_SOF             (long)_image270_mark_image_SOF+ \
                                            (_image270_mark_image_page_max*_page_size_270)
#define _image270_slideshow_address(x)      _image270_slideshow_SOF +(((long)x<<13)+((long)x<<12))

/** Segment B: 0xC8000~0xCFFFF, 8 pages of custom image */
#define _image270_custom_page_max           8
#define _image270_custom_SOF                (long)_image270_slideshow_SOF+ \
                                            (_image270_slideshow_page_max*_page_size_270)
#define _image270_custom_address(x)         _image270_custom_SOF +(((long)x<<13)+((long)x<<12))

/** The slideshow parameters are stored at flash segment starts from 0xFF000 to 0xFFFF0  */
#define _parameters_address					0xFF000
#define _parameters_address_max				0xFFFF0

/******************************************************************************/
#define _image_state_in_use    0xAF
#define _image_state_is_empty  0xFF
#define _NULL_address          -1

#define __ASCII_OFFSET		0x20
#define __TEXT_Width		8 /*!< The predefined ASCII character is 8*8 */
#define __TEXT_High		    8

/**
 * \brief The structure of ASCII input information
 */
typedef struct {
	uint16_t x;        /*!< x coordinate */
	uint16_t y;        /*!< y coordinate */
	uint8_t  str[16];  /*!< Input string */
} ASCII_info_t;

/******************************************************************************/
/** The Flash MX25 series command hex code definition */
#define ElectronicID   0x13

/** ID commands */
#define FLASH_CMD_RDID 0x9F      //RDID (Read Identification)
#define FLASH_CMD_RES  0xAB      //RES (Read Electronic ID)
#define FLASH_CMD_REMS 0x90      //REMS (Read Electronic & Device ID)

/** Register commands */
#define FLASH_CMD_WRSR 0x01      //WRSR (Write Status Register)
#define FLASH_CMD_RDSR 0x05      //RDSR (Read Status Register)

/** READ commands */
#define FLASH_CMD_READ     0x03  //READ (1 x I/O)
#define FLASH_CMD_FASTREAD 0x0B  //FAST READ (Fast read data)

/** Program commands */
#define FLASH_CMD_WREN 0x06      //WREN (Write Enable)
#define FLASH_CMD_WRDI 0x04      //WRDI (Write Disable)
#define FLASH_CMD_PP   0x02      //PP (page program)

/** Erase commands */
#define FLASH_CMD_SE 0x20        //SE (Sector Erase)
#define FLASH_CMD_BE 0xD8        //BE (Block Erase)
#define FLASH_CMD_CE 0x60        //CE (Chip Erase) hex code: 60 or C7

/** Mode setting commands */
#define FLASH_CMD_DP  0xB9       //DP (Deep Power Down)
#define FLASH_CMD_RDP 0xAB       //RDP (Release form Deep Power Down)

/** status register */
#define FLASH_WIP_MASK  0x01
#define FLASH_LDSO_MASK 0x02
#define FLASH_QE_MASK   0x40

uint8_t is_flash_existed(void);

void Flash_cs_high(void);
void Flash_cs_low(void);
void CMD_SE( long flash_address );

void CMD_CE(void);
//void EraseImage_Ex(long address,uint8_t ptype);

void Flash_init(void);
void read_flash(long Address,uint8_t *target_address, uint8_t byte_length);
void write_flash(long Address,uint8_t *source_address, uint8_t byte_length);

void erase_image(long address,uint8_t ptype);
void get_flash_image_info(image_information_t * ImageInfo);
long get_flash_mark_image_info(uint8_t PlaneType);
long get_custom_image_address(uint8_t  PlaneType,uint8_t ImageIdx,uint8_t IsClear);
long get_slideshow_image_address(uint8_t  PlaneType,uint8_t ImageIdx,uint8_t IsClear);
void write_mark(long address);
void Readtest(void);
void write_ascii(long CanvasAddress,long MarkAddress,uint16_t LocationX,uint16_t LocationY,char *Text);
void read_slideshow_parameters(slideshow_information_t * SlideshowInfo);
void write_slideshow_parameters(slideshow_information_t * SlideshowInfo);

#endif /* MEM_FLASH_H_ */
