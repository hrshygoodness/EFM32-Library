/**
* \file
*
* \brief The EPD configurations
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

#ifndef CONF_EPD_H_INCLUDED
#define CONF_EPD_H_INCLUDED

/** Define EPD_KIT_TOOL_FUNCTIONS if the code will work with EPD Kit Tool.
 * \note Comment out the #define below if you won't work with EPD Kit tool and
 *       just want to show two images change alternately by predefined image array.
 */
#define EPD_KIT_TOOL_FUNCTIONS

/** \brief Supported COG version
 *
 * \note
 * - place the value here to assign which COG driving waveform will be used.
 * - now=COG_V110, future=COG_V110_G2, COG_V230, etc. */
#define COG_V110

/** The SPI frequency of this kit (12MHz) */
#define COG_SPI_baudrate 12000000

/** Define the number of ram buffer for system packet used interchangeably*/
#define BUFFER_SIZE	2

/** System Packet length=6~64, maximum=64.
* ========================
* | Byte | Name          |
* |------|---------------|
* | 0    | Packet Header |
* | 1    | Packet Length |
* | 2-3  | Kit ID        |
* | 4    | Command Type  |
* | 5-62 | Data          |
* | 63   | CRC           |
*/
#define PAYLOAD_SIZE 64

/** Firmware version to display on EPD Kit Tool  */
#define EPD_KIT_TOOL_VERSION	"1.00 "

/** \brief Kit ID for recognizing by EPD Kit Tool
 *
 * \note
 * - High byte is IC vendor, low byte is the name of supported development kit
 * - Silocon Labs(EnergyMicro)=04, EFM32_Giant_Gecko_Starter_Kit(EFM32GG-STK3700)=01 */
#define EPD_KIT_TOOL_ID			(uint16_t)0x0401

#endif /* CONF_EPD_H_INCLUDED */

