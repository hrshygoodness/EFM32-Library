/**
* \file
*
* \brief The functions of EPD GPIO
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


#include "EPD_hardware_gpio.h"

/**
* \brief Set EPD_CS pin to high
*/
void EPD_cs_high (void) {
	set_gpio_high(EPD_CS_PORT,EPD_CS_PIN);
}

/**
* \brief Set EPD_CS pin to low
*/
void EPD_cs_low (void) {
	set_gpio_low(EPD_CS_PORT,EPD_CS_PIN);
}

/**
* \brief Set Flash_CS pin to high
*/
void EPD_flash_cs_high(void) {
	set_gpio_high(Flash_CS_PORT,Flash_CS_PIN);
}

/**
* \brief Set Flash_CS pin to low
*/
void EPD_flash_cs_low (void) {
	set_gpio_low(Flash_CS_PORT,Flash_CS_PIN);
}

/**
* \brief Set /RESET pin to high
*/
void EPD_rst_high (void) {
	set_gpio_high(EPD_RST_PORT,EPD_RST_PIN);
}

/**
* \brief Set /RESET pin to low
*/
void EPD_rst_low (void) {
	set_gpio_low(EPD_RST_PORT,EPD_RST_PIN);
}

/**
* \brief Set DISCHARGE pin to high
*/
void EPD_discharge_high (void) {
	set_gpio_high(EPD_DISCHARGE_PORT,EPD_DISCHARGE_PIN);
}

/**
* \brief Set DISCHARGE pin to low
*/
void EPD_discharge_low (void) {
	set_gpio_low(EPD_DISCHARGE_PORT,EPD_DISCHARGE_PIN);
}

/**
* \brief Set Vcc (PANEL_ON) to high
*/
void EPD_Vcc_turn_on (void) {
	set_gpio_high(EPD_PANELON_PORT,EPD_PANELON_PIN);
}

/**
* \brief Set Vcc (PANEL_ON) to low
*/
void EPD_Vcc_turn_off (void) {
	set_gpio_low(EPD_PANELON_PORT,EPD_PANELON_PIN);
}

/**
* \brief Set BORDER_CONTROL pin to high
*/
void EPD_border_high(void) {
	set_gpio_high(EPD_PANELON_PORT,EPD_BORDER_PIN);
}

/**
* \brief Set BORDER_CONTROL pin to low
*/
void EPD_border_low (void) {
	set_gpio_low(EPD_PANELON_PORT,EPD_BORDER_PIN);
}

/**
* \brief Set PWM pin to high
*/
void EPD_pwm_high(void) {
	set_gpio_high(PWM_PORT,PWM_PIN);
}

/**
* \brief Set PWM pin to low
*/
void EPD_pwm_low (void) {
    config_gpio_dir_o(SPIMISO_PORT,SPIMISO_PIN);
	set_gpio_low(PWM_PORT,PWM_PIN);
}

/**
* \brief Set MISO pin of SPI to low
*/
void SPIMISO_low(void) {
	config_gpio_dir_o(SPIMISO_PORT,SPIMISO_PIN);
	set_gpio_low(SPIMISO_PORT,SPIMISO_PIN);
}

/**
* \brief Set MOSI pin of SPI to low
*/
void SPIMOSI_low(void) {
	set_gpio_low(SPIMOSI_PORT,SPIMOSI_PIN);
}

/**
* \brief Set Clock of SPI to low
*/
void SPICLK_low(void) {
	set_gpio_low(SPICLK_PORT,SPICLK_PIN);
}

/**
* \brief Get BUSY pin status
*/
bool EPD_IsBusy(void) {
	return (bool)input_get(EPD_BUSY_PORT,EPD_BUSY_PIN);
}

/**
* \brief Configure GPIO
*/
void EPD_initialize_gpio(void) {
	config_gpio_dir_i( EPD_BUSY_PORT,EPD_BUSY_PIN);      
	config_gpio_dir_o( EPD_CS_PORT,EPD_CS_PIN);
	config_gpio_dir_o( EPD_RST_PORT,EPD_RST_PIN);
	config_gpio_dir_o( EPD_PANELON_PORT,EPD_PANELON_PIN);
	config_gpio_dir_o( EPD_DISCHARGE_PORT,EPD_DISCHARGE_PIN);
	config_gpio_dir_o( EPD_BORDER_PORT,EPD_BORDER_PIN);
	config_gpio_dir_o( Flash_CS_PORT,Flash_CS_PIN);
        config_gpio_dir_o( PWM_PORT,PWM_PIN);
	config_gpio_dir_i( Temper_PORT,Temper_PIN);
	EPD_flash_cs_high();
	EPD_border_low();
}


