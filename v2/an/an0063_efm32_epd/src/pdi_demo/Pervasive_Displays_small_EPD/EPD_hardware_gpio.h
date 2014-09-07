/**
* \file
*
* \brief The definition of EPD GPIO pins
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

#include "Pervasive_Displays_small_EPD.h"

#ifndef DISPLAY_HARDWARE_GPIO_H_INCLUDED
#define DISPLAY_HARDWARE_GPIO_H_INCLUDED

#define	_BV(bit)   (1 << (bit)) /**< left shift 1 bit */
#define	_HIGH      1            /**< signal high */
#define	_LOW       !_HIGH       /**< signal low */


#define	config_gpio_dir_o(DDR,Pin)  GPIO_PinModeSet(DDR, Pin, gpioModePushPull, 0) /**< set output direction for an IOPORT pin */
#define	config_gpio_dir_i(DDR,Pin)  GPIO_PinModeSet(DDR, Pin, gpioModeInput, 0)  /**< set input direction for an IOPORT pin */
#define	set_gpio_high(port,Pin)      GPIO_PortOutSet( port ,_BV(Pin)) /**< set HIGH for an IOPORT pin */
#define	set_gpio_low(port,Pin)       GPIO_PortOutClear(port ,_BV(Pin))  /**< set LOW for an IOPORT pin */
#define	set_gpio_invert(port,Pin)    GPIO_PinOutToggle(port ,Pin) /**< toggle the value of an IOPORT pin */
#define	input_get(port,Pin)          GPIO_PinInGet(port,Pin)  /**< get current value of an IOPORT pin */

/******************************************************************************
* GPIO Defines
*****************************************************************************/
#define Temper_PIN              4
#define Temper_PORT             gpioPortD       /**< PD4 Exp_12  */
#define SPICLK_PIN              2
#define SPICLK_PORT             gpioPortD       /**< PD2  Exp_8  */
#define EPD_BUSY_PIN            6
#define EPD_BUSY_PORT           gpioPortC       /**< PC6  Exp_15 */
#define PWM_PIN                 3
#define PWM_PORT                gpioPortD       /**< PD3  Exp_10 */
#define EPD_RST_PIN             5
#define EPD_RST_PORT            gpioPortD       /**< PD5  Exp_14 */
#define EPD_PANELON_PIN         6
#define EPD_PANELON_PORT        gpioPortD       /**< PD6  Exp_16 */
#define EPD_DISCHARGE_PIN       7
#define EPD_DISCHARGE_PORT      gpioPortD       /**< PD7  Exp_17 */
#define EPD_BORDER_PIN          12
#define EPD_BORDER_PORT         gpioPortB       /**< PB12 Exp_13 */
#define SPIMISO_PIN             1
#define SPIMISO_PORT            gpioPortD       /**< PD1  Exp_6  */
#define SPIMOSI_PIN             0
#define SPIMOSI_PORT            gpioPortD       /**< PD0  Exp_4  */
#define Flash_CS_PIN            11
#define Flash_CS_PORT           gpioPortB       /**< PB11 Exp_11 */
#define EPD_CS_PIN              5
#define EPD_CS_PORT             gpioPortC       /**< PC5  Exp_9  */


bool EPD_IsBusy(void);
void EPD_cs_high (void);
void EPD_cs_low (void);
void EPD_flash_cs_high(void);
void EPD_flash_cs_low (void);
void EPD_rst_high (void);
void EPD_rst_low (void);
void EPD_discharge_high (void);
void EPD_discharge_low (void);
void EPD_Vcc_turn_off (void);
void EPD_Vcc_turn_on (void);
void EPD_border_high(void);
void EPD_border_low (void);
void EPD_pwm_low (void);
void EPD_pwm_high(void);
void SPIMISO_low(void);
void SPIMOSI_low(void);
void SPICLK_low(void);
void EPD_initialize_gpio(void);

#endif	//DISPLAY_HARDWARE_GPIO_H_INCLUDED


