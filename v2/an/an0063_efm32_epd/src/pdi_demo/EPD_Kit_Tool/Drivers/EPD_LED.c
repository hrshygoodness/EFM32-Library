/**
 * \file
 *
 * \brief The functions of controlling LED on host kit
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
#include "EPD_LED.h"

void LED_ON(void) {
#if LED_DISABLE
	set_gpio_high(LED_Port, LED_Red_Pin);
#endif
}

void LED_OFF(void) {
#if LED_DISABLE
	set_gpio_low(LED_Port, LED_Red_Pin);
#endif
}

void LED_Trigger(void) {
#if LED_DISABLE
	set_gpio_invert(LED_Port, LED_Red_Pin);
#endif
}

void LED_init(void) {
	config_gpio_dir_o(LED_Port, LED_Red_Pin);
	LED_OFF();
}

