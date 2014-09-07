/******************************************************************************
 * @file hw_config.h
 * @brief Smart Card Driver pin/clk/uart config header file
 * @author Silicon Labs
 * @version 1.01
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 *
 ******************************************************************************/

#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

/* Define Timing Parameters */
/* This implementation only supports 9600 bps and standard clk freq multiplier (372) */
#define SC_BAUD_RATE (9600)
#define SC_CLK_MULTIPLIER (372)
#define SC_CLK_FREQ (SC_BAUD_RATE * SC_CLK_MULTIPLIER)

/* Byte time equals 1 start bit, 8 data, 1 parity and 2 guard bits. */
#define SC_BYTE_TIME (12*1000/SC_BAUD_RATE + 1)

/* ms to wait for any answer from card before timing out. */
#define SC_CMD_TIMEOUT (500)

/* Different timeout periods in ms for Reset procedure */
#define SC_PWR_ON_DELAY (10)
#define SC_CLK_DELAY (10)
#define SC_ATR_DELAY (100)

/* Timer to use for clk output */        
#define SC_TIMER TIMER0
#define SC_TIMER_CMU_CLOCK cmuClock_TIMER0

/* GPIO pins to use for smartcard comm. Clock pin must match with timer CC output. */
/* Data pin must match usart location and usart peripheral. */
#define SC_USART_MODULE USART1
#define SC_USART_LOCATION USART_ROUTE_LOCATION_LOC1
#define SC_GPIO_DATA_PORT gpioPortD
#define SC_GPIO_DATA_PIN 0 

#define SC_GPIO_CLOCK_PORT gpioPortD
#define SC_GPIO_CLOCK_PIN 2

#define SC_GPIO_VDD_PORT gpioPortD
#define SC_GPIO_VDD_PIN 3
#define SC_GPIO_GND_PORT gpioPortD
#define SC_GPIO_GND_PIN 4
#define SC_GPIO_RESET_PORT gpioPortD
#define SC_GPIO_RESET_PIN 5
#define SC_GPIO_INSERT_DETECTION_PORT gpioPortD
#define SC_GPIO_INSERT_DETECTION_PIN 1

#define SC_GPIO_ACTIVITY_LED_PORT gpioPortE
#define SC_GPIO_ACTIVITY_LED_PIN 2

#endif /* __HW_CONFIG_H */
