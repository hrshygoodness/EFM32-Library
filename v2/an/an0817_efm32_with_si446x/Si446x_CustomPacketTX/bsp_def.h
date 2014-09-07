/*! @file bsp.h
 * @brief This file contains application specific definitions and includes.
 *
 * @b COPYRIGHT
 * @n Silicon Laboratories Confidential
 * @n Copyright 2012 Silicon Laboratories, Inc.
 * @n http://www.silabs.com
 */

#ifndef BSP_H
#define BSP_H

/*------------------------------------------------------------------------*/
/*            Application specific global definitions                     */
/*------------------------------------------------------------------------*/
/*! Platform definition */
/* Note: Plaform is defined in IDE project file */
/*! Enable logging on UART */

/*! Extended driver support 
 * Known issues: Some of the example projects 
 * might not build with some extended drivers 
 * due to data memory overflow */
#define RADIO_DRIVER_EXTENDED_SUPPORT
#define RADIO_DRIVER_FULL_SUPPORT
#define SPI_DRIVER_EXTENDED_SUPPORT
#define HMI_DRIVER_EXTENDED_SUPPORT

/*------------------------------------------------------------------------*/
/*            Application specific includes                               */
/*------------------------------------------------------------------------*/

#ifdef EFM32LG990F256
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "bsp.h"
#include "bsp_trace.h"
#include "platform_defs.h"
#include "si446x_defs.h"
#include "spi.h"
#include "radio_config.h"
#include "radio_hal.h"
#include "radio.h"
#include "si446x_api_lib.h"
#include "board.h"
#include "hmi.h"
#include "sample_code_func.h"
#include "radio_comm.h"
#include "si446x_nirq.h"
#endif

#ifdef EFM32TG840F32
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "bsp.h"
#include "bsp_trace.h"
#include "platform_defs.h"
#include "si446x_defs.h"
#include "spi.h"
#include "radio_config.h"
#include "radio_hal.h"
#include "radio.h"
#include "si446x_api_lib.h"
#include "board.h"
#include "hmi.h"
#include "sample_code_func.h"
#include "radio_comm.h"
#include "si446x_nirq.h"
#endif

#ifdef SILABS_RADIO_SI446X
#include "si446x_api_lib.h"
#include "si446x_defs.h"
#include "si446x_nirq.h"
#endif

#ifdef SILABS_RADIO_SI4455
#include "si4455_api_lib.h"
#include "si4455_defs.h"
#include "si4455_nirq.h"
#endif

#ifndef TRUE
#define TRUE    1
#endif

#ifndef FALSE
#define FALSE   0
#endif

#ifndef NULL
#define NULL    0
#endif

#endif //BSP_H
