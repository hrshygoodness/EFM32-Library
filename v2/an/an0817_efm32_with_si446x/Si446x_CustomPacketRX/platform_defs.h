/*! @file platform_defs.h
 * @brief This file contains platform specific definitions.
 *
 * @b COPYRIGHT
 * @n Silicon Laboratories Confidential
 * @n Copyright 2012 Silicon Laboratories, Inc.
 * @n http://www.silabs.com
 */

#ifndef PLATFORM_DEFS_H_
#define PLATFORM_DEFS_H_


/*---------------------------------------------------------------------*/
/*            Platform specific global definitions                     */
/*---------------------------------------------------------------------*/

#define SILABS_RADIO_SI446X
#undef  SILABS_RADIO_SI4455

#ifdef EFM32LG990F256
#define SILABS_PLATFORM_COMPONENT_LED     2
#define SILABS_PLATFORM_COMPONENT_PB      2
//  #define SILABS_PLATFORM_COMPONENT_SWITCH  0
//  #define SILABS_PLATFORM_COMPONENT_BUZZER  0
#endif

#ifdef EFM32TG840F32
#define SILABS_PLATFORM_COMPONENT_LED     1
#define SILABS_PLATFORM_COMPONENT_PB      1  //We only use PB0 because the MCU PIN which connects to PB1 is occupied by the GPIO2 of the Radio.
//  #define SILABS_PLATFORM_COMPONENT_SWITCH  0
//  #define SILABS_PLATFORM_COMPONENT_BUZZER  0
#endif



#endif /* PLATFORM_DEFS_H_ */
