/***************************************************************************//**
 * @file    dvk_spi.h
 * @brief   header code for spi applications based on Cortex M3
 * @author  Energy Micro AS
 * @version 1.0.0
 *******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * This file is licensed under the Silicon Labs Software License Agreement. See 
 * "http://developer.silabs.com/legal/version/v11/Silicon_Labs_Software_License_Agreement.txt"  
 * for details. Before using this software for any purpose, you must agree to the 
 * terms of that agreement.
 *
 ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DVK_SPI_H__
#define __DVK_SPI_H__

/*! @addtogroup IEC60335_TOOLS
* @{
*/
/*! public function to enable LED communication, set LED0, set LED0 and 1
 * use this function to show that the WDOG reset occurred and changed the system.
 * a complete check will enable the LED0 until the WDOG reset occurred and 
 * than change the LEDs to LED0 and LED1.
 * @param mode  available options are : 0 = init interface and reset all LEDs\n
 *                                      1 = set LED0\n
 *                                  other = set LED0 and LED1
 */
void DVK_SPI(uint8_t mode);
/*!
 * @}
*/
#endif /* __DVK_SPI_H__ */

/************************************** EOF *********************************/
