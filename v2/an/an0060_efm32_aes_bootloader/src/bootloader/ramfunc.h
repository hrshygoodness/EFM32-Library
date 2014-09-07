/**************************************************************************//**
 * @file ramfunc.h
 * @brief Macros for placing functions in RAM
 * @author Silicon Labs
 * @version 1.03
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/
#ifndef _RAMFUNC_H_
#define _RAMFUNC_H_


/* Place function in RAM */
#if defined (__ICCARM__)               			
#define RAMFUNC __ramfunc                                /* IAR  */
#elif defined(__CC_ARM)
#define RAMFUNC __attribute__ ((section (".ramcode")))   /* Keil MDK-ARM  */

#elif defined(__GNUC__)                  	

#if defined (__CROSSWORKS_ARM)         			
#define RAMFUNC __attribute__ ((section(".fast")))   /* Rowley Crossworks */
#else                           						
#define RAMFUNC __attribute__ ((section(".ram")))    /* Sourcery G++ */
#endif

#endif

#endif
