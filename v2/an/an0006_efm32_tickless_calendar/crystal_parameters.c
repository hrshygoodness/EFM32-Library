/***************************************************************************//**
 * @file crystal_parameters.c
 * @brief Crystal temperature deviation
 * @author Silicon Labs
 * @version 2.06
 *******************************************************************************
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

/* Include standard libraries */
#include <stdint.h>

#include "crystal_parameters.h"

/* Crystal specific temperature dependent frequency deviation (dF) in Hz.
   The table is in the format:
     dF(T) = crystalCompensationTable[T-T0] / 1000
   i.e. the value of element n is the frequency offset in mHz at n degrees C offset from the crystal's nominal temperature.
   This table is based on the data sheet parameters for the 32.768 kHz crystal on EFM32GG-STK3700, whose
   crystal temperature deviation is given as -0.035 ppm/(degC)^2 */
const int16_t crystalCompensationTable[]    = {     0,    -1,    -5,   -10,   -18,   -29,   -41,   -56,
                                                  -73,   -93,  -115,  -139,  -165,  -194,  -225,  -258,
                                                 -294,  -331,  -372,  -414,  -459,  -506,  -555,  -607,
                                                 -661,  -717,  -775,  -836,  -899,  -965, -1032, -1102,
                                                -1174, -1249, -1326, -1405, -1486, -1570, -1656, -1744,
                                                -1835, -1928, -2023, -2121, -2220, -2322, -2427, -2533,
                                                -2642, -2754, -2867, -2983, -3101, -3222, -3344, -3469,
                                                -3597, -3726, -3858, -3992, -4129, -4268, -4409, -4552,
                                                -4698, -4846, -4996, -5148, -5303, -5460, -5620};
