/**************************************************************************//**
 * @file lesense_letouch_config.h
 * @brief LESENSE CAPACITIVE TOUCH code example
 * @author Silicon Labs
 * @version 2.05
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

#ifndef __LESENSE_LETOUCH_CONFIG_H
#define __LESENSE_LETOUCH_CONFIG_H

/** These are settings that need to be tuned for different PCB's, overlays and applications. */

/** Scan frequency for LESENSE, how often all the pads are scanned. */
#define LESENSE_SCAN_FREQUENCY          5

/** Sample delay, how long the rc-oscillations are sampled. */
#define SAMPLE_DELAY                   30 

/** Validate count is the number of consecutive scan-cycles a button needs to */
/** be in the changed state before an actual button press or release is acknowledged. */
#define VALIDATE_CNT                   10

/** Number of calibration events used to calculate threshold. */
#define NUMBER_OF_CALIBRATION_VALUES    10

/** Interval between calibration, in seconds. */
#define CALIBRATION_INTERVAL            5

#endif /* __LESENSE_LETOUCH_CONFIG_H */
