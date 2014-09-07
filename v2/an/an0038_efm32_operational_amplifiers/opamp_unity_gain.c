/**************************************************************************//**
 * @file opamp_unity_gain.c
 * @brief OPA2 configured as a voltage follower.
 * @author Silicon Labs
 * @version 1.07
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
#include <stdint.h>
#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_opamp.h"
#include "em_cmu.h"

int main(void)
{ 
    CHIP_Init();

    /*Turn on the DAC clock*/
    CMU_ClockEnable(cmuClock_DAC0, true);
  
    /*Define the configuration for OPA2*/
    OPAMP_Init_TypeDef configuration =  OPA_INIT_UNITY_GAIN_OPA2;

    /*Enable OPA2*/
    OPAMP_Enable(DAC0, OPA2, &configuration);
  
    /*Never end*/
    while(1);   
}
