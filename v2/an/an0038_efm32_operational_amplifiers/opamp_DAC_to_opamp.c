/**************************************************************************//**
 * @file opamp_DAC_to_opamp.c
 * @brief DACn_CH0 and DACn_CH1 connected to OPA2. OPA2 is used as a voltage reducer.
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
#include "em_dac.h"

void DAC_setup(void)
{
  /* Define DAC settings */
  DAC_Init_TypeDef init =  
  { 
    dacRefresh8,              /* Refresh every 8 prescaled cycles. */    \
    dacRef1V25,               /* 1.25V internal reference. */            \
    dacOutputPinADC,          /* Output to pin and ADC. */               \
    dacConvModeContinuous,    /* Continuous mode. */                     \
    0,                        /* No prescaling. */                       \
    false,                    /* Do not enable low pass filter. */       \
    false,                    /* Do not reset prescaler on ch0 start. */ \
    false,                    /* DAC output enable always on. */         \
    false,                    /* Disable sine mode. */                   \
    false                     /* Single ended mode. */                   \
  };
  
  DAC_InitChannel_TypeDef initChannel = DAC_INITCHANNEL_DEFAULT;

  /* Calculate the DAC clock prescaler value that will result in a DAC clock
   * close to 500kHz. Second parameter is zero, if the HFPERCLK value is 0, the
   * function will check what the current value actually is. */
  init.prescale = DAC_PrescaleCalc(500000, 0);

  /* Set reference voltage to vdd.*/
  init.reference = dacRefVDD;

  /* Initialize the DAC and DAC channel. */
  DAC_Init(DAC0, &init);
  
  /*Initialize DAC channel 0.*/
  DAC_InitChannel(DAC0, &initChannel, 0);
  
}

int main(void)
{ 
    CHIP_Init();
  
    /*Turn on the DAC clock.*/
    CMU_ClockEnable(cmuClock_DAC0, true);
    
    /*configure and enable the DAC.*/
    DAC_setup();
    DAC_Enable(DAC0, 0, true);
    
    /*Write data to registers. V1 = 1.0*/
    DAC0->CH0DATA = (uint32_t)((1.0 * 4096) / 3.3);
    
    /*configure OPA0, OPA1 and OPA2.*/
    OPAMP_Init_TypeDef configuration0 =  OPA_INIT_DIFF_RECEIVER_OPA0 ;
    OPAMP_Init_TypeDef configuration2 =  OPA_INIT_DIFF_RECEIVER_OPA2 ; 
    
    /*Redefine the resistances. Want to divide the difference by 3.*/
    configuration2.resSel = opaResSelDefault;
    configuration0.resSel = opaResSelR2eq3R1;
    
    /*OPA2 positive input = VSS*/
    configuration2.resInMux = opaResInMuxVss;
    
    /*Enable OPA0 and OPA2. All the configurations are set.*/
    OPAMP_Enable(DAC0, OPA0, &configuration0);
    OPAMP_Enable(DAC0, OPA2, &configuration2);

    /*Disable OPA0. This is done because we want to use OPA0 as a part of the DAC. 
    The configurations set above are still there.*/
    DAC0->OPACTRL &= ~DAC_OPACTRL_OPA0EN;
  
    /*Never end.*/
    while(1);   
}
















