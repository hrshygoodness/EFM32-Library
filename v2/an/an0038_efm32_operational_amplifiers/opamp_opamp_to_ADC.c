/**************************************************************************//**
 * @file opamp_opamp_to_ADC.c
 * @brief OPA1 connected to the ADC.
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
#include "em_device.h"
#include "em_chip.h"
#include "em_opamp.h"
#include "em_cmu.h"
#include "em_adc.h"
#include "segmentlcd.h"

static void ADCConfig(void)
{
    ADC_Init_TypeDef       init       = ADC_INIT_DEFAULT;
    ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;

    init.timebase = ADC_TimebaseCalc(0);
    init.prescale = ADC_PrescaleCalc(7000000, 0);
    ADC_Init(ADC0, &init);

    singleInit.reference = adcRefVDD;
    singleInit.input     = adcSingleInpCh1;
 
    ADC_InitSingle(ADC0, &singleInit);
}

int main(void)
{ 
    uint32_t voltage;
    
    CHIP_Init();
    
    /*Initilize LCD*/
    SegmentLCD_Init(false);
  
    /*Set comma symbol on*/
    SegmentLCD_Symbol(LCD_SYMBOL_DP10, 1);
    
    /*Write text to LCD display*/
    SegmentLCD_Write("Voltage");
    
    /*Enable clocks*/
    CMU_ClockEnable(cmuClock_DAC0, true);
    CMU_ClockEnable(cmuClock_ADC0, true);

    /*Configure ADC*/
    ADCConfig();
              
    /*Define the configuration for OPA1*/
    OPAMP_Init_TypeDef configuration =  OPA_INIT_NON_INVERTING;
    
    /*Send the output to ADC*/
    configuration.outMode = opaOutModeAll;
    configuration.outPen =  DAC_OPA1MUX_OUTPEN_OUT4;

    /*Enable OPA1*/
    OPAMP_Enable(DAC0, OPA1, &configuration);

    /*While loop that get data from ADC and displays it on the LCD screen*/
    while (1)
    {
        /*Start ADC*/
        ADC_Start(ADC0, adcStartSingle);
        
        /*Wait until the ADC has warmed up*/
        while (ADC0->STATUS & ADC_STATUS_SINGLEACT);

        /*Get data from ADC and convert to voltage*/
        voltage = ADC_DataSingleGet(ADC0)*330/4096;
        
        /*Write to LCD*/
        SegmentLCD_Number(voltage);
        
    } 
}
