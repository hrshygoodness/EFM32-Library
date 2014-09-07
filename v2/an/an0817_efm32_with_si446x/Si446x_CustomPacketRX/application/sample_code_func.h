/*! @file sample_code_func.h
 * @brief This file is the interface file for basic HMI functions.
 *
 * @b COPYRIGHT
 * @n Silicon Laboratories Confidential
 * @n Copyright 2012 Silicon Laboratories, Inc.
 * @n http://www.silabs.com
 */

#ifndef SAMPLE_CODE_FUNC_H_
#define SAMPLE_CODE_FUNC_H_


/*------------------------------------------------------------------------*/
/*                          Global definitions                            */
/*------------------------------------------------------------------------*/

/*------------------------------------------------------------------------*/
/*          Global structure & enumeration definitions                    */
/*------------------------------------------------------------------------*/


/*------------------------------------------------------------------------*/
/*                           Function prototypes                          */
/*------------------------------------------------------------------------*/

#if ((defined SILABS_PLATFORM_COMPONENT_PB) && (defined SILABS_PLATFORM_COMPONENT_LED) && (defined SILABS_PLATFORM_COMPONENT_BUZZER))
  #ifdef HMI_DRIVER_EXTENDED_SUPPORT
    void vSampleCode_ShowPbOnBuzzer(void);
  #endif
#endif
#endif /* SAMPLE_CODE_FUNC_H_ */
