/***************************************************************************//**
 * @file    iec60335_class_b_post.h
 * @brief   header file for POST test
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
#ifndef __IEC60335_CLASS_B_POST_H__
#define __IEC60335_CLASS_B_POST_H__

/*! @addtogroup IEC60335_POST
 * @{
 * @image html POST.jpg
 */

#include "iec60335_class_b_typedef.h"

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

/*!
 * @brief   This function runs a colletion of POST tests
 *          and will suck in error condition
 *
 * @attention  This function must be called once befor memeory initialization.
 */
void IEC60335_ClassB_POST(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

/*!
 * @}
 */

#endif  /* __IEC60335_CLASS_B_POST_H__ */

/************************************** EOF *********************************/
