/**
* \file
*
* \brief The definition of image data arrays
*
* Copyright (c) 2012-2013 Pervasive Displays Inc. All rights reserved.
*
*  Authors: Pervasive Displays Inc.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*  1. Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#define USE_EPD144 0 /**< 1.44 inch PDi EPD */
#define USE_EPD200 1 /**< 2 inch PDi EPD */
#define USE_EPD270 2 /**< 2.7 inch PDi EPD */

/* Using which EPD size for demonstration. 
 * Change to USE_EPD270 if you are connecting with 2.7 inch EPD. USE_EPD144 for 1.44 inch. */
#define USE_EPD_Type USE_EPD200 /**< default is connecting with 2 inch EPD */

#if(USE_EPD_Type==USE_EPD144) // array size=96*16, (128/8=16)
	extern unsigned char const image_array_144_1[]; /**< use 1.44" image array #1 */
	extern unsigned char const image_array_144_2[]; /**< use 1.44" image array #2 */
#elif(USE_EPD_Type==USE_EPD200) // array size=96*25, (200/8=25)
	extern unsigned char const image_array_200_1[]; /**< use 2" image array #1 */
	extern unsigned char const image_array_200_2[]; /**< use 2" image array #2 */
#elif(USE_EPD_Type==USE_EPD270) // array size=176*33, (264/8=33)
	extern unsigned char const image_array_270_1[]; /**< use 2.7" image array #1 */
	extern unsigned char const image_array_270_2[]; /**< use 2.7" image array #2 */
#endif
