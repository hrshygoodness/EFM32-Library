/**************************************************************************//**
 * @file matrix_test.c
 * @brief Example demonstrating matrix operations from the CMSIS DSP Library
 * @author Silicon Labs
 * @version 1.04
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
   
#include <stdio.h>
#include "em_device.h"
#include "arm_math.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_timer.h"

/* Prototype for swo setup function, use SWO printf terminal in */
/* eAcommander to read the printf text in this example. */
void setupSWOForPrint(void);

/* Need to implement the two Retarget IO functions with the read/write functions we want to use. */
int RETARGET_WriteChar(char c){
  return ITM_SendChar (c);
}
int RETARGET_ReadChar(void){
  return 0;
}

#define TIMER_TOP 0xffff

/* Number of rows in each matrix */
#define MATRIX_ROWS 10
/* Number of columns in each matrix */
#define MATRIX_COLS 10

/* Matrix A in floating point format */
float32_t A_f32[MATRIX_ROWS*MATRIX_COLS] =
{
  0.565, 0.378, 0.129, 0.229, 0.991, 0.555, 0.163, 0.102, 0.348, 0.816,
  0.602, 0.353, 0.569, 0.504, 0.397, 0.247, 0.037, 0.355, 0.043, 0.523,
  0.764, 0.139, 0.996, 0.890, 0.428, 0.755, 0.936, 0.228, 0.230, 0.192,
  0.373, 0.893, 0.598, 0.455, 0.987, 0.580, 0.416, 0.491, 0.744, 0.897,
  0.682, 0.141, 0.261, 0.179, 0.295, 0.665, 0.333, 0.439, 0.245, 0.862,
  0.092, 0.786, 0.363, 0.041, 0.385, 0.290, 0.028, 0.567, 0.963, 0.467,
  0.123, 0.787, 0.314, 0.238, 0.073, 0.506, 0.452, 0.030, 0.847, 0.857,
  0.232, 0.311, 0.559, 0.216, 0.264, 0.923, 0.554, 0.468, 0.534, 0.473,
  0.166, 0.760, 0.827, 0.430, 0.206, 0.089, 0.522, 0.647, 0.517, 0.051,
  0.399, 0.347, 0.507, 0.116, 0.873, 0.670, 0.443, 0.609, 0.301, 0.982,
};

/* Matrix C in floating point format */
float32_t B_f32[MATRIX_ROWS*MATRIX_COLS] =
{
  0.538, 0.777, 0.129, 0.195, 0.554, 0.628, 0.857, 0.795, 0.886, 0.470,
  0.303, 0.109, 0.767, 0.697, 0.314, 0.184, 0.125, 0.216, 0.257, 0.407,
  0.562, 0.972, 0.697, 0.911, 0.277, 0.384, 0.121, 0.614, 0.663, 0.007,
  0.889, 0.555, 0.037, 0.317, 0.912, 0.112, 0.942, 0.771, 0.746, 0.620,
  0.454, 0.571, 0.719, 0.040, 0.520, 0.358, 0.033, 0.232, 0.552, 0.241,
  0.804, 0.335, 0.597, 0.727, 0.607, 0.477, 0.268, 0.420, 0.978, 0.384,
  0.246, 0.398, 0.138, 0.938, 0.413, 0.769, 0.966, 0.964, 0.526, 0.196,
  0.776, 0.742, 0.411, 0.485, 0.434, 0.528, 0.031, 0.574, 0.429, 0.090,
  0.152, 0.708, 0.085, 0.869, 0.700, 0.173, 0.684, 0.799, 0.310, 0.491,
  0.251, 0.524, 0.432, 0.630, 0.136, 0.344, 0.370, 0.425, 0.341, 0.731,
};

/* Matrix C in floating point format */
float32_t C_f32[MATRIX_ROWS*MATRIX_COLS];

/* Matrices in Q31 fixed point format */
q31_t A_q31[MATRIX_ROWS*MATRIX_COLS];
q31_t B_q31[MATRIX_ROWS*MATRIX_COLS];
q31_t C_q31[MATRIX_ROWS*MATRIX_COLS];

/* Matrices in Q15 fixed point format */
q15_t A_q15[MATRIX_ROWS*MATRIX_COLS];
q15_t B_q15[MATRIX_ROWS*MATRIX_COLS];
q15_t C_q15[MATRIX_ROWS*MATRIX_COLS];

/* State matrix needed to perform Q15 matrix multiplication */
q15_t state_q15[MATRIX_ROWS*MATRIX_COLS];

/**************************************************************************//**
 * @brief
 *   Fill the fixed point matrices A and B with data from the floating point
 *   matrices.
 *****************************************************************************/
void matrixMultTestSetup(void)
{
  /* Fill the q31 data arrays with converted
   * numbers from the f32 data arrays
   */
  arm_float_to_q31(A_f32, A_q31, MATRIX_ROWS*MATRIX_COLS);
  arm_float_to_q31(B_f32, B_q31, MATRIX_ROWS*MATRIX_COLS);
  
  /* Fill the q15 data arrays with converted
   * numbers from the f32 data arrays
   */
  arm_float_to_q15(A_f32, A_q15, MATRIX_ROWS*MATRIX_COLS);
  arm_float_to_q15(B_f32, B_q15, MATRIX_ROWS*MATRIX_COLS);
}

/**************************************************************************//**
 * @brief  Initialize the floating point matrices and perform multiplication
 *****************************************************************************/
arm_status matrixMultF32Test(void)
{
  arm_status status;

  /* Matrix instances */
  arm_matrix_instance_f32 A;
  arm_matrix_instance_f32 B;
  arm_matrix_instance_f32 C;

  /* Initialize matrix instances with data arrays */
  arm_mat_init_f32(&A, MATRIX_ROWS, MATRIX_COLS, A_f32);
  arm_mat_init_f32(&B, MATRIX_ROWS, MATRIX_COLS, B_f32);
  arm_mat_init_f32(&C, MATRIX_ROWS, MATRIX_COLS, C_f32);

  /* Perform the actual multiplication, result in C matrix */
  status = arm_mat_mult_f32(&A, &B, &C);

  return status;
}

/**************************************************************************//**
 * @brief  Initialize the Q31 matrices and perform multiplication
 *****************************************************************************/
arm_status matrixMultQ31Test(void)
{
  arm_status status;

  /* Matrix instances */
  arm_matrix_instance_q31 A;
  arm_matrix_instance_q31 B;
  arm_matrix_instance_q31 C;

  /* Initialize matrix instances with data arrays */
  arm_mat_init_q31(&A, MATRIX_ROWS, MATRIX_COLS, A_q31);
  arm_mat_init_q31(&B, MATRIX_ROWS, MATRIX_COLS, B_q31);
  arm_mat_init_q31(&C, MATRIX_ROWS, MATRIX_COLS, C_q31);

  /* Perform the actual multiplication, result in C matrix */
  status = arm_mat_mult_q31(&A, &B, &C);
  
  return status;
}

/**************************************************************************//**
 * @brief  Initialize the Q15 matrices and perform multiplication
 *****************************************************************************/
arm_status matrixMultQ15Test(void)
{
  arm_status status;

  /* Matrix instances */
  arm_matrix_instance_q15 A;
  arm_matrix_instance_q15 B;
  arm_matrix_instance_q15 C;

  /* Initialize matrix instances with data arrays */
  arm_mat_init_q15(&A, MATRIX_ROWS, MATRIX_COLS, A_q15);
  arm_mat_init_q15(&B, MATRIX_ROWS, MATRIX_COLS, B_q15);
  arm_mat_init_q15(&C, MATRIX_ROWS, MATRIX_COLS, C_q15);

  /* Perform the actual multiplication, result in C matrix */
  status = arm_mat_mult_q15(&A, &B, &C, state_q15);
  
  return status;
}

/**************************************************************************//**
 * @brief  Main function
 * The timers are set up to count cycles and cascade (overflow into the next
 * timer. We convert the floating point data arrays to fixed point and perform
 * the matrix products.
 *****************************************************************************/
int main(void)
{
  arm_status status;

  uint64_t    time;
  
  /* Chip revision alignment and errata fixes */
  CHIP_Init();
  
  setupSWOForPrint();
  
  /* Set 14MHz default HFRCO frequency, this will set flash wait cycles to 0. */
  CMU_HFRCOBandSet(cmuHFRCOBand_14MHz);
  
  /* Enable DWT */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  /* Make sure CYCCNT is running */
  DWT->CTRL |= 1;
  
  /* Fill fixed point data arrays */
  matrixMultTestSetup();

  /* Floating point matrix test */
  time = DWT->CYCCNT;
  status = matrixMultF32Test();
  time = DWT->CYCCNT - time;
  
  if (status == ARM_MATH_SUCCESS)
  {
    printf("F32 matrix mult: %lld cycles\n\n", time);
  }
  else
  {
    printf("Error performing F32 matrix multiplication test\n\n");
  }

  /* Fixed point Q31 test */
  time = DWT->CYCCNT;
  status = matrixMultQ31Test();
  time = DWT->CYCCNT - time;
  
  if (status == ARM_MATH_SUCCESS)
  {
    printf("Q31 matrix mult: %lld cycles\n\n", time);
  }
  else
  {
    printf("Error performing Q31 matrix multiplication test\n\n");
  }

  /* Fixed point Q15 test */
  time = DWT->CYCCNT;
  status = matrixMultQ15Test();
  time = DWT->CYCCNT - time;
  
  if (status == ARM_MATH_SUCCESS)
  {
    printf("Q15 matrix mult: %lld cycles\n\n", time);
  }
  else
  {
    printf("Error performing Q15 matrix multiplication test\n\n");
  }

  while (1) ;
}


/**************************************************************************//**
 * @brief Configure SWO - serial wire output, function taken from 
 * application note an0043_efm32_debug_trace_capabilities.
 *****************************************************************************/
void setupSWOForPrint(void)
{
  /* Enable GPIO clock. */
  CMU->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO;

  /* Enable Serial wire output pin */
  GPIO->ROUTE |= GPIO_ROUTE_SWOPEN;

#if defined(_EFM32_GIANT_FAMILY) || defined(_EFM32_LEOPARD_FAMILY) || defined(_EFM32_WONDER_FAMILY)
  /* Set location 0 */
  GPIO->ROUTE = (GPIO->ROUTE & ~(_GPIO_ROUTE_SWLOCATION_MASK)) | GPIO_ROUTE_SWLOCATION_LOC0;

  /* Enable output on pin - GPIO Port F, Pin 2 */
  GPIO->P[5].MODEL &= ~(_GPIO_P_MODEL_MODE2_MASK);
  GPIO->P[5].MODEL |= GPIO_P_MODEL_MODE2_PUSHPULL;
#else
  /* Set location 1 */
  GPIO->ROUTE = (GPIO->ROUTE & ~(_GPIO_ROUTE_SWLOCATION_MASK)) |GPIO_ROUTE_SWLOCATION_LOC1;
  /* Enable output on pin */
  GPIO->P[2].MODEH &= ~(_GPIO_P_MODEH_MODE15_MASK);
  GPIO->P[2].MODEH |= GPIO_P_MODEH_MODE15_PUSHPULL;
#endif

  /* Enable debug clock AUXHFRCO */
  CMU->OSCENCMD = CMU_OSCENCMD_AUXHFRCOEN;

  /* Wait until clock is ready */
  while (!(CMU->STATUS & CMU_STATUS_AUXHFRCORDY));

  /* Enable trace in core debug */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  ITM->LAR  = 0xC5ACCE55;
  ITM->TER  = 0x0;
  ITM->TCR  = 0x0;
  TPI->SPPR = 2;
  TPI->ACPR = 0xf;
  ITM->TPR  = 0x0;
  DWT->CTRL = 0x400003FE;
  ITM->TCR  = 0x0001000D;
  TPI->FFCR = 0x00000100;
  ITM->TER  = 0x1;
}
